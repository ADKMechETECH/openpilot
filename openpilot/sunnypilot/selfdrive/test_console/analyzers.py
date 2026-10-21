"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console analyzers (vehicle-agnostic measurement engine): turn the sample stream of one capture window into
progress ("gear 3: 4/10 s"), a "complete" flag and a result (PASS / FAIL / CAPTURED + a one-line detail). Analysis only —
nothing here can reach a control path. Vehicle facts come from the profile through the Context (powertrain, signal
names); the pass rules that turn a measurement into a verdict are the profile's (VehicleProfile.results), except the
generic ones at the bottom.

  SegmentAnalyzer     qualifying time in contiguous segments, optionally binned (per gear, per command band, ...)
  EpisodeAnalyzer     counts detected maneuvers (shifts, handbacks, stops, ...) and measures each one
  GearRawMapAnalyzer  the car's raw gear signal vs the rpm/speed inference, at a stop, in reverse
  BitHuntAnalyzer     which bit of a raw payload is an input the driver toggles in a signature rhythm
  NonAdaptiveAnalyzer set-speed hold error and the command behind a slower lead
"""
import math
from collections import defaultdict

from openpilot.sunnypilot.selfdrive.test_console.conditions import BINS, Context
from openpilot.sunnypilot.selfdrive.test_console.profile import Powertrain
from openpilot.sunnypilot.selfdrive.test_console.registry import TestSpec
from openpilot.sunnypilot.selfdrive.test_console.signals import FAULT_ALERTS, HANDBACK_ALERT, OVERSPEED_ALERT, Sample

PASS, FAIL, CAPTURED = "PASS", "FAIL", "CAPTURED"

MAX_DT = 0.1                 # s, a gap longer than this between samples ends a segment
STAT_FIELDS = ("v_ego", "a_ego", "accel_cmd", "accel_out", "accel_sent", "a_target", "a_coast", "excess", "pitch", "rpm",
               "ratio", "rpm_rate", "v_cruise", "gas_raw")
JERK_RC = 0.1                # s, smoothing for command jerk
FLARE_RPM_RATE = 1500.       # rpm/s: the fork's clutch-gate flare threshold (pinned by a test), counted per segment
ROLL_SPEED = 0.3             # m/s
HILL_STOP_PITCH = 0.03       # rad (~3 %)


def _pct(values: list[float], q: float) -> float:
  v = sorted(values)
  if not v:
    return math.nan
  return v[min(len(v) - 1, max(0, math.ceil(q * len(v)) - 1))]


def _fmt(x, nd=2) -> str:
  return "n/a" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{nd}f}"


def _coupled(ratio: float, gear: int, pt: Powertrain) -> bool:
  return 1 <= gear <= pt.num_gears and ratio > 0. and abs(ratio / pt.overall_ratio(gear) - 1.) <= pt.match_tol


class JerkFilter:
  def __init__(self):
    self.prev: tuple[float, float] | None = None
    self.j = 0.

  def update(self, t: float, x: float) -> float:
    if self.prev is not None and t > self.prev[0]:
      dt = t - self.prev[0]
      raw = (x - self.prev[1]) / dt
      k = dt / (JERK_RC + dt)
      self.j += k * (raw - self.j)
    self.prev = (t, x)
    return self.j


class Analyzer:
  """Base: also keeps window-wide health stats (every sample in the capture window, qualifying or not)."""

  def __init__(self, spec: TestSpec, ctx: Context):
    self.spec, self.ctx, self.goal = spec, ctx, spec.goal
    self.frames = 0
    self.window_s = 0.
    self.acc_fault_edges = 0
    self.fcw_edges = 0
    self.disengages = 0
    self.tx_blocked_first: int | None = None
    self.tx_blocked_last = 0
    self.fault_alerts: set[str] = set()
    self._prev: Sample | None = None
    self.data: dict = {}   # extra, JSON-safe fields a result rule saves with the result (e.g. a model fit)

  # -- window ---------------------------------------------------------------------------------------------------
  def _window(self, s: Sample) -> float:
    p = self._prev
    dt = min(max(s.t - p.t, 0.), MAX_DT) if p is not None else 0.
    self.frames += 1
    self.window_s += dt
    if p is not None:
      self.acc_fault_edges += s.acc_faulted and not p.acc_faulted
      self.fcw_edges += s.fcw and not p.fcw
      self.disengages += p.enabled and not s.enabled
    if self.tx_blocked_first is None:
      self.tx_blocked_first = s.tx_blocked
    self.tx_blocked_last = s.tx_blocked
    if s.alert in FAULT_ALERTS:
      self.fault_alerts.add(s.alert)
    self._prev = s
    return dt

  @property
  def tx_blocked(self) -> int:
    return self.tx_blocked_last - (self.tx_blocked_first or 0)

  def feed(self, s: Sample, ok: bool) -> list[str]:
    """One sample of the open capture window. ok = all conditions held. Returns milestone notes (for markers)."""
    dt = self._window(s)
    return self._feed(s, ok, dt)

  def _feed(self, s: Sample, ok: bool, dt: float) -> list[str]:
    raise NotImplementedError

  def progress(self) -> tuple[str, float]:
    raise NotImplementedError

  @property
  def complete(self) -> bool:
    return self.progress()[1] >= 1. - 1e-9

  def result(self) -> tuple[str | None, str]:
    fn = results_for(self.ctx).get(self.spec.result)
    if fn is None:
      return None, ""
    status, detail = fn(self)
    # Task 12 D3: no rule may PASS before its test is complete (an early FAIL is fine)
    return (None if status == PASS and not self.complete else status), detail


# --- segments ------------------------------------------------------------------------------------------------------
class Segment:
  __slots__ = ("key", "t0", "t_last", "dur", "n", "sums", "sumsq", "cnts", "mins", "maxs", "flares", "rpm_changes",
               "jerk_max", "jerk_sum", "assist_on", "assist_n", "gears", "_rpm_prev", "_jerk")

  def __init__(self, key, t: float):
    self.key, self.t0, self.t_last, self.dur, self.n = key, t, t, 0., 0
    self.sums: dict[str, float] = defaultdict(float)
    self.sumsq: dict[str, float] = defaultdict(float)
    self.cnts: dict[str, int] = defaultdict(int)
    self.mins: dict[str, float] = {}
    self.maxs: dict[str, float] = {}
    self.flares = self.rpm_changes = self.assist_on = self.assist_n = 0
    self.gears: dict[int, int] = defaultdict(int)   # stats samples per inferred gear (a per-gear model correction)
    self.jerk_max = self.jerk_sum = 0.
    self._rpm_prev: float | None = None
    self._jerk = JerkFilter()

  def add(self, s: Sample, dt: float, stats: bool) -> None:
    self.t_last = s.t
    self.dur += dt
    j = self._jerk.update(s.t, s.accel_cmd)
    if not stats:
      return
    self.n += 1
    for f in STAT_FIELDS:
      v = getattr(s, f)
      if isinstance(v, (int, float)) and math.isfinite(v):
        self.sums[f] += v
        self.sumsq[f] += v * v
        self.cnts[f] += 1
        self.mins[f] = min(self.mins.get(f, v), v)
        self.maxs[f] = max(self.maxs.get(f, v), v)
    self.gears[int(s.gear)] += 1
    self.flares += s.rpm_rate > FLARE_RPM_RATE
    self.rpm_changes += self._rpm_prev is not None and s.rpm != self._rpm_prev
    self._rpm_prev = s.rpm
    self.jerk_max = max(self.jerk_max, abs(j))
    self.jerk_sum += abs(j)
    if s.brake_assist >= 0:
      self.assist_n += 1
      self.assist_on += s.brake_assist > 0

  def mean(self, f: str) -> float:
    return self.sums[f] / self.cnts[f] if self.cnts.get(f) else math.nan


class SegmentAnalyzer(Analyzer):
  def __init__(self, spec: TestSpec, ctx: Context):
    super().__init__(spec, ctx)
    g = self.goal
    self.bin = BINS[g["bin"]] if "bin" in g else None
    self.keys = list(g.get("keys", ["all"]))
    self.required = list(g.get("required", self.keys))
    self.seconds = g.get("seconds")
    self.count = g.get("count")
    self.min_segment = g.get("min_segment", 0.)
    self.settle = g.get("settle", 0.)
    self.cur: Segment | None = None
    self.done: list[Segment] = []
    self.secs: dict = defaultdict(float)
    self.nseg: dict = defaultdict(int)

  def _close(self) -> list[str]:
    seg, self.cur = self.cur, None
    if seg is None or seg.dur < self.min_segment or seg.dur <= 0.:
      return []
    self.secs[seg.key] += seg.dur
    self.nseg[seg.key] += 1
    if len(self.done) < 1000:
      self.done.append(seg)
    return [f"segment {seg.key} {seg.dur:.1f}s"]

  def _feed(self, s: Sample, ok: bool, dt: float) -> list[str]:
    key = "all"
    if ok and self.bin is not None:
      key = self.bin(s, self.ctx)
      ok = key is not None and key in self.keys
    notes = []
    if self.cur is not None and (not ok or key != self.cur.key or s.t - self.cur.t_last > MAX_DT):
      notes += self._close()
    if ok:
      new = self.cur is None
      if new:
        self.cur = Segment(key, s.t)
      self.cur.add(s, 0. if new else dt, s.t - self.cur.t0 >= self.settle)
    return notes

  def finish(self) -> list[str]:
    return self._close()

  def _live(self, key) -> tuple[float, int]:
    secs, n = self.secs[key], self.nseg[key]
    if self.cur is not None and self.cur.key == key and self.cur.dur >= self.min_segment:
      secs, n = secs + self.cur.dur, n + 1
    return secs, n

  def progress(self) -> tuple[str, float]:
    if self.count:
      n = sum(self._live(k)[1] for k in self.keys)
      return f"{n}/{self.count} segments", min(1., n / self.count)
    if self.keys == ["all"]:
      secs = self._live("all")[0]
      return f"{secs:.0f}/{self.seconds:.0f} s", min(1., secs / self.seconds)
    parts, fracs = [], []
    for k in self.keys:
      secs = self._live(k)[0]
      opt = "" if k in self.required else "?"
      parts.append(f"{k}{opt}: {secs:.0f}/{self.seconds:.0f}s")
      if k in self.required:
        fracs.append(min(1., secs / self.seconds))
    return " • ".join(parts), (sum(fracs) / len(fracs)) if fracs else 0.

  @property
  def segments(self) -> list[Segment]:
    """Committed segments plus the open one once it is long enough to count (results update live)."""
    if self.cur is not None and self.cur.dur >= self.min_segment and self.cur.dur > 0.:
      return self.done + [self.cur]
    return self.done

  def by_key(self, key) -> list[Segment]:
    return [g for g in self.segments if g.key == key]

  def pooled(self, f: str, key=None) -> float:
    segs = self.segments if key is None else self.by_key(key)
    n = sum(g.cnts.get(f, 0) for g in segs)
    return sum(g.sums.get(f, 0.) for g in segs) / n if n else math.nan

  def pooled_std(self, f: str, key=None) -> float:
    segs = self.segments if key is None else self.by_key(key)
    n = sum(g.cnts.get(f, 0) for g in segs)
    if n < 2:
      return math.nan
    m = sum(g.sums.get(f, 0.) for g in segs) / n
    return math.sqrt(max(sum(g.sumsq.get(f, 0.) for g in segs) / n - m * m, 0.))

  def gear_counts(self, key=None) -> dict[int, int]:
    out: dict[int, int] = defaultdict(int)
    for g in (self.segments if key is None else self.by_key(key)):
      for gear, n in g.gears.items():
        out[gear] += n
    return dict(out)

  def extreme(self, f: str, fn=min, key=None) -> float:
    vals = [(g.mins if fn is min else g.maxs).get(f) for g in (self.segments if key is None else self.by_key(key))]
    vals = [v for v in vals if v is not None]
    return fn(vals) if vals else math.nan


# --- episode detectors ---------------------------------------------------------------------------------------------
def gear_now(s: Sample, pt: Powertrain) -> int:
  """Gear for shift bookkeeping: the raw gear signal (1-N) when the tap is live, else the RPM/speed inference."""
  if s.gear_raw >= 0:
    return s.gear_raw if 1 <= s.gear_raw <= pt.num_gears else 0
  return s.gear


class ShiftDetector:
  """
  One episode per clutch-open period while rolling (the clutch role), with a 3 s post-window after the clutch closes:
  bite lag, decouple lag, lift-to-clutch, blips, gate states, resume jerk, throttle with the clutch in, false flares.
  """
  POST = 3.0
  CLOSE_DEBOUNCE = 0.15      # s the clutch must stay closed for the open period to end (shorter = a flicker)
  MIN_V = 2.0
  BITE_HOLD = 0.2            # s of ratio lock that counts as "clutch bit"
  GAS_LOOKBACK = 3.0
  FLARE_CONFIRM = 1.0        # s: a gate 'shifting' with no clutch open within this is a false flare
  REOPEN_LEAK = 1.0          # s: the clutch role re-opening this soon after a close = a raw-gear leak (Task 12 D3)

  def __init__(self, ctx: Context):
    self.ctx = ctx
    self.state = "closed"
    self.ep: dict | None = None
    self.last_gear = 0
    self.t_coupled: float | None = None
    self.t_gas_release: float | None = None
    self.gas_prev = False
    self.gate_prev = ""
    self.t_close_cand: float | None = None
    self._match_cand: float | None = None
    self.t_match: float | None = None
    self.jerk = JerkFilter()
    self.false_flares = 0
    self._flare_cand: float | None = None
    # Task 13 B2 [D-04]: a close that re-opens within REOPEN_LEAK is one clutch event split in two episodes; both halves
    # carry the same leak id, so a leak BEFORE the blip (first half, no blip) still marks the blipped second half
    self._leak_seq = 0
    self._leak_pending: int | None = None

  def _start(self, s: Sample) -> None:
    gas_dt = None
    if self.t_gas_release is not None and not s.gas and 0. <= s.t - self.t_gas_release <= self.GAS_LOOKBACK:
      gas_dt = s.t - self.t_gas_release
    coupled = _coupled(s.ratio, self.last_gear, self.ctx.powertrain)
    self.ep = {"t_open": s.t, "t_close": None, "dur": 0., "gear_before": self.last_gear, "gear_after": 0, "kind": "unknown",
               "v_open": s.v_ego, "pitch": s.pitch, "gas_release_dt": gas_dt, "blip": False, "flickers": 0,
               "gear_raw_open": [], "decouple_lag": None if coupled else s.t - (self.t_coupled or s.t),
               "gate_states": [], "gate_shift_dt": 0. if s.gate == "shifting" else None,
               "long_active_open": False, "accel_out_max_open": -math.inf, "rpm_max_open": s.rpm, "rpm_rate_max_open": 0.,
               "enabled_open": s.enabled, "enabled_min": s.enabled, "acc_fault": s.acc_faulted,
               "bite_lag": None, "resume_dt": None, "resume_jerk_max": 0., "post_cmd_max": -math.inf,
               "post_gas": False, "post_brake": False, "truncated": False,
               # Task 12: the gate's published state (A2: longitudinal stays active, the throttle is capped)
               "paused_open": s.gate_paused, "blip_cmd_abs": 0., "reopened": False, "close_gas": False,
               "leak": self._leak_pending}
    self._leak_pending = None
    self._decoupled_seen = not coupled
    self._flare_cand = None
    self.state = "open"

  def _finish(self, truncated: bool = False) -> dict:
    ep, self.ep = self.ep, None
    ep["truncated"] = truncated
    gb, ga = ep["gear_before"], ep["gear_after"]
    ep["kind"] = "unknown" if not (gb and ga) else ("up" if ga > gb else "down" if ga < gb else "same")
    ep["gear_raw_open"] = sorted(set(ep["gear_raw_open"]))
    ep["gate_states"] = list(dict.fromkeys(ep["gate_states"]))
    self.state = "closed"
    return ep

  def feed(self, s: Sample) -> list[dict]:
    done = []
    if self.gas_prev and not s.gas:
      self.t_gas_release = s.t
    self.gas_prev = s.gas

    # gate 'shifting' without a clutch-open: RPM backstop / gas-release hint, or a false flare
    if s.gate == "shifting" and self.gate_prev != "shifting" and self.state != "open" and not s.clutch:
      self._flare_cand = s.t
    if self._flare_cand is not None and s.t - self._flare_cand > self.FLARE_CONFIRM:
      self.false_flares += 1
      self._flare_cand = None
    self.gate_prev = s.gate

    ep = self.ep
    pt = self.ctx.powertrain
    if self.state == "post":
      ga = ep["gear_after"]
      if not ga and s.gear >= 1:
        ga = ep["gear_after"] = s.gear  # no raw gear signal: first inferred gear
      if ep["bite_lag"] is None and ga:
        if _coupled(s.ratio, ga, pt):
          self.t_match = s.t if self.t_match is None else self.t_match
          if s.t - self.t_match >= self.BITE_HOLD:
            ep["bite_lag"] = self.t_match - ep["t_close"]
        else:
          self.t_match = None
      ep["enabled_min"] = ep["enabled_min"] and s.enabled
      ep["acc_fault"] |= s.acc_faulted
      ep["post_gas"] |= s.gas
      ep["post_brake"] |= s.brake
      j = self.jerk.update(s.t, s.accel_cmd)
      # resume = the gate stops capping the throttle (published, Task 12 A2); without a published gate state that is
      # simply longitudinal being active again
      if ep["resume_dt"] is None and s.long_active and not s.gate_paused and ep["enabled_open"]:
        ep["resume_dt"] = s.t - ep["t_close"]
        self.jerk = JerkFilter()
        self.jerk.update(s.t, s.accel_cmd)
      elif ep["resume_dt"] is not None and s.t - ep["t_close"] - ep["resume_dt"] <= 2.:
        ep["resume_jerk_max"] = max(ep["resume_jerk_max"], abs(j))
      if ep["resume_dt"] is not None and not s.gas:
        ep["post_cmd_max"] = max(ep["post_cmd_max"], s.accel_cmd)
      if s.t - ep["t_close"] >= self.POST:
        done.append(self._finish())
      elif s.clutch and s.v_ego > self.MIN_V:
        ep["reopened"] = s.t - ep["t_close"] <= self.REOPEN_LEAK
        if ep["reopened"]:
          if ep["leak"] is None:
            self._leak_seq += 1
            ep["leak"] = self._leak_seq
          self._leak_pending = ep["leak"]
        done.append(self._finish(truncated=True))

    if self.state == "closed":
      if not s.clutch:
        g = gear_now(s, pt)
        if g:
          self.last_gear = g
        if _coupled(s.ratio, self.last_gear, pt):
          self.t_coupled = s.t
      elif s.v_ego > self.MIN_V:
        self._start(s)
      return done

    if self.state == "open":
      ep = self.ep
      if s.clutch:
        if self.t_close_cand is not None:
          ep["flickers"] += 1
          self.t_close_cand = None
        ep["blip"] |= s.gas
        ep["long_active_open"] |= s.long_active
        ep["paused_open"] |= s.gate_paused
        if s.gas:
          ep["blip_cmd_abs"] = max(ep["blip_cmd_abs"], abs(s.accel_cmd))
        if s.enabled:
          ep["accel_out_max_open"] = max(ep["accel_out_max_open"], s.accel_out)
        ep["rpm_max_open"] = max(ep["rpm_max_open"], s.rpm)
        ep["rpm_rate_max_open"] = max(ep["rpm_rate_max_open"], s.rpm_rate)
        if s.gear_raw >= 0:
          ep["gear_raw_open"].append(s.gear_raw)
        if s.gate:
          ep["gate_states"].append(s.gate)
        if ep["gate_shift_dt"] is None and s.gate == "shifting":
          ep["gate_shift_dt"] = s.t - ep["t_open"]
        if not self._decoupled_seen and not _coupled(s.ratio, ep["gear_before"], pt):
          self._decoupled_seen = True
          ep["decouple_lag"] = ep["t_open"] - s.t   # negative: the clutch role went open before the engine decoupled
        ep["enabled_min"] = ep["enabled_min"] and s.enabled
        ep["acc_fault"] |= s.acc_faulted
        ep["dur"] = s.t - ep["t_open"]
      else:
        if self.t_close_cand is None:
          self.t_close_cand = s.t
          self._gear_at_close = gear_now(s, pt)
          self._gas_at_close = s.gas
          self._match_cand = None
        # Task 15: track the ratio lock through the close debounce too - a gearbox that reports the gear only once the
        # drivetrain is locked would otherwise read every bite as >= CLOSE_DEBOUNCE
        if _coupled(s.ratio, gear_now(s, pt), pt):
          self._match_cand = s.t if self._match_cand is None else self._match_cand
        else:
          self._match_cand = None
        if s.t - self.t_close_cand >= self.CLOSE_DEBOUNCE:
          ep["t_close"] = self.t_close_cand
          ep["dur"] = ep["t_close"] - ep["t_open"]
          ep["gear_after"] = self._gear_at_close if s.gear_raw >= 0 else 0
          ep["close_gas"] = self._gas_at_close
          self.t_close_cand = None
          self.t_match = self._match_cand
          self.jerk = JerkFilter()
          self.state = "post"
          self.last_gear = ep["gear_after"] or self.last_gear
    return done


class HandbackDetector:
  """Task 5 handback: triggered by controlsd's manual_low_speed_handback event (or the alert, if logs are missing)."""
  WINDOW = 4.0

  def __init__(self, ctx: Context):
    self.ctx = ctx
    self.ep: dict | None = None
    self.alert_prev = False
    self.jerk = JerkFilter()

  def feed(self, s: Sample) -> list[dict]:
    done = []
    alert = s.alert == HANDBACK_ALERT
    alert_edge = alert and not self.alert_prev
    self.alert_prev = alert
    trigger = next((p for n, p in s.events if n == "manual_low_speed_handback" and p.get("process") == "controlsd"), None)
    if self.ep is None and (trigger is not None or alert_edge):
      p = trigger or {}
      ramp = float(p.get("ramp", self.ctx.handback_ramp))
      self.ep = {"t0": s.t, "v": float(p.get("v_ego", s.v_ego)), "accel": float(p.get("accel", s.accel_cmd)), "ramp": ramp,
                 "gear": gear_now(s, self.ctx.powertrain), "pitch": s.pitch, "alerts": 0, "jerk_max": 0., "disengaged": False,
                 "a_after": [], "v_ramp_end": None, "v_end": None, "from_alert": trigger is None}
      self.jerk = JerkFilter()
      self.jerk.update(s.t, s.a_ego)
    ep = self.ep
    if ep is None:
      return done
    dt = s.t - ep["t0"]
    ep["alerts"] += alert_edge
    ep["jerk_max"] = max(ep["jerk_max"], abs(self.jerk.update(s.t, s.a_ego)))
    ep["disengaged"] |= not s.enabled
    if dt >= ep["ramp"] and ep["v_ramp_end"] is None:
      ep["v_ramp_end"] = s.v_ego
    if ep["ramp"] + 0.1 <= dt <= ep["ramp"] + 2.1:
      ep["a_after"].append(s.a_ego)
    if dt >= self.WINDOW:
      ep["v_end"] = s.v_ego
      a = ep.pop("a_after")
      ep["a_after_mean"] = sum(a) / len(a) if a else math.nan
      done.append(ep)
      self.ep = None
    return done


class StopHoldDetector:
  """A stop after driving; scored once the driver releases the brake: does the car hold (Auto Brake Hold/hill-hold)?"""
  MAX = 60.

  def __init__(self, ctx: Context):
    self.ctx = ctx
    self.ep: dict | None = None
    self.t_moving: float | None = None

  def feed(self, s: Sample) -> list[dict]:
    done = []
    if s.v_ego > 2.:
      self.t_moving = s.t
    if self.ep is None:
      if s.v_ego < 0.1 and self.t_moving is not None and s.t - self.t_moving < 30.:
        self.ep = {"t0": s.t, "pitch": s.pitch, "released_s": 0., "brake_hold": False, "roll_v": 0., "_t": s.t}
        self.t_moving = None
      return done
    ep = self.ep
    dt = min(s.t - ep["_t"], MAX_DT)
    ep["_t"] = s.t
    ep["brake_hold"] |= s.brake_hold
    if not s.brake and not s.gas:
      ep["released_s"] += dt
      ep["roll_v"] = max(ep["roll_v"], s.v_ego)
    drive_off = s.gas or (1 <= s.gear_raw <= self.ctx.powertrain.num_gears if s.gear_raw >= 0 else not s.clutch) or s.v_ego > 3.
    if drive_off or s.t - ep["t0"] > self.MAX:
      self.ep = None
      ep.pop("_t")
      ep["rolled"] = ep["roll_v"] > ROLL_SPEED
      if ep["released_s"] >= 1. or ep["rolled"]:
        done.append(ep)
    return done


class NeutralCoastDetector:
  """Engaged, rolling, engine decoupled from every gear and near idle: coasting in neutral (or clutch held)."""
  MIN = 5.0

  def __init__(self, ctx: Context):
    self.ctx = ctx
    self.ep: dict | None = None
    self.t_start: float | None = None

  def feed(self, s: Sample) -> list[dict]:
    decoupled = s.enabled and s.v_ego > 3. and not s.gas and not s.brake and s.rpm < 1500. and \
                self.ctx.powertrain.infer_gear(s.rpm, s.v_ego) == 0
    if self.ep is None:
      if decoupled:
        self.t_start = s.t if self.t_start is None else self.t_start
        if s.t - self.t_start >= 1.:
          self.ep = {"t0": self.t_start, "gear_raw": defaultdict(int), "n": 0, "clutch_n": 0, "long_n": 0, "gate_states": []}
      else:
        self.t_start = None
      if self.ep is None:
        return []
    ep = self.ep
    if not decoupled:
      self.ep, self.t_start = None, None
      ep["dur"] = s.t - ep["t0"]
      ep["gear_raw"] = dict(ep["gear_raw"])
      ep["gate_states"] = list(dict.fromkeys(ep["gate_states"]))
      return [ep] if ep["dur"] >= self.MIN else []
    ep["n"] += 1
    ep["gear_raw"][s.gear_raw] += 1
    ep["clutch_n"] += s.clutch
    ep["long_n"] += s.long_active
    if s.gate:
      ep["gate_states"].append(s.gate)
    return []


class BackstopDetector:
  """
  NSB overspeed backstop, by NoServiceBrakeOverspeedMode:
    maintain  braking re-allowed above set + margin (cloudlog event, or allowBrake rising): one 3 s episode.
    pulse     (Task 11) one episode per brake pulse, from the planner's no_service_brake_pulse phase events (pulsing ..
              coasting): where it started / released vs the band, the command range while bounded, whether it
              escalated and how many "Overspeed: Take Over" alerts, and the coast before it (did the brake let go?).
  """
  WINDOW = 3.0

  def __init__(self, ctx: Context):
    self.ctx = ctx
    self.ep: dict | None = None
    self.allow_prev = True
    self.alert_prev = False
    self.t_released: float | None = None   # pulse mode: last release (start of the coast)
    self.coast_cmd_min = math.inf          # min command in that coast once the roll-off is done

  def feed(self, s: Sample) -> list[dict]:
    if self.ctx.nsb_mode == "pulse":
      return self._feed_pulse(s)
    trigger = next((p for n, p in s.events if n == "no_service_brake_backstop"), None)
    rising = s.plan_valid and s.allow_brake and not self.allow_prev
    self.allow_prev = s.allow_brake if s.plan_valid else self.allow_prev
    if self.ep is None and (trigger is not None or (rising and s.long_active and math.isfinite(s.v_cruise) and s.v_ego > s.v_cruise)):
      v_set = float(trigger["v_set"]) if trigger is not None and "v_set" in trigger else s.v_cruise
      self.ep = {"t0": s.t, "v_over": s.v_ego - v_set, "margin": self.ctx.nsb_margin, "a_min": s.a_ego, "cmd_min": s.accel_cmd,
                 "excess_min": s.excess, "from_event": trigger is not None}
      return []
    ep = self.ep
    if ep is None:
      return []
    ep["a_min"] = min(ep["a_min"], s.a_ego)
    ep["cmd_min"] = min(ep["cmd_min"], s.accel_cmd)
    if math.isfinite(s.excess):
      ep["excess_min"] = min(ep["excess_min"], s.excess) if math.isfinite(ep["excess_min"]) else s.excess
    if s.t - ep["t0"] >= self.WINDOW:
      self.ep = None
      return [ep]
    return []

  def _feed_pulse(self, s: Sample) -> list[dict]:
    c = self.ctx
    alert = s.alert == OVERSPEED_ALERT
    alert_edge = alert and not self.alert_prev
    self.alert_prev = alert
    done = []
    for name, p in s.events:
      if name != "no_service_brake_pulse":
        continue
      phase, over = p.get("phase"), float(p.get("overspeed", math.nan))
      if phase == "pulsing" and self.ep is None:
        coast = None if self.t_released is None else s.t - self.t_released
        self.ep = {"mode": "pulse", "t0": s.t, "v_over": over, "trigger": c.nsb_trigger, "release": c.nsb_release,
                   "decel": c.nsb_pulse_decel, "floor": c.nsb_floor, "cmd_min": s.accel_cmd, "cmd_max": s.accel_cmd,
                   "a_min": s.a_ego, "escalated": False, "v_over_esc": None, "alerts": 0, "v_over_end": None, "dur": 0.,
                   "coast_before": coast, "coast_cmd_min": self.coast_cmd_min if coast is not None else None}
      elif phase == "escalated" and self.ep is not None:
        self.ep["escalated"], self.ep["v_over_esc"] = True, over
      elif phase == "coasting" and self.ep is not None:
        ep, self.ep = self.ep, None
        ep["v_over_end"], ep["dur"] = over, s.t - ep["t0"]
        self.t_released, self.coast_cmd_min = s.t, math.inf
        done.append(ep)
    ep = self.ep
    if ep is not None:
      if not s.long_active:  # disengaged mid-pulse: the feature resets silently; not a scorable pulse
        self.ep, self.t_released = None, None
        return done
      ep["alerts"] += alert_edge
      ep["a_min"] = min(ep["a_min"], s.a_ego)
      if not ep["escalated"]:  # the pulse bound applies until an escalation
        ep["cmd_min"] = min(ep["cmd_min"], s.accel_cmd)
        ep["cmd_max"] = max(ep["cmd_max"], s.accel_cmd)
    elif self.t_released is not None:
      if not s.long_active:
        self.t_released = None
      elif s.t - self.t_released > (abs(c.nsb_pulse_decel) - c.nsb_floor) / c.nsb_pulse_jerk + 0.3:
        self.coast_cmd_min = min(self.coast_cmd_min, s.accel_cmd)  # after the roll-off: back on the coast clamp?
    return done


class EbpSuppressDetector:
  """EngineBrakePreferred (Task 17: the per-gear clamp) holding braking off: from a cloudlog engine_brake_clamp
  binding=True (the plan wanted more decel than the floor) to binding=False / the clamp ending / long inactive. Per
  episode: the gear, the floor, the lowest command (out and as sent) and how often it was below the published floor
  (a fault: the clamp's floor is the lowest request allowed), the excess decel vs coasting (service brakes)."""

  def __init__(self, ctx: Context):
    self.ep: dict | None = None

  def feed(self, s: Sample) -> list[dict]:
    done = []
    end = False
    for name, p in s.events:
      if name == "engine_brake_clamp" and p.get("binding") and self.ep is None:
        self.ep = {"t0": s.t, "gear": p.get("gear"), "floor": p.get("floor"), "out_min": s.accel_out,
                   "excess_min": math.inf, "reason": None, "sent_min": math.inf, "assist": 0, "under_floor": 0}
      elif self.ep is not None and ((name == "engine_brake_clamp" and not p.get("binding")) or
                                    (name == "engine_brake_preferred" and not p.get("active"))):
        self.ep["reason"] = p.get("reason", "released")
        end = True
    ep = self.ep
    if ep is None:
      return done
    ep["out_min"] = min(ep["out_min"], s.accel_out)
    if math.isfinite(s.excess):
      ep["excess_min"] = min(ep["excess_min"], s.excess)
    if math.isfinite(s.accel_sent):   # what actually went on the bus, and the brake-request bit (Task 12 D3)
      ep["sent_min"] = min(ep["sent_min"], s.accel_sent)
    if s.ebp and math.isfinite(s.ebp_floor) and not end:
      below = s.accel_out < s.ebp_floor - 0.01 or (math.isfinite(s.accel_sent) and s.accel_sent < s.ebp_floor - 0.01)
      ep["under_floor"] += int(below)
    ep["assist"] += s.brake_assist > 0
    if end or not s.long_active:
      ep["dur"] = s.t - ep["t0"]
      self.ep = None
      done.append(ep)
    return done


class LeadApproachDetector:
  """Closing on a lead with openpilot long active: min gap / TTC, whether EBP suppressed, FCW."""

  def __init__(self, ctx: Context):
    self.ctx = ctx
    self.ep: dict | None = None
    self.t_open: float | None = None

  def feed(self, s: Sample) -> list[dict]:
    closing = s.long_active and s.has_lead and s.lead_vrel < -1.
    if self.ep is None:
      if closing:
        self.ep = {"t0": s.t, "ebp_on": self.ctx.toggles.get("EngineBrakePreferred", False), "gap_min": s.lead_d,
                   "ttc_min": math.inf, "fcw": False, "ebp_seen": False, "release_reasons": [], "cmd_min": s.accel_cmd}
        self.t_open = None
      return []
    ep = self.ep
    if s.has_lead:
      ep["gap_min"] = min(ep["gap_min"], s.lead_d)
      if s.lead_vrel < -0.1:
        ep["ttc_min"] = min(ep["ttc_min"], s.lead_d / -s.lead_vrel)
    ep["fcw"] |= s.fcw
    ep["ebp_seen"] |= s.ebp
    ep["cmd_min"] = min(ep["cmd_min"], s.accel_cmd)
    ep["release_reasons"] += [p.get("reason") for n, p in s.events if n == "engine_brake_preferred" and not p.get("active")]
    opening = s.has_lead and s.lead_vrel > -0.2
    self.t_open = (self.t_open or s.t) if opening else None
    if not s.has_lead or not s.long_active or (self.t_open is not None and s.t - self.t_open >= 1.):
      self.ep = None
      ep["dur"] = s.t - ep["t0"]
      return [ep] if ep["dur"] >= 2. else []
    return []


class BrakePressDetector:
  """Brake pedal pressed while engaged: how long until openpilot disengages (stock: immediately)."""

  def __init__(self, ctx: Context):
    self.ep: dict | None = None
    self.brake_prev = False

  def feed(self, s: Sample) -> list[dict]:
    edge = s.brake and not self.brake_prev
    self.brake_prev = s.brake
    if self.ep is None:
      if edge and s.enabled:
        self.ep = {"t0": s.t, "disengage_dt": None}
      return []
    ep = self.ep
    if not s.enabled:
      ep["disengage_dt"] = s.t - ep["t0"]
    if ep["disengage_dt"] is not None or s.t - ep["t0"] > 1.:
      self.ep = None
      return [ep]
    return []


DETECTORS = {
  "shift": ShiftDetector, "handback": HandbackDetector, "stop_hold": StopHoldDetector, "neutral_coast": NeutralCoastDetector,
  "backstop": BackstopDetector, "ebp_suppress": EbpSuppressDetector, "lead_approach": LeadApproachDetector,
  "brake_press": BrakePressDetector,
}

FILTERS = {
  "after_gas": lambda e: e["gas_release_dt"] is not None,
  "blip": lambda e: e["blip"],
  "engaged_open": lambda e: e["enabled_open"] and e["dur"] >= 0.5,
  "engaged_shift": lambda e: e["enabled_open"] and e["kind"] in ("up", "down"),
  "engaged_blip": lambda e: e["enabled_open"] and e["blip"],
  "engaged_downshift_no_pedal": lambda e: e["enabled_open"] and e["kind"] == "down" and not e["post_gas"] and not e["post_brake"],
  "uphill_on_gas": lambda e: math.isfinite(e["pitch"]) and e["pitch"] > HILL_STOP_PITCH and e["accel"] > 0.,
}

EPISODE_KEYS = {
  "shift": lambda e: e["kind"],
  "stop_hold": lambda e: "hill" if math.isfinite(e["pitch"]) and abs(e["pitch"]) > HILL_STOP_PITCH else "flat",
  "lead_approach": lambda e: "ebp_on" if e["ebp_on"] else "ebp_off",
}

EPISODE_NOUN = {"shift": "shifts", "handback": "handbacks", "stop_hold": "stops", "neutral_coast": "coasts",
                "backstop": "backstops", "ebp_suppress": "trims", "lead_approach": "approaches", "brake_press": "brake presses"}


class EpisodeAnalyzer(Analyzer):
  def __init__(self, spec: TestSpec, ctx: Context):
    super().__init__(spec, ctx)
    g = self.goal
    self.det_name = g["detector"]
    self.detector = DETECTORS[self.det_name](ctx)
    self.filter = FILTERS.get(g.get("filter", ""), lambda e: True)
    self._count = g["count"]
    self._count_by = g.get("count_by")   # (Context attribute, {value: count}): e.g. NSB2 needs 2 pulses in pulse mode
    self.need: dict[str, int] = g.get("need", {})
    self.episodes: list[dict] = []
    self.rejected = 0
    extra = g.get("extra")
    self.extra_name = extra["detector"] if extra else None
    self.extra = DETECTORS[extra["detector"]](ctx) if extra else None
    self.extra_count = extra["count"] if extra else 0
    self.extra_episodes: list[dict] = []

  def _feed(self, s: Sample, ok: bool, dt: float) -> list[str]:
    # detectors see every sample of the window (a maneuver spans condition changes); the test's conditions gate
    # whether an episode counts: they must hold when it starts
    notes = []
    for ep in self.detector.feed(s):
      if ep.get("_ok", True) and self.filter(ep):
        self.episodes.append(ep)
        notes.append(f"{EPISODE_NOUN[self.det_name]} {len(self.episodes)}")
      else:
        self.rejected += 1
    if self.detector_ep_open() and "_ok" not in self.detector.ep:
      self.detector.ep["_ok"] = ok
    if self.extra is not None:
      for ep in self.extra.feed(s):
        self.extra_episodes.append(ep)
        notes.append(f"{EPISODE_NOUN[self.extra_name]} {len(self.extra_episodes)}")
    return notes

  @property
  def count(self) -> int:
    if self._count_by is None:
      return self._count
    attr, table = self._count_by
    return table.get(getattr(self.ctx, attr), self._count)

  def detector_ep_open(self) -> bool:
    return getattr(self.detector, "ep", None) is not None

  def keyed(self, key: str) -> list[dict]:
    fn = EPISODE_KEYS[self.det_name]
    return [e for e in self.episodes if fn(e) == key]

  def progress(self) -> tuple[str, float]:
    n = len(self.episodes)
    text = f"{EPISODE_NOUN[self.det_name]} {n}/{self.count}"
    fracs = [min(1., n / self.count)]
    if self.need:
      text += " (" + ", ".join(f"{k} {len(self.keyed(k))}/{v}" for k, v in self.need.items()) + ")"
      fracs += [min(1., len(self.keyed(k)) / v) for k, v in self.need.items()]
    if self.extra is not None:
      text += f" • {EPISODE_NOUN[self.extra_name]} {len(self.extra_episodes)}/{self.extra_count}"
      fracs.append(min(1., len(self.extra_episodes) / self.extra_count))
    return text, min(fracs)

  def finish(self) -> list[str]:
    return []


# --- custom analyzers ----------------------------------------------------------------------------------------------
class GearRawMapAnalyzer(Analyzer):
  """The car's raw gear signal vs the RPM/speed ratio per gear, what it reads at a stop and in reverse."""

  def __init__(self, spec: TestSpec, ctx: Context):
    super().__init__(spec, ctx)
    self.need_s = self.goal["seconds_per_gear"]
    self.stop_need = self.goal["stop_seconds"]
    self.total: dict[int, float] = defaultdict(float)
    self.agree: dict[int, float] = defaultdict(float)
    self.inferred_when: dict[int, dict[int, float]] = defaultdict(lambda: defaultdict(float))
    self.stop_hist: dict[int, float] = defaultdict(float)
    self.rev_hist: dict[int, int] = defaultdict(int)

  def _feed(self, s: Sample, ok: bool, dt: float) -> list[str]:
    if not ok:
      return []
    notes = []
    pt, name = self.ctx.powertrain, self.ctx.label("gear_raw", "raw gear")
    if s.reverse:
      if not self.rev_hist:
        notes.append(f"reverse {name}={s.gear_raw}")
      self.rev_hist[s.gear_raw] += 1
    elif s.standstill or s.v_ego < 0.1:
      self.stop_hist[s.gear_raw] += dt
    elif s.v_ego > 3. and not s.gas:
      g_inf = pt.infer_gear(s.rpm, s.v_ego)  # deliberately ignores the clutch role: it may BE the raw gear reading 0
      if 1 <= s.gear_raw <= pt.num_gears or g_inf:
        before = self.total[s.gear_raw]
        self.total[s.gear_raw] += dt
        self.agree[s.gear_raw] += dt if g_inf == s.gear_raw else 0.
        self.inferred_when[s.gear_raw][g_inf] += dt
        if before < self.need_s <= self.total[s.gear_raw]:
          notes.append(f"{name} {s.gear_raw} covered")
    return notes

  def progress(self) -> tuple[str, float]:
    n = self.ctx.powertrain.num_gears
    got = [g for g in range(1, n + 1) if self.total.get(g, 0.) >= self.need_s]
    stop = sum(self.stop_hist.values())
    fracs = [len(got) / n if n else 0., min(1., stop / self.stop_need), 1. if self.rev_hist else 0.]
    text = f"gears {len(got)}/{n} • stop {stop:.0f}/{self.stop_need:.0f}s • R {'✓' if self.rev_hist else '×'}"
    return text, min(fracs)


class BitHuntAnalyzer(Analyzer):
  """
  Which bit of the profile's raw payload (role raw_payload) is a driver input such as a clutch switch. Two scorers on the
  same per-bit toggle record:
    - rolling in gear: a bit that toggles with the raw gear signal -> 0 (a clutch switch would lead it);
    - parked in neutral (raw gear reads 0 throughout): the driver's press signature (the plan's rhythm) — a bit whose
      toggles form 3 groups (split at >= SIG_GAP s of stillness) of 2N toggles each (press + release) is an N-press
      candidate. Complete at `presses` raw-gear presses or `seconds` of window, whichever first.
  """
  WINDOW = 0.3
  SIG_GAP = 3.0      # s of stillness that separates the signature's groups (quick <1 s and slow ~2 s gaps stay inside)
  TAIL = 1.0         # s after the last press before the press count completes the window (its release lands) - D-19
  CACHE_PERIOD = 2.0  # s: the scorers re-run at most at 0.5 Hz, and only on new edges/toggles or a window close - D-07

  def __init__(self, spec: TestSpec, ctx: Context):
    super().__init__(spec, ctx)
    self.presses = self.goal["presses"]
    self.seconds = self.goal.get("seconds")
    self.signature_n = self.goal.get("signature_n")
    self.window_ok = 0.
    self.prev_raw: int | None = None
    self.prev_mt: int | None = None
    self.toggles: dict[int, list[float]] = defaultdict(list)
    self.edges: list[float] = []      # raw gear 0-edges (in and out)
    self.press_count = 0
    self.t_now = 0.
    self.t_last_edge = -math.inf
    self._cache: dict[str, tuple[tuple, float, object]] = {}
    self._closed = 0

  def _feed(self, s: Sample, ok: bool, dt: float) -> list[str]:
    self.t_now = s.t
    if not ok or s.raw_payload < 0:
      return []
    notes = []
    self.window_ok += dt
    if self.prev_raw is not None:
      diff = s.raw_payload ^ self.prev_raw
      while diff:
        bit = (diff & -diff).bit_length() - 1
        if len(self.toggles[bit]) < 2000:
          self.toggles[bit].append(s.t)
        diff &= diff - 1
    if self.prev_mt is not None and (s.gear_raw == 0) != (self.prev_mt == 0):
      self.edges.append(s.t)
      self.t_last_edge = s.t
      if s.gear_raw == 0:
        self.press_count += 1
        notes.append(f"clutch press {self.press_count}")
    self.prev_raw, self.prev_mt = s.raw_payload, s.gear_raw
    return notes

  def progress(self) -> tuple[str, float]:
    text = f"clutch presses {self.press_count}/{self.presses}"
    frac = self.press_count / self.presses
    if frac >= 1. and self.t_now - self.t_last_edge < self.TAIL:
      frac = 0.99   # D-19: hold the window open a moment so the last release is in it
    if self.seconds:
      text += f" • window {self.window_ok:.0f}/{self.seconds:.0f}s"
      frac = max(frac, self.window_ok / self.seconds)
    return text, min(1., frac)

  @staticmethod
  def dbc_bit(bit: int) -> int:
    """LSB-first index of the big-endian 64-bit payload -> DBC (Motorola) bit number byte*8 + bit-in-byte."""
    return (7 - bit // 8) * 8 + bit % 8

  @classmethod
  def signature_count(cls, ts: list[float]) -> int:
    """N if the toggle times form the 3-group N-press signature (2N toggles per group), else 0."""
    if len(ts) < 6:
      return 0
    groups, start = [], 0
    for i in range(1, len(ts) + 1):
      if i == len(ts) or ts[i] - ts[i - 1] >= cls.SIG_GAP:
        groups.append(i - start)
        start = i
    return groups[0] // 2 if len(groups) == 3 and len(set(groups)) == 1 and groups[0] % 2 == 0 else 0

  def finish(self) -> list[str]:
    self._closed += 1   # a window close always re-scores
    return []

  def _cached(self, name: str, fn):
    key = (len(self.edges), sum(len(ts) for ts in self.toggles.values()), self._closed)
    hit = self._cache.get(name)
    if hit is not None and (hit[0] == key or (self.t_now - hit[1] < self.CACHE_PERIOD and hit[0][2] == key[2])):
      return hit[2]
    out = fn()
    self._cache[name] = (key, self.t_now, out)
    return out

  def signature_candidates(self) -> list[tuple[int, int]]:
    """(DBC bit, N) for every bit whose toggles form the press signature; the registry's N first. Cached (D-07)."""
    def compute():
      out = [(self.dbc_bit(bit), n) for bit, ts in self.toggles.items() if (n := self.signature_count(ts))]
      return sorted(out, key=lambda c: (c[1] != self.signature_n, c[0]))
    return self._cached("signature", compute)

  def candidates(self) -> list[tuple[int, float, int]]:
    return self._cached("edges", self._candidates)

  def _candidates(self) -> list[tuple[int, float, int]]:
    out = []
    n = len(self.edges)
    if not n:
      return out
    for bit, ts in self.toggles.items():
      hits = sum(any(abs(t - e) <= self.WINDOW for t in ts) for e in self.edges)
      if hits / n >= 0.8 and len(ts) <= 2.5 * n:
        out.append((self.dbc_bit(bit), hits / n, len(ts)))
    return sorted(out, key=lambda c: -c[1])


class NonAdaptiveAnalyzer(Analyzer):
  """N1: set-speed hold error (no lead) and command while behind a slower lead (must not brake for it)."""

  def __init__(self, spec: TestSpec, ctx: Context):
    super().__init__(spec, ctx)
    self.hold_need, self.lead_need = self.goal["hold_seconds"], self.goal["lead_seconds"]
    self.hold_s = self.lead_s = 0.
    self.err_sum = 0.
    self.err_hist: dict[int, float] = defaultdict(float)   # |err| in 0.1 m/s bins, weighted by time
    self.lead_cmd_min = math.inf

  def _feed(self, s: Sample, ok: bool, dt: float) -> list[str]:
    if not ok or s.gas or s.brake or not math.isfinite(s.v_cruise):
      return []
    notes = []
    if not s.has_lead or s.lead_d > 80.:
      before = self.hold_s
      err = s.v_ego - s.v_cruise
      self.hold_s += dt
      self.err_sum += err * dt
      self.err_hist[min(int(abs(err) * 10), 100)] += dt
      if before < self.hold_need <= self.hold_s:
        notes.append("hold covered")
    elif s.lead_vrel < -0.5 and s.v_ego <= s.v_cruise + 0.3:
      before = self.lead_s
      self.lead_s += dt
      self.lead_cmd_min = min(self.lead_cmd_min, s.accel_cmd)
      if before < self.lead_need <= self.lead_s:
        notes.append("slower lead covered")
    return notes

  def progress(self) -> tuple[str, float]:
    return (f"hold {self.hold_s:.0f}/{self.hold_need:.0f}s • slower lead {self.lead_s:.0f}/{self.lead_need:.0f}s",
            min(1., self.hold_s / self.hold_need, self.lead_s / self.lead_need))

  def err_p95(self) -> float:
    total = sum(self.err_hist.values())
    acc = 0.
    for b in sorted(self.err_hist):
      acc += self.err_hist[b]
      if acc >= 0.95 * total:
        return (b + 1) / 10.
    return math.nan


ANALYZERS = {"segments": SegmentAnalyzer, "episodes": EpisodeAnalyzer, "gear_raw_map": GearRawMapAnalyzer,
             "bit_hunt": BitHuntAnalyzer, "non_adaptive": NonAdaptiveAnalyzer}


def make_analyzer(spec: TestSpec, ctx: Context) -> Analyzer:
  return ANALYZERS[spec.analyzer](spec, ctx)


# --- results: (status, detail). status None = nothing to score yet --------------------------------------------------
# The plan's pass rules live in the profile (VehicleProfile.results); these generic ones are available to every plan.
def pct(values: list[float], q: float) -> float:
  return _pct(values, q)


def fmt(x, nd=2) -> str:
  return _fmt(x, nd)


def health(a: Analyzer) -> list[str]:
  """Window-wide faults: ACC fault edges, panda-blocked TX frames, fault alerts."""
  bad = []
  if a.acc_fault_edges:
    bad.append(f"ACC fault x{a.acc_fault_edges}")
  if a.tx_blocked:
    bad.append(f"{a.tx_blocked} blocked TX")
  if a.fault_alerts:
    bad.append("alerts: " + ",".join(sorted(a.fault_alerts)))
  return bad


def r_baseline(a: SegmentAnalyzer):
  if a.window_s <= 0.:
    return None, ""
  bad = health(a)
  secs = a._live("all")[0]
  return (FAIL, "; ".join(bad)) if bad else (PASS, f"{secs:.0f}s engaged: no ACC fault, 0 blocked TX, no fault alerts")


RESULTS = {"baseline": r_baseline}


def results_for(ctx: Context) -> dict:
  """The result rules a plan can name: the generic ones plus the profile's."""
  return {**RESULTS, **(ctx.profile.results if ctx.profile is not None else {})}
