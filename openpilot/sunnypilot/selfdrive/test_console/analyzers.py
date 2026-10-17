"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console analyzers: turn the sample stream of one capture window into progress ("gear 3: 4/10 s"), a "complete"
flag and a result (PASS / FAIL / CAPTURED + a one-line detail). Analysis only — nothing here can reach a control path.

  SegmentAnalyzer   qualifying time in contiguous segments, optionally binned (per gear, per command band, ...)
  EpisodeAnalyzer   counts detected maneuvers (shifts, handbacks, stops, ...) and measures each one
  + a few custom analyzers (S1 GEAR_MT map, S6 clutch-bit hunt, N1 non-adaptive)

Thresholds marked "provisional" are first guesses for the car; the raw captures are kept so they can be re-scored.
"""
import math
from collections import defaultdict

from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import FLARE_RPM_RATE, GAS_RELEASE_WINDOW, REENGAGE_DELAY
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import GEAR_MATCH_TOL, GEAR_RATIOS, infer_gear, overall_ratio
from openpilot.sunnypilot.selfdrive.test_console.conditions import BINS, Context
from openpilot.sunnypilot.selfdrive.test_console.registry import TestSpec
from openpilot.sunnypilot.selfdrive.test_console.signals import FAULT_ALERTS, HANDBACK_ALERT, Sample

PASS, FAIL, CAPTURED = "PASS", "FAIL", "CAPTURED"

MAX_DT = 0.1                 # s, a gap longer than this between samples ends a segment
STAT_FIELDS = ("v_ego", "a_ego", "accel_cmd", "accel_out", "accel_sent", "a_target", "a_coast", "excess", "pitch", "rpm",
               "ratio", "rpm_rate", "v_cruise", "pedal_gas")
JERK_RC = 0.1                # s, smoothing for command jerk
RESUME_JERK_MAX = 3.0        # m/s^3, provisional "smooth resume" bound
HANDBACK_JERK_MAX = 2.5      # m/s^3, provisional "no lurch" bound
ROLL_SPEED = 0.3             # m/s
HILL_STOP_PITCH = 0.03       # rad (~3 %)


def _pct(values: list[float], q: float) -> float:
  v = sorted(values)
  if not v:
    return math.nan
  return v[min(len(v) - 1, max(0, math.ceil(q * len(v)) - 1))]


def _fmt(x, nd=2) -> str:
  return "n/a" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{nd}f}"


def _coupled(ratio: float, gear: int) -> bool:
  return 1 <= gear <= len(GEAR_RATIOS) and ratio > 0. and abs(ratio / overall_ratio(gear) - 1.) <= GEAR_MATCH_TOL


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
    fn = RESULTS.get(self.spec.result)
    return fn(self) if fn is not None else (None, "")


# --- segments ------------------------------------------------------------------------------------------------------
class Segment:
  __slots__ = ("key", "t0", "t_last", "dur", "n", "sums", "sumsq", "cnts", "mins", "maxs", "flares", "rpm_changes",
               "jerk_max", "jerk_sum", "assist_on", "assist_n", "_rpm_prev", "_jerk")

  def __init__(self, key, t: float):
    self.key, self.t0, self.t_last, self.dur, self.n = key, t, t, 0., 0
    self.sums: dict[str, float] = defaultdict(float)
    self.sumsq: dict[str, float] = defaultdict(float)
    self.cnts: dict[str, int] = defaultdict(int)
    self.mins: dict[str, float] = {}
    self.maxs: dict[str, float] = {}
    self.flares = self.rpm_changes = self.assist_on = self.assist_n = 0
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

  def extreme(self, f: str, fn=min, key=None) -> float:
    vals = [(g.mins if fn is min else g.maxs).get(f) for g in (self.segments if key is None else self.by_key(key))]
    vals = [v for v in vals if v is not None]
    return fn(vals) if vals else math.nan


# --- episode detectors ---------------------------------------------------------------------------------------------
def gear_now(s: Sample) -> int:
  """Gear for shift bookkeeping: raw GEAR_MT 1-6 when the tap is live, else the RPM/speed inference."""
  if s.gear_mt >= 0:
    return s.gear_mt if 1 <= s.gear_mt <= 6 else 0
  return s.gear


class ShiftDetector:
  """
  One episode per clutch-open period while rolling (carState clutchPressed; Honda: GEAR_MT == 0), with a 3 s
  post-window after the clutch closes. Measures what S2/S4/S5 and G1-G4 need.
  """
  POST = 3.0
  CLOSE_DEBOUNCE = 0.15      # s the clutch must stay closed for the open period to end (shorter = a flicker)
  MIN_V = 2.0
  BITE_HOLD = 0.2            # s of ratio lock that counts as "clutch bit"
  GAS_LOOKBACK = 3.0
  FLARE_CONFIRM = 1.0        # s: a gate 'shifting' with no clutch open within this is a false flare

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
    self.t_match: float | None = None
    self.jerk = JerkFilter()
    self.false_flares = 0
    self._flare_cand: float | None = None

  def _start(self, s: Sample) -> None:
    gas_dt = None
    if self.t_gas_release is not None and not s.gas and 0. <= s.t - self.t_gas_release <= self.GAS_LOOKBACK:
      gas_dt = s.t - self.t_gas_release
    coupled = _coupled(s.ratio, self.last_gear)
    self.ep = {"t_open": s.t, "t_close": None, "dur": 0., "gear_before": self.last_gear, "gear_after": 0, "kind": "unknown",
               "v_open": s.v_ego, "pitch": s.pitch, "gas_release_dt": gas_dt, "blip": False, "flickers": 0,
               "gear_mt_open": [], "decouple_lag": None if coupled else s.t - (self.t_coupled or s.t),
               "gate_states": [], "gate_shift_dt": 0. if s.gate == "shifting" else None,
               "long_active_open": False, "accel_out_max_open": -math.inf, "rpm_max_open": s.rpm, "rpm_rate_max_open": 0.,
               "enabled_open": s.enabled, "enabled_min": s.enabled, "acc_fault": s.acc_faulted,
               "bite_lag": None, "resume_dt": None, "resume_jerk_max": 0., "post_cmd_max": -math.inf,
               "post_gas": False, "post_brake": False, "truncated": False}
    self._decoupled_seen = not coupled
    self._flare_cand = None
    self.state = "open"

  def _finish(self, truncated: bool = False) -> dict:
    ep, self.ep = self.ep, None
    ep["truncated"] = truncated
    gb, ga = ep["gear_before"], ep["gear_after"]
    ep["kind"] = "unknown" if not (gb and ga) else ("up" if ga > gb else "down" if ga < gb else "same")
    ep["gear_mt_open"] = sorted(set(ep["gear_mt_open"]))
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
    if self.state == "post":
      ga = ep["gear_after"]
      if not ga and s.gear >= 1:
        ga = ep["gear_after"] = s.gear  # no raw GEAR_MT: first inferred gear
      if ep["bite_lag"] is None and ga:
        if _coupled(s.ratio, ga):
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
      if ep["resume_dt"] is None and s.long_active and ep["enabled_open"]:
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
        done.append(self._finish(truncated=True))

    if self.state == "closed":
      if not s.clutch:
        g = gear_now(s)
        if g:
          self.last_gear = g
        if _coupled(s.ratio, self.last_gear):
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
        if s.enabled:
          ep["accel_out_max_open"] = max(ep["accel_out_max_open"], s.accel_out)
        ep["rpm_max_open"] = max(ep["rpm_max_open"], s.rpm)
        ep["rpm_rate_max_open"] = max(ep["rpm_rate_max_open"], s.rpm_rate)
        if s.gear_mt >= 0:
          ep["gear_mt_open"].append(s.gear_mt)
        if s.gate:
          ep["gate_states"].append(s.gate)
        if ep["gate_shift_dt"] is None and s.gate == "shifting":
          ep["gate_shift_dt"] = s.t - ep["t_open"]
        if not self._decoupled_seen and not _coupled(s.ratio, ep["gear_before"]):
          self._decoupled_seen = True
          ep["decouple_lag"] = ep["t_open"] - s.t   # negative: GEAR_MT went 0 before the engine decoupled
        ep["enabled_min"] = ep["enabled_min"] and s.enabled
        ep["acc_fault"] |= s.acc_faulted
        ep["dur"] = s.t - ep["t_open"]
      else:
        if self.t_close_cand is None:
          self.t_close_cand = s.t
          self._gear_at_close = gear_now(s)
        if s.t - self.t_close_cand >= self.CLOSE_DEBOUNCE:
          ep["t_close"] = self.t_close_cand
          ep["dur"] = ep["t_close"] - ep["t_open"]
          ep["gear_after"] = self._gear_at_close if s.gear_mt >= 0 else 0
          self.t_close_cand = None
          self.t_match = None
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
                 "gear": gear_now(s), "pitch": s.pitch, "alerts": 0, "jerk_max": 0., "disengaged": False,
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
    drive_off = s.gas or (1 <= s.gear_mt <= 6 if s.gear_mt >= 0 else not s.clutch) or s.v_ego > 3.
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
    self.ep: dict | None = None
    self.t_start: float | None = None

  def feed(self, s: Sample) -> list[dict]:
    decoupled = s.enabled and s.v_ego > 3. and not s.gas and not s.brake and s.rpm < 1500. and infer_gear(s.rpm, s.v_ego) == 0
    if self.ep is None:
      if decoupled:
        self.t_start = s.t if self.t_start is None else self.t_start
        if s.t - self.t_start >= 1.:
          self.ep = {"t0": self.t_start, "gear_mt": defaultdict(int), "n": 0, "clutch_n": 0, "long_n": 0, "gate_states": []}
      else:
        self.t_start = None
      if self.ep is None:
        return []
    ep = self.ep
    if not decoupled:
      self.ep, self.t_start = None, None
      ep["dur"] = s.t - ep["t0"]
      ep["gear_mt"] = dict(ep["gear_mt"])
      ep["gate_states"] = list(dict.fromkeys(ep["gate_states"]))
      return [ep] if ep["dur"] >= self.MIN else []
    ep["n"] += 1
    ep["gear_mt"][s.gear_mt] += 1
    ep["clutch_n"] += s.clutch
    ep["long_n"] += s.long_active
    if s.gate:
      ep["gate_states"].append(s.gate)
    return []


class BackstopDetector:
  """NSB overspeed backstop: braking re-allowed above set + margin (cloudlog event, or allowBrake rising)."""
  WINDOW = 3.0

  def __init__(self, ctx: Context):
    self.ctx = ctx
    self.ep: dict | None = None
    self.allow_prev = True

  def feed(self, s: Sample) -> list[dict]:
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


class EbpSuppressDetector:
  """EngineBrakePreferred suppression periods (cloudlog active=True .. active=False)."""

  def __init__(self, ctx: Context):
    self.ep: dict | None = None

  def feed(self, s: Sample) -> list[dict]:
    done = []
    for name, p in s.events:
      if name == "engine_brake_preferred" and p.get("active") and self.ep is None:
        self.ep = {"t0": s.t, "gear": p.get("gear"), "a_eb": p.get("a_eb"), "out_min": s.accel_out, "excess_min": math.inf,
                   "capacity": False, "reason": None}
      elif name == "engine_brake_preferred" and not p.get("active") and self.ep is not None:
        self.ep["reason"] = p.get("reason")
      elif name == "ebp_capacity" and self.ep is not None:
        self.ep["capacity"] = True
    ep = self.ep
    if ep is None:
      return done
    ep["out_min"] = min(ep["out_min"], s.accel_out)
    if math.isfinite(s.excess):
      ep["excess_min"] = min(ep["excess_min"], s.excess)
    if ep["reason"] is not None or not s.long_active:
      ep["dur"] = s.t - ep["t0"]
      ep["capacity"] |= ep["reason"] == "capacity"
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
    self.count = g["count"]
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
class GearMtMapAnalyzer(Analyzer):
  """S1: raw GEAR_MT vs the RPM/speed ratio per gear, what it reads at a stop and in reverse."""

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
    if s.reverse:
      if not self.rev_hist:
        notes.append(f"reverse GEAR_MT={s.gear_mt}")
      self.rev_hist[s.gear_mt] += 1
    elif s.standstill or s.v_ego < 0.1:
      self.stop_hist[s.gear_mt] += dt
    elif s.v_ego > 3. and not s.gas:
      g_inf = infer_gear(s.rpm, s.v_ego)  # deliberately ignores clutchPressed: that IS GEAR_MT == 0
      if 1 <= s.gear_mt <= 6 or g_inf:
        before = self.total[s.gear_mt]
        self.total[s.gear_mt] += dt
        self.agree[s.gear_mt] += dt if g_inf == s.gear_mt else 0.
        self.inferred_when[s.gear_mt][g_inf] += dt
        if before < self.need_s <= self.total[s.gear_mt]:
          notes.append(f"GEAR_MT {s.gear_mt} covered")
    return notes

  def progress(self) -> tuple[str, float]:
    got = [g for g in range(1, 7) if self.total.get(g, 0.) >= self.need_s]
    stop = sum(self.stop_hist.values())
    fracs = [len(got) / 6, min(1., stop / self.stop_need), 1. if self.rev_hist else 0.]
    text = f"gears {len(got)}/6 • stop {stop:.0f}/{self.stop_need:.0f}s • R {'✓' if self.rev_hist else '×'}"
    return text, min(fracs)


class BitHuntAnalyzer(Analyzer):
  """S6: which raw POWERTRAIN_DATA bit toggles with GEAR_MT -> 0 (a clutch switch would lead it)."""
  WINDOW = 0.3

  def __init__(self, spec: TestSpec, ctx: Context):
    super().__init__(spec, ctx)
    self.presses = self.goal["presses"]
    self.prev_raw: int | None = None
    self.prev_mt: int | None = None
    self.toggles: dict[int, list[float]] = defaultdict(list)
    self.edges: list[float] = []      # GEAR_MT 0-edges (in and out)
    self.press_count = 0

  def _feed(self, s: Sample, ok: bool, dt: float) -> list[str]:
    if not ok or s.raw_17c < 0:
      return []
    notes = []
    if self.prev_raw is not None:
      diff = s.raw_17c ^ self.prev_raw
      while diff:
        bit = (diff & -diff).bit_length() - 1
        if len(self.toggles[bit]) < 2000:
          self.toggles[bit].append(s.t)
        diff &= diff - 1
    if self.prev_mt is not None and (s.gear_mt == 0) != (self.prev_mt == 0):
      self.edges.append(s.t)
      if s.gear_mt == 0:
        self.press_count += 1
        notes.append(f"clutch press {self.press_count}")
    self.prev_raw, self.prev_mt = s.raw_17c, s.gear_mt
    return notes

  def progress(self) -> tuple[str, float]:
    return f"clutch presses {self.press_count}/{self.presses}", min(1., self.press_count / self.presses)

  @staticmethod
  def dbc_bit(bit: int) -> int:
    """LSB-first index of the big-endian 64-bit payload -> DBC (Motorola) bit number byte*8 + bit-in-byte."""
    return (7 - bit // 8) * 8 + bit % 8

  def candidates(self) -> list[tuple[int, float, int]]:
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


ANALYZERS = {"segments": SegmentAnalyzer, "episodes": EpisodeAnalyzer, "gear_mt_map": GearMtMapAnalyzer,
             "bit_hunt": BitHuntAnalyzer, "non_adaptive": NonAdaptiveAnalyzer}


def make_analyzer(spec: TestSpec, ctx: Context) -> Analyzer:
  return ANALYZERS[spec.analyzer](spec, ctx)


# --- results: (status, detail). status None = nothing to score yet --------------------------------------------------
def _health(a: Analyzer) -> list[str]:
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
  bad = _health(a)
  secs = a._live("all")[0]
  return (FAIL, "; ".join(bad)) if bad else (PASS, f"{secs:.0f}s engaged: no ACC fault, 0 blocked TX, no fault alerts")


def r_brake_strength(a: SegmentAnalyzer):
  if not a.segments:
    return None, ""
  cmd, acc = a.pooled("accel_cmd"), a.pooled("a_ego")
  coast = a.pooled("a_coast")
  ratio = acc / cmd if cmd < -1e-3 else math.nan
  assist = sum(g.assist_on for g in a.segments) / max(1, sum(g.assist_n for g in a.segments))
  detail = (f"achieves {ratio:.0%} of request (cmd {cmd:.2f}, a_ego {acc:.2f}, coast est {_fmt(coast)}); " +
            f"brake-assist bit {assist:.0%}")
  return (PASS if ratio >= 0.5 else FAIL), detail + ("" if ratio >= 0.5 else " - weak/no braking (PR #2050)")


def r_cmd_semantics(a: SegmentAnalyzer):
  parts = []
  for key, name in (("zero", "cmd 0"), ("small_neg", "small neg")):
    if not a.by_key(key):
      continue
    acc, coast, cmd = a.pooled("a_ego", key), a.pooled("a_coast", key), a.pooled("accel_cmd", key)
    excess = acc - coast
    if key == "zero":
      verdict = "HOLDS speed (accel semantics)" if abs(acc) < 0.15 and excess > 0.15 else \
                "COASTS (closed throttle)" if abs(excess) < 0.15 else "unclear"
    else:
      verdict = "uses SERVICE BRAKES" if excess < -0.15 else "fuel-cut/coast only"
    parts.append(f"{name}: a {acc:.2f} vs coast est {_fmt(coast)} (cmd {cmd:.2f}) -> {verdict}")
  if not parts:
    return None, ""
  return CAPTURED, "; ".join(parts) + " [coast est uncalibrated: compare E0b]"


def r_rpm_noise(a: SegmentAnalyzer):
  if not a.segments:
    return None, ""
  secs = sum(g.dur for g in a.segments)
  flares = sum(g.flares for g in a.segments)
  rate = sum(g.rpm_changes for g in a.segments) / secs if secs else 0.
  detail = (f"{flares} false flares in {secs:.0f}s steady; RPM changes ~{rate:.0f}/s; " +
            f"max rate {_fmt(a.extreme('rpm_rate', max), 0)} rpm/s (threshold {FLARE_RPM_RATE:.0f}). Check tach by eye.")
  return (PASS if flares == 0 else FAIL), detail


def r_gear_mt_map(a: GearMtMapAnalyzer):
  if not (a.total or a.stop_hist or a.rev_hist):
    return None, ""
  bad, parts = [], []
  for g in range(1, 7):
    tot = a.total.get(g, 0.)
    if tot <= 0.:
      continue
    pct = a.agree[g] / tot
    parts.append(f"{g}:{pct:.0%}")
    if tot >= a.need_s and pct < 0.95:
      inferred = max(a.inferred_when[g].items(), key=lambda kv: kv[1])[0]
      bad.append(f"GEAR_MT {g} reads as inferred {inferred}")
  odd = sorted(k for k in a.total if not 1 <= k <= 6)
  if odd:
    parts.append("rolling non-gear values " + ",".join(map(str, odd)))
  rev = sorted(a.rev_hist)
  if rev and rev != [14]:
    bad.append(f"reverse reads {rev}")
  stop = ",".join(f"{k}:{v:.0f}s" for k, v in sorted(a.stop_hist.items()))
  detail = f"agree {' '.join(parts) or '-'}; R={rev or '-'}; at stop {stop or '-'}"
  if bad:
    return FAIL, "; ".join(bad) + " | " + detail
  return (PASS if a.complete else None), detail


def r_bite_lag(a: EpisodeAnalyzer):
  lags = [e["bite_lag"] for e in a.episodes if e["bite_lag"] is not None]
  dec = [e["decouple_lag"] for e in a.episodes if e["decouple_lag"] is not None]
  if not lags:
    return None, ""
  p90 = _pct(lags, 0.9)
  detail = (f"bite lag median {_pct(lags, 0.5):.2f}s p90 {p90:.2f}s (REENGAGE_DELAY {REENGAGE_DELAY}s); " +
            f"GEAR_MT->0 vs decouple median {_fmt(_pct(dec, 0.5))}s (+ = GEAR_MT lags)")
  return (PASS if p90 <= REENGAGE_DELAY else FAIL), detail


def r_lift_to_clutch(a: EpisodeAnalyzer):
  d = [e["gas_release_dt"] for e in a.episodes]
  if not d:
    return None, ""
  p90 = _pct(d, 0.9)
  return (PASS if p90 <= GAS_RELEASE_WINDOW else FAIL), \
         f"lift->clutch median {_pct(d, 0.5):.2f}s p90 {p90:.2f}s (GAS_RELEASE_WINDOW {GAS_RELEASE_WINDOW}s; only matters with the hint on)"


def r_blip_trace(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  flick = sum(e["flickers"] for e in a.episodes)
  leaks = sum(1 for e in a.episodes if "shifting" in e["gate_states"] and
              any(st != "shifting" for st in e["gate_states"][e["gate_states"].index("shifting"):]))
  gate_seen = any(e["gate_states"] for e in a.episodes)
  rates = [e["rpm_rate_max_open"] for e in a.episodes]
  detail = (f"{len(a.episodes)} blips: GEAR_MT flickers {flick}, gate leaks {leaks}" +
            f"{'' if gate_seen else ' (gate off: leak not exercised)'}; blip rpm rate max {max(rates):.0f} rpm/s")
  return (PASS if flick == 0 and leaks == 0 else FAIL), detail


def r_gate_accept(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  bad = [e for e in a.episodes if e["long_active_open"] or not e["enabled_min"] or e["acc_fault"]]
  detail = f"{len(a.episodes) - len(bad)}/{len(a.episodes)} presses paused, stayed engaged, no ACC fault"
  if bad:
    e = bad[0]
    detail += f"; first bad: long_active={e['long_active_open']} engaged={e['enabled_min']} fault={e['acc_fault']}"
  health = _health(a)
  return (FAIL, detail + "; " + "; ".join(health)) if bad or health else (PASS, detail)


def r_gate_shifts(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  throttle = [e for e in a.episodes if e["long_active_open"] or e["accel_out_max_open"] > 0.05]
  jerky = [e for e in a.episodes if e["resume_jerk_max"] > RESUME_JERK_MAX]
  flares = a.detector.false_flares
  aligned = [e["gate_shift_dt"] for e in a.episodes if e["gate_shift_dt"] is not None]
  detail = (f"{len(a.episodes)} shifts: throttle-with-clutch-in {len(throttle)}, jerky resumes {len(jerky)} " +
            f"(max {max(e['resume_jerk_max'] for e in a.episodes):.1f} m/s^3), false flares {flares}; " +
            f"gate 'shifting' after GEAR_MT->0: median {_fmt(_pct(aligned, 0.5))}s")
  return (PASS if not throttle and not jerky and flares == 0 else FAIL), detail


def r_gate_revmatch(a: EpisodeAnalyzer):
  if not a.episodes and not a.extra_episodes:
    return None, ""
  bad = [e for e in a.episodes if not e["enabled_min"] or e["long_active_open"]]
  brakes = [e["disengage_dt"] for e in a.extra_episodes]
  slow = [d for d in brakes if d is None or d > 0.3]
  detail = (f"blips {len(a.episodes) - len(bad)}/{len(a.episodes)} stayed engaged+paused; " +
            f"brake disengage {', '.join(_fmt(d) + 's' for d in brakes) or '-'}")
  return (PASS if not bad and not slow and a.complete else FAIL if bad or slow else None), detail


def r_eb_downshift(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  added = [e for e in a.episodes if e["post_cmd_max"] > 0.1]
  mx = max(e["post_cmd_max"] for e in a.episodes)
  return CAPTURED, f"throttle added after re-engage in {len(added)}/{len(a.episodes)} downshifts (max cmd {_fmt(mx)})"


def r_non_adaptive(a: NonAdaptiveAnalyzer):
  if a.hold_s <= 0. and a.lead_s <= 0.:
    return None, ""
  mean_err = a.err_sum / a.hold_s if a.hold_s else math.nan
  p95 = a.err_p95()
  hold_ok = abs(mean_err) < 0.5 and p95 < 1.5
  lead_ok = a.lead_cmd_min >= -0.1
  detail = (f"hold err mean {_fmt(mean_err)} p95 {_fmt(p95, 1)} m/s; behind slower lead min cmd " +
            f"{_fmt(a.lead_cmd_min if a.lead_s else math.nan)}; FCW x{a.fcw_edges}")
  if not a.complete:
    return (FAIL if (a.hold_s >= a.hold_need and not hold_ok) or (a.lead_s >= a.lead_need and not lead_ok) else None), detail
  return (PASS if hold_ok and lead_ok else FAIL), detail


def r_overset_brake(a: SegmentAnalyzer):
  if not a.segments:
    return None, ""
  assist = sum(g.assist_on for g in a.segments) / max(1, sum(g.assist_n for g in a.segments))
  return CAPTURED, (f"over set by {a.pooled('v_ego') - a.pooled('v_cruise'):.1f} m/s: cmd {a.pooled('accel_cmd'):.2f}, " +
                    f"sent {_fmt(a.pooled('accel_sent'))}, a_ego {a.pooled('a_ego'):.2f}, excess vs coast " +
                    f"{_fmt(a.pooled('excess'))}, brake-assist bit {assist:.0%}")


def r_nsb_floor(a: SegmentAnalyzer):
  parts, ok = [], True
  for key in a.keys:
    if not a.by_key(key):
      continue
    floor = float(key)
    out_min, exc = a.extreme("accel_out", min, key), a.pooled("excess", key)
    good = out_min >= -floor - 0.01 and not (exc < -0.3)
    ok &= good
    parts.append(f"floor {key}: min out {out_min:.2f}, excess {_fmt(exc)} {'✓' if good else '×'}")
  if not parts:
    return None, ""
  return (PASS if ok and a.complete else FAIL if not ok else None), "; ".join(parts)


def r_backstop(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  e = a.episodes[-1]
  at_margin = abs(e["v_over"] - e["margin"]) <= 0.5
  braked = e["a_min"] <= -0.3
  detail = f"triggered at set+{e['v_over']:.1f} m/s (margin {e['margin']:.1f}); min a_ego in 3 s {e['a_min']:.2f}, min cmd {e['cmd_min']:.2f}"
  return (PASS if at_margin and braked else FAIL), detail + ("" if braked else " - backstop too weak (see C1)")


def r_handback_feel(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  bad = [e for e in a.episodes if e["alerts"] != 1 or e["jerk_max"] > HANDBACK_JERK_MAX]
  detail = "; ".join(f"{e['v'] * 2.237:.1f}mph g{e['gear']} jerk {e['jerk_max']:.1f} alerts {e['alerts']}" for e in a.episodes[-3:])
  return (PASS if not bad and a.complete else FAIL if bad else None), f"{len(a.episodes) - len(bad)}/{len(a.episodes)} ok | {detail}"


def r_stop_hold(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  rolled = [e for e in a.episodes if e["rolled"]]
  hills = len(a.keyed("hill"))
  detail = (f"{len(a.episodes)} stops ({hills} hill): rollback {len(rolled)}; brake hold seen " +
            f"{sum(e['brake_hold'] for e in a.episodes)}x; max roll {max(e['roll_v'] for e in a.episodes):.2f} m/s")
  return (FAIL if rolled else PASS if a.complete else None), detail


def r_neutral_coast(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  e = a.episodes[-1]
  hist = ",".join(f"{k}:{v}" for k, v in sorted(e["gear_mt"].items()))
  return CAPTURED, (f"{len(a.episodes)} coasts; last {e['dur']:.0f}s: GEAR_MT {hist}, clutchPressed {e['clutch_n'] / max(1, e['n']):.0%}, " +
                    f"long active {e['long_n'] / max(1, e['n']):.0%}, gate {'/'.join(e['gate_states']) or '-'}")


def r_uphill_handback(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  return CAPTURED, "; ".join(f"pitch {e['pitch']:.3f}: decel after ramp {_fmt(e['a_after_mean'])}, " +
                             f"v {_fmt(e['v_ramp_end'])}->{_fmt(e['v_end'])} m/s" for e in a.episodes)


def r_ratio_check(a: SegmentAnalyzer):
  parts, bad = [], []
  for g in a.keys:
    if not a.by_key(g):
      continue
    r = a.pooled("ratio", g)
    err = r / overall_ratio(g) - 1.
    parts.append(f"{g}:{err:+.1%} (FD~{r / GEAR_RATIOS[g - 1]:.2f})")
    if abs(err) > 0.03:
      bad.append(g)
  if not parts:
    return None, ""
  detail = "ratio vs seed " + " ".join(parts) + " [FD assumes the seed tire radius]"
  return (FAIL if bad else PASS if a.complete else None), detail + (f"; off: gears {bad}" if bad else "")


def r_coast_fit(a: SegmentAnalyzer):
  parts = []
  for g in a.keys:
    if not a.by_key(g):
      continue
    parts.append(f"g{g} a {a.pooled('a_ego', g):.2f} (model {_fmt(a.pooled('a_coast', g))}, pitch {a.pooled('pitch', g):+.3f})")
  return (CAPTURED, "; ".join(parts)) if parts else (None, "")


def r_ebp_trims(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  bad = [e for e in a.episodes if e["capacity"] or e["out_min"] < a.ctx.ebp_coast_target - 0.01 or e["excess_min"] < -0.3]
  detail = (f"{len(a.episodes) - len(bad)}/{len(a.episodes)} clean; capacity releases {sum(e['capacity'] for e in a.episodes)}; " +
            f"reasons {','.join(str(e['reason']) for e in a.episodes[-3:])}")
  return (FAIL if bad else PASS if a.complete else None), detail


def r_ebp_handoff(a: EpisodeAnalyzer):
  on, off = a.keyed("ebp_on"), a.keyed("ebp_off")
  if not on and not off:
    return None, ""
  gap_on = min((e["gap_min"] for e in on), default=math.nan)
  gap_off = min((e["gap_min"] for e in off), default=math.nan)
  fcw = any(e["fcw"] for e in a.episodes)
  detail = f"min gap EBP on {_fmt(gap_on, 1)} m vs off {_fmt(gap_off, 1)} m; FCW {'yes' if fcw else 'no'}"
  if not (on and off):
    return (FAIL if fcw else None), detail
  return (PASS if gap_on >= gap_off - 2. and not fcw else FAIL), detail


def r_ebp_downhill(a: SegmentAnalyzer):
  if not a.segments:
    return None, ""
  over = max((g.maxs.get("v_ego", 0.) - g.mean("v_cruise")) for g in a.segments)
  detail = f"max over set {over:.1f} m/s; a_ego {a.pooled('a_ego'):.2f}, pitch {a.pooled('pitch'):+.3f}"
  return (FAIL if over > 3.6 else PASS if a.complete else None), detail


def r_jerk_by_gear(a: SegmentAnalyzer):
  parts = []
  for g in a.keys:
    segs = a.by_key(g)
    if segs:
      n = sum(s.n for s in segs)
      parts.append(f"g{g} jerk mean {sum(s.jerk_sum for s in segs) / max(1, n):.2f} max {max(s.jerk_max for s in segs):.2f}")
  return (CAPTURED, "; ".join(parts)) if parts else (None, "")


def r_accel_by_gear(a: SegmentAnalyzer):
  parts = [f"g{g} a {a.pooled('a_ego', g):.2f} @pitch {a.pooled('pitch', g):+.3f} rpm {a.pooled('rpm', g):.0f}"
           for g in a.keys if a.by_key(g)]
  return (CAPTURED, "; ".join(parts)) if parts else (None, "")


def r_bit_hunt(a: BitHuntAnalyzer):
  if not a.edges:
    return None, ""
  c = a.candidates()
  if not c:
    return CAPTURED, f"no bit tracks GEAR_MT->0 across {len(a.edges)} edges"
  return CAPTURED, "candidates (DBC bit, hit rate, toggles): " + ", ".join(f"{b} {h:.0%} {n}" for b, h, n in c[:5])


RESULTS = {
  "baseline": r_baseline, "brake_strength": r_brake_strength, "cmd_semantics": r_cmd_semantics, "rpm_noise": r_rpm_noise,
  "gear_mt_map": r_gear_mt_map, "bite_lag": r_bite_lag, "lift_to_clutch": r_lift_to_clutch, "blip_trace": r_blip_trace,
  "gate_accept": r_gate_accept, "gate_shifts": r_gate_shifts, "gate_revmatch": r_gate_revmatch, "eb_downshift": r_eb_downshift,
  "non_adaptive": r_non_adaptive, "overset_brake": r_overset_brake, "nsb_floor": r_nsb_floor, "backstop": r_backstop,
  "handback_feel": r_handback_feel, "stop_hold": r_stop_hold, "neutral_coast": r_neutral_coast,
  "uphill_handback": r_uphill_handback, "ratio_check": r_ratio_check, "coast_fit": r_coast_fit, "ebp_trims": r_ebp_trims,
  "ebp_handoff": r_ebp_handoff, "ebp_downhill": r_ebp_downhill, "jerk_by_gear": r_jerk_by_gear,
  "accel_by_gear": r_accel_by_gear, "bit_hunt": r_bit_hunt,
}
