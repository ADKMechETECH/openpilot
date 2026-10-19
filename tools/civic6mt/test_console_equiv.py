#!/usr/bin/env python3
"""Task 10 behavior-preservation harness for the dev-test test console (core + per-vehicle profile refactor).

Drives the console exactly as test_consoled does (SampleBuilder -> StartButton -> Console) over fixed inputs, once per
registry test (armed, background telemetry on), and writes every observable output — markers, results.json, capture
files, status snapshots, a digest of every Sample — normalized (wall clock and file timestamps stripped, field names in
the pre-refactor spelling) to <outdir>/<run>.json. Run it before and after the refactor on the same commit-independent
inputs; `diff -r before after` must be empty. (Written for the Task 10 refactor. Task 12 changes console behavior on
purpose - published gate state, verdict rules - so a before/after across Task 12 is expected to differ.)

Task 13 B4: it is also a proof on its own - it exits 1 unless every armed test's capture actually started (a "start"
marker) and wrote a non-empty capture file. The gap button resolves to its recommended TRIPLE press (Task 12 D-10), so
the scripted presses are triples (single presses never started a capture after Task 12). The frames carry the features'
PUBLISHED state as Task 12 runs them (carControlSP.manualLongitudinal / longitudinalPlanSP, built per run from that run's
toggles): longitudinal stays active through an engaged shift and the gate publishes 'shifting' + paused.

Inputs (deterministic): the HONDA CI segment's rlog, re-labelled a 6-speed manual (RPM from a speed->gear schedule, a
0.6 s clutch window with the raw gear reading 0 at each gear change, the raw gear signal otherwise), synthetic feature
cloudlog records (clutch gate states, handback, NSB backstop + Task 11 pulse phases, EBP), the escalation alert, and
gap-button presses (start at +2 s, stop at +20 s, resume at +25 s); then a scripted 6MT drive (Synth: reverse, the
clutch-bit signature parked, a pull-away through the gears, engaged cruise, braking requests, a downhill clamp + pulse +
escalation, engaged plain and rev-matched downshifts, a lead approach, a brake-press disengage, the low-speed handback,
a stop and hold, a neutral coast, an uphill pull, a downhill coast, EBP records). The capture window stays open across both.

  cd ~/civic-cruise/openpilot && source .venv/bin/activate && export PYTHONPATH="$(pwd)"   # civic-6mt-dev-test
  python tools/civic6mt/test_console_equiv.py <outdir>
"""
import hashlib
import json
import os
import sys
import tempfile

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.selfdrive.test.process_replay.test_processes import get_log_data, segments
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import engine_rpm_at
from openpilot.tools.lib.logreader import LogReader

ButtonType = structs.CarState.ButtonEvent.Type
OLD_NAMES = {"gear_raw": "gear_mt", "raw_payload": "raw_17c", "gas_raw": "pedal_gas"}   # post-refactor role -> old field
SCHEDULE = [(4.5, 1), (8., 2), (12., 3), (17., 4), (23., 5)]
CLUTCH_S = 0.6
# start / stop / resume, each a TRIPLE press (the gap button's default gesture since Task 12 D-10)
BUTTON = {round(at + dt, 2): pressed for at in (2.0, 20.0, 25.0)
          for dt, pressed in ((0., True), (0.1, False), (0.3, True), (0.4, False), (0.6, True), (0.7, False))}


def api():
  """The console's API, before (flat modules) or after (core + profiles) the refactor."""
  from openpilot.sunnypilot.selfdrive.test_console import capture, signals
  from openpilot.sunnypilot.selfdrive.test_console.button import StartButton
  try:
    from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE
  except ImportError:
    PROFILE = None
  if PROFILE is None:
    from openpilot.sunnypilot.selfdrive.test_console import registry
    from openpilot.sunnypilot.selfdrive.test_console.conditions import Context

    class Old:
      tests = registry.TESTS

      @staticmethod
      def ctx(toggles, nsb_mode):
        return Context(manual=True, op_long=True, toggles=toggles, nsb_mode=nsb_mode)

      @staticmethod
      def console(store, ctx):
        return capture.Console(store, ctx)

      @staticmethod
      def builder():
        return signals.SampleBuilder()

      @staticmethod
      def build(b, *a):
        return b.build(*a)

      button = StartButton
      log_tap = signals.LogEventTap
      store = capture.Store
      to_old = staticmethod(lambda name: name)
    return Old
  from openpilot.sunnypilot.selfdrive.test_console.conditions import Context

  class New:
    tests = PROFILE.registry.tests

    @staticmethod
    def ctx(toggles, nsb_mode):
      return Context(profile=PROFILE, manual=True, op_long=True, toggles=toggles, nsb_mode=nsb_mode)

    @staticmethod
    def console(store, ctx):
      return capture.Console(store, ctx)

    @staticmethod
    def builder():
      return signals.SampleBuilder(PROFILE)

    @staticmethod
    def build(b, *a):
      return b.build(*a)

    @staticmethod
    def button(*a):
      return StartButton(*a, buttons=PROFILE.buttons)

    log_tap = signals.LogEventTap
    store = capture.Store
    to_old = staticmethod(lambda name: OLD_NAMES.get(name, name))
  return New


def log_record(name: str, **payload) -> str:
  return json.dumps({"msg": {"event": name, **payload}, "level": "EVENT"})


def scripted_events(t: float) -> list[str]:
  """Feature cloudlog records at fixed offsets from the segment start (seconds)."""
  out = {
    3.0: [log_record("engine_brake_preferred", active=True, gear=4, a_eb=-0.3)],
    6.0: [log_record("engine_brake_preferred", active=False, reason="capacity"), "engine_brake_preferred: car not slowing"],
    9.0: [log_record("no_service_brake_backstop", v_ego=17.0, v_set=13.4)],
    14.0: [log_record("no_service_brake_pulse", phase="pulsing", prev="coasting", overspeed=3.62, v_set=13.4)],
    18.0: [log_record("no_service_brake_pulse", phase="coasting", prev="pulsing", overspeed=0.88, v_set=13.4)],
    26.0: [log_record("no_service_brake_pulse", phase="pulsing", prev="coasting", overspeed=3.65, v_set=13.4)],
    28.0: [log_record("no_service_brake_pulse", phase="escalated", prev="pulsing", overspeed=6.74, v_set=13.4)],
    33.0: [log_record("no_service_brake_pulse", phase="coasting", prev="escalated", overspeed=0.85, v_set=13.4)],
    36.0: [log_record("manual_low_speed_handback", process="controlsd", v_ego=2.2, accel=0.3, ramp=0.5)],
    40.0: [log_record("clutch_shift_gate", state="shifting"), log_record("engine_brake_coast", n=12)],
    41.0: [log_record("clutch_shift_gate", state="idle")],
  }
  return [rec for at, recs in out.items() if abs(t - at) < 0.005 for rec in recs]


def mutable(x):
  try:
    return x.as_builder()
  except AttributeError:   # still the placeholder builder: nothing received yet
    return x


def strip(d: dict, *keys) -> dict:
  return {k: v for k, v in d.items() if k not in keys}


class Frame:
  __slots__ = ("t", "cs", "cc", "co", "plan", "plan_valid", "radar", "ss", "tx", "records", "raw_gear", "payload", "buttons",
               "gate", "paused", "handback", "ebp")


def real_frames(msgs) -> list[Frame]:
  """The HONDA CI segment, re-labelled a manual (see the module docstring)."""
  latest = {w: messaging.new_message(w) for w in ("carControl", "carOutput", "longitudinalPlan", "radarState", "selfdriveState")}
  out, t0, prev_gear, clutch_until = [], None, 1, -1.
  for m in msgs:
    w = m.which()
    if w in latest:
      latest[w] = m
    if w != "carState":
      continue
    f = Frame()
    f.t = m.logMonoTime / 1e9
    t0 = f.t if t0 is None else t0
    rel = round(f.t - t0, 2)
    cs = m.carState.as_builder()
    gear = next((g for v_up, g in SCHEDULE if cs.vEgo < v_up), 6)
    if gear != prev_gear:
      clutch_until, prev_gear = f.t + CLUTCH_S, gear
    clutch = f.t < clutch_until
    cs.deprecated.clutchPressed = clutch
    cs.deprecated.engineRpm = float(900. if clutch else engine_rpm_at(gear, cs.vEgo))
    f.buttons = [structs.CarState.ButtonEvent(pressed=p, type=ButtonType.gapAdjustCruise) for at, p in BUTTON.items() if abs(rel - at) < 0.005]
    cs.buttonEvents = f.buttons
    f.cs = cs
    f.cc = mutable(latest["carControl"].carControl)
    f.cc.orientationNED = [0., -0.03 if 10. < rel < 35. else 0.005, 0.]
    f.ss = mutable(latest["selfdriveState"].selfdriveState)
    if 28.5 < rel < 29.5:
      f.ss.alertType = "noServiceBrakeOverspeed/warning"
    f.co, f.plan = latest["carOutput"].carOutput, latest["longitudinalPlan"].longitudinalPlan
    f.plan_valid, f.radar = latest["longitudinalPlan"].valid, latest["radarState"].radarState
    f.tx = len(out) // 3000
    f.records = scripted_events(rel)
    f.raw_gear = -1 if rel < 1. else (0 if clutch else gear)   # the raw gear signal (tap) once "live"
    f.payload = -1
    # the published gate state (Task 12 A2): paused while the clutch is in with openpilot engaged, longActive untouched
    f.paused = clutch and bool(f.cc.enabled)
    f.gate = "shifting" if f.paused else "idle"
    f.handback, f.ebp = 36. <= rel < 36.6, 3. <= rel < 6.
    out.append(f)
  return out


class Synth:
  """A scripted 6MT drive at 100 Hz (no physics beyond v += a dt; rpm follows the gear unless the clutch is in)."""
  DT = 0.01
  SIG_BIT = 37

  def __init__(self, t0: float):
    self.t, self.frames = t0, []
    # (Task 13 B4: a lint fix had moved most of these keys into the `noqa` comment - the drive raised KeyError)
    self.st = dict(v=0., a=0., gear=0, clutch=False, gas=False, brake=False, rpm=800., coupled=False,  # noqa: C408
                   enabled=False, long=False, cmd=0., a_target=0., v_cruise=0., pitch=0.005, allow=True, fcw=False,
                   lead=None, alert="", raw_gear=0, payload=0, hold=False, reverse=False, cruise=False, gate="idle",
                   paused=False, handback=False, ebp=False)

  def step(self, seconds: float, records=(), buttons=(), **kw) -> None:
    st = self.st
    st.update(kw)
    for i in range(max(1, int(round(seconds / self.DT)))):
      self.t += self.DT
      st["v"] = max(st["v"] + st["a"] * self.DT, 0.)
      if st["lead"] is not None:
        d, vrel = st["lead"]
        st["lead"] = (max(d + vrel * self.DT, 1.), vrel)
      if st["coupled"] and 1 <= st["gear"] <= 6 and not st["clutch"]:
        st["rpm"] = float(engine_rpm_at(st["gear"], st["v"]))
      st["payload"] ^= 1 << 2   # a counter bit chattering every frame (the bit hunt must ignore it)
      f = Frame()
      f.t = self.t
      cs = messaging.new_message("carState").carState
      cs.vEgo, cs.aEgo, cs.gasPressed, cs.brakePressed = st["v"], st["a"], st["gas"], st["brake"]
      cs.deprecated.clutchPressed, cs.deprecated.engineRpm = st["clutch"], st["rpm"]
      cs.vCruise = st["v_cruise"] * 3.6 if st["v_cruise"] else 255.
      cs.standstill, cs.brakeHoldActive, cs.cruiseState.enabled = st["v"] < 0.1, st["hold"], st["cruise"]
      cs.gearShifter = "reverse" if st["reverse"] else "drive"
      f.buttons = list(buttons) if i == 0 else []
      cs.buttonEvents = f.buttons
      cc = messaging.new_message("carControl").carControl
      cc.enabled, cc.longActive, cc.actuators.accel = st["enabled"], st["long"], st["cmd"]
      cc.orientationNED = [0., st["pitch"], 0.]
      co = messaging.new_message("carOutput").carOutput
      co.actuatorsOutput.accel = st["cmd"]
      plan = messaging.new_message("longitudinalPlan").longitudinalPlan
      plan.aTarget, plan.allowBrake, plan.fcw = st["a_target"], st["allow"], st["fcw"]
      radar = messaging.new_message("radarState").radarState
      if st["lead"] is not None:
        radar.leadOne.present, radar.leadOne.dRel, radar.leadOne.vRel = True, st["lead"][0], st["lead"][1]
      ss = messaging.new_message("selfdriveState").selfdriveState
      ss.alertType = st["alert"]
      f.cs, f.cc, f.co, f.plan, f.plan_valid, f.radar, f.ss, f.tx = cs, cc, co, plan, True, radar, ss, 0
      f.records = list(records) if i == 0 else []
      f.raw_gear, f.payload = st["raw_gear"], st["payload"]
      f.gate, f.paused, f.handback, f.ebp = st["gate"], st["paused"], st["handback"], st["ebp"]
      self.frames.append(f)

  def toggle_sig(self, n: int, gap: float) -> None:
    for _ in range(n):
      self.step(0.1, payload=self.st["payload"] ^ (1 << self.SIG_BIT))
      self.step(gap / 2, payload=self.st["payload"] ^ (1 << self.SIG_BIT))
      self.step(gap / 2)

  def shift(self, to: int, blip: bool = False, engaged: bool = False) -> None:
    """Task 12 A2 semantics: an engaged shift keeps longitudinal ACTIVE; the gate publishes 'shifting' + paused (throttle
    capped at the coast accel, braking available), then 'reengaging', then 'idle'. A blip: exactly 0 commanded."""
    gate = [log_record("clutch_shift_gate", state="shifting")] if engaged else []
    pub = {"gate": "shifting", "paused": True} if engaged else {}
    self.step(0.25, gas=False, a=-0.2, cmd=0. if engaged else self.st["cmd"])
    self.step(0.15, records=gate, clutch=True, raw_gear=0, coupled=False, rpm=self.st["rpm"] * 0.9, **pub)
    self.step(0.2, gas=blip, rpm=self.st["rpm"] * (1.4 if blip else 0.9), **({"cmd": 0.} if engaged and blip else {}))
    self.step(0.25, gas=False)
    self.step(0.2, records=[log_record("clutch_shift_gate", state="reengaging")] if engaged else [], clutch=False, raw_gear=to,
              gear=to, rpm=float(engine_rpm_at(to, self.st["v"]) * 1.15), **({"gate": "reengaging"} if engaged else {}))
    self.step(0.6, records=[log_record("clutch_shift_gate", state="idle")] if engaged else [], coupled=True,
              **({"gate": "idle", "paused": False} if engaged else {}))


def synth_frames(t0: float) -> list[Frame]:
  d = Synth(t0)
  # parked in neutral: reverse, the clutch-bit signature (N = 3) on one raw payload bit
  d.step(1., reverse=True, raw_gear=14)
  d.step(5., reverse=False, raw_gear=0)
  for gap in (0.5, 2.0, 0.5):
    d.toggle_sig(3, gap)
    d.step(5.)
  # pull away through the gears (driver's own throttle), then engage
  d.step(1., gear=1, raw_gear=1, coupled=True, gas=True, a=1.5)
  for to, v_up in ((2, 4.5), (3, 8.), (4, 12.), (5, 17.), (6, 23.)):
    while d.st["v"] < v_up:
      d.step(0.1, gas=True, a=1.5)
    d.shift(to)
  d.step(2., gas=True, a=1.0)
  d.step(1., gas=False, a=0., enabled=True, long=True, cruise=True, v_cruise=25.)
  d.step(20., pitch=0.005, cmd=0.02, a_target=0.)
  for _ in range(2):   # set speed dropped: a braking request
    d.step(3., cmd=-0.8, a_target=-0.8, a=-0.7)
    d.step(3., cmd=0.1, a_target=0.1, a=0.)
  d.step(10., cmd=-0.3, a_target=-0.3, a=-0.05)   # small negative
  # downhill over set: no-brake clamp, overspeed pulse + escalation
  d.step(22., pitch=-0.04, a=0.45, cmd=-0.3, a_target=-1.2, allow=False)
  d.step(4., records=[log_record("no_service_brake_pulse", phase="pulsing", prev="coasting", overspeed=3.6, v_set=25.)], allow=True,
         cmd=-1.0, a=-0.6)
  d.step(3., records=[log_record("no_service_brake_pulse", phase="coasting", prev="pulsing", overspeed=0.9, v_set=25.)], allow=False,
         cmd=-0.3, a=0.3)
  d.step(1., records=[log_record("no_service_brake_backstop", v_ego=d.st["v"], v_set=25.)], allow=True, cmd=-1.2, a=-0.5)
  d.step(3., pitch=0.005, allow=True, cmd=0., a=-0.3, a_target=0.)
  # engaged downshifts: one plain into engine braking, one rev-matched
  d.shift(5, engaged=True)
  d.step(3., a=-0.4, cmd=0.)
  d.shift(4, blip=True, engaged=True)
  d.step(3., a=-0.3, cmd=0.2)
  # lead approach, then a brake press disengages
  d.step(8., lead=(60., -3.), a=0., cmd=-0.2)
  d.step(2., lead=(35., 0.5))
  d.step(0.05, lead=None, brake=True)
  d.step(1., enabled=False, long=False, cruise=False, cmd=0.)
  d.step(1., brake=False)
  # re-engage, slow in 3rd/2nd to the low-speed handback, stop, hold, drive off
  d.step(1., enabled=True, long=True, cruise=True)
  d.shift(3, engaged=True)
  while d.st["v"] > 5.:
    d.step(0.1, a=-1.0, cmd=-1.0)
  d.shift(2, engaged=True)
  while d.st["v"] > 2.3:
    d.step(0.1, a=-0.6, cmd=-0.6)
  d.step(0.5, records=[log_record("manual_low_speed_handback", process="controlsd", v_ego=2.2, accel=-0.6, ramp=0.5)],
         alert="manualLowSpeedHandback/immediateDisable", cmd=-0.3, handback=True)
  d.step(1., enabled=False, long=False, alert="", cmd=0., handback=False)
  d.step(2., clutch=True, raw_gear=0, coupled=False, rpm=800., brake=True, a=-1.1)
  d.step(2., a=0., v=0., hold=True)
  d.step(3., brake=False)
  d.step(1., hold=False, clutch=False, raw_gear=1, gear=1, coupled=True, gas=True, a=1.2)
  # neutral coast while engaged, then uphill acceleration in 3rd
  while d.st["v"] < 12.:
    d.step(0.1, a=1.5)
  d.step(1., gas=False, a=-0.1, enabled=True, long=True, cruise=True, raw_gear=0, coupled=False, rpm=850.)
  d.step(0.5, gate="shifting", paused=True)
  d.step(8., records=[log_record("clutch_shift_gate", state="coastTimeout")], gate="coastTimeout", paused=False)
  d.step(1., gear=3, raw_gear=3, coupled=True, long=True, gate="idle")
  d.step(6., pitch=0.035, gas=True, a=1.0, enabled=False, long=False)
  d.step(8., pitch=-0.04, gas=False, a=0.15)   # downhill coast in 3rd, openpilot off (E0a)
  d.step(2., records=[log_record("engine_brake_preferred", active=True, gear=3, a_eb=-0.4)], gas=False, a=-0.4, pitch=0.005,
         enabled=True, long=True, cmd=0., ebp=True)
  d.step(2., records=[log_record("engine_brake_preferred", active=False, reason="plan")], ebp=False)
  d.step(2., enabled=False, long=False, cruise=False)
  return d.frames


class Published:
  """carControlSP / longitudinalPlanSP as controlsd / plannerd publish them for this run's latched toggles."""

  def __init__(self, toggles: dict, nsb_mode: str):
    from openpilot.cereal import custom
    self.custom, self.t, self.mode = custom, toggles, nsb_mode
    self._plan_sp = self._make_plan_sp()

  def _make_plan_sp(self):
    plan_sp = self.custom.LongitudinalPlanSP.new_message()
    plan_sp.features.nonAdaptive, plan_sp.features.gearAware = self.t["NonAdaptiveLongitudinal"], self.t["GearAwareAccel"]
    nsb = plan_sp.noServiceBrake
    nsb.enabled = self.t["NoServiceBrakeHold"] and self.t["NonAdaptiveLongitudinal"]
    nsb.backstop, nsb.mode, nsb.margin, nsb.decelFloor = self.t["NoServiceBrakeOverspeedBackstop"], self.mode, 3.6, 0.3
    return plan_sp.as_reader()

  def build(self, f: Frame):
    cc_sp = self.custom.CarControlSP.new_message()
    ml = cc_sp.manualLongitudinal
    gate_on = self.t["ClutchShiftGate"] and not self.t["DisengageOnAccelerator"]
    ml.gateEnabled, ml.gateBlocked = gate_on, self.t["ClutchShiftGate"] and not gate_on
    ml.gateState = f.gate if gate_on else "idle"
    ml.gatePaused = bool(f.paused) and gate_on
    ml.ebpEnabled = self.t["EngineBrakePreferred"]
    ml.ebpSuppressing = bool(f.ebp) and ml.ebpEnabled
    ml.handbackAvailable, ml.handbackActive = True, bool(f.handback)
    return cc_sp.as_reader(), self._plan_sp


def run(A, frames: list[Frame], test_id: str, nsb_mode: str) -> dict:
  tmp = tempfile.mkdtemp(prefix="tc_equiv_")
  spec = next(t for t in A.tests if t.id == test_id) if test_id else None
  toggles = {**dict.fromkeys(("NonAdaptiveLongitudinal", "ClutchShiftGate", "ClutchShiftGateGasReleaseHint", "DisengageOnAccelerator",
                              "NoServiceBrakeHold", "NoServiceBrakeOverspeedBackstop", "EngineBrakePreferred", "GearAwareAccel"), False),
             **(spec.toggles if spec else {})}
  ctx = A.ctx(toggles, nsb_mode)
  store = A.store(tmp)
  console = A.console(store, ctx)
  console.set_control(test_id, bool(test_id), True)
  builder, log_tap, button = A.builder(), A.log_tap(), A.button()
  digest, statuses = hashlib.sha256(), []
  published = Published(toggles, nsb_mode)
  for frame, f in enumerate(frames):
    log_events = log_tap.update(f.records)
    cc_sp, plan_sp = published.build(f)
    s = A.build(builder, f.t, f.cs, f.cc, f.co, f.plan, f.plan_valid, f.radar, f.ss, f.tx, None, log_tap, log_events,
                cc_sp, True, plan_sp, True)
    s_names = set(s.__slots__)
    setattr(s, "gear_raw" if "gear_raw" in s_names else "gear_mt", f.raw_gear)
    setattr(s, "raw_payload" if "raw_payload" in s_names else "raw_17c", f.payload)
    console.update(s, button.update(f.t, f.buttons))
    row = {A.to_old(k): getattr(s, k) for k in s.__slots__}
    digest.update(json.dumps(row, sort_keys=True, default=str).encode())
    if frame % 500 == 0:
      statuses.append(strip(console.status(s, {"name": button.name, "mode": button.mode, "seen": button.seen}), "stamp"))
  console.shutdown()
  with open(os.path.join(tmp, "markers.jsonl")) as f:
    markers = [strip(json.loads(line), "wall") for line in f]
  try:
    results = {k: strip(v, "wall") for k, v in json.load(open(os.path.join(tmp, "results.json"))).items()}
  except FileNotFoundError:
    results = {}
  caps = {}
  for name in sorted(os.listdir(os.path.join(tmp, "captures"))):
    with open(os.path.join(tmp, "captures", name)) as f:
      lines = [json.loads(line) for line in f]
    caps[name.split("_", 1)[1]] = [strip(lines[0], "wall", "mono")] + lines[1:]
  return {"frames": len(frames), "samples": digest.hexdigest(), "markers": markers, "results": results, "captures": caps,
          "statuses": statuses}


def main() -> None:
  outdir = sys.argv[1]
  os.makedirs(outdir, exist_ok=True)
  A = api()
  _, dat = get_log_data(dict(segments)["HONDA"])
  frames = real_frames(list(LogReader.from_bytes(dat)))
  frames += synth_frames(frames[-1].t + 10.)
  runs = [("", "maintain")] + [(t.id, "maintain") for t in A.tests] + [("NSB2", "pulse")]
  no_capture = []
  for test_id, mode in runs:
    out = run(A, frames, test_id, mode)
    name = f"{test_id or 'background'}{'@pulse' if mode == 'pulse' else ''}"
    with open(os.path.join(outdir, f"{name}.json"), "w") as f:
      json.dump(out, f, sort_keys=True, indent=1, default=str)
    res = out["results"].get(test_id, {})
    # Task 13 B4: an armed test must really capture - a start marker and a capture file with rows
    started = any(m.get("action") == "start" and m.get("test") == test_id and not m.get("background") for m in out["markers"])
    rows = sum(len(v) - 1 for k, v in out["captures"].items() if k.startswith(test_id + "_") and "_bg" not in k) if test_id else 0
    if test_id and not (started and rows > 0):
      no_capture.append(name)
    print(f"{name:12s} {out['frames']} frames, {len(out['markers'])} markers, {len(out['captures'])} captures, " +
          f"{'started' if started else 'NOT started'}, {rows} rows, {res.get('status')} {res.get('progress', '')}")
  print("OK: every armed test captured" if not no_capture else f"FAILED: no capture for {no_capture}")
  sys.exit(1 if no_capture else 0)


if __name__ == "__main__":
  main()
