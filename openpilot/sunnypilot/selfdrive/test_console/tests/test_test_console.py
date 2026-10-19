"""
On-car test console (dev-test build): registry integrity, conditions, the button, the cloudlog tap, every analyzer
family on synthetic maneuvers (fires only when it should, counts progress/goal, PASS/FAIL), the capture state machine
and its files, default-inertness, and the structural "zero driving authority" checks.
"""
import ast
import json
import logging
import math
import os
import pathlib
import tempfile

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.common.logging_extra import SwagFormatter
from openpilot.common.swaglog import cloudlog
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import engine_rpm_at
from openpilot.sunnypilot.selfdrive.test_console import analyzers as A
from openpilot.sunnypilot.selfdrive.test_console.analyzers import CAPTURED, FAIL, PASS, make_analyzer
from openpilot.sunnypilot.selfdrive.test_console.button import StartButton as _StartButton
from openpilot.sunnypilot.selfdrive.test_console.capture import Console, Store
from openpilot.sunnypilot.selfdrive.test_console.conditions import BINS, Context, check_conditions
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE as CIVIC, WATCHED_TOGGLES
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.registry import PHASES, TESTS, TESTS_BY_ID
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.tap import RawCanTap
from openpilot.sunnypilot.selfdrive.test_console.registry import Kind
from openpilot.sunnypilot.selfdrive.test_console.signals import SAMPLE_FIELDS, LogEventTap, Sample
from openpilot.sunnypilot.selfdrive.test_console.signals import SampleBuilder as _SampleBuilder
from openpilot.sunnypilot.selfdrive.test_console.status_io import MARKERS_FILE, RESULTS_FILE, read_json
from openpilot.sunnypilot.selfdrive.test_console.tests.sim import DT, Sim, feed

ButtonType = structs.CarState.ButtonEvent.Type
PKG = pathlib.Path(__file__).resolve().parents[1]
# the console's code: the core and every vehicle profile (Task 10), not the tests
CONSOLE_SRCS = sorted(p for p in PKG.rglob("*.py") if "tests" not in p.relative_to(PKG).parts)

ALL_ON = dict.fromkeys(WATCHED_TOGGLES, False)


def ctx(**toggles) -> Context:
  return Context(profile=CIVIC, manual=True, op_long=True, toggles={**ALL_ON, **toggles})


def calibrated(c: Context, bias: float = 0.) -> Context:
  """The coast model FIT (Task 13 B1): E0b complete with a per-gear correction saved, so the rules that judge braking
  against the model may PASS. bias = the fit's correction (a_ego - seed) for every gear."""
  c.results = {"E0b": {"complete": True, "status": "CAPTURED",
                       "coast_fit": {"version": 1, "gears": {str(g): {"bias": bias, "n": 1000} for g in range(1, 7)}}}}
  return c


# Task 10: the core runs the Civic through its vehicle profile
CONDITIONS = ctx().conditions
RESULTS = A.results_for(ctx())


def StartButton(*a, **kw):
  return _StartButton(*a, buttons=CIVIC.buttons, **kw)


def SampleBuilder():
  return _SampleBuilder(CIVIC)


def btn(pressed: bool, t=ButtonType.gapAdjustCruise):
  return [structs.CarState.ButtonEvent(pressed=pressed, type=t)]


def shift(sim: Sim, g_from: int, g_to: int, open_s=0.6, bite_lag=0.2, blip=False, lift_dt=0.25, flicker=False, enabled=None,
          long_active=None, post_gas=False, post=3.5, gate=False, resume_after: float | None = 0.5):
  """A clutch shift: gas in g_from, lift, clutch in (GEAR_MT 0), clutch out (GEAR_MT g_to), clutch bites bite_lag later.
  gate=True: controlsd's published gate state as Task 12 runs it - longitudinal stays active, the gate is paused
  ('shifting') with the clutch in and for resume_after s after it closes (None = it never resumes)."""
  kw = {} if enabled is None else {"enabled": enabled}
  out = list(sim.run(1.0, gas=True, gear_raw=g_from, clutch=False, **kw))
  out += sim.run(lift_dt, gas=False)
  sim.locked = False
  sim.s.rpm = engine_rpm_at(g_from, sim.s.v_ego)
  la = {} if long_active is None else {"long_active": long_active}
  for i in range(int(open_s / DT)):
    frac = i / (open_s / DT)
    blipping = blip and 0.3 < frac < 0.6
    rpm = sim.s.rpm + (60. if blipping else -15.)
    mt = 0
    clutch = True
    if flicker and 0.45 < frac < 0.47:
      mt, clutch = g_to, False
    g = {"gate_paused": True, "gate": "shifting"} if gate else {}
    out.append(sim.step(clutch=clutch, gear_raw=mt, gas=blipping, rpm=rpm, **la, **g))
  # clutch out: GEAR_MT reports the new gear immediately, rpm is still 15% off until the clutch bites
  target = engine_rpm_at(g_to, sim.s.v_ego)
  t_closed = 0.

  def gate_after():
    if not gate:
      return {}
    paused = resume_after is None or t_closed < resume_after
    return {"gate_paused": paused, "gate": "reengaging" if paused else "idle"}
  for _ in range(int(round(bite_lag / DT))):
    out.append(sim.step(clutch=False, gear_raw=g_to, rpm=target * 1.15, gas=post_gas, **la, **gate_after()))
    t_closed += DT
  sim.locked = True
  after = {"long_active": True} if long_active is not None else {}
  for _ in range(int(round(post / DT))):
    out.append(sim.step(clutch=False, gear_raw=g_to, gas=post_gas, **after, **gate_after()))
    t_closed += DT
  return out


class TestRegistry(OpenpilotTestCase):
  def test_rows_resolve(self):
    for t in TESTS:
      with self.subTest(t.id):
        self.assertIn(t.phase, PHASES)
        for c in t.conditions:
          self.assertIn(c, CONDITIONS)
        self.assertIn(t.analyzer, A.ANALYZERS)
        self.assertIn(t.result, RESULTS)
        for f in t.fields:
          self.assertIn(f, SAMPLE_FIELDS)
        for k in t.toggles:
          self.assertIn(k, WATCHED_TOGGLES)
        for p in t.prereqs:
          self.assertIn(p, TESTS_BY_ID)
          pre = TESTS_BY_ID[p]
          # a later-phase prereq is only allowed for the first-run calibration captures (collected up front)
          self.assertTrue(pre.phase <= t.phase or (pre.first_run and pre.kind == Kind.TELEMETRY), f"{t.id} <- {p}")
        if t.analyzer == "segments" and "bin" in t.goal:
          self.assertIn(t.goal["bin"], BINS)
        if t.analyzer == "episodes":
          self.assertIn(t.goal["detector"], A.DETECTORS)
          if "filter" in t.goal:
            self.assertIn(t.goal["filter"], A.FILTERS)
          if "need" in t.goal:
            self.assertIn(t.goal["detector"], A.EPISODE_KEYS)
        make_analyzer(t, ctx())  # constructs
        self.assertLessEqual(len(t.label), 24, "label must fit a mici button")

  def test_ordered_and_first_run(self):
    phases = [t.phase for t in TESTS]
    self.assertEqual(phases, sorted(phases), "registry is the recommended order")
    first = {t.id for t in TESTS if t.first_run}
    self.assertTrue({t.id for t in TESTS if t.phase in (0, 1, 2)} <= first)
    self.assertTrue({"E0a", "E0b", "E0c"} <= first)
    ids = [t.id for t in TESTS]
    self.assertLess(ids.index("E0c"), ids.index("E0a"), "E0c (gear inference) is the hard prereq of the coasts")
    self.assertEqual(len(TESTS), 30)

  def test_telemetry_tests_need_no_toggle(self):
    for t in TESTS:
      if t.kind == Kind.TELEMETRY:
        self.assertEqual(t.toggles, {}, t.id)
        self.assertNotIn("op_long", t.conditions, t.id)


class TestConditions(OpenpilotTestCase):
  def test_toggle_checks_and_pitch(self):
    c = ctx(ClutchShiftGate=True)
    s = Sample(v_ego=10., pitch=float("nan"), gear_raw=-1)
    res = dict(check_conditions(("flat", "downhill", "raw_gear_live", "rolling"), {"ClutchShiftGate": True, "DisengageOnAccelerator": False}, s, c))
    self.assertTrue(res["ClutchShiftGate ON"])
    self.assertTrue(res["DisengageOnAccelerator OFF"])
    self.assertFalse(res["flat road"], "no pitch is never 'flat'")
    self.assertFalse(res["downhill"])
    self.assertFalse(res["GEAR_MT tap live"])
    self.assertTrue(res["speed > 3 m/s"])
    c.toggles["ClutchShiftGate"] = False
    self.assertFalse(dict(check_conditions((), {"ClutchShiftGate": True}, s, c))["ClutchShiftGate ON"])

  def test_bins(self):
    c = ctx()
    self.assertEqual(BINS["cmd_band"](Sample(accel_cmd=0.02), c), "zero")
    self.assertEqual(BINS["cmd_band"](Sample(accel_cmd=-0.3), c), "small_neg")
    self.assertIsNone(BINS["cmd_band"](Sample(accel_cmd=-0.1), c))
    self.assertEqual(BINS["gear_any"](Sample(gear_raw=4), c), 4)
    self.assertIsNone(BINS["gear_any"](Sample(gear_raw=0), c))
    sim = Sim(gear_raw=-1)
    from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import measured_ratio
    s = Sample(gear_raw=-1, ratio=measured_ratio(engine_rpm_at(2, 10.) * 1.12, 10.))
    self.assertEqual(BINS["gear_any"](s, c), 2, "no tap: nearest seed gear within 20% (12% off still bins)")
    del sim


class TestButton(OpenpilotTestCase):
  def test_short(self):
    b = StartButton(mode="short")
    self.assertFalse(b.update(0., btn(True)))
    self.assertTrue(b.update(0.2, btn(False)))
    self.assertFalse(b.update(1.0, btn(True)))
    self.assertFalse(b.update(1.8, btn(False)), "a long hold is not a short press")
    self.assertFalse(b.update(2.0, btn(True, ButtonType.accelCruise)) or b.update(2.1, btn(False, ButtonType.accelCruise)))
    self.assertEqual(b.seen, 2)

  def test_long(self):
    up = ButtonType.accelCruise
    b = StartButton("accelCruise", "long")
    b.update(0., btn(True, up))
    self.assertFalse(b.update(0.3, []))
    self.assertTrue(b.update(0.5, []))
    self.assertFalse(b.update(0.9, []), "fires once per hold")
    self.assertFalse(b.update(1.0, btn(False, up)))
    # Task 12 D-10: never long on the gap button (a hold toggles Experimental mode); its recommended mode is triple
    self.assertEqual(StartButton(mode="long").mode, "triple")
    self.assertEqual(StartButton().mode, "triple")
    self.assertEqual(StartButton(mode="short").mode, "short")

  def test_triple(self):
    b = StartButton(mode="triple")
    fired = []
    for t in (0., 0.4, 0.8):
      b.update(t, btn(True))
      fired.append(b.update(t + 0.1, btn(False)))
    self.assertEqual(fired, [False, False, True])
    for t in (5., 6.0, 7.5):  # too slow
      b.update(t, btn(True))
      self.assertFalse(b.update(t + 0.1, btn(False)))

  def test_configurable_button(self):
    b = StartButton("lkas", "short")
    b.update(0., btn(True))
    self.assertFalse(b.update(0.1, btn(False)))
    b.update(0.2, btn(True, ButtonType.lkas))
    self.assertTrue(b.update(0.3, btn(False, ButtonType.lkas)))
    # Task 12 D-15: the RESOLVED name is stored and shown; cancel / mainCruise are no longer allowed (D-10)
    for name in ("notAButton", "cancel", "mainCruise", ""):
      self.assertEqual(StartButton(name).name, "gapAdjustCruise", name)
      self.assertEqual(StartButton(name).type, int(ButtonType.gapAdjustCruise), "falls back to the default")
    b = StartButton("cancel", "short")
    self.assertFalse(b.differs("cancel", "short"), "no reconfigure loop on an unresolvable name")
    self.assertTrue(b.differs("lkas", "short"))


class TestLogTap(OpenpilotTestCase):
  def _record(self, msg) -> str:
    rec = logging.LogRecord("swaglog", logging.INFO, __file__, 1, msg, None, None)
    return SwagFormatter(cloudlog).format(rec)

  def test_real_swaglog_format(self):
    tap = LogEventTap()
    recs = [self._record({"event": "clutch_shift_gate", "state": "shifting", "prev": "idle"}),
            self._record({"event": "engine_brake_preferred", "active": True, "a_demand": -0.2}),
            self._record({"event": "something_else", "x": 1}),
            self._record("engine_brake_preferred: car not slowing as estimated (gear 3); suppression off until re-engaged"),
            "not json clutch_shift_gate"]
    ev = tap.update(recs)
    self.assertEqual([e[0] for e in ev], ["clutch_shift_gate", "engine_brake_preferred", "ebp_capacity"])
    self.assertEqual(tap.gate, "shifting")
    self.assertTrue(tap.ebp)


class TestSegments(OpenpilotTestCase):
  def test_coast_bins_per_gear(self):
    spec = TESTS_BY_ID["E0b"]
    a = make_analyzer(spec, ctx())
    sim = Sim(gear_raw=3, a_ego=-0.4, a_coast=-0.35)
    feed(a, sim.run(6.), ok=True)
    feed(a, sim.run(0.5), ok=False)          # gas press etc. breaks the segment
    feed(a, sim.run(0.6), ok=True)           # < min_segment: discarded
    feed(a, sim.run(0.2), ok=False)
    feed(a, sim.run(5.), ok=True)
    text, frac = a.progress()
    self.assertIn("3: 11/10s", text)
    self.assertAlmostEqual(frac, 1 / 6, places=2)
    self.assertFalse(a.complete)
    status, detail = a.result()
    self.assertEqual(status, CAPTURED)
    self.assertIn("g3 a -0.40", detail)
    for g in (1, 2, 4, 5, 6):
      sim.apply(gear_raw=g, v_ego=8. if g < 3 else 20.)
      feed(a, sim.run(10.5), ok=True)
    self.assertTrue(a.complete)

  def run_c1(self, c, a_ego, a_coast, cmd=-1.0):
    a = make_analyzer(TESTS_BY_ID["C1"], c)
    for _ in range(5):
      sim = Sim(v_ego=15., accel_cmd=cmd, accel_sent=cmd, a_ego=0., a_coast=a_coast)
      feed(a, sim.run(0.5, a_ego=0.), ok=True)      # actuator lag: excluded by settle
      feed(a, sim.run(1.5, a_ego=a_ego), ok=True)
      feed(a, sim.run(0.3), ok=False)
    self.assertTrue(a.complete)
    return a.result()

  def test_brake_strength_pass_fail_and_settle(self):
    for a_ego, want in ((-0.9, PASS), (-0.15, FAIL)):
      status, detail = self.run_c1(calibrated(ctx()), a_ego, a_coast=0.)
      self.assertEqual(status, want, detail)
      if want == PASS:
        self.assertIn("90%", detail)
    # Task 12 D3 [D-03]: CAPTURED (provisional) until the coast model is calibrated
    self.assertEqual(self.run_c1(ctx(), -0.9, a_coast=0.)[0], CAPTURED)

  def test_brake_strength_uphill_no_service_braking_never_passes(self):
    # 3 % uphill in 3rd: gravity (-0.29) + engine braking (-0.3) give -0.6 with ZERO service braking. The old a_ego/cmd
    # score called that 60 % = PASS
    for c in (calibrated(ctx()), ctx()):
      status, detail = self.run_c1(c, a_ego=-0.6, a_coast=-0.6)
      self.assertNotEqual(status, PASS, detail)
    self.assertEqual(self.run_c1(calibrated(ctx()), a_ego=-0.6, a_coast=-0.6)[0], FAIL)

  def test_cmd_semantics(self):
    spec = TESTS_BY_ID["C2"]
    a = make_analyzer(spec, ctx())
    sim = Sim(v_ego=15., a_coast=-0.4)
    feed(a, sim.run(11., accel_cmd=0.0, a_ego=0.0), ok=True)     # holds speed at command 0
    feed(a, sim.run(11., accel_cmd=-0.3, a_ego=-0.8), ok=True)   # well beyond the coast estimate: brakes
    self.assertTrue(a.complete)
    status, detail = a.result()
    self.assertEqual(status, CAPTURED)
    self.assertIn("HOLDS speed", detail)
    self.assertIn("SERVICE BRAKES", detail)

  def test_rpm_noise(self):
    spec = TESTS_BY_ID["S3"]
    a = make_analyzer(spec, ctx())
    sim = Sim(gear_raw=4, v_ego=20.)
    feed(a, sim.run(61.), ok=True)
    self.assertEqual(a.result()[0], PASS)
    feed(a, sim.run(2., rpm_rate=2500.), ok=True)
    self.assertEqual(a.result()[0], FAIL)

  def test_ratio_check(self):
    spec = TESTS_BY_ID["E0c"]
    a = make_analyzer(spec, ctx())
    sim = Sim()
    for g in range(1, 7):
      sim.apply(gear_raw=g, v_ego=5. + 3 * g)
      feed(a, sim.run(3.5), ok=True)
    self.assertEqual(a.result()[0], PASS)
    b = make_analyzer(spec, ctx())   # a final drive 5% off the seed shows up as FAIL
    sim = Sim()
    sim.locked = False
    for g in range(1, 7):
      v = 5. + 3 * g
      sim.apply(gear_raw=g, v_ego=v)
      feed(b, sim.run(3.5, rpm=engine_rpm_at(g, v) * 1.05), ok=True)
    status, detail = b.result()
    self.assertEqual(status, FAIL, detail)
    self.assertIn("+5.0%", detail)

  def test_baseline_health(self):
    spec = TESTS_BY_ID["D1"]
    a = make_analyzer(spec, ctx())
    sim = Sim(long_active=True, enabled=True, tx_blocked=7)
    feed(a, sim.run(300.5), ok=True)
    self.assertTrue(a.complete)
    self.assertEqual(a.result()[0], PASS)
    feed(a, sim.run(0.1, tx_blocked=9), ok=True)
    self.assertEqual(a.result()[0], FAIL)
    self.assertIn("2 blocked TX", a.result()[1])

  def test_nsb_floor(self):
    spec = TESTS_BY_ID["NSB1"]
    for cal in (True, False):
      with self.subTest(calibrated=cal):
        c = calibrated(ctx()) if cal else ctx()
        a = make_analyzer(spec, c)
        sim = Sim(v_ego=20., accel_out=-0.3, accel_sent=-0.3, a_ego=0.1, a_coast=0.05, excess=0.05)
        feed(a, sim.run(31.), ok=True)
        self.assertIsNone(a.result()[0], "one floor done, 0.0 still to go")
        c.nsb_floor = 0.0
        feed(a, sim.run(31., accel_out=0.0, accel_sent=0.0), ok=True)
        self.assertEqual(a.result()[0], PASS if cal else CAPTURED, "provisional until E0b (Task 12 D3)")
        feed(a, sim.run(2., accel_out=-0.2), ok=True)
        self.assertEqual(a.result()[0], FAIL, "command below -floor: no calibration needed")


class TestShifts(OpenpilotTestCase):
  def test_bite_lag(self):
    for lag, want in ((0.3, PASS), (0.8, FAIL)):
      a = make_analyzer(TESTS_BY_ID["S2"], ctx())
      sim = Sim(gear_raw=2, v_ego=10.)
      feed(a, sim.run(1.))
      for i in range(10):
        feed(a, shift(sim, 2 + i % 2, 3 - i % 2, bite_lag=lag))
      self.assertTrue(a.complete, a.progress())
      status, detail = a.result()
      self.assertEqual(status, want, detail)
      self.assertAlmostEqual(A._pct([e["bite_lag"] for e in a.episodes], 0.5), lag, delta=0.06)
      self.assertEqual({e["kind"] for e in a.episodes}, {"up", "down"})

  def test_lift_to_clutch(self):
    a = make_analyzer(TESTS_BY_ID["S4"], ctx())
    sim = Sim(gear_raw=2, v_ego=10.)
    for i in range(10):
      feed(a, shift(sim, 2 + i % 2, 3 - i % 2, lift_dt=0.3))
    status, detail = a.result()
    self.assertEqual(status, PASS, detail)
    self.assertAlmostEqual(a.episodes[0]["gas_release_dt"], 0.3, delta=0.02)
    self.assertLessEqual(0.3, 0.6)

  def test_blip_flicker(self):
    a = make_analyzer(TESTS_BY_ID["S5"], ctx())
    sim = Sim(gear_raw=4, v_ego=15.)
    feed(a, shift(sim, 4, 3, blip=False))                  # no blip: filtered out
    self.assertEqual(len(a.episodes), 0)
    for _ in range(5):
      feed(a, shift(sim, 4, 3, blip=True, open_s=0.8))
      feed(a, shift(sim, 3, 4))
    self.assertTrue(a.complete)
    self.assertEqual(a.result()[0], PASS, a.result())
    feed(a, shift(sim, 4, 3, blip=True, open_s=0.8, flicker=True))
    status, detail = a.result()
    self.assertEqual(status, FAIL, detail)
    self.assertIn("flickers 1", detail)

  def test_gate_accept(self):
    # Task 12 A2: longitudinal stays ACTIVE through the press; the gate publishes 'paused' and caps the throttle
    spec = TESTS_BY_ID["G1"]
    a = make_analyzer(spec, ctx(ClutchShiftGate=True))
    sim = Sim(gear_raw=3, v_ego=12., enabled=True, long_active=True)
    for _ in range(5):
      feed(a, shift(sim, 3, 3, open_s=1.5, enabled=True, long_active=True, gate=True))
    self.assertTrue(a.complete)
    self.assertEqual(a.result()[0], PASS, a.result())
    self.assertTrue(all(e["kind"] == "same" for e in a.episodes))
    b = make_analyzer(spec, ctx(ClutchShiftGate=True))   # the gate never paused while the clutch was in
    for _ in range(5):
      feed(b, shift(sim, 3, 3, open_s=1.5, enabled=True, long_active=True))
    self.assertEqual(b.result()[0], FAIL)
    d = make_analyzer(spec, ctx(ClutchShiftGate=True))   # throttle sent with the clutch in
    for _ in range(5):
      feed(d, shift(sim, 3, 3, open_s=1.5, enabled=True, long_active=True, gate=True))
      feed(d, [])
    sim.apply(accel_out=0.3)
    feed(d, shift(sim, 3, 3, open_s=1.5, enabled=True, long_active=True, gate=True))
    sim.apply(accel_out=0.)
    self.assertEqual(d.result()[0], FAIL)
    c = make_analyzer(spec, ctx(ClutchShiftGate=True))   # cruise fault during the press
    feed(c, shift(sim, 3, 3, open_s=1.5, enabled=True, long_active=True, gate=True))
    feed(c, sim.run(0.1, acc_faulted=True))
    for _ in range(4):
      feed(c, shift(sim, 3, 3, open_s=1.5, enabled=True, long_active=True, gate=True))
    self.assertEqual(c.result()[0], FAIL)

  def test_gate_shifts_need_up_and_down_and_false_flares(self):
    a = make_analyzer(TESTS_BY_ID["G2"], ctx(ClutchShiftGate=True))
    sim = Sim(gear_raw=2, v_ego=12., enabled=True, long_active=True)
    for _ in range(6):
      feed(a, shift(sim, 2, 3, enabled=True, long_active=True, gate=True))
      sim.apply(gear_raw=2)
    text, frac = a.progress()
    self.assertIn("up 6/3", text)
    self.assertIn("down 0/3", text)
    self.assertLess(frac, 1.)
    self.assertIsNone(a.result()[0], "no PASS before complete (Task 12 D3)")
    for _ in range(3):
      feed(a, shift(sim, 3, 2, enabled=True, long_active=True, gate=True))
      sim.apply(gear_raw=3)
    self.assertTrue(a.complete)
    self.assertEqual(a.result()[0], PASS, a.result())
    feed(a, sim.run(0.05, gate="shifting"))   # gate starts a pause with no clutch: RPM backstop false flare
    feed(a, sim.run(1.2))
    feed(a, sim.run(0.5, gate="idle"))
    self.assertEqual(a.detector.false_flares, 1)
    self.assertEqual(a.result()[0], FAIL)


class TestOtherEpisodes(OpenpilotTestCase):
  def _handback(self, sim: Sim, alerts=1):
    sim.events = [("manual_low_speed_handback", {"event": "manual_low_speed_handback", "process": "controlsd", "v_ego": 2.2,
                                                 "accel": 0.2, "speed": 2.24, "ramp": 0.5})]
    out = list(sim.run(0.6, a_ego=-0.3, enabled=True))
    for _ in range(alerts):
      out += sim.run(0.3, alert="manualLowSpeedHandback", enabled=False)
      out += sim.run(0.2, alert="")
    out += sim.run(4.)
    return out

  def test_handback(self):
    a = make_analyzer(TESTS_BY_ID["H1"], ctx())
    sim = Sim(v_ego=3., enabled=True)
    for _ in range(5):
      feed(a, self._handback(sim))
      sim.apply(v_ego=3., enabled=True)
    self.assertTrue(a.complete)
    self.assertEqual(a.result()[0], PASS, a.result())
    feed(a, self._handback(sim, alerts=2))
    self.assertEqual(a.result()[0], FAIL)

  def test_stop_hold_rollback(self):
    a = make_analyzer(TESTS_BY_ID["H2"], ctx())
    sim = Sim(gear_raw=-1, v_ego=5.)
    feed(a, sim.run(1.))
    feed(a, sim.run(0.5, v_ego=0., brake=True, clutch=True, pitch=0.05))
    feed(a, sim.run(2., brake=False))                       # held
    feed(a, sim.run(1., gas=True, v_ego=4.))                # drive off
    self.assertEqual(len(a.episodes), 1)
    self.assertFalse(a.episodes[0]["rolled"])
    feed(a, sim.run(0.5, gas=False, v_ego=0., brake=True))
    feed(a, sim.run(0.5, brake=False, v_ego=0.5))           # rolls back
    feed(a, sim.run(1., gas=True, v_ego=4.))
    status, detail = a.result()
    self.assertEqual(status, FAIL, detail)
    self.assertIn("rollback 1", detail)

  def test_neutral_coast(self):
    a = make_analyzer(TESTS_BY_ID["H3"], ctx())
    sim = Sim(gear_raw=4, v_ego=12., enabled=True, long_active=True)
    feed(a, sim.run(1.))
    sim.locked = False
    feed(a, sim.run(7., gear_raw=0, clutch=True, rpm=800., long_active=False, gate="shifting"))
    sim.locked = True
    feed(a, sim.run(1., gear_raw=4, clutch=False, gate="idle", long_active=True))
    self.assertEqual(len(a.episodes), 1)
    status, detail = a.result()
    self.assertEqual(status, CAPTURED)
    self.assertIn("GEAR_MT 0:", detail)

  def test_backstop(self):
    c = ctx()
    a = make_analyzer(TESTS_BY_ID["NSB2"], c)
    sim = Sim(v_ego=17.0, v_cruise=13.4, long_active=True, allow_brake=False)
    feed(a, sim.run(1.))
    sim.events = [("no_service_brake_backstop", {"v_ego": 17.0, "v_set": 13.4})]
    feed(a, sim.run(3.5, allow_brake=True, a_ego=-0.6))
    self.assertEqual(a.result()[0], PASS, a.result())
    b = make_analyzer(TESTS_BY_ID["NSB2"], c)
    sim.events = [("no_service_brake_backstop", {"v_ego": 17.0, "v_set": 13.4})]
    feed(b, sim.run(3.5, a_ego=-0.05))
    self.assertEqual(b.result()[0], FAIL, "backstop with no real braking")

  def pulse_drive(self, a, sim, cycles=2, start=3.6, release=0.9, cmd=-1.0, coast_cmd=-0.3, pulse_s=4., coast_s=6.,
                  escalate=False, alerts=1):
    """Pulse-mode overspeed cycles as the planner logs them: no_service_brake_pulse phase events + the command."""
    for _ in range(cycles):
      feed(a, sim.run(coast_s, accel_cmd=coast_cmd))
      sim.events = [("no_service_brake_pulse", {"phase": "pulsing", "prev": "coasting", "overspeed": start, "v_set": 13.4})]
      feed(a, sim.run(pulse_s / 2, accel_cmd=cmd, a_ego=-0.5))
      if escalate:
        sim.events = [("no_service_brake_pulse", {"phase": "escalated", "prev": "pulsing", "overspeed": 6.75, "v_set": 13.4})]
        feed(a, sim.run(0.1, accel_cmd=-1.2))
        for _ in range(alerts):
          feed(a, sim.run(0.5, alert="noServiceBrakeOverspeed", accel_cmd=-1.2))
          feed(a, sim.run(0.5, alert="", accel_cmd=-1.2))
      feed(a, sim.run(pulse_s / 2, accel_cmd=cmd if not escalate else -1.2, a_ego=-0.5))
      sim.events = [("no_service_brake_pulse", {"phase": "coasting", "prev": "escalated" if escalate else "pulsing",
                                                 "overspeed": release, "v_set": 13.4})]
      feed(a, sim.run(0.5, accel_cmd=cmd * 0.5))  # rolling off

  def test_backstop_pulse_mode(self):
    c = ctx()
    c.nsb_mode = "pulse"
    a = make_analyzer(TESTS_BY_ID["NSB2"], c)
    self.assertEqual(a.count, 2, "pulse mode needs two pulses (a coast between them)")
    sim = Sim(v_ego=17.0, v_cruise=13.4, long_active=True, allow_brake=False)
    self.pulse_drive(a, sim, cycles=1)
    self.assertEqual(len(a.episodes), 1)
    self.assertIsNone(a.result()[0], "one clean pulse: not complete yet")
    self.pulse_drive(a, sim, cycles=1)
    status, detail = a.result()
    self.assertEqual(status, PASS, detail)
    self.assertIn("2/2 clean", detail)
    self.assertNotIn("ESCALATED", detail)
    c.nsb_mode = "maintain"
    self.assertEqual(a.count, 1)

  def test_backstop_pulse_faults(self):
    for kw, why in (({"start": 5.0}, "started at set+5.0"), ({"release": 2.5}, "released at set+2.5"),
                    ({"cmd": -1.6}, "outside [-1.0, 0]"), ({"pulse_s": 30.}, "brake hold"),
                    ({"coast_cmd": -0.8}, "no coast before it"), ({"coast_s": 0.2}, "no coast before it")):
      c = ctx()
      c.nsb_mode = "pulse"
      a = make_analyzer(TESTS_BY_ID["NSB2"], c)
      self.pulse_drive(a, Sim(v_ego=17.0, v_cruise=13.4, long_active=True), cycles=2, **kw)
      status, detail = a.result()
      self.assertEqual(status, FAIL, (kw, detail))
      self.assertIn(why, detail, kw)

  def test_backstop_pulse_escalation_flagged(self):
    c = ctx()
    c.nsb_mode = "pulse"
    a = make_analyzer(TESTS_BY_ID["NSB2"], c)
    sim = Sim(v_ego=17.0, v_cruise=13.4, long_active=True)
    self.pulse_drive(a, sim, cycles=2, escalate=True)
    status, detail = a.result()
    self.assertEqual(status, PASS, detail)
    self.assertIn("ESCALATED 2x", detail)
    self.assertIn("alerts 2", detail)
    b = make_analyzer(TESTS_BY_ID["NSB2"], c)
    self.pulse_drive(b, Sim(v_ego=17.0, v_cruise=13.4, long_active=True), cycles=2, escalate=True, alerts=2)
    self.assertEqual(b.result()[0], FAIL, "one alert per escalation")
    # a disengage mid-pulse is not a scorable pulse
    d = make_analyzer(TESTS_BY_ID["NSB2"], c)
    sim = Sim(v_ego=17.0, v_cruise=13.4, long_active=True)
    sim.events = [("no_service_brake_pulse", {"phase": "pulsing", "prev": "coasting", "overspeed": 3.6, "v_set": 13.4})]
    feed(d, sim.run(1., accel_cmd=-1.))
    feed(d, sim.run(1., long_active=False))
    self.assertEqual(len(d.episodes), 0)

  def test_ebp_trims(self):
    b = make_analyzer(TESTS_BY_ID["E1"], ctx(EngineBrakePreferred=True))   # uncalibrated: provisional (Task 12 D3)
    a = make_analyzer(TESTS_BY_ID["E1"], calibrated(ctx(EngineBrakePreferred=True)))
    sim = Sim(long_active=True, a_coast=-0.4, accel_sent=0.0)
    for _ in range(5):
      sim.events = [("engine_brake_preferred", {"active": True, "gear": 4})]
      samples = list(sim.run(2., accel_out=0.0, a_ego=-0.35))
      sim.events = [("engine_brake_preferred", {"active": False, "reason": "idle"})]
      samples += sim.run(0.5)
      feed(a, samples)
      feed(b, samples)
    self.assertTrue(a.complete)
    self.assertEqual(a.result()[0], PASS, a.result())
    self.assertEqual(b.result()[0], CAPTURED, b.result())
    sim.events = [("engine_brake_preferred", {"active": True, "gear": 4})]
    feed(a, sim.run(1., accel_out=0.0))
    sim.events = [("ebp_capacity", {}), ("engine_brake_preferred", {"active": False, "reason": "capacity"})]
    feed(a, sim.run(0.5))
    self.assertEqual(a.result()[0], FAIL)

  def test_lead_approach_on_off(self):
    c = ctx(EngineBrakePreferred=True)
    a = make_analyzer(TESTS_BY_ID["E2"], c)
    sim = Sim(long_active=True, has_lead=True, lead_d=60., lead_vrel=-3.)
    for d in range(300):
      sim.step(lead_d=60. - d * 0.1)
      a.feed(sim.s, True)
    feed(a, sim.run(1.5, lead_vrel=0.))
    self.assertIn("ebp_on 1/1", a.progress()[0])
    c.toggles["EngineBrakePreferred"] = False
    sim.apply(lead_d=60., lead_vrel=-3.)
    for d in range(300):
      sim.step(lead_d=60. - d * 0.08)
      a.feed(sim.s, True)
    feed(a, sim.run(1.5, lead_vrel=0.))
    self.assertTrue(a.complete)
    status, detail = a.result()
    self.assertEqual(status, FAIL, detail)   # EBP on closed to 30 m, off only to 36 m: worse by > 2 m

  def test_non_adaptive(self):
    a = make_analyzer(TESTS_BY_ID["N1"], ctx(NonAdaptiveLongitudinal=True))
    sim = Sim(v_ego=13.4, v_cruise=13.4, long_active=True)
    feed(a, sim.run(61., v_ego=13.5), ok=True)
    feed(a, sim.run(6., has_lead=True, lead_d=40., lead_vrel=-2., accel_cmd=0.0), ok=True)
    self.assertTrue(a.complete)
    self.assertEqual(a.result()[0], PASS, a.result())
    b = make_analyzer(TESTS_BY_ID["N1"], ctx(NonAdaptiveLongitudinal=True))
    feed(b, sim.run(61., has_lead=False, v_ego=13.5), ok=True)
    feed(b, sim.run(6., has_lead=True, lead_d=40., lead_vrel=-2., accel_cmd=-0.8), ok=True)
    self.assertEqual(b.result()[0], FAIL, "braked for the lead")


class TestCustom(OpenpilotTestCase):
  def test_gear_mt_map(self):
    a = make_analyzer(TESTS_BY_ID["S1"], ctx())
    sim = Sim(v_ego=0., gear_raw=0, clutch=True)
    feed(a, sim.run(6.), ok=True)                                    # parked, clutch down / neutral
    feed(a, sim.run(0.5, reverse=True, gear_raw=14, clutch=False), ok=True)
    for g in range(1, 7):
      sim.apply(reverse=False, gear_raw=g, clutch=False, v_ego=4. + 3 * g)
      feed(a, sim.run(2.5), ok=True)
    self.assertTrue(a.complete, a.progress())
    status, detail = a.result()
    self.assertEqual(status, PASS, detail)
    self.assertIn("R=[14]", detail)
    b = make_analyzer(TESTS_BY_ID["S1"], ctx())                      # GEAR_MT 5 actually behaves like 6th
    sim = Sim(v_ego=25., gear_raw=5)
    sim.locked = False
    feed(b, sim.run(3., rpm=engine_rpm_at(6, 25.)), ok=True)
    status, detail = b.result()
    self.assertEqual(status, FAIL)
    self.assertIn("GEAR_MT 5 reads as inferred 6", detail)

  def test_bit_hunt(self):
    a = make_analyzer(TESTS_BY_ID["S6"], ctx())
    sim = Sim(v_ego=0., gear_raw=3)
    raw = 0
    counter_bit = 1 << 4          # a counter-like bit toggling every frame
    clutch_bit = 1 << (63 - 35)   # payload bit: byte 3, bit-in-byte 3 -> DBC bit 27 (big-endian int)
    for _press in range(9):
      for _ in range(50):
        raw ^= counter_bit
        a.feed(sim.step(raw_payload=raw, gear_raw=3), True)
      raw |= clutch_bit
      for _ in range(50):
        raw ^= counter_bit
        a.feed(sim.step(raw_payload=raw, gear_raw=0 if _ >= 5 else 3), True)  # GEAR_MT lags the switch by 50 ms
      raw &= ~clutch_bit
    self.assertFalse(a.complete, "Task 12 D-19: the last release needs a tail")
    for _ in range(110):
      raw ^= counter_bit
      a.feed(sim.step(raw_payload=raw, gear_raw=3), True)
    self.assertTrue(a.complete)
    cands = a.candidates()
    self.assertEqual(len(cands), 1, cands)
    self.assertEqual(cands[0][0], A.BitHuntAnalyzer.dbc_bit(63 - 35))
    self.assertEqual(A.BitHuntAnalyzer.dbc_bit(63), 7, "MSB of byte 0 is DBC bit 7")
    self.assertEqual(A.BitHuntAnalyzer.dbc_bit(0), 56, "LSB of byte 7 is DBC bit 56")


class TestConsoleFlow(OpenpilotTestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.root = self.tmp.name
    self.c = ctx()
    self.console = Console(Store(self.root), self.c)

  def tearDown(self):
    self.tmp.cleanup()

  def _markers(self) -> list[dict]:
    path = os.path.join(self.root, MARKERS_FILE)
    if not os.path.exists(path):
      return []
    with open(path) as f:
      return [json.loads(line) for line in f]

  def test_default_inert(self):
    sim = Sim(gear_raw=3, v_ego=10.)
    for s in sim.run(5.):
      self.console.update(s, button=True)   # button presses with nothing armed do nothing
    self.console.shutdown()
    self.assertEqual(self._markers(), [])
    self.assertEqual(os.listdir(os.path.join(self.root, "captures")), [])
    self.assertFalse(os.path.exists(os.path.join(self.root, RESULTS_FILE)))
    st = self.console.status(sim.s, {})
    self.assertFalse(st["armed"])
    self.assertEqual(st["state"], "idle")

  def test_arm_start_complete(self):
    self.console.set_control("E0b", True, False)
    self.assertEqual(self.console.state, "armed")
    sim = Sim(gear_raw=3, v_ego=15., a_ego=-0.3, pitch=0.)
    self.console.update(sim.step(), False)
    self.assertTrue(self.console.qualifying, "conditions are live while armed (the driver sees what's missing)...")
    self.assertEqual(os.listdir(os.path.join(self.root, "captures")), [], "...but nothing is captured until the button")
    self.console.update(sim.step(), True)
    self.assertEqual(self.console.state, "capturing")
    for g in range(1, 7):
      sim.apply(gear_raw=g, v_ego=6. + 3 * g)
      for s in sim.run(10.5):
        self.console.update(s, False)
    self.assertEqual(self.console.state, "complete")
    actions = [m["action"] for m in self._markers()]
    self.assertEqual(actions[:2], ["arm", "start"])
    self.assertEqual(actions[-1], "complete")
    self.assertIn("sample", actions)
    done = self._markers()[-1]
    self.assertEqual(done["test"], "E0b")
    self.assertEqual(done["phase"], 7)
    self.assertEqual(done["status"], CAPTURED)
    self.assertIn("signals", done)
    res = read_json(os.path.join(self.root, RESULTS_FILE))["E0b"]
    self.assertTrue(res["complete"])
    caps = os.listdir(os.path.join(self.root, "captures"))
    self.assertEqual(len(caps), 1)
    with open(os.path.join(self.root, "captures", caps[0])) as f:
      lines = f.read().splitlines()
    header = json.loads(lines[0])
    self.assertEqual(header["fields"], [CIVIC.column(f) for f in TESTS_BY_ID["E0b"].fields])  # the Civic keeps its column names
    self.assertGreater(len(lines), 6 * 10 * 20 * 0.9, "rows at ~20 Hz")
    self.assertEqual(len(json.loads(lines[1])), len(header["fields"]))
    # a press after complete starts a fresh run
    self.console.update(sim.step(), True)
    self.assertEqual(self.console.state, "capturing")
    self.assertEqual(self.console.run.run, 2)

  def test_stop_resume_and_abort(self):
    self.console.set_control("E0b", True, False)
    sim = Sim(gear_raw=3, v_ego=15.)
    self.console.update(sim.step(), True)
    for s in sim.run(4.):
      self.console.update(s, False)
    self.console.update(sim.step(), True)            # stop = pause
    self.assertEqual(self.console.state, "stopped")
    for s in sim.run(4.):
      self.console.update(s, False)                  # not captured while stopped
    self.console.update(sim.step(), True)            # resume, new window
    for s in sim.run(4.):
      self.console.update(s, False)
    self.assertIn("3: 8/10s", self.console.run.analyzer.progress()[0])
    self.assertEqual(self.console.run.window, 2)
    self.console.set_control("E0b", False, False)    # disarm mid-capture
    actions = [m["action"] for m in self._markers()]
    self.assertEqual(actions, ["arm", "start", "sample", "stop", "start", "sample", "abort", "disarm"])

  def test_background_telemetry(self):
    self.console.set_control("", False, True)
    sim = Sim(gear_raw=3, v_ego=15.)
    for g in range(1, 7):
      sim.apply(gear_raw=g, v_ego=6. + 3 * g)
      for s in sim.run(10.5):
        self.console.update(s, False)
    res = read_json(os.path.join(self.root, RESULTS_FILE))
    self.assertTrue(res["E0b"]["complete"], "flat coasting completes E0b in the background")
    self.assertTrue(res["E0b"]["background"])
    self.assertNotIn("G1", self.console.bg_runs, "feature tests never run in the background")
    self.console.set_control("", False, False)


class TestSignals(OpenpilotTestCase):
  def test_builder(self):
    cs = messaging.new_message("carState").carState
    cs.vEgo, cs.aEgo, cs.vCruise = 20., -0.2, 72.
    cs.deprecated.engineRpm = engine_rpm_at(4, 20.)
    cc = messaging.new_message("carControl").carControl
    cc.orientationNED = [0., -0.03, 0.]
    cc.enabled, cc.longActive = True, True
    cc.actuators.accel = -0.4
    co = messaging.new_message("carOutput").carOutput
    plan = messaging.new_message("longitudinalPlan").longitudinalPlan
    radar = messaging.new_message("radarState").radarState
    radar.leadOne.present, radar.leadOne.dRel = True, 33.
    ss = messaging.new_message("selfdriveState").selfdriveState
    ss.alertType = "manualLowSpeedHandback/immediateDisable"
    s = SampleBuilder().build(1.0, cs, cc, co, plan, True, radar, ss, 3, None, LogEventTap(), ())
    self.assertAlmostEqual(s.v_cruise, 20., places=3)
    self.assertEqual(s.gear, 4)
    self.assertAlmostEqual(s.pitch, -0.03, places=5)
    self.assertTrue(s.has_lead and s.lead_d == 33.)
    self.assertEqual(s.alert, "manualLowSpeedHandback")
    self.assertEqual(s.gear_raw, -1)
    self.assertTrue(math.isfinite(s.a_coast))
    cs.vCruise = 255.
    self.assertTrue(math.isnan(SampleBuilder().build(1.0, cs, cc, co, plan, True, radar, ss, 3, None, LogEventTap(), ()).v_cruise))

  def test_raw_can_tap_civic(self):
    from opendbc.can import CANPacker
    from opendbc.car import Bus
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.honda.values import CAR, DBC
    CarInterface = interfaces[CAR.HONDA_CIVIC_2022]
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
    tap = RawCanTap(CP)
    self.assertTrue(tap.available)
    packer = CANPacker(DBC[CAR.HONDA_CIVIC_2022][Bus.pt])
    gear = packer.make_can_msg("GEARBOX_ALT_2", tap.bus, {"GEAR_MT": 4})
    pt = packer.make_can_msg("POWERTRAIN_DATA", tap.bus, {"PEDAL_GAS": 37})
    acc = packer.make_can_msg("ACC_CONTROL", tap.bus, {"ACCEL_COMMAND": -0.42, "CONTROL_ON": 1})
    tap.update([(int(1e9), [gear, pt])], [(int(1e9), [acc])])
    self.assertEqual(tap.gear_mt, 4)
    self.assertEqual(tap.pedal_gas, 37)
    self.assertGreaterEqual(tap.raw_17c, 0)
    self.assertAlmostEqual(tap.accel_sent, -0.42, places=2)
    toyota = structs.CarParams(brand="toyota")
    self.assertFalse(RawCanTap(toyota).available)


class TestNonActuating(OpenpilotTestCase):
  """Structural proof of 'zero driving authority' (the replay diff in tools/test_console_replay.py is the behavioral one)."""
  FORBIDDEN_CALLS = {"PubMaster", "pub_sock", "put", "put_bool", "put_nonblocking", "put_bool_nonblocking", "remove", "send"}

  def _calls(self, path: pathlib.Path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
      if isinstance(node, ast.Call):
        f = node.func
        if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name) and f.value.id == "os":
          continue  # os.remove etc. on its own files (capture pruning) is fine
        name = f.attr if isinstance(f, ast.Attribute) else f.id if isinstance(f, ast.Name) else None
        yield name, node

  def test_daemon_writes_nothing_but_files(self):
    self.assertTrue(any("profiles" in p.parts for p in CONSOLE_SRCS))
    for path in CONSOLE_SRCS:
      for name, _ in self._calls(path):
        self.assertNotIn(name, self.FORBIDDEN_CALLS, f"{path.name} calls {name}()")
      text = path.read_text()
      for mod in ("carcontroller", "longcontrol", "controlsd", "plannerd", "panda"):
        self.assertNotIn(f"import {mod}", text)
      self.assertNotIn("sendcan\", conflate", text.replace("sub_sock(\"sendcan\", conflate", ""))

  def test_ui_writes_only_through_the_allowlist(self):
    # Task 9 Part C: the UI may now set its sessions' feature toggles, but ONLY via dev_test_params.write_param, which
    # refuses anything outside sessions.UI_WRITABLE. No other Param write (direct or via a Param-bound widget) in the UI.
    ui = PKG.parents[2] / "selfdrive" / "ui" / "sunnypilot" / "mici"
    files = [ui / "layouts" / "dev_test.py", ui / "layouts" / "dev_test_sessions.py", ui / "onroad" / "test_console_chip.py"]
    for path in files:
      self.assertTrue(path.exists(), path)
      for name, _ in self._calls(path):
        self.assertNotIn(name, ("put", "put_bool", "put_nonblocking", "put_bool_nonblocking", "remove"), path.name)
        self.assertNotIn(name, ("BigParamControl", "BigMultiParamToggle", "BigCircleParamControl"), path.name)
    writer = [(name, node) for name, node in self._calls(ui / "layouts" / "dev_test_params.py") if name in ("put", "put_bool")]
    self.assertEqual(len(writer), 2, "write_param is the single writer")

  def test_imported_control_modules_untouched(self):
    # the console imports constants/pure functions from the feature modules; it must never assign into them
    for path in CONSOLE_SRCS:
      tree = ast.parse(path.read_text())
      for node in ast.walk(tree):
        if isinstance(node, (ast.Assign, ast.AugAssign)):
          targets = node.targets if isinstance(node, ast.Assign) else [node.target]
          for t in targets:
            if isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name):
              self.assertNotIn(t.value.id, ("clutch_shift_gate", "engine_brake_model", "no_service_brake"), path.name)


class TestUiHelpers(OpenpilotTestCase):
  def test_chip_lines(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad.test_console_chip import chip_lines
    st = {"selected": "G1", "state": "capturing", "progress": "shifts 2/5", "conditions": [["op engaged", False], ["manual transmission", True]]}
    lines, key = chip_lines(st)
    self.assertEqual(lines, ["G1 ● REC", "shifts 2/5", "× op engaged"])
    self.assertEqual(key, "capturing")
    lines, key = chip_lines({**st, "state": "complete", "status": "FAIL", "detail": "x"})
    self.assertEqual(key, "FAIL")
    self.assertEqual(chip_lines({**st, "state": "armed"})[0][0], "G1 armed: short-press to start")
    armed = {**st, "state": "armed", "conditions": [], "button": {"name": "gapAdjustCruise", "mode": "triple",
                                                                  "note": "gap press also steps the follow distance"}}
    self.assertEqual(chip_lines(armed)[0], ["G1 armed: triple-press gapAdjustCruise to start",
                                            "gap press also steps the follow distance"], "Task 12 D-10")

  def test_condition_lines_and_hints(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test import condition_lines
    from openpilot.sunnypilot.selfdrive.test_console.status_io import prereq_hint, result_line
    self.assertEqual(condition_lines([["a", True], ["b", True]]), "✓ all 2 met")
    many = [[f"c{i}", True] for i in range(7)] + [["bad", False]]
    text = condition_lines(many)
    self.assertTrue(text.startswith("× bad"))
    self.assertLessEqual(len(text.splitlines()), 5)
    self.assertIn("more", text)
    res = {"S1": {"status": "PASS", "complete": True}, "S2": {"status": "FAIL", "complete": True}}
    self.assertEqual(prereq_hint(("S1",), res), "prereqs done")
    self.assertEqual(prereq_hint(("S1", "S2", "D1"), res), "recommended first: S2 FAILED, D1")
    self.assertEqual(result_line({}), "not run")
    self.assertEqual(result_line({"status": None, "complete": False, "progress": "3/10 s"}), "partial 3/10 s")

  def test_glyphs_are_loaded_by_the_ui(self):
    # the UI only loads ASCII + EXTRA_FONT_CHARS (+ translations); anything else renders as '?'
    from openpilot.system.ui.lib.application import EXTRA_FONT_CHARS
    allowed = set(map(chr, range(32, 127))) | set(EXTRA_FONT_CHARS) | {"\n"}
    srcs = list(CONSOLE_SRCS) + [PKG.parents[2] / "selfdrive/ui/sunnypilot/mici/layouts/dev_test.py",
                                     PKG.parents[2] / "selfdrive/ui/sunnypilot/mici/layouts/dev_test_sessions.py",
                                     PKG.parents[2] / "selfdrive/ui/sunnypilot/mici/onroad/test_console_chip.py"]
    for path in srcs:
      tree = ast.parse(path.read_text())
      docstrings = {id(n.body[0].value) for n in ast.walk(tree)
                    if isinstance(n, (ast.Module, ast.ClassDef, ast.FunctionDef)) and n.body and isinstance(n.body[0], ast.Expr)}
      for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings:
          bad = set(node.value) - allowed
          self.assertFalse(bad, f"{path.name}:{node.lineno} uses glyphs the UI font doesn't load: {bad}")


class TestTask12Verdicts(OpenpilotTestCase):
  """Task 12 D3: the reviewers' harness cases - verdicts that used to false-PASS."""

  def test_g2_no_resume_fails(self):
    for resume_after, want in ((0.5, PASS), (None, FAIL), (2.0, FAIL)):   # 2.0 s > REENGAGE_DELAY + 1 s
      with self.subTest(resume_after=resume_after):
        a = make_analyzer(TESTS_BY_ID["G2"], ctx(ClutchShiftGate=True))
        sim = Sim(gear_raw=2, v_ego=12., enabled=True, long_active=True)
        for up in (True, False) * 3:
          g0, g1 = (2, 3) if up else (3, 2)
          feed(a, shift(sim, g0, g1, enabled=True, long_active=True, gate=True, resume_after=resume_after))
        self.assertTrue(a.complete)
        self.assertEqual(a.result()[0], want, a.result())

  def test_g2_needs_shifting_state(self):
    a = make_analyzer(TESTS_BY_ID["G2"], ctx(ClutchShiftGate=True))
    sim = Sim(gear_raw=2, v_ego=12., enabled=True, long_active=True)
    for up in (True, False) * 3:
      g0, g1 = (2, 3) if up else (3, 2)
      feed(a, shift(sim, g0, g1, enabled=True, long_active=True))   # gate never says 'shifting'
    self.assertEqual(a.result()[0], FAIL, a.result())

  def passing_s5(self):
    a = make_analyzer(TESTS_BY_ID["S5"], ctx())
    sim = Sim(gear_raw=4, v_ego=15.)
    for _ in range(5):
      feed(a, shift(sim, 4, 3, blip=True, open_s=0.8))
      feed(a, shift(sim, 3, 4))
    self.assertEqual(a.result()[0], PASS, a.result())
    feed(a, sim.run(1.0, gas=True, gear_raw=4, clutch=False))
    feed(a, sim.run(0.25, gas=False))
    return a, sim

  def test_s5_gear_mt_leaks_fail(self):
    # GEAR_MT reports a gear for `leak` s in the middle of a blipped downshift (off the gas by then), then 0 again
    for leak in (0.2, 0.4, 0.6):
      with self.subTest(leak=leak):
        a, sim = self.passing_s5()
        feed(a, sim.run(0.3, clutch=True, gear_raw=0, gas=True))
        feed(a, sim.run(0.1, clutch=True, gear_raw=0, gas=False))
        feed(a, sim.run(leak, clutch=False, gear_raw=3, gas=False))
        feed(a, sim.run(0.4, clutch=True, gear_raw=0, gas=False))
        feed(a, sim.run(3.5, clutch=False, gear_raw=3, gas=False))
        status, detail = a.result()
        self.assertEqual(status, FAIL, detail)
        self.assertIn("GEAR_MT leaks 1", detail)

  def test_s5_close_with_gas_pressed_is_a_leak(self):
    a, sim = self.passing_s5()
    feed(a, sim.run(0.4, clutch=True, gear_raw=0, gas=True))
    feed(a, sim.run(3.5, clutch=False, gear_raw=3, gas=True))   # GEAR_MT reports the gear while the blip is still on
    status, detail = a.result()
    self.assertEqual(status, FAIL, detail)

  def test_s2_one_gear_ratio_off_never_passes(self):
    # into 3rd the rpm settles 10 % off the 3rd-gear ratio (wrong seed ratio / bad decode): those bites are never
    # measured, so the gear's coverage is 0 % - must not PASS on the other gears' numbers
    a = make_analyzer(TESTS_BY_ID["S2"], ctx())
    sim = Sim(gear_raw=2, v_ego=10.)
    feed(a, sim.run(1.))
    for i in range(10):
      g_from, g_to = (2, 3) if i % 2 == 0 else (3, 2)
      out = shift(sim, g_from, g_to, bite_lag=0.3)
      if g_to == 3:
        target = engine_rpm_at(3, 10.) * 1.10
        out = [replace_rpm(s, target) if s.gear_raw == 3 and not s.clutch else s for s in out]
      feed(a, out)
    self.assertTrue(a.complete)
    status, detail = a.result()
    self.assertNotEqual(status, PASS, detail)
    self.assertIn("measured per gear", detail)

  def test_n1_says_fcw_not_exercised(self):
    a = make_analyzer(TESTS_BY_ID["N1"], ctx(NonAdaptiveLongitudinal=True))
    sim = Sim(v_ego=20., v_cruise=20., enabled=True, long_active=True)
    feed(a, sim.run(130.), ok=True)
    status, detail = a.result()
    self.assertIn("FCW not exercised", detail)

  def test_no_rule_passes_before_complete(self):
    # every rule that can PASS, fed a partial window, returns no PASS
    for spec in CIVIC.registry.tests:
      a = make_analyzer(spec, calibrated(ctx(**dict.fromkeys(WATCHED_TOGGLES, True))))
      feed(a, Sim(gear_raw=3, v_ego=15., enabled=True, long_active=True).run(0.5), ok=True)
      if not a.complete:
        self.assertNotEqual(a.result()[0], PASS, spec.id)


def replace_rpm(s, rpm):
  from dataclasses import replace
  out = replace(s, rpm=rpm)
  from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import infer_gear, measured_ratio
  out.gear, out.ratio = infer_gear(rpm, out.v_ego, out.clutch), measured_ratio(rpm, out.v_ego)
  return out
