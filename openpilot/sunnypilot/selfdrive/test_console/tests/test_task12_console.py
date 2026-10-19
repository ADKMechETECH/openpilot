"""
Task 12 Part D (dev-test): the console can never disengage openpilot or crash the ui (D1), toggle writes only while
disengaged and stopped, taps and latched state are truthful (D2).
"""
import json
import math
import os
import pathlib
import re
import tempfile
from dataclasses import replace

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.test_console import sessions as S
from openpilot.sunnypilot.selfdrive.test_console.capture import Console, Store
from openpilot.sunnypilot.selfdrive.test_console.conditions import Context, check_conditions
from openpilot.sunnypilot.selfdrive.test_console.guard import Guard, guarded
from openpilot.sunnypilot.selfdrive.test_console.profile import ButtonMap, VehicleProfile
from openpilot.sunnypilot.selfdrive.test_console.profiles import PROFILES
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE as CIVIC, WATCHED_TOGGLES
from openpilot.sunnypilot.selfdrive.test_console.status_io import RESULTS_FILE, STATE_FILE, StatusPoller, prereq_hint, \
  result_line, sanitize_results, sanitize_state
from openpilot.sunnypilot.selfdrive.test_console.tests.sim import Sim

ROOT = pathlib.Path(__file__).resolve().parents[4]   # the openpilot package dir
PARAMS_KEYS = (ROOT / "common" / "params_keys.h").read_text()
NAN = float("nan")


def ctx(**toggles) -> Context:
  return Context(profile=CIVIC, manual=True, op_long=True, toggles={**dict.fromkeys(WATCHED_TOGGLES, False), **toggles})


def eventually(cond, timeout=2.0) -> bool:
  import time
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    if cond():
      return True
    time.sleep(0.01)
  return cond()


def registered(key: str) -> bool:
  return re.search(r'\{\s*"' + re.escape(key) + r'"\s*,', PARAMS_KEYS) is not None


MALFORMED_RESULTS = {"S1": "garbage", "S2": [1, 2], "S3": {"status": 5, "complete": "yes", "frac": "x", "progress": None},
                     "G2": {"status": "PASS", "complete": True}, "": None}
MALFORMED_STATE = {"selected": 3, "armed": "yes", "state": None, "conditions": "nope", "progress": 7, "frac": "x",
                   "stamp": "now", "bg": [1], "button": "x", "live": 5, "detail": {"a": 1}}


class TestD1NeverCrashes(OpenpilotTestCase):
  def test_nan_signals_through_every_test(self):
    with tempfile.TemporaryDirectory() as d:
      console = Console(Store(d), ctx())
      for spec in CIVIC.registry.tests:
        console.set_control(spec.id, True, True)
        sim = Sim(gear_raw=3, v_ego=12.)
        s = None
        for i, s in enumerate(sim.run(1.0)):
          s = replace(s, rpm=NAN, rpm_rate=NAN, ratio=NAN, pitch=NAN, a_ego=NAN, accel_cmd=NAN, v_cruise=NAN)
          console.update(s, i == 10)
        json.dumps(console.status(s, {"name": "x", "mode": "short", "seen": 0}))
        console.set_control("", False, False)

  def test_malformed_results_and_state(self):
    res = sanitize_results(MALFORMED_RESULTS)
    self.assertEqual(set(res), {"S3", "G2"})
    self.assertEqual(res["S3"], {})
    for spec in CIVIC.registry.tests:
      result_line(res.get(spec.id, {}))
      prereq_hint(spec.prereqs, res)
    reg = CIVIC.registry
    for sess in reg.sessions:
      S.session_status(reg, sess, res)
      S.tile_line(sess, S.session_status(reg, sess, res))
      for t in sess.test_ids:
        S.pill_state(t, res, sanitize_state(MALFORMED_STATE), True)
    S.calibration_progress(reg, res, sanitize_state(MALFORMED_STATE))
    S.next_session(reg, res)
    st = sanitize_state(MALFORMED_STATE)
    self.assertEqual(st["conditions"], [])
    self.assertEqual((st["bg"], st["button"], st["live"]), ({}, {}, {}))
    S.chip_text(st, True, "G2")
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad.test_console_chip import chip_lines
    chip_lines({**st, "armed": True})
    self.assertEqual(sanitize_state([1, 2]), {})
    self.assertEqual(sanitize_results("x"), {})

  def test_files_on_disk_are_sanitized(self):
    with tempfile.TemporaryDirectory() as d:
      os.environ["TEST_CONSOLE_DIR"] = d
      try:
        for name, body in ((RESULTS_FILE, MALFORMED_RESULTS), (STATE_FILE, MALFORMED_STATE)):
          with open(os.path.join(d, name), "w") as f:
            json.dump(body, f)
        poller = StatusPoller()
        poller.update(1e9)
        self.assertTrue(all(isinstance(v, dict) for v in poller.results.values()))
        poller.live(1e9)
        store = Store(d)   # the daemon loads results.json the same way
        self.assertTrue(all(isinstance(v, dict) for v in store.results.values()))
      finally:
        del os.environ["TEST_CONSOLE_DIR"]

  def test_param_errors_never_raise_out_of_ui_paths(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import ParamNotAllowed, write_param
    p = Params()
    offroad = (False, False, True)
    with self.assertRaises(ParamNotAllowed):
      write_param(p, "ExperimentalMode", True, profile=CIVIC, car_state=offroad)
    with self.assertRaises(TypeError):
      write_param(p, "NoServiceBrakeDecelFloor", "abc", profile=CIVIC, car_state=offroad)   # wrong type
    # what the UI's callbacks are wrapped in: logs once, never raises
    self.assertIsNone(guarded(lambda: write_param(p, "ExperimentalMode", True, profile=CIVIC, car_state=offroad))())
    self.assertIsNone(guarded(lambda: write_param(p, "NoServiceBrakeDecelFloor", "abc", profile=CIVIC, car_state=offroad))())

  def test_daemon_guard(self):
    g = Guard()

    def boom():
      raise ValueError("x")
    for _ in range(Guard.MAX_CONSECUTIVE - 1):
      self.assertIsNone(g.run("frame", boom))
    self.assertFalse(g.inert)
    self.assertEqual(g.run("frame", lambda: 7), 7, "a success resets the streak")
    for _ in range(Guard.MAX_CONSECUTIVE):
      g.run("frame", boom)
    self.assertTrue(g.inert, "a persistent failure goes inert instead of spinning")
    self.assertIsNone(g.run("frame", lambda: 7))

  def test_selfdrived_ignores_test_consoled(self):
    src = (ROOT / "selfdrive" / "selfdrived" / "selfdrived.py").read_text()
    m = re.search(r"self\.ignored_processes\s*=\s*\{([^}]*)\}", src)
    self.assertIsNotNone(m)
    self.assertIn("'test_consoled'", m.group(1))
    procs = (ROOT / "system" / "manager" / "process_config.py").read_text()
    self.assertIn('PythonProcess("test_consoled"', procs, "sanity: it is a managed process on this branch")

  def test_allowlists_capped_and_registered(self):
    for profile in PROFILES:
      self.assertTrue(set(profile.allowlist) <= S.CORE_MAX_ALLOW, profile.name)
      for key in S.ui_writable(profile):
        self.assertTrue(registered(key), f"{key} not in params_keys.h")
    for key in S.CORE_MAX_ALLOW | set(S.CONSOLE_PARAMS):
      self.assertTrue(registered(key), key)
    # a profile can't widen the core: a hostile allowlist gets nothing outside the cap / past the denylist
    hostile = VehicleProfile("x", "x", lambda cp: False, (), CIVIC.signals, CIVIC.powertrain, ButtonMap("gapAdjustCruise"),
                             CIVIC.registry, allowlist=("ExperimentalMode", "OpenpilotEnabledToggle", "AlphaLongitudinalEnabled",
                                                        "LongitudinalPersonality", "RecordFront", "IsMetric",
                                                        "NonAdaptiveLongitudinal"),
                             watched_toggles=())
    self.assertEqual(S.ui_writable(hostile) - set(S.CONSOLE_PARAMS), {"NonAdaptiveLongitudinal"})
    self.assertFalse(S.ui_writable(hostile) & S.CORE_DENY)


class TestD2TruthfulWrites(OpenpilotTestCase):
  def test_write_refused_while_engaged_or_rolling(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import ParamWriteRefused, write_param
    p = Params()
    p.put_bool("ClutchShiftGate", False, block=True)
    for car_state, why in (((True, True, True), "disengage first"), ((True, True, False), "disengage first"),
                           ((True, False, False), "stop the car first")):
      with self.subTest(car_state=car_state):
        with self.assertRaises(ParamWriteRefused) as cm:
          write_param(p, "ClutchShiftGate", True, profile=CIVIC, car_state=car_state)
        self.assertEqual(str(cm.exception), why)
        self.assertFalse(p.get_bool("ClutchShiftGate"), "param unchanged")
    write_param(p, "ClutchShiftGate", True, profile=CIVIC, car_state=(True, False, True))   # stopped, disengaged
    self.assertTrue(eventually(lambda: p.get_bool("ClutchShiftGate")))   # (Params.put is asynchronous here)
    write_param(p, "ClutchShiftGate", False, profile=CIVIC, car_state=(False, False, True))  # offroad
    self.assertTrue(eventually(lambda: not p.get_bool("ClutchShiftGate")))
    write_param(p, "TestConsoleBackground", True, profile=CIVIC, car_state=(True, True, False))   # console keys: always

  def test_satisfied_requirement_is_not_flipped(self):
    sess = CIVIC.registry.sessions_by_id["5"]
    states = S.toggle_states(sess, lambda k: {"ClutchShiftGate": True, "DisengageOnAccelerator": False}.get(k, True),
                             S.ui_writable(CIVIC))
    gate = next(t for t in states if t.param == "ClutchShiftGate")
    self.assertTrue(gate.ok)
    self.assertIsNone(S.next_value(gate))

  def test_dismiss_swipe_ending_on_a_switch_does_nothing(self):
    import pyray as rl
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_sessions import TAP_SLOP, _Hits
    from openpilot.system.ui.lib.application import MousePos
    fired = []
    hits = _Hits()
    switch = rl.Rectangle(100, 100, 80, 28)
    hits.add(switch, lambda: fired.append("switch"))
    hits.add(rl.Rectangle(300, 100, 80, 28), lambda: fired.append("other"))
    on = MousePos(120, 110)
    # a downward dismiss swipe that starts above and ends on the switch
    self.assertIsNone(hits.tap(MousePos(120, 20), hits.hit(MousePos(120, 20)), on))
    # pressed on the switch, dragged away and back (> slop): not a tap either
    self.assertIsNone(hits.tap(MousePos(120, 110 - 2 * TAP_SLOP), hits.hit(on), on))
    # pressed on another target, slid onto the switch
    self.assertIsNone(hits.tap(MousePos(305, 110), hits.hit(MousePos(305, 110)), on))
    # pressed just outside the switch, released on it within the slop: the press hit no target
    self.assertIsNone(hits.tap(MousePos(95, 110), hits.hit(MousePos(95, 110)), MousePos(105, 110)))
    # pressed on the switch's edge, released just off it (within the slop): not a tap on anything
    edge = MousePos(178, 110)
    self.assertIsNone(hits.tap(edge, hits.hit(edge), MousePos(186, 110)))
    # a real tap
    action = hits.tap(on, hits.hit(on), MousePos(125, 112))
    self.assertIsNotNone(action)
    action()
    self.assertEqual(fired, ["switch"])

  def test_conditions_use_published_latched_values(self):
    c = ctx(EngineBrakePreferred=True)          # the Param says ON...
    s = Sim(gear_raw=3, v_ego=15., pitch=-0.05).step()
    spec = CIVIC.registry.tests_by_id["E3"]
    s.latched["EngineBrakePreferred"] = False   # ...but the feature latched OFF (changed while engaged)
    self.assertIn(("EngineBrakePreferred ON", False), check_conditions((), spec.toggles, s, c))
    s.latched["EngineBrakePreferred"] = True
    self.assertIn(("EngineBrakePreferred ON", True), check_conditions((), spec.toggles, s, c))

  def test_e3_cannot_pass_while_ebp_latched_off(self):
    with tempfile.TemporaryDirectory() as d:
      for latched, want_pass in ((False, False), (True, True)):
        with self.subTest(latched=latched):
          console = Console(Store(os.path.join(d, str(latched))), ctx(EngineBrakePreferred=True))
          console.set_control("E3", True, False)
          sim = Sim(gear_raw=4, v_ego=20., v_cruise=20., pitch=-0.05, enabled=True, long_active=True)
          for i, s in enumerate(sim.run(40.)):
            s.latched["EngineBrakePreferred"] = latched
            console.update(s, i == 1)
          status = console.run.summary()["status"]
          self.assertEqual(status == "PASS", want_pass, console.run.summary())
          self.assertFalse(math.isnan(console.run.summary()["frac"]))


class TestD4Nits(OpenpilotTestCase):
  def test_bit_hunt_scores_are_cached(self):
    from openpilot.sunnypilot.selfdrive.test_console.analyzers import make_analyzer
    a = make_analyzer(CIVIC.registry.tests_by_id["S6"], ctx())
    calls = []
    real = a._candidates
    a._candidates = lambda: (calls.append(1), real())[1]
    sim = Sim(v_ego=0., gear_raw=3)
    raw = 0
    for i in range(300):
      raw ^= 1 << 4
      a.feed(sim.step(raw_payload=raw, gear_raw=0 if 100 <= i < 150 else 3), True)
      a.candidates()   # what status() / result() call at up to 5 Hz
    self.assertLessEqual(len(calls), 3, "re-scored only on new edges, at most every 2 s")   # D-07
    n = len(calls)
    a.finish()
    a.candidates()
    self.assertEqual(len(calls), n + 1, "a window close always re-scores")

  def test_raw_tap_goes_stale(self):
    from opendbc.can import CANPacker
    from opendbc.car import Bus
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.honda.values import CAR, DBC
    from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.tap import RawCanTap
    from openpilot.sunnypilot.selfdrive.test_console.signals import Sample
    CP = interfaces[CAR.HONDA_CIVIC_2022].get_non_essential_params(CAR.HONDA_CIVIC_2022)
    tap = RawCanTap(CP)
    packer = CANPacker(DBC[CAR.HONDA_CIVIC_2022][Bus.pt])
    gear = packer.make_can_msg("GEARBOX_ALT_2", tap.bus, {"GEAR_MT": 4})
    pt = packer.make_can_msg("POWERTRAIN_DATA", tap.bus, {"PEDAL_GAS": 37})
    acc = packer.make_can_msg("ACC_CONTROL", tap.bus, {"ACCEL_COMMAND": -0.42, "CONTROL_ON": 1})
    t = int(1e9)
    tap.update([(t, [gear, pt])], [(t, [acc])])
    s = Sample()
    tap.fill(s)
    self.assertEqual((s.gear_raw, s.gas_raw), (4, 37))
    self.assertAlmostEqual(s.accel_sent, -0.42, places=2)
    # 0.3 s of bus traffic without GEARBOX_ALT_2 / ACC_CONTROL (POWERTRAIN_DATA keeps coming)
    for k in range(1, 31):   # (a fresh packed frame each time: the parser rejects a repeated counter)
      tap.update([(t + k * 10_000_000, [packer.make_can_msg("POWERTRAIN_DATA", tap.bus, {"PEDAL_GAS": 37})])], [])
    s = Sample()
    tap.fill(s)
    self.assertEqual(s.gear_raw, -1, "D-16: a quiet GEAR_MT is unavailable, not its last value")
    self.assertTrue(math.isnan(s.accel_sent))
    self.assertEqual(s.brake_assist, -1)
    self.assertEqual(s.gas_raw, 37, "a live signal stays")

  def test_capture_dir_cap_enforced_periodically(self):
    with tempfile.TemporaryDirectory() as d:
      store = Store(d)
      prunes = []
      real = store.prune
      store.prune = lambda: (prunes.append(1), real())[1]
      console = Console(store, ctx())
      console.set_control("D1", True, False)   # its conditions never hold here: the window stays open the whole time
      sim = Sim(gear_raw=4, v_ego=20.)
      for i, s in enumerate(sim.run(150.)):
        console.update(s, i == 1)
      self.assertGreaterEqual(len(prunes), 3, "D-21: not only when a capture file opens")
