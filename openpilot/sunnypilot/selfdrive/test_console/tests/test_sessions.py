"""
Test console sessions (Task 9, dev-test build) + the 7b signature tests: the session re-cut of the registry, the pure
UI-state derivations (tile status, ready check, pills, chip, calibration card), the UI's allowlisted write path, and
S6's signature scoring / S7's window.
"""
import tempfile

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.test_console import analyzers as A
from openpilot.sunnypilot.selfdrive.test_console.analyzers import CAPTURED, make_analyzer
from openpilot.sunnypilot.selfdrive.test_console.capture import Console, Store
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE as CIVIC, WATCHED_TOGGLES
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.registry import REGISTRY, SESSIONS, SESSIONS_BY_ID, TESTS, \
  TESTS_BY_ID
from openpilot.sunnypilot.selfdrive.test_console.registry import CAL, OP_LONG_PARAM, Kind
from openpilot.sunnypilot.selfdrive.test_console.sessions import ToggleState, chip_text, default_test, next_value, pill_state, \
  ready_check, ready_text, tile_line, ui_writable
from openpilot.sunnypilot.selfdrive.test_console import sessions as _sessions
from openpilot.sunnypilot.selfdrive.test_console.status_io import CONSOLE_PARAMS
from openpilot.sunnypilot.selfdrive.test_console.tests.sim import Sim
from openpilot.sunnypilot.selfdrive.test_console.tests.test_test_console import ctx

# Task 10: the session engine takes the plan (registry) and the profile's allowlist explicitly; bound to the Civic here
UI_WRITABLE, UI_WRITABLE_TOGGLES = ui_writable(CIVIC), CIVIC.allowlist
session_tests = REGISTRY.tests_in_session


def session_status(*a):
  return _sessions.session_status(REGISTRY, *a)


def next_session(*a):
  return _sessions.next_session(REGISTRY, *a)


def calibration_progress(*a):
  return _sessions.calibration_progress(REGISTRY, *a)


def toggle_states(*a):
  return _sessions.toggle_states(*a, UI_WRITABLE)


def write_param(*a, **kw):
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import write_param as _write
  return _write(*a, profile=CIVIC, **kw)

# the brief's table (Task 9 Part A)
MAPPING = {"1": ("S1", "S6", "S7"), "2": ("S2", "S3", "S4", "S5"), "3": ("D1",), "4": ("C1", "C2"), "5": ("G1", "G2"),
           "6": ("G3", "G4"), "7": ("N1", "N2"), "8": ("NSB1", "NSB2"), "9": ("H1", "H2", "H3", "H4"),
           "10": ("E1", "E3", "E2"), "11": ("GA1", "A7"), CAL: ("E0c", "E0a", "E0b")}


def closure(sid: str) -> set[str]:
  out, todo = set(), list(SESSIONS_BY_ID[sid].prereqs)
  while todo:
    p = todo.pop()
    if p not in out:
      out.add(p)
      todo += SESSIONS_BY_ID[p].prereqs
  return out


def done(*ids, status="PASS") -> dict:
  return {t: {"status": status, "complete": True, "frac": 1.0} for t in ids}


class TestSessionTable(OpenpilotTestCase):
  def test_mapping_matches_the_plan(self):
    self.assertEqual({s.id: s.test_ids for s in SESSIONS}, MAPPING)

  def test_every_test_in_exactly_one_session(self):
    ids = [t for s in SESSIONS for t in s.test_ids]
    self.assertEqual(sorted(ids), sorted(t.id for t in TESTS), "every test exactly once")
    self.assertEqual(len(ids), len(set(ids)))
    for t in TESTS:
      self.assertIn(t.id, SESSIONS_BY_ID[t.session].test_ids, f"{t.id}.session disagrees with the table")
    self.assertEqual([x.id for x in session_tests("5")], ["G1", "G2"])

  def test_prereqs_consistent_with_test_prereqs(self):
    for s in SESSIONS:
      self.assertNotIn(s.id, closure(s.id), f"cycle at {s.id}")
      for p in s.prereqs:
        self.assertIn(p, SESSIONS_BY_ID)
      for tid in s.test_ids:
        for p in TESTS_BY_ID[tid].prereqs:
          ps = TESTS_BY_ID[p].session
          self.assertTrue(ps == s.id or ps in closure(s.id), f"{tid} <- {p}: session {ps} not before {s.id}")
    # and no session claims a prerequisite none of its tests need (prereqs stay minimal / meaningful)
    for s in SESSIONS:
      needed = {TESTS_BY_ID[p].session for t in s.test_ids for p in TESTS_BY_ID[t].prereqs} - {s.id}
      for p in s.prereqs:
        self.assertTrue(p in needed or any(p in closure(n) or p == n for n in needed), f"{s.id} -> {p} unused")

  def test_required_toggles_cover_their_tests(self):
    for s in SESSIONS:
      req = dict(s.required_toggles)
      for tid in s.test_ids:
        for k, v in TESTS_BY_ID[tid].toggles.items():
          self.assertEqual(req.get(k), v, f"session {s.id} must require {k}={v} for {tid}")
      feature_tests = [t for t in s.test_ids if TESTS_BY_ID[t].kind == Kind.FEATURE and "op_long" in TESTS_BY_ID[t].conditions]
      if feature_tests:
        self.assertEqual(req.get(OP_LONG_PARAM), True, f"session {s.id} needs openpilot long")

  def test_the_brief_toggle_sets(self):
    req = {s.id: dict(s.required_toggles) for s in SESSIONS}
    for sid in ("1", "2", CAL):
      self.assertEqual(req[sid], {})
    self.assertTrue(all(v is False for k, v in req["3"].items() if k != OP_LONG_PARAM))
    self.assertEqual(req["5"], {OP_LONG_PARAM: True, "ClutchShiftGate": True, "DisengageOnAccelerator": False})
    self.assertEqual(req["6"], req["5"])
    self.assertTrue(req["7"]["NonAdaptiveLongitudinal"])
    self.assertTrue(req["8"]["NonAdaptiveLongitudinal"] and req["8"]["NoServiceBrakeHold"])
    self.assertEqual(req["9"], req["5"], "Task 12 D-20: H3 (neutral while rolling) needs the gate, and the gate needs DOA OFF")
    self.assertTrue(req["10"]["EngineBrakePreferred"])
    self.assertTrue(req["11"]["GearAwareAccel"])
    self.assertTrue(SESSIONS_BY_ID[CAL].card and all(TESTS_BY_ID[t].kind == Kind.TELEMETRY for t in MAPPING[CAL]))
    self.assertIn("E2", SESSIONS_BY_ID["10"].test_ids)
    self.assertTrue(TESTS_BY_ID["E2"].optional)

  def test_session_toggles_settable_except_op_long(self):
    for s in SESSIONS:
      for k, _ in s.required_toggles + s.optional_toggles:
        self.assertEqual(k in UI_WRITABLE, k != OP_LONG_PARAM, k)
    self.assertNotIn(OP_LONG_PARAM, UI_WRITABLE)
    self.assertTrue(set(WATCHED_TOGGLES) <= set(UI_WRITABLE_TOGGLES), "every toggle a test checks can be set")
    self.assertTrue(set(CONSOLE_PARAMS) <= UI_WRITABLE)

  def test_names_fit(self):
    for s in SESSIONS:
      self.assertLessEqual(len(s.name), 20, s.id)


class TestReadyCheck(OpenpilotTestCase):
  def states(self, sid: str, values: dict) -> list[ToggleState]:
    return toggle_states(SESSIONS_BY_ID[sid], lambda k: values.get(k))

  def test_ready_and_set_n(self):
    st = self.states("5", {OP_LONG_PARAM: True, "ClutchShiftGate": True, "DisengageOnAccelerator": False})
    self.assertEqual(ready_check(st), (True, 0))
    self.assertEqual(ready_text(st), "ready ✓")
    st = self.states("5", {OP_LONG_PARAM: False, "ClutchShiftGate": False, "DisengageOnAccelerator": True})
    self.assertEqual(ready_check(st), (False, 3))
    self.assertEqual(ready_text(st), "set 3")
    # optional switches (A/B) never count against ready
    st = self.states("5", {OP_LONG_PARAM: True, "ClutchShiftGate": True, "ClutchReengageSmart": True})
    self.assertEqual(ready_check(st), (True, 0))
    self.assertEqual([t.param for t in st if not t.required], ["ClutchReengageSmart", "ClutchShiftGateGasReleaseHint"])

  def test_missing_param_reads_as_off(self):
    st = self.states("3", {OP_LONG_PARAM: True})  # every feature unset = OFF = what the baseline wants
    self.assertEqual(ready_check(st), (True, 0))
    st = self.states("3", {OP_LONG_PARAM: None})
    self.assertEqual(ready_check(st), (False, 1))
    self.assertFalse(next(t for t in st if t.param == OP_LONG_PARAM).settable)

  def test_tap_values(self):
    st = {t.param: t for t in self.states("8", {OP_LONG_PARAM: True, "NonAdaptiveLongitudinal": False,
                                                "NoServiceBrakeHold": True, "NoServiceBrakeOverspeedBackstop": True,
                                                "NoServiceBrakeDecelFloor": 0.3})}
    self.assertIs(next_value(st["NonAdaptiveLongitudinal"]), True)    # unmet -> the requirement
    self.assertIsNone(next_value(st["NoServiceBrakeHold"]))           # met -> nothing (Task 12 D2: never flipped off)
    self.assertEqual(next_value(st["NoServiceBrakeDecelFloor"]), 0.0)  # optional float -> next choice
    st = {t.param: t for t in self.states("8", {"NoServiceBrakeDecelFloor": 0.0})}
    self.assertEqual(next_value(st["NoServiceBrakeDecelFloor"]), 0.3)
    self.assertEqual(st["NoServiceBrakeDecelFloor"].label, "floor 0.0")
    # Task 11: the overspeed mode is a string choice (maintain <-> pulse)
    st = {t.param: t for t in self.states("8", {"NoServiceBrakeOverspeedMode": "maintain"})}
    self.assertEqual(next_value(st["NoServiceBrakeOverspeedMode"]), "pulse")
    self.assertEqual(st["NoServiceBrakeOverspeedMode"].label, "mode maintain")
    st = {t.param: t for t in self.states("8", {"NoServiceBrakeOverspeedMode": "pulse"})}
    self.assertEqual(next_value(st["NoServiceBrakeOverspeedMode"]), "maintain")


class TestStatusDerivation(OpenpilotTestCase):
  def test_tiles(self):
    res = {}
    self.assertEqual(session_status(SESSIONS_BY_ID["1"], res).state, "current")
    self.assertEqual(session_status(SESSIONS_BY_ID["2"], res).gating, ("1",))
    self.assertEqual(tile_line(SESSIONS_BY_ID["2"], session_status(SESSIONS_BY_ID["2"], res)), "after 1")
    self.assertEqual(next_session(res), "1")
    res = done("S1")                             # S6/S7 are optional: S1 alone completes session 1
    st = session_status(SESSIONS_BY_ID["1"], res)
    self.assertEqual((st.state, st.done, st.total), ("done", 1, 1))
    self.assertEqual(session_status(SESSIONS_BY_ID["2"], res).state, "current")
    self.assertEqual(next_session(res), "2")
    res |= done("S2", "S3", "S4", "S5", "D1")
    self.assertEqual(session_status(SESSIONS_BY_ID["4"], res).gating, (CAL,))
    self.assertEqual(session_status(SESSIONS_BY_ID["5"], res).state, "current")
    self.assertEqual(next_session(res), "5")
    res |= done("G1") | done("G2", status="FAIL")  # a FAIL is complete but not done
    st = session_status(SESSIONS_BY_ID["5"], res)
    self.assertEqual((st.state, st.failed), ("current", ("G2",)))
    self.assertIn("1 FAIL", tile_line(SESSIONS_BY_ID["5"], st))
    self.assertEqual(session_status(SESSIONS_BY_ID["6"], res).gating, ("5",))

  def test_calibration_card(self):
    frac, text = calibration_progress({"E0c": {"complete": True}, "E0a": {"frac": 0.5}}, {})
    self.assertAlmostEqual(frac, 0.5)
    self.assertEqual(text, "50% • background")
    self.assertIn("collecting", calibration_progress({}, {"bg": {"E0b": "3: 2/10s"}})[1])

  def test_pills_and_default(self):
    state = {"armed": True, "selected": "G2", "state": "capturing"}
    self.assertEqual(pill_state("G2", {}, state, live=True), "rec")
    self.assertEqual(pill_state("G2", {}, {**state, "state": "armed"}, live=True), "armed")
    self.assertEqual(pill_state("G2", {}, state, live=False), "not run")   # stale state.json: not live
    self.assertEqual(pill_state("G1", done("G1"), state, live=True), "PASS")
    self.assertEqual(pill_state("G1", {"G1": {"status": "FAIL", "complete": False}}, {}, live=False), "FAIL")
    self.assertEqual(pill_state("G1", {"G1": {"progress": "2/5"}}, {}, live=False), "partial")
    self.assertEqual(default_test(SESSIONS_BY_ID["5"], done("G1"), ""), "G2")
    self.assertEqual(default_test(SESSIONS_BY_ID["5"], {}, "G2"), "G2")
    self.assertEqual(default_test(SESSIONS_BY_ID["5"], {}, "N1"), "G1")

  def test_chip(self):
    st = {"armed": True, "selected": "G2", "state": "capturing", "progress": "shifts 2/6"}
    self.assertEqual(chip_text(st, True, "G2"), ("STOP • ● REC  shifts 2/6", "capturing"))
    self.assertEqual(chip_text({**st, "state": "armed"}, True, "G2")[1], "armed")
    self.assertEqual(chip_text({**st, "state": "complete", "status": "PASS"}, True, "G2")[1], "PASS")
    self.assertEqual(chip_text({**st, "state": "stopped"}, True, "G2")[1], "stopped")
    self.assertIn("G2 is armed", chip_text(st, True, "G1")[0])
    self.assertIn("ARM G1", chip_text({}, False, "G1")[0])


class FakeParams:
  def __init__(self):
    self.writes: list = []

  def put(self, k, v):
    self.writes.append((k, v))

  def put_bool(self, k, v):
    self.writes.append((k, bool(v)))


class TestUiWritePath(OpenpilotTestCase):
  def test_rejects_non_allowlisted_keys(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import ParamNotAllowed
    p = FakeParams()
    for key in (OP_LONG_PARAM, "OpenpilotEnabledToggle", "ExperimentalMode", "LongitudinalPersonality", "CarParams",
                "EngineBrakeHardFloor", "ManualLowSpeedHandbackSpeed", "ClutchShiftGateMaxCoast", "IsLdwEnabled", "",
                "clutchshiftgate", "ClutchShiftGate "):
      with self.assertRaises(ParamNotAllowed, msg=key):
        write_param(p, key, True)
    self.assertEqual(p.writes, [], "a refused key writes nothing")

  def test_allowlisted_keys_write_with_type(self):
    p = FakeParams()
    write_param(p, "ClutchShiftGate", True)
    write_param(p, "NoServiceBrakeDecelFloor", 0.0)
    write_param(p, "TestConsoleSelected", "G2")
    self.assertEqual(p.writes, [("ClutchShiftGate", True), ("NoServiceBrakeDecelFloor", 0.0), ("TestConsoleSelected", "G2")])
    for key in UI_WRITABLE:
      write_param(p, key, False)

  def test_allowlist_is_the_brief(self):
    self.assertEqual(set(UI_WRITABLE_TOGGLES), {
      "NonAdaptiveLongitudinal", "NoServiceBrakeHold", "NoServiceBrakeDecelFloor", "NoServiceBrakeOverspeedBackstop",
      "NoServiceBrakeOverspeedMargin", "ClutchShiftGate", "ClutchShiftGateGasReleaseHint", "ClutchReengageSmart",
      "EngineBrakePreferred", "GearAwareAccel", "DisengageOnAccelerator",
      "NoServiceBrakeOverspeedMode"})  # + Task 11 (maintain / pulse A/B in session 8)
    self.assertEqual(UI_WRITABLE - set(UI_WRITABLE_TOGGLES), set(CONSOLE_PARAMS))
    self.assertTrue(all(k.startswith("TestConsole") for k in CONSOLE_PARAMS))


class TestSignatureHunts(OpenpilotTestCase):
  def signature(self, a, n: int, bit: int, quick=0.4, slow=2.0, pause=5.0, gear_raw=0):
    """Parked in neutral (GEAR_MT 0 throughout): the 7b rhythm on one raw bit, plus a counter bit chattering."""
    sim, raw = Sim(v_ego=0., gear_raw=gear_raw, clutch=True), 0
    counter = 1 << 2

    def hold(seconds):
      nonlocal raw
      for _ in range(round(seconds / 0.01)):
        raw ^= counter
        a.feed(sim.step(raw_payload=raw), True)

    hold(pause)
    for gap in (quick, slow, quick):
      for _ in range(n):
        raw |= bit
        hold(0.2)
        raw &= ~bit
        hold(gap)
      hold(pause)
    hold(15.)  # the driver lets the window run out (the signature itself is ~30 s for N=3)

  def test_s6_finds_the_clutch_bit_by_signature(self):
    a = make_analyzer(TESTS_BY_ID["S6"], ctx())
    bit = 1 << (63 - 35)
    self.signature(a, 3, bit)
    self.assertTrue(a.complete, a.progress())   # parked: no GEAR_MT presses, completes on the window
    self.assertEqual(a.signature_candidates(), [(A.BitHuntAnalyzer.dbc_bit(63 - 35), 3)])
    status, detail = a.result()
    self.assertEqual(status, CAPTURED)
    self.assertIn(f"{A.BitHuntAnalyzer.dbc_bit(63 - 35)} N=3", detail)

  def test_signature_counts(self):
    def ts(n, gaps):
      out, t = [], 0.
      for g in gaps:
        for _ in range(n):
          out += [t, t + 0.2]
          t += 0.2 + g
        t += 5.
      return out
    sc = A.BitHuntAnalyzer.signature_count
    self.assertEqual(sc(ts(3, (0.4, 2.0, 0.4))), 3)
    self.assertEqual(sc(ts(4, (0.4, 2.0, 0.4))), 4)
    self.assertEqual(sc(ts(5, (0.4, 2.0, 0.4))), 5)
    self.assertEqual(sc(ts(3, (0.4, 2.0))), 0)                 # two groups
    self.assertEqual(sc([0.01 * i for i in range(600)]), 0)   # a counter
    self.assertEqual(sc(ts(3, (0.4, 2.0, 0.4))[:-1]), 0)       # a group with an odd toggle count

  def test_s7_window(self):
    a = make_analyzer(TESTS_BY_ID["S7"], ctx())
    self.assertEqual(a.result(), (None, ""))
    sim = Sim(v_ego=0.)
    for _ in range(9100):
      a.feed(sim.step(), True)
    self.assertTrue(a.complete)
    status, detail = a.result()
    self.assertEqual(status, CAPTURED)
    self.assertIn("scrub every logged bus", detail)

  def test_signature_tests_never_run_in_background(self):
    self.assertFalse(TESTS_BY_ID["S6"].background or TESTS_BY_ID["S7"].background)
    self.assertTrue(all(t.background for t in TESTS if t.id not in ("S6", "S7")))
    with tempfile.TemporaryDirectory() as root:
      console = Console(Store(root), ctx())
      console.set_control("", False, True)
      sim = Sim(v_ego=0., gear_raw=0, clutch=True)
      for s in sim.run(2.):
        console.update(s, False)
      self.assertNotIn("S6", console.bg_runs)
      self.assertNotIn("S7", console.bg_runs)
      self.assertIn("S1", console.bg_runs)
      console.shutdown()
