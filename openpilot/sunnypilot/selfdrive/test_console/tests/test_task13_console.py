"""
Task 13 Part B — console verdicts (dev-test only).
  B1  C1 / NSB1 / E1 never PASS on an uncalibrated coast model: PASS needs a FIT from the E0b capture, and the sent
      ACCEL_COMMAND is part of the verdict. The plant's coast differs from the scorer's seed so the two can disagree.
  B3  errors the guard catches are visible, and a verdict whose run lost data to them never PASSes.
"""
import json
import math

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.test_console.analyzers import CAPTURED, FAIL, PASS, make_analyzer
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.registry import TESTS_BY_ID
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.results import FIT_MIN_SAMPLES, coast_fit
from openpilot.sunnypilot.selfdrive.test_console.status_io import sanitize_results
from openpilot.sunnypilot.selfdrive.test_console.tests.sim import Sim, feed
from openpilot.sunnypilot.selfdrive.test_console.tests.test_test_console import calibrated, ctx

SEED = {g: -(0.15 + 0.1 * (7 - g)) for g in range(1, 7)}   # the scorer's (uncalibrated) coast model, per gear
TRUE_SCALE = 1.2                                           # the plant coasts 20 % harder than the seed says


def e0b_capture(scale=TRUE_SCALE):
  """Run E0b through its analyzer on a plant whose coast is `scale` x the seed; return the result entry as it is saved
  and read back (results.json round trip, sanitized)."""
  c = ctx()
  a = make_analyzer(TESTS_BY_ID["E0b"], c)
  for g in range(1, 7):
    sim = Sim(gear_raw=g, v_ego=25., a_coast=SEED[g], a_ego=SEED[g] * scale, pitch=0.)
    feed(a, sim.run(10.5), ok=True)
  assert a.complete
  status, detail = a.result()
  entry = {**a.data, "status": status, "detail": detail, "complete": a.complete}
  return sanitize_results(json.loads(json.dumps({"E0b": entry})))["E0b"]


def with_results(c, results):
  c.results = results
  return c


def run_c1(c, a_ego, cmd, gear=3, sent=None):
  """C1 on the plant: the car coasts at TRUE_SCALE x seed, plus whatever service braking a_ego adds beyond that."""
  a = make_analyzer(TESTS_BY_ID["C1"], c)
  for _ in range(5):
    sim = Sim(gear_raw=gear, v_ego=15., accel_cmd=cmd, accel_sent=cmd if sent is None else sent, a_ego=0.,
              a_coast=SEED[gear])
    feed(a, sim.run(0.5, a_ego=0.), ok=True)
    feed(a, sim.run(1.5, a_ego=a_ego), ok=True)
    feed(a, sim.run(0.3), ok=False)
  assert a.complete
  return a.result()


class TestB1CoastFit(OpenpilotTestCase):
  def test_e0b_completion_fits_the_model(self):
    entry = e0b_capture()
    fit = coast_fit(with_results(ctx(), {"E0b": entry}))
    self.assertIsNotNone(fit, entry)
    for g in range(1, 7):
      self.assertAlmostEqual(fit[g], SEED[g] * (TRUE_SCALE - 1.), places=3)
    self.assertIn("fit g1", entry["detail"])

  def test_complete_without_a_fit_is_not_calibrated(self):
    for results in ({"E0b": {"complete": True, "status": "CAPTURED"}},                     # the Task 12 meaning
                    {"E0b": {"complete": False, "coast_fit": {"version": 2, "gears": {str(g): {"bias": 0., "n": 1000}
                                                                       for g in range(1, 7)}}}},   # a gear not "done"
                    {"E0b": {"complete": True, "coast_fit": {"gears": {"1": {"bias": 0., "n": 1000}}}}},  # some gears
                    {"E0b": {"complete": True, "coast_fit": {"gears": {str(g): {"bias": 0., "n": FIT_MIN_SAMPLES - 1}
                                                                       for g in range(1, 7)}}}},      # too few samples
                    {"E0b": {"complete": True, "coast_fit": "garbage"}}):
      with self.subTest(results=str(results)[:60]):
        self.assertIsNone(coast_fit(with_results(ctx(), results)))

  def test_c1_no_pass_with_seed_20pct_low_and_zero_service_braking(self):
    # the finding: seed -0.8 in 3rd, the car really coasts at -0.96, cmd -1.0, ZERO service braking (a_ego = coast).
    # Task 12's rule: (a_ego - seed) / (cmd - seed) = 0.8 -> PASS with "E0b complete"
    true_coast = SEED[3] * TRUE_SCALE
    cmd = SEED[3] - 0.2
    self.assertGreaterEqual((true_coast - SEED[3]) / (cmd - SEED[3]), 0.5, "precondition: the old rule would PASS")
    # E0b merely complete (no fit): not PASS
    status, detail = run_c1(with_results(ctx(), {"E0b": {"complete": True, "status": "CAPTURED"}}), true_coast, cmd)
    self.assertNotEqual(status, PASS, detail)
    self.assertEqual(status, CAPTURED, detail)
    # E0b actually captured on this plant and fit: still not PASS (the request is within coasting)
    status, detail = run_c1(with_results(ctx(), {"E0b": e0b_capture()}), true_coast, cmd)
    self.assertNotEqual(status, PASS, detail)
    # a request well beyond coasting, still zero service braking: FAIL against the fit
    status, detail = run_c1(with_results(ctx(), {"E0b": e0b_capture()}), true_coast, -1.6)
    self.assertEqual(status, FAIL, detail)

  def test_c1_passes_on_real_braking_with_a_fit(self):
    status, detail = run_c1(with_results(ctx(), {"E0b": e0b_capture()}), -1.6, -1.6)
    self.assertEqual(status, PASS, detail)
    self.assertIn("coast fit", detail)

  def test_c1_sent_accel_is_in_the_verdict(self):
    c = lambda: with_results(ctx(), {"E0b": e0b_capture()})  # noqa: E731
    status, detail = run_c1(c(), -1.6, -1.6, sent=0.0)        # the car "braked" but the bus never carried the request
    self.assertEqual(status, FAIL, detail)
    self.assertIn("not on the bus", detail)
    status, detail = run_c1(c(), -1.6, -1.6, sent=math.nan)   # raw tap not live: can't confirm
    self.assertEqual(status, CAPTURED, detail)

  def test_c1_unfit_gear_is_not_scored(self):
    # gear 0 (clutch in / no rpm) samples have no fit: unknown, never PASS
    status, detail = run_c1(calibrated(ctx()), -1.6, -1.6, gear=3)
    self.assertEqual(status, PASS, detail)
    c = calibrated(ctx())
    c.results["E0b"]["coast_fit"]["gears"].pop("3")
    status, detail = run_c1(c, -1.6, -1.6, gear=3)
    self.assertEqual(status, CAPTURED, detail)

  def test_nsb1_judged_against_the_fit(self):
    def run(results, a_ego, sent=True, gear=6, no_gear=False):
      c = with_results(ctx(), results)
      a = make_analyzer(TESTS_BY_ID["NSB1"], c)
      sim = Sim(gear_raw=gear, v_ego=25., a_coast=SEED[gear], a_ego=a_ego, accel_out=-0.3,
                accel_sent=-0.3 if sent else math.nan)
      sim.locked = not no_gear   # no_gear: rpm not locked to the wheels (clutch slipping / no rpm) -> inferred gear 0
      feed(a, sim.run(31., v_ego=25.), ok=True)   # downhill: held at 25 m/s over set (no stop, the gear stays known)
      c.nsb_floor = 0.0
      feed(a, sim.run(31., v_ego=25., accel_out=0.0, accel_sent=0.0 if sent else math.nan), ok=True)
      return a.result()
    coasting = SEED[6] * TRUE_SCALE
    self.assertEqual(run({"E0b": {"complete": True}}, coasting)[0], CAPTURED)
    self.assertEqual(run({"E0b": e0b_capture()}, coasting)[0], PASS)              # coasting at the true coast: clean
    self.assertEqual(run({"E0b": e0b_capture()}, coasting - 0.5)[0], FAIL)        # 0.5 m/s^2 beyond coasting: brakes
    self.assertEqual(run({"E0b": e0b_capture()}, coasting, sent=False)[0], CAPTURED)
    # samples with no inferred gear have no fitted coast estimate: unknown (NaN excess used to read as "clean" -> PASS),
    # even with braking well beyond coasting
    self.assertEqual(run({"E0b": e0b_capture()}, coasting - 0.5, no_gear=True)[0], CAPTURED)

  def test_nsb1_sent_below_floor_fails(self):
    c = with_results(ctx(), {"E0b": e0b_capture()})
    a = make_analyzer(TESTS_BY_ID["NSB1"], c)
    sim = Sim(gear_raw=6, v_ego=25., a_coast=SEED[6], a_ego=SEED[6] * TRUE_SCALE, accel_out=-0.3, accel_sent=-0.8)
    feed(a, sim.run(5.), ok=True)
    status, detail = a.result()
    self.assertEqual(status, FAIL, detail)

  def test_e1_sent_and_fit(self):
    def run(results, sent=0.0, gear=4):
      a = make_analyzer(TESTS_BY_ID["E1"], with_results(ctx(EngineBrakePreferred=True), results))
      sim = Sim(long_active=True, gear_raw=gear, a_coast=SEED[gear], accel_sent=sent)
      for _ in range(5):
        sim.events = [("engine_brake_preferred", {"active": True, "gear": gear})]
        samples = list(sim.run(2., accel_out=0.0, a_ego=SEED[gear] * TRUE_SCALE))
        sim.events = [("engine_brake_preferred", {"active": False, "reason": "idle"})]
        samples += sim.run(0.5)
        feed(a, samples)
      self.assertTrue(a.complete)
      return a.result()
    self.assertEqual(run({"E0b": {"complete": True}})[0], CAPTURED)
    self.assertEqual(run({"E0b": e0b_capture()})[0], PASS)
    status, detail = run({"E0b": e0b_capture()}, sent=-0.6)
    self.assertEqual(status, FAIL, detail)
    self.assertEqual(run({"E0b": e0b_capture()}, sent=math.nan)[0], CAPTURED)


class TestB3CaughtErrors(OpenpilotTestCase):
  """Task 13 B3 [review2 problem 3]: errors the daemon's guard catches are visible (state.json, chip, detail page), and a
  run that lost frames to them can't PASS (provisional); an inert daemon keeps saying so instead of vanishing."""

  def setUp(self):
    import tempfile
    from openpilot.sunnypilot.selfdrive.test_console.capture import Console, Store
    self.tmp = tempfile.TemporaryDirectory()
    self.console = Console(Store(self.tmp.name), ctx())

  def tearDown(self):
    self.tmp.cleanup()

  def run_s3(self, fail_frames=()):
    """S3 (rpm noise: 60 s clean in 4th -> PASS) through the daemon's guarded_step; frames in fail_frames raise."""
    from openpilot.sunnypilot.selfdrive.test_console.guard import Guard
    from openpilot.sunnypilot.selfdrive.test_console.test_consoled import guarded_step
    guard, console = Guard(), self.console
    console.set_control("S3", True, False)
    sim = Sim(gear_raw=4, v_ego=20.)
    console.update(sim.step(), True)
    for i, s in enumerate(sim.run(62.)):
      def step(s=s, i=i):
        if i in fail_frames:
          raise ValueError("injected console bug")
        console.update(s, False)
      guarded_step(guard, console, "frame", step)
    return guard

  def test_clean_run_passes(self):
    guard = self.run_s3()
    self.assertEqual(guard.errors, 0)
    self.assertEqual(self.console.state, "complete")
    self.assertEqual(self.console.store.results["S3"]["status"], PASS)

  def test_caught_errors_make_the_verdict_provisional_and_visible(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad.test_console_chip import chip_lines
    from openpilot.sunnypilot.selfdrive.test_console.test_consoled import status_for
    guard = self.run_s3(fail_frames=(100, 101, 2000, 3000, 4000))
    self.assertEqual(guard.errors, 5)
    res = self.console.store.results["S3"]
    self.assertTrue(res["complete"])
    self.assertNotEqual(res["status"], PASS, res)
    self.assertEqual(res["status"], CAPTURED)
    self.assertIn("5 frame(s) lost to console errors", res["detail"])
    self.assertEqual(res["dropped"], 5)
    st = status_for(guard, self.console, None, {})
    self.assertEqual(st["errors"], 5)
    self.assertFalse(st["inert"])
    self.assertIn("5 console errors", st["error"])
    lines, _ = chip_lines(st)
    self.assertTrue(any("5 console errors" in line for line in lines), lines)

  def test_inert_daemon_says_so(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad.test_console_chip import chip_lines
    from openpilot.sunnypilot.selfdrive.test_console.guard import Guard
    from openpilot.sunnypilot.selfdrive.test_console.test_consoled import guarded_step, status_for
    guard, console = Guard(), self.console
    console.set_control("S3", True, False)
    sim = Sim(gear_raw=4, v_ego=20.)
    console.update(sim.step(), True)

    def boom():
      raise ValueError("persistent bug")
    for _ in range(Guard.MAX_CONSECUTIVE + 20):
      guarded_step(guard, console, "frame", boom)
    self.assertTrue(guard.inert)
    self.assertEqual(console.run.dropped, Guard.MAX_CONSECUTIVE + 20, "frames keep counting as lost once inert")
    st = guard.run("status", status_for, guard, console, None, {}, force=True)
    self.assertIsNotNone(st, "the status writer still runs when inert")
    self.assertTrue(st["inert"])
    self.assertEqual(st["state"], "inert")
    lines, key = chip_lines(st)
    self.assertIn("INERT", lines[0])
    self.assertEqual(key, "FAIL")
    # the run's verdict, if it is saved (shutdown), is never a PASS
    status, detail = console.run.result()
    self.assertNotEqual(status, PASS)
    self.assertIn("lost to console errors", detail)

  def test_daemon_loop_wiring(self):
    # the loop itself needs messaging; its wiring is pinned structurally: frames/drains go through guarded_step (lost
    # frames are counted), the status writer runs even when inert, and results are still saved at shutdown
    import inspect
    from openpilot.sunnypilot.selfdrive.test_console import test_consoled
    src = inspect.getsource(test_consoled.run_console)
    self.assertIn('guarded_step(guard, console, "frame", step)', src)
    self.assertIn('guarded_step(guard, console, "drain", drain)', src)
    self.assertIn('guard.run("status", save, force=True)', src)
    self.assertIn('guard.run("shutdown", console.shutdown, force=True)', src)
    self.assertIn("status_for(guard, console", src)


class TestA2KnockOn(OpenpilotTestCase):
  def test_console_scores_against_the_clamped_margin(self):
    # Task 13 A2 on the console: NSB2 compares the trigger speed with ctx.nsb_margin, which must be the margin the car
    # uses (min(configured, 6.7, ceiling - 0.45)), not the raw param
    from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import MAX_OVERSPEED_MARGIN, read_nsb_params
    from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import set_params
    from openpilot.sunnypilot.selfdrive.test_console.test_consoled import read_context
    p = set_params(NoServiceBrakeOverspeedMargin=15., NoServiceBrakeOverspeedHardCeiling=11.)
    c = ctx()
    read_context(p, c)
    self.assertEqual(c.nsb_margin, read_nsb_params(p, True).margin)
    self.assertLessEqual(c.nsb_margin, MAX_OVERSPEED_MARGIN)
    self.assertTrue(0. <= c.nsb_release < c.nsb_trigger < c.nsb_ceiling)


class TestB4ProofTools(OpenpilotTestCase):
  def test_equiv_presses_start_a_capture(self):
    # Task 13 B4: the equivalence tool's scripted presses must fire the console's start button as the profile resolves it
    # by default (the gap button's recommended TRIPLE press since Task 12 D-10) - single presses never started anything
    import importlib.util
    from opendbc.car import structs
    from openpilot.common.basedir import BASEDIR
    from openpilot.sunnypilot.selfdrive.test_console.button import StartButton
    from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE
    spec = importlib.util.spec_from_file_location("tc_equiv", f"{BASEDIR}/tools/civic6mt/test_console_equiv.py")
    equiv = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(equiv)
    button = StartButton(None, None, buttons=PROFILE.buttons)
    self.assertEqual(button.mode, "triple", "precondition: the default gesture")
    fired = []
    for at in sorted(equiv.BUTTON):
      ev = [structs.CarState.ButtonEvent(pressed=equiv.BUTTON[at], type=structs.CarState.ButtonEvent.Type.gapAdjustCruise)]
      if button.update(100. + at, ev):
        fired.append(at)
    self.assertEqual(len(fired), 3, f"start / stop / resume, got {fired}")


class TestB4GuardStreak(OpenpilotTestCase):
  def test_persistent_failure_goes_inert_despite_other_sites_succeeding(self):
    # the isolation replay's finding: the daemon loop runs drain (succeeds) and frame (fails) every iteration
    from openpilot.sunnypilot.selfdrive.test_console.guard import Guard
    g = Guard()

    def boom():
      raise ValueError("x")
    for _ in range(Guard.MAX_CONSECUTIVE):
      g.run("drain", lambda: None)
      g.run("frame", boom)
    self.assertTrue(g.inert, "a frame step failing every frame must go inert")



class TestC1PerGearFit(OpenpilotTestCase):
  """Task 14 C1 [N13-1]: the PASS gate is the fit for the gear(s) the test's own samples used, not all six."""

  def e0b_sparse(self, sparse=5):
    """E0b on the plant: every gear's 10 s except `sparse`, which gets 2 s - E0b never completes."""
    c = ctx()
    a = make_analyzer(TESTS_BY_ID["E0b"], c)
    for g in range(1, 7):
      sim = Sim(gear_raw=g, v_ego=25., a_coast=SEED[g], a_ego=SEED[g] * TRUE_SCALE, pitch=0.)
      feed(a, sim.run(2. if g == sparse else 10.5), ok=True)
    self.assertFalse(a.complete, "precondition: E0b incomplete (one sparse gear)")
    status, detail = a.result()
    return sanitize_results(json.loads(json.dumps({"E0b": {**a.data, "status": status, "complete": a.complete}})))["E0b"]

  def test_fits_the_done_gears_only(self):
    fit = coast_fit(with_results(ctx(), {"E0b": self.e0b_sparse()}))
    self.assertEqual(sorted(fit), [1, 2, 3, 4, 6], "gear 5 (sparse) has no fit")

  def test_c1_passes_in_a_fitted_gear_while_another_is_sparse(self):
    results = {"E0b": self.e0b_sparse(5)}
    status, detail = run_c1(with_results(ctx(), results), -1.6, -1.6, gear=3)
    self.assertEqual(status, PASS, detail)   # Task 13: CAPTURED (all six required)

  def test_c1_in_the_unfit_gear_stays_captured(self):
    results = {"E0b": self.e0b_sparse(5)}
    status, detail = run_c1(with_results(ctx(), results), -1.6, -1.6, gear=5)
    self.assertEqual(status, CAPTURED, detail)
    self.assertIn("gear(s) [5]", detail)
    # and the finding's zero-service-braking case still never PASSes in a fitted gear
    status, _ = run_c1(with_results(ctx(), results), SEED[3] * TRUE_SCALE, SEED[3] - 0.2, gear=3)
    self.assertNotEqual(status, PASS)

  def test_nsb1_and_e1_per_gear(self):
    results = {"E0b": self.e0b_sparse(5)}
    for gear, want in ((6, PASS), (5, CAPTURED)):
      with self.subTest(gear=gear):
        c = with_results(ctx(), results)
        a = make_analyzer(TESTS_BY_ID["NSB1"], c)
        sim = Sim(gear_raw=gear, v_ego=25., a_coast=SEED[gear], a_ego=SEED[gear] * TRUE_SCALE, accel_out=-0.3, accel_sent=-0.3)
        feed(a, sim.run(31., v_ego=25.), ok=True)
        c.nsb_floor = 0.0
        feed(a, sim.run(31., v_ego=25., accel_out=0.0, accel_sent=0.0), ok=True)
        self.assertEqual(a.result()[0], want, a.result())
    for gear, want in ((4, PASS), (5, CAPTURED)):
      with self.subTest(e1_gear=gear):
        a = make_analyzer(TESTS_BY_ID["E1"], with_results(ctx(EngineBrakePreferred=True), results))
        sim = Sim(long_active=True, gear_raw=gear, a_coast=SEED[gear], accel_sent=0.0)
        for _ in range(5):
          sim.events = [("engine_brake_preferred", {"active": True, "gear": gear})]
          samples = list(sim.run(2., accel_out=0.0, a_ego=SEED[gear] * TRUE_SCALE))
          sim.events = [("engine_brake_preferred", {"active": False, "reason": "idle"})]
          samples += sim.run(0.5)
          feed(a, samples)
        self.assertEqual(a.result()[0], want, a.result())
