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
                    {"E0b": {**e0b_capture(), "complete": False}},                         # fit, but not complete
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

