"""
Non-adaptive (set-speed only) longitudinal: with the NonAdaptiveLongitudinal toggle ON, openpilot longitudinal
holds the set speed and does not slow for leads, while the MPC (and so FCW) keeps running. With it OFF (default),
behavior is unchanged.
"""
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, LongitudinalPlanSource
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant
from openpilot.sunnypilot.selfdrive.controls.lib.dec.tests.test_dec_planner_gate import E2E_ACCEL, MockDec, build_sm

V_CRUISE = 25.0  # m/s


def build_planner() -> LongitudinalPlanner:
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC_2022)
  planner = LongitudinalPlanner(CP, CP_SP, init_v=20.0)
  planner.dec = MockDec(active=False, mode='acc')  # DEC inactive: experimental mode alone decides e2e
  return planner


def run(non_adaptive: bool, t_end: float, speed: float, lead: bool = False, d_lead: float = 200., v_lead: float = 0.,
        prob_throttle: float = 1.0, e2e: bool = False):
  Params().put_bool("NonAdaptiveLongitudinal", non_adaptive)
  plant = Plant(lead_relevancy=lead, speed=speed, distance_lead=d_lead, e2e=e2e)
  logs = []
  while plant.current_time < t_end:
    log = plant.step(v_lead=v_lead, v_cruise=V_CRUISE, prob_throttle=prob_throttle)
    log["source"] = plant.planner.mpc.source
    logs.append(log)
  return plant, logs


class TestNonAdaptiveLongitudinal(OpenpilotTestCase):
  def test_default_off(self):
    plant, _ = run(False, 0.5, V_CRUISE)
    self.assertFalse(Params().get_bool("NonAdaptiveLongitudinal"))
    self.assertFalse(plant.planner.non_adaptive)

  def test_on_holds_set_speed_and_ignores_lead(self):
    # closing on a lead doing 10 m/s from 60 m at 25 m/s: lead-following would brake hard
    _, logs = run(True, 6.0, V_CRUISE, lead=True, d_lead=60., v_lead=10.)
    self.assertTrue(all(l["source"] == LongitudinalPlanSource.cruise for l in logs))
    self.assertGreater(min(l["acceleration"] for l in logs), -0.05, "braked for a lead in non-adaptive mode")
    self.assertAlmostEqual(logs[-1]["speed"], V_CRUISE, delta=0.3)

  def test_on_fcw_still_fires(self):
    # FCW is computed from the MPC, which still runs in non-adaptive mode
    _, logs = run(True, 6.0, V_CRUISE, lead=True, d_lead=60., v_lead=0.)
    self.assertTrue(any(l["fcw"] for l in logs), "FCW must still warn when closing on a stopped lead")

  def test_on_accelerates_to_set_speed(self):
    _, logs = run(True, 15.0, 18.0)
    self.assertGreater(max(l["acceleration"] for l in logs), 0.3)
    self.assertAlmostEqual(logs[-1]["speed"], V_CRUISE, delta=0.3)

  def test_on_ignores_model_predicted_lift(self):
    # the model predicting a lift (prob_throttle=0) caps throttle to coast in normal mode; set-speed-only must ignore it
    _, logs_off = run(False, 15.0, 18.0, prob_throttle=0.0)
    _, logs_on = run(True, 15.0, 18.0, prob_throttle=0.0)
    self.assertLess(logs_off[-1]["speed"], V_CRUISE - 2.0, "precondition: OFF mode should coast, not reach set speed")
    self.assertAlmostEqual(logs_on[-1]["speed"], V_CRUISE, delta=0.3)

  def test_on_ignores_e2e(self):
    # experimental mode with the model asking to brake hard: the e2e candidate must not be used when ON
    for non_adaptive, expect_e2e in ((False, True), (True, False)):
      with self.subTest(non_adaptive=non_adaptive):
        Params().put_bool("NonAdaptiveLongitudinal", non_adaptive)
        planner = build_planner()
        planner.update(build_sm(experimental_mode=True))
        self.assertEqual(planner.mpc.source == LongitudinalPlanSource.e2e, expect_e2e)
        if not expect_e2e:
          self.assertGreater(planner.output_a_target, E2E_ACCEL)

  def test_off_still_follows_lead(self):
    _, logs = run(False, 20.0, V_CRUISE, lead=True, d_lead=60., v_lead=10.)
    self.assertLess(min(l["acceleration"] for l in logs), -0.5, "lead-following must be unchanged with the toggle OFF")
    self.assertLess(logs[-1]["speed"], 12.0)
    self.assertGreater(min(l["distance_lead"] - l["distance"] for l in logs), 2.0)

  def test_toggle_only_latches_while_long_inactive(self):
    plant, _ = run(False, 0.5, V_CRUISE)
    Params().put_bool("NonAdaptiveLongitudinal", True)
    for _ in range(60):  # 3 s engaged: the param is re-read, but the mode must not switch mid-engagement
      plant.step(v_cruise=V_CRUISE)
    self.assertFalse(plant.planner.non_adaptive)

    plant.enabled = False
    plant.step(v_cruise=V_CRUISE)
    plant.enabled = True
    plant.step(v_cruise=V_CRUISE)
    self.assertTrue(plant.planner.non_adaptive)
