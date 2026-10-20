"""Task 16 (Stage 2 on-car finding): after a gas override, a manual coasts first instead of resuming the driver's accel.

Stage 2 (route 00000004--9b8c913b70, t=137.7 s): lifted at +1.8 m/s^2, 3.3 mph over set; the planner had re-seeded from
aEgo, so openpilot requested +1.6 / +1.3 m/s^2 after the lift and the PCM re-opened the throttle - ~1.5 s of
re-acceleration to 5.5 mph over set. The fix seeds min(aEgo, 0) on a manual's override exit; automatics stay stock.
"""
from opendbc.car.structs import car

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import ManualCarSim

TransmissionType = car.CarParams.TransmissionType
SET = 13.   # m/s (~29 mph, the Stage 2 set speed)


def override_then_lift(transmission=TransmissionType.manual, v_set=SET, speed=SET, gas_s=1.5):
  """Engaged at set speed, the driver accelerates with the gas (the sim's open plant gives ~+1 m/s^2), then lifts."""
  sim = ManualCarSim(speed=speed, gear=3, v_set=v_set, transmission=transmission, ClutchShiftGate=False)
  sim.run(2.)
  sim.run(gas_s, gas=True)
  n0 = len(sim.log)
  a_at_lift, v_at_lift = sim.a_ego, sim.speed
  sim.run(3., gas=False)
  return sim, sim.log[n0:], a_at_lift, v_at_lift


class TestGasOverrideSeed(OpenpilotTestCase):
  def test_manual_lift_over_set_never_requests_throttle(self):
    sim, after, a_lift, v_lift = override_then_lift()
    self.assertGreater(a_lift, 0.8, "precondition: the driver was accelerating at the lift")
    self.assertGreater(v_lift, SET + 0.5, "precondition: above the set speed at the lift")
    over = [r for r in after if r["v"] > SET]   # until the car is back down to set (it may then dip and recover)
    self.assertGreater(len(over), 50, "precondition: it takes a while to come back down")
    self.assertLessEqual(max(r["a_target"] for r in over), 1e-6, "no throttle requested while above set after the lift")
    self.assertLessEqual(max(r["sent"] for r in over), 1e-6, "nor sent")
    self.assertLessEqual(max(r["v"] for r in after), v_lift + 0.1, "no re-surge: the car does not speed up again")

  def test_manual_seed_is_zero_or_negative(self):
    # planner level: the first plan after the lift starts from <= 0, never from the driver's +1 m/s^2
    sim = ManualCarSim(speed=SET, gear=3, v_set=SET, ClutchShiftGate=False)
    sim.run(2.)
    sim.run(1.5, gas=True)
    self.assertTrue(sim.planner.gas_override_reset, "the reset is flagged as a gas override")
    self.assertLessEqual(sim.planner.output_a_target, 0., "seeded at min(aEgo, 0) while overriding")
    sim.run(0.05, gas=False)
    self.assertLessEqual(sim.plan.aTarget, 0., "first plan after the lift")

  def test_manual_lift_below_set_coasts_then_resumes(self):
    # lifting below the set speed: coast first, then the plan accelerates back toward set as usual (not stuck at 0)
    sim, after, _, v_lift = override_then_lift(v_set=20., speed=12.)
    self.assertLess(v_lift, 20.)
    # coast first: the plan starts from 0 and moves at its own jerk limit (one plan step ~+0.05), not from the driver's
    # accel at the lift (automatic: the first request is ~ that accel)
    self.assertLessEqual(after[0]["a_target"], 0.1, "coast first")
    self.assertGreater(max(r["a_target"] for r in after), 0.3, "then the plan resumes accelerating toward set")

  def test_flag_clears_once_longitudinal_is_back(self):
    sim, _, _, _ = override_then_lift()
    self.assertFalse(sim.planner.gas_override_reset, "only the override's own reset is affected")

  def test_automatic_unchanged(self):
    # automatics keep the stock seed (aEgo): the first plan after the lift is still the driver's positive accel
    sim, after, a_lift, _ = override_then_lift(TransmissionType.automatic)
    self.assertFalse(sim.planner.manual)
    self.assertGreater(after[0]["a_target"], 0.5, f"stock: resumes from the driver's accel ({a_lift:+.2f})")
