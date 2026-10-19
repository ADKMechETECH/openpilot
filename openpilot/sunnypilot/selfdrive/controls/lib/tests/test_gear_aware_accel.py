"""
GearAwareAccel: gear-indexed scaling of the planner's set-speed jerk limit. Shaping logic only (feel is car work).
"""
import numpy as np

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel, A_CRUISE_MAX_BP, J_CRUISE_VALS
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import engine_rpm_at
from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import GearAwareAccel, JERK_SCALE_BY_GEAR, STOCK
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_manual_low_speed_handback import make_cp, TransmissionType
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_engine_brake_preferred import ManualSim, setup as ebp_setup


def cs(v=15., gear=2, clutch=False):
  CS = messaging.new_message('carState').carState
  CS.vEgo = v
  CS.deprecated.engineRpm = float(engine_rpm_at(gear, v)) if gear else 800.
  CS.deprecated.clutchPressed = clutch
  return CS


def make(on=True, **kw):
  Params().put_bool("GearAwareAccel", on)
  CP, _ = make_cp(**kw)
  return GearAwareAccel(CP, Params())


class TestGearAwareAccel(OpenpilotTestCase):
  def test_table(self):
    self.assertEqual(len(JERK_SCALE_BY_GEAR), 6)
    self.assertTrue(all(0. < s <= 1.0 for s in JERK_SCALE_BY_GEAR), "never looser than stock")
    self.assertEqual(list(JERK_SCALE_BY_GEAR), sorted(JERK_SCALE_BY_GEAR), "low gears tighter")
    for g in (0, 7, 14, -1):
      self.assertEqual(GearAwareAccel.jerk_scale(g), 1.0)

  def test_off_and_unavailable_are_stock(self):
    for factory in (lambda: make(on=False), lambda: make(transmission=TransmissionType.automatic),
                    lambda: make(op_long=False)):
      gaa = factory()  # (the param stays as each case set it while that case runs)
      for g in range(7):
        self.assertEqual(gaa.update(True, cs(gear=g), 0., None, 0.5, 1.0), STOCK)
        self.assertEqual(gaa.update(False, cs(gear=g), 0., None, 0.5, -0.2), STOCK)
    Params().remove("GearAwareAccel")
    self.assertFalse(Params().get_bool("GearAwareAccel"))

  def test_scales_per_gear(self):
    gaa = make()
    for g in range(1, 7):
      v = 15. if g > 1 else 8.
      rise, fall = gaa.update(False, cs(v=v, gear=g), 0., None, 0.5, 0.0)  # lift-off within coast span
      self.assertEqual(rise, JERK_SCALE_BY_GEAR[g - 1])
      self.assertEqual(fall, JERK_SCALE_BY_GEAR[g - 1])
    for kw in ({'clutch': True}, {'gear': 0}):  # clutch in / neutral: unknown gear -> stock
      self.assertEqual(gaa.update(False, cs(**kw), 0., None, 0.5, 0.0), STOCK)

  def test_falling_stock_when_braking_needed_or_urgent(self):
    gaa = make()
    s = JERK_SCALE_BY_GEAR[1]
    self.assertEqual(gaa.update(False, cs(), 0., None, 0.5, -1.2), (s, 1.0), "target needs the brakes")
    # Rule 5 (Task 12): any braking urgency -> fully stock, rise and fall
    for urgency in ("fcw", "lead_closing", "force_decel", "stock_aeb", "invalid_plan"):
      self.assertEqual(gaa.update(False, cs(), 0., urgency, 0.5, 0.0), STOCK, urgency)
    self.assertEqual(gaa.update(False, cs(), None, None, 0.5, 0.0), STOCK, "no pitch")
    self.assertEqual(gaa.update(False, cs(), 0., None, 0.5, 0.0, clutch_valid=False), STOCK, "no clutch signal")
    self.assertEqual(gaa.update(False, cs(), 0., None, -1.0, -1.0), (s, 1.0), "already below engine braking")
    self.assertEqual(gaa.update(False, cs(), -0.08, None, 0.5, -0.2), (s, 1.0), "downhill: no coast span")
    self.assertEqual(gaa.update(False, cs(), 0., None, 0.5, -0.2), (s, s), "gentle lift-off: blended")

  def test_get_cruise_accel_scaling(self):
    CP, _ = make_cp()
    args = {'e2e': False, 'v_cruise': 20., 'v_ego': 10., 'angle_steers': 0., 'CP': CP, 'dt': DT_MDL, 'accel_coast': 1.0,
                'allow_throttle': True}
    j = np.interp(10., A_CRUISE_MAX_BP, J_CRUISE_VALS)
    self.assertEqual(get_cruise_accel(a_cruise_prev=0., **args), get_cruise_accel(a_cruise_prev=0., j_scale=(1., 1.), **args))
    self.assertAlmostEqual(get_cruise_accel(a_cruise_prev=0., j_scale=(0.5, 1.), **args), j * 0.5 * DT_MDL)
    args["v_cruise"] = 5.
    self.assertAlmostEqual(get_cruise_accel(a_cruise_prev=0.5, j_scale=(1., 0.65), **args), 0.5 - j * 0.65 * DT_MDL)
    self.assertAlmostEqual(get_cruise_accel(a_cruise_prev=0.5, j_scale=(0.65, 1.), **args), 0.5 - j * DT_MDL)

  def test_latch_only_while_long_inactive(self):
    gaa = make(on=False)
    Params().put_bool("GearAwareAccel", True)
    for _ in range(int(2. / DT_MDL)):   # engaged (the 1 Hz param read happens in here): no switch mid-engagement
      self.assertEqual(gaa.update(True, cs(), 0., None, 0.5, 1.0), STOCK)
    gaa.update(False, cs(), 0., None, 0.5, 1.0)   # disengaged: latches
    self.assertNotEqual(gaa.update(True, cs(), 0., None, 0.5, 1.0), STOCK)

  def test_closed_loop_throttle_on_softer_in_low_gear(self):
    # planner in the loop: resuming toward set speed in 2nd ramps a_cruise slower with the toggle on; OFF is stock
    runs = {}
    for on in (False, True):
      ebp_setup(non_adaptive=True)
      Params().put_bool("GearAwareAccel", on)
      sim = ManualSim(speed=8., v_set=12., gear=2)
      a = []
      for _ in range(int(3. / DT_MDL)):
        sim.run(DT_MDL)
        a.append(sim.planner.a_cruise)
      runs[on] = np.diff(a) / DT_MDL
      if not on:
        self.assertEqual(sim.planner.gear_aware_accel.scales, STOCK)
    j = np.interp(8., A_CRUISE_MAX_BP, J_CRUISE_VALS)
    self.assertLessEqual(max(runs[True]), j * JERK_SCALE_BY_GEAR[1] * 1.01)  # (speed dips just under 8 m/s at first)
    self.assertGreater(max(runs[False]), j * JERK_SCALE_BY_GEAR[1] + 0.1)
    Params().put_bool("GearAwareAccel", False)

  def test_mpc_lead_braking_untouched(self):
    # lead-following braking comes from the MPC candidate, which GearAwareAccel never scales
    logs = {}
    for on in (False, True):
      ebp_setup()
      Params().put_bool("GearAwareAccel", on)
      lead = {"gap": 40., "v": 20., "profile": [(1., 3.0)]}
      logs[on] = [l["cmd"] for l in ManualSim(speed=20., v_set=25., gear=3, lead=lead).run(6.) if l["cmd"] < -0.3]
    self.assertTrue(logs[False])
    self.assertEqual(logs[False][:50], logs[True][:50])
    Params().put_bool("GearAwareAccel", False)

