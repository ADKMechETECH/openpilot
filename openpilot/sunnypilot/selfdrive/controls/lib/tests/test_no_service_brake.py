"""
No-service-brake ("coast over set speed") mode, closed loop: LongitudinalPlanner -> controlsd's brake floor ->
LongControl -> a simple vehicle on a downhill (constant +accel road load), so without braking the car speeds up.
"""
import numpy as np

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import apply_brake_floor, read_decel_floor, \
                                                                         DEFAULT_OVERSPEED_MARGIN, BACKSTOP_RELEASE_RATIO

V_SET = 25.0      # m/s
DOWNHILL = 0.6    # m/s^2 of road load pushing the car forward (~6% grade, no gas)


class DownhillSim:
  def __init__(self, op_long=True, speed=V_SET, downhill=DOWNHILL, lead=None, force_decel=False):
    from opendbc.car.honda.interface import CarInterface
    from opendbc.car.honda.values import CAR
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP.openpilotLongitudinalControl = op_long
    CP.pcmCruise = not op_long
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    self.planner = LongitudinalPlanner(CP, CP_SP, init_v=speed)
    self.loc = LongControl(CP, CP_SP)
    self.params = Params()
    self.speed, self.a_ego, self.downhill, self.force_decel = speed, 0., downhill, force_decel
    self.distance, self.lead = 0., lead  # lead: (initial gap m, lead speed m/s)
    self.lead_pos = lead[0] if lead else 0.
    self.log = []

  def _sm(self):
    radar = messaging.new_message('radarState')
    if self.lead:
      lead = log.RadarState.LeadData.new_message()
      lead.dRel = float(max(self.lead_pos - self.distance, 0.))
      lead.vLead = lead.vLeadK = float(self.lead[1])
      lead.vRel = float(self.lead[1] - self.speed)
      lead.aLeadTau = float(_LEAD_ACCEL_TAU)
      lead.present, lead.modelProb, lead.radar = True, 1.0, True
      radar.radarState.leadOne = lead
      radar.radarState.leadTwo = lead
    control = messaging.new_message('controlsState')
    control.controlsState.longControlState = LongCtrlState.pid
    control.controlsState.forceDecel = self.force_decel
    car_state = messaging.new_message('carState')
    car_state.carState.vEgo, car_state.carState.aEgo = float(self.speed), float(self.a_ego)
    car_state.carState.vCruise = V_SET * 3.6
    model = messaging.new_message('modelV2')
    model.modelV2.position.x = [float(x) for x in (self.speed + 0.5) * np.array(ModelConstants.T_IDXS)]
    model.modelV2.velocity.x = [float(self.speed)] * len(ModelConstants.T_IDXS)
    model.modelV2.acceleration.x = [0.] * len(ModelConstants.T_IDXS)
    model.modelV2.meta.disengagePredictions.gasPressProbs = [1.] * 6
    sm = {'radarState': radar.radarState, 'carState': car_state.carState, 'controlsState': control.controlsState}
    for s in ('carControl', 'selfdriveState', 'vehicleParameters', 'carStateSP', 'liveMapDataSP', 'gpsLocation'):
      sm[s] = getattr(messaging.new_message(s), s)
    sm['modelV2'] = model.modelV2
    return sm

  def run(self, seconds):
    floor = read_decel_floor(self.params)
    enabled = self.params.get_bool("NoServiceBrakeHold")
    for _ in range(round(seconds / DT_MDL)):
      self.planner.update(self._sm())
      limits = apply_brake_floor((ACCEL_MIN, ACCEL_MAX), self.planner.allow_brake, True, enabled, floor)
      for _ in range(round(DT_MDL / DT_CTRL)):
        CS = messaging.new_message('carState').carState
        CS.vEgo, CS.aEgo = float(self.speed), float(self.a_ego)
        cmd = float(self.loc.update(True, CS, self.planner.output_a_target, self.planner.output_should_stop, limits))
        self.a_ego = cmd + self.downhill
        self.speed = max(self.speed + self.a_ego * DT_CTRL, 0.)
        self.distance += self.speed * DT_CTRL
        if self.lead:
          self.lead_pos += self.lead[1] * DT_CTRL
        self.log.append({"cmd": cmd, "speed": self.speed, "allow_brake": self.planner.allow_brake,
                         "fcw": self.planner.fcw, "lower_limit": limits[0]})
    return self.log


def setup(non_adaptive, no_service_brake, floor=None, backstop=None):
  p = Params()
  p.put_bool("NonAdaptiveLongitudinal", non_adaptive)
  p.put_bool("NoServiceBrakeHold", no_service_brake)
  if floor is not None:
    p.put("NoServiceBrakeDecelFloor", floor)
  if backstop is not None:
    p.put_bool("NoServiceBrakeOverspeedBackstop", backstop)


class TestNoServiceBrake(OpenpilotTestCase):
  def test_defaults(self):
    p = Params()
    self.assertFalse(p.get_bool("NoServiceBrakeHold"))
    self.assertTrue(p.get("NoServiceBrakeOverspeedBackstop", return_default=True), "backstop must default ON")
    sim = DownhillSim()
    self.assertTrue(sim.planner.no_service_brake.backstop_enabled)
    self.assertAlmostEqual(read_decel_floor(p), 0.3)

  def test_off_brakes_to_hold_set_speed(self):
    setup(non_adaptive=True, no_service_brake=False)
    log = DownhillSim().run(20.)
    self.assertLess(min(l["cmd"] for l in log), -0.4, "should brake against the downhill")
    self.assertLess(max(l["speed"] for l in log), V_SET + 1.0)
    self.assertTrue(all(l["allow_brake"] for l in log))

  def test_on_throttle_only_drifts_over_set_speed(self):
    # floor 0.0 = pure throttle-only: zero service brake commanded, the car coasts down the hill over set speed
    setup(non_adaptive=True, no_service_brake=True, floor=0.0, backstop=False)
    log = DownhillSim().run(15.)
    self.assertGreaterEqual(min(l["cmd"] for l in log), 0.0, "no negative (brake) command may be sent")
    self.assertGreater(log[-1]["speed"], V_SET + 3.0)

  def test_on_default_floor_limits_decel(self):
    setup(non_adaptive=True, no_service_brake=True, backstop=False)
    log = DownhillSim().run(15.)
    self.assertGreaterEqual(min(l["cmd"] for l in log), -0.3 - 1e-6)
    self.assertGreater(log[-1]["speed"], V_SET + 1.5)

  def test_clamp_inert_without_non_adaptive(self):
    # safety coupling: never clamp braking while lead-following could need it
    setup(non_adaptive=False, no_service_brake=True, floor=0.0, backstop=False)
    log = DownhillSim().run(20.)
    self.assertTrue(all(l["allow_brake"] for l in log))
    self.assertLess(min(l["cmd"] for l in log), -0.4)
    self.assertLess(max(l["speed"] for l in log), V_SET + 1.0)

  def test_clamp_inert_without_non_adaptive_brakes_for_lead(self):
    setup(non_adaptive=False, no_service_brake=True, floor=0.0, backstop=False)
    log = DownhillSim(lead=(60., 10.)).run(15.)
    self.assertLess(min(l["cmd"] for l in log), -1.0, "must still brake for a slower lead")

  def test_fcw_still_asserts(self):
    setup(non_adaptive=True, no_service_brake=True, floor=0.0, backstop=False)
    log = DownhillSim(lead=(70., 0.)).run(6.)
    self.assertTrue(any(l["fcw"] for l in log))

  def test_overspeed_backstop_reengages_braking(self):
    setup(non_adaptive=True, no_service_brake=True, floor=0.0, backstop=True)
    log = DownhillSim().run(60.)
    self.assertTrue(any(not l["allow_brake"] for l in log))
    self.assertTrue(any(l["allow_brake"] for l in log[100:]), "backstop must re-allow braking past the margin")
    self.assertLess(min(l["cmd"] for l in log), -0.3)
    self.assertLess(max(l["speed"] for l in log), V_SET + DEFAULT_OVERSPEED_MARGIN + 1.0, "long downhill can't run away")
    self.assertGreater(max(l["speed"] for l in log), V_SET + DEFAULT_OVERSPEED_MARGIN * BACKSTOP_RELEASE_RATIO,
                       "should still coast over set speed before the backstop")

  def test_force_decel_still_brakes(self):
    # driver-monitoring forceDecel must never be clamped
    setup(non_adaptive=True, no_service_brake=True, floor=0.0, backstop=False)
    log = DownhillSim(force_decel=True).run(10.)
    self.assertTrue(all(l["allow_brake"] for l in log))
    self.assertLess(min(l["cmd"] for l in log), -0.5)

  def test_pcm_cruise_no_op(self):
    # stock ACC: the planner doesn't own longitudinal; the feature must no-op without errors
    setup(non_adaptive=True, no_service_brake=True, floor=0.0)
    sim = DownhillSim(op_long=False)
    log = sim.run(5.)
    self.assertFalse(sim.planner.no_service_brake.available)
    self.assertTrue(all(l["allow_brake"] for l in log))

  def test_invalid_plan_never_clamps(self):
    # capnp's default allowBrake is False: a missing/invalid plan must fail toward allowing braking
    self.assertEqual(apply_brake_floor((-3.5, 2.0), False, False, True, 0.0), (-3.5, 2.0))
    self.assertEqual(apply_brake_floor((-3.5, 2.0), False, True, False, 0.0), (-3.5, 2.0))
    self.assertEqual(apply_brake_floor((-3.5, 2.0), True, True, True, 0.0), (-3.5, 2.0))
    self.assertEqual(apply_brake_floor((-3.5, 2.0), False, True, True, 0.3), (-0.3, 2.0))

  def test_toggle_latches_only_while_long_inactive(self):
    setup(non_adaptive=True, no_service_brake=False)
    sim = DownhillSim()
    sim.run(1.)
    Params().put_bool("NoServiceBrakeHold", True)
    sim.run(3.)
    self.assertFalse(sim.planner.no_service_brake.enabled, "must not switch on mid-engagement")
