"""
Manual low-speed handback, closed loop. Each frame mirrors the two processes:
  selfdrived: CarSpecificEventsSP (handback events) -> StateMachine -> AlertManager
  controlsd:  ControlsExt.update_low_speed_handback -> longActive -> LowSpeedHandback.limit_accel -> LongControl
with LongitudinalPlanner (20 Hz) and a simple vehicle. Default params: handback at 2.24 m/s, 0.5 s ramp.
"""
from types import SimpleNamespace

import numpy as np

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.cereal import custom, log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import apply_brake_floor, read_decel_floor
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import LowSpeedHandback, \
  DEFAULT_HANDBACK_SPEED, DEFAULT_RAMP_TIME, DISENGAGE_MARGIN, MIN_HANDBACK_SPEED, ENGAGE_HYSTERESIS

TransmissionType = structs.CarParams.TransmissionType
State = log.SelfdriveState.OpenpilotState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
HANDBACK_ALERT = "manualLowSpeedHandback/immediateDisable"


def make_cp(transmission=TransmissionType.manual, op_long=True):
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  fingerprint = gen_empty_fingerprint()  # no gearbox message -> manual
  if transmission != TransmissionType.manual:
    fingerprint[0][0x1A3] = 8  # GEARBOX_AUTO
  CP = CarInterface.get_params(CAR.HONDA_CIVIC_2022, fingerprint, [], op_long, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.HONDA_CIVIC_2022, fingerprint, [], op_long, False, False)
  assert CP.transmissionType == transmission and CP.openpilotLongitudinalControl == op_long
  return CP, CP_SP


class _Ext(ControlsExt):
  """ControlsExt with only the handback piece (the real __init__ blocks on CarParamsSP and loads models)."""
  def __init__(self, CP, params):  # noqa: super-init-not-called
    self.low_speed_handback = LowSpeedHandback(CP, params, process="controlsd")


class Sim:
  def __init__(self, CP, CP_SP, speed, lead=None, grade=0., v_set=20.):
    self.CP = CP
    self.planner = LongitudinalPlanner(CP, CP_SP, init_v=speed)
    self.loc = LongControl(CP, CP_SP)
    params = Params()
    self.ext = _Ext(CP, params)
    self.car_events_sp = CarSpecificEventsSP(CP, CP_SP, params)
    self.state = StateMachine()
    self.state.state = State.enabled
    self.enabled = True
    self.AM = AlertManager()
    self.speed, self.a_ego, self.distance, self.grade, self.v_set = speed, 0., 0., grade, v_set
    self.lead = lead  # dict(pos, v, decel, stop_after) or None
    self.driver_brake = 0.  # m/s^2 the driver applies once openpilot hands back
    self.brake_pedal = False  # driver on the brake pedal (Honda: disengages, like selfdrived's pedalPressed)
    self.nsb_enabled = params.get_bool("NoServiceBrakeHold")
    self.nsb_floor = read_decel_floor(params)
    self.enable_request = False
    self.frame = 0
    self.long_state = LongCtrlState.pid
    self.long_active = True
    self.plan = None

  def _lead_msg(self):
    lead = log.RadarState.LeadData.new_message()
    if self.lead:
      lead.dRel = float(max(self.lead["pos"] - self.distance, 0.))
      lead.vLead = lead.vLeadK = float(self.lead["v"])
      lead.aLeadK = float(-self.lead["decel"] if self.lead["v"] > 0 else 0.)
      lead.vRel = float(self.lead["v"] - self.speed)
      lead.aLeadTau = float(_LEAD_ACCEL_TAU)
      lead.present = True
      lead.modelProb, lead.radar = 1.0, True
    return lead

  def _car_state(self):
    CS = messaging.new_message('carState').carState
    CS.vEgo, CS.aEgo = float(self.speed), float(self.a_ego)
    CS.standstill = self.speed < 1e-3
    CS.vCruise = self.v_set * 3.6
    CS.cruiseState.available = True
    CS.brakePressed = self.brake_pedal
    if self.enable_request:
      CS.buttonEvents = [structs.CarState.ButtonEvent(pressed=False, type=structs.CarState.ButtonEvent.Type.setCruise)]
    return CS

  def _plan(self, CS):
    radar = messaging.new_message('radarState')
    radar.radarState.leadOne = radar.radarState.leadTwo = self._lead_msg()
    control = messaging.new_message('controlsState')
    control.controlsState.longControlState = self.long_state
    model = messaging.new_message('modelV2')
    model.modelV2.position.x = [float(x) for x in (self.speed + 0.5) * np.array(ModelConstants.T_IDXS)]
    model.modelV2.velocity.x = [float(self.speed)] * len(ModelConstants.T_IDXS)
    model.modelV2.acceleration.x = [0.] * len(ModelConstants.T_IDXS)
    model.modelV2.orientationRate.z = [0.] * len(ModelConstants.T_IDXS)
    model.modelV2.meta.disengagePredictions.gasPressProbs = [1.] * 6
    sm = {'radarState': radar.radarState, 'carState': CS, 'controlsState': control.controlsState, 'modelV2': model.modelV2}
    for s in ('carControl', 'selfdriveState', 'vehicleParameters', 'carStateSP', 'liveMapDataSP', 'gpsLocation'):
      sm[s] = getattr(messaging.new_message(s), s)
    sm['carControl'].enabled = self.enabled
    sm['carControl'].longActive = self.long_active
    self.planner.update(sm)
    return SimpleNamespace(aTarget=float(self.planner.output_a_target), shouldStop=bool(self.planner.output_should_stop),
                           allowBrake=bool(self.planner.allow_brake), fcw=bool(self.planner.fcw))

  def step(self):
    CS = self._car_state()
    # selfdrived: events (enabled from the previous step) -> state machine -> alerts
    events = Events()
    if self.enable_request:
      events.add(EventName.buttonEnable)
    if self.brake_pedal and not CS.standstill:
      events.add(EventName.pedalPressed)
    events_sp = self.car_events_sp.update(CS, events, self.enabled)
    self.enabled, _ = self.state.update(events)
    callback_args = [self.CP, CS, None, False, 0, None]
    self.AM.add_many(self.frame, events.create_alerts(self.state.current_alert_types, callback_args) +
                     events_sp.create_alerts(self.state.current_alert_types, callback_args))
    self.AM.process_alerts(self.frame, set())

    # controlsd
    if self.plan is None or self.frame % round(DT_MDL / DT_CTRL) == 0:
      self.plan = self._plan(CS)
    handback_done = self.ext.update_low_speed_handback(self.enabled, CS.vEgo, self.loc.last_output_accel)
    self.long_active = self.enabled and not handback_done
    if not self.long_active:
      self.loc.reset()
    # same order as controlsd: no-service-brake floor, then the handback envelope
    limits = apply_brake_floor((ACCEL_MIN, ACCEL_MAX), self.plan.allowBrake, True, self.nsb_enabled, self.nsb_floor)
    limits = self.ext.low_speed_handback.limit_accel(limits)
    cmd = float(self.loc.update(self.long_active, CS, self.plan.aTarget, self.plan.shouldStop, limits))
    self.long_state = self.loc.long_control_state

    # vehicle: openpilot's command while active, else the driver's own braking; brakes hold a stopped car
    a = cmd if self.long_active else -self.driver_brake
    self.a_ego = a + self.grade
    self.speed = max(self.speed + self.a_ego * DT_CTRL, 0.)
    self.distance += self.speed * DT_CTRL
    if self.lead:
      self.lead["v"] = max(self.lead["v"] - self.lead["decel"] * DT_CTRL, 0.)
      self.lead["pos"] += self.lead["v"] * DT_CTRL
    rec = {"t": self.frame * DT_CTRL, "v_in": CS.vEgo, "speed": self.speed, "cmd": cmd, "enabled": self.enabled,
           "long_active": self.long_active, "state": self.long_state, "handback": self.ext.low_speed_handback.active,
           "events": set(events.names), "allow_brake": self.plan.allowBrake, "fcw": self.plan.fcw, "limits": limits,
           "events_sp": set(events_sp.names), "alert": self.AM.current_alert,
           "alert_types": list(self.state.current_alert_types)}
    self.frame += 1
    return rec

  def run(self, seconds):
    return [self.step() for _ in range(round(seconds / DT_CTRL))]


def first(log, cond):
  return next((i for i, l in enumerate(log) if cond(l)), None)


class TestManualLowSpeedHandback(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    p = Params()
    for k in ("ManualLowSpeedHandbackSpeed", "ManualLowSpeedHandbackRamp"):
      p.remove(k)
    p.put_bool("NonAdaptiveLongitudinal", False)

  def _lead_stop(self, transmission=TransmissionType.manual):
    CP, CP_SP = make_cp(transmission)
    sim = Sim(CP, CP_SP, speed=12., lead={"pos": 45., "v": 12., "decel": 1.5})
    return sim, sim.run(30.)

  def test_defaults(self):
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, Params())
    self.assertTrue(hb.available)
    self.assertAlmostEqual(hb.speed, DEFAULT_HANDBACK_SPEED)
    self.assertAlmostEqual(hb.ramp_time, DEFAULT_RAMP_TIME)

  def test_adaptive_hands_back_smoothly_with_one_alert(self):
    sim, log = self._lead_stop()
    start = first(log, lambda l: l["handback"])
    off = first(log, lambda l: not l["enabled"])
    self.assertIsNotNone(start)
    self.assertIsNotNone(off)
    # openpilot followed the lead down to the handback speed (braking) and handed back right there
    self.assertLess(log[start]["v_in"], DEFAULT_HANDBACK_SPEED)
    self.assertGreater(log[start - 1]["v_in"], DEFAULT_HANDBACK_SPEED)
    self.assertLess(log[start - 1]["cmd"], -0.3, "openpilot was braking for the lead at handback")
    self.assertTrue(all(l["long_active"] for l in log[:start]))
    # disengages ramp + margin after the start, while still rolling
    self.assertAlmostEqual((off - start) * DT_CTRL, DEFAULT_RAMP_TIME + DISENGAGE_MARGIN, delta=0.03)
    self.assertGreater(log[off]["speed"], 0.5, "hands back while still rolling")
    # smooth: no step in the command through the ramp, and it is 0 before the disengage
    cmds = [l["cmd"] for l in log[start - 1:off + 1]]
    max_step = max(abs(b - a) for a, b in zip(cmds, cmds[1:], strict=False))
    self.assertLess(max_step, abs(log[start - 1]["cmd"]) / (DEFAULT_RAMP_TIME / DT_CTRL) * 1.5 + 0.02)
    self.assertEqual(log[off - 1]["cmd"], 0.)
    # exactly one alert: the handback alert, shown at the disengage (it outranks the stock "Speed too low")
    shown = {l["alert"].alert_type for l in log if l["alert"].alert_type}
    self.assertEqual(shown, {HANDBACK_ALERT})
    self.assertEqual(log[off]["alert"].alert_type, HANDBACK_ALERT)
    self.assertEqual(log[off]["alert"].audible_alert, structs.CarControl.HUDControl.AudibleAlert.prompt)
    self.assertEqual(sum(EventNameSP.manualLowSpeedHandback in l["events_sp"] for l in log), 1, "fired once, no spam")

  def test_no_brake_to_stop_no_standstill_hold_no_launch(self):
    sim, log = self._lead_stop()
    # openpilot never stops the car or asks for a standstill hold: it is gone before the stop
    for l in log:
      if l["long_active"]:
        self.assertNotEqual(l["state"], LongCtrlState.stopping)
        self.assertGreater(l["speed"], 1.0)
    self.assertFalse(any(l["enabled"] for l in log if l["speed"] < 1.0))
    # driver brakes to a stop; the lead pulls away; trying to engage at a standstill is refused
    sim.driver_brake = 2.0
    log = sim.run(10.)
    self.assertEqual(log[-1]["speed"], 0.)
    sim.driver_brake = 0.
    sim.lead = {"pos": sim.distance + 20., "v": 5., "decel": 0.}
    sim.enable_request = True
    log = sim.run(0.5)
    sim.enable_request = False
    log += sim.run(10.)
    self.assertFalse(any(l["enabled"] for l in log), "engaging below the handback speed must be refused")
    self.assertEqual(max(l["speed"] for l in log), 0., "never launches")
    no_entry = [l["alert"].alert_type for l in log if l["alert"].alert_type]
    self.assertIn("manualLowSpeedNoEntry/noEntry", no_entry)
    self.assertNotIn("belowEngageSpeed/noEntry", no_entry)

  def test_can_engage_above_handback_speed(self):
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=DEFAULT_HANDBACK_SPEED + ENGAGE_HYSTERESIS + 1.)
    sim.state.state, sim.enabled = State.disabled, False
    sim.enable_request = True
    sim.step()
    sim.enable_request = False
    self.assertTrue(sim.run(1.)[-1]["enabled"])

  def test_non_adaptive_also_hands_back(self):
    # set-speed only, but a steep hill drags the car below the handback speed: openpilot is accelerating at handback
    Params().put_bool("NonAdaptiveLongitudinal", True)
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=4., grade=-3.0, v_set=10.)
    log = sim.run(6.)
    start = first(log, lambda l: l["handback"])
    off = first(log, lambda l: not l["enabled"])
    self.assertIsNotNone(start)
    self.assertGreater(log[start - 1]["cmd"], 0.3, "openpilot was on the gas at handback")
    self.assertGreaterEqual(min(l["cmd"] for l in log[start:off]), 0., "positive ramp never turns into braking")
    self.assertEqual(log[off - 1]["cmd"], 0.)
    self.assertEqual(log[off]["alert"].alert_type, HANDBACK_ALERT)

  def test_automatic_unchanged(self):
    sim, log = self._lead_stop(TransmissionType.automatic)
    self.assertTrue(all(l["enabled"] for l in log), "automatics keep openpilot down to (and at) the stop")
    self.assertFalse(any(l["handback"] or l["events_sp"] for l in log))
    self.assertEqual(log[-1]["speed"], 0.)
    self.assertEqual(log[-1]["state"], LongCtrlState.stopping, "automatic still brakes to a stop and holds")

  def test_pcm_cruise_noop(self):
    CP, CP_SP = make_cp(op_long=False)
    self.assertTrue(CP.pcmCruise)
    hb = LowSpeedHandback(CP, Params())
    self.assertFalse(hb.available)
    hb.update(True, 0.5, -1.)
    self.assertFalse(hb.active or hb.ramp_done or hb.disengage_due or hb.below_engage_speed(0.))
    self.assertEqual(hb.limit_accel((-3.5, 2.)), (-3.5, 2.))
    ev = Events()
    ev_sp = CarSpecificEventsSP(CP, CP_SP, Params()).update(messaging.new_message('carState').carState, ev, True)
    self.assertFalse(ev.names or ev_sp.names)

  def test_params_clamped_and_latched_while_disengaged(self):
    p = Params()
    p.put("ManualLowSpeedHandbackSpeed", 0., block=True)  # can't be tuned into a brake-to-stop
    p.put("ManualLowSpeedHandbackRamp", 9., block=True)
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, p)
    self.assertEqual(hb.speed, MIN_HANDBACK_SPEED)
    self.assertEqual(hb.ramp_time, 2.0)
    hb.update(True, 5., 0.)
    p.put("ManualLowSpeedHandbackSpeed", 4., block=True)
    hb.read_params()
    hb.update(True, 3., 0.)
    self.assertFalse(hb.active, "new threshold must not apply while engaged")
    hb.update(False, 3., 0.)
    self.assertEqual(hb.speed, 4.)

  def test_zero_ramp_is_immediate(self):
    Params().put("ManualLowSpeedHandbackRamp", 0., block=True)
    sim, log = self._lead_stop()
    start = first(log, lambda l: l["handback"])
    off = first(log, lambda l: not l["enabled"])
    self.assertEqual(log[start]["cmd"], 0.)
    self.assertLessEqual((off - start) * DT_CTRL, DISENGAGE_MARGIN + 0.02)

  def test_ramp_envelope(self):
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, Params())
    hb.update(True, 1., -2.0)
    self.assertEqual(hb.limit_accel((-3.5, 2.)), (-2.0, 0.))
    for _ in range(round(DEFAULT_RAMP_TIME / 2 / DT_CTRL)):
      hb.update(True, 1.)
    lo, hi = hb.limit_accel((-3.5, 2.))
    self.assertAlmostEqual(lo, -1.0, delta=0.05)
    self.assertEqual(hi, 0.)
    # tighter existing limits (e.g. the no-service-brake floor) still win
    self.assertEqual(hb.limit_accel((-0.3, 2.)), (-0.3, 0.))
    for _ in range(round(DEFAULT_RAMP_TIME / DT_CTRL)):
      hb.update(True, 1.)
    self.assertEqual(hb.limit_accel((-3.5, 2.)), (0., 0.))
    self.assertTrue(hb.ramp_done and hb.disengage_due)
