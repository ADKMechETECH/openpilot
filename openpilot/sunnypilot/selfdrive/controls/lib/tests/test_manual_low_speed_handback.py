"""
Manual low-speed handback, closed loop. Each frame mirrors the two processes:
  selfdrived: CarSpecificEventsSP on controlsd's PUBLISHED state (carControlSP.manualLongitudinal, previous frame)
              -> StateMachine -> AlertManager (FCW raised from the plan, as selfdrived does)
  controlsd:  Rule-5 urgency -> ControlsExt.update_low_speed_handback -> longActive -> NSB floor -> handback envelope
              -> LongControl
with LongitudinalPlanner (20 Hz) and a car that tracks the command. A lead's gap is never clamped: contact fails a test.
Default params: handback at 2.24 m/s, 0.5 s ramp.
"""
import math

import numpy as np

from opendbc.car import structs
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
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import LowSpeedHandback, \
  DEFAULT_HANDBACK_SPEED, DEFAULT_RAMP_TIME, DISENGAGE_MARGIN, MIN_HANDBACK_SPEED, ENGAGE_HYSTERESIS, STANDSTILL_RESOLVE, \
  STANDSTILL_SPEED
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import HANDBACK_DRIVER_DECEL, HANDBACK_DRIVER_REACTION, \
  HANDBACK_MARGIN, handback_lead_reason, handback_urgency
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import Ext, FakeSM, Pose, make_cp, set_params

TransmissionType = structs.CarParams.TransmissionType
State = log.SelfdriveState.OpenpilotState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
HANDBACK_ALERT = "manualLowSpeedHandback/warning"


class Sim:
  def __init__(self, CP, CP_SP, speed, lead=None, grade=0., v_set=20., pitch=None):
    self.CP = CP
    self.planner = LongitudinalPlanner(CP, CP_SP, init_v=speed)
    self.loc = LongControl(CP, CP_SP)
    self.params = Params()
    self.ext = Ext(CP, self.params)
    self.car_events_sp = CarSpecificEventsSP(CP, CP_SP, self.params)
    self.state = StateMachine()
    self.state.state = State.enabled
    self.enabled = True
    self.AM = AlertManager()
    self.speed, self.a_ego, self.distance, self.grade, self.v_set = speed, 0., 0., grade, v_set
    # ONE way to put a slope on the car: `grade` is a raw additive accel (m/s^2, the older uphill tests), `pitch` (rad,
    # negative = downhill) is the calibrated pose controlsd sees AND gravity on the car (Task 14 A4). Both at once would
    # count the slope twice.
    if pitch is not None and grade != 0.:
      raise ValueError("Sim: give the slope as grade (m/s^2) or pitch (rad), not both")
    self.pitch = pitch
    self.tracks_request = False  # True: while openpilot is active the car's brake controller delivers the requested decel
                                 # whatever the grade (the VSA tracking assumption, Task 11); gravity acts once handed back
    self.lead = lead  # dict(pos, v, decel) or None
    self.driver_brake = 0.  # m/s^2 the driver applies once openpilot hands back
    self.driver_reaction = 0.  # s after longitudinal stops before the driver brakes
    self._released = 0.
    self.brake_pedal = False  # driver on the brake pedal (Honda: disengages, like selfdrived's pedalPressed)
    self.enable_request = False
    self.force_decel = False
    self.fcw = False
    self.contact = False
    self.frame = 0
    self.long_state = LongCtrlState.pid
    self.long_active = True
    self.plan = None
    self.ml = custom.CarControlSP.new_message().manualLongitudinal   # what controlsd published last frame

  def _radar(self):
    radar = messaging.new_message('radarState').radarState
    if self.lead:
      lead = radar.leadOne
      lead.dRel = float(self.lead["pos"] - self.distance)   # never clamped
      lead.vLead = lead.vLeadK = float(self.lead["v"])
      lead.aLeadK = float(-self.lead["decel"] if self.lead["v"] > 0 else 0.)
      lead.vRel = float(self.lead["v"] - self.speed)
      lead.aLeadTau = float(_LEAD_ACCEL_TAU)
      lead.present, lead.modelProb, lead.radar = True, 1.0, True
    return radar

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

  def _common(self, CS):
    sd = messaging.new_message('selfdriveState').selfdriveState
    sd.enabled = self.enabled
    dm = messaging.new_message('driverMonitoringState').driverMonitoringState
    dm.noResponseForceDecel = self.force_decel
    sp = messaging.new_message('carStateSP').carStateSP
    sp.clutchSignalValid, sp.gearRaw = True, 2
    return {'carState': CS, 'radarState': self._radar(), 'selfdriveState': sd, 'driverMonitoringState': dm, 'carStateSP': sp}

  def _plan(self, CS):
    sm = FakeSM(self._common(CS))
    control = messaging.new_message('controlsState').controlsState
    control.longControlState, control.forceDecel = self.long_state, self.force_decel
    model = messaging.new_message('modelV2').modelV2
    model.position.x = [float(x) for x in (self.speed + 0.5) * np.array(ModelConstants.T_IDXS)]
    model.velocity.x = [float(self.speed)] * len(ModelConstants.T_IDXS)
    model.acceleration.x = [0.] * len(ModelConstants.T_IDXS)
    model.orientationRate.z = [0.] * len(ModelConstants.T_IDXS)
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    sm.update({'controlsState': control, 'modelV2': model})
    for s in ('carControl', 'vehicleParameters', 'liveMapDataSP', 'gpsLocation'):
      sm[s] = getattr(messaging.new_message(s), s)
    sm['carControl'].enabled = self.enabled
    sm['carControl'].longActive = self.long_active
    self.planner.update(sm)
    plan = messaging.new_message('longitudinalPlan').longitudinalPlan
    plan.aTarget, plan.shouldStop = float(self.planner.output_a_target), bool(self.planner.output_should_stop)
    plan.allowBrake, plan.fcw = bool(self.planner.allow_brake), bool(self.planner.fcw) or self.fcw
    plan.hasLead = bool(self.lead)
    return plan

  def step(self):
    CS = self._car_state()
    # selfdrived: events (enabled from the previous step) -> state machine -> alerts; acts on controlsd's published state
    events = Events()
    if self.enable_request:
      events.add(EventName.buttonEnable)
    if self.brake_pedal and not CS.standstill:
      events.add(EventName.pedalPressed)
    if self.plan is not None and self.plan.fcw:
      events.add(EventName.fcw)
    events_sp = self.car_events_sp.update(CS, events, self.enabled, self.ml, True)
    self.enabled, _ = self.state.update(events)
    sd_sm = FakeSM({'carControlSP': custom.CarControlSP.new_message(manualLongitudinal=self.ml.to_dict())})
    callback_args = [self.CP, CS, sd_sm, False, 0, None]
    self.AM.add_many(self.frame, events.create_alerts(self.state.current_alert_types, callback_args) +
                     events_sp.create_alerts(self.state.current_alert_types, callback_args))
    self.AM.process_alerts(self.frame, set())

    # controlsd
    if self.plan is None or self.frame % round(DT_MDL / DT_CTRL) == 0:
      self.plan = self._plan(CS)
    sm = FakeSM(self._common(CS))
    sm['longitudinalPlan'] = self.plan
    sm['longitudinalPlanSP'] = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP
    self.ext.update_sp_state(sm, self.enabled, self.loc.long_control_state != LongCtrlState.off)
    pose = Pose(self.pitch) if self.pitch is not None else None
    handback_done = self.ext.update_low_speed_handback(self.enabled, CS.vEgo, self.loc.last_output_accel, sm, pose)
    self.long_active = self.enabled and not handback_done
    if not self.long_active:
      self.loc.reset()
    # same order as controlsd: no-service-brake floor, then the handback envelope
    limits = self.ext.apply_no_service_brake(sm, (ACCEL_MIN, ACCEL_MAX), self.long_active, self.loc.last_output_accel)
    limits = self.ext.low_speed_handback.limit_accel(limits)
    cmd = float(self.loc.update(self.long_active, CS, self.plan.aTarget, self.plan.shouldStop, limits))
    self.long_state = self.loc.long_control_state
    self.ml = custom.CarControlSP.new_message().manualLongitudinal
    self.ext.fill_manual_longitudinal(self.ml)

    # vehicle: openpilot's command while active, else the driver's own braking (after the reaction time); brakes hold
    # a stopped car
    self._released = self._released + DT_CTRL if not self.long_active else 0.
    a = cmd if self.long_active else (-self.driver_brake if self._released > self.driver_reaction else 0.)
    gravity = -9.81 * math.sin(self.pitch) if self.pitch is not None else 0.
    self.a_ego = a + self.grade + (0. if self.tracks_request and self.long_active else gravity)
    self.speed = max(self.speed + self.a_ego * DT_CTRL, 0.)
    self.distance += self.speed * DT_CTRL
    if self.lead:
      self.lead["v"] = max(self.lead["v"] - self.lead["decel"] * DT_CTRL, 0.)
      self.lead["pos"] += self.lead["v"] * DT_CTRL
      self.contact |= self.lead["pos"] - self.distance <= 0.
    rec = {"t": self.frame * DT_CTRL, "v_in": CS.vEgo, "speed": self.speed, "cmd": cmd, "enabled": self.enabled,
           "long_active": self.long_active, "state": self.long_state, "handback": self.ext.low_speed_handback.active,
           "events": set(events.names), "allow_brake": self.plan.allowBrake, "fcw": self.plan.fcw, "limits": limits,
           "events_sp": set(events_sp.names), "alert": self.AM.current_alert, "urgency": self.ext.urgency,
           "hb_urgent": self.ext.low_speed_handback.urgent, "hb_timer": self.ext.low_speed_handback.timer,
           "hb_latched": self.ext.low_speed_handback.lead_latched,
           "gap": (self.lead["pos"] - self.distance) if self.lead else None,
           "alert_types": list(self.state.current_alert_types), "collision": self.planner.collision_hold > 0.}
    self.frame += 1
    return rec

  def run(self, seconds):
    return [self.step() for _ in range(round(seconds / DT_CTRL))]


def first(log, cond):
  return next((i for i, l in enumerate(log) if cond(l)), None)


class TestManualLowSpeedHandback(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    set_params(NonAdaptiveLongitudinal=False)

  def _uphill(self, **kw):
    """Non-urgent: no lead, a steep hill drags the car below the handback speed while openpilot is on the gas."""
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=4., grade=-3.0, v_set=10., **kw)
    return sim, sim.run(6.)

  def assert_not_held(self, log):
    """[Task 13 A1] openpilot never holds a manual at a stop: engaged at standstill for at most the standstill
    resolution + the ramp + the disengage margin, and the run ends handed back."""
    held = sum(1 for l in log if l["enabled"] and l["speed"] < STANDSTILL_SPEED) * DT_CTRL
    self.assertLessEqual(held, STANDSTILL_RESOLVE + DEFAULT_RAMP_TIME + DISENGAGE_MARGIN + 0.05, "held at a standstill")
    self.assertFalse(log[-1]["enabled"], "the run must end handed back")
    self.assertFalse(log[-1]["long_active"])

  def test_defaults(self):
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, Params())
    self.assertTrue(hb.available)
    self.assertAlmostEqual(hb.speed, DEFAULT_HANDBACK_SPEED)
    self.assertAlmostEqual(hb.ramp_time, DEFAULT_RAMP_TIME)

  def test_non_urgent_handback_smooth_with_one_prompt(self):
    # [Task 12 A3.3] unchanged except the prompt: shown from the ramp start (it used to come at the disengage)
    sim, log = self._uphill()
    start = first(log, lambda l: l["handback"])
    off = first(log, lambda l: not l["enabled"])
    self.assertIsNotNone(start)
    self.assertIsNotNone(off)
    self.assertTrue(all(l["urgency"] is None for l in log))
    self.assertGreater(log[start - 1]["cmd"], 0.3, "openpilot was on the gas at handback")
    self.assertAlmostEqual((off - start) * DT_CTRL, DEFAULT_RAMP_TIME + DISENGAGE_MARGIN, delta=0.04)
    # smooth: the command ramps to 0 without a step, never turns into braking, is 0 before the disengage
    cmds = [l["cmd"] for l in log[start - 1:off]]
    self.assertLess(max(abs(b - a) for a, b in zip(cmds, cmds[1:], strict=False)), abs(cmds[0]) / (DEFAULT_RAMP_TIME / DT_CTRL) * 1.5 + 0.02)
    self.assertGreaterEqual(min(cmds[1:]), 0.)
    self.assertEqual(log[off - 1]["cmd"], 0.)
    # one prompt, from the ramp start (a frame later: selfdrived acts on the previous frame's published state)
    prompt = first(log, lambda l: l["alert"].alert_type == HANDBACK_ALERT)
    self.assertLessEqual(prompt - start, 2)
    self.assertLess(prompt, off)
    self.assertEqual(log[prompt]["alert"].audible_alert, structs.CarControl.HUDControl.AudibleAlert.prompt)
    shown = {l["alert"].alert_type for l in log if l["alert"].alert_type}
    self.assertEqual(shown - {"speedTooLow/immediateDisable"}, {HANDBACK_ALERT})
    self.assertIn(EventName.speedTooLow, log[off]["events"], "the stock event does the disengage")

  def test_stopped_lead_close_at_handback_start(self):
    # [A-F-H2, B-H2] a stopped car 3-5 m ahead as the car crosses the handback speed: braking is never handed back
    for gap in (3.0, 4.0, 5.0):
      with self.subTest(gap=gap):
        CP, CP_SP = make_cp()
        sim = Sim(CP, CP_SP, speed=DEFAULT_HANDBACK_SPEED + 0.3, lead={"pos": gap + 1.5, "v": 0., "decel": 0.})
        log = sim.run(6.)
        self.assertFalse(sim.contact, gap)
        self.assertEqual(log[-1]["speed"], 0., "the car stops")
        self.assertTrue(any(l["handback"] for l in log))
        self.assertTrue(all(l["long_active"] for l in log if l["speed"] > 0.), "braking stays while urgent")
        self.assert_not_held(log)

  def test_lead_stopping_ahead_adaptive(self):
    # following a lead that brakes to a stop: the lead is urgent (braking / close), so openpilot brakes to the stop
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=12., lead={"pos": 45., "v": 12., "decel": 1.5})
    log = sim.run(30.)
    self.assertFalse(sim.contact)
    self.assertGreater(min(l["gap"] for l in log), 1.0)
    self.assertEqual(log[-1]["speed"], 0.)
    self.assert_not_held(log)

  def test_force_decel_blocks_and_aborts(self):
    # [B-M1] driver-monitoring forceDecel while crossing the handback speed: no handback, the car stops
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=4.)
    sim.force_decel = True
    log = sim.run(8.)
    self.assertFalse(any(l["handback"] for l in log))
    self.assertEqual(log[-1]["speed"], 0.)
    self.assertTrue(all(l["enabled"] for l in log))
    # a handback already running is aborted by forceDecel
    sim2, _ = self._uphill()
    hb = sim2.ext.low_speed_handback
    hb.update(True, 1.0, 0.3)
    self.assertTrue(hb.active)
    hb.update(True, 1.0, 0.3, None, True)
    self.assertFalse(hb.active)

  def test_fcw_stays_the_displayed_alert(self):
    # [A3.4] FCW active when the handback starts: FCW "BRAKE!" stays on top (the handback prompt is below it)
    sim, _ = self._uphill()
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=4., grade=-3.0, v_set=10.)
    sim.fcw = True
    log = sim.run(4.)
    during = [l for l in log if l["handback"]]
    self.assertTrue(during)
    self.assertTrue(any(EventNameSP.manualLowSpeedHandback in l["events_sp"] for l in during))
    for l in during[2:]:
      self.assertTrue(l["alert"].alert_type.startswith("fcw"), l["alert"].alert_type)
    self.assertTrue(all(l["enabled"] for l in log), "no handback disengage while FCW is up")

  def test_throttle_start_then_lead_stops(self):
    # [B-M4] the handback started on the throttle (uphill), then a stopped car appears 4 m ahead at ~2 m/s — imminent
    # (Task 13 A1: the lead must be one the driver couldn't stop for after the handback): braking is available at once,
    # the car stops short of it, then hands back (standstill resolution) instead of holding.
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=2.35, grade=-1.0, v_set=10.)
    log = sim.run(0.5)
    self.assertTrue(any(l["handback"] and l["cmd"] > 0. for l in log), "handback starts while openpilot is on the gas")
    self.assertTrue(log[-1]["handback"] and log[-1]["long_active"], "precondition: still in the ramp")
    sim.lead = {"pos": sim.distance + 4., "v": 0., "decel": 0.}
    log += sim.run(5.)
    after = [l for l in log if l["gap"] is not None]
    self.assertFalse(sim.contact)
    self.assertEqual(after[1]["limits"][0], ACCEL_MIN, "braking side back at the stock limit the frame it's seen")
    self.assertLess(min(l["cmd"] for l in after), -0.5)
    self.assertGreater(min(l["gap"] for l in after), 1.0)
    self.assert_not_held(log)

  def test_no_brake_to_stop_no_standstill_hold_no_launch_when_not_urgent(self):
    sim, log = self._uphill()
    for l in log:
      if l["long_active"]:
        self.assertNotEqual(l["state"], LongCtrlState.stopping)
    self.assertFalse(any(l["enabled"] for l in log if l["speed"] < 0.5))
    # driver brakes to a stop; trying to engage at a standstill is refused; never launches
    sim.grade, sim.driver_brake = 0., 2.0
    log = sim.run(5.)
    self.assertEqual(log[-1]["speed"], 0.)
    sim.driver_brake = 0.
    sim.enable_request = True
    log = sim.run(0.5)
    sim.enable_request = False
    log += sim.run(5.)
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

  def test_one_source_of_truth_across_processes(self):
    # [B-M4/A3.5] change the handback params just before engaging: selfdrived acts on what controlsd latched and
    # published - it reads no param and runs no timer of its own, so the two can't disagree
    import inspect
    from openpilot.sunnypilot.selfdrive.car import car_specific
    src = inspect.getsource(car_specific.CarSpecificEventsSP)
    self.assertNotIn("read_params", src)
    self.assertNotIn("LowSpeedHandback(", src)
    CP, CP_SP = make_cp()
    v = DEFAULT_HANDBACK_SPEED + ENGAGE_HYSTERESIS + 0.5     # engageable at the default, not at 4.0 m/s
    sim = Sim(CP, CP_SP, speed=v)
    sim.state.state, sim.enabled = State.disabled, False
    sim.step()
    Params().put("ManualLowSpeedHandbackSpeed", 4.0, block=True)   # controlsd hasn't re-read it yet
    sim.enable_request = True
    sim.step()
    sim.enable_request = False
    log = sim.run(0.5)
    self.assertTrue(log[-1]["enabled"], "both processes still use the latched 2.24 m/s")
    self.assertAlmostEqual(sim.ml.handbackSpeed, DEFAULT_HANDBACK_SPEED)

  def test_automatic_unchanged(self):
    CP, CP_SP = make_cp(TransmissionType.automatic)
    sim = Sim(CP, CP_SP, speed=12., lead={"pos": 45., "v": 12., "decel": 1.5})
    log = sim.run(30.)
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
    sim, log = self._uphill()
    start = first(log, lambda l: l["handback"])
    off = first(log, lambda l: not l["enabled"])
    self.assertEqual(log[start]["cmd"], 0.)
    self.assertLessEqual((off - start) * DT_CTRL, DISENGAGE_MARGIN + 0.03)

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

  def test_urgency_freezes_the_ramp_and_keeps_braking(self):
    # [Rule 5] urgency mid-ramp: the braking side is back at the incoming limit, the clock stops, no completion
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, Params())
    hb.update(True, 1., 0.5)
    for _ in range(20):
      hb.update(True, 1., 0.3)
    timer = hb.timer
    for _ in range(200):
      hb.update(True, 1., -2.0, "lead_close")
    self.assertEqual(hb.timer, timer)
    self.assertFalse(hb.ramp_done or hb.disengage_due)
    self.assertEqual(hb.limit_accel((-3.5, 2.))[0], -3.5)
    hb.update(True, 1., -2.0)   # urgency clears: the ramp restarts from the current command
    self.assertAlmostEqual(hb.limit_accel((-3.5, 2.))[0], -2.0, delta=0.05)


class TestTask13HandbackCarveOut(OpenpilotTestCase):
  """Task 13 A1 [N-1, review2 N6]: only a genuinely imminent lead freezes the handback; ordinary approach to slow or
  stopped traffic hands back at the handback speed; a stop while urgent ends in a clean handback, never a held stop."""
  def setUp(self):
    super().setUp()
    set_params(NonAdaptiveLongitudinal=False)

  def _driver(self, sim):
    # the driver the kinematic test assumes: brakes at HANDBACK_DRIVER_DECEL, HANDBACK_DRIVER_REACTION after the handback
    sim.driver_brake, sim.driver_reaction = HANDBACK_DRIVER_DECEL, HANDBACK_DRIVER_REACTION
    return sim

  def test_stopped_car_40m_ahead_hands_back(self):
    # the finding: approaching a stopped car 40 m ahead at ~3 m/s used to freeze the handback (lead_closing) and hold
    # the manual engaged at a standstill for the rest of the run
    CP, CP_SP = make_cp()
    sim = self._driver(Sim(CP, CP_SP, speed=3., lead={"pos": 40., "v": 0., "decel": 0.}, v_set=10.))
    log = sim.run(30.)
    start = first(log, lambda l: l["handback"])
    off = first(log, lambda l: not l["enabled"])
    self.assertIsNotNone(start)
    self.assertTrue(any(l["urgency"] in ("lead_closing", "lead_ttc") for l in log[:start + 1]),
                    "precondition: the broad Rule-5 lead reasons are present (they used to freeze it)")
    self.assertFalse(any(l["hb_urgent"] for l in log), "not imminent: the handback is never frozen")
    self.assertAlmostEqual((off - start) * DT_CTRL, DEFAULT_RAMP_TIME + DISENGAGE_MARGIN, delta=0.04,
                           msg="hands back at the handback speed, on the no-lead schedule")
    self.assertGreater(log[off]["v_in"], 1.0, "disengaged while still rolling, not after a stop")
    self.assertIn(EventName.speedTooLow, log[off]["events"])
    # end state, not just the ramp: disengaged for the rest of the run, never engaged at a standstill, stopped short
    self.assertFalse(any(l["enabled"] for l in log[off:]))
    self.assertFalse(any(l["enabled"] and l["speed"] < STANDSTILL_SPEED for l in log))
    self.assertEqual(log[-1]["speed"], 0.)
    self.assertFalse(sim.contact)
    self.assertGreater(min(l["gap"] for l in log), HANDBACK_MARGIN)
    prompts = [i for i in range(1, len(log)) if log[i]["alert"].alert_type == HANDBACK_ALERT and
               log[i - 1]["alert"].alert_type != HANDBACK_ALERT]
    self.assertEqual(len(prompts), 1, "one prompt")

  def test_slow_lead_ahead_hands_back(self):
    # following a 1 m/s lead (stop-and-go crawl): used to keep openpilot driving the manual below the handback speed
    CP, CP_SP = make_cp()
    sim = self._driver(Sim(CP, CP_SP, speed=3., lead={"pos": 40., "v": 1., "decel": 0.}, v_set=10.))
    log = sim.run(30.)
    start, off = first(log, lambda l: l["handback"]), first(log, lambda l: not l["enabled"])
    self.assertAlmostEqual((off - start) * DT_CTRL, DEFAULT_RAMP_TIME + DISENGAGE_MARGIN, delta=0.04)
    self.assertFalse(any(l["enabled"] for l in log[off:]))
    self.assertFalse(sim.contact)

  def test_slow_far_lead_does_not_freeze_the_ramp(self):
    # the uphill handback with a slow car 40 m ahead (closing > 1 m/s: a broad lead_closing reason). Non-adaptive (the
    # planner ignores that lead, so only the handback could react to it): the same handback as with no lead, frame for
    # frame. Adaptive (the MPC slows for it first): the ramp still runs on the no-lead schedule.
    def run(lead, non_adaptive):
      set_params(NonAdaptiveLongitudinal=non_adaptive)
      CP, CP_SP = make_cp()
      sim = Sim(CP, CP_SP, speed=4., grade=-3.0, v_set=10., lead=lead)
      log = sim.run(6.)
      return log, first(log, lambda l: l["handback"]), first(log, lambda l: not l["enabled"])
    log_none, start_none, off_none = run(None, True)
    log, start, off = run({"pos": 40., "v": 0.5, "decel": 0.}, True)
    self.assertTrue(any(l["urgency"] == "lead_closing" for l in log[start:off]), "precondition: broad reason present")
    self.assertFalse(any(l["hb_urgent"] for l in log))
    self.assertEqual((start, off), (start_none, off_none))
    self.assertEqual([l["cmd"] for l in log[start:off]], [l["cmd"] for l in log_none[start:off]])
    log, start, off = run({"pos": 40., "v": 0.5, "decel": 0.}, False)
    self.assertFalse(any(l["hb_urgent"] for l in log))
    self.assertAlmostEqual((off - start) * DT_CTRL, DEFAULT_RAMP_TIME + DISENGAGE_MARGIN, delta=0.04)

  def test_imminent_lead_during_ramp_keeps_full_braking(self):
    # a stopped car 3 m ahead appears mid-ramp at ~2 m/s: braking at the stock limit the same frame, no contact
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=2.35, grade=-1.0, v_set=10.)
    log = sim.run(0.5)
    self.assertTrue(log[-1]["handback"] and log[-1]["long_active"])
    sim.lead = {"pos": sim.distance + 3., "v": 0., "decel": 0.}
    log2 = sim.run(5.)
    self.assertTrue(log2[0]["hb_urgent"])
    self.assertEqual(log2[0]["limits"][0], ACCEL_MIN)
    self.assertFalse(sim.contact)
    self.assertEqual(log2[-1]["speed"], 0.)
    self.assertTrue(all(l["long_active"] for l in log2 if l["speed"] > 0.), "braking until the stop (latched)")
    self.assertFalse(log2[-1]["enabled"], "then a clean handback, not a held stop")

  def test_standstill_resolution(self):
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, Params())
    hb.update(True, 2.0, -1.0, "lead_imminent")
    self.assertTrue(hb.active and hb.urgent)
    t = 0.
    while hb.urgent:   # stopped, the lead still "imminent" (it's 2 m ahead): resolves after STANDSTILL_RESOLVE
      hb.update(True, 0., -1.0, "lead_imminent")
      t += DT_CTRL
      self.assertLess(t, 5.)
    self.assertAlmostEqual(t, STANDSTILL_RESOLVE, delta=1.5 * DT_CTRL)
    # the brake ramps off from the command at the stop, then the handback is due
    lo = [hb.limit_accel((ACCEL_MIN, ACCEL_MAX))[0]]
    while not hb.disengage_due:
      hb.update(True, 0., lo[-1], "lead_imminent")
      lo.append(hb.limit_accel((ACCEL_MIN, ACCEL_MAX))[0])
      self.assertLess(len(lo), 500)
    self.assertAlmostEqual(lo[1], -1.0, delta=0.05)
    self.assertEqual(lo[-1], 0.)
    self.assertTrue(all(b >= a - 1e-9 for a, b in zip(lo[1:], lo[2:], strict=False)), "monotonic release, no step")
    # rolling again before the disengage: urgency counts again
    hb2 = LowSpeedHandback(CP, Params())
    hb2.update(True, 2.0, -1.0, "lead_imminent")
    for _ in range(round(STANDSTILL_RESOLVE / DT_CTRL) + 2):
      hb2.update(True, 0., -1.0, "lead_imminent")
    self.assertFalse(hb2.urgent)
    hb2.update(True, 0.8, -0.2, "lead_imminent")
    self.assertTrue(hb2.urgent)

  def test_fcw_is_never_resolved(self):
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, Params())
    hb.update(True, 2.0, -1.0, "fcw")
    for _ in range(round(5. / DT_CTRL)):
      hb.update(True, 0., -1.0, "fcw")
    self.assertTrue(hb.urgent)
    self.assertFalse(hb.ramp_done)

  def test_handback_lead_reason(self):
    from openpilot.cereal import messaging
    def lead(d, v_rel, v_lead, a=0.):
      l = messaging.new_message('radarState').radarState.leadOne
      l.present, l.dRel, l.vRel, l.vLead, l.aLeadK = True, d, v_rel, v_lead, a
      return l
    r = DEFAULT_RAMP_TIME + DISENGAGE_MARGIN + HANDBACK_DRIVER_REACTION
    v = DEFAULT_HANDBACK_SPEED
    cases = [  # (gap, vRel, vLead, aLead) at the handback speed -> imminent?
      ((40., -v, 0.), False), ((10.4, -v, 0.), False), ((7., -v, 0.), True), ((4., -v, 0.), True),
      ((9., -1.24, 1.), False), ((1.5, 0., v), False), ((6., 0., v, -2.), True), ((12., 0., v, -1.0), False)]
    for args, want in cases:
      with self.subTest(args=args):
        self.assertEqual(handback_lead_reason(lead(*args), v, r) is not None, want)
    radar = messaging.new_message('radarState').radarState
    for reason in ("invalid_plan", "invalid_radar", "force_decel", "fcw", "stock_aeb", "hard_decel"):
      self.assertEqual(handback_urgency(reason, radar, v, r), reason)
    for reason in ("stopping", "lead_close", "lead_closing", "lead_ttc", "lead_braking", None):
      self.assertIsNone(handback_urgency(reason, radar, v, r), reason)
    radar.leadOne = lead(4., -v, 0.)
    self.assertEqual(handback_urgency(None, radar, v, r), "lead_imminent")


class TestTask14Latch(OpenpilotTestCase):
  """Task 14 [W1-N1..N4, N8]: the imminent-lead latch sets on the imminence test itself (not on whichever reason
  handback_urgency returns), releases once no lead is still closing within reach, FCW survives a standstill
  resolution, and a downhill grade enlarges the imminent distance. End states asserted, not just the ramp."""
  def setUp(self):
    super().setUp()
    set_params(NonAdaptiveLongitudinal=False)

  def _ramp_start(self, log):
    i = first(log, lambda l: l["handback"] and l["hb_timer"] > 0.)
    return None if i is None else log[i]

  def test_hard_braking_approach_latches_and_holds_to_the_stop(self):
    # [W1-N2] FCW (the broad reason, returned first) held while the car is above v_off, approaching a stopped car that
    # is imminent at the handback speed. Task 13: no latch (FCW hid lead_imminent); when FCW cleared at 0.6-1.0 m/s the
    # instantaneous test was already clear and the ramp ran a few metres behind the stopped car (one run: contact).
    for v0, gap, v_off in ((3.0, 7.0, 0.8), (3.5, 8.0, 1.0), (2.8, 6.5, 0.6)):
      with self.subTest(v0=v0, gap=gap, v_off=v_off):
        CP, CP_SP = make_cp()
        sim = Sim(CP, CP_SP, speed=v0, lead={"pos": gap, "v": 0., "decel": 0.})
        log = []
        for _ in range(round(10. / DT_CTRL)):
          sim.fcw = sim.speed > v_off
          log.append(sim.step())
        self.assertTrue(any(l["fcw"] and l["hb_latched"] for l in log), "the latch sets while FCW is the reason")
        start = self._ramp_start(log)
        self.assertIsNotNone(start)
        self.assertLessEqual(start["speed"], 0.3, "the ramp must not start while still rolling toward the stopped car")
        self.assertFalse(sim.contact)
        self.assertFalse(log[-1]["enabled"], "then it hands back (standstill resolution), not a held stop")

  def test_latched_lead_drives_off_hands_back(self):
    # [W1-N1] latched on a slow, close lead; the lead then drives off while the car still rolls. Task 13: latched for
    # good - engaged, frozen ramp, no throttle, never handing back (60 s). Now: release, then the normal handback.
    from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import LATCH_RELEASE
    for v0, gap, v_lead, t_go in ((2.6, 5.0, 1.0, 0.8), (2.4, 4.5, 0.5, 1.0)):
      with self.subTest(v0=v0, gap=gap, v_lead=v_lead):
        CP, CP_SP = make_cp()
        sim = Sim(CP, CP_SP, speed=v0, lead={"pos": gap, "v": v_lead, "decel": 0.})
        log = sim.run(t_go)
        self.assertTrue(log[-1]["hb_latched"], "precondition: latched on the imminent lead")
        self.assertGreater(sim.speed, 0.5, "precondition: still rolling when the lead leaves")
        sim.lead["decel"] = -1.5   # the lead pulls away
        log = sim.run(60.)
        release = first(log, lambda l: not l["hb_latched"])
        off = first(log, lambda l: not l["enabled"])
        self.assertIsNotNone(release, "the latch must release")
        self.assertIsNotNone(off, "the handback must complete")
        self.assertLessEqual((off - release) * DT_CTRL, DEFAULT_RAMP_TIME + DISENGAGE_MARGIN + 2 * DT_CTRL,
                             "completes within ramp + margin of the release")
        self.assertFalse(any(l["enabled"] for l in log[off:]), "end state: handed back for the rest of the run")
        self.assertGreaterEqual(release * DT_CTRL, LATCH_RELEASE - DT_CTRL, "debounced")
        self.assertFalse(sim.contact)

  def test_latch_open_loop(self):
    # the latch alone: set by the imminence test with FCW as the returned reason; held while a lead is closing within
    # reach; released LATCH_RELEASE after it isn't; then ramp + margin -> disengage
    from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import LATCH_RELEASE
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, Params())
    hb.update(True, 2.0, -1.0, "fcw", lead_imminent=True, lead_holding=True)
    self.assertTrue(hb.active and hb.lead_latched and hb.urgent)
    for _ in range(round(30. / DT_CTRL)):   # still closing within reach: held, however long
      hb.update(True, 1.5, -1.0, None, lead_imminent=False, lead_holding=True)
    self.assertTrue(hb.urgent and not hb.ramp_done)
    n = 0
    while not hb.disengage_due:
      hb.update(True, 1.5, -0.5, None, lead_imminent=False, lead_holding=False)
      n += 1
      self.assertLess(n, 500)
    self.assertAlmostEqual(n * DT_CTRL, LATCH_RELEASE + DEFAULT_RAMP_TIME + DISENGAGE_MARGIN, delta=3 * DT_CTRL)
    # a momentary gap in "holding" (radar chatter) doesn't release it
    hb2 = LowSpeedHandback(CP, Params())
    hb2.update(True, 2.0, -1.0, None, lead_imminent=True, lead_holding=True)
    for i in range(round(5. / DT_CTRL)):
      hb2.update(True, 1.5, -1.0, None, lead_holding=(i % 20 != 0))
    self.assertTrue(hb2.lead_latched)

  def test_fcw_after_standstill_resolution(self):
    # [W1-N3] resolved at a standstill on a latched lead, then the car creeps and FCW arrives: braking at the limit
    CP, _ = make_cp()
    hb = LowSpeedHandback(CP, Params())
    hb.update(True, 2.0, -1.0, None, lead_imminent=True, lead_holding=True)
    for _ in range(round((STANDSTILL_RESOLVE + 0.1) / DT_CTRL)):
      hb.update(True, 0., -1.0, None, lead_holding=True)
    self.assertTrue(hb.resolved and not hb.urgent, "precondition: resolved")
    hb.update(True, 0.3, -0.5, "fcw", lead_holding=True)   # creeping < 0.5 m/s: still "resolved"
    self.assertTrue(hb.resolved)
    self.assertTrue(hb.urgent, "FCW is never resolved away")
    self.assertEqual(hb.limit_accel((ACCEL_MIN, ACCEL_MAX))[0], ACCEL_MIN)
    self.assertFalse(hb.ramp_done)

  def test_grade_enlarges_the_imminent_distance(self):
    # [W1-N4] a stopped car at a gap that is NOT imminent on the flat but is on a -6 % grade
    import math as m
    from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import grade_accel
    r = DEFAULT_RAMP_TIME + DISENGAGE_MARGIN + HANDBACK_DRIVER_REACTION
    v = DEFAULT_HANDBACK_SPEED
    lead = messaging.new_message('radarState').radarState.leadOne
    lead.present, lead.vRel, lead.vLead = True, -v, 0.
    down, up = grade_accel(-m.atan(0.06)), grade_accel(m.atan(0.06))
    self.assertGreater(down, 0.5)
    self.assertEqual(up, 0., "uphill never makes it less conservative than the flat")
    self.assertEqual(grade_accel(None), 0., "no pose = level")
    flat_edge = next(d / 10. for d in range(20, 400) if handback_lead_reason(_with(lead, d / 10.), v, r) is None)
    down_edge = next(d / 10. for d in range(20, 400) if handback_lead_reason(_with(lead, d / 10.), v, r, down) is None)
    self.assertGreater(down_edge, flat_edge + 1.0, f"flat {flat_edge} m vs -6 % {down_edge} m")
    self.assertIsNotNone(handback_lead_reason(_with(lead, 30.), v, r, 1.9), "a grade that leaves < 0.3 m/s^2: imminent")

  def test_downhill_approach_keeps_a_gap(self):
    # [W1-N4] -6 % grade, a stopped car at various distances: the driver (2 m/s^2 brake against gravity, 1.5 s after the
    # handback) always stops short - openpilot keeps braking when the grade makes it imminent
    import math as m
    for gap in (6., 8., 10., 12., 14., 16., 19., 22., 26., 30.):
      with self.subTest(gap=gap):
        CP, CP_SP = make_cp()
        sim = Sim(CP, CP_SP, speed=3., lead={"pos": gap, "v": 0., "decel": 0.}, v_set=10., pitch=-m.atan(0.06))
        sim.driver_brake, sim.driver_reaction = HANDBACK_DRIVER_DECEL, HANDBACK_DRIVER_REACTION
        # the post-handback margin is the question here, so openpilot's own braking is delivered as requested (this
        # sim's planner + PID don't compensate a grade at all; the real car's VSA tracks the requested decel)
        sim.tracks_request = True
        log = sim.run(30.)
        self.assertFalse(sim.contact, gap)
        self.assertGreater(min(l["gap"] for l in log), 0.5, gap)


def _with(lead, d):
  lead.dRel = d
  return lead


class TestSimPlant(OpenpilotTestCase):
  def test_slope_counted_once(self):
    # the plant adds exactly -g sin(pitch) on top of openpilot's command (no second grade term), and refuses both knobs
    CP, CP_SP = make_cp()
    with self.assertRaises(ValueError):
      Sim(CP, CP_SP, speed=3., grade=-1.0, pitch=-0.06)
    sim = Sim(CP, CP_SP, speed=3., v_set=10., pitch=-0.06)
    for _ in range(50):
      r = sim.step()
      self.assertAlmostEqual(sim.a_ego - r["cmd"], -9.81 * math.sin(-0.06), places=9)
    sim.tracks_request = True
    for _ in range(50):
      r = sim.step()
      self.assertAlmostEqual(sim.a_ego - r["cmd"], 0., places=9)
