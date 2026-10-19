"""
Manual low-speed handback, closed loop. Each frame mirrors the two processes:
  selfdrived: CarSpecificEventsSP on controlsd's PUBLISHED state (carControlSP.manualLongitudinal, previous frame)
              -> StateMachine -> AlertManager (FCW raised from the plan, as selfdrived does)
  controlsd:  Rule-5 urgency -> ControlsExt.update_low_speed_handback -> longActive -> NSB floor -> handback envelope
              -> LongControl
with LongitudinalPlanner (20 Hz) and a car that tracks the command. A lead's gap is never clamped: contact fails a test.
Default params: handback at 2.24 m/s, 0.5 s ramp.
"""
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
  DEFAULT_HANDBACK_SPEED, DEFAULT_RAMP_TIME, DISENGAGE_MARGIN, MIN_HANDBACK_SPEED, ENGAGE_HYSTERESIS
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import Ext, FakeSM, make_cp, set_params

TransmissionType = structs.CarParams.TransmissionType
State = log.SelfdriveState.OpenpilotState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
HANDBACK_ALERT = "manualLowSpeedHandback/warning"


class Sim:
  def __init__(self, CP, CP_SP, speed, lead=None, grade=0., v_set=20.):
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
    self.lead = lead  # dict(pos, v, decel) or None
    self.driver_brake = 0.  # m/s^2 the driver applies once openpilot hands back
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
    handback_done = self.ext.update_low_speed_handback(self.enabled, CS.vEgo, self.loc.last_output_accel, sm)
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

    # vehicle: openpilot's command while active, else the driver's own braking; brakes hold a stopped car
    a = cmd if self.long_active else -self.driver_brake
    self.a_ego = a + self.grade
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

  def test_lead_stopping_ahead_adaptive(self):
    # following a lead that brakes to a stop: the lead is urgent (braking / close), so openpilot brakes to the stop
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=12., lead={"pos": 45., "v": 12., "decel": 1.5})
    log = sim.run(30.)
    self.assertFalse(sim.contact)
    self.assertGreater(min(l["gap"] for l in log), 1.0)
    self.assertEqual(log[-1]["speed"], 0.)

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
    # [B-M4] the handback started on the throttle (uphill), then a car ahead stops: braking is available at once
    CP, CP_SP = make_cp()
    sim = Sim(CP, CP_SP, speed=2.6, grade=-3.0, v_set=10.)
    log = sim.run(0.6)
    self.assertTrue(any(l["handback"] and l["cmd"] > 0. for l in log), "handback starts while openpilot is on the gas")
    sim.lead = {"pos": sim.distance + 6., "v": 0., "decel": 0.}
    log += sim.run(4.)
    after = [l for l in log if l["gap"] is not None]
    self.assertFalse(sim.contact)
    self.assertLess(min(l["limits"][0] for l in after), -1.0, "braking side back at the stock limit")
    self.assertLess(min(l["cmd"] for l in after), -0.5)

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
