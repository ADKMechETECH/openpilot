"""
Task 12 closed-loop harness: LongitudinalPlanner (+ its real publish) -> the real controlsd glue (ControlsExt: Rule-5
urgency, NSB / EBP floors, clutch gate + takeover ramp, low-speed handback) -> LongControl -> a manual car.

The car is either
  open   ACCEL_COMMAND ADDS to engine braking (closed throttle at command 0, the brakes for a negative command on top),
  closed the car TRACKS the commanded accel (the VSA/PCM close the loop: command 0 holds speed, -1.0 is -1.0 whatever
         the grade or gear) — on-car test C2 decides which one the radarless Civic is; tests run both.
A lead (optional) moves with its own speed / accel; its gap is NEVER clamped: contact is recorded and is a test failure.
"""
import math

import numpy as np

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_reengage import TakeoverRamp
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import ClutchShiftGate
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import G, engine_rpm_at, estimate_coast_decel, \
                                                                           resistive_decel
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import EngineBrakePreferred
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import LowSpeedHandback
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import PulseFloor, read_decel_floor

TransmissionType = structs.CarParams.TransmissionType
IDLE_RPM = 750.
ACTUATOR_TAU = 0.3          # s, first-order lag from command to the car's accel
THROTTLE_FADE = 0.4         # m/s^2 of command that fully opens the throttle (engine braking fades out)
FEATURE_KEYS = ("NonAdaptiveLongitudinal", "NoServiceBrakeHold", "NoServiceBrakeDecelFloor", "NoServiceBrakeOverspeedBackstop",
                "NoServiceBrakeOverspeedMargin", "NoServiceBrakeOverspeedMode", "ClutchShiftGate", "ClutchShiftGateGasReleaseHint",
                "ClutchReengageSmart", "ClutchReengageGasDebounce", "ClutchReengageRatioTol", "ClutchReengageMaxRamp",
                "ClutchReengageSeed", "ClutchShiftGateMaxCoast", "ClutchShiftGateCoastAccel", "EngineBrakePreferred",
                "GearAwareAccel", "ManualLowSpeedHandbackSpeed", "ManualLowSpeedHandbackRamp", "DisengageOnAccelerator")


def make_cp(transmission=TransmissionType.manual, op_long=True):
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  fingerprint = gen_empty_fingerprint()  # no gearbox message -> manual
  if transmission != TransmissionType.manual:
    fingerprint[0][0x1A3] = 8  # GEARBOX_AUTO
  CP = CarInterface.get_params(CAR.HONDA_CIVIC_2022, fingerprint, [], op_long, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.HONDA_CIVIC_2022, fingerprint, [], op_long, False, False)
  return CP, CP_SP


def set_params(**params):
  """Every feature param back to its default, then these (blocking writes: read back right away)."""
  p = Params()
  for k in FEATURE_KEYS:
    p.remove(k)
  for k, v in params.items():
    if isinstance(v, bool):
      p.put_bool(k, v, block=True)
    else:
      p.put(k, v, block=True)
  return p


class FakeSM(dict):
  """SubMaster stand-in: every service fresh and valid unless listed in `invalid` (all_checks and .valid fail) or
  `stale` (.valid still True, but all_checks fails: not alive / below its frequency)."""
  def __init__(self, *a, invalid=(), stale=(), **kw):
    super().__init__(*a, **kw)
    self.invalid = set(invalid)
    self.stale = set(stale)
    self.valid = {k: k not in self.invalid for k in self}
    self.logMonoTime = {'modelV2': 0}

  @property
  def data(self):
    return self

  def all_checks(self, service_list=None):
    services = self.keys() if service_list is None else service_list
    return not any(s in self.invalid or s in self.stale for s in services)


class FakePM:
  def __init__(self):
    self.msgs = {}

  def send(self, name, msg):
    self.msgs[name] = getattr(messaging.log_from_bytes(msg.to_bytes(), log.Event), name)


class Ext(ControlsExt):
  """ControlsExt's longitudinal pieces with their real constructors (the full __init__ blocks on CarParamsSP)."""
  def __init__(self, CP, params):  # noqa: super-init-not-called
    self.CP, self.params = CP, params
    self.clutch_shift_gate = ClutchShiftGate(CP, params)
    self.clutch_takeover = TakeoverRamp()
    self.low_speed_handback = LowSpeedHandback(CP, params, process="controlsd")
    self._nsb_requested = (params.get_bool("NoServiceBrakeHold"), read_decel_floor(params))
    self.no_service_brake_enabled, self.no_service_brake_floor = self._nsb_requested
    self.no_service_brake_pulse = PulseFloor()
    self.engine_brake_preferred = EngineBrakePreferred(CP, params)
    self.urgency = None
    self.clutch_valid = False

  def read_params_sp(self) -> None:
    """get_params_sp's longitudinal part (it also polls lateral params we don't build here)."""
    self.clutch_shift_gate.get_params()
    self.low_speed_handback.read_params()
    self._nsb_requested = (self.params.get_bool("NoServiceBrakeHold"), read_decel_floor(self.params))
    self.engine_brake_preferred.read_params()


class Pose:
  def __init__(self, pitch):
    self.orientation = type("O", (), {"xyz": np.array([0., pitch, 0.])})()


def controls_frame(ext, sm, loc, enabled, gas_pressed, last_cmd_hint=None):
  """One controlsd frame (the same sequence controlsd.state_control runs). Returns (long_active, limits, cmd)."""
  CS = sm['carState']
  ext.update_sp_state(sm, enabled, loc.long_control_state != LongCtrlState.off)
  handback_done = ext.update_low_speed_handback(enabled, CS.vEgo, loc.last_output_accel, sm, sm.get('_pose'))
  # stock gasPressedOverride; selfdrived drops it only while the gate says it is paused and the clutch is in (A2.6)
  gate = ext.clutch_shift_gate
  override = gas_pressed and not (gate.paused and CS.deprecated.clutchPressed)
  long_active = enabled and not override and not handback_done
  if not long_active:
    loc.reset()
  pose = sm.get('_pose')
  limits = (ACCEL_MIN, ACCEL_MAX)
  limits = ext.apply_no_service_brake(sm, limits, long_active, loc.last_output_accel)
  limits = ext.apply_engine_brake_preferred(sm, enabled, long_active, limits, loc.long_control_state, pose)
  limits = ext.apply_clutch_takeover(sm, long_active, limits, loc.last_output_accel, pose)
  limits = ext.low_speed_handback.limit_accel(limits)
  plan = sm['longitudinalPlan']
  ext.last_a_target = ext.plan_a_target(sm)
  cmd = float(loc.update(long_active, CS, ext.last_a_target, plan.shouldStop, limits))
  ext.post_clutch_takeover(loc)
  return long_active, limits, cmd


class ManualCarSim:
  def __init__(self, speed=20., gear=4, v_set=None, plant="open", grade=0., slip=0.3, lead=None, rpm_hold=1, **params):
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
    params.setdefault("ClutchShiftGate", True)
    self.params = set_params(**params)
    self.CP, self.CP_SP = make_cp()
    self.planner = LongitudinalPlanner(self.CP, self.CP_SP, init_v=speed)
    self.ext = Ext(self.CP, self.params)
    self.loc = LongControl(self.CP, self.CP_SP)
    self.plant, self.grade, self.slip, self.rpm_hold = plant, grade, slip, rpm_hold
    self.speed, self.a_ego, self.gear, self.pos = speed, 0., gear, 0.
    self.v_set = speed if v_set is None else v_set
    self.clutch = self.gas = self.blip = False
    self.slip_t, self.rpm = slip, engine_rpm_at(gear, speed)
    self.rpm_reported, self.frame = self.rpm, 0
    self.long_active, self.enabled, self.force_decel = True, True, False
    self.invalid: tuple[str, ...] = ()
    self.fcw_override = False
    self.curvature = 0.   # 1/m: the road ahead (the model's predicted yaw rate) and the car's own, for SCC-V
    self.lead = None if lead is None else {"pos": lead[0], "v": lead[1], "a": lead[2] if len(lead) > 2 else 0.}
    self.contact = False
    self.driver_brake, self.driver_reaction = 0., 0.   # m/s^2 / s: the driver after a low-speed handback (Task 13 A1)
    self._handed_back = 0.
    self.t = 0.
    self.log: list[dict] = []
    self.plan = self.plan_sp = None

  # -- the car ---------------------------------------------------------------------------------------------------------
  def coupled(self):
    return not self.clutch and self.slip_t >= self.slip

  def set(self, clutch=None, gear=None, gas=None, blip=None, v_set=None, lead_a=None):
    if clutch is False and self.clutch:
      self.slip_t, self.rpm_bite = 0., self.rpm
    if clutch is not None:
      self.clutch = clutch
    if gear is not None:
      self.gear = gear
    if gas is not None:
      self.gas = gas
    if blip is not None:
      self.blip = blip
    if v_set is not None:
      self.v_set = v_set
    if lead_a is not None and self.lead is not None:
      self.lead["a"] = lead_a

  def car_accel(self, cmd, long_active):
    a_road = resistive_decel(self.speed) - G * math.sin(self.grade)
    if self.plant == "closed":
      if self.gas and self.coupled():
        return 1.0 + a_road
      if not long_active:
        return estimate_coast_decel(self.gear, self.rpm, self.speed, self.grade) if self.coupled() else a_road
      # the car tracks the request; decoupled it can't add throttle, only brake
      return cmd if self.coupled() else min(cmd, a_road)
    brake = min(cmd, 0.) if long_active else 0.
    if not self.coupled():
      return a_road + brake
    if self.gas:
      return 1.0 + a_road
    a_eb = estimate_coast_decel(self.gear, self.rpm, self.speed, self.grade)
    if not long_active:
      return a_eb
    fade = float(np.clip(1. - max(cmd, 0.) / THROTTLE_FADE, 0., 1.))
    return cmd + a_eb * fade

  def update_rpm(self):
    target = engine_rpm_at(self.gear, self.speed)
    if self.clutch:
      self.rpm = target * 1.05 if self.blip else max(self.rpm - 1500. * DT_CTRL, IDLE_RPM)
    elif self.slip_t < self.slip:
      self.slip_t += DT_CTRL
      self.rpm = self.rpm_bite + (target - self.rpm_bite) * min(self.slip_t / self.slip, 1.)
    else:
      self.rpm = target
    if self.frame % self.rpm_hold == 0:   # ENGINE_DATA sample-and-hold (rpm_hold=4: 25 Hz)
      self.rpm_reported = self.rpm

  # -- messages ---------------------------------------------------------------------------------------------------------
  def radar(self):
    radar = messaging.new_message('radarState').radarState
    if self.lead is not None:
      ld = radar.leadOne
      ld.present, ld.radar, ld.modelProb = True, True, 1.0
      ld.dRel = float(self.lead["pos"] - self.pos)
      ld.vLead = ld.vLeadK = float(self.lead["v"])
      ld.vRel = float(self.lead["v"] - self.speed)
      ld.aLeadK, ld.aLeadTau = float(self.lead["a"]), float(_LEAD_ACCEL_TAU)
    return radar

  def car_state(self):
    CS = messaging.new_message('carState').carState
    CS.vEgo, CS.aEgo, CS.gasPressed = float(self.speed), float(self.a_ego), bool(self.gas)
    CS.vCruise = self.v_set * 3.6
    CS.standstill = self.speed < 0.01
    CS.deprecated.clutchPressed = self.clutch
    CS.deprecated.engineRpm = float(self.rpm_reported)
    return CS

  def car_state_sp(self):
    sp = messaging.new_message('carStateSP').carStateSP
    sp.clutchSignalValid = True
    sp.gearRaw = 0 if self.clutch else self.gear
    return sp

  def common_sm(self):
    sd = messaging.new_message('selfdriveState').selfdriveState
    sd.enabled = self.enabled
    dm = messaging.new_message('driverMonitoringState').driverMonitoringState
    dm.noResponseForceDecel = self.force_decel
    return {'carState': self.car_state(), 'carStateSP': self.car_state_sp(), 'radarState': self.radar(),
            'selfdriveState': sd, 'driverMonitoringState': dm}

  def planner_sm(self):
    sm = FakeSM(self.common_sm(), invalid=self.invalid)
    control = messaging.new_message('controlsState').controlsState
    control.longControlState = self.loc.long_control_state
    control.forceDecel = self.force_decel
    control.curvature = float(self.curvature)
    cc = messaging.new_message('carControl').carControl
    cc.longActive, cc.enabled = self.long_active, self.enabled
    cc.orientationNED = [0., float(self.grade), 0.]
    model = messaging.new_message('modelV2').modelV2
    model.position.x = [float(x) for x in (self.speed + 0.5) * np.array(ModelConstants.T_IDXS)]
    model.velocity.x = [float(self.speed)] * len(ModelConstants.T_IDXS)
    model.acceleration.x = [0.] * len(ModelConstants.T_IDXS)
    model.orientationRate.z = [float(self.curvature * self.speed)] * len(ModelConstants.T_IDXS)
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    sm.update({'controlsState': control, 'carControl': cc, 'modelV2': model})
    for s in ('vehicleParameters', 'liveMapDataSP', 'gpsLocation', 'selfdriveStateSP'):
      sm[s] = getattr(messaging.new_message(s), s)
    sm.valid = {k: k not in sm.invalid for k in sm}
    return sm

  # -- the loop --------------------------------------------------------------------------------------------------------
  def run(self, seconds, **actions):
    if actions:
      self.set(**actions)
    for _ in range(round(seconds / DT_MDL)):
      sm = self.planner_sm()
      self.planner.update(sm)
      pm = FakePM()
      self.planner.publish(sm, pm)
      self.plan, self.plan_sp = pm.msgs['longitudinalPlan'], pm.msgs['longitudinalPlanSP']
      if self.fcw_override:
        self.plan = self.plan.as_builder()
        self.plan.fcw = True
      for _ in range(round(DT_MDL / DT_CTRL)):
        self.update_rpm()
        csm = FakeSM(self.common_sm(), invalid=self.invalid)
        csm.update({'longitudinalPlan': self.plan, 'longitudinalPlanSP': self.plan_sp, '_pose': Pose(self.grade)})
        csm.valid = {k: k not in csm.invalid for k in csm}
        self.long_active, limits, cmd = controls_frame(self.ext, csm, self.loc, self.enabled, self.gas)
        sent = cmd if self.long_active else 0.
        a_car = self.car_accel(sent, self.long_active)
        self._handed_back = self._handed_back + DT_CTRL if self.ext.low_speed_handback.ramp_done else 0.
        if self.driver_brake and self._handed_back > self.driver_reaction and self.speed > 0.:
          a_car -= self.driver_brake
        self.a_ego += (a_car - self.a_ego) * DT_CTRL / ACTUATOR_TAU
        self.speed = max(self.speed + self.a_ego * DT_CTRL, 0.)
        if self.speed <= 0.:
          self.a_ego = max(self.a_ego, 0.)
        self.pos += self.speed * DT_CTRL
        gap = None
        if self.lead is not None:
          self.lead["v"] = max(self.lead["v"] + self.lead["a"] * DT_CTRL, 0.)
          self.lead["pos"] += self.lead["v"] * DT_CTRL
          gap = self.lead["pos"] - self.pos
          self.contact |= gap <= 0.
        gate = self.ext.clutch_shift_gate
        self.log.append({"t": self.t, "cmd": cmd, "sent": sent, "a_ego": self.a_ego, "v": self.speed, "active": self.long_active,
                         "limits": limits, "coupled": self.coupled(), "gap": gap, "gate": gate.state.name,
                         "paused": gate.paused, "ramp": self.ext.clutch_takeover.active, "urgency": self.ext.urgency,
                         "fcw": bool(self.plan.fcw), "a_target": float(self.plan.aTarget),
                         "handback": self.ext.low_speed_handback.active, "clutch": self.clutch,
                         "collision": bool(self.plan_sp.features.collisionBraking),
                         "source": str(self.plan.longitudinalPlanSource), "allow_brake": bool(self.plan.allowBrake),
                         "target_source": str(self.planner.source)})
        self.t += DT_CTRL
        self.frame += 1
    return self

  def min_gap(self):
    gaps = [r["gap"] for r in self.log if r["gap"] is not None]
    return min(gaps) if gaps else math.inf

  def plant_jerk(self, t0, t1):
    a = [r["a_ego"] for r in self.log if t0 <= r["t"] <= t1]
    return float(np.max(np.abs(np.diff(a))) / DT_CTRL) if len(a) > 1 else 0.
