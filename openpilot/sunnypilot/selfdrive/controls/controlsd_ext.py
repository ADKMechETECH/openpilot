"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom

from opendbc.car import structs
from opendbc.car.structs import CLUTCH_PEDAL_UNKNOWN
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import controls_urgency, force_decel_from, grade_accel, \
  handback_lead_holding, handback_lead_imminent, handback_urgency
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_reengage import TakeoverRamp, takeover_seed
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import ClutchShiftGate
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import EngineBrakePreferred
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import LowSpeedHandback
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import PulseFloor, apply_brake_floor, read_decel_floor
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()
    self.clutch_shift_gate = ClutchShiftGate(CP, params)
    self.clutch_takeover = TakeoverRamp()
    self.low_speed_handback = LowSpeedHandback(CP, params, process="controlsd")
    # latched while disengaged (Task 12 B3): (enabled, floor)
    self._nsb_requested = (params.get_bool("NoServiceBrakeHold"), read_decel_floor(params))
    self.no_service_brake_enabled, self.no_service_brake_floor = self._nsb_requested
    self.no_service_brake_pulse = PulseFloor()
    self.urgency: str | None = None    # Rule 5: braking_urgency() for this control frame
    self.clutch_valid = False
    self.engine_brake_preferred = EngineBrakePreferred(CP, params)

    self.lane_centering = LaneCenteringController()
    self.update_lane_centering_params()

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP', 'longitudinalPlanSP', 'carStateSP']
    self.pm_services_ext = ['carControlSP']

  def initialize_lateral_control(self, lac, CI, dt):
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_versions = self.params.get("TorqueControlTune")
    if not enforce_torque_control:
      if self.CP.lateralTuning.which() == 'torque':
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)  # FIXME-SP: revert when upstream fixes tuning issues with v1
      return lac

    if torque_versions == 0.0:  # v0
      return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
    else:
      return lac

  def update_lane_centering_params(self) -> None:
    self.lane_centering_enabled = self.params.get_bool("LaneCentering")
    self.lane_centering_pause_on_signal = bool(self.params.get("LaneCenteringPauseOnSignal", return_default=True))
    self.lane_centering_e2e_authority = float(self.params.get("LaneCenteringE2EAuthority", return_default=True))
    self.lane_center_offset = float(self.params.get("LaneCenterOffset", return_default=True))

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()
      self.clutch_shift_gate.get_params()
      self.low_speed_handback.read_params()
      self._nsb_requested = (self.params.get_bool("NoServiceBrakeHold"), read_decel_floor(self.params))
      self.engine_brake_preferred.read_params()
      self.update_lane_centering_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

  def update_sp_state(self, sm: messaging.SubMaster, enabled: bool, plan_live: bool = True) -> None:
    """Once per control frame, before longActive and the accel limits: Rule-5 urgency, latching while disengaged
    (Task 12 B3) and the clutch gate (it caps throttle during a shift; it never drops longitudinal - Task 12 A2).
    plan_live: LongControl was not off last frame (else the plan is the planner's aEgo re-seed, not a request)."""
    self.urgency = controls_urgency(sm, plan_live)
    if not enabled:
      self.no_service_brake_enabled, self.no_service_brake_floor = self._nsb_requested
    CS_SP = sm['carStateSP']
    self.clutch_valid = bool(CS_SP.clutchSignalValid) and sm.all_checks(['carStateSP'])
    # Must run every frame (not short-circuited) so the gate sees every gas/clutch/RPM edge.
    # TODO(on-car): verify the Honda radarless VSA/PCM accept ACC_CONTROL with CONTROL_ON set and a coast command for the
    #  length of a shift with the clutch in (no CRUISE_FAULT, no cruise cancel).
    self.clutch_shift_gate.update(sm['carState'], enabled, CS_SP.gearRaw if self.clutch_valid else -1, self.clutch_valid,
                                  CS_SP.clutchPedal if self.clutch_valid else CLUTCH_PEDAL_UNKNOWN)

  def apply_no_service_brake(self, sm: messaging.SubMaster, accel_limits: tuple[float, float], long_active: bool,
                             last_accel: float) -> tuple[float, float]:
    # Rule 5: urgency releases the floor in this control frame, whatever the (up to 50 ms old) plan said.
    if self.urgency is not None:
      self.no_service_brake_pulse.floor = None
      return accel_limits
    # plannerd decides (longitudinalPlan.allowBrake); an invalid or stale plan never clamps (all_checks: braking allowed).
    # Overspeed pulse mode: plannerd's longitudinalPlanSP.noServiceBrake says coasting/pulsing and the target floor,
    # PulseFloor moves the limit at the pulse jerk. Anything else (maintain mode, escalated/urgent, invalid) -> legacy.
    plan_valid = sm.all_checks(['longitudinalPlan'])
    pulse = self.no_service_brake_pulse.apply(accel_limits, long_active, plan_valid and sm.all_checks(['longitudinalPlanSP']),
                                              self.no_service_brake_enabled, sm['longitudinalPlanSP'].noServiceBrake,
                                              last_accel, DT_CTRL)
    if pulse is not None:
      return pulse
    return apply_brake_floor(accel_limits, sm['longitudinalPlan'].allowBrake, plan_valid,
                             self.no_service_brake_enabled, self.no_service_brake_floor)

  def apply_engine_brake_preferred(self, sm: messaging.SubMaster, enabled: bool, long_active: bool,
                                   accel_limits: tuple[float, float], long_control_state, calibrated_pose) -> tuple[float, float]:
    # Must run every frame (latching, hold timer, logging). Rule-5 urgency or no pose -> never clamps.
    pitch = float(calibrated_pose.orientation.xyz[1]) if calibrated_pose is not None else None
    CS_SP = sm['carStateSP']
    self.engine_brake_preferred.update(enabled, long_active, sm['carState'], sm['longitudinalPlan'], sm['radarState'],
                                       self.urgency, long_control_state, pitch, self.clutch_valid,
                                       CS_SP.gearRaw if self.clutch_valid else -1,
                                       CS_SP.clutchPedal if self.clutch_valid else CLUTCH_PEDAL_UNKNOWN)
    return self.engine_brake_preferred.limit_accel(accel_limits)

  def apply_clutch_takeover(self, sm: messaging.SubMaster, long_active: bool, accel_limits: tuple[float, float],
                            last_accel: float = 0., calibrated_pose=None) -> tuple[float, float]:
    """Clutch gate: the throttle cap during a shift / after a coast timeout, then the takeover ramp back to the plan.
    Call after the NSB/EBP floors (the seed and ramp stay inside them) and before the low-speed handback."""
    gate = self.clutch_shift_gate
    accel_limits = gate.limit_accel(accel_limits, self.urgency)
    CS, plan = sm['carState'], sm['longitudinalPlan']
    if gate.resumed and long_active:
      if gate.reengage.smart:
        pitch = float(calibrated_pose.orientation.xyz[1]) if calibrated_pose is not None else None
        seed = takeover_seed(gate.reengage.seed, CS, gate.ratio_lock.gear, pitch)
      else:
        seed = last_accel  # the command kept running (capped) through the shift: lift the cap from there
      self.clutch_takeover.start(seed, plan.aTarget, accel_limits, gate.ratio_lock.gear, gate.reengage.max_ramp)
    if not self.clutch_takeover.active:
      return accel_limits
    return self.clutch_takeover.limit_accel(long_active, CS.vEgo, self.urgency, plan.aTarget, accel_limits)

  def plan_a_target(self, sm: messaging.SubMaster) -> float:
    """The aTarget LongControl runs this frame. GearAware (Task 13 A3) shapes only the set-speed candidate in plannerd;
    with a Rule-5 reason this control frame, take the stock-jerk plan plannerd published beside it (never higher than
    the plan: only ever more braking / less throttle). GearAware off, or no valid plan-SP message: the plan unchanged."""
    a_target = sm['longitudinalPlan'].aTarget
    if self.urgency is None or not sm.all_checks(['longitudinalPlanSP']):
      return a_target
    features = sm['longitudinalPlanSP'].features
    if not features.gearAware or not math.isfinite(features.aTargetStock):
      return a_target
    return min(a_target, features.aTargetStock)

  def post_clutch_takeover(self, LoC) -> None:
    pid = LoC.pid
    self.clutch_takeover.post_update(LoC.last_output_accel, pid.p + pid.i + pid.d + pid.f,
                                     LoC.long_control_state == LongCtrlState.pid)

  def update_low_speed_handback(self, enabled: bool, v_ego: float, last_accel: float, sm: messaging.SubMaster,
                                calibrated_pose=None) -> bool:
    """Manual transmission: returns True once the low-speed handback ramp is done (longitudinal must stop). Rule 5,
    narrowed (Task 13 A1): it doesn't complete while handback_urgency() (FCW, stock AEB, a hard request, invalid inputs,
    an imminent lead) holds — until the car has stood still a moment; forceDecel blocks / aborts it. Task 14: the lead
    latch's set / keep tests run on their own (A1/A2), and a downhill grade from the calibrated pose counts (A4; no
    pose = level)."""
    hb = self.low_speed_handback
    imminent = holding = False
    urgency = None
    if hb.available:
      pitch = float(calibrated_pose.orientation.xyz[1]) if calibrated_pose is not None else None
      radar, a_down, reaction = sm['radarState'], grade_accel(pitch), hb.reaction_time
      urgency = handback_urgency(self.urgency, radar, v_ego, reaction, a_down)
      imminent = handback_lead_imminent(radar, v_ego, reaction, a_down)
      holding = handback_lead_holding(radar, v_ego, reaction, a_down)
    hb.update(enabled, v_ego, last_accel, urgency, force_decel_from(sm), imminent, holding)
    return self.low_speed_handback.ramp_done

  def lead_visible(self, sm: messaging.SubMaster) -> bool:
    """Task 12 A1: in non-adaptive mode the cluster shows no ACC-follow lead unless openpilot is braking for a collision
    threat. longitudinalPlan.hasLead stays in the logs unchanged."""
    has_lead = bool(sm['longitudinalPlan'].hasLead)
    if not sm.all_checks(['longitudinalPlanSP']):
      return has_lead
    features = sm['longitudinalPlanSP'].features
    return has_lead and (not features.nonAdaptive or features.collisionBraking)

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  @staticmethod
  def get_lead_data(_lead, src: log.RadarState.LeadData) -> None:
    _lead.dRel = src.dRel
    _lead.yRel = src.yRel
    _lead.vRel = src.vRel
    _lead.aRel = src.deprecated.aRel
    _lead.vLead = src.vLead
    _lead.dPath = src.deprecated.dPath
    _lead.vLat = src.deprecated.vLat
    _lead.vLeadK = src.vLeadK
    _lead.aLeadK = src.aLeadK
    _lead.fcw = src.deprecated.fcw
    _lead.status = src.present
    _lead.aLeadTau = src.aLeadTau
    _lead.modelProb = src.modelProb
    _lead.radar = src.radar
    _lead.radarTrackId = src.radarTrackId

  def fill_manual_longitudinal(self, ml) -> None:
    """carControlSP.manualLongitudinal (Task 12): the state controlsd owns, as selfdrived / the console / the HUD see it."""
    gate, hb, ebp = self.clutch_shift_gate, self.low_speed_handback, self.engine_brake_preferred
    ml.urgency = self.urgency or ""
    ml.gateEnabled, ml.gateState, ml.gatePaused, ml.gateBlocked = gate.enabled, gate.state.name, gate.paused, gate.blocked
    ml.reengageSmart = bool(gate.enabled and gate.reengage.smart)
    ml.ebpEnabled, ml.ebpSuppressing, ml.ebpLockedOut = ebp.enabled, ebp.active, False
    ml.ebpFloor, ml.ebpGear, ml.ebpBinding = float(ebp.floor), int(ebp.gear), bool(ebp.binding)
    ml.handbackAvailable, ml.handbackActive = hb.available, hb.active
    ml.handbackRampDone, ml.handbackDisengage = hb.ramp_done, hb.disengage_due
    ml.handbackSpeed, ml.handbackRamp = float(hb.speed), float(hb.ramp_time)
    ml.nsbEnabled, ml.nsbDecelFloor = bool(self.no_service_brake_enabled), float(self.no_service_brake_floor)

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    self.get_lead_data(CC_SP.leadOne, sm['radarState'].leadOne)
    self.get_lead_data(CC_SP.leadTwo, sm['radarState'].leadTwo)

    # MADS state
    mads_src = sm['selfdriveStateSP'].mads
    CC_SP.mads.state = mads_src.state
    CC_SP.mads.enabled = mads_src.enabled
    CC_SP.mads.active = mads_src.active
    CC_SP.mads.available = mads_src.available

    # Task 12: the manual-longitudinal state controlsd owns (selfdrived, the console and the HUD read it)
    self.fill_manual_longitudinal(CC_SP.manualLongitudinal)

    # ICBM state
    icbm_src = sm['selfdriveStateSP'].intelligentCruiseButtonManagement

    CC_SP.intelligentCruiseButtonManagement.state = icbm_src.state
    CC_SP.intelligentCruiseButtonManagement.sendButton = icbm_src.sendButton
    CC_SP.intelligentCruiseButtonManagement.vTarget = icbm_src.vTarget

    return CC_SP

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
