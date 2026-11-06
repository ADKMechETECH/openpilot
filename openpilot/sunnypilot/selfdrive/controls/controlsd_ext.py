"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.controller import NonAccController, PULSING
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.clutch_gate import ClutchShiftGate
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.handback import LowSpeedHandback


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()

    self.lane_centering = LaneCenteringController()
    self.update_lane_centering_params()

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP', 'carStateSP', 'longitudinalPlanSP']
    self.pm_services_ext = ['carControlSP']

    # fork-only (civic-cruise): non-ACC overspeed controller (engine-brake-priority floor + sawtooth)
    self.nonacc = NonAccController(CP, DT_CTRL)
    # fork-only (civic-cruise): clutch/shift gate (manual only) - caps the throttle while the clutch is in
    self.clutch_gate = ClutchShiftGate(CP, DT_CTRL)
    # fork-only (civic-cruise): manual low-speed handback (hand longitudinal back below ~5 mph, no brake-to-stop/launch)
    self.handback = LowSpeedHandback(CP, DT_CTRL)

  def handback_limits(self, CS, enabled: bool, force_decel: bool, plan_a_target: float,
                      lower_limit: float, upper_limit: float) -> tuple[float, float]:
    """fork-only: ramp the accel range to 0 for the low-speed handback; controlsd drops longActive when ramp_done."""
    cs_sp = self.sm['carStateSP']
    gear = cs_sp.gearRaw if (cs_sp.clutchSignalValid and not CS.deprecated.clutchPressed
                             and 1 <= cs_sp.gearRaw <= 6) else 0
    self.handback.update(enabled, CS.vEgo, plan_a_target, gear, force_decel)
    return self.handback.limit_accel(lower_limit, upper_limit)

  def clutch_gate_limits(self, CS, lower_limit: float, upper_limit: float) -> tuple[float, float]:
    """fork-only: advance the clutch gate and cap the throttle (upper) while the drivetrain is decoupled. Braking (the
    lower limit) always passes."""
    cs_sp = self.sm['carStateSP']
    self.clutch_gate.update(clutch_signal_valid=bool(cs_sp.clutchSignalValid), decoupled=bool(CS.deprecated.clutchPressed),
                            clutch_pedal=int(cs_sp.clutchPedal), gear_raw=int(cs_sp.gearRaw))
    return self.clutch_gate.limit_accel(lower_limit, upper_limit)

  def nonacc_lower_limit(self, long_active: bool, CS, calibrated_pose, force_decel: bool, lower_limit: float) -> float:
    """fork-only: raise LongControl's lower accel limit for the non-ACC overspeed controller. Returns the (possibly
    raised) lower limit; a no-op (returns lower_limit) whenever the controller steps aside. The low-speed hold may
    also ask for an upper cap: nonacc_upper_limit()."""
    v_set = CS.vCruise * CV.KPH_TO_MS
    cs_sp = self.sm['carStateSP']
    # numeric gear 1..6, trusted only while the clutch signal is live and the drivetrain is coupled
    gear_raw = int(cs_sp.gearRaw)
    in_gear = bool(cs_sp.clutchSignalValid and not CS.deprecated.clutchPressed and 1 <= gear_raw <= 6)
    gear = gear_raw if in_gear else None
    pitch = calibrated_pose.orientation.xyz[1] if calibrated_pose is not None else None
    # a real clutch-in / neutral (signal live): the controller keeps a declutched floor instead of stepping aside.
    # "Not in gear" rather than clutchPressed alone: carState and carStateSP are two messages, and for a frame as the
    # clutch comes up one can say "coupled" while the other still says "no gear" (session 14 replay: that one frame
    # of step-aside let the planner's request through and cost 0.6 s of ramp). Reverse (14) still steps aside.
    decoupled = bool(cs_sp.clutchSignalValid and not in_gear and gear_raw != 14)
    # the car's ACC mode (owner 2026-11-06): the same coast floor, engine braking first also behind a lead. plannerd
    # publishes the lowest accel the floor must let through (nonacc/acc_coast.py: the lead plan's braking once inside
    # the approach margin, for a stopped car or hard braking). No live, valid plan, or FCW -> None -> no floor at all.
    lead_accel_limit = None
    if self.sm.alive['longitudinalPlanSP'] and self.sm.valid['longitudinalPlanSP'] and not self.sm['longitudinalPlan'].fcw:
      lead_accel_limit = float(self.sm['longitudinalPlanSP'].coastFloorBound)
    plan_is_cruise = self.sm['longitudinalPlan'].longitudinalPlanSource == log.LongitudinalPlan.LongitudinalPlanSource.cruise
    return self.nonacc.update(long_active, CS.vEgo, v_set, gear, pitch, force_decel, lower_limit,
                              bool(CS.cruiseState.nonAdaptive), decoupled=decoupled,
                              last_accel=float(self.LoC.last_output_accel), lead_accel_limit=lead_accel_limit,
                              computer_braking=bool(cs_sp.computerBraking), plan_is_cruise=bool(plan_is_cruise))

  def nonacc_upper_limit(self, upper_limit: float) -> float:
    """fork-only: the low-speed hold pins the request on its coasting side (an upper cap); otherwise a no-op."""
    cap = self.nonacc.upper_cap
    return upper_limit if cap is None else min(upper_limit, float(cap))

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
      self.update_lane_centering_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

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

    # ICBM state
    icbm_src = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.intelligentCruiseButtonManagement.state = icbm_src.state
    CC_SP.intelligentCruiseButtonManagement.sendButton = icbm_src.sendButton
    CC_SP.intelligentCruiseButtonManagement.vTarget = icbm_src.vTarget

    # fork-only (civic-cruise): non-ACC mode status for the HUD overlay. Show the ARMED state (mode enabled, live-read,
    # engaged or not) so a gap-button long-press is always visibly confirmed - not just while op-long is controlling.
    # mode.active refreshes every control frame via nonacc_lower_limit. .pulsing/.overspeed stay the controller's
    # engaged-and-braking state, so "NON-ACC" shows whenever armed and "NON-ACC - BRAKING" only when actually braking.
    CC_SP.nonAcc.active = bool(self.nonacc.mode.active)
    CC_SP.nonAcc.pulsing = bool(self.nonacc.active and self.nonacc.state == PULSING)
    CC_SP.nonAcc.overspeed = float(self.nonacc.overspeed)
    CC_SP.nonAcc.gatePaused = bool(self.clutch_gate.paused)
    CC_SP.nonAcc.holdState = int(self.nonacc.hold)

    # manual low-speed handback (controlsd owns; selfdrived acts on this)
    CC_SP.handback.active = bool(self.handback.active)
    CC_SP.handback.disengage = bool(self.handback.disengage_due)
    CC_SP.handback.belowEngageSpeed = bool(self.handback.below_engage_speed(sm['carState'].vEgo))
    CC_SP.handback.speed = float(self.handback.threshold)

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
