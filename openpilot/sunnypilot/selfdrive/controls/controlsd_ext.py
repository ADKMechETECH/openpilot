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
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import ClutchShiftGate
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import EngineBrakePreferred
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import LowSpeedHandback
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import apply_brake_floor, read_decel_floor
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()
    self.clutch_shift_gate = ClutchShiftGate(CP, params)
    self.low_speed_handback = LowSpeedHandback(CP, params, process="controlsd")
    self.no_service_brake_enabled = params.get_bool("NoServiceBrakeHold")
    self.no_service_brake_floor = read_decel_floor(params)
    self.engine_brake_preferred = EngineBrakePreferred(CP, params)

    self.lane_centering = LaneCenteringController()
    self.update_lane_centering_params()

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP']
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
      self.no_service_brake_enabled = self.params.get_bool("NoServiceBrakeHold")
      self.no_service_brake_floor = read_decel_floor(self.params)
      self.engine_brake_preferred.read_params()
      self.update_lane_centering_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

  def apply_no_service_brake(self, sm: messaging.SubMaster, accel_limits: tuple[float, float]) -> tuple[float, float]:
    # plannerd decides (longitudinalPlan.allowBrake); an invalid plan never clamps (fails toward allowing braking)
    return apply_brake_floor(accel_limits, sm['longitudinalPlan'].allowBrake, sm.valid['longitudinalPlan'],
                             self.no_service_brake_enabled, self.no_service_brake_floor)

  def apply_engine_brake_preferred(self, sm: messaging.SubMaster, long_active: bool, accel_limits: tuple[float, float],
                                   long_control_state, calibrated_pose) -> tuple[float, float]:
    # Must run every frame (latching, hold/capacity timers). Invalid plan/radar or no pose -> never suppresses.
    force_decel = bool(sm['driverMonitoringState'].noResponseForceDecel or
                       sm['selfdriveState'].state == log.SelfdriveState.OpenpilotState.softDisabling)
    pitch = float(calibrated_pose.orientation.xyz[1]) if calibrated_pose is not None else None
    self.engine_brake_preferred.update(long_active, sm['carState'], sm['longitudinalPlan'], sm.valid['longitudinalPlan'],
                                       sm['radarState'], sm.valid['radarState'], force_decel, long_control_state, pitch)
    return self.engine_brake_preferred.limit_accel(accel_limits)

  def get_long_paused(self, sm: messaging.SubMaster) -> bool:
    # Must run every frame (not short-circuited) so the gate sees every gas/clutch/RPM edge.
    # TODO(on-car): verify the Honda radarless VSA/PCM accept ACC_CONTROL with CONTROL_ON set but no gas/brake and
    #  no driver pedal input for the length of a shift (no CRUISE_FAULT, no cruise cancel). Fleet gas overrides only
    #  exercise this frame with the pedal pressed.
    return self.clutch_shift_gate.update(sm['carState'])

  def update_low_speed_handback(self, enabled: bool, v_ego: float, last_accel: float) -> bool:
    """Manual transmission: returns True once the low-speed handback ramp is done (longitudinal must stop)."""
    self.low_speed_handback.update(enabled, v_ego, last_accel)
    return self.low_speed_handback.ramp_done

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
