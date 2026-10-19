"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import NoServiceBrake, overspeed_urgent
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import CoastEventLogger
from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import GearAwareAccel
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

EventNameSP = custom.OnroadEventSP.EventName
DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.

    # Non-adaptive (set-speed only) longitudinal: ignore leads and the e2e model, hold set speed.
    # Transmission- and platform-agnostic; only meaningful when openpilot owns longitudinal (with stock ACC the
    # plan doesn't actuate, and must not be reshaped behind the stock controller's back).
    self._params = Params()
    self.non_adaptive_available = bool(CP.openpilotLongitudinalControl)
    self.non_adaptive = self.non_adaptive_available and self._params.get_bool("NonAdaptiveLongitudinal")
    self._non_adaptive_requested = self.non_adaptive
    self._non_adaptive_frame = 0

    # No-service-brake ("coast over set speed"); only ever active together with non-adaptive (see no_service_brake.py)
    self.no_service_brake = NoServiceBrake(CP, self._params)

    # Manual only: gear-indexed set-speed jerk shaping (toggle GearAwareAccel, default OFF), and coast-event telemetry
    # for calibrating engine_brake_model (not a toggle: logging only, can't change driving — like the Phase 1 signals)
    self.gear_aware_accel = GearAwareAccel(CP, self._params)
    self.coast_logger = CoastEventLogger(CP.transmissionType == structs.CarParams.TransmissionType.manual, DT_MDL)

  @staticmethod
  def _pitch(sm: messaging.SubMaster) -> float | None:
    orientation = sm['carControl'].orientationNED
    return orientation[1] if len(orientation) == 3 else None

  def update_coast_logger(self, sm: messaging.SubMaster, fcw: bool) -> None:
    pitch = self._pitch(sm)
    self.coast_logger.update(sm['carState'], sm['carControl'].longActive, fcw, pitch if pitch is not None else float('nan'))

  def update_gear_aware_accel(self, sm: messaging.SubMaster, long_inactive: bool, fcw: bool, a_cruise_prev: float,
                              cruise_target: float) -> tuple[float, float]:
    return self.gear_aware_accel.update(long_inactive, sm['carState'], self._pitch(sm), fcw, sm['radarState'],
                                        a_cruise_prev, cruise_target)

  def update_no_service_brake(self, sm: messaging.SubMaster, long_inactive: bool, v_ego: float, fcw: bool = False) -> bool:
    CS = sm['carState']
    v_set = min(CS.vCruise, V_CRUISE_MAX) * CV.KPH_TO_MS  # driver's set speed, before SCC/SLA targets
    nsb = self.no_service_brake
    # urgent only matters to the overspeed pulse mode (maintain ignores it); an invalid radarState invalidates the whole
    # plan, which controlsd already treats as braking allowed
    allow_brake = nsb.update(long_inactive, self.non_adaptive, v_ego, v_set, sm['controlsState'].forceDecel,
                             overspeed_urgent(CS, fcw, sm['radarState']))
    if nsb.escalation_event:
      self.events_sp.add(EventNameSP.noServiceBrakeOverspeed)
    return allow_brake

  def update_non_adaptive(self, long_inactive: bool) -> None:
    # The toggle is re-read at 1 Hz but only applied while longitudinal is inactive, so the mode never flips mid-engagement
    if self._non_adaptive_frame % int(1. / DT_MDL) == 0:
      self._non_adaptive_requested = self.non_adaptive_available and self._params.get_bool("NonAdaptiveLongitudinal")
    self._non_adaptive_frame += 1
    if long_inactive:
      self.non_adaptive = self._non_adaptive_requested

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    # No-service-brake overspeed pulse mode (maintain mode leaves it unset: state off)
    if self.no_service_brake.state != "off":
      nsb = longitudinalPlanSP.noServiceBrake
      nsb.state = self.no_service_brake.state
      nsb.accelFloor = float(self.no_service_brake.accel_floor)
      nsb.rampRate = float(self.no_service_brake.pulse.jerk)
      nsb.overspeed = float(self.no_service_brake.overspeed)

    pm.send('longitudinalPlanSP', plan_sp_send)
