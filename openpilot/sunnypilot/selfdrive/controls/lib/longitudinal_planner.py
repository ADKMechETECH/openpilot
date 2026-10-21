"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import MPC_NEEDED_DECEL, braking_urgency, collision_threat
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import NoServiceBrake
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import CoastEventLogger
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_learning import EngineBrakeLearner
from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import GearAwareAccel
from openpilot.sunnypilot.selfdrive.controls.lib.cruise_profile import CruiseProfile
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

EventNameSP = custom.OnroadEventSP.EventName
DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
COLLISION_DWELL = 2.0   # s: non-adaptive keeps the lead MPC re-admitted until no collision threat for this long
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
    # Task 12 A1 (owner decision 2026-10-19): non-adaptive keeps emergency collision braking. The lead MPC candidate is
    # re-admitted only for a real threat (braking_urgency.collision_threat / FCW / the MPC itself needing real braking),
    # held for COLLISION_DWELL after it clears so it doesn't chatter; never for ordinary following.
    self.collision_hold = 0.
    self.collision_reason = ""
    self.urgency: str | None = None

    # No-service-brake ("coast over set speed"); only ever active together with non-adaptive (see no_service_brake.py)
    self.no_service_brake = NoServiceBrake(CP, self._params)

    # Manual only: gear-indexed set-speed jerk shaping (toggle GearAwareAccel, default OFF), and coast-event telemetry
    # for calibrating engine_brake_model (not a toggle: logging only, can't change driving — like the Phase 1 signals)
    self.gear_aware_accel = GearAwareAccel(CP, self._params)
    # session 10 (owner, 2026-10-21): SetSpeedCoast / CruiseAccelBoost, opt-in, any transmission (cruise_profile.py)
    self.cruise_profile = CruiseProfile(CP, self._params)
    self.coast_logger = CoastEventLogger(CP.transmissionType == structs.CarParams.TransmissionType.manual, DT_MDL)
    # Task 17: engine-brake learning - brake-free coast samples + the axle-torque brake check, persisted; telemetry only
    # (its table proposals are not applied). Logs engaged frames while EngineBrakePreferred is latched on.
    self.eb_learner = EngineBrakeLearner(CP.transmissionType == structs.CarParams.TransmissionType.manual and
                                         bool(CP.openpilotLongitudinalControl), DT_MDL, self._params)
    self._ebp_requested = self._ebp_on = False

    # Task 16 (Stage 2 on-car, always on for manuals): the planner re-seeds from aEgo while the driver overrides with the
    # gas; on a manual that is the driver's own acceleration, which vanishes the instant they lift (closed throttle).
    self.manual = CP.transmissionType == structs.CarParams.TransmissionType.manual
    self.gas_override_reset = False

  def update_gas_override(self, sm: messaging.SubMaster, reset_state: bool) -> None:
    """Task 16: remembers that the current planner reset is a driver gas override while engaged. It stays set through
    the lift until the planner stops resetting (longitudinal back), so the first plans after the lift see it too."""
    if not reset_state:
      self.gas_override_reset = False
    elif self._engaged(sm) and sm['carState'].gasPressed:
      self.gas_override_reset = True

  def reset_accel(self, a_ego: float) -> float:
    """Task 16: the accel the planner re-seeds from. Manual + gas override: 'coast first, then plan' - never above 0, so
    openpilot never re-opens the throttle the driver just closed (Stage 2: +1.6 m/s^2 requested after a lift at +1.8
    re-accelerated the car for ~1.5 s to 5.5 mph over set). Automatics and every other reset: stock (aEgo)."""
    return min(a_ego, 0.) if self.manual and self.gas_override_reset else a_ego

  @staticmethod
  def _pitch(sm: messaging.SubMaster) -> float | None:
    orientation = sm['carControl'].orientationNED
    return orientation[1] if len(orientation) == 3 else None

  def update_coast_logger(self, sm: messaging.SubMaster, fcw: bool) -> None:
    pitch = self._pitch(sm)
    self.coast_logger.update(sm['carState'], sm['carControl'].longActive, fcw, pitch if pitch is not None else float('nan'))
    if self.eb_learner.available:
      engaged = self._engaged(sm)
      if self._non_adaptive_frame % int(1. / DT_MDL) == 0:
        self._ebp_requested = bool(self._params.get("EngineBrakePreferred", return_default=True))   # default ON
      if not engaged:
        self._ebp_on = self._ebp_requested   # latched like controlsd's (Task 12 B3)
      cc = sm['carControl']
      self.eb_learner.update(sm['carState'], sm['carStateSP'], engaged, cc.longActive, cc.actuators.accel, self._ebp_on,
                             pitch)

  @staticmethod
  def _engaged(sm) -> bool:
    return bool(sm['selfdriveState'].enabled)

  @staticmethod
  def _checks(sm, service: str) -> bool:
    # a real SubMaster: alive + frequency + valid; the unit-test sims pass plain dicts of fresh messages
    return sm.all_checks([service]) if hasattr(sm, 'all_checks') else True

  def _clutch_valid(self, sm) -> bool:
    return bool(sm['carStateSP'].clutchSignalValid) and self._checks(sm, 'carStateSP')

  def update_urgency(self, sm: messaging.SubMaster, fcw: bool, should_stop: bool, a_target: float, v_ego: float) -> str | None:
    """Rule 5, plannerd side: urgency from the most braking-demanding candidate (the lead MPC's output and stop request,
    the planner's FCW). An invalid/stale radarState counts; the plan itself is this process."""
    CS = sm['carState']
    radar_ok = self._checks(sm, 'radarState')
    self.urgency = braking_urgency(plan_ok=True, radar_ok=radar_ok, force_decel=sm['controlsState'].forceDecel, fcw=fcw,
                                   stock_aeb=CS.stockAeb or CS.stockFcw, should_stop=should_stop, a_target=a_target,
                                   v_ego=v_ego, radar=sm['radarState'])
    return self.urgency

  def update_collision_braking(self, sm: messaging.SubMaster, fcw: bool, a_target_mpc: float, v_ego: float, dt: float) -> bool:
    """A1: non-adaptive re-admits the lead MPC (and releases the NSB clamp) while a collision threat is present, plus
    COLLISION_DWELL. Returns whether it is re-admitted this frame. Always False outside non-adaptive mode."""
    if not self.non_adaptive:
      self.collision_hold, self.collision_reason = 0., ""
      return False
    CS, radar = sm['carState'], sm['radarState']
    reason = "fcw" if fcw else "stock_aeb" if (CS.stockAeb or CS.stockFcw) else None
    for lead in (radar.leadOne, radar.leadTwo):
      reason = reason or collision_threat(lead, v_ego)
    if reason is None and radar.leadOne.present and a_target_mpc < MPC_NEEDED_DECEL:
      reason = "mpc_needs_braking"
    if reason is not None:
      if self.collision_hold <= 0.:
        cloudlog.event("non_adaptive_collision_braking", active=True, reason=reason, v_ego=round(v_ego, 2))
      self.collision_hold, self.collision_reason = COLLISION_DWELL, reason
    elif self.collision_hold > 0.:
      self.collision_hold = max(self.collision_hold - dt, 0.)
      if self.collision_hold <= 0.:
        cloudlog.event("non_adaptive_collision_braking", active=False, reason=self.collision_reason)
        self.collision_reason = ""
    return self.collision_hold > 0.

  def update_gear_aware_accel(self, sm: messaging.SubMaster, a_cruise_prev: float, cruise_target: float) -> tuple[float, float]:
    return self.gear_aware_accel.update(self._engaged(sm), sm['carState'], self._pitch(sm), self.urgency, a_cruise_prev,
                                        cruise_target, self._clutch_valid(sm))

  def update_cruise_profile(self, sm: messaging.SubMaster, v_cruise: float, v_ego: float) -> tuple[float, list[float]]:
    return self.cruise_profile.update(self._engaged(sm), v_cruise, v_ego, self.urgency)

  def update_no_service_brake(self, sm: messaging.SubMaster, v_ego: float, collision: bool = False) -> bool:
    CS = sm['carState']
    v_set = min(CS.vCruise, V_CRUISE_MAX) * CV.KPH_TO_MS  # the driver's set speed
    # B1: the effective target after Smart Cruise Control (vision/map) / Speed Limit Assist, when one of them is lower
    v_target = float(self.output_v_target) if self.source != LongitudinalPlanSource.cruise else None
    nsb = self.no_service_brake
    urgency = self.urgency or ("collision" if collision else None)
    allow_brake = nsb.update(self._engaged(sm), self.non_adaptive, v_ego, v_set, sm['controlsState'].forceDecel,
                             urgency, v_target)
    if nsb.escalation_event:
      self.events_sp.add(EventNameSP.noServiceBrakeOverspeed)
    return allow_brake

  def update_non_adaptive(self, sm: messaging.SubMaster) -> None:
    # The toggle is re-read at 1 Hz but only applied while openpilot is disengaged (Task 12 B3: not merely while
    # longitudinal is off — a gas override or a shift must not flip it), so the mode never flips mid-engagement
    if self._non_adaptive_frame % int(1. / DT_MDL) == 0:
      self._non_adaptive_requested = self.non_adaptive_available and self._params.get_bool("NonAdaptiveLongitudinal")
    self._non_adaptive_frame += 1
    if not self._engaged(sm):
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

    # No-service-brake: state + target floor (pulse mode) and the latched values (every frame, Task 12 B3)
    n = self.no_service_brake
    nsb = longitudinalPlanSP.noServiceBrake
    nsb.state = n.state
    nsb.accelFloor = float(n.accel_floor)
    nsb.rampRate = float(n.pulse.jerk)
    nsb.overspeed = float(n.overspeed)
    nsb.enabled, nsb.mode, nsb.backstop = bool(n.enabled), n.mode, bool(n.backstop_enabled)
    nsb.margin, nsb.decelFloor, nsb.urgency = float(n.overspeed_margin), float(n.decel_floor), n.urgency or ""

    features = longitudinalPlanSP.features
    features.nonAdaptive = bool(self.non_adaptive)
    features.collisionBraking = self.collision_hold > 0.
    features.collisionReason = self.collision_reason
    features.gearAware = bool(self.gear_aware_accel.enabled)
    cp = self.cruise_profile
    features.setSpeedCoast, features.setSpeedCoastActive = bool(cp.coast_enabled), bool(cp.coast_active)
    features.accelBoost = bool(cp.boost_enabled)
    if self.gear_aware_accel.enabled or cp.shaped:   # only while on: OFF publishes exactly what it did before Task 13
      features.aTargetStock = float(self.output_a_target_stock)
    features.urgency = self.urgency or ""

    pm.send('longitudinalPlanSP', plan_sp_send)
