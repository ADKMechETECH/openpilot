"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.mode import NonAccMode
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.acc_coast import LeadBrakeGate
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.approach import ApproachWarning
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.learner import EngineBrakeLearner

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

    # fork-only (civic-cruise): non-ACC ("set-speed only") mode = the car's own conventional-cruise mode
    # (carState.cruiseState.nonAdaptive). Followed live (no latch) - the ACC<->non-ACC flip is smoothed by the
    # consumers. The planner uses it to drop the lead-following MPC (and e2e) candidates.
    self.nonacc = NonAccMode(CP, DT_MDL)
    self.non_adaptive = self.nonacc.active

    # fork-only (civic-cruise): engine-brake learner (manual + op long only). Learns from brake-free coasts, persists
    # the applied learned table for the controller (controlsd) to read.
    self.manual = CP.transmissionType == structs.CarParams.TransmissionType.manual
    self.eb_learner = EngineBrakeLearner(self.manual and bool(CP.openpilotLongitudinalControl), DT_MDL)

    # fork-only (civic-cruise, Task 16 port): gas-override re-surge guard (always on for manuals)
    self.gas_override_reset = False

    # fork-only (civic-cruise): ACC-mode engine-brake priority (see update_coast_floor_bound). The default bound is
    # the minimum accel = "no floor": nothing is assumed until the planner has run.
    self.lead_brake_gate = LeadBrakeGate(DT_MDL)
    self.coast_floor_bound = -10.

    # fork-only (civic-cruise): non-ACC approach warning (chime + text only; owner 2026-11-06, default ON)
    self.approach = ApproachWarning(DT_MDL)
    self._approach_params = Params()
    self.approach_enabled = self._approach_params.get_bool("NonAccApproachWarning")
    self._approach_frame = 0

    self.output_v_target = 0.
    self.output_a_target = 0.

  def update_gas_override(self, sm: messaging.SubMaster, reset_state: bool) -> None:
    """Remembers that the current planner reset is a driver gas override while engaged; stays set through the lift
    until the planner stops resetting (longitudinal back), so the first plans after the lift see it too."""
    if not reset_state:
      self.gas_override_reset = False
    elif bool(sm['selfdriveState'].enabled) and sm['carState'].gasPressed:
      self.gas_override_reset = True

  def reset_accel(self, a_ego: float) -> float:
    """The accel the planner re-seeds from. Manual + gas override: coast first (never above 0) so openpilot does not
    re-open the throttle the driver just closed. Automatics and every other reset: stock (aEgo)."""
    return min(a_ego, 0.) if self.manual and self.gas_override_reset else a_ego

  def update_non_adaptive(self, sm: messaging.SubMaster) -> None:
    # fork-only: refresh the non-ACC mode once per planner frame (live flip, no latch).
    self.non_adaptive = self.nonacc.update(bool(sm['carState'].cruiseState.nonAdaptive))

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    # non-ACC never uses the e2e/experimental candidate (it brakes to stops and launches; a manual can't use that, and
    # non-ACC is set-speed only by definition).
    if self.non_adaptive:
      return False
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
    self.update_eb_learner(sm)
    self.update_approach_warning(sm)

  def update_coast_floor_bound(self, sm: messaging.SubMaster, lead_accel: float | None, should_stop: bool, is_e2e: bool,
                               accel_min: float, accel_max: float) -> None:
    """fork-only (civic-cruise): the lowest accel controlsd's coast floor must let through in the car's ACC mode.
    lead_accel: what the lead MPC asks for now (None in non-ACC, where the bound is not used).
      - a stop is wanted, an e2e plan is driving, or one of sunnypilot's own speed targets (curves, speed limit) is
        what limits the speed: the minimum accel = no floor at all (stock braking);
      - the lead gate allows braking (inside the approach margin, a stopped car, or the plan needs hard braking):
        the lead plan's own accel - its braking passes in full;
      - otherwise: the maximum accel = engine braking only (owner 2026-11-06)."""
    if lead_accel is None or should_stop or is_e2e or self.source != LongitudinalPlanSource.cruise:
      self.lead_brake_gate.reset()
      self.coast_floor_bound = float(accel_min)
      return
    rs = sm['radarState']
    leads = [(bool(ld.present), float(ld.modelProb), float(ld.dRel), float(ld.vLead)) for ld in (rs.leadOne, rs.leadTwo)]
    need = float(min(max(lead_accel, accel_min), accel_max))
    if self.lead_brake_gate.update(float(sm['carState'].vEgo), leads, need):
      self.coast_floor_bound = need
    else:
      self.coast_floor_bound = float(accel_max)

  def update_approach_warning(self, sm: messaging.SubMaster) -> None:
    # fork-only (civic-cruise): warn (never act) when non-ACC cruise is closing inside half of ACC's closest gap.
    if self._approach_frame % int(1. / DT_MDL) == 0:
      self.approach_enabled = self._approach_params.get_bool("NonAccApproachWarning")
    self._approach_frame += 1
    CS = sm['carState']
    lead = sm['radarState'].leadOne
    active = bool(self.approach_enabled and self.nonacc.available and CS.cruiseState.nonAdaptive
                  and sm['selfdriveState'].enabled and not CS.gasPressed)
    if self.approach.update(active, float(CS.vEgo), bool(lead.present), float(lead.dRel), float(lead.vLead),
                            float(lead.vRel), float(lead.modelProb)):
      self.events_sp.add(custom.OnroadEventSP.EventName.nonAccApproach)

  def update_eb_learner(self, sm: messaging.SubMaster) -> None:
    # fork-only (civic-cruise): feed the engine-brake learner. Learns only from brake-free coasts (disengaged, no gas /
    # brake, trusted gear, closed throttle); the gate lives in the learner.
    CS = sm['carState']
    cs_sp = sm['carStateSP']
    gear = cs_sp.gearRaw if (cs_sp.clutchSignalValid and not CS.deprecated.clutchPressed
                             and 1 <= cs_sp.gearRaw <= 6) else None
    orientation = sm['carControl'].orientationNED
    pitch = orientation[1] if len(orientation) == 3 else None
    self.eb_learner.observe(engaged=bool(sm['selfdriveState'].enabled), gas=bool(CS.gasPressed),
                            brake_switch=bool(CS.brakePressed), user_brake=float(CS.deprecated.brake), gear=gear,
                            torque_axle=float(cs_sp.engineTorqueAxle), v_ego=float(CS.vEgo), a_ego=float(CS.aEgo),
                            pitch=pitch)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.coastFloorBound = float(self.coast_floor_bound)
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

    pm.send('longitudinalPlanSP', plan_sp_send)
