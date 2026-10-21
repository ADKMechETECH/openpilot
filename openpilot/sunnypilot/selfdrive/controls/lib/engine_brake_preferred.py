"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

EngineBrakePreferred (manual transmission + openpilot longitudinal, toggle default OFF) - Task 17: a per-gear clamp on
openpilot's decel request so the car slows on engine braking and the service brakes stay off.

Why a clamp works on this car (Stage 2, on-car): the radarless Civic sends one ACCEL_COMMAND and the car splits it
itself - any negative request closes the throttle (fuel cut), and the VSA brakes only for what the car doesn't already
achieve. So a request no stronger than the car's closed-throttle decel in the current gear never uses the brakes. The
clamp raises LongControl's lower accel limit (the FINAL command, all LoC states, PID anti-windup) to

  floor = engine_brake_table.brake_free_floor(table.decel(gear, v), pitch)     trusted gear in 1-6
        = UNKNOWN_GEAR_FLOOR (0: coast only, no decel request)                  gear not trusted

with the grade from the calibrated pose (the VSA tracks the request downhill too, so a flat-road number alone would
brake on every descent). Task 16 found the previous behavior (a coast target only while the plan's demand was within
the estimated engine braking) changed nothing on this car - it already engine-brakes first for any such request. The
clamp is different: when the plan asks for MORE than engine braking (a lowered set speed, a descent, a lead in adaptive
mode) it does not hand off to the brakes; the car keeps slowing on engine braking alone.

Gear trust (owner, 2026-10-20): GEAR_MT (CarStateSP.gearRaw), trusted only when the clutch pedal switch reads fully up,
the drivetrain isn't decoupled (deprecated.clutchPressed: pedal off the top, GEAR_MT 0, or the carstate phantom-gear
guard), the engine isn't in phantom territory (< PHANTOM_RPM_MAX rpm below PHANTOM_MAX_SPEED) and the rpm/speed ratio
agrees with that gear within RATIO_TOL. Otherwise the floor is 0.

Never clamped - the plan's braking passes untouched, in the same control frame:
  - any Rule-5 urgency reason (braking_urgency: FCW, forceDecel, stop / hard request, close / closing / braking lead,
    invalid or stale plan / radar, stock AEB/FCW) - emergency braking always has full authority;
  - EBP's own lead check (TTC < EngineBrakeLeadTtc, closing >= EngineBrakeLeadClosingSpeed, close, lead braking) and a
    plan that wants less than EngineBrakeHardFloor anywhere in the next PLAN_HORIZON;
    these three start a RELEASE_HOLD so the clamp doesn't chatter back in;
  - a descent steeper than the gear's engine braking (floor weaker than engine_brake_table.FLOOR_MAX): the brakes
    are needed to hold set speed whatever the request, and stock does it with the least (see engine_brake_table);
  - overspeed: above set speed + OVERSPEED_MARGIN and not slowing (aEgo > OVERSPEED_DECEL) for OVERSPEED_TIME, released below set +
    OVERSPEED_MARGIN / 2 - a runaway backstop for the floor-0 case (neutral rolling downhill) and a badly wrong table.
    A car well over set that is slowing on engine braking (a lowered set speed) is not a runaway and stays clamped;
  - no calibrated pitch, no live clutch signal, LoC not in pid (stopping), slower than MIN_SPEED, longitudinal inactive.
The toggle and every EBP param latch only while openpilot is disengaged (Task 12 B3). It does not touch FCW, opendbc
safety, panda or the camera path: it only narrows the command range openpilot already has.

Logging: `engine_brake_preferred` on every active edge (reason), `engine_brake_clamp` on every binding edge (the plan's
target below the floor: the clamp is actually holding braking off) with gear / speed / floor / target / pitch, and the
binding time per engagement.
"""
import math

from opendbc.car import structs
from opendbc.car.structs import CLUTCH_PEDAL_UP
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import lead_urgent as _lead_urgent
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import infer_gear
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_table import FLOOR_MAX, UNKNOWN_GEAR_FLOOR, \
  EngineBrakeTable, brake_free_floor

TransmissionType = structs.CarParams.TransmissionType
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Params (registered in params_keys.h); every value is clamped so it can't be tuned into an unsafe range.
DEFAULT_HARD_FLOOR = -1.5          # m/s^2: a plan wanting less than this (now or within PLAN_HORIZON) is never clamped
HARD_FLOOR_RANGE = (-2.0, -0.3)
DEFAULT_LEAD_TTC = 6.0             # s
LEAD_TTC_RANGE = (3.0, 15.0)
DEFAULT_LEAD_CLOSING = 1.0         # m/s
LEAD_CLOSING_RANGE = (0.2, 3.0)

# Gear trust. The phantom constants mirror opendbc's honda carstate PHANTOM_RPM_MAX / PHANTOM_MAX_SPEED (pinned by a test).
RATIO_TOL = 0.05                   # rpm/speed within 5 % of the reported gear (Stage 1: 96.5-99.2 % of in-gear frames at 3 %)
PHANTOM_RPM_MAX = 1100.
PHANTOM_MAX_SPEED = 13.0

PLAN_HORIZON = 1.0                 # s of the plan's accel trajectory checked against the hard floor
RELEASE_HOLD = 1.5                 # s: after a lead / hard-floor / urgency release, no clamp for at least this long
HOLD_REASONS = ("urgency", "lead", "hard_floor")
OVERSPEED_MARGIN = 3.6             # m/s over set speed while NOT slowing (aEgo > OVERSPEED_DECEL): stop clamping
OVERSPEED_DECEL = -0.1             # m/s^2: slowing at least this much is not a runaway (e.g. after a set-speed drop)
OVERSPEED_TIME = 2.0               # s over the margin without slowing before it counts (the clamp needs ~0.5 s to bite)
MIN_SPEED = 5.0                    # m/s
# TODO(review): the learner's values (engine_brake_learning.py) are collected but not applied until the logs are
#  reviewed; switching this on makes table cells max(static, learned).
APPLY_LEARNED = False


def _read_float(params: Params, key: str, default: float, rng: tuple[float, float]) -> float:
  try:
    v = params.get(key, return_default=True)
    v = default if v is None else float(v)
  except (TypeError, ValueError):
    v = default
  if not math.isfinite(v):
    v = default
  return min(max(v, rng[0]), rng[1])


def lead_urgent(lead, v_ego: float, ttc_limit: float, closing_limit: float) -> bool:
  """radarState LeadData: True if this lead rules out clamping (the Rule-5 criteria with EBP's own, tunable TTC /
  closing thresholds - braking_urgency always applies the defaults on top)."""
  return _lead_urgent(lead, v_ego, ttc_limit, closing_limit)


def trusted_gear(gear_raw: int, clutch_pedal: int, decoupled: bool, rpm: float, v_ego: float) -> int:
  """The gear (1-6) GEAR_MT reports if every cross-check agrees, else 0."""
  if not 1 <= gear_raw <= 6 or clutch_pedal != CLUTCH_PEDAL_UP or decoupled:
    return 0
  if not (math.isfinite(rpm) and math.isfinite(v_ego)) or (rpm < PHANTOM_RPM_MAX and v_ego < PHANTOM_MAX_SPEED):
    return 0
  return gear_raw if infer_gear(rpm, v_ego, False, RATIO_TOL) == gear_raw else 0


class EngineBrakePreferred:
  def __init__(self, CP: structs.CarParams, params: Params | None = None, table: EngineBrakeTable | None = None):
    self.params = params or Params()
    self.available = CP.transmissionType == TransmissionType.manual and bool(CP.openpilotLongitudinalControl)
    self.table = table or EngineBrakeTable(apply_learned=APPLY_LEARNED)
    self._enabled_requested = False
    self.hard_floor = DEFAULT_HARD_FLOOR
    self.lead_ttc = DEFAULT_LEAD_TTC
    self.lead_closing = DEFAULT_LEAD_CLOSING
    self.read_params()
    self._latch()
    self._plan_horizon_n = sum(t <= PLAN_HORIZON for t in ModelConstants.T_IDXS)
    self.reset()

  def reset(self) -> None:
    self.active = False
    self.binding = False
    self.floor = 0.
    self.gear = 0
    self.hold_timer = 0.
    self.overspeed = False
    self.overspeed_timer = 0.
    self.binding_time = 0.
    self.reason = "inactive"

  def read_params(self) -> None:
    # 1 Hz; applied by _latch() only while openpilot is disengaged (Task 12 B3)
    self._enabled_requested = self.available and self.params.get_bool("EngineBrakePreferred")
    self._requested = (_read_float(self.params, "EngineBrakeHardFloor", DEFAULT_HARD_FLOOR, HARD_FLOOR_RANGE),
                       _read_float(self.params, "EngineBrakeLeadTtc", DEFAULT_LEAD_TTC, LEAD_TTC_RANGE),
                       _read_float(self.params, "EngineBrakeLeadClosingSpeed", DEFAULT_LEAD_CLOSING, LEAD_CLOSING_RANGE))

  def _latch(self) -> None:
    self.enabled = self._enabled_requested
    self.hard_floor, self.lead_ttc, self.lead_closing = self._requested

  def _blocked(self, CS, plan, radar, urgency: str | None, long_control_state, pitch: float | None,
               clutch_valid: bool) -> str | None:
    """Reason the clamp is ruled out this frame, or None. Rule 5 first."""
    if urgency is not None:
      return "urgency"
    if lead_urgent(radar.leadOne, CS.vEgo, self.lead_ttc, self.lead_closing) or \
       lead_urgent(radar.leadTwo, CS.vEgo, self.lead_ttc, self.lead_closing):
      return "lead"
    if plan.aTarget < self.hard_floor or any(a < self.hard_floor for a in list(plan.accels)[:self._plan_horizon_n]):
      return "hard_floor"
    if not clutch_valid:
      return "no_clutch_signal"
    if pitch is None or not math.isfinite(pitch):
      return "no_pitch"
    if long_control_state != LongCtrlState.pid:
      return "stopping"
    if CS.vEgo < MIN_SPEED:
      return "slow"
    v_set = CS.vCruise / 3.6
    runaway = CS.vEgo > v_set + OVERSPEED_MARGIN and CS.aEgo > OVERSPEED_DECEL
    self.overspeed_timer = self.overspeed_timer + DT_CTRL if runaway else 0.
    if not self.overspeed and self.overspeed_timer > OVERSPEED_TIME:
      self.overspeed = True
      cloudlog.event("engine_brake_preferred_overspeed", over=round(CS.vEgo - v_set, 2), gear=self.gear)
    elif self.overspeed and CS.vEgo < v_set + OVERSPEED_MARGIN / 2:
      self.overspeed = False
    if self.overspeed:
      return "overspeed"
    return None

  def _set_active(self, active: bool, reason: str) -> None:
    if active != self.active:
      cloudlog.event("engine_brake_preferred", active=active, reason=reason, gear=self.gear, floor=round(self.floor, 3))
    if not active:
      self._set_binding(False, None, None)
    self.active = active
    self.reason = reason

  def _set_binding(self, binding: bool, CS, plan, pitch: float | None = None) -> None:
    if binding != self.binding:
      fields = {"binding": binding, "gear": self.gear, "floor": round(self.floor, 3), "t_bound": round(self.binding_time, 2)}
      if CS is not None:
        fields.update(v_ego=round(CS.vEgo, 2), a_target=round(plan.aTarget, 3), a_ego=round(CS.aEgo, 3),
                      pitch=round(pitch, 4) if pitch is not None else None)
      cloudlog.event("engine_brake_clamp", **fields)
    self.binding = binding

  def update(self, enabled: bool, long_active: bool, CS, plan, radar, urgency: str | None, long_control_state,
             pitch: float | None, clutch_valid: bool = True, gear_raw: int = -1,
             clutch_pedal: int = 0) -> bool:
    """Call every controlsd frame. Returns True while the clamp applies (limit_accel raises the lower limit).
    enabled: openpilot engaged (latching); urgency: braking_urgency(); gear_raw / clutch_pedal: CarStateSP (-1 / 0 when
    the clutch signal isn't valid)."""
    if not enabled:
      self._latch()   # never switches on/off, or re-tunes, mid-engagement
      if self.active:
        self._set_active(False, "disengaged")
      self.reset()
      return False
    if not self.enabled:
      return False

    self.gear = trusted_gear(gear_raw, clutch_pedal, CS.deprecated.clutchPressed, CS.deprecated.engineRpm, CS.vEgo)
    reason = "inactive" if not long_active else self._blocked(CS, plan, radar, urgency, long_control_state, pitch,
                                                                clutch_valid)
    if reason is not None:
      if reason in HOLD_REASONS:
        self.hold_timer = RELEASE_HOLD
      self._set_active(False, reason)
      return False
    if self.hold_timer > 0.:
      self.hold_timer = max(self.hold_timer - DT_CTRL, 0.)
      self._set_active(False, "hold")
      return False

    floor = brake_free_floor(self.table.decel(self.gear, CS.vEgo), pitch) if self.gear else UNKNOWN_GEAR_FLOOR
    if self.gear and floor > FLOOR_MAX:
      # a descent steeper than this gear's engine braking: the brakes are needed to hold speed anyway - stock
      self._set_active(False, "descent")
      return False
    self.floor = floor
    self._set_active(True, "clamping" if self.gear else "no_gear")
    binding = plan.aTarget < self.floor
    if binding:
      self.binding_time += DT_CTRL
    self._set_binding(binding, CS, plan, pitch)
    return True

  def limit_accel(self, accel_limits: tuple[float, float]) -> tuple[float, float]:
    """controlsd: raise LongControl's lower accel limit to the floor while the clamp applies. Only ever narrows."""
    # an upper limit already below the floor means something upstream demands braking: leave it alone
    if not self.active or accel_limits[1] < self.floor:
      return accel_limits
    return max(accel_limits[0], self.floor), accel_limits[1]
