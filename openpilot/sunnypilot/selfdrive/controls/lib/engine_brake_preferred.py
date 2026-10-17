"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

EngineBrakePreferred (manual transmission + openpilot longitudinal, toggle default OFF): for gentle, non-urgent decel
that lifting off and engine braking in the current gear can cover, don't command the service brakes — command ~coast
and let the engine do the slowing, the way a manual driver trims speed by hand. Anything steeper, or any sign of
urgency, gets openpilot's real (braking) command untouched: a hard hand-off, never a blend.

  a_eb      = engine_brake_model.estimate_coast_decel(gear, rpm, v_ego, grade)   (only part of it is credited, below)
  a_demand  = longitudinalPlan.aTarget
  suppress  = no backstop trigger  and  a_eb_credited <= a_demand < 0
  -> LongControl's lower accel limit is raised to the coast target (default 0.0), i.e. the FINAL command (all LoC
     states, PID anti-windup) can't go into the brake range. Clamping the plan alone would not work: the PID adds its
     own braking to track it.

Backstops — any one of them means NO suppression (and suppression stays off for RELEASE_HOLD afterwards):
  - lead present with closing speed >= EngineBrakeLeadClosingSpeed, or TTC < EngineBrakeLeadTtc, or closer than
    LEAD_MIN_HEADWAY, or decelerating harder than LEAD_DECEL (leadOne and leadTwo);
  - demanded decel (now, or anywhere in the plan's next PLAN_HORIZON) steeper than EngineBrakeHardFloor (-1.5);
  - openpilot FCW, stock AEB/FCW, driver-monitoring forceDecel, shouldStop / LoC not in pid;
  - invalid plan / radarState, no calibrated pitch, not in gear 1-6 (clutch in, neutral, no RPM, unknown ratio), slow;
  - capacity: while suppressing, the car isn't actually slowing as demanded — grossly (a_ego > a_demand + CAPACITY_TOL
    for CAPACITY_TIME) or persistently (the integrated shortfall, i.e. how much faster the car is now going than the
    plan wanted, exceeds CAPACITY_DV) — the model over-estimated engine braking, or the coast command doesn't coast on
    this car; brake,
    and stop suppressing for the rest of the engagement (the estimate isn't trusted again until re-engaged).
This is a comfort feature: every doubt resolves toward braking. It does not touch FCW, openpilot's accel limits,
opendbc safety or panda — it only narrows the command range openpilot already has.

Manual-only (the estimate is gear/RPM-based), openpilot-longitudinal only (pcmCruise -> no-op).

TODO(on-car): FIRST QUESTION — what a coast-target command actually does on the radarless Civic. Its ACC_CONTROL
 carries only ACCEL_COMMAND (an accel request; no separate gas command), and the car arbitrates throttle vs. brake
 itself. This design assumes command ~0 = closed-throttle coast (engine braking slows the car) and negative = service
 brakes. If ACCEL_COMMAND 0 instead HOLDS speed (PCM adds throttle), suppression can't slow the car: the capacity
 backstop then hands back to the brakes within ~CAPACITY_TIME and stops suppressing for the engagement (safe, but the
 feature does nothing) — and the coast target would have to follow the engine-brake level instead. If the car already
 prefers fuel-cut for small negative requests, this feature is unnecessary. Also interacts with opendbc PR #2050
 (radarless brake strength).
"""
import math

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import G, estimate_coast_decel, infer_gear

TransmissionType = structs.CarParams.TransmissionType
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Params (registered in params_keys.h); every value is clamped so it can't be tuned into an unsafe range.
# TODO(on-car): tune all of these by feel — and road-validate the lead-approach hand-off with wide margin first.
DEFAULT_HARD_FLOOR = -1.5          # m/s^2: demand steeper than this always goes to the brakes
HARD_FLOOR_RANGE = (-2.0, -0.3)
DEFAULT_COAST_TARGET = 0.0         # m/s^2: lowest command while suppressing (small negative = less "freewheel" feel)
COAST_TARGET_RANGE = (-0.3, 0.0)
DEFAULT_LEAD_TTC = 6.0             # s
LEAD_TTC_RANGE = (3.0, 15.0)
DEFAULT_LEAD_CLOSING = 1.0         # m/s
LEAD_CLOSING_RANGE = (0.2, 3.0)

# Fixed backstop constants
CAPACITY_MARGIN = 0.7              # credit only 70% of the estimated engine + road decel (gravity is added in full)
LEAD_MIN_HEADWAY = 1.0             # s: a lead closer than this is "close" regardless of closing speed
LEAD_MIN_GAP = 10.0                # m
LEAD_DECEL = -1.0                  # m/s^2: lead braking harder than this
PLAN_HORIZON = 1.0                 # s of the plan's accel trajectory checked against the hard floor
CAPACITY_TOL = 0.15                # m/s^2 (also catches a coast command that just holds speed instead of coasting)
CAPACITY_TIME = 0.75               # s
CAPACITY_DV = 0.25                 # m/s of accumulated speed shortfall vs. the demand
RELEASE_HOLD = 1.5                 # s: after a release or an urgent trigger, brake normally at least this long
URGENT_REASONS = ("lead", "hard_floor", "fcw_aeb", "force_decel", "capacity", "demand")
MIN_SPEED = 5.0                    # m/s


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
  """radarState LeadData: True if this lead rules out suppressing the brakes."""
  if not lead.present:
    return False
  closing = -lead.vRel
  if closing >= closing_limit:
    return True
  if closing > 0.1 and lead.dRel / closing < ttc_limit:
    return True
  if lead.dRel < max(LEAD_MIN_GAP, LEAD_MIN_HEADWAY * v_ego):
    return True
  return lead.aLeadK < LEAD_DECEL


class EngineBrakePreferred:
  def __init__(self, CP: structs.CarParams, params: Params | None = None):
    self.params = params or Params()
    self.available = CP.transmissionType == TransmissionType.manual and bool(CP.openpilotLongitudinalControl)
    self._enabled_requested = False
    self.hard_floor = DEFAULT_HARD_FLOOR
    self.coast_target = DEFAULT_COAST_TARGET
    self.lead_ttc = DEFAULT_LEAD_TTC
    self.lead_closing = DEFAULT_LEAD_CLOSING
    self.read_params()
    self.enabled = self._enabled_requested
    self._plan_horizon_n = sum(t <= PLAN_HORIZON for t in ModelConstants.T_IDXS)
    self.reset()

  def reset(self) -> None:
    self.suppress = False
    self.hold_timer = 0.
    self.capacity_timer = 0.
    self.capacity_failed = False
    self.capacity_dv = 0.
    self.a_eb = 0.
    self.a_eb_credited = 0.
    self.gear = 0
    self.reason = "inactive"

  def read_params(self) -> None:
    self._enabled_requested = self.available and self.params.get_bool("EngineBrakePreferred")
    self.hard_floor = _read_float(self.params, "EngineBrakeHardFloor", DEFAULT_HARD_FLOOR, HARD_FLOOR_RANGE)
    self.coast_target = _read_float(self.params, "EngineBrakeCoastTarget", DEFAULT_COAST_TARGET, COAST_TARGET_RANGE)
    self.lead_ttc = _read_float(self.params, "EngineBrakeLeadTtc", DEFAULT_LEAD_TTC, LEAD_TTC_RANGE)
    self.lead_closing = _read_float(self.params, "EngineBrakeLeadClosingSpeed", DEFAULT_LEAD_CLOSING, LEAD_CLOSING_RANGE)

  def _backstop(self, CS, plan, plan_valid: bool, radar, radar_valid: bool, force_decel: bool,
                long_control_state, pitch: float | None) -> str | None:
    """Reason suppression is ruled out this frame, or None."""
    if not plan_valid:
      return "plan_invalid"
    if not radar_valid:
      return "radar_invalid"
    if pitch is None or not math.isfinite(pitch):
      return "no_pitch"
    if force_decel:
      return "force_decel"
    if plan.fcw or CS.stockAeb or CS.stockFcw:
      return "fcw_aeb"
    if plan.shouldStop or long_control_state != LongCtrlState.pid:
      return "stopping"
    if CS.vEgo < MIN_SPEED:
      return "slow"
    if plan.aTarget < self.hard_floor or any(a < self.hard_floor for a in list(plan.accels)[:self._plan_horizon_n]):
      return "hard_floor"
    if lead_urgent(radar.leadOne, CS.vEgo, self.lead_ttc, self.lead_closing) or \
       lead_urgent(radar.leadTwo, CS.vEgo, self.lead_ttc, self.lead_closing):
      return "lead"
    if self.gear == 0:
      return "no_gear"
    if self.capacity_failed:
      return "capacity_failed"
    return None

  def _release(self, reason: str) -> None:
    # hold off re-entry after ending a suppression or after any urgency (no chatter, err toward braking); plain
    # "not applicable" reasons (no gear, slow, invalid input, LoC not in pid yet) don't start a hold
    if self.suppress or reason in URGENT_REASONS:
      self.hold_timer = RELEASE_HOLD
    if self.suppress:
      cloudlog.event("engine_brake_preferred", active=False, reason=reason, a_eb=round(self.a_eb, 3), gear=self.gear)
    self.suppress = False
    self.capacity_timer = 0.
    self.capacity_dv = 0.
    self.reason = reason

  def update(self, long_active: bool, CS, plan, plan_valid: bool, radar, radar_valid: bool, force_decel: bool,
             long_control_state, pitch: float | None) -> bool:
    """Call every controlsd frame. Returns True while service-brake commands are suppressed."""
    if not long_active:
      self.enabled = self._enabled_requested  # never switches on/off mid-engagement
      self.reset()
      return False
    if not self.enabled:
      return False

    self.gear = infer_gear(CS.deprecated.engineRpm, CS.vEgo, CS.deprecated.clutchPressed)
    grade = pitch if pitch is not None and math.isfinite(pitch) else 0.
    self.a_eb = estimate_coast_decel(self.gear, CS.deprecated.engineRpm, CS.vEgo, grade)
    # credit only part of the (uncalibrated) engine + road estimate; gravity in full (it's measured, not modeled)
    a_gravity = -G * math.sin(grade)
    self.a_eb_credited = max(CAPACITY_MARGIN * (self.a_eb - a_gravity) + a_gravity, self.hard_floor)

    reason = self._backstop(CS, plan, plan_valid, radar, radar_valid, force_decel, long_control_state, pitch)
    a_demand = plan.aTarget
    if reason is None and self.suppress:
      # capacity: the car must actually be slowing about as demanded while the brakes are held off
      self.capacity_timer = self.capacity_timer + DT_CTRL if CS.aEgo > a_demand + CAPACITY_TOL else 0.
      self.capacity_dv = max(self.capacity_dv + (CS.aEgo - a_demand) * DT_CTRL, 0.)
      if self.capacity_timer > CAPACITY_TIME or self.capacity_dv > CAPACITY_DV:
        reason = "capacity"
        self.capacity_failed = True
        msg = f"gear {self.gear}, a_eb {self.a_eb:.2f}, demand {a_demand:.2f}, a_ego {CS.aEgo:.2f}"
        cloudlog.warning(f"engine_brake_preferred: car not slowing as estimated ({msg}); suppression off until re-engaged")
    if reason is None and self.suppress and a_demand < self.a_eb_credited:
      reason = "demand"  # needs more than engine braking: hard hand-off to the real command

    if reason is not None:
      self._release(reason)
      return False

    if self.hold_timer > 0.:
      self.hold_timer = max(self.hold_timer - DT_CTRL, 0.)
      return False

    want = self.a_eb_credited <= a_demand < 0.
    if want and not self.suppress:
      cloudlog.event("engine_brake_preferred", active=True, a_demand=round(a_demand, 3), a_eb=round(self.a_eb, 3),
                     gear=self.gear)
      self.capacity_timer = self.capacity_dv = 0.
    self.suppress = want
    self.reason = "suppressing" if want else "idle"
    return self.suppress

  def limit_accel(self, accel_limits: tuple[float, float]) -> tuple[float, float]:
    """controlsd: raise LongControl's lower accel limit to the coast target while suppressing. Only ever narrows."""
    # an upper limit already below the coast target means something upstream demands braking: leave it alone
    if not self.suppress or accel_limits[1] < self.coast_target:
      return accel_limits
    return max(accel_limits[0], self.coast_target), accel_limits[1]
