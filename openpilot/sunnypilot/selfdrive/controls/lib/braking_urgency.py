"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Rule 5 — ONE braking-urgency predicate for every fork feature that pauses longitudinal, raises the lower accel limit or
rate-limits braking (clutch gate, smart takeover, low-speed handback, no-service-brake maintain/pulse, engine-brake-
preferred, gear-aware accel, non-adaptive). If braking_urgency() returns a reason, the feature steps aside within the
same control frame: the lower accel limit goes back to stock and the plan's braking passes through untouched.

Reasons, most severe first:
  invalid_plan / invalid_radar  the plan or radarState fails sm.all_checks() (alive, frequency, valid) — stale counts
  force_decel                   driver-monitoring / soft-disable forced decel
  fcw                           openpilot FCW (longitudinalPlan.fcw / the planner's own fcw)
  stock_aeb                     stock AEB/FCW flags (dead while openpilot long runs on radarless Bosch: the camera's
                                0x1C8 is blocked; kept for other platforms)
  stopping                      the plan asks to stop
  hard_decel                    the plan asks for more decel than URGENT_DECEL
  lead_ttc / lead_close / lead_braking / lead_closing   a close or closing lead (the Task 6 lead_urgent() criteria)

The lead criteria are deliberately broad (any lead closing >= 1 m/s, at any distance, counts): for a comfort feature,
stepping aside means "brake normally", which is always safe. collision_threat() is the narrower set non-adaptive mode
re-admits the lead MPC on (owner decision 2026-10-19: slow only when braking is actually needed for a real slow vehicle
or a collision — never ordinary car-following): FCW / stock AEB, a lead inside the TTC window, a close lead that is
also closing (or braking), a lead braking hard within COLLISION_BRAKING_MAX_HEADWAY, or the lead MPC itself demanding
more than MPC_NEEDED_DECEL. A slower car far ahead, or steady following with no closing, is not a threat.

handback_urgency() is the manual low-speed handback's own, narrower carve-out (Task 13 A1): the handback exists so the
driver takes the final stop, so an ordinary approach to slow or stopped traffic must NOT freeze it (the broad lead
reasons did: a stopped car 40 m ahead held the manual at a standstill, engaged, for good). It keeps every non-lead
reason except "stopping" (the plan asking to stop is exactly what the handback hands over), and replaces the lead
reasons with one kinematic test: the lead is imminent when a driver who starts braking only after the handback
(ramp + margin + reaction) at a moderate HANDBACK_DRIVER_DECEL could not stop HANDBACK_MARGIN short of it.
"""
import math

URGENT_DECEL = -1.5          # m/s^2: a plan asking for more decel than this is urgent
LEAD_TTC = 6.0               # s
LEAD_CLOSING = 1.0           # m/s
LEAD_MIN_HEADWAY = 1.0       # s: a lead closer than this is "close"
LEAD_MIN_GAP = 10.0          # m
LEAD_DECEL = -1.0            # m/s^2: lead braking harder than this

INVALID_REASONS = ("invalid_plan", "invalid_radar")
LEAD_REASONS = ("lead_ttc", "lead_close", "lead_braking", "lead_closing")
# a collision threat, not just "a slower car somewhere ahead"
COLLISION_REASONS = ("fcw", "stock_aeb", "lead_ttc", "lead_close", "lead_braking")
COLLISION_CLOSE_MIN_CLOSING = 0.3   # m/s: "close" is a collision threat only while closing (or the lead is braking)
COLLISION_BRAKING_MAX_HEADWAY = 3.0  # s: a lead braking hard is a threat when within this headway
MPC_NEEDED_DECEL = -1.0              # m/s^2: the lead MPC asking for more than this means braking is really needed
# Task 13 A1: the low-speed handback's own lead test (handback_lead_reason)
HANDBACK_REASONS = ("invalid_plan", "invalid_radar", "force_decel", "fcw", "stock_aeb", "hard_decel")  # kept from R5
HANDBACK_DRIVER_REACTION = 1.5       # s: from the handback prompt to the driver braking (on top of ramp + margin)
HANDBACK_DRIVER_DECEL = 2.0          # m/s^2: a moderate driver stop
HANDBACK_MARGIN = 2.0                # m: the driver's stop must end at least this far short of the lead
HANDBACK_RELEASE_MARGIN = 2.0        # m: Task 14 A1 - a latched lead stays "holding" until it is this much past imminent


def lead_reason(lead, v_ego: float, ttc_limit: float = LEAD_TTC, closing_limit: float = LEAD_CLOSING) -> str | None:
  """radarState LeadData -> the lead urgency reason, or None (Task 6's lead_urgent criteria, split by cause)."""
  if not lead.present:
    return None
  closing = -lead.vRel
  if closing > 0.1 and lead.dRel / closing < ttc_limit:
    return "lead_ttc"
  if lead.dRel < max(LEAD_MIN_GAP, LEAD_MIN_HEADWAY * v_ego):
    return "lead_close"
  if lead.aLeadK < LEAD_DECEL:
    return "lead_braking"
  if closing >= closing_limit:
    return "lead_closing"
  return None


def lead_urgent(lead, v_ego: float, ttc_limit: float = LEAD_TTC, closing_limit: float = LEAD_CLOSING) -> bool:
  return lead_reason(lead, v_ego, ttc_limit, closing_limit) is not None


def collision_threat(lead, v_ego: float) -> str | None:
  """The subset of a lead's urgency that is a collision threat (non-adaptive re-admits the lead MPC on these)."""
  reason = lead_reason(lead, v_ego)
  if reason == "lead_close":
    closing = -lead.vRel
    return reason if closing > COLLISION_CLOSE_MIN_CLOSING or lead.aLeadK < LEAD_DECEL else None
  if reason == "lead_braking":
    return reason if lead.dRel < COLLISION_BRAKING_MAX_HEADWAY * max(v_ego, 1.) else None
  return reason if reason in COLLISION_REASONS else None


def handback_lead_reason(lead, v_ego: float, reaction: float, extra: float = 0.) -> str | None:
  """'lead_imminent' if a driver braking at HANDBACK_DRIVER_DECEL after `reaction` s could not stop HANDBACK_MARGIN
  (+ extra) short of this lead: with the lead holding its speed (closing kinematics), or braking to a stop at its current
  decel."""
  if not lead.present:
    return None
  room = lead.dRel - HANDBACK_MARGIN - extra
  closing = -lead.vRel
  if closing > 0. and closing * reaction + closing ** 2 / (2. * HANDBACK_DRIVER_DECEL) > room:
    return "lead_imminent"
  if lead.aLeadK < -0.1:
    lead_stop = max(lead.vLead, 0.) ** 2 / (2. * -lead.aLeadK)
    v = max(v_ego, 0.)
    if v * reaction + v ** 2 / (2. * HANDBACK_DRIVER_DECEL) > room + lead_stop:
      return "lead_imminent"
  return None


def handback_lead_imminent(radar, v_ego: float, reaction: float) -> bool:
  """Task 14 A2: the imminent-lead test on its own, whatever reason handback_urgency() returns (FCW / hard_decel used to
  hide it, so the latch never set during a hard-braking approach)."""
  return any(handback_lead_reason(lead, v_ego, reaction) is not None for lead in (radar.leadOne, radar.leadTwo))


def handback_lead_holding(radar, v_ego: float, reaction: float) -> bool:
  """Task 14 A1: a latched lead still holds the braking while some lead is closing (vRel < 0) and still within the
  imminent test plus HANDBACK_RELEASE_MARGIN. No lead, or every lead opening or well clear -> release (debounced)."""
  return any(lead.present and lead.vRel < 0. and
             handback_lead_reason(lead, v_ego, reaction, HANDBACK_RELEASE_MARGIN) is not None
             for lead in (radar.leadOne, radar.leadTwo))


def handback_urgency(urgency: str | None, radar, v_ego: float, reaction: float) -> str | None:
  """The handback's carve-out (Task 13 A1), from this frame's braking_urgency() reason: the non-lead reasons in
  HANDBACK_REASONS pass through (and take precedence, so FCW / stock AEB keep their unconditional handling); "stopping"
  and the broad lead reasons do not; an imminent lead always counts. forceDecel is in the set, but the handback aborts
  on it before it looks at urgency. The latch uses handback_lead_imminent() separately (Task 14 A2)."""
  if urgency in HANDBACK_REASONS:
    return urgency
  return "lead_imminent" if handback_lead_imminent(radar, v_ego, reaction) else None


def braking_urgency(*, plan_ok: bool, radar_ok: bool, force_decel: bool, fcw: bool, stock_aeb: bool, should_stop: bool,
                    a_target: float, v_ego: float, radar, plan_live: bool = True) -> str | None:
  """The reason braking must not be limited this frame, or None.
  plan_live: the plan was computed while longitudinal was active. While it is off the planner re-seeds its output from
  the car's own aEgo, so a coasting car's decel (a hill, the driver's braking) would read as a "hard decel" / "stop"
  request: those two plan-derived reasons only count from a live plan. FCW, leads, forceDecel, invalid inputs always do."""
  if not plan_ok:
    return "invalid_plan"
  if not radar_ok:
    return "invalid_radar"
  if force_decel:
    return "force_decel"
  if fcw:
    return "fcw"
  if stock_aeb:
    return "stock_aeb"
  if not math.isfinite(a_target):
    return "hard_decel"
  if plan_live and should_stop:
    return "stopping"
  if plan_live and a_target < URGENT_DECEL:
    return "hard_decel"
  for lead in (radar.leadOne, radar.leadTwo):
    reason = lead_reason(lead, v_ego)
    if reason is not None:
      return reason
  return None


def force_decel_from(sm) -> bool:
  """controlsd's own definition (controlsState.forceDecel): DM no-response decel or soft-disabling."""
  from openpilot.cereal import log
  return bool(sm['driverMonitoringState'].noResponseForceDecel or
              sm['selfdriveState'].state == log.SelfdriveState.OpenpilotState.softDisabling)


def controls_urgency(sm, plan_live: bool = True) -> str | None:
  """controlsd: urgency from the plan it is about to execute. sm.all_checks, not .valid: stale counts as invalid.
  plan_live: LongControl was active (not off) when this plan was made - see braking_urgency()."""
  CS, plan = sm['carState'], sm['longitudinalPlan']
  return braking_urgency(plan_ok=sm.all_checks(['longitudinalPlan']), radar_ok=sm.all_checks(['radarState']),
                         force_decel=force_decel_from(sm), fcw=plan.fcw, stock_aeb=CS.stockAeb or CS.stockFcw,
                         should_stop=plan.shouldStop, a_target=plan.aTarget, v_ego=CS.vEgo, radar=sm['radarState'],
                         plan_live=plan_live)
