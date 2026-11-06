"""
fork-only (civic-cruise) non-ACC approach warning: a chime + HUD text, NO action.

Owner decision 2026-11-06 (session 14, routes 00000056/57): "OP did not warn at all of an approaching vehicle. While I
don't want OP to act, we should still have the warning, about half the minimum distance it would normally trigger at."
Default ON, toggle NonAccApproachWarning to disable.

Why it is needed: non-ACC ignores leads by owner decision, and the only lead warning in the base is stock FCW
("BRAKE! Risk of Collision"), which fires when the lead MPC's own solution still ends within 0.25 m of the lead. That
solution assumes openpilot is braking for the lead; in non-ACC it never does, so stock FCW is later than it looks.
On the session 14 drives the car closed to a 0.47 s gap at 75 mph with no warning of any kind.

Trigger (owner: "yes, that's good enough for now"): the lead is inside FRACTION (half) of the gap ACC would hold at
its CLOSEST setting, and the gap is closing.

  ACC's desired gap (long_mpc: get_safe_obstacle_distance - get_stopped_equivalence_factor)
      = (v_ego^2 - v_lead^2) / (2 * COMFORT_BRAKE) + T_FOLLOW_MIN * v_ego + STOP_DISTANCE

The (v_ego^2 - v_lead^2) term makes it earlier the faster the car is closing. Replayed on the session 14 routes
(analysis/session14/approach_replay.py) it fires 4-5 s before each of the owner's cancels behind a lead.

Session 15 (route 0000005a, the first drive with it: 7 warnings at a 0.9-1.1 s gap). Owner 2026-11-06: "keep as-is but
increase it by 10 %, it's a little too tight. Suppress it if the gap is holding or increasing."
  - FRACTION 0.5 -> 0.55: the warning distance is 10 % longer.
  - The closing speed is now a 1 s low-pass of the lead's relative speed. The vision lead's vRel jumps by several m/s
    from frame to frame, so the raw value kept re-triggering while the owner held a steady 0.5-0.9 s gap in traffic
    (the text stayed up 14 s and 7 s), and delayed real warnings by 3-5 s when it happened to read "opening".
  - The warning ends as soon as that filtered gap is no longer closing (after MIN_SHOW), instead of a second after.

It never touches the plan, the accel limits or longActive: this module only says "warn now". Stock FCW is unchanged.
"""

T_FOLLOW_MIN = 1.25      # s: ACC's closest follow time (long_mpc.get_T_FOLLOW, aggressive personality)
COMFORT_BRAKE = 2.5      # m/s^2 (long_mpc.COMFORT_BRAKE)
STOP_DISTANCE = 6.0      # m     (long_mpc.STOP_DISTANCE)
FRACTION = 0.55          # warn inside this fraction of ACC's closest desired gap (owner: "about half", then +10 %)
CLOSING_MIN = 0.5        # m/s (~1 mph): the gap must be shrinking at least this fast (filtered) to start a warning
CLOSING_TAU = 1.0        # s: low-pass on the lead's relative speed (the vision vRel is noisy frame to frame)
LEAD_PROB_MIN = 0.5      # the vision lead's probability (radard's own "present" threshold)
MIN_SPEED = 4.5          # m/s (~10 mph): no warning in crawling traffic
ON_TIME = 0.3            # s the condition must hold before the warning starts
MIN_SHOW = 2.0           # s the warning stays up once started
OFF_TIME = 1.0           # s the condition must be gone before the warning ends (lead lost / outside the distance)
REARM_TIME = 1.0         # s after a warning ends before the next may start (with OFF_TIME: no new chime inside 2 s)


def acc_closest_gap(v_ego: float, v_lead: float) -> float:
  """The gap (m) lead-following ACC would hold at its closest setting, for these two speeds."""
  return (v_ego * v_ego - v_lead * v_lead) / (2. * COMFORT_BRAKE) + T_FOLLOW_MIN * v_ego + STOP_DISTANCE


def inside(v_ego: float, d_rel: float, v_lead: float) -> bool:
  return d_rel < FRACTION * acc_closest_gap(v_ego, v_lead)


def too_close(v_ego: float, d_rel: float, v_lead: float, v_rel: float) -> bool:
  """The trigger for one sample: inside the distance and closing (v_rel = the closing estimate, negative = closing)."""
  return v_rel < -CLOSING_MIN and inside(v_ego, d_rel, v_lead)


class ApproachWarning:
  def __init__(self, dt: float):
    self.dt = dt
    self.warning = False
    self._on_t = 0.
    self._off_t = 0.
    self._shown_t = 0.
    self._since_end = REARM_TIME
    self.v_rel_f = None      # low-passed relative speed of the lead (m/s, negative = closing); None = no lead yet

  def reset(self) -> None:
    self.warning = False
    self._on_t = self._off_t = self._shown_t = 0.
    self._since_end = REARM_TIME
    self.v_rel_f = None

  def update(self, active: bool, v_ego: float, lead_present: bool, d_rel: float, v_lead: float, v_rel: float,
             lead_prob: float) -> bool:
    """active: the toggle is on, non-ACC is the mode, openpilot is engaged and the driver is not on the gas.
    Returns True while the warning should be shown (the chime sounds once, when it starts)."""
    if not active:
      self.reset()
      return False

    lead = bool(lead_present and lead_prob >= LEAD_PROB_MIN)
    if not lead:
      self.v_rel_f = None                 # a new lead starts its own estimate
    elif self.v_rel_f is None:
      self.v_rel_f = float(v_rel)
    else:
      self.v_rel_f += (float(v_rel) - self.v_rel_f) * self.dt / (CLOSING_TAU + self.dt)
    cond = bool(lead and v_ego >= MIN_SPEED and too_close(v_ego, d_rel, v_lead, self.v_rel_f))
    # owner: "suppress it if the gap is holding or increasing"
    opening = bool(lead and self.v_rel_f >= 0.)

    if not self.warning:
      self._since_end += self.dt
      self._on_t = self._on_t + self.dt if cond else 0.
      if self._on_t >= ON_TIME and self._since_end >= REARM_TIME:
        self.warning = True
        self._shown_t = self._off_t = 0.
    else:
      self._shown_t += self.dt
      self._off_t = 0. if cond else self._off_t + self.dt
      if self._shown_t >= MIN_SHOW and (self._off_t >= OFF_TIME or opening):
        self.warning = False
        self._on_t = 0.
        self._since_end = 0.
    return self.warning
