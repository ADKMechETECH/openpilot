"""
fork-only (civic-cruise) ACC-mode engine-brake priority: when may the car brake for a lead?

Owner decisions 2026-11-06 (session 15):
  "Adaptive cruise should have the same coast to set speed and brake only if necessary" - always active.
  "We should still prioritize engine braking only unless we enter the threshold set in [the approach warning]. The
   idea is to anticipate the decel requirement at our normal (unmodified) minimum follow distance and coast on engine
   brake to decel. If we continue to close the gap and enter our margin, we can brake."
  "Yes, stopped car should always cause brakes."

So in the car's ACC mode the lead plan is left exactly as it is (it still plans to the normal follow distance, which is
what makes the car lift off early), but its braking only reaches the car - past controlsd's coast floor - when one of
these holds:
  - MARGIN:   a lead is inside the approach-warning distance (approach.FRACTION of ACC's closest gap), debounced;
  - STOPPED:  a lead is stopped or nearly (below STOPPED_SPEED);
  - HARD:     the plan needs more than HARD_BRAKE - a lead braking hard or a large speed difference, where waiting for
              the margin would leave more braking to do than the car has (a stopped car from 45 mph first seen at
              the normal gap needs 1.9 m/s^2; at the margin it would need 3.6);
and always for a stop request, an e2e plan, FCW and forceDecel (those are handled by the callers).
Each reason is held for a while after it clears so the brakes do not chatter against the plan's own feedback.

Pure logic, no messaging: plannerd calls update() at its own rate and publishes the resulting bound for controlsd.
"""
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.approach import LEAD_PROB_MIN, inside

STOPPED_SPEED = 2.5      # m/s (~5.5 mph): a lead this slow is a stopped car - always brake for it
HARD_BRAKE = -2.0        # m/s^2: the plan needs at least this much -> let it through at once
HARD_RELEASE = -1.0      # m/s^2: ... until it needs less than this for HOLD_TIME
MARGIN_ON_TIME = 0.2     # s inside the margin before braking is allowed (the vision lead distance jumps)
HOLD_TIME = 1.5          # s every reason is held after it clears


class LeadBrakeGate:
  def __init__(self, dt: float):
    self.dt = dt
    self._inside_t = 0.
    self._hold = 0.
    self._hard = False
    self.reason = ""       # why braking is allowed right now ("" = engine braking only), for logs / tests

  def reset(self) -> None:
    self._inside_t = self._hold = 0.
    self._hard = False
    self.reason = ""

  def update(self, v_ego: float, leads, need: float) -> bool:
    """leads: iterable of (present, prob, d_rel, v_lead). need: the accel the lead plan asks for now (m/s^2).
    Returns True while the plan's braking for a lead may pass the coast floor."""
    live = [(d, vl) for present, prob, d, vl in leads if present and prob >= LEAD_PROB_MIN]
    stopped = any(vl < STOPPED_SPEED for _, vl in live)
    in_margin = any(inside(v_ego, d, vl) for d, vl in live)
    self._inside_t = self._inside_t + self.dt if in_margin else 0.

    if need <= HARD_BRAKE:
      self._hard = True
    elif need > HARD_RELEASE:
      self._hard = False

    if stopped:
      now = "stopped"
    elif self._inside_t >= MARGIN_ON_TIME or (in_margin and self.reason == "margin"):
      now = "margin"       # debounced on the way in; while it is held, any sample inside renews it (noisy distance)
    elif self._hard:
      now = "hard"
    else:
      now = ""
    if now:
      self._hold = HOLD_TIME
      self.reason = now
    else:
      self._hold = max(self._hold - self.dt, 0.)
      if self._hold <= 0.:
        self.reason = ""
    return bool(self.reason)
