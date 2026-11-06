"""
fork-only (civic-cruise) non-ACC overspeed controller (controlsd side).

Only ever RAISES LongControl's lower accel limit (never lowers it), and only above the set speed. It implements goals
1b/1c/1d of the non-ACC mode:
  - COASTING (set < v < set + trigger): raise the lower limit to the brake-free floor so the car prefers engine braking
    and the service brakes stay off. On the flat this engine-brakes back to set; on a descent steeper than engine
    braking it drifts up toward the trigger (the brakes never join: the request is the car's own coast accel, positive
    on such a descent, up to COAST_CAP).
  - PULSING (v >= set + trigger): a firm but bounded brake (PULSE_DECEL, ramped at PULSE_JERK) down to within RELEASE
    (3 mph) of set, then release -> back to COASTING. Repeats = sawtooth.
  - Steps aside (returns the incoming limit unchanged -> stock/plan braking) on: forceDecel, above the hard ceiling,
    no calibrated pitch, a gear that is untrusted for any reason other than a live clutch-in (clutch signal dead,
    reverse), standstill, or when the mode / longitudinal is off.
  - Clutch in / neutral with the clutch signal live (session 14, owner: "the clutch-in behaviour is not good"): the
    floor STAYS, at the declutched car's own accel (road load + grade, no gear needed). Before, the controller stepped
    aside there and the planner's request went from +0.05 to -1.20 m/s^2 in one frame on a downshift above set.
    On a descent that accel is positive and is requested as such (bounded by COAST_CAP), so the VSA does not hold
    the declutched car back either.
  - Coming back from a step-aside (gas override released, mode switched, hard ceiling left) the floor restarts from
    the last commanded accel, not from the raw lower limit: no ~2 s window in which the planner brakes unfloored
    (session 14: -1.19 m/s^2 on releasing the gas 6.8 mph over set), and a release of real braking (ACC -> non-ACC
    while ACC brakes for a lead) is still ramped at PULSE_JERK.

  - Session 15 (owner 2026-11-06: "the car braked for small decel with non-ACC set improperly", seen as the brake
    lights on the cluster): the car's own flag VSA_STATUS.COMPUTER_BRAKING (0x1A4 bit 23) was on at 23 set-speed taps
    on route 0000005a, 18 s in all, and 26 s + 8 s on the two session 14 drives - with almost no brake force (the
    accelerometer shows <= 0.15 m/s^2), which is why the earlier checks called those taps brake-free. Measured over the
    three drives (analysis/session15/flag_margin.py): the car arms its brakes when the request is closer than about
    0.25 m/s^2 to its own closed-throttle accel (flag on for 35-80 % of the time below +0.10, 6 % at +0.20...0.25,
    0.4 % above +0.30, never above +0.40), and far more readily when the request falls fast (-0.6...-0.9 m/s^3 after a
    tap: 50-90 % inside +0.2; slower than -0.4 m/s^3: 0 of 6 at or above the coast). The throttle is already closed in
    much of that band. The margin the car needs grows with how hard it coasts (flag on, by coast accel and margin:
    descent > +0.05: 9 % at 0, 3 % at +0.1; -0.25...-0.10: 21 % at +0.1, 3 % at +0.2; flat -0.45...-0.25: 67 % at
    +0.1, 14 % at +0.2, 1 % at +0.3; -0.70...-0.45: 35 % at +0.2, 10 % at +0.3, 0 above +0.35; climbs: 38 % at +0.3,
    0 above +0.35). So the coasting floor now
      * sits coast_margin(coast) above the estimated coast accel: 0.10 - 0.40 * coast, between 0.05 and 0.45 - e.g.
        6th at 68 mph on the flat (coast -0.56) asks for -0.24 - and never asks for more than DESCENT_MARGIN above
        max(coast, 0): above set speed the car is not powered beyond a whisper,
      * lets the request fall below 0 no faster than COAST_FALL_RATE while coasting (a tap eases the throttle shut
        instead of dropping the request), and
      * is trimmed up while the car reports COMPUTER_BRAKING outside a pulse (bounded, slow decay): the car's own flag
        is the final word on whether it is braking.
  - ACC mode (owner 2026-11-06: "adaptive cruise should have the same coast to set speed and brake only if necessary",
    always active; "prioritize engine braking only unless we enter the [approach-warning] threshold"; "stopped car
    should always cause brakes"): the same floor applies in the car's ACC mode, never above lead_accel_limit. plannerd
    sets that bound (acc_coast.py): the maximum accel while the lead is to be handled on engine braking, the lead
    plan's own accel once braking is allowed (inside the approach margin, a stopped car, or the plan needs
    HARD_BRAKE), the minimum accel for a stop. Braking that is allowed comes in at ACC_RELEASE_JERK, and at once when
    the plan needs HARD_BRAKE or more.
  - A SET press arrives one frame before the new set speed (session 15, twice): the old set speed could be >= 8 mph
    under the car, which started a pulse for that one frame and seeded the floor at -1.0 for 0.8 s. The pulse now needs
    the overspeed for PULSE_CONFIRM.

  - Low-speed hold (owner 2026-11-06, "low speed non-ACC is twitchy on and off the throttle"; A/B on the road):
    the PCM cuts fuel whenever the request is within ~0.35 m/s^2 of the car's closed-throttle coast and re-fuels only
    ~0.5 above it (analysis/session15/lowspeed_twitch.py), so what the car can deliver has a hole: full engine braking,
    or >= ~0.5 above it. Where holding speed needs something inside the hole (a slight descent, a tall gear at low
    speed) no steady state exists and the speed loop hunted across the edge every 2-3 s (6-19 throttle shut-offs a
    minute at 35-55 mph, 0.2 above 60 mph). Inside the hole the controller now holds the speed in a band instead:
    DRIVING (request pinned >= coast + HOLD_DRIVE, the engine stays fuelled, the speed rises) to the top of the band,
    then COASTING (request pinned at coast + HOLD_COAST: fuel cut and stays cut, brakes not armed) to the bottom, and
    so on - one clean transition per band crossing. NonAccHoldBand = the band in mph (default 4), NonAccHoldBias =
    the share of it above the set speed (0.5 centred = "A", 1.0 all above set = "B"), both live.

Leads: non-ACC ignores them by design (the driver brakes; FCW and the approach warning warn). In ACC mode the lead plan
is the hard upper bound on this controller's floor.
"""
import math

import numpy as np

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.mode import NonAccMode
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.engine_brake import G, declutched_coast_accel
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.learner import effective_decel, load_applied

COASTING, PULSING = 0, 1

RELEASE = 1.34            # m/s over set (~3 mph): release the pulse (goal 1d "within 3 mph")
HARD_CEILING = 6.7       # m/s over set (~15 mph): stop clamping, full plan braking + alert
PULSE_DECEL = -1.0       # m/s^2: firm but not aggressive
PULSE_JERK = 1.5         # m/s^3: smooth pulse on/off
# m/s^2: upper bound on the (downhill) coast-follow request. Was 0.3 until session 14: on the 12-17 % descent in 2nd the
# car's own closed-throttle accel was above that, so the VSA braked all the way down to hold the request at +0.3 and the
# sawtooth pulses came on top (grade-corrected decel 0.9-1.4 m/s^2 against 0.73 from brake-free coasts). Owner
# 2026-11-06: send the car's real coast accel - brakes fully off between pulses, the car reaches the trigger sooner,
# one firm pulse, coast again. 1.0 covers ~17 % in 2nd and ~15 % in the upper gears and still bounds a bad pitch /
# table estimate (a request above the true coast is a whisper of throttle, bounded by the sawtooth trigger).
COAST_CAP = 1.0
# The coasting floor sits coast_margin() above the estimated coast accel (session 15, see above): m/s^2
MARGIN_BASE = 0.10       # at a coast accel of 0
MARGIN_SLOPE = 0.40      # more margin the harder the car coasts (per m/s^2 of coast decel)
MARGIN_MIN = 0.05
MARGIN_MAX = 0.45
DESCENT_MARGIN = 0.05    # the floor never exceeds max(coast, 0) by more than this (+ a share of the flag trim)
TRIM_DESCENT_SHARE = 0.4
COAST_FALL_RATE = 0.4    # m/s^3: while coasting the request may fall below 0 no faster than this
TRIM_RISE = 1.0          # m/s^3: how fast the floor is raised while the car reports COMPUTER_BRAKING outside a pulse
TRIM_DECAY = 0.03        # m/s^3: how fast that trim fades once the flag is off (0.25 in ~8 s)
TRIM_MAX = 0.25          # m/s^2
TRIM_HOLDOFF = 1.5       # s after a pulse ends before the flag may trim (the car is still releasing real braking)
PULSE_CONFIRM = 0.1      # s the overspeed must exceed the trigger before a pulse starts (one stale frame at SET)
# low-speed hold (see above): m/s^2 above the estimated coast accel
HOLE_ENTER = 0.35        # hold margin (engine braking + road load + grade) below which the car hunted (10 shut-offs/min under 0.30,
HOLE_LEAVE = 0.45        # 2.6 at 0.30-0.45, none above); leave above this (hysteresis on the estimate)
HOLD_COAST = 0.30        # coasting side: the PCM cuts fuel below ~0.35 and the brakes arm below ~0.25
HOLD_DRIVE = 0.60        # driving side: the PCM re-fuels at ~0.5; the estimate is good to about +-0.08
HOLD_JERK = 0.5          # m/s^3: the floor moves between the two sides at this rate
HOLD_BAND_RANGE = (0.9, 2.7)   # m/s (2-6 mph)
HOLD_OFF, HOLD_DRIVING, HOLD_COASTING = 0, 1, 2
ACC_RELEASE_JERK = 2.5   # m/s^3: ACC mode, how fast the floor gives way to lead braking that has just been allowed
ACC_HARD_BRAKE = -2.0    # m/s^2: ... unless the plan needs this much or more - then at once (acc_coast.HARD_BRAKE)
MIN_SPEED = 2.0          # m/s: below this the coast estimate is unreliable, step aside
DEFAULT_TRIGGER = 3.6    # m/s over set (~8 mph): start the firm brake pulse (user-settable)
TRIGGER_MIN = RELEASE + 0.5
TRIGGER_MAX = HARD_CEILING - 0.45   # keep RELEASE < trigger < HARD_CEILING


def coast_margin(coast: float) -> float:
  """How far above the car's own closed-throttle accel the request must stay for the car not to arm its brakes."""
  return float(np.clip(MARGIN_BASE - MARGIN_SLOPE * coast, MARGIN_MIN, MARGIN_MAX))


def coast_floor(coast: float, trim: float = 0., declutched: bool = False) -> float:
  """The coasting request for an estimated coast accel: the margin above it, bounded so the car is never powered
  above set speed by more than a whisper (DESCENT_MARGIN over max(coast, 0)), plus the COMPUTER_BRAKING trim.
  With the clutch in there is no whisper at all: a request above the rolling car's own accel would rev the engine."""
  over = 0. if declutched else DESCENT_MARGIN + TRIM_DESCENT_SHARE * trim
  return min(coast + coast_margin(coast) + trim, max(coast, 0.) + over, COAST_CAP)


class NonAccController:
  def __init__(self, CP, dt: float):
    self.dt = dt
    self.mode = NonAccMode(CP, dt)
    self._params = Params()
    self._frame = 0
    self._read_period = max(1, int(round(1.0 / dt)))   # 1 Hz param read
    self.trigger = DEFAULT_TRIGGER
    self.state = COASTING
    self.floor_cmd = 0.0     # the ramped lower-limit the controller is requesting
    self._aside = False      # stepped aside last frame: the next active frame re-seeds floor_cmd
    self.trim = 0.0          # raised while the car reports COMPUTER_BRAKING outside a pulse
    self.hold = HOLD_OFF     # low-speed hold state (published as nonAcc.holdState)
    self.hold_band = 1.8     # m/s
    self.hold_bias = 0.5
    self.upper_cap = None    # an upper accel limit the hold asks controlsd to apply (None = none)
    self._over_t = 0.0       # s the overspeed has been above the trigger
    self._since_pulse = TRIM_HOLDOFF
    self.learned = load_applied(self._params)   # applied learned engine-brake table (from the learner in plannerd)
    # exposed for the overlay
    self.active = False
    self.overspeed = 0.0

  def _read_params(self) -> None:
    t = self._params.get("NonAccOverspeedThreshold", return_default=True)
    if t is not None:
      self.trigger = float(np.clip(float(t), TRIGGER_MIN, TRIGGER_MAX))
    self.learned = load_applied(self._params)
    b = self._params.get("NonAccHoldBand", return_default=True)
    if b is not None:
      self.hold_band = float(np.clip(float(b) * 0.44704, *HOLD_BAND_RANGE))
    b = self._params.get("NonAccHoldBias", return_default=True)
    if b is not None:
      self.hold_bias = float(np.clip(float(b), 0., 1.))

  def _step_aside(self, lower_limit: float) -> float:
    self.floor_cmd = lower_limit
    self._aside = True
    self._over_t = 0.
    self.hold = HOLD_OFF
    self.upper_cap = None
    return lower_limit

  def update(self, long_active: bool, v_ego: float, v_set: float, gear, pitch, force_decel: bool,
             lower_limit: float, car_non_adaptive: bool, decoupled: bool = False,
             last_accel: float | None = None, lead_accel_limit: float | None = None,
             computer_braking: bool = False, plan_is_cruise: bool = True) -> float:
    """decoupled: the drivetrain is decoupled AND the clutch signal is live (a real clutch-in / neutral, not a dead
    signal). last_accel: the accel LongControl commanded last frame (0 while longitudinal was inactive); None = unknown,
    then a resume ramps from the raw lower limit as it always did.
    lead_accel_limit: ACC mode only - the lowest accel the floor must let through (plannerd: longitudinalPlanSP.
    coastFloorBound). The floor is never raised above it; None = not known, then ACC mode gets no floor at all.
    computer_braking: the car's own VSA_STATUS.COMPUTER_BRAKING flag.
    plan_is_cruise: the plan is limited by the set speed (not a lead): the low-speed hold may drive the speed up."""
    if self._frame % self._read_period == 0:
      self._read_params()
    self._frame += 1

    non_acc = self.mode.update(car_non_adaptive)       # the car's own ACC <-> Cruise mode is the switch
    # ACC mode (owner 2026-11-06, always on): the same floor, bounded by the lead plan. Without that bound: no floor.
    acc_coast = bool(self.mode.available and not non_acc and lead_accel_limit is not None and math.isfinite(lead_accel_limit))
    mode_active = non_acc or acc_coast
    self.overspeed = v_ego - v_set

    # step aside -> stock/plan braking: mode or long off, bad/low speed, forceDecel, or no pitch. NOT gated on overspeed:
    # the floor must engage at and just below set so the car can drift up to the sawtooth on a descent (goal 1c).
    if not (mode_active and long_active) or v_set <= 0. or v_ego < MIN_SPEED or force_decel \
       or pitch is None or not math.isfinite(pitch):
      self.state = COASTING
      self.active = False
      return self._step_aside(lower_limit)

    self.active = True

    # sawtooth hysteresis. The trigger must hold for PULSE_CONFIRM: on a SET press the first active frame still has
    # the previous set speed, and one frame of "8 mph over" seeded the floor at -1.0 for 0.8 s (session 15).
    self._over_t = self._over_t + self.dt if self.overspeed > self.trigger else 0.
    if self.state == COASTING and self._over_t >= PULSE_CONFIRM:
      self.state = PULSING
      if last_accel is not None:
        # the pulse ramps in from what is being commanded now (the request may sit above the floor), not as a step
        self.floor_cmd = max(self.floor_cmd, float(last_accel))
    elif self.state == PULSING and self.overspeed < RELEASE:
      self.state = COASTING
    self._since_pulse = 0. if self.state == PULSING else self._since_pulse + self.dt

    # too far over set: hand back to full plan braking (+ the overlay flags it)
    if self.overspeed > HARD_CEILING:
      return self._step_aside(lower_limit)

    if self.state == COASTING and gear is None and not decoupled:
      # gear not trusted and not a live clutch-in (clutch signal dead, reverse): cannot estimate what the car does on
      # its own -> let the plan hold set speed (stock)
      return self._step_aside(lower_limit)

    # COASTING target = the car's closed-throttle accel (engine-brake priority): command it and the car coasts -
    # engine-brakes back to set on the flat, drifts up toward the trigger on a descent, service brakes off. Capped on
    # the positive side so a bad pitch/table estimate can't demand strong throttle. The learned table (if trusted)
    # refines the engine-brake estimate; else the static seed.
    if self.state == PULSING:
      target = PULSE_DECEL
      coast = None
      self.hold = HOLD_OFF
      self.upper_cap = None
    elif gear is None:
      # clutch in / neutral: no engine braking, the car rolls on road load + grade. The request is that accel itself,
      # clutch in or out (owner 2026-11-06: "what is required for no throttle and no brake, regardless of clutch"):
      # positive on a descent. Capping it at 0 there (the first version of this fix) made the VSA hold the car back
      # with the clutch in - about 1 m/s^2 on a 12-17 % hill. The clutch gate lets a floor above its throttle cap
      # through (clutch_gate.limit_accel), so the command is exactly this.
      coast = declutched_coast_accel(v_ego, pitch)
    else:
      coast = -effective_decel(gear, v_ego, self.learned) - G * math.sin(pitch)
    if coast is not None:
      # the car's own flag is the final word: while it reports braking outside a pulse, raise the floor; fade slowly
      if computer_braking and self._since_pulse >= TRIM_HOLDOFF and self._over_t == 0.:
        self.trim = min(self.trim + TRIM_RISE * self.dt, TRIM_MAX)
      else:
        self.trim = max(self.trim - TRIM_DECAY * self.dt, 0.)
      # the margin above the coast keeps the car's brakes un-armed; COAST_CAP bounds everything
      target = coast_floor(coast, self.trim, declutched=gear is None)
      # low-speed hold: inside the PCM's fuel-cut hole hold the speed in a band instead of hunting at its edge
      # the hole: the engine must add something, but less than it can deliver steadily. A coast that is itself >= 0
      # (a descent the car speeds up on) is the sawtooth's business, not the hold's.
      hole = 0. < -coast < (HOLE_LEAVE if self.hold != HOLD_OFF else HOLE_ENTER)
      if hole and gear is not None and plan_is_cruise:
        hi = min(v_set + self.hold_bias * self.hold_band, v_set + self.trigger - 0.5)
        lo = v_set - (1. - self.hold_bias) * self.hold_band
        if self.hold == HOLD_OFF:
          self.hold = HOLD_DRIVING if v_ego < 0.5 * (lo + hi) else HOLD_COASTING
        elif self.hold == HOLD_DRIVING and v_ego >= hi:
          self.hold = HOLD_COASTING
        elif self.hold == HOLD_COASTING and v_ego <= lo:
          self.hold = HOLD_DRIVING
        if self.hold == HOLD_DRIVING:
          target = min(coast + HOLD_DRIVE + self.trim, COAST_CAP)
          self.upper_cap = None
        else:
          target = min(coast + HOLD_COAST + self.trim, COAST_CAP)
          self.upper_cap = target             # pinned: a plan request inside the hole would make the engine flutter
      else:
        self.hold = HOLD_OFF
        self.upper_cap = None
    if self._aside:
      # back from a step-aside: restart from what was last commanded (never above the target), so the floor is there
      # at once after a gas override (last command 0) and real braking (ACC -> non-ACC) is still released on the ramp
      seed = lower_limit if last_accel is None else max(lower_limit, float(last_accel))
      self.floor_cmd = min(target, seed)
      self._aside = False
    step = (HOLD_JERK if self.hold != HOLD_OFF else PULSE_JERK) * self.dt
    self.floor_cmd = float(np.clip(target, self.floor_cmd - step, self.floor_cmd + step))
    if self.upper_cap is not None:
      self.upper_cap = max(self.floor_cmd, self.upper_cap)   # the cap follows the ramp down, never below the floor
    floor = self.floor_cmd
    if self.state == COASTING and last_accel is not None:
      # ease the throttle shut: below 0 the request falls no faster than COAST_FALL_RATE (from 0 if it was above 0)
      floor = max(floor, min(float(last_accel), 0.) - COAST_FALL_RATE * self.dt)
    if acc_coast and float(lead_accel_limit) < floor:
      # ACC mode: braking for the lead has been allowed. It comes in on a ramp from what is being commanded now -
      # or at once when the plan needs hard braking (or the last command is unknown).
      bound = float(lead_accel_limit)
      if bound > ACC_HARD_BRAKE and last_accel is not None:
        bound = max(bound, min(float(last_accel), floor) - ACC_RELEASE_JERK * self.dt)
      floor = bound
    # only ever raise the lower limit
    return max(lower_limit, floor)
