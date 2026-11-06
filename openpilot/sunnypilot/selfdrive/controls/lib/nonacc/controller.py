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
  - Coming back from a step-aside (gas override released, mode switched, hard ceiling left) the floor restarts from
    the last commanded accel, not from the raw lower limit: no ~2 s window in which the planner brakes unfloored
    (session 14: -1.19 m/s^2 on releasing the gas 6.8 mph over set), and a release of real braking (ACC -> non-ACC
    while ACC brakes for a lead) is still ramped at PULSE_JERK.

No lead-based backstops: non-ACC ignores leads by design (the driver brakes; FCW still warns).
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
MIN_SPEED = 2.0          # m/s: below this the coast estimate is unreliable, step aside
DEFAULT_TRIGGER = 3.6    # m/s over set (~8 mph): start the firm brake pulse (user-settable)
TRIGGER_MIN = RELEASE + 0.5
TRIGGER_MAX = HARD_CEILING - 0.45   # keep RELEASE < trigger < HARD_CEILING


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
    self.learned = load_applied(self._params)   # applied learned engine-brake table (from the learner in plannerd)
    # exposed for the overlay
    self.active = False
    self.overspeed = 0.0

  def _read_params(self) -> None:
    t = self._params.get("NonAccOverspeedThreshold", return_default=True)
    if t is not None:
      self.trigger = float(np.clip(float(t), TRIGGER_MIN, TRIGGER_MAX))
    self.learned = load_applied(self._params)

  def _step_aside(self, lower_limit: float) -> float:
    self.floor_cmd = lower_limit
    self._aside = True
    return lower_limit

  def update(self, long_active: bool, v_ego: float, v_set: float, gear, pitch, force_decel: bool,
             lower_limit: float, car_non_adaptive: bool, decoupled: bool = False,
             last_accel: float | None = None) -> float:
    """decoupled: the drivetrain is decoupled AND the clutch signal is live (a real clutch-in / neutral, not a dead
    signal). last_accel: the accel LongControl commanded last frame (0 while longitudinal was inactive); None = unknown,
    then a resume ramps from the raw lower limit as it always did."""
    if self._frame % self._read_period == 0:
      self._read_params()
    self._frame += 1

    mode_active = self.mode.update(car_non_adaptive)   # the car's own ACC <-> Cruise mode is the switch
    self.overspeed = v_ego - v_set

    # step aside -> stock/plan braking: mode or long off, bad/low speed, forceDecel, or no pitch. NOT gated on overspeed:
    # the floor must engage at and just below set so the car can drift up to the sawtooth on a descent (goal 1c).
    if not (mode_active and long_active) or v_set <= 0. or v_ego < MIN_SPEED or force_decel \
       or pitch is None or not math.isfinite(pitch):
      self.state = COASTING
      self.active = False
      return self._step_aside(lower_limit)

    self.active = True

    # sawtooth hysteresis
    if self.state == COASTING and self.overspeed > self.trigger:
      self.state = PULSING
    elif self.state == PULSING and self.overspeed < RELEASE:
      self.state = COASTING

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
    elif gear is None:
      # clutch in / neutral: no engine braking, the car rolls on road load + grade. Keep the floor there so a shift
      # above set does not hand the brakes to the planner. Never above 0: there is no throttle to give a declutched
      # car (the clutch gate caps that side anyway), so on a descent the floor is a request of 0.
      target = min(declutched_coast_accel(v_ego, pitch), 0.0)
    else:
      coast = -effective_decel(gear, v_ego, self.learned) - G * math.sin(pitch)
      target = min(coast, COAST_CAP)
    if self._aside:
      # back from a step-aside: restart from what was last commanded (never above the target), so the floor is there
      # at once after a gas override (last command 0) and real braking (ACC -> non-ACC) is still released on the ramp
      seed = lower_limit if last_accel is None else max(lower_limit, float(last_accel))
      self.floor_cmd = min(target, seed)
      self._aside = False
    step = PULSE_JERK * self.dt
    self.floor_cmd = float(np.clip(target, self.floor_cmd - step, self.floor_cmd + step))
    # only ever raise the lower limit
    return max(lower_limit, self.floor_cmd)
