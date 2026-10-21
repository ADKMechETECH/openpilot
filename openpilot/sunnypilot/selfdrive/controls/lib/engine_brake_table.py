"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-gear engine-braking table for EngineBrakePreferred (Task 17, HONDA_CIVIC_2022 6MT).

How the radarless Civic answers an ACCEL_COMMAND (Stage 2, 2026-10-20, on-car): 0 = zero axle torque (rolls on road
load); any negative request closes the throttle (fuel cut, engine braking); the VSA then brakes only for whatever the
car does not already achieve - it TRACKS the request, on the flat and downhill alike. So the service brakes stay off
exactly while the request is no stronger than what the car does with the throttle closed:

  closed-throttle accel  a_ct  = -D(gear, v) - g*sin(pitch)        D = flat-road engine braking + road load (> 0)
  brake-free floor       floor = -MARGIN * D(gear, v) - g*sin(pitch)       (clamp only while floor <= FLOOR_MAX)

Any request between the floor and 0 is met by engine braking alone - and because the car always slows at the
stronger of the request and its own engine braking, a smaller D costs no decel on the flat: the table only decides
where the brakes would join. MARGIN 0.8 leaves 20 % of D for model and pitch error (1 deg of pitch = 0.17 m/s^2, so
on a descent the margin is mostly the pitch estimate's).

FLOOR_MAX: on a descent steeper than the gear's engine braking can hold, a_ct > 0 and no negative request avoids the
brakes - holding set speed needs them whatever the clamp does. The floor is then weaker than FLOOR_MAX and the clamp
steps aside (EngineBrakePreferred: reason "descent", stock braking): in the sim, capping the request there instead
used as much brake in total and a higher peak (it alternated between a zero-torque roll and braking at the cap).
FLOOR_MAX is -0.15 because the request must stay clearly in the closed-throttle regime: Stage 2 saw -0.01 held at
~zero axle torque (no engine braking), -0.17 closed the throttle, -0.06 was in between.
TODO(on-car): where between -0.01 and -0.17 the PCM switches from torque-holding to fuel cut (a weaker FLOOR_MAX lets
 the clamp work on slightly steeper descents). And whether a small POSITIVE request below a_ct keeps the throttle
 closed on a descent - if it does, the clamp could be brake-free on any grade.

D comes from the Stage 1 disengaged coasts (route 00000002--6c1a3bb7d4; analysis/stage1/eb_table.py in the project
repo): per gear and speed bin, min(median measured flat decel, the 0x130 axle-torque engine part + the fitted road
load); bins without data use that model (1st gear: 23 N*m crank drag through the gearing); 0 where the engine would be
under 900 or over 6500 rpm. EngineBrakeLearner (engine_brake_learning.py) refines it; its values are NOT applied
until APPLY_LEARNED is switched on after review.
"""
import math

import numpy as np

G = 9.81
MARGIN = 0.8                # use 80 % of the measured closed-throttle decel
FLOOR_MAX = -0.15           # m/s^2: a brake-free floor weaker than this can't be held with the throttle closed
UNKNOWN_GEAR_FLOOR = 0.0    # gear not trusted (clutch in, neutral, phantom, ratio mismatch): coast only, no decel request

# Speed bin centres (m/s) and flat-road closed-throttle decel (m/s^2, positive) per gear, from Stage 1.
V_MIDS = (5.0, 9.0, 13.0, 17.0, 21.0, 25.0, 29.0, 35.5)
STATIC_DECEL = {
  1: (0.95, 0.95, 0.96, 0.00, 0.00, 0.00, 0.00, 0.00),
  2: (0.70, 0.70, 0.69, 0.73, 0.75, 0.77, 0.00, 0.00),
  3: (0.00, 0.52, 0.46, 0.55, 0.57, 0.59, 0.62, 0.67),
  4: (0.00, 0.44, 0.45, 0.47, 0.49, 0.51, 0.54, 0.59),
  5: (0.00, 0.41, 0.42, 0.43, 0.39, 0.47, 0.50, 0.55),
  6: (0.00, 0.00, 0.38, 0.40, 0.38, 0.44, 0.47, 0.52),
}
GEARS = tuple(sorted(STATIC_DECEL))


def table_decel(table: dict, gear: int, v_ego: float) -> float:
  """Flat-road closed-throttle decel (>= 0) for gear at v_ego, linear between bin centres, held at the ends; 0 for a
  gear the table doesn't have. A bin of 0 (engine out of its rpm range) pulls its neighbours down - conservative."""
  row = table.get(gear)
  if row is None or not math.isfinite(v_ego):
    return 0.
  return max(float(np.interp(v_ego, V_MIDS, row)), 0.)


def brake_free_floor(decel: float, pitch: float) -> float:
  """The most negative request the car meets on engine braking alone (see the module docstring). pitch in radians,
  positive = uphill. Above FLOOR_MAX it is not usable (the descent is steeper than engine braking can hold)."""
  return -MARGIN * max(decel, 0.) - G * math.sin(pitch)


class EngineBrakeTable:
  """The static table plus (once enabled) the learner's values. Learned values only ever ADD decel authority: each cell
  is max(static, learned), so the conservative static table is always the floor."""

  def __init__(self, learned: dict | None = None, apply_learned: bool = False):
    self.apply_learned = apply_learned
    self.learned = learned or {}

  def decel(self, gear: int, v_ego: float) -> float:
    static = table_decel(STATIC_DECEL, gear, v_ego)
    if not self.apply_learned or gear not in self.learned:
      return static
    return max(static, table_decel(self.learned, gear, v_ego))
