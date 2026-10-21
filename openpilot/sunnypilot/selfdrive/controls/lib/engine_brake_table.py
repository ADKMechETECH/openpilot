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

D comes from the disengaged closed-throttle coasts of Stage 1 and Stage 3 (routes 00000002--6c1a3bb7d4,
00000006--65015c3735 = the Stage 3 coast-down drive, and the disengaged parts of 00000007--001552d195; ~850 s, 276
coasts; analysis/stage3/eb_table3.py + build_table.py in the project repo): per gear and speed bin with >= 3 coasts and
>= 3 s, min(median measured flat decel over frames >= 1 s into a coast, the 0x130 axle-torque engine part + the fitted
road load 0.188 + 1.52e-4*v^2); a bin without that much data takes min(that model, the gear's nearest measured bin)
(engine braking grows with rpm, so a neighbour caps a guess); 0 where the engine would be under 900 or over 6500 rpm.
Stage 1's table (Task 17) was close in 3rd-6th but about 0.03 m/s^2 optimistic, and far too high in 1st (0.95, a crank
drag model: measured 0.62) and in 2nd at 5 m/s (0.70: measured 0.33, ~1300 rpm). EngineBrakeLearner
(engine_brake_learning.py) refines it; its values are NOT applied until APPLY_LEARNED is switched on after review.
"""
import math

import numpy as np

G = 9.81
MARGIN = 0.8                # use 80 % of the measured closed-throttle decel
FLOOR_MAX = -0.15           # m/s^2: a brake-free floor weaker than this can't be held with the throttle closed
UNKNOWN_GEAR_FLOOR = 0.0    # gear not trusted (clutch in, neutral, phantom, ratio mismatch): coast only, no decel request

# Speed bin centres (m/s) and flat-road closed-throttle decel (m/s^2, positive) per gear, from Stage 1 + Stage 3.
# Measured bins (>= 3 coasts, >= 3 s): 1st 5; 2nd 5-13; 3rd 13-17; 4th 13-25; 5th 17-25, 35.5; 6th 21-25, 35.5.
# The rest are capped model values (see the module docstring) - gaps: 1st above 7 m/s, 2nd above 15, 3rd below 11 and
# above 19, 4th below 11 and above 27, 5th below 15 and 27-31, 6th below 19 and 27-31.
V_MIDS = (5.0, 9.0, 13.0, 17.0, 21.0, 25.0, 29.0, 35.5)
STATIC_DECEL = {
  1: (0.62, 0.62, 0.62, 0.00, 0.00, 0.00, 0.00, 0.00),
  2: (0.33, 0.64, 0.65, 0.67, 0.69, 0.70, 0.00, 0.00),
  3: (0.00, 0.50, 0.52, 0.53, 0.53, 0.53, 0.53, 0.53),
  4: (0.00, 0.42, 0.44, 0.45, 0.46, 0.47, 0.47, 0.47),
  5: (0.00, 0.35, 0.35, 0.35, 0.39, 0.42, 0.42, 0.50),
  6: (0.00, 0.00, 0.36, 0.36, 0.36, 0.43, 0.43, 0.53),
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
