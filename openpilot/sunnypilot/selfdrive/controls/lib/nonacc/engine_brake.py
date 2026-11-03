"""
fork-only (civic-cruise) non-ACC: per-gear engine-braking table + brake-free floor (HONDA_CIVIC_2022 6MT).

How the radarless Civic answers an ACCEL_COMMAND (Stage 2, on-car): 0 = ~zero axle torque (rolls on road load); any
negative request closes the throttle (engine braking); the VSA then brakes only for whatever the car does not already
achieve - it TRACKS the request, flat and downhill alike. So the service brakes stay off exactly while the request is no
stronger than what the car does with the throttle closed:

  closed-throttle accel  a_ct  = -D(gear, v) - g*sin(pitch)              D = flat-road engine braking + road load (> 0)
  brake-free floor       floor = -MARGIN * D(gear, v) - g*sin(pitch)

MARGIN 0.8 leaves 20 % of D for model + pitch error (1 deg pitch = 0.17 m/s^2). The car always slows at the stronger of
the request and its own engine braking, so a smaller D costs no decel on the flat - the table only decides where the
brakes would join.

D is from the Stage 1 + Stage 3 disengaged closed-throttle coasts (~850 s, 276 coasts; analysis/stage3). Bins with
>= 3 coasts / >= 3 s use the measured median; the rest are capped model values; 0 where the engine is under 900 or over
6500 rpm. The learner (Phase 3) refines this; here it is a fixed seed.

Data (Stage 2, on-car): the VSA TRACKS the request - a command below the car's own coast accel is met by service
braking (achieved approx requested, flat and downhill alike), and a command of ~0 on the flat holds a whisper of
throttle (rolls on road load, no brake). So capping the coast-follow command at COAST_CAP means: on a gentle descent
(coast <= cap) the command ~= the coast accel and the car drifts ~brake-free; on a steep descent (coast > cap) the car
drifts at the cap with mild braking (the VSA brakes off the extra gravity). Both are bounded and both reach the
sawtooth trigger. Commanding the FULL coast accel to be brake-free on any grade risks a whisper of throttle if the
estimate is high - deferred until logs confirm it; the cap bounds any such over-command to a gentle accel.
"""
import math

import numpy as np

G = 9.81
MARGIN = 0.8                # use 80 % of the measured closed-throttle decel
UNKNOWN_GEAR_FLOOR = 0.0    # gear not trusted (clutch in, neutral, phantom): coast only, no decel request

# Speed bin centres (m/s) and flat-road closed-throttle decel (m/s^2, positive) per gear, from Stage 1 + Stage 3.
V_MIDS = (5.0, 9.0, 13.0, 17.0, 21.0, 25.0, 29.0, 35.5)
STATIC_DECEL = {
  1: (0.62, 0.62, 0.62, 0.00, 0.00, 0.00, 0.00, 0.00),
  2: (0.33, 0.64, 0.65, 0.67, 0.69, 0.70, 0.00, 0.00),
  3: (0.00, 0.50, 0.52, 0.53, 0.53, 0.53, 0.53, 0.53),
  4: (0.00, 0.42, 0.44, 0.45, 0.46, 0.47, 0.47, 0.47),
  5: (0.00, 0.35, 0.35, 0.35, 0.39, 0.42, 0.42, 0.50),
  6: (0.00, 0.00, 0.36, 0.36, 0.36, 0.43, 0.43, 0.53),
}


def table_decel(gear: int, v_ego: float) -> float:
  """Flat-road closed-throttle decel (>= 0) for gear at v_ego, linear between bin centres, held at the ends; 0 for a
  gear the table doesn't have (then the floor is coast-only)."""
  row = STATIC_DECEL.get(gear)
  if row is None or not math.isfinite(v_ego):
    return 0.
  return max(float(np.interp(v_ego, V_MIDS, row)), 0.)


def coast_accel(gear: int, v_ego: float, pitch: float) -> float:
  """The car's own closed-throttle accel (m/s^2): -D - g*sin(pitch). Negative = slowing (flat/uphill), positive = the
  car speeds up on a descent steeper than engine braking. pitch in radians, positive = uphill."""
  return -table_decel(gear, v_ego) - G * math.sin(pitch)


def brake_free_floor(gear: int, v_ego: float, pitch: float) -> float:
  """The most negative accel request the car still meets on engine braking alone (no service brakes), with an 0.8
  margin on D for model + pitch error. Raising LongControl's lower limit to this keeps the brakes off while the plan
  wants no more decel than engine braking provides."""
  return -MARGIN * table_decel(gear, v_ego) - G * math.sin(pitch)
