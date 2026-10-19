"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Clutch-gate re-engagement pieces (manual transmission, ClutchShiftGate ON). Used by ClutchShiftGate (clutch_shift_gate.py)
and controlsd:

  RatioLock (Task 8a, tightened in Task 12 B4) — "coupled" = the drivetrain is mechanically connected again, judged on
    FILTERED engine rpm and wheel speed:
      - the gear inferred from rpm / speed (engine_brake_model.infer_gear, the +-8 % gear-ID tolerance) must equal the
        gear the gearbox itself reports (GEAR_MT via CarStateSP.gearRaw) — never a lock onto the old gear;
      - the measured ratio must be within the LOCK tolerance (ClutchReengageRatioTol, default 2 %, much tighter than
        the 8 % gear-ID band: a slipping clutch at 85-95 % of the new gear's rpm is inside the band but not locked);
      - and stay there, steady (spread <= RATIO_LOCK_STEADY), for RATIO_LOCK_TIME (0.12 s).
    GEAR_MT reporting a gear (clutch signal alive, clutchPressed False) is required first. Smart mode only.

  TakeoverRamp (Task 8c) — the hand-back from a shift to the plan. Since Task 12 A2 a shift no longer drops longitudinal:
    the gate caps the THROTTLE side at the coast value and the plan's braking passes through, so at the end of a shift
    the command is still running. The ramp lifts that cap gradually (no step to the plan's throttle): seeded from the
    current command (fixed-delay mode) or, in smart mode, from ClutchReengageSeed — "aego" (the car's measured accel,
    the Task 8 default) or "coast" (aEgo minus the engine-brake coast estimate: what the command must be if the car
    ADDS it to engine braking). Slewed at the planner's set-speed jerk x JERK_SCALE_BY_GEAR, escalating to CATCHUP_RATE
    by ClutchReengageMaxRamp. Applied as accel limits intersected with the existing ones (only ever narrows).
    It ends at once on Rule-5 urgency (braking_urgency) and on a step brake request (the plan falling more than
    STEP_BRAKE below the ramp's current value) — needed braking is never rate-limited.

TODO(on-car): C2 decides the seed (does a negative ACCEL_COMMAND add brakes on top of engine braking?) — "coast" if it
 does, "aego" if the car tracks the requested accel. Tune ClutchReengageRatioTol, ClutchReengageGasDebounce,
 ClutchReengageMaxRamp and JERK_SCALE_BY_GEAR by feel; check the lock against real ENGINE_RPM / wheel-speed noise (S2/S3).
"""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import INFER_MIN_SPEED, estimate_coast_decel, \
                                                                           infer_gear, measured_ratio, overall_ratio
from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import GearAwareAccel
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import read_handback_params

RATIO_LOCK_TIME = 0.12             # s the filtered ratio must sit inside the lock tolerance, steady
RATIO_LOCK_STEADY = 0.01           # max (max - min) / mean of the filtered ratio over the lock window
LOCK_FILTER_RC = 0.05              # s, rpm and speed are filtered alike before the ratio (25 Hz sample-and-hold rpm)

# Params (registered in params_keys.h): (default, (min, max)). Every value is clamped.
DEFAULT_GAS_DEBOUNCE = 0.15        # s the gas must be released before openpilot takes over
GAS_DEBOUNCE_RANGE = (0., 0.5)
DEFAULT_RATIO_TOL = 0.02           # the LOCK tolerance (gear identification stays engine_brake_model's +-8 %)
RATIO_TOL_RANGE = (0.01, 0.04)
DEFAULT_MAX_RAMP = 1.0             # s, longest the takeover ramp may last
MAX_RAMP_RANGE = (0.3, 2.0)
DEFAULT_MAX_COAST = 5.0            # s uncoupled with the clutch in before the gate stops forcing a coast (8b)
MAX_COAST_RANGE = (2., 5.)         # Task 12 A2.3: at most 5 s
SEED_MODES = ("aego", "coast")
DEFAULT_SEED = "aego"

# Base takeover rate = the planner's set-speed jerk limit (longitudinal_planner J_CRUISE_VALS over A_CRUISE_MAX_BP;
# duplicated here so controlsd doesn't import the planner/MPC), times the per-gear scale. Pinned by a test.
TAKEOVER_JERK_BP = [0., 10.0, 25., 40.]
TAKEOVER_JERK_V = [1.6, 1.2, 0.8, 0.6]
# The ramp starts at the per-gear rate and escalates linearly in time to CATCHUP_RATE at ClutchReengageMaxRamp, so it
# catches a plan that keeps moving before the ramp ends, instead of stepping to it at the end.
CATCHUP_RATE = 3.0                 # m/s^3
STEP_BRAKE = 0.5                   # m/s^2: the plan this far below the ramp = a step brake request -> end the ramp


def _read_float(params: Params, key: str, default: float, rng: tuple[float, float]) -> float:
  try:
    v = params.get(key, return_default=True)
    v = default if v is None else float(v)
  except (TypeError, ValueError):
    v = default
  if not math.isfinite(v):
    v = default
  return min(max(v, rng[0]), rng[1])


def _read_seed(params: Params) -> str:
  try:
    v = params.get("ClutchReengageSeed", return_default=True)
  except (TypeError, ValueError):
    v = None
  v = v.strip().lower() if isinstance(v, str) else DEFAULT_SEED
  return v if v in SEED_MODES else DEFAULT_SEED


@dataclass(frozen=True)
class ReengageParams:
  smart: bool = False
  gas_debounce: float = DEFAULT_GAS_DEBOUNCE
  ratio_tol: float = DEFAULT_RATIO_TOL
  max_ramp: float = DEFAULT_MAX_RAMP
  max_coast: float = DEFAULT_MAX_COAST
  ratio_min_speed: float = INFER_MIN_SPEED
  seed: str = DEFAULT_SEED


def read_reengage_params(params: Params) -> ReengageParams:
  handback_speed, _ = read_handback_params(params)
  return ReengageParams(
    smart=params.get_bool("ClutchReengageSmart"),
    gas_debounce=_read_float(params, "ClutchReengageGasDebounce", DEFAULT_GAS_DEBOUNCE, GAS_DEBOUNCE_RANGE),
    ratio_tol=_read_float(params, "ClutchReengageRatioTol", DEFAULT_RATIO_TOL, RATIO_TOL_RANGE),
    max_ramp=_read_float(params, "ClutchReengageMaxRamp", DEFAULT_MAX_RAMP, MAX_RAMP_RANGE),
    max_coast=_read_float(params, "ClutchShiftGateMaxCoast", DEFAULT_MAX_COAST, MAX_COAST_RANGE),
    # ratio lock is noisy near idle, and below the handback speed the handback owns longitudinal anyway
    ratio_min_speed=max(INFER_MIN_SPEED, handback_speed),
    seed=_read_seed(params),
  )


class RatioLock:
  """Filtered engine/wheel ratio lock onto the gear the gearbox reports (see the module docstring)."""
  def __init__(self):
    self.rpm_f = FirstOrderFilter(0., LOCK_FILTER_RC, DT_CTRL)
    self.v_f = FirstOrderFilter(0., LOCK_FILTER_RC, DT_CTRL)
    self.seeded = False
    self.n = max(1, round(RATIO_LOCK_TIME / DT_CTRL))
    self.reset()

  def reset(self) -> None:
    self.gear = 0
    self.ratios: deque[float] = deque(maxlen=self.n)

  @property
  def coupled(self) -> bool:
    if len(self.ratios) < self.n:
      return False
    return (max(self.ratios) - min(self.ratios)) / (sum(self.ratios) / len(self.ratios)) <= RATIO_LOCK_STEADY

  @property
  def frames(self) -> int:
    return len(self.ratios)

  def update(self, CS, gear_raw: int, tol: float) -> bool:
    rpm, v = CS.deprecated.engineRpm, CS.vEgo
    if not self.seeded:
      self.rpm_f.x, self.v_f.x, self.seeded = rpm, v, True
    rpm_f, v_f = self.rpm_f.update(rpm), self.v_f.update(v)
    gear = infer_gear(rpm_f, v_f, CS.deprecated.clutchPressed)
    # must be the gear the gearbox reports, and inside the (tight) lock tolerance of its ratio
    ratio = measured_ratio(rpm_f, v_f)
    if gear == 0 or gear != gear_raw or abs(ratio / overall_ratio(gear) - 1.) > tol:
      self.reset()
      return False
    if gear != self.gear:
      self.gear = gear
      self.ratios.clear()
    self.ratios.append(ratio)
    return self.coupled


def takeover_rate(v_ego: float, gear: int) -> float:
  """m/s^3: the takeover ramp's accel rate in this gear (gear 0 / unknown -> the stock set-speed jerk)."""
  return float(np.interp(v_ego, TAKEOVER_JERK_BP, TAKEOVER_JERK_V)) * GearAwareAccel.jerk_scale(gear)


def takeover_seed(mode: str, CS, gear: int, pitch: float | None) -> float:
  """Smart mode's seed value (before clipping to the limits): 'aego', or 'coast' = aEgo minus the engine-brake coast
  estimate (what the command must be if the car ADDS it to engine braking). Unknown gear -> aEgo."""
  if mode == "coast" and gear >= 1:
    grade = pitch if pitch is not None and math.isfinite(pitch) else 0.
    return CS.aEgo - estimate_coast_decel(gear, CS.deprecated.engineRpm, CS.vEgo, grade)
  return CS.aEgo


class TakeoverRamp:
  def __init__(self):
    self.active = False
    self.seed = 0.
    self.prev = 0.
    self.t = 0.
    self.gear = 0
    self.max_ramp = DEFAULT_MAX_RAMP
    self.reason = "inactive"

  def start(self, seed_value: float, a_target: float, accel_limits: tuple[float, float], gear: int, max_ramp: float) -> float:
    """Takeover instant: returns the seed (the first command)."""
    seed = float(np.clip(seed_value, accel_limits[0], accel_limits[1]))
    if seed > 0.:
      # residual accel from the driver's throttle is not something to keep commanding
      seed = max(min(seed, max(a_target, 0.)), accel_limits[0])
    self.active, self.seed, self.prev, self.t, self.gear, self.max_ramp = True, seed, seed, 0., gear, max_ramp
    self.reason = "ramping"
    cloudlog.event("clutch_reengage_takeover", seed=round(seed, 3), a_target=round(a_target, 3), gear=gear)
    return seed

  def end(self, reason: str) -> None:
    if self.active:
      cloudlog.event("clutch_reengage_takeover_end", reason=reason, t=round(self.t, 2), accel=round(self.prev, 3))
    self.active = False
    self.reason = reason

  def limit_accel(self, long_active: bool, v_ego: float, urgent: str | None, a_target: float,
                  accel_limits: tuple[float, float]) -> tuple[float, float]:
    """controlsd, before LongControl: the slew envelope intersected with the limits (only ever narrows)."""
    if not self.active:
      return accel_limits
    if not long_active:
      self.end("inactive")
      return accel_limits
    if urgent is not None:
      self.end(urgent)
      return accel_limits
    if a_target < self.prev - STEP_BRAKE:
      self.end("step_brake")
      return accel_limits
    if self.t >= self.max_ramp - 1e-6:
      self.end("max_ramp")
      return accel_limits

    if self.t == 0.:
      env = (self.seed, self.seed)
    else:
      rate = takeover_rate(v_ego, self.gear)
      rate += max(CATCHUP_RATE - rate, 0.) * self.t / self.max_ramp
      env = (self.prev - rate * DT_CTRL, self.prev + rate * DT_CTRL)
    self.t += DT_CTRL
    lo, hi = max(accel_limits[0], env[0]), min(accel_limits[1], env[1])
    if lo > hi:  # the envelope is outside the limits (a floor rose / a ceiling fell): the limits win
      lo = hi = float(np.clip(self.prev, accel_limits[0], accel_limits[1]))
    return lo, hi

  def post_update(self, output: float, unclamped: float, in_pid: bool) -> None:
    """controlsd, after LongControl: end once the controller's own output is inside the envelope (converged)."""
    if not self.active:
      return
    self.prev = float(output)
    if not in_pid:
      self.end("not_pid")
    elif abs(unclamped - output) < 1e-6:
      self.end("converged")
