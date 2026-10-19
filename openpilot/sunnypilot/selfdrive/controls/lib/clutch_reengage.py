"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Clutch-gate smart re-engagement (manual transmission, ClutchShiftGate ON, sub-toggle ClutchReengageSmart, default OFF).
Two pieces used by ClutchShiftGate (clutch_shift_gate.py) and controlsd:

  RatioLock (8a) — "coupled" = the drivetrain is mechanically connected again: engine rpm / wheel speed matches a gear
    ratio (engine_brake_model.infer_gear, the same inference Task 6 uses) for RATIO_LOCK_FRAMES frames in a row, with
    the ratio steady over them (a slipping clutch sweeps through it, possibly across an adjacent gear's band). It
    replaces the gate's fixed REENGAGE_DELAY guess at the bite point. GEAR_MT reporting a gear (clutchPressed False) is
    still required first, as a pre-cue: rolling in neutral at idle rpm passes through a speed band where the ratio
    coincidentally matches a gear (~7.5-8.8 m/s for 6th at 700 rpm), and GEAR_MT == 0 rules that out.

  TakeoverRamp (8c) — bumpless hand-back from the gate to openpilot longitudinal. At the takeover instant the command
    is seeded from the car's measured accel (aEgo, clipped to the accel limits already narrowed by NoServiceBrakeHold /
    EngineBrakePreferred, and never above the plan's own throttle), then slewed toward what LongControl wants at a
    per-gear rate: the planner's set-speed jerk x gear_aware_accel.JERK_SCALE_BY_GEAR (independent of the GearAwareAccel
    toggle). The ramp is applied as LongControl accel limits, like the low-speed handback, so the PID's anti-windup
    holds the integrator while the envelope binds; LongControl's integrator is NOT seeded (seeding it with
    seed - aTarget would make the PID itself hold the seed and unwind at the ki rate, not converge at the ramp rate).
    The envelope is only ever intersected with the existing limits (tighter wins), so it can't command below an NSB/EBP
    floor or add throttle the plan doesn't ask for. Negative seed (engine braking after a downshift): kept — the ramp
    sets the rate, the target stays the plan; no throttle is forced on to cancel engine braking.
    Urgent bypass: any close/closing lead, FCW, stock AEB/FCW, forceDecel, a hard decel demand, shouldStop or an invalid
    plan/radar ends the ramp at once, so needed braking is never rate-limited (same backstop pattern as Task 6).

TODO(on-car): tune ClutchReengageRatioTol, ClutchReengageGasDebounce, ClutchReengageMaxRamp, ClutchShiftGateMaxCoast and
 JERK_SCALE_BY_GEAR (shared with GearAwareAccel) by feel; confirm no lurch/hole at takeover; A/B against the fixed-delay
 gate (ClutchReengageSmart OFF).
TODO(on-car): the seed is a negative ACCEL_COMMAND after an engine-braking downshift. Whether the radarless car treats
 that as "keep decelerating like this" (closed loop on accel) or as a service-brake request on top of the engine braking
 is test C2 (command-0 / small-negative semantics). If it adds brakes, the downshift hand-back will feel like a brake
 tap — seed from max(aEgo, coast estimate) or 0 instead.
"""
import math
from dataclasses import dataclass

import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import GEAR_MATCH_TOL, INFER_MIN_SPEED, infer_gear, \
                                                                           measured_ratio
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import DEFAULT_HARD_FLOOR, DEFAULT_LEAD_CLOSING, \
                                                                               DEFAULT_LEAD_TTC, lead_urgent
from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import GearAwareAccel
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import read_handback_params

RATIO_LOCK_FRAMES = 3              # consecutive frames (100 Hz) at one gear's ratio before "coupled"
# ...and the ratio steady over those frames: a locked clutch holds rpm/speed constant, while a slipping one sweeps
# through the ratio (and can cross an ADJACENT gear's +-8% band on the way, e.g. 85% of 4th's rpm reads as 5th).
# TODO(on-car): check against ENGINE_RPM / wheel-speed noise in steady cruise (test S3).
RATIO_LOCK_STEADY = 0.01           # max (max - min) / mean of the measured ratio over the lock window

# Params (registered in params_keys.h): (default, (min, max)). Every value is clamped.
DEFAULT_GAS_DEBOUNCE = 0.15        # s the gas must be released before openpilot takes over
GAS_DEBOUNCE_RANGE = (0., 0.5)
DEFAULT_RATIO_TOL = GEAR_MATCH_TOL  # +-8%, Task 6's tolerance
RATIO_TOL_RANGE = (0.02, 0.09)     # adjacent gears are >= 21% apart (5th/6th): above ~0.095 the bands would overlap
DEFAULT_MAX_RAMP = 1.0             # s, longest the takeover ramp may last
MAX_RAMP_RANGE = (0.3, 2.0)
DEFAULT_MAX_COAST = 8.0            # s uncoupled before the gate stops forcing a coast (8b)
MAX_COAST_RANGE = (2., 20.)

# Base takeover rate = the planner's set-speed jerk limit (longitudinal_planner J_CRUISE_VALS over A_CRUISE_MAX_BP;
# duplicated here so controlsd doesn't import the planner/MPC), times the per-gear scale. Pinned by a test.
TAKEOVER_JERK_BP = [0., 10.0, 25., 40.]
TAKEOVER_JERK_V = [1.6, 1.2, 0.8, 0.6]
URGENT_DECEL = DEFAULT_HARD_FLOOR  # m/s^2: a plan asking for more decel than this bypasses the ramp
# The ramp starts at the per-gear rate and escalates linearly in time to CATCHUP_RATE at
# ClutchReengageMaxRamp, so it catches a plan that keeps moving (the stock planner ramps faster than the per-gear rate
# when GearAwareAccel is OFF) before the ramp ends, instead of stepping to it at the end.
CATCHUP_RATE = 3.0                 # m/s^3


def _read_float(params: Params, key: str, default: float, rng: tuple[float, float]) -> float:
  try:
    v = params.get(key, return_default=True)
    v = default if v is None else float(v)
  except (TypeError, ValueError):
    v = default
  if not math.isfinite(v):
    v = default
  return min(max(v, rng[0]), rng[1])


@dataclass(frozen=True)
class ReengageParams:
  smart: bool = False
  gas_debounce: float = DEFAULT_GAS_DEBOUNCE
  ratio_tol: float = DEFAULT_RATIO_TOL
  max_ramp: float = DEFAULT_MAX_RAMP
  max_coast: float = DEFAULT_MAX_COAST
  ratio_min_speed: float = INFER_MIN_SPEED


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
  )


class RatioLock:
  """Debounced engine/wheel-speed ratio lock: coupled once one gear's ratio matches RATIO_LOCK_FRAMES frames running."""
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.gear = 0
    self.ratios: list[float] = []

  @property
  def coupled(self) -> bool:
    if len(self.ratios) < RATIO_LOCK_FRAMES:
      return False
    return (max(self.ratios) - min(self.ratios)) / (sum(self.ratios) / len(self.ratios)) <= RATIO_LOCK_STEADY

  @property
  def frames(self) -> int:
    return len(self.ratios)

  def update(self, CS, tol: float) -> bool:
    gear = infer_gear(CS.deprecated.engineRpm, CS.vEgo, CS.deprecated.clutchPressed, tol)
    if gear == 0:
      self.reset()
      return False
    if gear != self.gear:
      self.gear, self.ratios = gear, []
    self.ratios = (self.ratios + [measured_ratio(CS.deprecated.engineRpm, CS.vEgo)])[-RATIO_LOCK_FRAMES:]
    return self.coupled


def takeover_rate(v_ego: float, gear: int) -> float:
  """m/s^3: the takeover ramp's accel rate in this gear (gear 0 / unknown -> the stock set-speed jerk)."""
  return float(np.interp(v_ego, TAKEOVER_JERK_BP, TAKEOVER_JERK_V)) * GearAwareAccel.jerk_scale(gear)


def takeover_urgent(CS, plan, plan_valid: bool, radar, radar_valid: bool, force_decel: bool) -> str | None:
  """Reason the takeover ramp must not limit the command this frame (needed braking passes at once), or None."""
  if not plan_valid or not radar_valid:
    return "invalid_input"
  if force_decel:
    return "force_decel"
  if plan.fcw or CS.stockAeb or CS.stockFcw:
    return "fcw_aeb"
  if plan.shouldStop:
    return "stopping"
  if plan.aTarget < URGENT_DECEL:
    return "hard_decel"
  if lead_urgent(radar.leadOne, CS.vEgo, DEFAULT_LEAD_TTC, DEFAULT_LEAD_CLOSING) or \
     lead_urgent(radar.leadTwo, CS.vEgo, DEFAULT_LEAD_TTC, DEFAULT_LEAD_CLOSING):
    return "lead"
  return None


class TakeoverRamp:
  def __init__(self):
    self.active = False
    self.seed = 0.
    self.prev = 0.
    self.t = 0.
    self.gear = 0
    self.max_ramp = DEFAULT_MAX_RAMP
    self.reason = "inactive"

  def start(self, a_ego: float, a_target: float, accel_limits: tuple[float, float], gear: int, max_ramp: float) -> float:
    """Takeover instant: returns the seed (the first command)."""
    seed = float(np.clip(a_ego, accel_limits[0], accel_limits[1]))
    if seed > 0.:
      # residual accel from the driver's throttle is not something to keep commanding
      seed = max(min(seed, max(a_target, 0.)), accel_limits[0])
    self.active, self.seed, self.prev, self.t, self.gear, self.max_ramp = True, seed, seed, 0., gear, max_ramp
    self.reason = "ramping"
    cloudlog.event("clutch_reengage_takeover", seed=round(seed, 3), a_ego=round(a_ego, 3), a_target=round(a_target, 3),
                   gear=gear)
    return seed

  def end(self, reason: str) -> None:
    if self.active:
      cloudlog.event("clutch_reengage_takeover_end", reason=reason, t=round(self.t, 2), accel=round(self.prev, 3))
    self.active = False
    self.reason = reason

  def limit_accel(self, long_active: bool, v_ego: float, urgent: str | None,
                  accel_limits: tuple[float, float]) -> tuple[float, float]:
    """controlsd, before LongControl: the slew envelope intersected with the limits (only ever narrows them)."""
    if not self.active:
      return accel_limits
    if not long_active:
      self.end("inactive")
      return accel_limits
    if urgent is not None:
      self.end(urgent)
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
