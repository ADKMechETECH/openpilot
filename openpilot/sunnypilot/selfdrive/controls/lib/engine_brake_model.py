"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Engine-brake model for a manual transmission: how much the car slows by itself when it coasts in gear with the clutch
engaged and no gas/brake (closed throttle -> fuel cut -> engine pumping + friction losses through the gearing), plus
the rolling/aero/grade terms that apply in neutral too.

  *** The numbers below are an UNCALIBRATED PHYSICS SEED (HONDA_CIVIC_2022, 2.0 L K20C2, 6MT). ***
  They exist so the interface and the consumers (EngineBrakePreferred, GearAwareAccel) can be built and tested now.
  TODO(on-car): on-car phase — fit a per-gear/rpm decel table from the CoastEventLogger logs (engine_brake_coast/*.jsonl),
   replace the seed values (and confirm the gear ratios / final drive / tire radius below).

Gear: carState does not carry the numeric gear (Phase 1 deliberately left GEAR_MT 1-6 out of the schema). infer_gear()
recovers it from the engine-speed / wheel-speed ratio, which is what actually sets engine braking at the wheels. That
is also fail-safe: clutch pressed, neutral with the clutch out (GEAR_MT reads 0 for both), a slipping clutch, a missing
RPM signal or wrong seed ratios all fail to match a gear -> gear 0 -> no engine braking credited -> consumers brake.

Sign convention: accelerations in m/s^2, negative = deceleration. grade is the road pitch in radians, positive = uphill
(same convention as the planner's get_coast_accel(orientationNED[1])).
"""
import json
import logging
import math
import os
import time
from logging.handlers import RotatingFileHandler

import numpy as np

from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog

G = 9.81

# --- Seed vehicle data (HONDA_CIVIC_2022 Sport hatchback, 6MT) --------------------------------------------------------
# 1st..6th, from Honda's 2022 Civic Hatchback spec sheet (6MT: 3.643 / 2.080 / 1.361 / 1.024 / 0.830 / 0.686, FD 4.10).
# TODO(on-car): confirm the final drive for the 2024 2.0 L 6MT — sources list 4.10 (2022 hatchback spec) or 4.11 (2020
#  2.0 L 6MT); the 1.5 T Si's 4.35 does not apply. The logged rpm/speed ratio per gear settles it.
GEAR_RATIOS = (3.643, 2.080, 1.361, 1.024, 0.830, 0.686)
FINAL_DRIVE = 4.10
# TODO(on-car): 235/40R18 is ~0.323 m unloaded; 0.315 m is a loaded rolling-radius guess.
WHEEL_RADIUS = 0.315             # m
MASS = 1326. + 80.               # kg, CarSpecs mass + one occupant
WHEEL_INERTIA_MASS = 40.         # kg, 4 wheels/tires/rotors as equivalent translating mass (seed)
ENGINE_INERTIA = 0.12            # kg*m^2, crank + flywheel + clutch (seed); reflected through the gearing
ROLLING_COEFF = 0.011
CDA = 0.65                       # m^2, Cd ~0.30 x ~2.15 m^2 frontal area (seed)
AIR_DENSITY = 1.225              # kg/m^3

# Closed-throttle engine braking torque at the crank vs rpm: friction + pumping (FMEP ~2.2..4 bar on a 2.0 L). Seed.
EB_TORQUE_BP = [700., 2000., 4000., 6500.]    # rpm
EB_TORQUE_V = [20., 30., 42., 58.]            # N*m

# Gear inference
GEAR_MATCH_TOL = 0.08            # measured ratio within +-8% of a gear's (adjacent gears are >= 20% apart)
INFER_MIN_SPEED = 3.0            # m/s, wheel-speed resolution / clutch slip at crawl make the ratio meaningless
INFER_MIN_RPM = 500.

MAX_ABS_ACCEL = 4.0              # sanity clip on the estimate


def overall_ratio(gear: int) -> float:
  """Engine revolutions per wheel revolution in gear (1-6); 0.0 for anything else."""
  return GEAR_RATIOS[gear - 1] * FINAL_DRIVE if 1 <= gear <= len(GEAR_RATIOS) else 0.


def engine_rpm_at(gear: int, v_ego: float) -> float:
  """Engine rpm the seed gearing predicts in gear at v_ego, clutch engaged."""
  return v_ego / WHEEL_RADIUS * overall_ratio(gear) * 60. / (2. * math.pi)


def measured_ratio(rpm: float, v_ego: float) -> float:
  """Engine-to-wheel speed ratio from the signals (0.0 when undefined)."""
  if v_ego < 0.1:
    return 0.
  return rpm * 2. * math.pi / 60. * WHEEL_RADIUS / v_ego


def infer_gear(rpm: float, v_ego: float, clutch_pressed: bool = False, tol: float = GEAR_MATCH_TOL) -> int:
  """Gear (1-6) whose ratio matches rpm/speed within +-tol, else 0 (clutch in, neutral, slipping, no RPM, too slow)."""
  if clutch_pressed or not (math.isfinite(rpm) and math.isfinite(v_ego)) or v_ego < INFER_MIN_SPEED or rpm < INFER_MIN_RPM:
    return 0
  ratio = measured_ratio(rpm, v_ego)
  errors = [abs(ratio / overall_ratio(g) - 1.) for g in range(1, len(GEAR_RATIOS) + 1)]
  best = int(np.argmin(errors))
  return best + 1 if errors[best] <= tol else 0


def engine_brake_torque(rpm: float) -> float:
  """Seed closed-throttle braking torque at the crank, N*m (positive = resisting)."""
  return float(np.interp(rpm, EB_TORQUE_BP, EB_TORQUE_V))


def resistive_decel(v_ego: float) -> float:
  """Rolling + aero decel (m/s^2, <= 0), present in gear and in neutral."""
  v = max(v_ego, 0.)
  force = ROLLING_COEFF * MASS * G * (1. if v > 0.1 else 0.) + 0.5 * AIR_DENSITY * CDA * v * v
  return -force / (MASS + WHEEL_INERTIA_MASS)


def engine_brake_decel(gear: int, rpm: float) -> float:
  """Engine-braking component only (m/s^2, <= 0); 0 when not in gear 1-6."""
  ratio = overall_ratio(gear)
  if ratio == 0. or rpm <= 0.:
    return 0.
  wheel_ratio = ratio / WHEEL_RADIUS                        # wheel force per crank torque, 1/m
  force = engine_brake_torque(rpm) * wheel_ratio
  # the engine's rotating inertia, reflected through the gearing, resists the slowdown too (big in 1st)
  m_eff = MASS + WHEEL_INERTIA_MASS + ENGINE_INERTIA * wheel_ratio ** 2
  return -force / m_eff


def estimate_coast_decel(gear: int, rpm: float, v_ego: float, grade: float = 0.0) -> float:
  """
  Estimated acceleration while coasting (no gas, no brake, clutch engaged if gear is 1-6), m/s^2, negative = decel.
  gear 0 (clutch in / neutral / unknown) -> no engine braking, only rolling + aero + grade. On a steep enough downhill
  the result is >= 0: coasting would not slow the car at all.
  UNCALIBRATED SEED — see the module docstring.
  """
  a = engine_brake_decel(gear, rpm) + resistive_decel(v_ego) - G * math.sin(grade)
  return float(np.clip(a, -MAX_ABS_ACCEL, MAX_ABS_ACCEL)) if math.isfinite(a) else 0.


# --- Calibration hook --------------------------------------------------------------------------------------------------
COAST_LOG_DIR_ENV = "ENGINE_BRAKE_COAST_LOG_DIR"
COAST_MIN_EVENT = 1.0            # s, shorter coasts are too noisy to fit
COAST_MAX_EVENT = 30.0           # s, longer coasts are flushed in chunks
COAST_SAMPLE_PERIOD = 0.1        # s, ~10 Hz is plenty for a decel fit
COAST_LOG_MAX_BYTES = 1024 * 1024
COAST_LOG_BACKUPS = 10           # <= ~11 MB on disk


def coast_log_dir() -> str:
  # next to swaglog's directory (/data/log -> /data/engine_brake_coast on device), not in realdata (the deleter's turf)
  return os.environ.get(COAST_LOG_DIR_ENV) or os.path.join(os.path.dirname(Paths.swaglog_root().rstrip("/")),
                                                           "engine_brake_coast")


class CoastEventLogger:
  """
  Records in-gear coast events for the offline fit (telemetry only — never affects control):
    in gear 1-6 (inferred), clutch engaged, driver not on gas or brake, openpilot NOT commanding longitudinal (its gas
    feedforward would contaminate the coast), no stock AEB/FCW, no openpilot FCW, rolling.
  One JSON line per event (one gear per event) in a rotating log, plus a cloudlog summary event in the rlog.
  Raw rpm/speed ratio is logged too, so the fit can re-bin gears if the seed ratios turn out wrong.
  TODO(on-car): on-car phase — fit per-gear/rpm decel table from these logs, replace the seed values.
  """

  def __init__(self, available: bool, dt: float, log_dir: str | None = None):
    self.available = available
    self.dt = dt
    self.log_dir = log_dir
    self._logger: logging.Logger | None = None
    self._failed = False
    self.events_written = 0
    self._reset()

  def _reset(self) -> None:
    self.gear = 0
    self.duration = 0.
    self._since_sample = COAST_SAMPLE_PERIOD
    self.samples: list[list[float]] = []
    self.t_start = 0.

  def _file_logger(self) -> logging.Logger | None:
    if self._logger is None and not self._failed:
      try:
        log_dir = self.log_dir or coast_log_dir()
        os.makedirs(log_dir, exist_ok=True)
        handler = RotatingFileHandler(os.path.join(log_dir, "coast.jsonl"), maxBytes=COAST_LOG_MAX_BYTES,
                                      backupCount=COAST_LOG_BACKUPS)
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger = logging.getLogger(f"engine_brake_coast.{id(self)}")
        logger.propagate = False
        logger.setLevel(logging.INFO)
        logger.addHandler(handler)
        self._logger = logger
      except OSError:
        cloudlog.exception("engine_brake_coast: can't open coast log")
        self._failed = True
    return self._logger

  def _flush(self) -> None:
    if self.duration >= COAST_MIN_EVENT and self.samples:
      a = [s[4] for s in self.samples]
      event = {"type": "coast_event", "version": 1, "mono_time": self.t_start, "gear": self.gear,
               "duration": round(self.duration, 2),
               "fields": ["t", "rpm", "v_ego", "ratio", "a_ego", "pitch"], "samples": self.samples}
      cloudlog.event("engine_brake_coast", gear=self.gear, duration=round(self.duration, 2), n=len(self.samples),
                     a_ego_mean=round(float(np.mean(a)), 3), v_start=self.samples[0][2], v_end=self.samples[-1][2])
      logger = self._file_logger()
      if logger is not None:
        try:
          logger.info(json.dumps(event, separators=(",", ":")))
          self.events_written += 1
        except (OSError, ValueError):
          cloudlog.exception("engine_brake_coast: write failed")
    self._reset()

  @staticmethod
  def is_coasting(CS, long_active: bool, fcw: bool) -> int:
    """Returns the coasting gear (1-6), or 0 when the frame doesn't qualify."""
    clutch = CS.deprecated.clutchPressed
    if clutch or CS.gasPressed or CS.brakePressed or long_active or fcw or CS.stockAeb or CS.stockFcw:
      return 0
    return infer_gear(CS.deprecated.engineRpm, CS.vEgo, clutch)

  def update(self, CS, long_active: bool, fcw: bool, pitch: float) -> None:
    if not self.available:
      return
    gear = self.is_coasting(CS, long_active, fcw)
    if gear != self.gear or self.duration >= COAST_MAX_EVENT:
      self._flush()
      if gear:
        self.gear = gear
        self.t_start = time.monotonic()  # matches the rlog's logMonoTime clock (seconds)
    if not gear:
      return

    self._since_sample += self.dt
    if self._since_sample >= COAST_SAMPLE_PERIOD - 1e-6:
      self._since_sample = 0.
      rpm, v = float(CS.deprecated.engineRpm), float(CS.vEgo)
      self.samples.append([round(self.duration, 2), round(rpm, 1), round(v, 3), round(measured_ratio(rpm, v), 4),
                           round(float(CS.aEgo), 4), round(float(pitch), 5)])
    self.duration += self.dt
