"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

No-service-brake ("coast over set speed") mode: openpilot will not command service-brake-level deceleration to hold
the set speed, so on downhills the car coasts/drifts above set speed like a classic throttle-only cruise.

Transmission- and platform-agnostic: it acts on openpilot's own commanded longitudinal accel, nothing else (not the
camera/AEB path, not panda safety). Validated only on the Honda Civic; per-platform longitudinal/AEB behavior must be
verified before using it on any other car.

Split of responsibility (so the safety coupling can't diverge between processes):
  - plannerd (NoServiceBrake.update) is the single authority on WHETHER braking is allowed and publishes it as
    longitudinalPlan.allowBrake (an existing field that was always True).
  - controlsd (apply_brake_floor) only applies the configured decel floor to LongControl's accel limits when the plan
    says braking is not allowed. A missing/invalid plan means braking IS allowed (capnp's default for allowBrake is
    False, so that default must never be trusted).

Safety coupling: only active while NonAdaptiveLongitudinal is actually active. Clamping braking while lead-following
would stop openpilot braking for a lead.
"""
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog

# TODO(on-car): tune the default decel floor. 0.0 = pure throttle-only cruise; small negative accel requests may still
#  be served by the brakes on some platforms (Honda radarless: VSA decides from ACCEL_COMMAND).
DEFAULT_DECEL_FLOOR = 0.3       # m/s^2, magnitude of the most negative accel openpilot may command while clamped
MAX_DECEL_FLOOR = 3.5
# TODO(on-car): tune the overspeed margin. ~8 mph. The backstop is only as strong as openpilot's braking on the car
#  (Honda radarless: the camera-path negative ACCEL_COMMAND; how hard the VSA brakes on it is the open opendbc PR #2050
#  question — unverified on our car. Not 0x1BE: that is the alternate user-brake-pedal message, not brake authority.)
DEFAULT_OVERSPEED_MARGIN = 3.6  # m/s above set speed at which braking is allowed again
BACKSTOP_RELEASE_RATIO = 0.5    # braking stays allowed until back under set speed + margin * ratio (hysteresis)
# The stock-AEB consequence of openpilot longitudinal is platform-specific (disabled on Honda radarless, retained on some
# other platforms). Nothing here assumes the Honda behavior; this list only drives a warning.
VALIDATED_PLATFORMS = ("HONDA_CIVIC_2022",)


def _read_float(params: Params, key: str, default: float, lo: float, hi: float) -> float:
  try:
    v = params.get(key, return_default=True)
    v = default if v is None else float(v)
  except (TypeError, ValueError):
    v = default
  return min(max(v, lo), hi)


class NoServiceBrake:
  def __init__(self, CP, params: Params | None = None):
    self.params = params or Params()
    self.available = bool(CP.openpilotLongitudinalControl)  # pcmCruise cars: no-op
    self.validated_platform = CP.carFingerprint in VALIDATED_PLATFORMS
    self._warned_platform = False
    self._enabled_requested = False
    self.backstop_enabled = True
    self.overspeed_margin = DEFAULT_OVERSPEED_MARGIN
    self.read_params()
    self.enabled = self._enabled_requested
    self.backstop_active = False
    self.allow_brake = True
    self._frame = 0

  def read_params(self) -> None:
    self._enabled_requested = self.available and self.params.get_bool("NoServiceBrakeHold")
    # default ON: get_bool() ignores the registered default for an unset key (would read False), so use get()
    self.backstop_enabled = bool(self.params.get("NoServiceBrakeOverspeedBackstop", return_default=True))
    self.overspeed_margin = _read_float(self.params, "NoServiceBrakeOverspeedMargin", DEFAULT_OVERSPEED_MARGIN, 0.5, 15.)
    if self._enabled_requested and not self.validated_platform and not self._warned_platform:
      cloudlog.warning("no_service_brake: enabled on an unvalidated platform; verify its openpilot-long braking and AEB first")
      self._warned_platform = True

  def update(self, long_inactive: bool, non_adaptive: bool, v_ego: float, v_set: float, force_decel: bool) -> bool:
    """Call every planner frame. Returns allow_brake."""
    if self._frame % int(1. / DT_MDL) == 0:
      self.read_params()
    self._frame += 1
    if long_inactive:  # like non-adaptive: never switch modes mid-engagement
      self.enabled = self._enabled_requested

    clamp_wanted = self.enabled and non_adaptive and not force_decel
    if not clamp_wanted:
      self.backstop_active = False
    elif self.backstop_enabled:
      if v_ego > v_set + self.overspeed_margin:
        if not self.backstop_active:
          cloudlog.event("no_service_brake_backstop", v_ego=v_ego, v_set=v_set)
        self.backstop_active = True
      elif v_ego < v_set + self.overspeed_margin * BACKSTOP_RELEASE_RATIO:
        self.backstop_active = False
    else:
      self.backstop_active = False

    self.allow_brake = not clamp_wanted or self.backstop_active
    return self.allow_brake


def read_decel_floor(params: Params) -> float:
  return _read_float(params, "NoServiceBrakeDecelFloor", DEFAULT_DECEL_FLOOR, 0., MAX_DECEL_FLOOR)


def apply_brake_floor(accel_limits: tuple[float, float], allow_brake: bool, plan_valid: bool, enabled: bool,
                      decel_floor: float) -> tuple[float, float]:
  """controlsd side: raise LongControl's lower accel limit to -decel_floor when the plan forbids braking."""
  if enabled and plan_valid and not allow_brake:
    return max(accel_limits[0], -decel_floor), accel_limits[1]
  return accel_limits
