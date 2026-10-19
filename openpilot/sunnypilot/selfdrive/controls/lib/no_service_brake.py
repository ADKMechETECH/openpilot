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

Overspeed backstop modes (NoServiceBrakeOverspeedMode, latched like the toggle):
  - maintain (default, the original backstop): above set + margin braking is simply allowed again (allowBrake True) and
    the plan brakes toward set speed until under set + margin/2. Steps the floor on and off; on a grade steeper than
    the plan's -1.2 m/s^2 can hold it rides the brakes continuously at set + margin.
  - pulse: a hysteretic, bounded brake pulse. COASTING (coast clamp) until set + margin -> PULSING: the lower accel
    limit moves at NoServiceBrakeOverspeedPulseJerk down to NoServiceBrakeOverspeedPulseDecel, until back at
    set + NoServiceBrakeOverspeedReleaseMargin -> the limit moves back up at the same jerk (brake rolled off) and the
    car coasts again. Above set + NoServiceBrakeOverspeedHardCeiling while PULSING (the bounded force can't beat the
    grade) -> ESCALATED: full openpilot braking + a one-shot "Overspeed: Take Over" alert, until under set + release or
    disengaged. A close/closing lead, planner FCW or stock AEB/FCW -> urgent: full openpilot braking (forceDecel
    already turns the whole clamp off). plannerd publishes the state and the target floor in
    longitudinalPlanSP.noServiceBrake; controlsd (PulseFloor) moves the limit. PULSING publishes allowBrake True, so
    a missing longitudinalPlanSP falls back to full braking, never to the coast clamp.
"""
from dataclasses import dataclass

import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import DEFAULT_LEAD_CLOSING, DEFAULT_LEAD_TTC, \
                                                                             lead_urgent

# TODO(on-car): tune the default decel floor. 0.0 = pure throttle-only cruise; small negative accel requests may still
#  be served by the brakes on some platforms (Honda radarless: VSA decides from ACCEL_COMMAND).
DEFAULT_DECEL_FLOOR = 0.3       # m/s^2, magnitude of the most negative accel openpilot may command while clamped
MAX_DECEL_FLOOR = 3.5
# TODO(on-car): tune the overspeed margin. ~8 mph. The backstop is only as strong as openpilot's braking on the car
#  (Honda radarless: the camera-path negative ACCEL_COMMAND; how hard the VSA brakes on it is the open opendbc PR #2050
#  question — unverified on our car. Not 0x1BE: that is the alternate user-brake-pedal message, not brake authority.)
DEFAULT_OVERSPEED_MARGIN = 3.6  # m/s above set speed at which braking is allowed again
BACKSTOP_RELEASE_RATIO = 0.5    # braking stays allowed until back under set speed + margin * ratio (hysteresis)
# Overspeed backstop pulse mode. TODO(on-car): tune after C1 (radarless brake strength) and the maintain-mode feel
#  (NSB2) are known; the band and the pulse force are only meaningful against the real brake authority.
OVERSPEED_MODES = ("maintain", "pulse")
DEFAULT_OVERSPEED_MODE = "maintain"
DEFAULT_RELEASE_MARGIN = 0.9    # m/s above set: a pulse ends here (~2 mph)
RELEASE_MARGIN_RANGE = (0., 3.6)
DEFAULT_PULSE_DECEL = -1.0      # m/s^2: the pulse's bounded decel
PULSE_DECEL_RANGE = (-2.5, -0.3)
DEFAULT_PULSE_JERK = 1.5        # m/s^3: roll-on and roll-off rate of the pulse
PULSE_JERK_RANGE = (0.5, 3.0)
DEFAULT_HARD_CEILING = 6.7      # m/s above set (~15 mph): escalate to full braking past this while pulsing
HARD_CEILING_RANGE = (4.5, 11.)
PULSE_MIN_BAND = 0.45           # m/s (~1 mph): enforced gap release < trigger < ceiling (a bad combo tightens the trigger)
# The stock-AEB consequence of openpilot longitudinal is platform-specific (disabled on Honda radarless, retained on some
# other platforms). Nothing here assumes the Honda behavior; this list only drives a warning.
VALIDATED_PLATFORMS = ("HONDA_CIVIC_2022",)


def _read_float(params: Params, key: str, default: float, lo: float, hi: float) -> float:
  try:
    v = params.get(key, return_default=True)
    v = default if v is None else float(v)
  except (TypeError, ValueError):
    v = default
  if not np.isfinite(v):
    v = default
  return min(max(v, lo), hi)


@dataclass(frozen=True)
class PulseParams:
  trigger: float = DEFAULT_OVERSPEED_MARGIN
  release: float = DEFAULT_RELEASE_MARGIN
  decel: float = DEFAULT_PULSE_DECEL
  jerk: float = DEFAULT_PULSE_JERK
  ceiling: float = DEFAULT_HARD_CEILING


def read_overspeed_mode(params: Params) -> str:
  try:
    mode = params.get("NoServiceBrakeOverspeedMode", return_default=True)
  except (TypeError, ValueError):
    mode = None
  if isinstance(mode, bytes):
    mode = mode.decode(errors="replace")
  mode = mode.strip().lower() if isinstance(mode, str) else DEFAULT_OVERSPEED_MODE
  return mode if mode in OVERSPEED_MODES else DEFAULT_OVERSPEED_MODE


def read_pulse_params(params: Params, trigger: float) -> PulseParams:
  """trigger = NoServiceBrakeOverspeedMargin (already clamped). Keeps release < trigger < ceiling by PULSE_MIN_BAND,
  tightening the trigger (earlier braking) if it sits too close to the ceiling."""
  ceiling = _read_float(params, "NoServiceBrakeOverspeedHardCeiling", DEFAULT_HARD_CEILING, *HARD_CEILING_RANGE)
  trigger = min(trigger, ceiling - PULSE_MIN_BAND)
  release = _read_float(params, "NoServiceBrakeOverspeedReleaseMargin", DEFAULT_RELEASE_MARGIN, *RELEASE_MARGIN_RANGE)
  release = max(min(release, trigger - PULSE_MIN_BAND), 0.)
  return PulseParams(
    trigger=trigger, release=release, ceiling=ceiling,
    decel=_read_float(params, "NoServiceBrakeOverspeedPulseDecel", DEFAULT_PULSE_DECEL, *PULSE_DECEL_RANGE),
    jerk=_read_float(params, "NoServiceBrakeOverspeedPulseJerk", DEFAULT_PULSE_JERK, *PULSE_JERK_RANGE),
  )


def overspeed_urgent(CS, fcw: bool, radar) -> str | None:
  """Pulse mode: a reason the overspeed logic must not limit braking at all this frame, or None."""
  if fcw or CS.stockAeb or CS.stockFcw:
    return "fcw_aeb"
  if lead_urgent(radar.leadOne, CS.vEgo, DEFAULT_LEAD_TTC, DEFAULT_LEAD_CLOSING) or \
     lead_urgent(radar.leadTwo, CS.vEgo, DEFAULT_LEAD_TTC, DEFAULT_LEAD_CLOSING):
    return "lead"
  return None


class NoServiceBrake:
  def __init__(self, CP, params: Params | None = None):
    self.params = params or Params()
    self.available = bool(CP.openpilotLongitudinalControl)  # pcmCruise cars: no-op
    self.validated_platform = CP.carFingerprint in VALIDATED_PLATFORMS
    self._warned_platform = False
    self._enabled_requested = False
    self.backstop_enabled = True
    self.overspeed_margin = DEFAULT_OVERSPEED_MARGIN
    self._mode_requested = DEFAULT_OVERSPEED_MODE
    self.decel_floor = DEFAULT_DECEL_FLOOR
    self.pulse = PulseParams()
    self.read_params()
    self.enabled = self._enabled_requested
    self.mode = self._mode_requested
    self.backstop_active = False
    self.allow_brake = True
    self._frame = 0
    # pulse mode (published in longitudinalPlanSP.noServiceBrake; "off" = maintain mode / not clamping)
    self.phase = "coasting"   # coasting | pulsing | escalated
    self.state = "off"        # what controlsd sees: off | coasting | pulsing | escalated | urgent
    self.accel_floor = 0.
    self.overspeed = 0.
    self.escalation_event = False

  def read_params(self) -> None:
    self._enabled_requested = self.available and self.params.get_bool("NoServiceBrakeHold")
    # default ON: get_bool() ignores the registered default for an unset key (would read False), so use get()
    self.backstop_enabled = bool(self.params.get("NoServiceBrakeOverspeedBackstop", return_default=True))
    self.overspeed_margin = _read_float(self.params, "NoServiceBrakeOverspeedMargin", DEFAULT_OVERSPEED_MARGIN, 0.5, 15.)
    self._mode_requested = read_overspeed_mode(self.params)
    self.decel_floor = read_decel_floor(self.params)
    self.pulse = read_pulse_params(self.params, self.overspeed_margin)
    if self._enabled_requested and not self.validated_platform and not self._warned_platform:
      cloudlog.warning("no_service_brake: enabled on an unvalidated platform; verify its openpilot-long braking and AEB first")
      self._warned_platform = True

  def update(self, long_inactive: bool, non_adaptive: bool, v_ego: float, v_set: float, force_decel: bool,
             urgent: str | None = None) -> bool:
    """Call every planner frame. Returns allow_brake. urgent (pulse mode only): see overspeed_urgent()."""
    if self._frame % int(1. / DT_MDL) == 0:
      self.read_params()
    self._frame += 1
    if long_inactive:  # like non-adaptive: never switch modes mid-engagement
      self.enabled = self._enabled_requested
      self.mode = self._mode_requested

    clamp_wanted = self.enabled and non_adaptive and not force_decel
    if self.mode == "pulse":
      self.allow_brake = self._update_pulse(clamp_wanted, long_inactive, v_ego, v_set, urgent)
      return self.allow_brake
    self.state = "off"
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

  def _set_phase(self, phase: str, v_ego: float, v_set: float) -> None:
    cloudlog.event("no_service_brake_pulse", phase=phase, prev=self.phase, overspeed=round(v_ego - v_set, 2),
                   v_set=round(v_set, 2))
    self.phase = phase

  def _update_pulse(self, clamp_wanted: bool, long_inactive: bool, v_ego: float, v_set: float, urgent: str | None) -> bool:
    self.escalation_event = False
    self.backstop_active = False
    self.overspeed = v_ego - v_set
    p = self.pulse
    if not clamp_wanted or long_inactive or not self.backstop_enabled:
      self.phase = "coasting"  # a disengage (or forceDecel, toggle off) ends a pulse or an escalation
    else:
      if self.phase == "coasting" and self.overspeed > p.trigger:
        self._set_phase("pulsing", v_ego, v_set)
      if self.phase == "pulsing":
        if self.overspeed > p.ceiling:
          # the bounded pulse can't beat this grade: full braking, tell the driver once
          self._set_phase("escalated", v_ego, v_set)
          self.escalation_event = True
        elif self.overspeed <= p.release:
          self._set_phase("coasting", v_ego, v_set)
      elif self.phase == "escalated" and self.overspeed < p.release:
        self._set_phase("coasting", v_ego, v_set)

    if not clamp_wanted:
      self.state, self.accel_floor = "off", 0.
      return True
    self.state = "urgent" if urgent else self.phase
    self.accel_floor = {"coasting": -self.decel_floor, "pulsing": p.decel}.get(self.state, 0.)
    # PULSING allows braking: controlsd bounds it with the pulse floor, and a missing plan SP message fails to full braking
    return self.state != "coasting"


def read_decel_floor(params: Params) -> float:
  return _read_float(params, "NoServiceBrakeDecelFloor", DEFAULT_DECEL_FLOOR, 0., MAX_DECEL_FLOOR)


def apply_brake_floor(accel_limits: tuple[float, float], allow_brake: bool, plan_valid: bool, enabled: bool,
                      decel_floor: float) -> tuple[float, float]:
  """controlsd side: raise LongControl's lower accel limit to -decel_floor when the plan forbids braking."""
  if enabled and plan_valid and not allow_brake:
    return max(accel_limits[0], -decel_floor), accel_limits[1]
  return accel_limits


class PulseFloor:
  """controlsd side of pulse mode: moves LongControl's lower accel limit toward the plan's target floor at the pulse
  jerk, so a pulse rolls the brake on and off instead of stepping it. Returns None when it doesn't apply (maintain
  mode, escalated/urgent, invalid plan, toggle off) and the caller falls back to apply_brake_floor()."""
  def __init__(self):
    self.floor: float | None = None

  def apply(self, accel_limits: tuple[float, float], long_active: bool, plan_valid: bool, enabled: bool, nsb,
            last_accel: float, dt: float) -> tuple[float, float] | None:
    if not (long_active and enabled and plan_valid and str(nsb.state) in ("coasting", "pulsing")):
      self.floor = None
      return None
    target = float(np.clip(nsb.accelFloor, -MAX_DECEL_FLOOR, 0.))
    step = float(np.clip(nsb.rampRate, *PULSE_JERK_RANGE)) * dt
    if self.floor is None:
      # entering from full braking (escalated/urgent) rolls the brake off from where it is; never starts above 0
      self.floor = min(last_accel, 0.)
    self.floor += float(np.clip(target - self.floor, -step, step))
    return min(max(accel_limits[0], self.floor), accel_limits[1]), accel_limits[1]
