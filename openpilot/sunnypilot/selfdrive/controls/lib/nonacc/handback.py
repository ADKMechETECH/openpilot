"""
fork-only (civic-cruise) manual low-speed longitudinal handback.

Below a low speed openpilot hands longitudinal back to the driver instead of braking a manual to a stop, holding it, or
launching it. Sequence, once engaged and below the handback speed (latched until disengaged):
  1. controlsd ramps LongControl's accel range from the last command to 0 over RAMP (no step, no PID windup),
  2. then pauses longitudinal (longActive=False: no gas, no brake, no stop/standstill request),
  3. selfdrived disengages DISENGAGE_MARGIN later with the stock speedTooLow event.
Engaging below the handback speed (+ hysteresis) is refused (stock belowEngageSpeed). Not a toggle (owner decision):
without it openpilot brakes a manual to a stop and launches it; only the speed/ramp are tunable.

Gear-aware speed (Stage 3): the handback speed is max(ManualLowSpeedHandbackSpeed, the current gear's value) so it hands
back near ~1000 rpm (2nd 8.7 mph, 3rd 13 mph); 1st and 4th-6th use the setting (owner). Gear read before the latch only.

NON-ACC scope: this build ignores leads, so openpilot never follows a lead down to low speed - the Task 13/14
imminent-lead latch (keep braking behind a close stopping lead, then hand back) is not needed and is omitted. force_decel
(driver monitoring) still aborts the handback (stock brings the car to a stop). Controlsd owns the state and publishes
it; selfdrived acts on the published state.
"""
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from opendbc.car import structs

MPH_TO_MS = 0.44704
DEFAULT_HANDBACK_SPEED = 2.24   # m/s (~5 mph)
GEAR_HANDBACK_SPEED = {2: 8.7 * MPH_TO_MS, 3: 13.0 * MPH_TO_MS}   # ~1000 rpm in 2nd / 3rd
MIN_HANDBACK_SPEED = 1.5        # m/s: floor so the param can't be tuned into braking to a stop / launching
MAX_HANDBACK_SPEED = 8.0        # m/s
DEFAULT_RAMP_TIME = 0.5         # s: the accel command ramps to 0 over this before the disengage
MAX_RAMP_TIME = 2.0             # s
DISENGAGE_MARGIN = 0.1          # s: selfdrived disengages this long after the ramp ends
ENGAGE_HYSTERESIS = 0.5         # m/s above the threshold still refuses to engage


def handback_available(CP: structs.CarParams) -> bool:
  return (CP.transmissionType == structs.CarParams.TransmissionType.manual
          and bool(CP.openpilotLongitudinalControl))


class LowSpeedHandback:
  def __init__(self, CP: structs.CarParams, dt: float, params: Params | None = None, process: str = "controlsd"):
    self._params = params or Params()
    self.dt = dt
    self.process = process
    self.available = handback_available(CP)
    self.speed = DEFAULT_HANDBACK_SPEED
    self.ramp_time = DEFAULT_RAMP_TIME
    self.active = False
    self.timer = 0.
    self.accel_start = 0.
    self.gear = 0

  def read_params(self) -> None:
    def _f(key, default, lo, hi):
      try:
        v = self._params.get(key, return_default=True)
        return min(max(float(v), lo), hi) if v is not None else default
      except (ValueError, TypeError):
        return default
    self.speed = _f("ManualLowSpeedHandbackSpeed", DEFAULT_HANDBACK_SPEED, MIN_HANDBACK_SPEED, MAX_HANDBACK_SPEED)
    self.ramp_time = _f("ManualLowSpeedHandbackRamp", DEFAULT_RAMP_TIME, 0., MAX_RAMP_TIME)

  @property
  def threshold(self) -> float:
    """The handback speed now: the setting, raised to the current gear's value (2nd/3rd)."""
    return max(self.speed, GEAR_HANDBACK_SPEED.get(self.gear, 0.))

  def update(self, enabled: bool, v_ego: float, last_accel: float, gear: int, force_decel: bool) -> None:
    if not self.active:
      self.read_params()        # params and gear only change the threshold before the latch
      self.gear = gear
    if not self.available or not enabled or force_decel:
      if self.active and force_decel:
        cloudlog.event("manual_low_speed_handback_abort", process=self.process, reason="force_decel", v_ego=v_ego)
      self.active = False
      self.timer = 0.
    elif not self.active:
      if v_ego < self.threshold:
        self.active = True
        self.timer = 0.
        self.accel_start = last_accel
        cloudlog.event("manual_low_speed_handback", process=self.process, v_ego=v_ego, accel=last_accel,
                       speed=self.threshold, gear=self.gear, ramp=self.ramp_time)
    else:
      self.timer += self.dt

  @property
  def ramp_done(self) -> bool:
    """controlsd: the command has reached 0, drop longActive."""
    return self.active and self.timer >= self.ramp_time

  @property
  def disengage_due(self) -> bool:
    """published for selfdrived: hand back (stock speedTooLow) now."""
    return self.active and self.timer >= self.ramp_time + DISENGAGE_MARGIN - 1e-6

  def below_engage_speed(self, v_ego: float) -> bool:
    return self.available and v_ego < self.threshold + ENGAGE_HYSTERESIS

  def limit_accel(self, lower: float, upper: float) -> tuple[float, float]:
    """controlsd: shrink the accel range linearly from the command at handback start to 0."""
    if not self.active:
      return lower, upper
    f = max(1. - self.timer / self.ramp_time, 0.) if self.ramp_time > 0. else 0.
    hi = min(upper, max(self.accel_start, 0.) * f)
    lo = max(lower, min(self.accel_start, 0.) * f)
    return lo, max(hi, lo)
