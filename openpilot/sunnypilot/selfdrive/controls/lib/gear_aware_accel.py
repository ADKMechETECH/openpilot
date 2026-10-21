"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

GearAwareAccel (manual transmission + openpilot longitudinal, toggle default OFF): gear-indexed scaling of the
planner's set-speed accel-rate (jerk) limit, so throttle-on / throttle-off feel isn't abrupt in the low gears, where
the driveline is stiff and a small accel change is a big torque change at the engine.

  - Rising accel (throttle application): j_cruise x JERK_SCALE_BY_GEAR[gear].
  - Falling accel through the power -> coast span (a_cruise still above the engine-brake level, and the target is one
    engine braking can cover): the same gear scale, so the lift-off blends toward the engine-brake decel instead of
    snapping (driveline lash at the torque reversal is harshest in 1st/2nd).
  - Falling accel when braking is needed (target below the credited engine-brake decel): stock rate.
  - Any Rule-5 urgency (braking_urgency: FCW, a close/closing lead, forceDecel, a hard/stop request, invalid inputs):
    stock rate both ways, the same frame (Task 12).
Scales are <= 1.0 (never looser than stock). OFF, automatic, stock ACC, gear unknown (clutch in, neutral, no RPM), no
calibrated pitch (Task 12 C-11) or no live clutch signal -> 1.0 = stock. Latches only while openpilot is disengaged.
Only the cruise (set-speed) candidate's rate limit is scaled: the lead MPC (its jerk cost, and so FCW) is untouched, so
lead-following braking response is exactly stock.
"""
import math

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import G, estimate_coast_decel, infer_gear

TransmissionType = structs.CarParams.TransmissionType

# TODO(on-car): tune by feel (the 1st/2nd values especially). Index = gear 1..6. Must stay <= 1.0.
JERK_SCALE_BY_GEAR = (0.5, 0.65, 0.8, 0.9, 1.0, 1.0)
STOCK = (1.0, 1.0)
# share of the estimated engine + road decel credited as the power->coast span (was EngineBrakePreferred's credit
# before Task 17 replaced it with the per-gear clamp; kept here unchanged so GearAwareAccel behaves as before)
CREDIT_MARGIN = 0.7


class GearAwareAccel:
  def __init__(self, CP: structs.CarParams, params: Params | None = None):
    self.params = params or Params()
    self.available = CP.transmissionType == TransmissionType.manual and bool(CP.openpilotLongitudinalControl)
    self._enabled_requested = self.available and self.params.get_bool("GearAwareAccel")
    self.enabled = self._enabled_requested
    self._frame = 0
    self.gear = 0
    self.scales = STOCK

  @staticmethod
  def jerk_scale(gear: int) -> float:
    return JERK_SCALE_BY_GEAR[gear - 1] if 1 <= gear <= len(JERK_SCALE_BY_GEAR) else 1.0

  def update(self, engaged: bool, CS, pitch: float | None, urgency: str | None, a_cruise_prev: float,
             cruise_target: float, clutch_valid: bool = True) -> tuple[float, float]:
    """Call every planner frame. Returns (rise, fall) scale factors for get_cruise_accel's jerk limit.
    engaged: selfdriveState.enabled (latching); urgency: braking_urgency()."""
    if self._frame % int(1. / DT_MDL) == 0:
      self._enabled_requested = self.available and self.params.get_bool("GearAwareAccel")
    self._frame += 1
    if not engaged:  # never switches mid-engagement
      self.enabled = self._enabled_requested

    usable = self.enabled and clutch_valid and urgency is None and pitch is not None and math.isfinite(pitch)
    self.gear = infer_gear(CS.deprecated.engineRpm, CS.vEgo, CS.deprecated.clutchPressed) if usable else 0
    if self.gear == 0:
      self.scales = STOCK
      return self.scales

    scale = self.jerk_scale(self.gear)
    a_gravity = -G * math.sin(pitch)
    a_eb = estimate_coast_decel(self.gear, CS.deprecated.engineRpm, CS.vEgo, pitch)
    a_eb_credited = CREDIT_MARGIN * (a_eb - a_gravity) + a_gravity
    coast_span = a_cruise_prev > a_eb_credited and cruise_target >= a_eb_credited
    self.scales = (scale, scale if coast_span else 1.0)
    return self.scales
