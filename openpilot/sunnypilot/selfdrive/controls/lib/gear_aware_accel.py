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
  - Falling accel when braking is needed (target below the credited engine-brake decel), FCW, or an urgent lead
    (same test as EngineBrakePreferred): stock rate, never slower braking.
Scales are <= 1.0 (never looser than stock). OFF, automatic, stock ACC, or gear unknown (clutch in, neutral, no RPM)
-> 1.0 = stock. Only the cruise (set-speed) candidate's rate limit is scaled: the lead MPC (its jerk cost, and so FCW)
is untouched, so lead-following braking response is exactly stock.
"""
import math

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import G, estimate_coast_decel, infer_gear
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import CAPACITY_MARGIN, DEFAULT_LEAD_CLOSING, \
                                                                               DEFAULT_LEAD_TTC, lead_urgent

TransmissionType = structs.CarParams.TransmissionType

# TODO(on-car): tune by feel (the 1st/2nd values especially). Index = gear 1..6. Must stay <= 1.0.
JERK_SCALE_BY_GEAR = (0.5, 0.65, 0.8, 0.9, 1.0, 1.0)
STOCK = (1.0, 1.0)


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

  def update(self, long_inactive: bool, CS, pitch: float | None, fcw: bool, radar, a_cruise_prev: float,
             cruise_target: float) -> tuple[float, float]:
    """Call every planner frame. Returns (rise, fall) scale factors for get_cruise_accel's jerk limit."""
    if self._frame % int(1. / DT_MDL) == 0:
      self._enabled_requested = self.available and self.params.get_bool("GearAwareAccel")
    self._frame += 1
    if long_inactive:  # never switches mid-engagement
      self.enabled = self._enabled_requested

    self.gear = infer_gear(CS.deprecated.engineRpm, CS.vEgo, CS.deprecated.clutchPressed) if self.enabled else 0
    if self.gear == 0:
      self.scales = STOCK
      return self.scales

    scale = self.jerk_scale(self.gear)
    grade = pitch if pitch is not None and math.isfinite(pitch) else 0.
    a_gravity = -G * math.sin(grade)
    a_eb = estimate_coast_decel(self.gear, CS.deprecated.engineRpm, CS.vEgo, grade)
    a_eb_credited = CAPACITY_MARGIN * (a_eb - a_gravity) + a_gravity

    urgent = fcw or lead_urgent(radar.leadOne, CS.vEgo, DEFAULT_LEAD_TTC, DEFAULT_LEAD_CLOSING) or \
             lead_urgent(radar.leadTwo, CS.vEgo, DEFAULT_LEAD_TTC, DEFAULT_LEAD_CLOSING)
    coast_span = a_cruise_prev > a_eb_credited and cruise_target >= a_eb_credited
    self.scales = (scale, scale if coast_span and not urgent else 1.0)
    return self.scales
