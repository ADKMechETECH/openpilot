"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set-speed (cruise candidate) profile, two opt-in toggles (session 10 owner decisions, 2026-10-21). Transmission- and
platform-agnostic like non-adaptive: available only when openpilot owns longitudinal. Both shape only the planner's
set-speed candidate (get_cruise_accel); the lead MPC, e2e and FCW are untouched. Latched only while disengaged.

SetSpeedCoast (default OFF): a gentle "coast down to the set speed". While the set speed is at or above
SetSpeedCoastMinSpeed (default 24.6 m/s = 55 mph), the cruise candidate asks for no more than SetSpeedCoastDecel
(default 0.12 m/s^2) of deceleration instead of stock A_CRUISE_MIN (-1.2). Session 10 (route 00000008): at 56-78 mph
in 6th the PCM meets a -0.05..-0.15 request on part throttle (-0.14 delivered, +114 Nm), while -0.15..-0.25 already
closes the throttle (engine braking, -0.37); below ~55 mph any negative request closed the throttle in 4th, so the cap
only buys a gentler slow-down at highway speeds - hence the set-speed threshold (the car is above the set speed while
the cap matters, so it is in that regime too). On a descent the car's brake controller tracks the request, so the car
still slows at the capped rate - it never runs away; a large set-speed drop just takes long (5 mph at 0.12: ~19 s).
  Rule 5: with any braking_urgency() reason the cap is gone the same planner frame (stock A_CRUISE_MIN), and plannerd
  publishes the stock plan (features.aTargetStock) so controlsd runs min(aTarget, stock) in the same control frame -
  the GearAware mechanism (Task 13 A3). forceDecel sets the cruise target to 0, below the threshold: stock.

CruiseAccelBoost (default OFF): a higher set-speed accel ceiling above 10 m/s - get_max_accel's table
[1.6, 1.2, 0.8, 0.6] over [0, 10, 25, 40] m/s becomes [1.6, 1.2, 1.1, 0.9]. Stock takes >= 9.2 s for 60 -> 75 mph
(0.78 -> 0.69 m/s^2); boosted about 6.6 s. The driver's own re-accel in 6th on route 00000008 was 0.9-1.2 m/s^2. The
set-speed jerk limit is unchanged, and the ceiling also bounds the whole plan (as stock get_max_accel does). Throttle
side only: not a Rule-5 path.
"""
import numpy as np

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog

A_CRUISE_MIN = -1.2                    # longitudinal_planner.A_CRUISE_MIN (stock)
A_CRUISE_MAX_BP = [0., 10., 25., 40.]  # longitudinal_planner.A_CRUISE_MAX_BP (stock)
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_VALS_BOOST = [1.6, 1.2, 1.1, 0.9]

COAST_DECEL_DEFAULT, COAST_DECEL_RANGE = 0.12, (0.05, 0.3)
COAST_MIN_SPEED_DEFAULT, COAST_MIN_SPEED_RANGE = 24.6, (20., 40.)


def _read_float(params: Params, key: str, default: float, rng: tuple[float, float]) -> float:
  try:
    value = float(params.get(key, return_default=True))
  except (TypeError, ValueError):
    value = default
  return float(np.clip(value, *rng)) if np.isfinite(value) else default


class CruiseProfile:
  def __init__(self, CP: structs.CarParams, params: Params | None = None):
    self.params = params or Params()
    self.available = bool(CP.openpilotLongitudinalControl)
    self._frame = 0
    self._read()
    self.coast_enabled, self.boost_enabled = self._coast_requested, self._boost_requested
    self.coast_decel, self.coast_min_speed = self._coast_decel_requested, self._coast_min_speed_requested
    self.coast_active = False
    self.a_min = A_CRUISE_MIN
    self.a_max_vals = A_CRUISE_MAX_VALS

  def _read(self) -> None:
    p = self.params
    self._coast_requested = self.available and p.get_bool("SetSpeedCoast")
    self._boost_requested = self.available and p.get_bool("CruiseAccelBoost")
    self._coast_decel_requested = _read_float(p, "SetSpeedCoastDecel", COAST_DECEL_DEFAULT, COAST_DECEL_RANGE)
    self._coast_min_speed_requested = _read_float(p, "SetSpeedCoastMinSpeed", COAST_MIN_SPEED_DEFAULT, COAST_MIN_SPEED_RANGE)

  @property
  def shaped(self) -> bool:
    """The set-speed candidate may differ from stock in braking (the planner then keeps the stock shadow)."""
    return self.coast_enabled

  def update(self, engaged: bool, v_cruise: float, v_ego: float, urgency: str | None) -> tuple[float, list[float]]:
    """Call every planner frame. Returns (a_min, a_max_vals) for get_cruise_accel. engaged: selfdriveState.enabled
    (latching); urgency: braking_urgency() this frame (Rule 5)."""
    if self._frame % int(1. / DT_MDL) == 0:
      self._read()
    self._frame += 1
    if not engaged:  # never switches mid-engagement
      self.coast_enabled, self.boost_enabled = self._coast_requested, self._boost_requested
      self.coast_decel, self.coast_min_speed = self._coast_decel_requested, self._coast_min_speed_requested

    active = self.coast_enabled and engaged and urgency is None and v_cruise >= self.coast_min_speed
    if active != self.coast_active:
      cloudlog.event("set_speed_coast", active=active, urgency=urgency or "", v_cruise=round(v_cruise, 2),
                     v_ego=round(v_ego, 2))
    self.coast_active = active
    self.a_min = -self.coast_decel if active else A_CRUISE_MIN
    self.a_max_vals = A_CRUISE_MAX_VALS_BOOST if self.boost_enabled else A_CRUISE_MAX_VALS
    return self.a_min, self.a_max_vals
