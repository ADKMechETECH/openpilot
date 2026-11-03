"""
fork-only (civic-cruise) clutch/shift gate (manual only) - MINIMAL scope.

While the drivetrain is decoupled (clutch in, or neutral) the gate caps the THROTTLE (the upper accel limit) so a large
pending throttle request can't flare the engine during a shift. Braking always passes (the lower limit is untouched),
so the non-ACC engine-brake floor and emergency braking are never blocked. longActive is never dropped.

This is insurance only: on this car the PCM already drops throttle on clutch-in for small requests (Stage 1/3), and the
confirmed shift-time surge (the driver's rev-match blip) is handled by the always-on gas-override guard in the planner.
The gate covers the untested large-request case and bounds a rolling neutral coast.

Deliberately minimal (owner: "do minimal, with the option for the full gate if needed"). NOT included - add here if logs
show a need: an RPM-flare backstop (detect a decoupled engine from an rpm rise when the clutch signal misses it), a
ratio-lock smart re-engagement, and a seeded takeover ramp to smooth the resume. Hook points are marked below.

States: idle -> shifting (decoupled) -> reengaging (coupled for REENGAGE_DELAY, gear steady) -> idle.
coastTimeout = decoupled past MaxCoast, or confirmed neutral (clutch pedal fully up for NEUTRAL_CONFIRM): same throttle
cap, bounded. Toggle ClutchShiftGate (default OFF); needs the clutch signal live, else the gate stays off.
"""
from enum import IntEnum

import numpy as np

from openpilot.common.params import Params
from opendbc.car import structs

DEFAULT_COAST_ACCEL = 0.0        # m/s^2: the throttle cap during a shift
COAST_ACCEL_RANGE = (-0.5, 0.0)
DEFAULT_MAX_COAST = 5.0          # s: decoupled longer than this (rolling neutral) -> coastTimeout
MAX_COAST_RANGE = (2.0, 20.0)
REENGAGE_DELAY = 0.15            # s coupled (gear steady) before resuming (Stage 1: GEAR_MT reports at full engagement)
NEUTRAL_CONFIRM = 0.5            # s decoupled with the clutch pedal fully up = neutral -> coastTimeout at once

CLUTCH_PEDAL_UP = 1              # CarStateSP.clutchPedal fully-up value


class GateState(IntEnum):
  idle = 0
  shifting = 2        # clutch in / decoupled
  reengaging = 3      # clutch back in gear, waiting REENGAGE_DELAY
  coastTimeout = 4    # decoupled past MaxCoast, or confirmed neutral


PAUSED = (GateState.shifting, GateState.reengaging, GateState.coastTimeout)


class ClutchShiftGate:
  def __init__(self, CP: structs.CarParams, dt: float, params: Params | None = None):
    self._params = params or Params()
    self.dt = dt
    self.available = CP.transmissionType == structs.CarParams.TransmissionType.manual
    self._frame = 0
    self._read_period = max(1, int(round(1.0 / dt)))
    self.enabled = False
    self.coast_accel = DEFAULT_COAST_ACCEL
    self.max_coast = DEFAULT_MAX_COAST
    self._read_params()

    self.state = GateState.idle
    self.decoupled_t = 0.
    self.coupled_t = 0.
    self.lock_gear = None

  def _read_params(self) -> None:
    self.enabled = self.available and bool(self._params.get_bool("ClutchShiftGate"))
    self.coast_accel = float(np.clip(self._params.get("ClutchShiftGateCoastAccel", return_default=True) or 0.0,
                                     *COAST_ACCEL_RANGE))
    self.max_coast = float(np.clip(self._params.get("ClutchShiftGateMaxCoast", return_default=True) or DEFAULT_MAX_COAST,
                                   *MAX_COAST_RANGE))

  def update(self, *, clutch_signal_valid: bool, decoupled: bool, clutch_pedal: int, gear_raw: int) -> GateState:
    if self._frame % self._read_period == 0:
      self._read_params()
    self._frame += 1

    if not (self.enabled and clutch_signal_valid):
      self.state = GateState.idle
      return self.state

    # (full-gate hook) an RPM-flare backstop would OR into `decoupled` here when the clutch signal misses a shift.
    neutral = decoupled and clutch_pedal == CLUTCH_PEDAL_UP   # pedal up, no gear = neutral, not a shift

    if self.state == GateState.idle:
      if decoupled:
        self.decoupled_t = 0.
        self.state = GateState.shifting

    elif self.state == GateState.shifting:
      if decoupled:
        self.decoupled_t += self.dt
        if (neutral and self.decoupled_t >= NEUTRAL_CONFIRM) or self.decoupled_t >= self.max_coast:
          self.state = GateState.coastTimeout
      else:
        self.coupled_t = 0.
        self.lock_gear = gear_raw
        self.state = GateState.reengaging

    elif self.state == GateState.reengaging:
      if decoupled:
        self.decoupled_t = 0.
        self.state = GateState.shifting
      else:
        if gear_raw != self.lock_gear:          # gear still settling, restart the delay
          self.lock_gear = gear_raw
          self.coupled_t = 0.
        self.coupled_t += self.dt
        if self.coupled_t >= REENGAGE_DELAY:
          self.state = GateState.idle           # (full-gate hook) start a seeded takeover ramp here

    elif self.state == GateState.coastTimeout:
      if not decoupled:
        self.coupled_t = 0.
        self.lock_gear = gear_raw
        self.state = GateState.reengaging

    return self.state

  @property
  def paused(self) -> bool:
    return self.state in PAUSED

  def limit_accel(self, lower: float, upper: float) -> tuple[float, float]:
    """Cap the throttle while decoupled; braking (the lower limit) always passes."""
    if self.paused:
      return lower, min(upper, self.coast_accel)
    return lower, upper
