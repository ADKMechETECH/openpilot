"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Clutch/shift gate for manual transmissions: while the driver is shifting, pause openpilot longitudinal
(longActive=False) so the car coasts with no gas and no brake, without disengaging — mimicking the stock
PCM's fuel cut on clutch-in. Longitudinal resumes once the clutch is re-engaged; the planner re-seeds from
aEgo and LongControl restarts from zero, so the resume is smooth.

Detection, fastest to slowest:
  1. gas pedal released after a driver override: hold the pause briefly in case a clutch press follows
  2. clutchPressed (Honda: GEAR_MT == 0, a PCM-inferred gear, not a raw switch)
  3. RPM flare backstop: RPM rising fast while the car isn't accelerating (engine decoupled). This lags
     the clutch (the flare happens after it opens), so it is only a backstop for a missed/late signal.
"""
from enum import IntEnum

from opendbc.car.structs import car

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog

TransmissionType = car.CarParams.TransmissionType

# TODO(on-car): verify all timing/threshold constants below against cabana logs of real shifts on the car.
GAS_RELEASE_WINDOW = 0.6  # s, time after a gas release to wait for a clutch press before resuming
REENGAGE_DELAY = 0.5      # s, coast after the clutch re-engages before resuming (GEAR_MT may report a gear mid-engagement)
FLARE_RPM_RATE = 1500.    # rpm/s, RPM rise rate treated as a free-revving (decoupled) engine
FLARE_MAX_A_EGO = 0.5     # m/s^2, ...but only if the car isn't accelerating with it
FLARE_MIN_SPEED = 2.0     # m/s, below this RPM/speed coupling is not meaningful (launch, standstill)
FLARE_CLEAR_TIME = 1.0    # s, the backstop holds the pause until no flare has been seen for this long
RPM_FILTER_RC = 0.05      # s


class GateState(IntEnum):
  idle = 0         # not shifting, longitudinal allowed
  gasReleased = 1  # driver just lifted off the gas, waiting to see if the clutch goes in
  shifting = 2     # clutch in (or RPM flare), coasting
  reengaging = 3   # clutch back in gear, coasting for REENGAGE_DELAY before resuming


class ClutchShiftGate:
  def __init__(self, CP: car.CarParams, params: Params | None = None):
    self.params = params or Params()
    self.available = CP.transmissionType == TransmissionType.manual
    self._enabled_requested = False
    self.get_params()
    self.enabled = self._enabled_requested

    self.state = GateState.idle
    self.timer = 0.0
    self.flare_clear_timer = 0.0
    self.gas_pressed_prev = False
    self.rpm_filter = FirstOrderFilter(0., RPM_FILTER_RC, DT_CTRL)
    self.rpm_prev = 0.
    self.rpm_seeded = False

  def get_params(self) -> None:
    # applied in update() only while idle, so toggling never cuts a shift short
    self._enabled_requested = self.available and self.params.get_bool("ClutchShiftGate")

  def _rpm_flare(self, CS: car.CarState) -> bool:
    if not self.rpm_seeded:  # seed from the first sample so startup isn't read as a flare
      self.rpm_filter.x = self.rpm_prev = CS.deprecated.engineRpm
      self.rpm_seeded = True
    rpm = self.rpm_filter.update(CS.deprecated.engineRpm)
    rpm_rate = (rpm - self.rpm_prev) / DT_CTRL
    self.rpm_prev = rpm
    # TODO(on-car): verify ENGINE_RPM update rate/noise keeps rpm_rate clean enough for this threshold
    return CS.vEgo > FLARE_MIN_SPEED and rpm_rate > FLARE_RPM_RATE and CS.aEgo < FLARE_MAX_A_EGO

  def _set_state(self, state: GateState) -> None:
    if state != self.state:
      cloudlog.event("clutch_shift_gate", state=state.name, prev=self.state.name)
      self.state = state

  def update(self, CS: car.CarState) -> bool:
    """Call every controls frame. Returns True while longitudinal should be paused (coasting)."""
    flare = self._rpm_flare(CS)
    gas_released = self.gas_pressed_prev and not CS.gasPressed
    self.gas_pressed_prev = CS.gasPressed

    if self.state == GateState.idle:
      self.enabled = self._enabled_requested
    if not self.enabled:
      self._set_state(GateState.idle)
      return False

    clutch = CS.deprecated.clutchPressed
    # The backstop only *starts* a pause (a missed clutch signal). It must not extend a shift already in progress:
    # the RPM jump as the clutch bites on a downshift looks like a flare but is the engine re-coupling.
    if flare and self.state in (GateState.idle, GateState.gasReleased):
      self.flare_clear_timer = FLARE_CLEAR_TIME
    else:
      self.flare_clear_timer = max(self.flare_clear_timer - DT_CTRL, 0.)
    decoupled = clutch or self.flare_clear_timer > 0.

    if decoupled:
      self._set_state(GateState.shifting)
    elif CS.gasPressed:
      # driver is overriding; the gas override already pauses longitudinal
      self._set_state(GateState.idle)
    elif self.state == GateState.shifting:
      self._set_state(GateState.reengaging)
      self.timer = REENGAGE_DELAY
    elif gas_released and self.state == GateState.idle:
      self._set_state(GateState.gasReleased)
      self.timer = GAS_RELEASE_WINDOW
    elif self.state in (GateState.gasReleased, GateState.reengaging):
      self.timer -= DT_CTRL
      if self.timer <= 0.:
        self._set_state(GateState.idle)

    return self.state != GateState.idle
