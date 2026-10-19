"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Clutch/shift gate for manual transmissions: while the driver is shifting, pause openpilot longitudinal
(longActive=False) so the car coasts with no gas and no brake, without disengaging — mimicking the stock
PCM's fuel cut on clutch-in. Longitudinal resumes once the clutch is re-engaged; the planner re-seeds from
aEgo and LongControl restarts from zero, so the resume is smooth.

Clutch-open (clutchPressed; Honda: GEAR_MT == 0, a PCM-inferred gear, not a raw switch) is the AUTHORITATIVE
trigger. While it is open the gas pedal is ignored for longitudinal purposes: a rev-match / heel-toe blip only
revs a disconnected engine, so it is never treated as a driver accel request (see also car_specific.py, which
drops gasPressedOverride while the clutch is open). Normal gas override resumes once the clutch re-engages.

Optional/secondary cues:
  - gas released after a driver override (ClutchShiftGateGasReleaseHint, default OFF): a short "anticipate" hold
    in case a clutch press follows. Never the sole decider of a shift; off by default because it also delays
    resuming longitudinal after every plain gas override.
  - RPM flare backstop: RPM rising fast while the car isn't accelerating (engine decoupled). This lags the
    clutch (the flare happens after it opens), so it is only a backstop for a missed/late clutch signal, and it
    can only start a pause, never extend one (a rev-match blip or downshift re-coupling also flares).

Requires DisengageOnAccelerator OFF (the default): with it ON the panda itself drops controls on any gas rising
edge, so a rev-match blip disengages openpilot and that cannot be suppressed without changing panda safety.

Smart re-engagement (ClutchReengageSmart, default OFF; OFF = the fixed REENGAGE_DELAY above, unchanged):
  - the re-engage hold ends on RATIO LOCK (clutch_reengage.RatioLock: rpm/speed matches a gear) instead of a fixed
    timer; GEAR_MT reporting a gear stays the pre-cue. Below the ratio lock's valid speed (max(3 m/s, handback speed))
    the fixed REENGAGE_DELAY is the fallback.
  - one takeover rule for every shift: coupled AND gas released for ClutchReengageGasDebounce. Off-throttle (incl. a
    downshift into engine braking) that is immediate at the bite; on-throttle (upshift) it waits for the lift.
  - at the takeover controlsd seeds the command from aEgo and ramps to the plan (clutch_reengage.TakeoverRamp);
    `resumed` flags that frame.
Shift timeout (ClutchShiftGateMaxCoast, active whenever the gate is on): uncoupled for longer than that is not a shift
but neutral-coasting while rolling — stop forcing the coast (state coastTimeout: longitudinal resumes so openpilot can
brake again, with the command capped at <= 0 while GEAR_MT still reads 0, since throttle would only rev a disconnected
engine). If GEAR_MT shows a gear but the ratio never locks (wrong seed ratios, a slipping clutch), the gate goes idle.
"""
from enum import IntEnum

from opendbc.car.structs import car

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_reengage import RatioLock, ReengageParams, read_reengage_params

TransmissionType = car.CarParams.TransmissionType

# TODO(on-car): verify all timing/threshold constants below against cabana logs of real shifts on the car.
# TODO(on-car): verify feel when downshifting to engine-brake while longitudinal is active and the brake pedal is NOT
#  pressed: after the re-engage delay openpilot resumes and may add throttle to hold set speed, fighting the
#  driver's intended engine braking. Road-test tuning item (e.g. a longer hold after downshifts), no code change yet.
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
  reengaging = 3   # clutch back in gear, coasting for REENGAGE_DELAY (smart: until ratio lock + gas clear) before resuming
  coastTimeout = 4  # uncoupled past ClutchShiftGateMaxCoast (neutral coast): not paused, throttle capped at 0


class ClutchShiftGate:
  def __init__(self, CP: car.CarParams, params: Params | None = None):
    self.params = params or Params()
    self.available = CP.transmissionType == TransmissionType.manual
    self._enabled_requested = False
    self.gas_release_hint = False
    self._warned_disengage_on_gas = False
    self._reengage_requested = ReengageParams()
    self.get_params()
    self.enabled = self._enabled_requested
    self.reengage = self._reengage_requested

    self.state = GateState.idle
    self.timer = 0.0
    self.flare_clear_timer = 0.0
    self.gas_pressed_prev = False
    self.rpm_filter = FirstOrderFilter(0., RPM_FILTER_RC, DT_CTRL)
    self.rpm_prev = 0.
    self.rpm_seeded = False
    self.ratio_lock = RatioLock()
    self.gas_clear_time = 0.
    self.reengage_time = 0.
    self.coast_time = 0.
    self.paused = False
    self.resumed = False  # True on the frame a pause ended (controlsd starts the smart takeover there)

  def get_params(self) -> None:
    # applied in update() only while idle, so toggling never cuts a shift short
    self._enabled_requested = self.available and self.params.get_bool("ClutchShiftGate")
    self.gas_release_hint = self.params.get_bool("ClutchShiftGateGasReleaseHint")
    self._reengage_requested = read_reengage_params(self.params) if self.available else ReengageParams()
    if self._enabled_requested and self.params.get_bool("DisengageOnAccelerator") and not self._warned_disengage_on_gas:
      cloudlog.warning("clutch_shift_gate: DisengageOnAccelerator is ON - a rev-match throttle blip will disengage openpilot")
      self._warned_disengage_on_gas = True

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
    self.gas_clear_time = 0. if CS.gasPressed else min(self.gas_clear_time + DT_CTRL, 10.)

    if self.state == GateState.idle:
      self.enabled = self._enabled_requested
      self.reengage = self._reengage_requested
    smart = self.reengage.smart
    coupled = self.ratio_lock.update(CS, self.reengage.ratio_tol) if smart else False
    # smart: a ratio-locked engine is coupled by definition, so an RPM rise then (the tail of a downshift's re-coupling,
    # which the smart takeover no longer waits out) is not a flare
    flare = flare and not coupled
    if not self.enabled:
      self._set_state(GateState.idle)
      return self._result(False)

    # TODO(on-car): GEAR_MT == 0 means "no gear meshed": it reads 0 for clutch pressed AND for neutral with the clutch
    #  out. Coasting in neutral WHILE ROLLING with openpilot engaged is bounded by ClutchShiftGateMaxCoast (state
    #  coastTimeout) - verify what GEAR_MT reads there and that the timeout fires (test H3).
    clutch = CS.deprecated.clutchPressed
    # The backstop only *starts* a pause (a missed clutch signal). It must not extend a shift already in progress:
    # the RPM jump as the clutch bites on a downshift looks like a flare but is the engine re-coupling.
    if clutch:
      # the authoritative signal owns this shift: don't arm the backstop (a blip on the first clutch frame would
      # otherwise hold the pause past clutch-out), and cancel a backstop hold already running
      self.flare_clear_timer = 0.
    elif flare and self.state in (GateState.idle, GateState.gasReleased):
      self.flare_clear_timer = FLARE_CLEAR_TIME
    else:
      self.flare_clear_timer = max(self.flare_clear_timer - DT_CTRL, 0.)
    decoupled = clutch or self.flare_clear_timer > 0.

    if decoupled:
      # clutch open: gas (rev-match / heel-toe blip) is deliberately ignored here
      if self.state != GateState.coastTimeout:  # a timed-out coast stays timed out until the clutch re-engages
        self._set_state(GateState.shifting)
    elif CS.gasPressed and not (smart and self.state in (GateState.shifting, GateState.reengaging, GateState.coastTimeout)):
      # clutch engaged and driver on the gas: a real override, which already pauses longitudinal
      # (smart: after a shift, stay in reengaging - the takeover waits for the driver to roll off the gas)
      self._set_state(GateState.idle)
    elif self.state in (GateState.shifting, GateState.coastTimeout):
      self._set_state(GateState.reengaging)
      self.timer = REENGAGE_DELAY
      self.reengage_time = 0.
    elif gas_released and self.state == GateState.idle and self.gas_release_hint:
      self._set_state(GateState.gasReleased)
      self.timer = GAS_RELEASE_WINDOW
    elif self.state == GateState.reengaging and smart:
      self.reengage_time += DT_CTRL
      if self._takeover_ready(CS, coupled):
        self._set_state(GateState.idle)
    elif self.state in (GateState.gasReleased, GateState.reengaging):
      self.timer -= DT_CTRL
      if self.timer <= 0.:
        self._set_state(GateState.idle)

    self._shift_timeout(CS, clutch, coupled)
    return self._result(self.state not in (GateState.idle, GateState.coastTimeout))

  def _ratio_lock_valid(self, CS: car.CarState) -> bool:
    return CS.vEgo >= self.reengage.ratio_min_speed

  def _takeover_ready(self, CS: car.CarState, coupled: bool) -> bool:
    """Smart: one rule for every shift - drivetrain coupled AND the driver off the gas (debounced)."""
    # below the ratio lock's valid speed (near idle / handback territory): the fixed delay after GEAR_MT is the fallback
    coupled_ok = coupled if self._ratio_lock_valid(CS) else self.reengage_time >= REENGAGE_DELAY - 1e-6
    gas_ok = not CS.gasPressed and self.gas_clear_time >= self.reengage.gas_debounce - 1e-6
    return coupled_ok and gas_ok

  def _shift_timeout(self, CS: car.CarState, clutch: bool, coupled: bool) -> None:
    """8b: bound the coast. Uncoupled longer than ClutchShiftGateMaxCoast is neutral-coasting, not a shift."""
    waiting_on_gas = self.reengage.smart and self.state == GateState.reengaging and \
                     (coupled if self._ratio_lock_valid(CS) else self.reengage_time >= REENGAGE_DELAY - 1e-6)
    if self.state not in (GateState.shifting, GateState.reengaging) or waiting_on_gas:
      self.coast_time = 0.
      return
    self.coast_time += DT_CTRL
    if self.coast_time > self.reengage.max_coast:
      reason = "neutral_coast" if clutch else "no_ratio_lock"
      cloudlog.event("clutch_shift_gate_timeout", reason=reason, coast_time=round(self.coast_time, 2), v_ego=CS.vEgo)
      if not clutch:
        msg = "GEAR_MT shows a gear but the rpm/speed ratio never locked - check the gear ratios / ClutchReengageRatioTol"
        cloudlog.warning(f"clutch_shift_gate: {msg}")
      self._set_state(GateState.coastTimeout if clutch else GateState.idle)
      self.coast_time = 0.

  def _result(self, paused: bool) -> bool:
    self.resumed = self.paused and not paused
    self.paused = paused
    return paused

  def limit_accel(self, accel_limits: tuple[float, float]) -> tuple[float, float]:
    """controlsd: after a coast timeout, no throttle while the engine is still disconnected (braking stays available)."""
    if self.state != GateState.coastTimeout:
      return accel_limits
    hi = min(accel_limits[1], 0.)
    return min(accel_limits[0], hi), hi
