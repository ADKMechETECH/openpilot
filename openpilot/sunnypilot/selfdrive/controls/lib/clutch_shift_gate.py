"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Clutch/shift gate for manual transmissions (ClutchShiftGate, default OFF; OFF = stock). While the driver shifts,
openpilot must not push THROTTLE into a disconnected engine — but it must never lose BRAKING (Task 12 A2): brakes work
with the clutch in, and a shift is no reason to drop them.

  - A shift does NOT pause longitudinal (longActive stays True). controlsd caps the UPPER accel limit at the coast value
    (ClutchShiftGateCoastAccel, 0.0 by default, clamped -0.5..0.0 — the right value depends on on-car test C2: does an
    ACCEL_COMMAND of 0 coast or hold speed?). The lower limit is untouched: the plan's braking passes through, and every
    other feature may only narrow it under its own Rule-5 rules.
  - No positive accel is ever produced while clutchPressed, in any gate state (the cap is <= 0 whenever the clutch is in).
  - Gas pressed during a paused shift (a rev-match / heel-toe blip — selfdrived drops gasPressedOverride ONLY while the
    gate is actually paused, from the state controlsd publishes): the command is exactly 0, which is what the panda
    requires while gas is pressed (it blocks any other 0x1C8 accel).

Trigger: clutch open (clutchPressed; Honda: GEAR_MT not 1-6 / 14) is authoritative. Secondary cues:
  - ClutchShiftGateGasReleaseHint (default OFF): a short throttle cap after a gas-override release, in case a clutch
    press follows;
  - the RPM-flare backstop (a missed clutch signal while openpilot itself is on the throttle: the engine free-revs):
    rate computed on the filtered rpm over FLARE_WINDOW, judged once per NEW ENGINE_DATA sample, and it must hold for
    FLARE_SAMPLES consecutive new samples with the gas pedal NOT pressed (a driver on the gas is overriding anyway). It
    only starts a cap from idle, never extends a shift, and after FLARE_COOLDOWN_COUNT flares within FLARE_COOLDOWN_WINDOW
    it rests for FLARE_COOLDOWN (a noisy rpm can't keep re-capping).

Resume: fixed REENGAGE_DELAY after GEAR_MT reports a gear, or (ClutchReengageSmart) on ratio lock + gas released for
ClutchReengageGasDebounce; if smart has no lock within SMART_LOCK_TIMEOUT it falls back to the fixed delay (logged) — it
never waits ClutchShiftGateMaxCoast. `resumed` flags the frame a shift ends into idle (controlsd starts the takeover
ramp there; never on the transition into coastTimeout).
Shift timeout (ClutchShiftGateMaxCoast, <= 5 s): clutch in longer than that is neutral-coasting, not a shift -> state
coastTimeout: throttle stays capped at <= 0, nothing else is limited.

Inert (= stock, never enabled) while DisengageOnAccelerator is ON (a blip would disengage at the panda; the gate refuses
and publishes gateBlocked so the HUD says why) and while the clutch signal isn't alive (CarStateSP.clutchSignalValid).
Toggle and params latch only while openpilot is disengaged (Task 12 B3).
"""
from collections import deque
from enum import IntEnum

from opendbc.car.structs import car

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_reengage import RatioLock, ReengageParams, _read_float, \
                                                                       read_reengage_params

TransmissionType = car.CarParams.TransmissionType

# TODO(on-car): verify all timing/threshold constants below against cabana logs of real shifts on the car.
GAS_RELEASE_WINDOW = 0.6  # s, time after a gas release to wait for a clutch press before resuming
REENGAGE_DELAY = 0.5      # s, after the clutch re-engages before resuming (GEAR_MT may report a gear mid-engagement)
SMART_LOCK_TIMEOUT = 1.0  # s after GEAR_MT reports a gear: no ratio lock by then -> the fixed-delay takeover
FLARE_RPM_RATE = 1500.    # rpm/s, RPM rise rate treated as a free-revving (decoupled) engine
FLARE_MAX_A_EGO = 0.5     # m/s^2, ...but only if the car isn't accelerating with it
FLARE_MIN_SPEED = 2.0     # m/s, below this RPM/speed coupling is not meaningful (launch, standstill)
FLARE_CLEAR_TIME = 1.0    # s, the backstop holds the cap until no flare has been seen for this long
FLARE_WINDOW = 0.1        # s, baseline of the filtered-rpm rate
FLARE_SAMPLES = 3         # consecutive NEW ENGINE_DATA samples over the rate
FLARE_SAMPLE_GAP = 0.2    # s with no new ENGINE_DATA sample (a steady rpm repeats its value) ends a flare in progress
FLARE_COOLDOWN_COUNT = 2  # flares within FLARE_COOLDOWN_WINDOW -> rest for FLARE_COOLDOWN
FLARE_COOLDOWN_WINDOW = 10.
FLARE_COOLDOWN = 10.
RPM_FILTER_RC = 0.05      # s
DEFAULT_COAST_ACCEL = 0.0           # m/s^2, the throttle cap during a shift
COAST_ACCEL_RANGE = (-0.5, 0.0)
PAUSED_STATES_NAMES = ("gasReleased", "shifting", "reengaging")


class GateState(IntEnum):
  idle = 0         # not shifting
  gasReleased = 1  # driver just lifted off the gas, waiting to see if the clutch goes in (throttle capped)
  shifting = 2     # clutch in (or RPM flare): throttle capped, braking passes
  reengaging = 3   # clutch back in gear, throttle capped for REENGAGE_DELAY (smart: until ratio lock + gas clear)
  coastTimeout = 4  # clutch in past ClutchShiftGateMaxCoast (neutral coast): throttle capped at 0


PAUSED_STATES = (GateState.gasReleased, GateState.shifting, GateState.reengaging)


class ClutchShiftGate:
  def __init__(self, CP: car.CarParams, params: Params | None = None):
    self.params = params or Params()
    self.available = CP.transmissionType == TransmissionType.manual
    self._enabled_requested = False
    self._hint_requested = False
    self._doa = False
    self._coast_requested = DEFAULT_COAST_ACCEL
    self._reengage_requested = ReengageParams()
    self.get_params()
    self.enabled = self.blocked = False
    self.gas_release_hint = False
    self.coast_accel = DEFAULT_COAST_ACCEL
    self.reengage = ReengageParams()
    self._latch(True)

    self.state = GateState.idle
    self.timer = 0.0
    self.flare_clear_timer = 0.0
    self.gas_pressed_prev = False
    self.rpm_filter = FirstOrderFilter(0., RPM_FILTER_RC, DT_CTRL)
    self.rpm_hist: deque[float] = deque(maxlen=max(1, round(FLARE_WINDOW / DT_CTRL)) + 1)
    self.rpm_raw_prev: float | None = None
    self.flare_samples = 0
    self.since_sample = 0.
    self.flare_times: deque[float] = deque(maxlen=FLARE_COOLDOWN_COUNT)
    self.flare_rest = 0.
    self.flare_prev = False
    self.t = 0.
    self.ratio_lock = RatioLock()
    self.gas_clear_time = 0.
    self.reengage_time = 0.
    self.coast_time = 0.
    self.fallback = False
    self.paused = False
    self.resumed = False  # True on the frame a shift ended into idle (controlsd starts the takeover ramp there)
    self.clutch = False
    self.gas = False

  def get_params(self) -> None:
    # 1 Hz; applied by _latch() only while openpilot is disengaged (Task 12 B3)
    self._enabled_requested = self.available and self.params.get_bool("ClutchShiftGate")
    self._hint_requested = self.params.get_bool("ClutchShiftGateGasReleaseHint")
    self._doa = self.params.get_bool("DisengageOnAccelerator")
    self._coast_requested = _read_float(self.params, "ClutchShiftGateCoastAccel", DEFAULT_COAST_ACCEL, COAST_ACCEL_RANGE)
    self._reengage_requested = read_reengage_params(self.params) if self.available else ReengageParams()

  def _latch(self, clutch_valid: bool) -> None:
    blocked = self._enabled_requested and (self._doa or not clutch_valid)
    if blocked and not self.blocked:
      why = "DisengageOnAccelerator is ON" if self._doa else "no clutch signal"
      cloudlog.warning(f"clutch_shift_gate: refused ({why}) - the gate stays off")
    self.blocked = blocked
    self.enabled = self._enabled_requested and not blocked
    self.gas_release_hint = self._hint_requested
    self.coast_accel = self._coast_requested
    self.reengage = self._reengage_requested

  def _rpm_flare(self, CS: car.CarState) -> bool:
    """A new ENGINE_DATA sample: filtered-rpm rate over FLARE_WINDOW above FLARE_RPM_RATE, with the car not
    accelerating with it and the driver off the gas, for FLARE_SAMPLES new samples in a row."""
    rpm = CS.deprecated.engineRpm
    if self.rpm_raw_prev is None:  # seed from the first sample so startup isn't read as a flare
      self.rpm_filter.x = rpm
      self.rpm_raw_prev = rpm
    rpm_f = self.rpm_filter.update(rpm)
    self.rpm_hist.append(rpm_f)
    self.since_sample += DT_CTRL
    if rpm != self.rpm_raw_prev:
      self.rpm_raw_prev = rpm
      self.since_sample = 0.
      rate = (rpm_f - self.rpm_hist[0]) / max((len(self.rpm_hist) - 1) * DT_CTRL, DT_CTRL)
      cond = CS.vEgo > FLARE_MIN_SPEED and rate > FLARE_RPM_RATE and CS.aEgo < FLARE_MAX_A_EGO and not CS.gasPressed
      self.flare_samples = self.flare_samples + 1 if cond else 0
    elif CS.gasPressed or self.since_sample > FLARE_SAMPLE_GAP:
      self.flare_samples = 0   # the driver's on the gas, or the rpm has stopped moving: no flare in progress
    flare = self.flare_samples >= FLARE_SAMPLES
    # cool-down: after FLARE_COOLDOWN_COUNT flares within FLARE_COOLDOWN_WINDOW the backstop rests (noise, not shifts)
    self.flare_rest = max(self.flare_rest - DT_CTRL, 0.)
    out = flare and self.flare_rest <= 0.
    if out and not self.flare_prev:
      self.flare_times.append(self.t)
      if len(self.flare_times) == FLARE_COOLDOWN_COUNT and self.t - self.flare_times[0] <= FLARE_COOLDOWN_WINDOW:
        self.flare_rest = FLARE_COOLDOWN
        self.flare_times.clear()
        cloudlog.event("clutch_shift_gate_flare_cooldown", flares=FLARE_COOLDOWN_COUNT)
    self.flare_prev = out
    return out

  def _set_state(self, state: GateState) -> None:
    if state != self.state:
      cloudlog.event("clutch_shift_gate", state=state.name, prev=self.state.name)
      self.state = state

  def update(self, CS: car.CarState, enabled: bool, gear_raw: int = -1, clutch_valid: bool = False) -> GateState:
    """Call every controls frame. enabled = openpilot engaged (latching); gear_raw / clutch_valid from CarStateSP."""
    self.t += DT_CTRL
    flare = self._rpm_flare(CS)
    gas_released = self.gas_pressed_prev and not CS.gasPressed
    self.gas_pressed_prev = CS.gasPressed
    self.gas_clear_time = 0. if CS.gasPressed else min(self.gas_clear_time + DT_CTRL, 10.)
    self.clutch, self.gas = bool(CS.deprecated.clutchPressed), bool(CS.gasPressed)

    if not enabled:
      self._latch(clutch_valid)
    elif self.enabled and not clutch_valid:
      self.enabled = False   # the clutch signal died mid-engagement: inert (stock) until re-engaged with it alive
      self.blocked = True
    smart = self.reengage.smart
    coupled = self.ratio_lock.update(CS, gear_raw, self.reengage.ratio_tol) if smart else False
    # smart: a ratio-locked engine is coupled by definition, so an RPM rise then is not a flare
    flare = flare and not coupled
    if not self.enabled:
      self._set_state(GateState.idle)
      self.flare_clear_timer = 0.
      return self._result()

    clutch = self.clutch
    # The backstop only *starts* a cap (a missed clutch signal). It must not extend a shift already in progress:
    # the RPM jump as the clutch bites on a downshift looks like a flare but is the engine re-coupling.
    if clutch:
      self.flare_clear_timer = 0.
    elif flare and self.state in (GateState.idle, GateState.gasReleased):
      self.flare_clear_timer = FLARE_CLEAR_TIME
    else:
      self.flare_clear_timer = max(self.flare_clear_timer - DT_CTRL, 0.)
    decoupled = clutch or self.flare_clear_timer > 0.

    if decoupled:
      if self.state != GateState.coastTimeout:  # a timed-out coast stays timed out until the clutch re-engages
        self._set_state(GateState.shifting)
    elif CS.gasPressed and not (smart and self.state in (GateState.shifting, GateState.reengaging, GateState.coastTimeout)):
      # clutch engaged and driver on the gas: a real override (stock gas override takes longitudinal)
      self._set_state(GateState.idle)
    elif self.state in (GateState.shifting, GateState.coastTimeout):
      self._set_state(GateState.reengaging)
      self.timer = REENGAGE_DELAY
      self.reengage_time = 0.
      self.fallback = False
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

    self._shift_timeout(CS, clutch)
    return self._result()

  def _ratio_lock_valid(self, CS: car.CarState) -> bool:
    return CS.vEgo >= self.reengage.ratio_min_speed

  def _takeover_ready(self, CS: car.CarState, coupled: bool) -> bool:
    """Smart: one rule for every shift - drivetrain coupled AND the driver off the gas (debounced)."""
    if not self.fallback and not coupled and self._ratio_lock_valid(CS) and self.reengage_time >= SMART_LOCK_TIMEOUT - 1e-6:
      self.fallback = True
      cloudlog.event("clutch_reengage_no_lock", t=round(self.reengage_time, 2), v_ego=round(CS.vEgo, 2))
    # below the ratio lock's valid speed, or no lock in time: the fixed delay after GEAR_MT is the fallback
    fixed = self.reengage_time >= REENGAGE_DELAY - 1e-6
    coupled_ok = coupled if self._ratio_lock_valid(CS) and not self.fallback else fixed
    gas_ok = not CS.gasPressed and self.gas_clear_time >= self.reengage.gas_debounce - 1e-6
    return coupled_ok and gas_ok

  def _shift_timeout(self, CS: car.CarState, clutch: bool) -> None:
    """8b: bound the coast. Clutch in longer than ClutchShiftGateMaxCoast (<= 5 s) is neutral-coasting, not a shift."""
    if self.state != GateState.shifting or not clutch:
      self.coast_time = 0.
      return
    self.coast_time += DT_CTRL
    if self.coast_time > self.reengage.max_coast:
      cloudlog.event("clutch_shift_gate_timeout", reason="neutral_coast", coast_time=round(self.coast_time, 2), v_ego=CS.vEgo)
      self._set_state(GateState.coastTimeout)
      self.coast_time = 0.

  def _result(self) -> GateState:
    paused = self.state in PAUSED_STATES
    self.resumed = self.paused and not paused and self.state == GateState.idle
    self.paused = paused
    return self.state

  def limit_accel(self, accel_limits: tuple[float, float], urgency: str | None = None) -> tuple[float, float]:
    """controlsd: cap the THROTTLE side only (braking passes). Gas pressed during a paused shift -> exactly 0.
    Rule 5: the lower limit is never raised here, so urgency has nothing to release - except the gas case, where the
    panda (not this code) blocks any nonzero ACC_CONTROL accel while gas is pressed, urgent or not. That is the same
    outcome as stock's gasPressedOverride (longActive False: no braking while the driver is on the gas)."""
    if not self.enabled:
      return accel_limits
    lo, hi = accel_limits
    if self.paused:
      if self.gas:   # a blip with the override dropped: the panda allows only an inactive (0) accel while gas is pressed
        return 0., 0.   # (with or without urgency: see above)
      hi = min(hi, self.coast_accel)
    if self.state == GateState.coastTimeout or self.clutch:
      hi = min(hi, 0.)   # never throttle into a disconnected engine
    return min(lo, hi), hi
