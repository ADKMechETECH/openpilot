"""
fork-only (civic-cruise) Phase 4 (minimal gate): the clutch/shift gate caps the throttle while the drivetrain is
decoupled; braking always passes; neutral / long coasts go to coastTimeout. Toggle + signal-liveness gated.
"""
from openpilot.common.params import Params
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.clutch_gate import ClutchShiftGate, GateState, RELEASE_JERK

DT = 0.01
PEDAL_UP, PEDAL_TRAVEL = 1, 2


def _gate(enabled=True):
  Params().put_bool("ClutchShiftGate", enabled, block=True)
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)   # manual
  return ClutchShiftGate(CP, DT)


def _step(g, n, **kw):
  frame = {"clutch_signal_valid": True, "decoupled": True, "clutch_pedal": PEDAL_TRAVEL, "gear_raw": 0}
  frame.update(kw)
  for _ in range(n):
    g.update(**frame)


class TestClutchGate:
  def test_off_is_noop(self):
    g = _gate(enabled=False)
    _step(g, 5)
    assert g.state == GateState.idle
    assert g.limit_accel(-3.5, 2.0) == (-3.5, 2.0)

  def test_shift_caps_throttle_braking_passes(self):
    g = _gate()
    _step(g, 1, decoupled=True, clutch_pedal=PEDAL_TRAVEL, gear_raw=0)
    assert g.state == GateState.shifting
    lower, upper = g.limit_accel(-3.5, 2.0)
    assert lower == -3.5          # braking always passes
    assert upper == 0.0           # throttle capped at coast_accel

  def test_reengage_starts_the_ramp_after_the_delay(self):
    # owner 2026-11-06 (session 15): the throttle may come back 0.05 s after the gear is back (was 0.15 s)
    g = _gate()
    _step(g, 1, decoupled=True)                                  # shifting
    _step(g, 4, decoupled=False, clutch_pedal=PEDAL_UP, gear_raw=3)    # coupled in 3rd for 0.04 s: still waiting
    assert g.state == GateState.reengaging and g.paused
    assert g.limit_accel(-3.5, 2.0) == (-3.5, 0.0)
    _step(g, 2, decoupled=False, clutch_pedal=PEDAL_UP, gear_raw=3)    # past REENGAGE_DELAY
    assert g.state == GateState.ramping and not g.paused

  def test_throttle_returns_on_a_ramp_not_a_step(self):
    # owner 2026-11-06: "the re-engage ramp rate is a little harsh" - the limit rises at RELEASE_JERK from the cap
    g = _gate()
    _step(g, 1, decoupled=True)
    uppers = []
    for _ in range(200):
      _step(g, 1, decoupled=False, clutch_pedal=PEDAL_UP, gear_raw=3)
      lower, upper = g.limit_accel(-3.5, 2.0)
      assert lower == -3.5                                       # braking always passes
      uppers.append(upper)
    rise = [b - a for a, b in zip(uppers[:-1], uppers[1:], strict=True)]
    assert max(rise) <= RELEASE_JERK * DT + 1e-9                 # never a step
    i06 = next(i for i, u in enumerate(uppers) if u >= 0.6)
    assert 0.36 <= (i06 + 1) * DT <= 0.46                        # +0.6 m/s^2 about 0.4 s after the gear is back
    assert uppers[-1] == 2.0 and g.state == GateState.idle       # and the gate is fully out of the way afterwards
    assert g.limit_accel(-3.5, 2.0) == (-3.5, 2.0)

  def test_clutch_in_during_the_ramp_caps_again(self):
    g = _gate()
    _step(g, 1, decoupled=True)
    _step(g, 20, decoupled=False, clutch_pedal=PEDAL_UP, gear_raw=3)
    assert g.state == GateState.ramping
    _step(g, 1, decoupled=True, clutch_pedal=PEDAL_TRAVEL, gear_raw=0)
    assert g.state == GateState.shifting
    assert g.limit_accel(-3.5, 2.0) == (-3.5, 0.0)

  def test_ramp_never_cuts_a_floor_above_it(self):
    # the non-ACC floor on a descent (positive) must still pass whole while the ramp is below it
    g = _gate()
    _step(g, 1, decoupled=True)
    _step(g, 7, decoupled=False, clutch_pedal=PEDAL_UP, gear_raw=3)
    assert g.state == GateState.ramping
    assert g.limit_accel(0.5, 2.0) == (0.5, 0.5)

  def test_neutral_goes_to_coast_timeout(self):
    g = _gate()
    _step(g, 70, decoupled=True, clutch_pedal=PEDAL_UP, gear_raw=0)   # pedal up + no gear = neutral, > NEUTRAL_CONFIRM
    assert g.state == GateState.coastTimeout
    _, upper = g.limit_accel(-3.5, 2.0)
    assert upper == 0.0           # still capped

  def test_signal_invalid_stays_off(self):
    g = _gate()
    _step(g, 5, clutch_signal_valid=False, decoupled=True)
    assert g.state == GateState.idle
    assert g.limit_accel(-3.5, 2.0) == (-3.5, 2.0)

  def test_automatic_unavailable(self):
    Params().put_bool("ClutchShiftGate", True, block=True)
    CP = CarInterface.get_non_essential_params(CAR.HONDA_ACCORD)   # automatic (if manual, skip)
    if CP.transmissionType == __import__("opendbc.car", fromlist=["structs"]).structs.CarParams.TransmissionType.manual:
      return
    g = ClutchShiftGate(CP, DT)
    _step(g, 5, decoupled=True)
    assert g.state == GateState.idle

  def test_floor_above_the_cap_passes_as_the_whole_range(self):
    # non-ACC floor on a descent with the clutch in = the declutched car's own accel (positive): it must reach the
    # car as it is (owner 2026-11-06: no brake, no throttle, clutch in or out). Anything above it is still cut, and
    # ordinary braking limits are untouched.
    g = _gate()
    _step(g, 1, decoupled=True)
    assert g.limit_accel(0.6, 2.0) == (0.6, 0.6)
    assert g.limit_accel(-0.2, 2.0) == (-0.2, 0.0)
    assert g.limit_accel(-3.5, 2.0) == (-3.5, 0.0)
