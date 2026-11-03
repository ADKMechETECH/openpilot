"""
fork-only (civic-cruise) Phase 4 (minimal gate): the clutch/shift gate caps the throttle while the drivetrain is
decoupled; braking always passes; neutral / long coasts go to coastTimeout. Toggle + signal-liveness gated.
"""
from openpilot.common.params import Params
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.clutch_gate import ClutchShiftGate, GateState

DT = 0.01
PEDAL_UP, PEDAL_TRAVEL = 1, 2


def _gate(enabled=True):
  Params().put_bool("ClutchShiftGate", enabled, block=True)
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)   # manual
  return ClutchShiftGate(CP, DT)


def _step(g, n, **kw):
  frame = dict(clutch_signal_valid=True, decoupled=True, clutch_pedal=PEDAL_TRAVEL, gear_raw=0)
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

  def test_reengage_releases_after_delay(self):
    g = _gate()
    _step(g, 1, decoupled=True)                                  # shifting
    _step(g, 25, decoupled=False, clutch_pedal=PEDAL_UP, gear_raw=3)   # coupled in 3rd past REENGAGE_DELAY
    assert g.state == GateState.idle
    assert g.limit_accel(-3.5, 2.0) == (-3.5, 2.0)              # cap released

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
