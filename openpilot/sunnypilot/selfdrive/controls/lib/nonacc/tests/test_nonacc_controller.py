"""
fork-only (civic-cruise) Phase 2c/2d: the non-ACC overspeed controller.

Unit tests for the step-asides + sawtooth state machine, and a closed-loop sim on a model of the measured car (the car
slows at the stronger of the request and its own engine braking; grade adds g*sin(pitch)):
  - flat overspeed  -> engine-brakes back to set, service brakes never join
  - descent         -> sawtooth: coast (drift up) to the trigger, firm brake to within RELEASE of set, release, repeat
"""
import numpy as np

from openpilot.common.params import Params
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import controller as C
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.engine_brake import coast_accel

DT = 0.01
SET = 20.0
LOWER = -3.5   # a wide incoming pid lower limit


def _ctrl():
  Params().put_bool("NonAdaptiveLongitudinal", True)
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  return C.NonAccController(CP, DT)


def teardown_function(_):
  Params().put_bool("NonAdaptiveLongitudinal", False)


class TestStepAside:
  def test_below_set_flat_never_forces_throttle(self):
    # the controller engages below set too (so it can drift up to the sawtooth on a descent), but on the flat its
    # floor is <= 0 - it only caps braking, never forces throttle, so it can't interfere with accelerating to set.
    c = _ctrl()
    floor = c.update(True, SET - 2.0, SET, 4, 0.0, False, LOWER)
    assert LOWER < floor <= 0.0

  def test_force_decel_steps_aside(self):
    c = _ctrl()
    assert c.update(True, SET + 5.0, SET, 4, 0.0, True, LOWER) == LOWER

  def test_no_pitch_steps_aside(self):
    c = _ctrl()
    assert c.update(True, SET + 5.0, SET, 4, None, False, LOWER) == LOWER

  def test_untrusted_gear_coasting_steps_aside(self):
    c = _ctrl()
    assert c.update(True, SET + 1.0, SET, None, 0.0, False, LOWER) == LOWER

  def test_long_inactive_is_noop(self):
    c = _ctrl()
    assert c.update(False, SET + 5.0, SET, 4, 0.0, False, LOWER) == LOWER

  def test_hard_ceiling_hands_back_full_braking(self):
    c = _ctrl()
    assert c.update(True, SET + C.HARD_CEILING + 1.0, SET, 4, 0.0, False, LOWER) == LOWER


class TestFloorAndSawtooth:
  def test_flat_coast_floor_is_engine_brake(self):
    # just over set on the flat: raise the lower limit to the brake-free floor (negative, engine braking)
    c = _ctrl()
    floor = c.update(True, SET + 1.0, SET, 4, 0.0, False, LOWER)
    assert LOWER < floor < 0.0
    assert c.state == C.COASTING

  def test_sawtooth_transitions(self):
    c = _ctrl()
    # drive overspeed above the trigger -> PULSING, floor ramps toward the firm decel
    for _ in range(400):
      floor = c.update(True, SET + C.DEFAULT_TRIGGER + 1.0, SET, 4, -0.05, False, LOWER)
    assert c.state == C.PULSING
    assert floor < -0.5   # ramped toward PULSE_DECEL
    # drop overspeed below the release -> back to COASTING
    for _ in range(400):
      c.update(True, SET + C.RELEASE - 0.5, SET, 4, -0.05, False, LOWER)
    assert c.state == C.COASTING


def _sim(pitch, v0, gear, secs):
  # measured car: LongControl clips the set-speed plan to [floor, upper]; the car tracks the commanded accel (engine
  # braking and grade are implicit in what the controller chooses to command via the closed-throttle coast accel).
  c = _ctrl()
  v = v0
  cmds, speeds = [], []
  for _ in range(int(secs / DT)):
    floor = c.update(True, v, SET, gear, pitch, False, LOWER)
    a_target = float(np.clip(SET - v, -1.2, 1.5))   # the set-speed (cruise) candidate
    a_cmd = float(np.clip(a_target, floor, 1.5))     # clipped to the controller's raised lower limit
    v = max(0.0, v + a_cmd * DT)
    cmds.append(a_cmd); speeds.append(v)
  return np.array(cmds), np.array(speeds)


class TestClosedLoop:
  def test_flat_overspeed_engine_brakes_no_service_brake(self):
    # start 2 m/s over set on the flat -> engine-brakes back to set and holds, service brakes never join
    cmds, speeds = _sim(pitch=0.0, v0=SET + 2.0, gear=4, secs=12.0)
    assert abs(speeds[-100:].mean() - SET) < 0.3    # settled at set
    assert speeds.min() >= SET - 0.6                # no undershoot below set
    # the service brakes join only if a command is stronger than the car's own closed-throttle decel. The deepest coast
    # over the run is at the start (highest speed = most engine braking); the command never goes below it.
    ca = coast_accel(4, SET + 2.0, 0.0)
    assert (cmds >= ca - 0.02).all(), "a command below the coast decel would use the service brakes"

  def test_descent_sawtooths(self):
    # a descent steeper than engine braking can hold -> drift up to the trigger, firm brake to the release, repeat
    cmds, speeds = _sim(pitch=-0.08, v0=SET, gear=4, secs=60.0)
    over = speeds - SET
    assert over.max() < C.DEFAULT_TRIGGER + 0.8, "overspeed should be bounded near the trigger"
    assert (cmds < -0.9).any(), "should pulse the brakes firmly at least once"
    assert (cmds > 0.05).any(), "should coast (drift up) between pulses, not ride the brakes"
    brake_duty = float((cmds < -0.5).mean())
    assert 0.05 < brake_duty < 0.9, f"expected intermittent braking (sawtooth), got duty {brake_duty:.2f}"
