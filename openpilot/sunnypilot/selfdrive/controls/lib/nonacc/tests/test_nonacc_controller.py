"""
fork-only (civic-cruise) Phase 2c/2d: the non-ACC overspeed controller.

Unit tests for the step-asides + sawtooth state machine, and a closed-loop sim on a model of the measured car (the car
slows at the stronger of the request and its own engine braking; grade adds g*sin(pitch)):
  - flat overspeed  -> engine-brakes back to set, service brakes never join
  - descent         -> sawtooth: coast (drift up) to the trigger, firm brake to within RELEASE of set, release, repeat
"""
import numpy as np

from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import controller as C
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import engine_brake as E
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.engine_brake import coast_accel

DT = 0.01
SET = 20.0
LOWER = -3.5   # a wide incoming pid lower limit


def _ctrl():
  # manual + openpilot longitudinal: the only configuration the mode exists in. The last update() argument is the
  # car's own conventional-cruise mode (carState.cruiseState.nonAdaptive) - True in these tests unless stated.
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP.openpilotLongitudinalControl = True
  return C.NonAccController(CP, DT)


class TestStepAside:
  def test_below_set_flat_never_forces_throttle(self):
    # the controller engages below set too (so it can drift up to the sawtooth on a descent), but on the flat its
    # floor is <= 0 - it only caps braking, never forces throttle, so it can't interfere with accelerating to set.
    c = _ctrl()
    floor = c.update(True, SET - 2.0, SET, 4, 0.0, False, LOWER, True)
    assert LOWER < floor <= 0.0

  def test_force_decel_steps_aside(self):
    c = _ctrl()
    assert c.update(True, SET + 5.0, SET, 4, 0.0, True, LOWER, True) == LOWER

  def test_no_pitch_steps_aside(self):
    c = _ctrl()
    assert c.update(True, SET + 5.0, SET, 4, None, False, LOWER, True) == LOWER

  def test_untrusted_gear_coasting_steps_aside(self):
    c = _ctrl()
    assert c.update(True, SET + 1.0, SET, None, 0.0, False, LOWER, True) == LOWER

  def test_car_in_acc_mode_is_noop(self):
    # the car's own ACC <-> Cruise mode is the switch: in ACC mode the controller never touches the limit
    c = _ctrl()
    assert c.update(True, SET + 5.0, SET, 4, 0.0, False, LOWER, False) == LOWER
    assert not c.mode.active
    assert c.update(True, SET + 1.0, SET, 4, 0.0, False, LOWER, True) > LOWER    # flips live, no latch
    assert c.mode.active
    assert c.update(True, SET + 5.0, SET, 4, 0.0, False, LOWER, False) == LOWER
    assert not c.mode.active

  def test_never_active_without_manual_and_op_long(self):
    for manual, op_long in ((True, False), (False, True)):
      CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
      CP.transmissionType = "manual" if manual else "automatic"
      CP.openpilotLongitudinalControl = op_long
      c = C.NonAccController(CP, DT)
      assert c.update(True, SET + 5.0, SET, 4, 0.0, False, LOWER, True) == LOWER
      assert not c.mode.active

  def test_long_inactive_is_noop(self):
    c = _ctrl()
    assert c.update(False, SET + 5.0, SET, 4, 0.0, False, LOWER, True) == LOWER

  def test_hard_ceiling_hands_back_full_braking(self):
    c = _ctrl()
    assert c.update(True, SET + C.HARD_CEILING + 1.0, SET, 4, 0.0, False, LOWER, True) == LOWER


class TestFloorAndSawtooth:
  def test_flat_coast_floor_is_engine_brake(self):
    # just over set on the flat: raise the lower limit to the brake-free floor (negative, engine braking)
    c = _ctrl()
    floor = c.update(True, SET + 1.0, SET, 4, 0.0, False, LOWER, True)
    assert LOWER < floor < 0.0
    assert c.state == C.COASTING

  def test_sawtooth_transitions(self):
    c = _ctrl()
    # drive overspeed above the trigger -> PULSING, floor ramps toward the firm decel
    for _ in range(400):
      floor = c.update(True, SET + C.DEFAULT_TRIGGER + 1.0, SET, 4, -0.05, False, LOWER, True)
    assert c.state == C.PULSING
    assert floor < -0.5   # ramped toward PULSE_DECEL
    # drop overspeed below the release -> back to COASTING
    for _ in range(400):
      c.update(True, SET + C.RELEASE - 0.5, SET, 4, -0.05, False, LOWER, True)
    assert c.state == C.COASTING


def _sim(pitch, v0, gear, secs):
  # measured car: LongControl clips the set-speed plan to [floor, upper]; the car tracks the commanded accel (engine
  # braking and grade are implicit in what the controller chooses to command via the closed-throttle coast accel).
  c = _ctrl()
  v = v0
  cmds, speeds = [], []
  for _ in range(int(secs / DT)):
    floor = c.update(True, v, SET, gear, pitch, False, LOWER, True)
    a_target = float(np.clip(SET - v, -1.2, 1.5))   # the set-speed (cruise) candidate
    a_cmd = float(np.clip(a_target, floor, 1.5))     # clipped to the controller's raised lower limit
    v = max(0.0, v + a_cmd * DT)
    cmds.append(a_cmd)
    speeds.append(v)
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


class TestFloorContinuity:
  """Session 14 (routes 00000056/57): the floor dropped out on every clutch-in and gas override and took ~2 s to ramp
  back from the raw lower limit; the planner braked at up to -1.2 m/s^2 meanwhile."""

  def _coasting(self, c, over=1.5, pitch=-0.04, gear=6, frames=300):
    floor = None
    for _ in range(frames):
      floor = c.update(True, SET + over, SET, gear, pitch, False, LOWER, True, decoupled=False, last_accel=0.0)
    return floor

  def test_clutch_in_keeps_the_floor(self):
    # 6th, a 4 % descent, 3.4 mph over set (800.65 s on route 00000056): clutch in must not hand the brakes to the plan
    c = _ctrl()
    before = self._coasting(c)
    floors = [c.update(True, SET + 1.5, SET, None, -0.04, False, LOWER, True, decoupled=True, last_accel=before)
              for _ in range(200)]
    assert min(floors) > -0.3, "the floor dropped on clutch-in"
    assert abs(floors[0] - before) <= C.PULSE_JERK * DT + 1e-9, "the floor stepped on clutch-in"
    assert abs(floors[-1] - E.declutched_coast_accel(SET + 1.5, -0.04)) < 1e-6   # the rolling car's own accel (> 0 here)
    assert floors[-1] > 0.0
    # and back in gear: still no step, still no drop
    back = [c.update(True, SET + 1.5, SET, 4, -0.04, False, LOWER, True, decoupled=False, last_accel=floors[-1])
            for _ in range(200)]
    assert min(back) > -0.3
    assert max(abs(b - a) for a, b in zip([floors[-1]] + back, back, strict=False)) <= C.PULSE_JERK * DT + 1e-9

  def test_declutched_floor_on_the_flat_is_road_load_only(self):
    c = _ctrl()
    for _ in range(300):
      floor = c.update(True, SET + 1.0, SET, None, 0.0, False, LOWER, True, decoupled=True, last_accel=0.0)
    # road load at 21 m/s is -0.26: with COAST_MARGIN the request sits at 0 (never above), i.e. exactly what every
    # shift below set speed already sends with the clutch in
    assert -0.01 < floor <= 0.0
    assert floor > coast_accel(4, SET + 1.0, 0.0)

  def test_dead_clutch_signal_still_steps_aside(self):
    # gear untrusted and NOT a live clutch-in (signal dead, reverse): inert, as every feature is without the signal
    c = _ctrl()
    self._coasting(c)
    assert c.update(True, SET + 1.5, SET, None, -0.04, False, LOWER, True, decoupled=False, last_accel=0.0) == LOWER

  def test_force_decel_wins_with_the_clutch_in(self):
    c = _ctrl()
    self._coasting(c)
    assert c.update(True, SET + 1.5, SET, None, -0.04, True, LOWER, True, decoupled=True, last_accel=0.0) == LOWER

  def test_pulse_still_fires_with_the_clutch_in(self):
    c = _ctrl()
    for _ in range(300):
      floor = c.update(True, SET + C.DEFAULT_TRIGGER + 0.5, SET, None, -0.08, False, LOWER, True, decoupled=True, last_accel=0.0)
    assert c.state == C.PULSING and floor < -0.9

  def test_gas_release_above_set_has_the_floor_at_once(self):
    # gas override (longitudinal inactive, last command 0), released 6.8 mph over set in 5th on the flat (1038.6 s)
    c = _ctrl()
    for _ in range(300):
      assert c.update(False, SET + 3.0, SET, 5, 0.0, False, LOWER, True, decoupled=False, last_accel=0.0) == LOWER
    floor = c.update(True, SET + 3.0, SET, 5, 0.0, False, LOWER, True, decoupled=False, last_accel=0.0)
    assert floor >= coast_accel(5, SET + 3.0, 0.0) - 0.02, "no floor on the first frame after the gas release"
    assert c.state == C.COASTING

  def test_gas_release_above_the_trigger_pulses(self):
    # owner: no brakes on the way down unless over the sawtooth trigger - then the sawtooth as normal
    c = _ctrl()
    for _ in range(50):
      c.update(False, SET + C.DEFAULT_TRIGGER + 1.0, SET, 5, 0.0, False, LOWER, True, decoupled=False, last_accel=0.0)
    floors, last = [], 0.0
    for _ in range(120):
      last = c.update(True, SET + C.DEFAULT_TRIGGER + 1.0, SET, 5, 0.0, False, LOWER, True, decoupled=False, last_accel=last)
      floors.append(last)
    assert c.state == C.PULSING and floors[-1] <= C.PULSE_DECEL + 1e-6
    assert floors[int(C.PULSE_CONFIRM / DT) - 2] > -0.05, "the pulse must wait PULSE_CONFIRM"
    assert min(b - a for a, b in zip(floors, floors[1:], strict=False)) >= -C.PULSE_JERK * DT - 1e-9   # and comes on as a ramp

  def test_acc_to_nonacc_while_braking_is_released_on_the_ramp(self):
    # the car's mode flips live while ACC brakes at -2.0 for a lead: the brake must come off at PULSE_JERK, not at once
    c = _ctrl()
    c.update(True, SET - 1.0, SET, 6, 0.0, False, LOWER, False, decoupled=False, last_accel=-2.0)
    floors = []
    last = -2.0
    for _ in range(100):
      last = c.update(True, SET - 1.0, SET, 6, 0.0, False, LOWER, True, decoupled=False, last_accel=last)
      floors.append(last)
    assert floors[0] <= -2.0 + C.PULSE_JERK * DT + 1e-9
    assert max(b - a for a, b in zip(floors, floors[1:], strict=False)) <= C.PULSE_JERK * DT + 1e-9

  def test_unknown_last_accel_ramps_from_the_lower_limit(self):
    # callers that do not pass last_accel keep the old behaviour
    c = _ctrl()
    c.update(False, SET + 1.0, SET, 4, 0.0, False, LOWER, True)
    assert c.update(True, SET + 1.0, SET, 4, 0.0, False, LOWER, True) <= LOWER + C.PULSE_JERK * DT + 1e-9

  def test_shift_on_a_descent_closed_loop(self):
    # 6th -> clutch in 2 s -> 4th on a 4 % descent, 1.5 m/s over set, the plan wanting -1.2 throughout: the command
    # never goes below the car's own accel by more than a whisker, and never steps
    c = _ctrl()
    cmd, cmds, own = 0.0, [], []
    seq = [(6, False)] * 300 + [(None, True)] * 200 + [(4, False)] * 300
    for gear, dec in seq:
      floor = c.update(True, SET + 1.5, SET, gear, -0.04, False, LOWER, True, decoupled=dec, last_accel=cmd)
      upper = max(0.0, floor) if dec else 1.5       # the gate: throttle cap 0, the floor passes if above it
      cmd = float(np.clip(-1.2, floor, upper))
      cmds.append(cmd)
      own.append(E.declutched_coast_accel(SET + 1.5, -0.04) if dec else coast_accel(gear, SET + 1.5, -0.04))
    cmds, own = np.array(cmds), np.array(own)
    assert np.abs(np.diff(cmds)).max() <= 0.05, "the command stepped at the clutch"
    assert (cmds[100:] >= np.minimum(own[100:], 0.0) - 0.02).all(), "a request below the car's own accel = brakes"
    assert cmds.min() > -0.2, f"brake request during the shift: {cmds.min():+.2f}"


class TestSteepDescent:
  """Session 14: with the coast request capped at +0.3 the VSA braked between the sawtooth pulses on a 12-17 % descent
  in 2nd. Owner 2026-11-06: send the car's real coast accel."""

  def test_floor_is_the_real_coast_accel_on_a_steep_descent(self):
    c = _ctrl()
    pitch = -0.149            # ~15 %
    for _ in range(300):
      floor = c.update(True, 9.0, 6.7, 2, pitch, False, LOWER, True, decoupled=False, last_accel=0.0)   # 20 mph, set 15
    own = coast_accel(2, 9.0, pitch)
    assert own > 0.6
    assert abs(floor - (own + C.DESCENT_MARGIN)) < 1e-6, "the request must be the car's own coast accel plus a whisper (no brake drag)"

  def test_cap_still_bounds_a_bad_estimate(self):
    c = _ctrl()
    for _ in range(300):
      floor = c.update(True, 20.0, 19.0, 4, -0.5, False, LOWER, True, decoupled=False, last_accel=0.0)   # a wild pitch
    assert floor == C.COAST_CAP
    assert 0.9 <= C.COAST_CAP <= 1.2

  def test_steep_descent_sawtooths_without_riding_the_brakes(self):
    # closed loop on a 15 % descent in 2nd: between pulses the request is the coast accel itself
    c = _ctrl()
    v, v_set, pitch = 6.7, 6.7, -0.149
    cmds, own, pulsing = [], [], []
    cmd = 0.0
    for _ in range(int(60 / DT)):
      floor = c.update(True, v, v_set, 2, pitch, False, LOWER, True, decoupled=False, last_accel=cmd)
      cmd = float(np.clip(np.clip(v_set - v, -1.2, 1.5), floor, 1.5))
      o = coast_accel(2, v, pitch)
      v = max(0.0, v + min(cmd, o) * DT)            # the car cannot out-accelerate its own coast without throttle
      cmds.append(cmd)
      own.append(o)
      pulsing.append(c.state == C.PULSING)
    cmds, own, pulsing = np.array(cmds), np.array(own), np.array(pulsing)
    assert pulsing.any() and (~pulsing).any()
    settled = ~pulsing & (np.arange(len(cmds)) > 300)
    ramp = np.zeros(len(cmds), bool)                 # the release ramp after each pulse is not coasting yet
    ends = np.flatnonzero(pulsing[:-1] & ~pulsing[1:])
    for e in ends:
      ramp[e:e + int(2.0 / DT)] = True
    coasting = settled & ~ramp
    assert coasting.sum() > 500
    assert (cmds[coasting] >= own[coasting] - 0.02).all(), "request below the car's own coast between pulses = brakes"

  def test_clutch_in_on_a_steep_descent_rolls_free(self):
    # 2nd on a 15 % descent, clutch in: the request is the rolling car's own accel (about +1.0), not 0 - a request of 0
    # there had the VSA holding the car back at ~1 m/s^2. Bounded by COAST_CAP.
    c = _ctrl()
    pitch = -0.149
    for _ in range(300):
      c.update(True, 9.0, 6.7, 2, pitch, False, LOWER, True, decoupled=False, last_accel=0.0)
    for _ in range(300):
      floor = c.update(True, 9.0, 6.7, None, pitch, False, LOWER, True, decoupled=True, last_accel=0.0)
    own = E.declutched_coast_accel(9.0, pitch)
    assert own > 1.0
    assert floor == min(own, C.COAST_CAP)


class TestNoBrakeLightsOnTaps:
  """Session 15 (route 0000005a + the two session 14 drives): the car's own COMPUTER_BRAKING flag (and the cluster's
  brake lights, owner) came on at set-speed taps in non-ACC although the request was at the estimated coast. Measured:
  the car arms its brakes inside ~0.25 m/s^2 of its own coast, and far more readily when the request falls fast."""

  def _settle(self, c, v, gear, pitch, frames=400, **kw):
    last = getattr(c, "_test_last", 0.0)                 # the request carries over between calls, as on the car
    for _ in range(frames):
      last = c.update(True, v, SET, gear, pitch, False, LOWER, True, decoupled=False, last_accel=last, **kw)
    c._test_last = last
    return last

  def test_floor_sits_a_margin_above_the_coast(self):
    c = _ctrl()
    floor = self._settle(c, SET + 1.0, 4, 0.0)
    ca = coast_accel(4, SET + 1.0, 0.0)
    assert abs(floor - C.coast_floor(ca)) < 1e-6
    assert 0.2 < floor - ca < 0.35                       # the measured band on the flat in the upper gears
    assert floor < -0.1                                  # still a decel request

  def test_margin_shape(self):
    # more margin the harder the car coasts; a whisper at most above max(coast, 0)
    assert abs(C.coast_margin(-0.35) - 0.24) < 1e-9 and abs(C.coast_margin(-0.55) - 0.32) < 1e-9
    assert C.coast_margin(-2.0) == C.MARGIN_MAX and C.coast_margin(1.0) == C.MARGIN_MIN
    xs = np.linspace(-1.5, 1.0, 251)
    fl = np.array([C.coast_floor(float(x)) for x in xs])
    assert (np.diff(fl) >= -1e-12).all()                              # monotonic, no jump anywhere
    assert (fl >= xs + C.MARGIN_MIN - 1e-9)[xs < C.COAST_CAP - 0.1].all()   # always above the coast
    assert (fl <= np.maximum(xs, 0.) + C.DESCENT_MARGIN + 1e-9).all()  # never powered beyond a whisper

  def test_margin_never_asks_for_power_above_set(self):
    # a gentle descent: the coast is -0.1; the floor stops at +DESCENT_MARGIN
    c = _ctrl()
    pitch = float(np.arcsin((coast_accel(4, SET + 1.0, 0.0) + 0.1) / E.G))
    assert abs(coast_accel(4, SET + 1.0, pitch) + 0.1) < 1e-6
    assert abs(self._settle(c, SET + 1.0, 4, pitch) - min(-0.1 + C.coast_margin(-0.1), C.DESCENT_MARGIN)) < 1e-6

  def test_steep_descent_floor_is_the_coast_plus_a_whisper(self):
    c = _ctrl()
    ca = coast_accel(2, 9.0, -0.14)
    assert ca > 0.3
    assert abs(self._settle(c, 9.0, 2, -0.14) - (ca + C.DESCENT_MARGIN)) < 1e-6

  def test_same_rule_on_a_climb(self):
    # owner: "it should behave the same way as flat ground or on a decline" - no special case uphill
    c = _ctrl()
    floor = self._settle(c, SET + 1.0, 5, 0.08)
    assert abs(floor - C.coast_floor(coast_accel(5, SET + 1.0, 0.08))) < 1e-6

  def test_request_eases_below_zero(self):
    # a tap: the plan wants -1.2 at once. From a +0.5 request the throttle may come off in one frame, but below 0 the
    # request falls no faster than COAST_FALL_RATE
    c = _ctrl()
    out, last = [], 0.5
    for _ in range(300):
      floor = c.update(True, SET + 1.0, SET, 4, 0.0, False, LOWER, True, decoupled=False, last_accel=last)
      last = max(-1.2, floor)                            # LongControl: the plan's request, clipped at the lower limit
      out.append(last)
    assert -C.COAST_FALL_RATE * DT - 1e-9 <= out[0] <= 0.0
    assert min(b - a for a, b in zip(out, out[1:], strict=False)) >= -C.COAST_FALL_RATE * DT - 1e-9
    assert abs(out[-1] - C.coast_floor(coast_accel(4, SET + 1.0, 0.0))) < 1e-6

  def test_computer_braking_flag_raises_the_floor(self):
    c = _ctrl()
    base = self._settle(c, SET + 1.0, 4, 0.0)
    up = self._settle(c, SET + 1.0, 4, 0.0, frames=10, computer_braking=True)
    assert abs((up - base) - C.TRIM_RISE * 10 * DT) < 1e-6          # rises at TRIM_RISE while the car says it brakes
    top = self._settle(c, SET + 1.0, 4, 0.0, frames=200, computer_braking=True)
    assert abs(top - (base + C.TRIM_MAX)) < 1e-6 and c.trim == C.TRIM_MAX   # bounded
    after = self._settle(c, SET + 1.0, 4, 0.0, frames=100)
    assert abs(c.trim - (C.TRIM_MAX - C.TRIM_DECAY * 1.0)) < 1e-6   # and fades slowly once the flag is off
    assert after <= top + 1e-9

  def test_flag_during_and_just_after_a_pulse_does_not_trim(self):
    c = _ctrl()
    self._settle(c, SET + C.DEFAULT_TRIGGER + 0.5, 4, -0.08, frames=100, computer_braking=True)
    assert c.state == C.PULSING and c.trim == 0.0
    self._settle(c, SET + 1.0, 4, -0.08, frames=int(C.TRIM_HOLDOFF / DT) - 5, computer_braking=True)
    assert c.state == C.COASTING and c.trim == 0.0                  # the car is still releasing real braking
    self._settle(c, SET + 1.0, 4, -0.08, frames=20, computer_braking=True)
    assert c.trim > 0.0

  def test_one_stale_frame_at_set_does_not_pulse(self):
    # 648.6 s on route 0000005a: SET pressed at 52 mph with the previous set speed (42) still in the first frame
    c = _ctrl()
    c.update(False, SET, SET - 4.7, 5, -0.064, False, LOWER, True, decoupled=False, last_accel=0.0)
    first = c.update(True, SET, SET - 4.7, 5, -0.064, False, LOWER, True, decoupled=False, last_accel=0.0)
    assert c.state == C.COASTING and first > -0.05
    floors = [c.update(True, SET, SET, 5, -0.064, False, LOWER, True, decoupled=False, last_accel=0.0) for _ in range(20)]
    assert c.state == C.COASTING and min(floors) > -0.05, "the floor was seeded from a one-frame pulse"


class TestAccModeCoast:
  """Owner 2026-11-06: "adaptive cruise should have the same coast to set speed and brake only if necessary", always
  active. In the car's ACC mode the floor applies only up to what the lead / e2e plan asks for."""

  def test_no_lead_plan_known_means_no_floor(self):
    c = _ctrl()
    assert c.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, False, last_accel=0.0) == LOWER
    assert c.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, False, last_accel=0.0, lead_accel_limit=float("nan")) == LOWER

  def test_free_road_coasts_like_non_acc(self):
    acc, na = _ctrl(), _ctrl()
    a = n = 0.0
    for _ in range(400):
      a = acc.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, False, last_accel=a, lead_accel_limit=1.5)
      n = na.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, True, last_accel=n)
    assert abs(a - n) < 1e-9 and abs(a - C.coast_floor(coast_accel(4, SET + 2.0, 0.0))) < 1e-6
    assert not acc.mode.active                            # the HUD's NON-ACC label stays the car's own mode

  def test_lead_braking_passes_in_full_the_same_frame(self):
    c = _ctrl()
    last = 0.0
    for _ in range(300):
      last = c.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, False, last_accel=last, lead_accel_limit=1.5)
    assert last > -0.3
    assert c.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, False, last_accel=last, lead_accel_limit=-2.0) == -2.0
    assert c.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, False, last_accel=-2.0, lead_accel_limit=-3.6) == LOWER

  def test_mild_lead_decel_is_the_ceiling_of_the_floor(self):
    # the lead plan wants -0.3, the set speed wants -1.2: the request may not be gentler than -0.3
    c = _ctrl()
    last = 0.0
    for _ in range(300):
      last = max(-1.2, c.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, False, last_accel=last, lead_accel_limit=-0.3))
    assert abs(last + 0.3) < 1e-6

  def test_sawtooth_and_force_decel_as_in_non_acc(self):
    c = _ctrl()
    last = 0.0
    for _ in range(200):
      last = c.update(True, SET + C.DEFAULT_TRIGGER + 0.5, SET, 4, -0.08, False, LOWER, False, last_accel=last, lead_accel_limit=1.5)
    assert c.state == C.PULSING and last <= C.PULSE_DECEL + 1e-6
    assert c.update(True, SET + 2.0, SET, 4, 0.0, True, LOWER, False, last_accel=last, lead_accel_limit=1.5) == LOWER

  def test_automatic_or_stock_long_never_gets_it(self):
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP.openpilotLongitudinalControl = False
    c = C.NonAccController(CP, DT)
    assert c.update(True, SET + 2.0, SET, 4, 0.0, False, LOWER, False, last_accel=0.0, lead_accel_limit=1.5) == LOWER
