from opendbc.car.structs import car

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import ClutchShiftGate, GateState, FLARE_CLEAR_TIME, \
                                                                         GAS_RELEASE_WINDOW, REENGAGE_DELAY

TransmissionType = car.CarParams.TransmissionType
CRUISE_RPM = 2200.
V_CRUISE = 25.


def make_cp(transmission=TransmissionType.manual):
  CP = car.CarParams.new_message()
  CP.transmissionType = transmission
  return CP


def cs(clutch=False, gas=False, rpm=CRUISE_RPM, v=V_CRUISE, a=0., brake=False):
  CS = car.CarState.new_message()
  CS.deprecated.clutchPressed = clutch
  CS.deprecated.engineRpm = rpm
  CS.gasPressed = gas
  CS.brakePressed = brake
  CS.vEgo = v
  CS.aEgo = a
  return CS


def upd(gate, CS, enabled=True, gear_raw=3, clutch_valid=True) -> bool:
  """One controls frame of the gate (engaged, live clutch signal); returns whether it is capping throttle for a shift
  (Task 12 A2: a shift no longer pauses longitudinal - `paused` now means "throttle capped, braking passes")."""
  gate.update(CS, enabled, gear_raw, clutch_valid)
  return gate.paused


class TestClutchShiftGate(OpenpilotTestCase):
  def setup_method(self):
    Params().put_bool("ClutchShiftGate", True)
    self.gate = ClutchShiftGate(make_cp())

  def run_for(self, seconds, **kw):
    return [upd(self.gate, cs(**kw)) for _ in range(round(seconds / DT_CTRL))]

  def test_default_off(self):
    Params().remove("ClutchShiftGate")
    gate = ClutchShiftGate(make_cp())
    self.assertFalse(gate.enabled)
    self.assertFalse(any(upd(gate, cs(clutch=True)) for _ in range(100)))

  def test_unavailable_on_automatics(self):
    for transmission in (TransmissionType.automatic, TransmissionType.cvt):
      gate = ClutchShiftGate(make_cp(transmission))
      self.assertFalse(gate.available)
      self.assertFalse(any(upd(gate, cs(clutch=True)) for _ in range(100)))

  def test_full_shift_sequence(self):
    # cruising -> clutch in -> shift (rpm falls) -> clutch out in the next gear -> resume
    self.assertFalse(any(self.run_for(2.0)))                       # cruising: longitudinal allowed
    self.assertTrue(all(self.run_for(0.8, clutch=True, rpm=1500.)))  # clutch in: paused for the whole shift
    self.assertEqual(self.gate.state, GateState.shifting)

    held = self.run_for(REENGAGE_DELAY - 0.05, rpm=1800.)         # clutch out: keep coasting while it bites
    self.assertTrue(all(held))
    self.assertEqual(self.gate.state, GateState.reengaging)
    self.assert_resumes_within(0.05, rpm=1800.)                    # then resume on time

  def assert_resumes_within(self, remaining, **kw):
    after = self.run_for(1.0, **kw)
    self.assertLessEqual(sum(after), round(remaining / DT_CTRL) + 1)
    self.assertFalse(any(after[round(remaining / DT_CTRL) + 1:]))
    self.assertEqual(self.gate.state, GateState.idle)

  def test_downshift_rpm_jump_does_not_extend_shift(self):
    # 4th -> 3rd at 25 m/s: RPM jumps up as the clutch bites, with mild engine braking (aEgo < 0)
    self.run_for(1.0)
    self.run_for(0.6, clutch=True, rpm=1000.)
    rpms = [1000. + 150. * i for i in range(15)]  # +15000 rpm/s while re-coupling
    held = [upd(self.gate, cs(rpm=r, a=-0.4)) for r in rpms]
    self.assertTrue(all(held))
    self.assertEqual(self.gate.state, GateState.reengaging)
    self.assert_resumes_within(REENGAGE_DELAY - len(rpms) * DT_CTRL, rpm=rpms[-1], a=-0.2)

  def test_reclutch_during_reengage(self):
    self.run_for(0.5, clutch=True)
    self.run_for(REENGAGE_DELAY / 2)
    self.assertTrue(all(self.run_for(0.3, clutch=True)))
    self.assertEqual(self.gate.state, GateState.shifting)

  def enable_gas_release_hint(self):
    Params().put_bool("ClutchShiftGateGasReleaseHint", True)
    self.gate.get_params()
    upd(self.gate, cs(), enabled=False)  # Task 12 B3: params latch only while openpilot is disengaged

  def test_gas_release_hint_default_off(self):
    # clutch is authoritative: by default a plain gas release (no clutch) never pauses longitudinal
    self.assertFalse(self.gate.gas_release_hint)
    self.run_for(1.0, gas=True)
    self.assertFalse(any(self.run_for(1.0)))
    self.assertEqual(self.gate.state, GateState.idle)

  def test_rev_matched_downshift(self):
    # 4th -> 3rd with a throttle blip while the clutch is open: gas pressed + RPM flare must not end the coast,
    # must not be read as an override, and longitudinal resumes cleanly after the clutch re-engages
    self.run_for(1.0)
    self.assertTrue(all(self.run_for(0.2, clutch=True, rpm=1600.)))                      # clutch in, lift
    blip = [upd(self.gate, cs(clutch=True, gas=True, rpm=1600. + 120. * i)) for i in range(15)]  # +12000 rpm/s
    self.assertTrue(all(blip))
    self.assertEqual(self.gate.state, GateState.shifting)
    self.assertTrue(all(self.run_for(0.2, clutch=True, rpm=3400.)))                      # off the gas, clutch still in
    self.assertEqual(self.gate.state, GateState.shifting)
    self.assertTrue(all(self.run_for(REENGAGE_DELAY - 0.05, rpm=3300., a=-0.3)))         # clutch out: re-engage hold
    self.assert_resumes_within(0.05, rpm=3300., a=-0.3)

  def test_rev_match_blip_never_uses_gas_release_hint(self):
    # even with the hint on, a gas release while the clutch is open is part of the shift, not a new cue
    self.enable_gas_release_hint()
    self.run_for(1.0)
    self.run_for(0.2, clutch=True, rpm=1600.)
    self.run_for(0.15, clutch=True, gas=True, rpm=3000.)
    self.run_for(0.1, clutch=True, rpm=3000.)
    self.assertEqual(self.gate.state, GateState.shifting)
    self.run_for(REENGAGE_DELAY + 0.1, rpm=3000.)
    self.assertEqual(self.gate.state, GateState.idle)

  def test_heel_toe_downshift(self):
    # braking with the heel while blipping with the toe, clutch open: the gate stays coasting on clutch state
    # alone (brake handling itself is stock - openpilot's normal brake disengage still applies upstream)
    self.run_for(1.0)
    held = self.run_for(0.2, clutch=True, brake=True, rpm=1800., a=-2.0)
    held += [upd(self.gate, cs(clutch=True, brake=True, gas=True, rpm=1800. + 120. * i, a=-2.0)) for i in range(15)]
    held += self.run_for(0.2, clutch=True, brake=True, rpm=3500., a=-2.0)
    self.assertTrue(all(held))
    self.assertEqual(self.gate.state, GateState.shifting)
    self.assertTrue(all(self.run_for(REENGAGE_DELAY - 0.05, brake=True, rpm=3400., a=-1.5)))
    self.assert_resumes_within(0.05, rpm=3400., a=-1.0)

  def test_clutch_out_with_gas_hands_over_to_override(self):
    # once the clutch re-engages with the driver still on the gas, it is a real override again (gate idle)
    self.run_for(1.0)
    self.run_for(0.4, clutch=True, gas=True, rpm=3000.)
    self.assertFalse(any(self.run_for(0.3, gas=True, rpm=3000., a=0.8)))
    self.assertEqual(self.gate.state, GateState.idle)

  def test_clutch_cancels_running_backstop_hold(self):
    # backstop fired (clutch signal late), then the clutch signal shows up: the shift is now timed by the clutch
    # (re-engage delay after clutch-out), not by the longer FLARE_CLEAR_TIME
    self.run_for(1.0)
    # (Task 12: the backstop needs FLARE_SAMPLES new samples over a FLARE_WINDOW-long rate, so a longer ramp)
    [upd(self.gate, cs(rpm=CRUISE_RPM + 40. * i)) for i in range(25)]
    self.assertGreater(self.gate.flare_clear_timer, 0.)
    self.run_for(0.2, clutch=True, rpm=2600.)
    self.assertEqual(self.gate.flare_clear_timer, 0.)
    self.assertTrue(all(self.run_for(REENGAGE_DELAY - 0.05, rpm=2600.)))
    self.assert_resumes_within(0.05, rpm=2600.)

  def test_resume_reseeds_from_a_ego(self):
    # the gate pauses via longActive=False -> LongControl off -> the planner resets its output to the measured aEgo,
    # so resuming after a shift starts from the car's actual decel instead of the pre-shift command
    from openpilot.cereal import messaging
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_non_adaptive_longitudinal import build_planner
    from openpilot.sunnypilot.selfdrive.controls.lib.dec.tests.test_dec_planner_gate import build_sm
    planner = build_planner()
    sm = build_sm(experimental_mode=False)
    planner.update(sm)  # engaged, cruising
    for long_state, a_ego in ((LongCtrlState.off, -0.6), (LongCtrlState.pid, -0.6)):
      car_state = messaging.new_message('carState')
      car_state.carState.vEgo, car_state.carState.aEgo = 20.0, a_ego
      car_state.carState.vCruise = car_state.carState.vCruiseCluster = 100.0
      controls = messaging.new_message('controlsState')
      controls.controlsState.longControlState = long_state
      sm['carState'], sm['controlsState'] = car_state.carState.as_reader(), controls.controlsState.as_reader()
      planner.update(sm)
    # first frame back: starts from -0.6 and is jerk-limited toward the set-speed target, not a step
    self.assertAlmostEqual(planner.output_a_target, -0.6, delta=0.15)

  def test_gas_release_then_clutch(self):
    # driver overriding on gas lifts, then clutches: no gap where longitudinal could resume between the two
    self.enable_gas_release_hint()
    self.run_for(1.0, gas=True)
    paused = self.run_for(0.2) + self.run_for(0.5, clutch=True)
    self.assertTrue(all(paused))
    self.assertEqual(self.gate.state, GateState.shifting)

  def test_gas_release_without_clutch_resumes(self):
    self.enable_gas_release_hint()
    self.run_for(1.0, gas=True)
    self.assertTrue(all(self.run_for(GAS_RELEASE_WINDOW - 0.05)))
    self.assert_resumes_within(0.05)

  def test_gas_pressed_is_not_gated(self):
    # the gas override already pauses longitudinal; the gate shouldn't add anything while gas is held
    self.assertFalse(any(self.run_for(1.0, gas=True)))

  def test_rpm_flare_backstop(self):
    # clutch signal missed (stays False) but RPM shoots up with no acceleration: engine decoupled
    self.run_for(1.0)
    rpms = [CRUISE_RPM + 40. * i for i in range(30)]  # +4000 rpm/s
    flared = [upd(self.gate, cs(rpm=r)) for r in rpms]
    self.assertTrue(any(flared))
    self.assertEqual(self.gate.state, GateState.shifting)
    # holds through FLARE_CLEAR_TIME after the flare stops, then re-engage delay, then resumes
    self.assertTrue(all(self.run_for(FLARE_CLEAR_TIME - 0.1, rpm=rpms[-1])))
    self.assert_resumes_within(0.1 + REENGAGE_DELAY, rpm=rpms[-1])

  def test_rpm_rise_while_accelerating_is_not_a_flare(self):
    self.run_for(1.0)
    flared = [upd(self.gate, cs(rpm=CRUISE_RPM + 40. * i, a=1.5)) for i in range(30)]
    self.assertFalse(any(flared))

  def test_rpm_flare_ignored_at_low_speed(self):
    self.run_for(1.0, v=1.0)
    flared = [upd(self.gate, cs(rpm=900. + 40. * i, v=1.0)) for i in range(30)]
    self.assertFalse(any(flared))

  def test_startup_does_not_fake_a_flare(self):
    # first sample already at cruise RPM: must not read as a 0 -> 2200 rpm jump
    self.assertFalse(any(self.run_for(0.5)))

  def test_toggle_change_waits_for_disengage(self):
    # Task 12 B3: a toggle change applies only while openpilot is disengaged - not at the end of a shift, not after a
    # gas override - so it never cuts a shift short and never changes behaviour mid-engagement
    self.run_for(0.3, clutch=True)
    Params().put_bool("ClutchShiftGate", False)
    self.gate.get_params()
    self.assertTrue(all(self.run_for(0.3, clutch=True)), "disabling mid-shift must not cut the shift short")
    self.run_for(REENGAGE_DELAY + 0.2)
    self.run_for(0.5, gas=True)
    self.assertTrue(self.gate.enabled, "still engaged: the old value holds")
    self.assertTrue(any(self.run_for(0.5, clutch=True)))
    upd(self.gate, cs(), enabled=False)
    self.assertFalse(self.gate.enabled)
    self.assertFalse(any(self.run_for(0.5, clutch=True)))


class TestGateTask12(OpenpilotTestCase):
  """Task 12 A2: a shift caps throttle, never braking; the flare backstop is noise-proof; smart never waits MaxCoast."""
  def setup_method(self):
    p = Params()
    for k in ("ClutchShiftGateCoastAccel", "ClutchShiftGateMaxCoast", "ClutchReengageSmart", "DisengageOnAccelerator"):
      p.remove(k)
    p.put_bool("ClutchShiftGate", True, block=True)
    self.gate = ClutchShiftGate(make_cp())

  def run_for(self, seconds, **kw):
    return [upd(self.gate, cs(**kw)) for _ in range(round(seconds / DT_CTRL))]

  def test_shift_caps_throttle_only(self):
    # [A-F-H1, B-C2] longitudinal stays active; the upper limit goes to the coast value, the lower limit is untouched
    self.run_for(0.5, clutch=True)
    self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (-3.5, 0.0))
    for state_setup in (lambda: self.run_for(0.2), ):   # reengaging (clutch out, delay running)
      state_setup()
      self.assertEqual(self.gate.state, GateState.reengaging)
      self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (-3.5, 0.0))

  def test_coast_value_param_clamped(self):
    for value, want in ((-0.3, -0.3), (-5., -0.5), (1.0, 0.0), (float("nan"), 0.0)):
      Params().put("ClutchShiftGateCoastAccel", value, block=True)
      self.gate.get_params()
      upd(self.gate, cs(), enabled=False)
      self.assertEqual(self.gate.coast_accel, want, value)   # (the cap with the clutch in is <= 0 regardless)
      self.run_for(0.3, clutch=True)
      self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (-3.5, want), value)
      self.run_for(1.0)

  def test_blip_while_paused_is_exactly_zero(self):
    # [A-F-M1] the gas override is dropped only while paused; then the panda allows nothing but an inactive accel
    self.run_for(0.3, clutch=True)
    upd(self.gate, cs(clutch=True, gas=True))
    self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (0., 0.))

  def test_no_positive_accel_with_clutch_in_fuzz(self):
    # [B-H1] any gate state, any limits, any gas/rpm/flare history: with the clutch in the upper limit is <= 0
    import random
    rng = random.Random(4)
    for _ in range(6000):
      clutch = rng.random() < 0.4
      upd(self.gate, cs(clutch=clutch, gas=rng.random() < 0.2, rpm=rng.uniform(700, 6000), a=rng.uniform(-2, 2),
                        v=rng.uniform(0, 30)))
      hi = rng.uniform(-1, 2)
      lo, out_hi = self.gate.limit_accel((rng.uniform(-3.5, hi), hi))
      if clutch:
        self.assertLessEqual(out_hi, 0., self.gate.state)
      self.assertLessEqual(lo, out_hi)

  def test_max_coast_clamped_to_5s(self):
    # [A-F-M2]
    for value, want in ((20., 5.), (8., 5.), (3., 3.), (0.5, 2.)):
      Params().put("ClutchShiftGateMaxCoast", value, block=True)
      self.gate.get_params()
      upd(self.gate, cs(), enabled=False)
      self.assertEqual(self.gate.reengage.max_coast, want)
    Params().put("ClutchShiftGateMaxCoast", 20., block=True)
    self.gate.get_params()
    upd(self.gate, cs(), enabled=False)
    self.run_for(5.0, clutch=True)
    self.run_for(0.05, clutch=True)
    self.assertEqual(self.gate.state, GateState.coastTimeout)
    self.assertFalse(self.gate.paused)
    self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (-3.5, 0.0), "coastTimeout: no throttle, braking passes")

  def test_coast_timeout_does_not_start_the_ramp(self):
    # [C-12] `resumed` only on a shift ending into idle
    resumed = []
    for _ in range(round(5.5 / DT_CTRL)):
      upd(self.gate, cs(clutch=True))
      resumed.append(self.gate.resumed)
    self.assertEqual(self.gate.state, GateState.coastTimeout)
    self.assertFalse(any(resumed))

  def noisy_cruise(self, sigma, hold_frames, minutes=10., seed=0):
    import random
    rng = random.Random(seed)
    flares, rpm = 0, CRUISE_RPM
    for i in range(round(minutes * 60 / DT_CTRL)):
      if i % hold_frames == 0:
        rpm = CRUISE_RPM + rng.gauss(0., sigma)
      upd(self.gate, cs(rpm=rpm))
      flares += self.gate.state == GateState.shifting
    return flares

  def test_noisy_rpm_no_false_flares(self):
    # [A-F-M4, B-M3] Gaussian rpm noise at steady cruise, 100 Hz and 25 Hz sample-and-hold: 10 minutes, zero flares
    for sigma in (25., 30.):
      for hold in (1, 4):
        with self.subTest(sigma=sigma, hold=hold):
          self.assertEqual(self.noisy_cruise(sigma, hold), 0)

  def test_genuine_flare_still_detected(self):
    # [A-F-M4] a missed clutch signal while openpilot itself is on the throttle (wheelspin / free-revving engine): the
    # pedal isn't pressed, the rpm runs away at 3000 rpm/s with the car not accelerating with it - also at 25 Hz
    for hold in (1, 4):
      with self.subTest(hold=hold):
        gate = ClutchShiftGate(make_cp())
        for _ in range(100):
          upd(gate, cs())
        rpm, flared = CRUISE_RPM, False
        for i in range(40):
          if i % hold == 0:
            rpm = CRUISE_RPM + 3000. * i * DT_CTRL
          flared |= upd(gate, cs(rpm=rpm, a=0.1))
        self.assertTrue(flared)

  def test_flare_cooldown(self):
    # [B-L2] repeated flares in a short time: the backstop rests instead of re-capping every time
    starts = 0
    for _k in range(4):
      for _ in range(150):   # back to steady (and past the clear hold + re-engage delay)
        upd(self.gate, cs())
      rpm = CRUISE_RPM
      for _i in range(30):
        rpm += 30.
        was = self.gate.state
        upd(self.gate, cs(rpm=rpm))
        starts += was == GateState.idle and self.gate.state == GateState.shifting
      for _ in range(20):
        upd(self.gate, cs(rpm=rpm))
    self.assertEqual(starts, 2, "two flares, then the cool-down")

  def test_smart_without_lock_falls_back(self):
    # [A-F-L3] smart mode, GEAR_MT reports a gear but the ratio never locks: the fixed-delay takeover within 1.5 s
    Params().put_bool("ClutchReengageSmart", True, block=True)
    self.gate.get_params()
    upd(self.gate, cs(), enabled=False)
    self.run_for(0.5, clutch=True)
    t = 0.
    while self.gate.state != GateState.idle and t < 5.:
      upd(self.gate, cs(rpm=1200.), gear_raw=3)   # a ratio that matches no gear: never locks
      t += DT_CTRL
    self.assertLessEqual(t, 1.5)
    self.assertTrue(self.gate.fallback)

  def test_refuses_with_disengage_on_accelerator(self):
    # [B-L3] DisengageOnAccelerator ON: the gate refuses (stays off) and says so (gateBlocked -> HUD)
    Params().put_bool("DisengageOnAccelerator", True, block=True)
    self.gate.get_params()
    upd(self.gate, cs(), enabled=False)
    self.assertFalse(self.gate.enabled)
    self.assertTrue(self.gate.blocked)
    self.assertFalse(any(self.run_for(0.5, clutch=True)))

  def test_inert_without_clutch_signal(self):
    # [A-F-M6] no live clutch signal (a missing GEARBOX_ALT_2 reads as GEAR_MT 0 = clutch in): never enabled
    upd(self.gate, cs(), enabled=False, clutch_valid=False)
    self.assertFalse(self.gate.enabled)
    self.assertFalse(any(upd(self.gate, cs(clutch=True), clutch_valid=False) for _ in range(50)))
    # the signal dying mid-engagement also makes it inert
    upd(self.gate, cs(), enabled=False, clutch_valid=True)
    self.assertTrue(self.gate.enabled)
    upd(self.gate, cs(clutch=True), clutch_valid=False)
    self.assertFalse(self.gate.enabled)
    self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (-3.5, 2.0))
