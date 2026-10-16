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


class TestClutchShiftGate(OpenpilotTestCase):
  def setup_method(self):
    Params().put_bool("ClutchShiftGate", True)
    self.gate = ClutchShiftGate(make_cp())

  def run_for(self, seconds, **kw):
    return [self.gate.update(cs(**kw)) for _ in range(round(seconds / DT_CTRL))]

  def test_default_off(self):
    Params().remove("ClutchShiftGate")
    gate = ClutchShiftGate(make_cp())
    self.assertFalse(gate.enabled)
    self.assertFalse(any(gate.update(cs(clutch=True)) for _ in range(100)))

  def test_unavailable_on_automatics(self):
    for transmission in (TransmissionType.automatic, TransmissionType.cvt):
      gate = ClutchShiftGate(make_cp(transmission))
      self.assertFalse(gate.available)
      self.assertFalse(any(gate.update(cs(clutch=True)) for _ in range(100)))

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
    held = [self.gate.update(cs(rpm=r, a=-0.4)) for r in rpms]
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
    blip = [self.gate.update(cs(clutch=True, gas=True, rpm=1600. + 120. * i)) for i in range(15)]  # +12000 rpm/s
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
    held += [self.gate.update(cs(clutch=True, brake=True, gas=True, rpm=1800. + 120. * i, a=-2.0)) for i in range(15)]
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
    [self.gate.update(cs(rpm=CRUISE_RPM + 40. * i)) for i in range(10)]
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
    flared = [self.gate.update(cs(rpm=r)) for r in rpms]
    self.assertTrue(any(flared))
    self.assertEqual(self.gate.state, GateState.shifting)
    # holds through FLARE_CLEAR_TIME after the flare stops, then re-engage delay, then resumes
    self.assertTrue(all(self.run_for(FLARE_CLEAR_TIME - 0.1, rpm=rpms[-1])))
    self.assert_resumes_within(0.1 + REENGAGE_DELAY, rpm=rpms[-1])

  def test_rpm_rise_while_accelerating_is_not_a_flare(self):
    self.run_for(1.0)
    flared = [self.gate.update(cs(rpm=CRUISE_RPM + 40. * i, a=1.5)) for i in range(30)]
    self.assertFalse(any(flared))

  def test_rpm_flare_ignored_at_low_speed(self):
    self.run_for(1.0, v=1.0)
    flared = [self.gate.update(cs(rpm=900. + 40. * i, v=1.0)) for i in range(30)]
    self.assertFalse(any(flared))

  def test_startup_does_not_fake_a_flare(self):
    # first sample already at cruise RPM: must not read as a 0 -> 2200 rpm jump
    self.assertFalse(any(self.run_for(0.5)))

  def test_toggle_change_waits_for_idle(self):
    self.run_for(0.3, clutch=True)
    Params().put_bool("ClutchShiftGate", False)
    self.gate.get_params()
    self.assertTrue(all(self.run_for(0.3, clutch=True)), "disabling mid-shift must not cut the shift short")
    self.run_for(REENGAGE_DELAY + 0.2)
    self.assertFalse(self.gate.enabled)
    self.assertFalse(any(self.run_for(0.5, clutch=True)))
