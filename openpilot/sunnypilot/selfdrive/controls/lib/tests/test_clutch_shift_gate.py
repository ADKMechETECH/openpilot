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


def cs(clutch=False, gas=False, rpm=CRUISE_RPM, v=V_CRUISE, a=0.):
  CS = car.CarState.new_message()
  CS.deprecated.clutchPressed = clutch
  CS.deprecated.engineRpm = rpm
  CS.gasPressed = gas
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

  def test_gas_release_then_clutch(self):
    # driver overriding on gas lifts, then clutches: no gap where longitudinal could resume between the two
    self.run_for(1.0, gas=True)
    paused = self.run_for(0.2) + self.run_for(0.5, clutch=True)
    self.assertTrue(all(paused))
    self.assertEqual(self.gate.state, GateState.shifting)

  def test_gas_release_without_clutch_resumes(self):
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
