"""
Task 13 A3 [review2 N2], end to end: GEARBOX_ALT_2 (0x1DD) present but bad. The closed loop's CarStateSP comes from the
real opendbc Honda carstate, fed real CAN frames every control frame (every subscribed message valid; 0x1DD with GEAR_MT
from the sim's clutch/gear and either a clean or a stuck counter). With the bad counter: canValid stays True (the car
keeps driving) and the clutch gate, smart re-engagement, engine-brake-preferred and gear-aware accel are all inert.
The pair to opendbc's canValid-with-0x1DD-absent test.
"""
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.cereal import messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import STOCK
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import ManualCarSim


class OpendbcClutchSim(ManualCarSim):
  def __init__(self, stuck_counter=False, **kw):
    CP = CarInterface.get_params(CAR.HONDA_CIVIC_2022, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.HONDA_CIVIC_2022, gen_empty_fingerprint(), [], False, False, False)
    self.dbc_cs = CarInterface.CarState(CP, CP_SP)
    self.parsers = self.dbc_cs.get_can_parsers(CP, CP_SP)
    self.dbc_cs.update(self.parsers)
    self.packers = {b: CANPacker(cp.dbc_name) for b, cp in self.parsers.items()}
    self.stuck_counter, self.can_frames, self.can_valid_seen = stuck_counter, 0, []
    super().__init__(**kw)

  def car_state_sp(self):
    self.can_frames += 1
    t = int(self.can_frames * 1e7)
    for b, cp in self.parsers.items():
      msgs = [self.packers[b].make_can_msg(addr, cp.bus, {}) for addr in cp.message_states if addr != 0x1DD]
      if b == Bus.pt:
        vals = {"GEAR_MT": 0 if self.clutch else self.gear}
        if self.stuck_counter:
          vals["COUNTER"] = 1
        msgs.append(self.packers[b].make_can_msg("GEARBOX_ALT_2", cp.bus, vals))
      cp.update([t, msgs])
    _, dbc_sp = self.dbc_cs.update(self.parsers)
    self.can_valid_seen.append(all(cp.can_valid for cp in self.parsers.values()))
    sp = messaging.new_message('carStateSP').carStateSP
    sp.clutchSignalValid, sp.gearRaw = dbc_sp.clutchSignalValid, dbc_sp.gearRaw
    return sp


class TestPresentButBadClutchSignal(OpenpilotTestCase):
  def test_gate_and_smart_inert(self):
    for stuck in (False, True):
      with self.subTest(stuck_counter=stuck):
        sim = OpendbcClutchSim(stuck, speed=25., gear=5, v_set=9., NonAdaptiveLongitudinal=True, ClutchShiftGate=True,
                               ClutchReengageSmart=True)
        sim.enabled = False
        sim.run(0.5)
        sim.enabled = True
        sim.run(1.)
        sim.run(0.6, clutch=True, gear=6)
        gate = sim.ext.clutch_shift_gate
        self.assertTrue(all(sim.can_valid_seen), "canValid must stay True")
        if stuck:
          self.assertFalse(sim.ext.clutch_valid)
          self.assertTrue(gate.blocked and not gate.enabled, "gate refuses (HUD 'Shift gate off')")
          self.assertFalse(any(r["paused"] for r in sim.log), "no pause / throttle cap from a bad signal")
        else:
          self.assertTrue(sim.ext.clutch_valid)
          self.assertTrue(gate.enabled)
          self.assertTrue(any(r["paused"] for r in sim.log), "control: the clean signal does pause the shift")

  def test_ebp_inert(self):
    for stuck in (False, True):
      with self.subTest(stuck_counter=stuck):
        sim = OpendbcClutchSim(stuck, speed=25., gear=6, v_set=24.7, NonAdaptiveLongitudinal=True,
                               EngineBrakePreferred=True, ClutchShiftGate=False)
        sim.enabled = False
        sim.run(0.5)
        sim.enabled = True
        suppressed = False
        for _ in range(60):
          sim.run(0.1)
          suppressed |= sim.ext.engine_brake_preferred.suppress
        self.assertTrue(all(sim.can_valid_seen))
        self.assertEqual(suppressed, not stuck, "EBP suppresses only with a live clutch signal")

  def test_gear_aware_inert(self):
    for stuck in (False, True):
      with self.subTest(stuck_counter=stuck):
        sim = OpendbcClutchSim(stuck, speed=12., gear=2, v_set=10., NonAdaptiveLongitudinal=True, GearAwareAccel=True,
                               ClutchShiftGate=False)
        sim.enabled = False
        sim.run(0.5)
        sim.enabled = True
        sim.run(1.)
        self.assertTrue(all(sim.can_valid_seen))
        self.assertEqual(sim.planner.gear_aware_accel.scales == STOCK, stuck)

  def test_goes_inert_mid_engagement(self):
    # clean, the gate and EBP working; then the counter sticks: inert within a few frames, CAN still valid
    sim = OpendbcClutchSim(False, speed=25., gear=6, v_set=24.7, NonAdaptiveLongitudinal=True,
                           EngineBrakePreferred=True, ClutchShiftGate=True)
    sim.enabled = False
    sim.run(0.5)
    sim.enabled = True
    for _ in range(60):
      sim.run(0.1)
      if sim.ext.engine_brake_preferred.suppress:
        break
    self.assertTrue(sim.ext.engine_brake_preferred.suppress, "precondition: EBP suppressing")
    self.assertTrue(sim.ext.clutch_shift_gate.enabled)
    sim.stuck_counter = True
    sim.run(0.05)
    self.assertFalse(sim.ext.clutch_valid)
    self.assertFalse(sim.ext.engine_brake_preferred.suppress)
    self.assertTrue(all(sim.can_valid_seen))
