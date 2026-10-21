"""
Task 12 B1 / B2 / B3, closed loop where it matters (planner -> controlsd glue -> LongControl -> manual car):
  B1  NSB never caps a curve-speed (SCC-V) / map / speed-limit target below the set speed;
  B2  NSB maintain applies Rule 5, margin clamp, repeating overspeed alert (both modes), set-speed-drop grace;
  B3  every feature toggle/param latches only while openpilot is disengaged — not on a shift or a gas override — and
      the latched values are published.
"""
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import ALERT_REPEAT, DEFAULT_HARD_CEILING, \
  HARD_CEILING_RANGE, MAX_OVERSPEED_MARGIN, PULSE_MIN_BAND, SET_DROP_GRACE, NoServiceBrake, read_nsb_params
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import ManualCarSim, set_params
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_no_service_brake_pulse import FakeCP

EventNameSP = custom.OnroadEventSP.EventName
V_SET = 25.


class TestB1CurveAndLimitTargets(OpenpilotTestCase):
  def curve_run(self, nsb, mode="maintain"):
    sim = ManualCarSim(speed=V_SET, gear=6, v_set=V_SET, grade=-0.05, NonAdaptiveLongitudinal=True, NoServiceBrakeHold=nsb,
                       NoServiceBrakeOverspeedMode=mode, SmartCruiseControlVision=True, ClutchShiftGate=False)
    sim.run(1.)
    sim.curvature = 0.012   # ~83 m radius ahead at 25 m/s: SCC-V wants roughly 13 m/s
    sim.run(8.)
    return sim

  def test_scc_vision_braking_not_capped(self):
    for mode in ("maintain", "pulse"):
      with self.subTest(mode=mode):
        on = self.curve_run(True, mode)
        scc = [r for r in on.log if r["target_source"] == str(custom.LongitudinalPlanSP.LongitudinalPlanSource.sccVision)]
        self.assertTrue(scc, "precondition: SCC-V is the plan source")
        self.assertLess(min(r["a_target"] for r in scc), -0.5, "precondition: the curve needs real braking")
        for r in scc:
          self.assertTrue(r["allow_brake"])
          self.assertLessEqual(r["limits"][0], -3.0, "no NSB floor under a lowered target")
        # decel reaches what the plan asks for (command tracks the plan within the LoC's own lag)
        self.assertLess(min(r["sent"] for r in scc), min(r["a_target"] for r in scc) + 0.1)
        off = self.curve_run(False, mode)
        self.assertEqual([r["sent"] for r in on.log], [r["sent"] for r in off.log], "NSB on == NSB off here")

  def test_overspeed_measured_against_effective_target(self):
    nsb = NoServiceBrake(FakeCP(), set_params(NoServiceBrakeHold=True, NoServiceBrakeOverspeedMode="pulse"))
    nsb.update(False, True, 20., 20., False)
    self.assertTrue(nsb.update(True, True, 20., 25., False, None, v_target=20. - 4.))
    self.assertEqual(nsb.state, "urgent", "a lowered target is treated as urgent in pulse mode")
    self.assertAlmostEqual(nsb.overspeed, 4.)


class TestB2Bounds(OpenpilotTestCase):
  def test_fcw_over_set_speed_maintain_releases(self):
    sim = ManualCarSim(speed=V_SET + 2., gear=6, v_set=V_SET, grade=-0.05, NonAdaptiveLongitudinal=True,
                       NoServiceBrakeHold=True, ClutchShiftGate=False)
    sim.run(2.)
    self.assertFalse(sim.plan.allowBrake, "precondition: clamping")
    self.assertGreater(sim.log[-1]["limits"][0], -0.5)
    sim.fcw_override = True
    sim.run(DT_MDL)
    first = sim.log[-round(DT_MDL / 0.01)]
    self.assertTrue(first["fcw"])
    self.assertLessEqual(first["limits"][0], -3.0, "released in the first control frame of the FCW plan")

  def test_margin_clamped_to_ceiling(self):
    for mode in ("maintain", "pulse"):
      p = set_params(NoServiceBrakeOverspeedMargin=12., NoServiceBrakeOverspeedMode=mode)
      self.assertLessEqual(read_nsb_params(p, True).margin, DEFAULT_HARD_CEILING)
      # Task 13 A2 [N-2, N3]: the ceiling raised to its max must not raise the margin past the fixed 6.7 m/s
      p = set_params(NoServiceBrakeOverspeedMargin=15., NoServiceBrakeOverspeedHardCeiling=HARD_CEILING_RANGE[1],
                     NoServiceBrakeOverspeedMode=mode)
      nsb = read_nsb_params(p, True)
      self.assertEqual(nsb.pulse.ceiling, HARD_CEILING_RANGE[1], "precondition: ceiling at its max")
      self.assertLessEqual(nsb.margin, MAX_OVERSPEED_MARGIN)
      self.assertTrue(0. <= nsb.pulse.release < nsb.pulse.trigger < nsb.pulse.ceiling)
      self.assertLessEqual(nsb.pulse.trigger, MAX_OVERSPEED_MARGIN)
      # and the low end: the margin stays under a low ceiling (braking starts before the alert point)
      p = set_params(NoServiceBrakeOverspeedMargin=15., NoServiceBrakeOverspeedHardCeiling=HARD_CEILING_RANGE[0],
                     NoServiceBrakeOverspeedMode=mode)
      self.assertLessEqual(read_nsb_params(p, True).margin, HARD_CEILING_RANGE[0] - PULSE_MIN_BAND + 1e-9)

  def test_braking_begins_by_the_fixed_margin_with_the_ceiling_raised(self):
    # Task 13 A2, closed loop: a steep descent, margin and ceiling both set to their max. Braking (maintain: the plan
    # allows it; pulse: the floor drops below the coast clamp) must begin by set + 6.7 m/s, not near set + 11.
    for mode in ("maintain", "pulse"):
      with self.subTest(mode=mode):
        sim = ManualCarSim(speed=V_SET + 1., gear=6, v_set=V_SET, grade=-0.12, NonAdaptiveLongitudinal=True,
                           NoServiceBrakeHold=True, NoServiceBrakeOverspeedMode=mode, NoServiceBrakeOverspeedMargin=15.,
                           NoServiceBrakeOverspeedHardCeiling=HARD_CEILING_RANGE[1], ClutchShiftGate=False)
        sim.run(1.)
        self.assertFalse(sim.plan.allowBrake if mode == "maintain" else False, "precondition: coasting over set")
        sim.run(25.)
        braking = next((r for r in sim.log if r["limits"][0] < -0.3 - 1e-3), None)
        self.assertIsNotNone(braking, "braking never began")
        self.assertLessEqual(braking["v"] - V_SET, MAX_OVERSPEED_MARGIN + 0.1, "braking began past the fixed margin")
        self.assertLess(max(r["v"] for r in sim.log) - V_SET, HARD_CEILING_RANGE[1], "never reached the raised ceiling")

  def _rising_over_ceiling(self, mode, seconds=12.):
    nsb = NoServiceBrake(FakeCP(), set_params(NoServiceBrakeHold=True, NoServiceBrakeOverspeedMode=mode))
    nsb.update(False, True, V_SET, V_SET, False)
    events, v = [], V_SET + 4.
    for _ in range(round(seconds / DT_MDL)):
      v += 0.05 * DT_MDL * 20   # +1 m/s per second
      nsb.update(True, True, v, V_SET, False)
      events.append(nsb.escalation_event)
    return events

  def test_alert_repeats_while_rising(self):
    for mode in ("pulse", "maintain"):
      with self.subTest(mode=mode):
        events = self._rising_over_ceiling(mode)
        idx = [i for i, e in enumerate(events) if e]
        self.assertGreaterEqual(len(idx), 2, f"{mode}: repeats while still over the ceiling and rising")
        gaps = [(b - a) * DT_MDL for a, b in zip(idx, idx[1:], strict=False)]
        self.assertTrue(all(abs(g - ALERT_REPEAT) < 2 * DT_MDL for g in gaps), gaps)

  def test_set_speed_drop_no_false_escalation(self):
    nsb = NoServiceBrake(FakeCP(), set_params(NoServiceBrakeHold=True, NoServiceBrakeOverspeedMode="pulse"))
    nsb.update(False, True, V_SET, V_SET, False)
    for _ in range(20):
      nsb.update(True, True, V_SET, V_SET, False)
    # the driver drops the set speed by 10 m/s: now "over the ceiling", but that's the driver's doing
    states, events = [], []
    for _ in range(round(SET_DROP_GRACE / DT_MDL) - 1):
      nsb.update(True, True, V_SET, V_SET - 10., False)
      states.append(nsb.state)
      events.append(nsb.escalation_event)
    self.assertNotIn("escalated", states)
    self.assertFalse(any(events))
    # still way over once the grace is up: now it escalates (and alerts)
    for _ in range(3):
      nsb.update(True, True, V_SET, V_SET - 10., False)
    self.assertEqual(nsb.state, "escalated")


class TestB3LatchOnEngagement(OpenpilotTestCase):
  CHANGES = {"NonAdaptiveLongitudinal": True, "NoServiceBrakeHold": True, "NoServiceBrakeOverspeedMode": "pulse",
             "NoServiceBrakeDecelFloor": 0.0, "NoServiceBrakeOverspeedMargin": 2.0, "EngineBrakePreferred": True,
             "GearAwareAccel": True, "ClutchShiftGate": False, "ClutchShiftGateGasReleaseHint": True,
             "ManualLowSpeedHandbackSpeed": 4.0, "ManualLowSpeedHandbackRamp": 1.0}

  @staticmethod
  def latched(sim):
    ext, pl, ml = sim.ext, sim.planner, custom.CarControlSP.new_message().manualLongitudinal
    ext.fill_manual_longitudinal(ml)
    nsb = sim.plan_sp.noServiceBrake
    return {"non_adaptive": pl.non_adaptive, "plan_nsb": (pl.no_service_brake.enabled, pl.no_service_brake.mode,
            pl.no_service_brake.decel_floor, pl.no_service_brake.overspeed_margin),
            "pub_nsb": (nsb.enabled, str(nsb.mode), round(nsb.decelFloor, 3), round(nsb.margin, 3)),
            "ctl_nsb": (ext.no_service_brake_enabled, ext.no_service_brake_floor), "ebp": ext.engine_brake_preferred.enabled,
            "gaa": pl.gear_aware_accel.enabled, "pub_features": (sim.plan_sp.features.nonAdaptive, sim.plan_sp.features.gearAware),
            "gate": (ext.clutch_shift_gate.enabled, ext.clutch_shift_gate.gas_release_hint),
            "handback": (ext.low_speed_handback.speed, ext.low_speed_handback.ramp_time),
            # handbackSpeed is published gear-aware since Stage 3 (it follows GEAR_MT by design); the latched setting is
            # "handback" above
            "pub_ml": (ml.gateEnabled, ml.ebpEnabled, ml.nsbEnabled, round(ml.handbackRamp, 3))}

  def test_mid_engagement_changes_wait_for_disengage(self):
    sim = ManualCarSim(speed=15., gear=3, v_set=15., plant="closed", ClutchShiftGate=True)
    sim.run(1.)
    before = self.latched(sim)
    p = Params()
    for k, v in self.CHANGES.items():
      p.put_bool(k, v, block=True) if isinstance(v, bool) else p.put(k, v, block=True)
    sim.ext.read_params_sp()        # controlsd's 1 Hz read
    sim.run(1.2)                    # plannerd's own 1 Hz read
    sim.run(0.5, clutch=True, gear=4)
    sim.run(1.5, clutch=False)      # a shift
    sim.run(0.5, gas=True)
    sim.run(1.0, gas=False)         # a gas override
    self.assertEqual(self.latched(sim), before, "nothing changes mid-engagement, through a shift and an override")
    sim.enabled = False
    sim.run(DT_MDL)
    sim.enabled = True
    sim.run(DT_MDL * 2)
    after = self.latched(sim)
    for key in before:
      self.assertNotEqual(after[key], before[key], f"{key} applied after disengage + re-engage")

