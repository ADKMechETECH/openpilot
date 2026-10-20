"""
Clutch-gate re-engagement (Task 8, reworked by Task 12 A2 / B4): the ratio lock (onto the gear GEAR_MT reports, a tight
lock tolerance, steady on filtered signals), the takeover ramp (Rule-5 urgency and step-brake exits), the controlsd
wiring, and closed loops through real shifts with both plant models (manual_sim.ManualCarSim).
"""
import math

import numpy as np

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import HANDBACK_DRIVER_DECEL, HANDBACK_DRIVER_REACTION
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MAX_BP, J_CRUISE_VALS
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_reengage import RatioLock, TakeoverRamp, RATIO_LOCK_TIME, \
  TAKEOVER_JERK_BP, TAKEOVER_JERK_V, CATCHUP_RATE, DEFAULT_GAS_DEBOUNCE, DEFAULT_RATIO_TOL, STEP_BRAKE, \
  read_reengage_params, takeover_rate, takeover_seed, GAS_DEBOUNCE_RANGE, RATIO_TOL_RANGE, MAX_RAMP_RANGE, MAX_COAST_RANGE
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import ClutchShiftGate, GateState, REENGAGE_DELAY, \
  SMART_LOCK_TIMEOUT
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import GEAR_RATIOS, engine_rpm_at, estimate_coast_decel, \
  overall_ratio
from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import JERK_SCALE_BY_GEAR
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import ManualCarSim, make_cp, set_params

V = 20.
LOCK_FRAMES = round(RATIO_LOCK_TIME / DT_CTRL)


def cs(clutch=False, gas=False, gear=4, v=V, rpm=None, a=0.):
  CS = messaging.new_message('carState').carState
  CS.deprecated.clutchPressed = clutch
  CS.deprecated.engineRpm = float(engine_rpm_at(gear, v) if rpm is None else rpm)
  CS.gasPressed, CS.vEgo, CS.aEgo = gas, v, a
  return CS


class TestParamsAndPieces(OpenpilotTestCase):
  def test_defaults_and_clamps(self):
    set_params()
    p = read_reengage_params(Params())
    self.assertEqual((p.smart, p.gas_debounce, p.ratio_tol, p.max_ramp, p.max_coast, p.seed), (False, 0.15, 0.02, 1.0, 5.0, "aego"))
    set_params(ClutchReengageGasDebounce=9., ClutchReengageRatioTol=9., ClutchReengageMaxRamp=9., ClutchShiftGateMaxCoast=99.,
               ClutchReengageSeed="bogus")
    p = read_reengage_params(Params())
    self.assertEqual((p.gas_debounce, p.ratio_tol, p.max_ramp, p.max_coast, p.seed),
                     (GAS_DEBOUNCE_RANGE[1], RATIO_TOL_RANGE[1], MAX_RAMP_RANGE[1], MAX_COAST_RANGE[1], "aego"))
    self.assertLessEqual(MAX_COAST_RANGE[1], 5.0)   # Task 12 A2.3
    set_params(ClutchReengageSeed="coast")
    self.assertEqual(read_reengage_params(Params()).seed, "coast")

  def test_lock_tolerance_much_tighter_than_gear_id(self):
    # Task 12 B4.2 [C-02]: the lock tolerance (<= 4 %) is far inside the +-8 % gear-identification band
    from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import GEAR_MATCH_TOL
    self.assertLessEqual(RATIO_TOL_RANGE[1], GEAR_MATCH_TOL / 2)
    self.assertEqual(DEFAULT_RATIO_TOL, 0.02)

  def test_takeover_jerk_matches_planner(self):
    self.assertEqual(TAKEOVER_JERK_BP, A_CRUISE_MAX_BP)
    self.assertEqual(TAKEOVER_JERK_V, J_CRUISE_VALS)
    for g in range(1, 7):
      self.assertAlmostEqual(takeover_rate(20., g), float(np.interp(20., TAKEOVER_JERK_BP, TAKEOVER_JERK_V)) * JERK_SCALE_BY_GEAR[g - 1])


class TestRatioLock(OpenpilotTestCase):
  def feed(self, lock, frames, gear_raw, **kw):
    out = []
    for _ in range(frames):
      out.append(lock.update(cs(**kw), gear_raw, DEFAULT_RATIO_TOL))
    return out

  def test_locks_at_each_gear_after_the_steady_window(self):
    for g in range(1, len(GEAR_RATIOS) + 1):
      lock = RatioLock()
      out = self.feed(lock, 40, g, gear=g, v=12.)
      self.assertTrue(out[-1], g)
      first = out.index(True)
      self.assertGreaterEqual(first + 1, LOCK_FRAMES, "never before RATIO_LOCK_TIME")
      self.assertEqual(lock.gear, g)

  def test_never_locks_onto_a_gear_the_gearbox_does_not_report(self):
    # [C-02] rpm/speed say 3rd but GEAR_MT says 4th (still slipping down from 3rd): no lock - never onto the old gear
    lock = RatioLock()
    self.assertFalse(any(self.feed(lock, 60, 4, gear=3)))
    self.assertFalse(any(self.feed(RatioLock(), 60, -1, gear=3)), "no raw gear -> no lock")

  def test_uncoupled_cases(self):
    for kw in ({"clutch": True}, {"v": 1.0}, {"rpm": 0.}):
      with self.subTest(**kw):
        self.assertFalse(any(self.feed(RatioLock(), 60, 4, **kw)))

  def slow_release(self, release_s, hold=1, hang_s=0., gear_from=3, gear_to=4, v=15.):
    """Clutch out at t=0 with GEAR_MT reporting gear_to at once; rpm hangs, then converges from gear_from's rpm to
    gear_to's linearly over release_s (a slow, slipping release). Returns (lock frame, first frame the RAW ratio is
    within the lock tolerance and stays there, the locked gear)."""
    lock = RatioLock()
    self.feed(lock, 30, gear_from, gear=gear_from, v=v)
    r0, r1 = engine_rpm_at(gear_from, v), engine_rpm_at(gear_to, v)
    n_hang, n_rel = round(hang_s / DT_CTRL), round(release_s / DT_CTRL)
    in_tol, locked_at, rpm = None, None, r0
    for i in range(n_hang + n_rel + 80):
      raw = r0 if i < n_hang else r0 + (r1 - r0) * min((i - n_hang) / n_rel, 1.)
      if i % hold == 0:
        rpm = raw
      within = abs(rpm / r1 - 1.) <= DEFAULT_RATIO_TOL
      in_tol = (i if in_tol is None else in_tol) if within else None
      if lock.update(cs(gear=gear_to, v=v, rpm=rpm), gear_to, DEFAULT_RATIO_TOL) and locked_at is None:
        locked_at = i
    return locked_at, in_tol, lock.gear

  def test_slow_releases_lock_only_once_slip_is_under_tolerance(self):
    # Task 12 B4 [C-02, C-03]: 0.6 / 0.8 / 1.0 s releases, at 100 Hz and 25 Hz sample-and-hold rpm
    for release in (0.6, 0.8, 1.0):
      for hold in (1, 4):
        with self.subTest(release=release, hold=hold):
          locked_at, in_tol, gear = self.slow_release(release, hold)
          self.assertIsNotNone(locked_at)
          self.assertGreaterEqual(locked_at, in_tol, "never before the slip is under the lock tolerance")
          self.assertEqual(gear, 4, "never the old gear")

  def test_rpm_hang_and_slow_decay(self):
    for hang, release in ((0.3, 1.0), (0.5, 1.5)):
      for hold in (1, 4):
        with self.subTest(hang=hang, release=release, hold=hold):
          locked_at, in_tol, gear = self.slow_release(release, hold, hang_s=hang)
          self.assertGreaterEqual(locked_at, in_tol)
          self.assertEqual(gear, 4)

  def test_steady_slip_outside_the_lock_tolerance_never_locks(self):
    # Task 12 B4.2: the lock tolerance (~2 %) is much tighter than the +-8 % gear identification. A clutch slipping
    # steadily 5 % off 4th's ratio is "4th" to infer_gear and perfectly steady, but must not lock.
    lock = RatioLock()
    target = engine_rpm_at(4, 15.)
    out = [lock.update(cs(gear=4, v=15., rpm=target * 1.05), 4, DEFAULT_RATIO_TOL) for _ in range(150)]
    self.assertFalse(any(out))
    out = [lock.update(cs(gear=4, v=15., rpm=target * 1.01), 4, DEFAULT_RATIO_TOL) for _ in range(150)]
    self.assertTrue(out[-1], "and it locks once the slip is inside the lock tolerance")

  def test_steady_for_at_least_100ms(self):
    # Task 12 B4.3: the ratio must sit steady for at least 100-150 ms - a literal bound, not the module's own constant
    lock = RatioLock()
    out = [lock.update(cs(gear=4, v=15.), 4, DEFAULT_RATIO_TOL) for _ in range(60)]
    self.assertGreaterEqual((out.index(True) + 1) * DT_CTRL, 0.10 - 1e-9)
    # a transient pass through the band (50 ms in tolerance, then slipping again) never locks
    lock = RatioLock()
    target = engine_rpm_at(4, 15.)
    seq = [target * 1.1] * 30 + [target] * 5 + [target * 1.1] * 30
    self.assertFalse(any(lock.update(cs(gear=4, v=15., rpm=r), 4, DEFAULT_RATIO_TOL) for r in seq))

  def test_downshift_slip(self):
    for hold in (1, 4):
      locked_at, in_tol, gear = self.slow_release(0.8, hold, gear_from=4, gear_to=3, v=15.)
      self.assertGreaterEqual(locked_at, in_tol)
      self.assertEqual(gear, 3)


class GateHarness(OpenpilotTestCase):
  def make_gate(self, **params):
    set_params(ClutchShiftGate=True, ClutchReengageSmart=True, **params)
    self.gate = ClutchShiftGate(make_cp()[0], Params())
    self.gear_raw = 4

  def run_for(self, seconds, gear_raw=None, **kw):
    gr = self.gear_raw if gear_raw is None else gear_raw
    out = []
    for _ in range(round(seconds / DT_CTRL)):
      CS = cs(**kw)
      self.gate.update(CS, True, 0 if CS.deprecated.clutchPressed else gr, True)
      out.append(self.gate.paused)
    return out

  def shift(self, gear_to=5, clutch_s=0.5, gas=False, **kw):
    self.run_for(clutch_s, clutch=True, rpm=1500., gas=gas)
    self.gear_raw = gear_to
    return self.run_for(2.0, gear=gear_to, gas=gas, **kw)


class TestSmartGate(GateHarness):
  def test_off_throttle_takeover_on_lock(self):
    self.make_gate()
    self.run_for(1.0)
    after = self.shift(gear_to=5)
    resume = after.index(False)
    self.assertGreaterEqual(resume + 1, LOCK_FRAMES)
    # Task 15: the fixed delay is now 0.15 s (GEAR_MT reports at the lock on the real car), shorter than lock + steady +
    # gas debounce, so "before the fixed delay" no longer holds; the takeover is still at the lock, not the fallback
    self.assertLess(resume * DT_CTRL, SMART_LOCK_TIMEOUT, "at the lock, not the no-lock fallback")

  def test_on_throttle_takeover_waits_for_gas_clear(self):
    self.make_gate()
    self.run_for(1.0)
    self.run_for(0.5, clutch=True, rpm=1500.)
    self.gear_raw = 5
    self.assertTrue(all(self.run_for(1.0, gear=5, gas=True)), "driver still on the gas")
    after = self.run_for(1.0, gear=5)
    self.assertAlmostEqual(after.index(False) * DT_CTRL, DEFAULT_GAS_DEBOUNCE, delta=0.02)

  def test_gas_debounce_prevents_flutter(self):
    self.make_gate()
    self.run_for(0.5, clutch=True, rpm=1500.)
    self.gear_raw = 5
    for _ in range(10):  # feathering: 0.1 s on / 0.1 s off
      self.assertTrue(all(self.run_for(0.1, gear=5, gas=True)))
      self.assertTrue(all(self.run_for(0.1, gear=5)))

  def test_fixed_delay_fallback_below_ratio_lock_speed(self):
    self.make_gate()
    self.run_for(0.5, clutch=True, rpm=900., v=2.5)
    self.gear_raw = 1
    after = self.run_for(1.0, gear=1, v=2.5)
    self.assertAlmostEqual(after.index(False) * DT_CTRL, REENGAGE_DELAY, delta=0.02)

  def test_no_lock_in_time_falls_back(self):
    # Task 12 A2.5 [A-F-L3]: never waits MaxCoast; the fixed-delay takeover after SMART_LOCK_TIMEOUT
    self.make_gate()
    self.run_for(0.5, clutch=True, rpm=1500.)
    self.gear_raw = 5
    after = self.run_for(3.0, gear=5, rpm=4200.)   # matches no gear's ratio
    self.assertAlmostEqual(after.index(False) * DT_CTRL, SMART_LOCK_TIMEOUT, delta=0.02)
    self.assertTrue(self.gate.fallback)

  def test_neutral_coast_never_locks(self):
    # rolling in neutral at idle: idle rpm matches 6th's ratio at ~8 m/s, but GEAR_MT reads 0 (clutch) -> no lock
    self.make_gate()
    self.gear_raw = 0
    self.assertTrue(all(self.run_for(3.0, clutch=True, rpm=750., v=8.)))

  def test_flare_not_raised_while_locked(self):
    self.make_gate()
    self.run_for(1.0)
    rpm = engine_rpm_at(4, V)
    for i in range(40):  # a locked engine accelerating hard: rpm rises with speed, it's coupled
      v = V + 0.05 * i
      self.gate.update(cs(gear=4, v=v, rpm=engine_rpm_at(4, v) + 0.), True, 4, True)
    self.assertEqual(self.gate.state, GateState.idle)
    del rpm

  def test_smart_latches_only_while_disengaged(self):
    self.make_gate()
    Params().put_bool("ClutchReengageSmart", False, block=True)
    self.gate.get_params()
    self.gate.update(cs(), False, 4, True)
    self.assertFalse(self.gate.reengage.smart)
    Params().put_bool("ClutchReengageSmart", True, block=True)
    self.gate.get_params()
    self.run_for(0.5)
    self.assertFalse(self.gate.reengage.smart, "engaged: unchanged")
    self.gate.update(cs(), False, 4, True)
    self.assertTrue(self.gate.reengage.smart)

  def test_resumed_flag_only_into_idle(self):
    self.make_gate()
    self.run_for(0.5)
    self.run_for(0.5, clutch=True, rpm=1500.)
    self.gear_raw = 5
    flags = []
    for _ in range(100):
      self.gate.update(cs(gear=5), True, 5, True)
      flags.append(self.gate.resumed)
    self.assertEqual(sum(flags), 1)


class TestTakeoverRamp(OpenpilotTestCase):
  def ramp_run(self, seed, a_target, frames, gear=4, limits=(ACCEL_MIN, ACCEL_MAX), urgent_at=None, max_ramp=1.0, plan_fn=None):
    ramp = TakeoverRamp()
    ramp.start(seed, a_target, limits, gear, max_ramp)
    out = []
    for k in range(frames):
      tgt = plan_fn(k) if plan_fn else a_target
      lo, hi = ramp.limit_accel(True, V, "fcw" if urgent_at is not None and k >= urgent_at else None, tgt, limits)
      cmd = float(np.clip(tgt, lo, hi))
      ramp.post_update(cmd, tgt, True)
      out.append((cmd, ramp.active))
    return out

  def test_seed_then_per_gear_slew(self):
    out = self.ramp_run(-0.4, 0.6, 200)
    self.assertAlmostEqual(out[0][0], -0.4)
    steps = np.diff([c for c, _ in out])
    self.assertLessEqual(steps.max(), CATCHUP_RATE * DT_CTRL + 1e-9)
    # the first step is the per-gear rate (escalating toward CATCHUP_RATE over the ramp, so slightly above it)
    self.assertGreaterEqual(steps[0], takeover_rate(V, 4) * DT_CTRL - 1e-9)
    self.assertLessEqual(steps[0], takeover_rate(V, 4) * DT_CTRL * 1.05)

  def test_lower_gears_ramp_slower(self):
    first = {g: np.diff([c for c, _ in self.ramp_run(0., 0.6, 5, gear=g)])[0] for g in (1, 4)}
    self.assertLess(first[1], first[4])

  def test_max_ramp_bounds_duration(self):
    out = self.ramp_run(-1.0, 1.5, 300, max_ramp=0.5)
    self.assertFalse(out[60][1])

  def test_positive_seed_never_above_plan_throttle(self):
    ramp = TakeoverRamp()
    self.assertAlmostEqual(ramp.start(0.8, 0.2, (ACCEL_MIN, ACCEL_MAX), 4, 1.0), 0.2)
    self.assertAlmostEqual(TakeoverRamp().start(0.8, -0.3, (ACCEL_MIN, ACCEL_MAX), 4, 1.0), 0.0)

  def test_urgent_ends_it_at_once(self):
    out = self.ramp_run(0.0, -2.5, 30, urgent_at=5)
    self.assertFalse(out[5][1])
    self.assertAlmostEqual(out[5][0], -2.5, msg="needed braking passes the same frame")

  def test_step_brake_request_ends_it(self):
    # Task 12 A2.10 [A-F-L2]: the plan falling more than STEP_BRAKE below the ramp -> full plan braking at once
    out = self.ramp_run(-0.5, 0.3, 30, plan_fn=lambda k: 0.3 if k < 10 else -1.2)
    self.assertTrue(out[9][1])
    self.assertLess(-1.2, out[9][0] - STEP_BRAKE)
    self.assertFalse(out[10][1])
    self.assertAlmostEqual(out[10][0], -1.2)

  def test_never_widens_limits(self):
    for limits in ((-0.3, 2.0), (-3.5, 0.0), (-1.0, -0.2)):
      for cmd, _ in self.ramp_run(-2.0, 1.0, 120, limits=limits):
        self.assertGreaterEqual(cmd, limits[0] - 1e-9)
        self.assertLessEqual(cmd, limits[1] + 1e-9)

  def test_seed_modes(self):
    # Task 12 B4.4: "aego" = the measured accel; "coast" = aEgo minus the engine-brake coast estimate
    c = cs(gear=3, v=15., a=-0.9)
    self.assertAlmostEqual(takeover_seed("aego", c, 3, 0.), -0.9)
    self.assertAlmostEqual(takeover_seed("coast", c, 3, 0.), -0.9 - estimate_coast_decel(3, c.deprecated.engineRpm, 15., 0.))
    self.assertAlmostEqual(takeover_seed("coast", c, 0, 0.), -0.9, msg="unknown gear -> aEgo")


def downshift(sim):
  """4th -> 3rd while holding set speed (driver downshifts for engine braking), 0.6 s clutch with a blip."""
  sim.run(3.0)
  sim.run(0.25, clutch=True)
  sim.run(0.15, blip=True)
  sim.run(0.2, blip=False)
  sim.run(4.0, clutch=False, gear=3)
  return sim


class TestClosedLoop(OpenpilotTestCase):
  def resume_t(self, sim, after):
    return next(r["t"] for r in sim.log if r["t"] >= after and r["ramp"])

  def test_braking_passes_during_a_shift(self):
    # Task 12 A2.1 [A-F-H1, B-C2]: the plan asks to brake (set speed dropped 6 m/s) mid-shift: the command goes negative
    # while the clutch is in - no coast hole
    for plant in ("open", "closed"):
      sim = ManualCarSim(speed=22., gear=4, plant=plant)
      sim.run(2.0)
      sim.run(1.5, clutch=True, v_set=16.)
      shifting = [r for r in sim.log if r["t"] > 2.3 and r["clutch"]]
      self.assertTrue(all(r["active"] for r in shifting), plant)
      self.assertLess(min(r["cmd"] for r in shifting), -0.5, plant)
      self.assertTrue(all(r["cmd"] <= 1e-9 for r in shifting), "never throttle with the clutch in")

  def test_resume_plant_jerk_and_decel(self):
    # Task 12 B4 [C-03]: the seed-continuity check measures the CAR (plant jerk, decel) around the resume, not cmd ~ aEgo.
    # Numbers (both plants, fixed-delay and smart with each seed) are recorded in CLAUDE.md / printed here.
    results = {}
    for plant in ("open", "closed"):
      for label, kw in (("fixed", {}), ("smart_aego", {"ClutchReengageSmart": True}),
                        ("smart_coast", {"ClutchReengageSmart": True, "ClutchReengageSeed": "coast"})):
        sim = downshift(ManualCarSim(speed=20., gear=4, plant=plant, **kw))
        t0 = self.resume_t(sim, 3.6)
        window = [r for r in sim.log if t0 - 0.5 <= r["t"] <= t0 + 1.5]
        jerk = sim.plant_jerk(t0 - 0.5, t0 + 1.5)
        decel = min(r["a_ego"] for r in window)
        results[(plant, label)] = (round(t0, 2), round(jerk, 2), round(decel, 2))
    print("resume (t, peak plant jerk m/s^3, min a_ego m/s^2):", results)
    # provisional "smooth resume" bound (the console's G2 rule), for every combination except the one C2 rules out
    for key, (_, jerk, decel) in results.items():
      if key != ("open", "smart_aego"):
        self.assertLess(jerk, 3.0, key)
        self.assertGreater(decel, -0.5, key)
    # the known C2 risk, pinned so it stays visible: if the car ADDS a negative command to engine braking (open plant),
    # the default "aego" seed brakes on top of the engine braking at the takeover - a brake tap. "coast" avoids it.
    self.assertGreater(results[("open", "smart_aego")][1], results[("open", "smart_coast")][1])
    self.assertLess(results[("open", "smart_aego")][2], results[("open", "smart_coast")][2] - 0.2)

  def test_on_throttle_upshift_waits_for_the_lift(self):
    sim = ManualCarSim(speed=15., gear=3, v_set=25., ClutchReengageSmart=True)
    sim.run(2.0, gas=True)
    sim.run(0.5, clutch=True, gas=False)
    sim.run(1.5, clutch=False, gear=4, gas=True)
    self.assertFalse(any(r["ramp"] for r in sim.log))
    sim.run(2.0, gas=False)
    t = self.resume_t(sim, 4.0)
    self.assertAlmostEqual(t - 4.0, DEFAULT_GAS_DEBOUNCE, delta=0.04)

  def test_floors_hold_through_takeover(self):
    for kw in ({"EngineBrakePreferred": True}, {"NoServiceBrakeHold": True, "NonAdaptiveLongitudinal": True}):
      sim = downshift(ManualCarSim(speed=20., gear=4, ClutchReengageSmart=True, **kw))
      for r in sim.log:
        self.assertGreaterEqual(r["cmd"], r["limits"][0] - 1e-9, kw)

  def test_neutral_coast_timeout_closed_loop(self):
    # coasting downhill in neutral, engaged: braking available throughout, never throttle, timeout after MaxCoast
    sim = ManualCarSim(speed=20., v_set=20., grade=-0.04)
    sim.run(1.0)
    sim.run(7.0, clutch=True)
    back = [r for r in sim.log if r["t"] > 1.0 + 5.0 + 0.05]
    self.assertTrue(all(r["gate"] == "coastTimeout" for r in back))
    self.assertTrue(all(r["cmd"] <= 0. for r in sim.log if r["clutch"]))
    self.assertLess(min(r["cmd"] for r in sim.log if r["clutch"]), -0.1)

  def test_neutral_coast_toward_stopped_lead_adaptive(self):
    # Task 12 A2 [A-F-H1]: adaptive mode, coasting in neutral (clutch in) toward a stopped car: openpilot brakes, no contact.
    # Task 13 A1: it brakes down to the low-speed handback (the stopped car is still ~9-12 m off at 2.2 m/s: not
    # imminent), hands back there, and the driver (2 m/s^2 after a 1.5 s reaction) takes the stop.
    for plant in ("open", "closed"):
      sim = ManualCarSim(speed=20., gear=4, plant=plant, lead=(120., 0.))
      sim.driver_brake, sim.driver_reaction = HANDBACK_DRIVER_DECEL, HANDBACK_DRIVER_REACTION
      sim.run(1.0)
      sim.run(14.0, clutch=True)
      self.assertFalse(sim.contact, plant)
      self.assertGreater(sim.min_gap(), 2.0, plant)
      self.assertLess(sim.log[-1]["v"], 0.5, plant)
      hb = next(r for r in sim.log if r["handback"])
      self.assertLess(min(r["sent"] for r in sim.log if r["t"] < hb["t"]), -1.0, f"{plant}: openpilot braked from 20 m/s")
      self.assertGreater(hb["gap"], 8., plant)


class TestSeedMath(OpenpilotTestCase):
  def test_coast_seed_is_relative_to_engine_braking(self):
    v, g = 15., 3
    rpm = engine_rpm_at(g, v)
    a_eb = estimate_coast_decel(g, rpm, v, 0.)
    # pure engine braking measured: the coast-corrected seed is ~0 (no brakes on top)
    self.assertAlmostEqual(takeover_seed("coast", cs(gear=g, v=v, a=a_eb), g, 0.), 0., places=6)
    self.assertTrue(math.isfinite(overall_ratio(g)))
