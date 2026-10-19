"""
Task 8 — clutch-gate smart re-engagement: ratio-lock coupling (8a), shift timeout (8b), bumpless throttle-aware takeover
(8c). Unit tests on synthetic carState sequences, the controlsd wiring (ControlsExt.apply_clutch_takeover), and a closed
loop through real shifts: LongitudinalPlanner -> NSB floor / EBP -> gate + takeover -> LongControl -> a manual car with a
clutch, a slipping bite, engine braking and a driver throttle.
"""
import math

import numpy as np

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, A_CRUISE_MAX_BP, J_CRUISE_VALS
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_reengage import RatioLock, TakeoverRamp, RATIO_LOCK_FRAMES, \
  TAKEOVER_JERK_BP, TAKEOVER_JERK_V, CATCHUP_RATE, DEFAULT_MAX_COAST, DEFAULT_GAS_DEBOUNCE, read_reengage_params, takeover_rate, \
  takeover_urgent, GAS_DEBOUNCE_RANGE, RATIO_TOL_RANGE, MAX_RAMP_RANGE, MAX_COAST_RANGE
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import ClutchShiftGate, GateState, REENGAGE_DELAY
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import G, GEAR_RATIOS, engine_rpm_at, \
  estimate_coast_decel, resistive_decel
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import EngineBrakePreferred
from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import JERK_SCALE_BY_GEAR
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import apply_brake_floor, read_decel_floor
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_manual_low_speed_handback import make_cp

V = 20.
KEYS = ("ClutchShiftGate", "ClutchReengageSmart", "ClutchReengageGasDebounce", "ClutchReengageRatioTol",
        "ClutchReengageMaxRamp", "ClutchShiftGateMaxCoast", "ClutchShiftGateGasReleaseHint", "NonAdaptiveLongitudinal",
        "NoServiceBrakeHold", "NoServiceBrakeDecelFloor", "EngineBrakePreferred", "ManualLowSpeedHandbackSpeed")


def setup_params(gate=True, smart=True, **params):
  p = Params()
  for k in KEYS:
    p.remove(k)
  p.put_bool("ClutchShiftGate", gate)
  p.put_bool("ClutchReengageSmart", smart)
  for k, v in params.items():
    if isinstance(v, bool):
      p.put_bool(k, v)
    else:
      p.put(k, v)


def cs(clutch=False, gas=False, gear=4, v=V, rpm=None, a=0., brake=False, aeb=False):
  CS = messaging.new_message('carState').carState
  CS.deprecated.clutchPressed = clutch
  CS.deprecated.engineRpm = float(engine_rpm_at(gear, v) if rpm is None else rpm)
  CS.gasPressed, CS.brakePressed, CS.vEgo, CS.aEgo = gas, brake, v, a
  CS.stockAeb = aeb
  return CS


def plan_msg(a_target=0., fcw=False, should_stop=False):
  plan = messaging.new_message('longitudinalPlan').longitudinalPlan
  plan.aTarget, plan.fcw, plan.shouldStop = a_target, fcw, should_stop
  plan.allowBrake = True
  return plan


def radar_msg(lead=None):
  radar = messaging.new_message('radarState').radarState
  if lead is not None:
    d_rel, v_rel = lead
    radar.leadOne.present = True
    radar.leadOne.dRel, radar.leadOne.vRel, radar.leadOne.vLead = d_rel, v_rel, V + v_rel
  return radar


class TestParamsAndPieces(OpenpilotTestCase):
  def test_defaults_and_clamps(self):
    setup_params()
    rp = read_reengage_params(Params())
    self.assertTrue(rp.smart)
    self.assertEqual((rp.gas_debounce, rp.ratio_tol, rp.max_ramp, rp.max_coast), (0.15, 0.08, 1.0, DEFAULT_MAX_COAST))
    self.assertEqual(rp.ratio_min_speed, 3.0)  # infer_gear's floor is above the default handback speed (2.24)
    for key, rng in (("ClutchReengageGasDebounce", GAS_DEBOUNCE_RANGE), ("ClutchReengageRatioTol", RATIO_TOL_RANGE),
                     ("ClutchReengageMaxRamp", MAX_RAMP_RANGE), ("ClutchShiftGateMaxCoast", MAX_COAST_RANGE)):
      attr = {"ClutchReengageGasDebounce": "gas_debounce", "ClutchReengageRatioTol": "ratio_tol",
              "ClutchReengageMaxRamp": "max_ramp", "ClutchShiftGateMaxCoast": "max_coast"}[key]
      for raw, want in ((-100., rng[0]), (100., rng[1]), (float("nan"), getattr(rp, attr))):
        setup_params(**{key: raw})
        self.assertEqual(getattr(read_reengage_params(Params()), attr), want, (key, raw))
    setup_params(ManualLowSpeedHandbackSpeed=6.0)
    self.assertEqual(read_reengage_params(Params()).ratio_min_speed, 6.0)

  def test_smart_default_off(self):
    Params().remove("ClutchReengageSmart")
    self.assertFalse(read_reengage_params(Params()).smart)

  def test_ratio_tol_max_keeps_gear_bands_apart(self):
    tol = RATIO_TOL_RANGE[1]
    for hi, lo in zip(GEAR_RATIOS, GEAR_RATIOS[1:], strict=False):
      self.assertLess(lo * (1 + tol), hi * (1 - tol))

  def test_takeover_jerk_matches_planner(self):
    # the takeover's base rate is the planner's set-speed jerk; it's duplicated so controlsd doesn't import the MPC
    self.assertEqual((TAKEOVER_JERK_BP, TAKEOVER_JERK_V), (list(A_CRUISE_MAX_BP), list(J_CRUISE_VALS)))
    for gear in range(1, 7):
      self.assertAlmostEqual(takeover_rate(V, gear), np.interp(V, A_CRUISE_MAX_BP, J_CRUISE_VALS) * JERK_SCALE_BY_GEAR[gear - 1])
    self.assertAlmostEqual(takeover_rate(V, 0), np.interp(V, A_CRUISE_MAX_BP, J_CRUISE_VALS))


class TestRatioLock(OpenpilotTestCase):
  def test_locks_at_each_gear_after_debounce(self):
    for gear in range(1, 7):
      lock = RatioLock()
      states = [lock.update(cs(gear=gear, v=15.), 0.08) for _ in range(RATIO_LOCK_FRAMES)]
      self.assertEqual(states, [False] * (RATIO_LOCK_FRAMES - 1) + [True], gear)
      self.assertEqual(lock.gear, gear)

  def test_uncoupled_cases(self):
    lock = RatioLock()
    rpm4 = engine_rpm_at(4, V)
    for label, state in (("clutch in", cs(clutch=True)), ("slip 10%", cs(rpm=rpm4 * 0.9)), ("neutral idle", cs(rpm=750.)),
                         ("no rpm", cs(rpm=0.)), ("nan rpm", cs(rpm=float("nan"))), ("too slow", cs(v=2.0, gear=1)),
                         ("between gears", cs(rpm=(engine_rpm_at(4, V) + engine_rpm_at(5, V)) / 2))):
      self.assertFalse(any(lock.update(state, 0.08) for _ in range(20)), label)

  def test_transient_match_does_not_lock(self):
    # a slipping clutch sweeping through a gear's band for fewer than RATIO_LOCK_FRAMES frames is not a lock
    lock = RatioLock()
    rpm4 = engine_rpm_at(4, V)
    seq = [rpm4 * 0.9] * 5 + [rpm4] * (RATIO_LOCK_FRAMES - 1) + [rpm4 * 0.9] * 5
    self.assertFalse(any(lock.update(cs(rpm=r), 0.08) for r in seq))

  def test_slip_sweeping_through_adjacent_gear_does_not_lock(self):
    # upshift 4th -> 5th: the slipping clutch pulls rpm down from above, through 4th's band, to 5th's - only the steady
    # end is a lock, and it's the right gear
    lock = RatioLock()
    rpm4, rpm5 = engine_rpm_at(4, V), engine_rpm_at(5, V)
    sweep = [rpm4 * 1.05 + (rpm5 - rpm4 * 1.05) * k / 30 for k in range(31)]
    out = [lock.update(cs(rpm=r), 0.08) for r in sweep]
    self.assertFalse(any(out[:-1]))
    out = [lock.update(cs(rpm=rpm5), 0.08) for _ in range(RATIO_LOCK_FRAMES)]
    self.assertTrue(out[-1])
    self.assertEqual(lock.gear, 5)

  def test_gear_change_restarts_debounce(self):
    lock = RatioLock()
    lock.update(cs(gear=4), 0.08)
    lock.update(cs(gear=4), 0.08)
    self.assertFalse(lock.update(cs(gear=5), 0.08))
    self.assertEqual(lock.frames, 1)

  def test_tolerance_param(self):
    lock = RatioLock()
    state = cs(rpm=engine_rpm_at(4, V) * 1.06)
    self.assertTrue(any(lock.update(state, 0.08) for _ in range(5)))
    lock.reset()
    self.assertFalse(any(lock.update(state, 0.04) for _ in range(5)))


class GateHarness(OpenpilotTestCase):
  def make_gate(self, **params):
    setup_params(**params)
    self.gate = ClutchShiftGate(make_cp()[0])

  def run_for(self, seconds, **kw):
    return [self.gate.update(cs(**kw)) for _ in range(round(seconds / DT_CTRL))]

  def shift(self, gear_to=5, clutch_s=0.5, gas=False, **kw):
    """clutch in, then clutch out in gear_to with GEAR_MT reporting the gear (ratio not locked yet: slipping)."""
    self.run_for(clutch_s, clutch=True, rpm=1500., gas=gas, **kw)


class TestSmartGate(GateHarness):
  def test_off_throttle_takeover_immediate_on_lock(self):
    self.make_gate()
    self.run_for(1.0)
    self.shift()
    slip = self.run_for(0.3, gear=5, rpm=engine_rpm_at(5, V) * 0.9)  # GEAR_MT shows 5th, clutch still slipping
    self.assertTrue(all(slip))
    self.assertEqual(self.gate.state, GateState.reengaging)
    locked = self.run_for(0.1, gear=5)
    # resumes exactly when the ratio lock is debounced (no fixed delay, gas already clear)
    self.assertEqual(locked.index(False), RATIO_LOCK_FRAMES - 1)
    self.assertEqual(self.gate.state, GateState.idle)

  def test_on_throttle_takeover_waits_for_gas_clear(self):
    self.make_gate()
    self.run_for(1.0, gas=True)
    self.shift(gas=False)
    held = self.run_for(1.0, gear=5, gas=True)  # coupled, driver still on the throttle
    self.assertTrue(all(held))
    self.assertEqual(self.gate.state, GateState.reengaging)
    self.assertEqual(self.gate.coast_time, 0.)  # coupled + waiting on gas is not "coasting" for the timeout
    after = self.run_for(0.5, gear=5)
    self.assertEqual(after.index(False), round(DEFAULT_GAS_DEBOUNCE / DT_CTRL) - 1)

  def test_gas_debounce_prevents_flutter(self):
    # driver feathering the throttle (on/off every 0.1 s) after the shift: no grab/yield cycling
    self.make_gate()
    self.shift(gas=True)  # blip in the shift: gas just released at clutch-out
    out = []
    for _ in range(10):
      out += self.run_for(0.1, gear=5, gas=True) + self.run_for(0.1, gear=5)
    self.assertTrue(all(out))
    self.assertEqual(self.gate.state, GateState.reengaging)
    self.assertIn(False, self.run_for(0.3, gear=5))

  def test_zero_debounce(self):
    self.make_gate(ClutchReengageGasDebounce=0.0)
    self.shift()
    self.run_for(0.2, gear=5, gas=True)
    self.assertFalse(self.gate.update(cs(gear=5)))

  def test_fixed_delay_fallback_below_ratio_lock_speed(self):
    # near idle ratio lock isn't meaningful (and the handback owns that regime): fall back to the fixed delay
    self.make_gate()
    self.shift(v=2.6)
    out = self.run_for(1.0, gear=1, v=2.6)
    self.assertEqual(out.index(False), round(REENGAGE_DELAY / DT_CTRL))  # same timing as the fixed-delay gate

  def test_neutral_coast_never_locks_from_ratio_alone(self):
    # rolling in neutral at idle can pass through a speed where idle rpm matches 6th's ratio; GEAR_MT == 0 still holds
    self.make_gate()
    v_match = 750. / engine_rpm_at(6, 1.)
    self.assertTrue(all(self.run_for(1.0, clutch=True, rpm=750., v=v_match)))
    self.assertEqual(self.gate.state, GateState.shifting)

  def test_flare_not_raised_while_locked(self):
    # the tail of a downshift's re-coupling: rpm still climbing after the lock must not re-pause via the backstop
    self.make_gate()
    self.shift()
    rpm4 = engine_rpm_at(4, V)
    rpms = [rpm4 * (0.93 + 0.004 * i) for i in range(15)]
    out = [self.gate.update(cs(gear=4, rpm=r, a=-0.8)) for r in rpms]
    self.assertFalse(any(out[RATIO_LOCK_FRAMES:]))
    self.assertEqual(self.gate.flare_clear_timer, 0.)

  def test_smart_latches_only_when_idle(self):
    self.make_gate(smart=False)
    self.shift()
    Params().put_bool("ClutchReengageSmart", True)
    self.gate.get_params()
    self.run_for(0.1, clutch=True)
    self.assertFalse(self.gate.reengage.smart)  # mid-shift: unchanged
    out = self.run_for(REENGAGE_DELAY + 0.1, gear=5)
    self.assertEqual(out.index(False), round(REENGAGE_DELAY / DT_CTRL))  # this shift: fixed delay
    self.assertTrue(self.gate.reengage.smart)

  def test_resumed_flag(self):
    self.make_gate()
    self.shift()
    flags = []
    for _ in range(10):
      self.gate.update(cs(gear=5))
      flags.append(self.gate.resumed)
    self.assertEqual(sum(flags), 1)


class TestShiftTimeout(GateHarness):
  def test_neutral_coast_times_out(self):
    for smart in (False, True):
      self.make_gate(smart=smart)
      out = self.run_for(DEFAULT_MAX_COAST + 1.0, clutch=True, rpm=750.)
      n = round(DEFAULT_MAX_COAST / DT_CTRL)
      self.assertTrue(all(out[:n]), smart)
      self.assertFalse(any(out[n + 1:]), smart)
      self.assertEqual(self.gate.state, GateState.coastTimeout)
      # braking allowed, throttle capped while the engine is still disconnected
      self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (-3.5, 0.))
      self.assertEqual(self.gate.limit_accel((0.2, 2.0)), (0., 0.))

  def test_timeout_param_and_back_into_gear(self):
    self.make_gate(ClutchShiftGateMaxCoast=3.0)
    self.run_for(3.2, clutch=True, rpm=750.)
    self.assertEqual(self.gate.state, GateState.coastTimeout)
    self.assertFalse(any(self.run_for(2.0, clutch=True, rpm=750.)))  # stays timed out while in neutral
    # back into gear: a normal re-engagement (pause through the bite, then takeover), throttle cap gone
    self.assertTrue(self.gate.update(cs(gear=4, rpm=engine_rpm_at(4, V) * 0.8)))
    self.assertEqual(self.gate.state, GateState.reengaging)
    self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (-3.5, 2.0))
    self.assertIn(False, self.run_for(0.2, gear=4))

  def test_no_ratio_lock_times_out_to_idle(self):
    # GEAR_MT says in gear but the ratio never matches (wrong seed ratios / slipping clutch): don't coast forever
    self.make_gate()
    self.shift()
    out = self.run_for(DEFAULT_MAX_COAST + 0.5, gear=5, rpm=engine_rpm_at(5, V) * 1.15)
    self.assertFalse(out[-1])
    self.assertEqual(self.gate.state, GateState.idle)
    self.assertEqual(self.gate.limit_accel((-3.5, 2.0)), (-3.5, 2.0))

  def test_real_shifts_never_time_out(self):
    self.make_gate()
    for _ in range(20):
      self.shift(clutch_s=1.5)
      self.run_for(0.5, gear=5)
    self.assertEqual(self.gate.state, GateState.idle)


class FakeSM(dict):
  def __init__(self, *a, **kw):
    super().__init__(*a, **kw)
    self.valid = dict.fromkeys(('longitudinalPlan', 'radarState'), True)


class _Ext(ControlsExt):
  """ControlsExt with only the clutch pieces (the real __init__ blocks on CarParamsSP and loads models)."""
  def __init__(self, CP, params):  # noqa: super-init-not-called
    from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import ClutchShiftGate as Gate
    self.clutch_shift_gate = Gate(CP, params)
    self.clutch_takeover = TakeoverRamp()


class TestTakeoverRamp(OpenpilotTestCase):
  def ramp_run(self, seed_a_ego, a_target, frames, gear=4, limits=(ACCEL_MIN, ACCEL_MAX), urgent_at=None, max_ramp=1.0,
               v=V, plan_fn=None):
    """Drive TakeoverRamp + a PID-like controller (output = a_target, i.e. what LongControl converges to)."""
    ramp = TakeoverRamp()
    ramp.start(seed_a_ego, a_target, limits, gear, max_ramp)
    out = []
    for k in range(frames):
      tgt = plan_fn(k) if plan_fn else a_target
      lo, hi = ramp.limit_accel(True, v, "lead" if urgent_at is not None and k >= urgent_at else None, limits)
      cmd = float(np.clip(tgt, lo, hi))
      ramp.post_update(cmd, tgt, True)
      out.append(cmd)
    return ramp, out

  def test_seed_then_per_gear_slew(self):
    for gear in (1, 3, 6):
      ramp, out = self.ramp_run(-0.6, 0.4, 250, gear=gear, max_ramp=2.0)
      self.assertEqual(out[0], -0.6)
      rate = takeover_rate(V, gear)
      steps = np.diff(out)
      # starts at the per-gear rate, escalates smoothly toward CATCHUP_RATE only late in the window
      self.assertAlmostEqual(steps[0], (rate + (CATCHUP_RATE - rate) * DT_CTRL / 2.0) * DT_CTRL, delta=1e-9)
      self.assertLessEqual(steps[:50].max(), (rate + (CATCHUP_RATE - rate) * 50 * DT_CTRL / 2.0) * DT_CTRL + 1e-9)
      self.assertLessEqual(steps.max(), CATCHUP_RATE * DT_CTRL + 1e-9)
      self.assertEqual(ramp.reason, "converged")
      self.assertAlmostEqual(out[-1], 0.4)

  def test_lower_gears_ramp_slower(self):
    _, o1 = self.ramp_run(-0.6, 0.4, 30, gear=1, max_ramp=2.0)
    _, o6 = self.ramp_run(-0.6, 0.4, 30, gear=6, max_ramp=2.0)
    self.assertLess(o1[-1], o6[-1])

  def test_max_ramp_bounds_duration(self):
    # default window: a 1 m/s^2 gap in 1st gear is closed inside it, without a step
    ramp, out = self.ramp_run(-0.6, 0.4, 200, gear=1, max_ramp=1.0)
    self.assertLessEqual(next(k for k, c in enumerate(out) if abs(c - 0.4) < 1e-9), round(1.0 / DT_CTRL))
    self.assertLessEqual(np.abs(np.diff(out)).max(), CATCHUP_RATE * DT_CTRL + 1e-9)
    self.assertEqual(ramp.reason, "converged")
    # a gap too big for the shortest window: the ramp still ends on time (the duration bound wins) and hands the
    # residual to LongControl
    ramp, out = self.ramp_run(-1.5, 1.0, 200, gear=1, max_ramp=0.3)
    self.assertEqual(ramp.reason, "max_ramp")
    self.assertEqual(out[round(0.3 / DT_CTRL)], 1.0)

  def test_positive_seed_never_above_plan_throttle(self):
    ramp = TakeoverRamp()
    self.assertEqual(ramp.start(0.9, 0.2, (ACCEL_MIN, ACCEL_MAX), 3, 1.0), 0.2)
    self.assertEqual(ramp.start(0.9, -0.5, (ACCEL_MIN, ACCEL_MAX), 3, 1.0), 0.0)
    self.assertEqual(ramp.start(-0.7, 0.5, (ACCEL_MIN, ACCEL_MAX), 3, 1.0), -0.7)  # negative seed kept (engine braking)
    self.assertEqual(ramp.start(-9., 0., (ACCEL_MIN, ACCEL_MAX), 3, 1.0), ACCEL_MIN)

  def test_urgent_bypass_passes_full_decel(self):
    # lead suddenly urgent mid-ramp: the next command is the controller's full decel, not rate-limited
    ramp, out = self.ramp_run(0.0, -3.0, 20, urgent_at=5, plan_fn=lambda k: 0.3 if k < 5 else -3.0)
    self.assertEqual(out[5], -3.0)
    self.assertEqual(ramp.reason, "lead")

  def test_urgent_reasons(self):
    radar_ok = radar_msg()
    base = {"CS": cs(), "plan": plan_msg(), "plan_valid": True, "radar": radar_ok, "radar_valid": True, "force_decel": False}
    self.assertIsNone(takeover_urgent(**base))
    cases = {"force_decel": {"force_decel": True}, "fcw_aeb": {"plan": plan_msg(fcw=True)},
             "invalid_input": {"radar_valid": False}, "stopping": {"plan": plan_msg(should_stop=True)},
             "hard_decel": {"plan": plan_msg(a_target=-2.0)}, "lead": {"radar": radar_msg((30., -3.))}}
    for reason, override in cases.items():
      self.assertEqual(takeover_urgent(**{**base, **override}), reason, reason)
    self.assertEqual(takeover_urgent(**{**base, "CS": cs(aeb=True)}), "fcw_aeb")
    self.assertEqual(takeover_urgent(**{**base, "plan_valid": False}), "invalid_input")
    self.assertIsNone(takeover_urgent(**{**base, "radar": radar_msg((80., 0.5))}))  # far, opening lead: not urgent

  def test_never_violates_floor_or_widens_limits(self):
    # NSB floor -0.3 / EBP coast target 0.0 already in the limits: the seed and every step stay inside them
    for limits in ((-0.3, ACCEL_MAX), (0.0, ACCEL_MAX), (ACCEL_MIN, 0.1)):
      _, out = self.ramp_run(-1.0, 0.5, 120, limits=limits, plan_fn=lambda k: 0.5 if k % 40 < 20 else -1.5)
      self.assertTrue(all(limits[0] - 1e-9 <= c <= limits[1] + 1e-9 for c in out), limits)

  def test_limits_tighten_mid_ramp(self):
    ramp = TakeoverRamp()
    ramp.start(-1.0, 0.5, (ACCEL_MIN, ACCEL_MAX), 4, 1.0)
    lo, _ = ramp.limit_accel(True, V, None, (ACCEL_MIN, ACCEL_MAX))  # first frame: exactly the seed
    ramp.post_update(lo, 0.5, True)
    lo, hi = ramp.limit_accel(True, V, None, (-0.3, ACCEL_MAX))  # a floor appeared above the envelope
    self.assertEqual((lo, hi), (-0.3, -0.3))

  def test_ends_when_long_inactive(self):
    ramp = TakeoverRamp()
    ramp.start(-1.0, 0.5, (ACCEL_MIN, ACCEL_MAX), 4, 1.0)
    self.assertEqual(ramp.limit_accel(False, V, None, (ACCEL_MIN, ACCEL_MAX)), (ACCEL_MIN, ACCEL_MAX))
    self.assertFalse(ramp.active)

  def test_controlsd_wiring(self):
    setup_params()
    CP, _ = make_cp()
    ext = _Ext(CP, Params())
    loc = LongControl(*make_cp())
    sm = FakeSM(carState=cs(), longitudinalPlan=plan_msg(0.4), radarState=radar_msg(),
                driverMonitoringState=messaging.new_message('driverMonitoringState').driverMonitoringState,
                selfdriveState=messaging.new_message('selfdriveState').selfdriveState)
    cmds, active = [], []

    def frame(state, a_target=0.4):
      sm['carState'], sm['longitudinalPlan'] = state, plan_msg(a_target)
      paused = ext.get_long_paused(sm)
      long_active = not paused
      if not long_active:
        loc.reset()
      limits = ext.apply_clutch_takeover(sm, long_active, (ACCEL_MIN, ACCEL_MAX))
      cmds.append(float(loc.update(long_active, state, a_target, False, limits)))
      ext.post_clutch_takeover(loc)
      active.append(long_active)

    for _ in range(100):
      frame(cs(gear=4))
    for _ in range(50):
      frame(cs(clutch=True, rpm=1500.))
    for _ in range(20):
      frame(cs(gear=5, a=-0.5))
    resume = active.index(True, 100)
    self.assertAlmostEqual(cmds[resume], -0.5)
    self.assertLess(max(abs(np.diff(cmds[resume:]))), 0.02)
    self.assertTrue(ext.clutch_takeover.active)

  def test_controlsd_wiring_urgent_bypass(self):
    # mid-ramp, the controlsd wiring feeds the urgent check from radarState / driver monitoring / validity
    def setup_cases():
      dm = messaging.new_message('driverMonitoringState').driverMonitoringState
      dm.noResponseForceDecel = True
      return {"lead": ({'radarState': radar_msg((25., -4.))}, {}), "force_decel": ({'driverMonitoringState': dm}, {}),
              "invalid_input": ({}, {'radarState': False})}
    for reason, (msgs, valid) in setup_cases().items():
      setup_params()
      CP, _ = make_cp()
      ext = _Ext(CP, Params())
      sm = FakeSM(carState=cs(), longitudinalPlan=plan_msg(0.4), radarState=radar_msg(),
                  driverMonitoringState=messaging.new_message('driverMonitoringState').driverMonitoringState,
                  selfdriveState=messaging.new_message('selfdriveState').selfdriveState)
      for state in [cs(clutch=True, rpm=1500.)] * 50 + [cs(gear=5, a=-0.5)] * 5:
        sm['carState'] = state
        paused = ext.get_long_paused(sm)
        ext.apply_clutch_takeover(sm, not paused, (ACCEL_MIN, ACCEL_MAX))
      self.assertTrue(ext.clutch_takeover.active)
      sm.update(msgs)
      sm.valid.update(valid)
      ext.get_long_paused(sm)
      self.assertEqual(ext.apply_clutch_takeover(sm, True, (ACCEL_MIN, ACCEL_MAX)), (ACCEL_MIN, ACCEL_MAX), reason)
      self.assertEqual(ext.clutch_takeover.reason, reason)

  def test_controlsd_wiring_smart_off_unchanged(self):
    # smart OFF: no takeover is ever started, limits pass through untouched (outside a coast timeout)
    setup_params(smart=False)
    CP, _ = make_cp()
    ext = _Ext(CP, Params())
    sm = FakeSM(carState=cs(), longitudinalPlan=plan_msg(), radarState=radar_msg(),
                driverMonitoringState=messaging.new_message('driverMonitoringState').driverMonitoringState,
                selfdriveState=messaging.new_message('selfdriveState').selfdriveState)
    for state in [cs(clutch=True)] * 50 + [cs(gear=5)] * 80:
      sm['carState'] = state
      paused = ext.get_long_paused(sm)
      self.assertEqual(ext.apply_clutch_takeover(sm, not paused, (ACCEL_MIN, ACCEL_MAX)), (ACCEL_MIN, ACCEL_MAX))
      self.assertFalse(ext.clutch_takeover.active)


# --- closed loop -------------------------------------------------------------------------------------------------------
ACTUATOR_TAU = 0.25
THROTTLE_FADE = 0.05
IDLE_RPM = 750.


class ShiftSim:
  """Manual car through real shifts. Clutch in: no engine (rolling + aero + grade), rpm falls toward idle (or a blip).
  Clutch out: GEAR_MT reports the gear at once, the clutch slips for `slip` s (rpm converges to the gear's, little
  torque), then it's locked: closed throttle -> engine braking, command > 0 opens the throttle, command < 0 = brakes on
  top. Driver throttle (gas=True) = +1.0 m/s^2 and longitudinal overridden (gasPressed)."""
  def __init__(self, speed=20., gear=4, v_set=None, non_adaptive=True, nsb=False, ebp=False, grade=0., slip=0.3, **params):
    setup_params(NonAdaptiveLongitudinal=non_adaptive, NoServiceBrakeHold=nsb, EngineBrakePreferred=ebp, **params)
    self.CP, self.CP_SP = make_cp()
    self.params = Params()
    self.planner = LongitudinalPlanner(self.CP, self.CP_SP, init_v=speed)
    self.loc = LongControl(self.CP, self.CP_SP)
    self.loc.long_control_state = LongCtrlState.pid
    self.gate = ClutchShiftGate(self.CP, self.params)
    self.takeover = TakeoverRamp()
    self.ebp = EngineBrakePreferred(self.CP, self.params)
    self.nsb_enabled, self.nsb_floor = nsb, read_decel_floor(self.params)
    self.speed, self.a_ego, self.gear, self.grade, self.slip = speed, 0., gear, grade, slip
    self.v_set = speed if v_set is None else v_set
    self.clutch, self.gas, self.blip, self.slip_t, self.rpm = False, False, False, slip, engine_rpm_at(gear, speed)
    self.long_active = True
    self.t = 0.
    self.log: list[dict] = []

  def coupled(self):
    return not self.clutch and self.slip_t >= self.slip

  def set(self, clutch=None, gear=None, gas=None, blip=None):
    if clutch is False and self.clutch:
      self.slip_t, self.rpm_bite = 0., self.rpm
    if clutch is not None:
      self.clutch = clutch
    if gear is not None:
      self.gear = gear
    if gas is not None:
      self.gas = gas
    if blip is not None:
      self.blip = blip

  def car_state(self):
    CS = messaging.new_message('carState').carState
    CS.vEgo, CS.aEgo, CS.gasPressed = float(self.speed), float(self.a_ego), bool(self.gas)
    CS.vCruise = self.v_set * 3.6
    CS.deprecated.clutchPressed = self.clutch
    CS.deprecated.engineRpm = float(self.rpm)
    return CS

  def planner_sm(self):
    control = messaging.new_message('controlsState').controlsState
    control.longControlState = self.loc.long_control_state
    cc = messaging.new_message('carControl').carControl
    cc.longActive = self.long_active
    cc.orientationNED = [0., float(self.grade), 0.]
    model = messaging.new_message('modelV2').modelV2
    model.position.x = [float(x) for x in (self.speed + 0.5) * np.array(ModelConstants.T_IDXS)]
    model.velocity.x = [float(self.speed)] * len(ModelConstants.T_IDXS)
    model.acceleration.x = [0.] * len(ModelConstants.T_IDXS)
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    sm = {'radarState': messaging.new_message('radarState').radarState, 'carState': self.car_state(),
          'controlsState': control, 'carControl': cc, 'modelV2': model}
    for s in ('selfdriveState', 'vehicleParameters', 'carStateSP', 'liveMapDataSP', 'gpsLocation'):
      sm[s] = getattr(messaging.new_message(s), s)
    return sm

  def plan_msg(self):
    plan = messaging.new_message('longitudinalPlan').longitudinalPlan
    plan.aTarget = float(self.planner.output_a_target)
    plan.shouldStop = bool(self.planner.output_should_stop)
    plan.fcw = bool(self.planner.fcw)
    plan.accels = [float(a) for a in self.planner.a_desired_trajectory]
    plan.allowBrake = bool(self.planner.allow_brake)
    return plan

  def car_accel(self, cmd):
    a_road = resistive_decel(self.speed) - G * math.sin(self.grade)
    brake = min(cmd, 0.) if self.long_active else 0.
    if not self.coupled():
      return a_road + brake
    if self.gas:
      return 1.0 + a_road
    a_eb = estimate_coast_decel(self.gear, self.rpm, self.speed, self.grade)
    if not self.long_active:
      return a_eb
    fade = float(np.clip(1. - max(cmd, 0.) / THROTTLE_FADE, 0., 1.))
    return cmd + a_eb * fade

  def update_rpm(self):
    target = engine_rpm_at(self.gear, self.speed)
    if self.clutch:
      self.rpm = engine_rpm_at(self.gear, self.speed) * 1.05 if self.blip else max(self.rpm - 1500. * DT_CTRL, IDLE_RPM)
    elif self.slip_t < self.slip:
      self.slip_t += DT_CTRL
      self.rpm = self.rpm_bite + (target - self.rpm_bite) * min(self.slip_t / self.slip, 1.)
    else:
      self.rpm = target

  def run(self, seconds, **actions):
    if actions:
      self.set(**actions)
    radar = messaging.new_message('radarState').radarState
    for _ in range(round(seconds / DT_MDL)):
      self.planner.update(self.planner_sm())
      plan = self.plan_msg()
      for _ in range(round(DT_MDL / DT_CTRL)):
        self.update_rpm()
        CS = self.car_state()
        paused = self.gate.update(CS)
        self.long_active = not paused and not self.gas
        if not self.long_active:
          self.loc.reset()
        limits = apply_brake_floor((ACCEL_MIN, ACCEL_MAX), plan.allowBrake, True, self.nsb_enabled, self.nsb_floor)
        self.ebp.update(self.long_active, CS, plan, True, radar, True, False, self.loc.long_control_state, self.grade)
        limits = self.ebp.limit_accel(limits)
        limits = self.gate.limit_accel(limits)
        if self.gate.resumed and self.gate.reengage.smart and self.long_active:
          self.takeover.start(CS.aEgo, plan.aTarget, limits, self.gate.ratio_lock.gear, self.gate.reengage.max_ramp)
        if self.takeover.active:
          urgent = takeover_urgent(CS, plan, True, radar, True, False)
          limits = self.takeover.limit_accel(self.long_active, CS.vEgo, urgent, limits)
        cmd = float(self.loc.update(self.long_active, CS, plan.aTarget, plan.shouldStop, limits))
        pid = self.loc.pid
        self.takeover.post_update(cmd, pid.p + pid.i + pid.d + pid.f, self.loc.long_control_state == LongCtrlState.pid)
        self.a_ego += (self.car_accel(cmd) - self.a_ego) * DT_CTRL / ACTUATOR_TAU
        self.speed = max(self.speed + self.a_ego * DT_CTRL, 0.)
        self.log.append({"t": self.t, "cmd": cmd, "a_ego": self.a_ego, "active": self.long_active, "v": self.speed,
                             "coupled": self.coupled(), "limits": limits, "ramp": self.takeover.active})
        self.t += DT_CTRL
    return self

  def resume_index(self, after_t):
    return next(k for k, r in enumerate(self.log) if r['t'] >= after_t and r['active'] and not self.log[k - 1]['active'])


def downshift(sim):
  """4th -> 3rd while holding set speed (driver downshifts for engine braking), 0.6 s clutch with a blip."""
  sim.run(3.0)
  sim.run(0.25, clutch=True)
  sim.run(0.15, blip=True)
  sim.run(0.2, blip=False)
  sim.run(4.0, clutch=False, gear=3)
  return sim


class TestClosedLoop(OpenpilotTestCase):
  def test_downshift_into_engine_braking(self):
    # the command at resume starts at the car's actual (engine-braking) accel; OFF starts from the plan with LoC from 0
    on, off = downshift(ShiftSim(smart=True)), downshift(ShiftSim(smart=False))
    i_on, i_off = on.resume_index(3.0), off.resume_index(3.0)
    r_on, r_off = on.log[i_on], off.log[i_off]
    a_on, a_off = on.log[i_on - 1]['a_ego'], off.log[i_off - 1]['a_ego']  # the aEgo controlsd saw that frame
    self.assertAlmostEqual(r_on['cmd'], a_on, delta=1e-6)
    self.assertLess(r_on['cmd'], -0.1)  # no throttle forced on to cancel the engine braking
    # takeover at the bite (ratio lock), not after the fixed delay from GEAR_MT (clutch out at 3.6 s, 0.3 s slip)
    bite = next(k for k, r in enumerate(on.log) if r['t'] >= 3.6 and r['coupled'])
    self.assertLessEqual(i_on - bite, RATIO_LOCK_FRAMES)
    self.assertAlmostEqual(off.log[i_off]['t'], 3.6 + REENGAGE_DELAY, delta=0.02)
    self.assertLessEqual(abs(r_on['cmd'] - a_on), abs(r_off['cmd'] - a_off))

  def test_ramp_hands_over_without_a_step(self):
    # while the ramp runs the command moves no faster than it allows, and it ends by converging onto LongControl's own
    # output (no step at the hand-over); afterwards it's the stock controller
    for kw in ({}, {"speed": 25., "v_set": 27.}, {"nsb": True}):
      sim = downshift(ShiftSim(smart=True, **kw))
      i = sim.resume_index(3.0)
      end = next(k for k in range(i, len(sim.log)) if not sim.log[k]['ramp'])
      cmds = [r['cmd'] for r in sim.log[i:end + 1]]
      self.assertLessEqual(np.abs(np.diff(cmds)).max(), CATCHUP_RATE * DT_CTRL + 1e-9, kw)
      self.assertEqual(sim.takeover.reason, "converged", kw)

  def test_on_throttle_upshift(self):
    # driver on the gas through 3rd -> 4th, keeps accelerating, then rolls off: takeover only after the lift + debounce
    sim = ShiftSim(smart=True, speed=15., gear=3, v_set=25.)
    sim.run(2.0, gas=True)
    sim.run(0.5, clutch=True, gas=False)
    sim.run(1.5, clutch=False, gear=4, gas=True)
    self.assertFalse(any(r['active'] for r in sim.log[-100:]))
    sim.run(2.0, gas=False)
    lift = next(k for k, r in enumerate(sim.log) if r['t'] >= 4.0)
    i = sim.resume_index(4.0)
    self.assertAlmostEqual(sim.log[i]['t'] - sim.log[lift]['t'], DEFAULT_GAS_DEBOUNCE, delta=0.03)
    self.assertLessEqual(sim.log[i]['cmd'], max(sim.log[i]['a_ego'], 0.) + 1e-6)

  def test_off_throttle_upshift_resumes_at_bite(self):
    sim = ShiftSim(smart=True, speed=15., gear=3, v_set=25.)
    sim.run(2.0)
    sim.run(0.5, clutch=True)
    sim.run(2.0, clutch=False, gear=4)
    i = sim.resume_index(2.5)
    bite = next(k for k, r in enumerate(sim.log) if r['t'] >= 2.5 and r['coupled'])
    # here the engine had already fallen to within ~6% of 4th's rpm at clutch-out, so the slow, near-synced slip reads
    # as locked a little before it completes (TODO(on-car) in clutch_reengage.RATIO_LOCK_STEADY); never after the bite,
    # and never before GEAR_MT reports the gear
    self.assertLessEqual(i, bite + RATIO_LOCK_FRAMES)
    self.assertGreaterEqual(sim.log[i]['t'], 2.5 + (RATIO_LOCK_FRAMES - 1) * DT_CTRL - 1e-9)
    self.assertLess(sim.log[i]['t'], 2.5 + REENGAGE_DELAY)

  def test_ebp_and_nsb_floors_hold_through_takeover(self):
    for kw in ({"ebp": True}, {"nsb": True}, {"ebp": True, "nsb": True}):
      sim = downshift(ShiftSim(smart=True, **kw))
      i = sim.resume_index(3.0)
      for r in sim.log[i:i + 150]:
        self.assertGreaterEqual(r['cmd'], r['limits'][0] - 1e-9, kw)
      if kw.get('nsb'):
        self.assertTrue(all(r['cmd'] >= -0.3 - 1e-9 for r in sim.log[i:]), kw)

  def test_smart_off_matches_fixed_delay(self):
    sim = downshift(ShiftSim(smart=False))
    i = sim.resume_index(3.0)
    clutch_out = next(k for k, r in enumerate(sim.log) if r['t'] >= 3.6 - 1e-9)
    self.assertAlmostEqual((i - clutch_out) * DT_CTRL, REENGAGE_DELAY, delta=0.02)
    self.assertFalse(any(r['ramp'] for r in sim.log))

  def test_neutral_coast_timeout_closed_loop(self):
    # coasting downhill in neutral, engaged: after the timeout openpilot may brake again but never adds throttle
    sim = ShiftSim(smart=True, speed=20., v_set=20., grade=-0.04)
    sim.run(1.0)
    sim.run(DEFAULT_MAX_COAST + 4.0, clutch=True)
    back = [r for r in sim.log if r['t'] > 1.0 + DEFAULT_MAX_COAST + 0.05]
    self.assertTrue(all(r['active'] for r in back))
    self.assertTrue(all(r['cmd'] <= 0. for r in back))
    self.assertLess(min(r['cmd'] for r in back), -0.1)  # braking against the overspeed downhill
