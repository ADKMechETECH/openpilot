"""
No-service-brake overspeed backstop, pulse mode (Task 11): unit tests of the plannerd state machine and controlsd's
PulseFloor, the maintain mode pinned to the pre-Task-11 backstop, and closed loop through the real wiring:
LongitudinalPlanner -> published longitudinalPlan + longitudinalPlanSP (serialized) -> ControlsExt.apply_no_service_brake
-> LongControl -> a vehicle on a downhill.
"""
import math
import random

import numpy as np

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.cereal import custom, log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_planner import J_CRUISE_VALS
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import NoServiceBrake, PulseFloor, PulseParams, \
  apply_brake_floor, overspeed_urgent, read_decel_floor, read_overspeed_mode, read_pulse_params, BACKSTOP_RELEASE_RATIO, \
  DEFAULT_HARD_CEILING, DEFAULT_OVERSPEED_MARGIN, DEFAULT_PULSE_DECEL, DEFAULT_PULSE_JERK, DEFAULT_RELEASE_MARGIN, \
  HARD_CEILING_RANGE, PULSE_DECEL_RANGE, PULSE_JERK_RANGE, PULSE_MIN_BAND, RELEASE_MARGIN_RANGE
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_no_service_brake import DownhillSim, V_SET
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
from openpilot.selfdrive.selfdrived.events import ET

EventNameSP = custom.OnroadEventSP.EventName
KEYS = ("NonAdaptiveLongitudinal", "NoServiceBrakeHold", "NoServiceBrakeDecelFloor", "NoServiceBrakeOverspeedBackstop",
        "NoServiceBrakeOverspeedMargin", "NoServiceBrakeOverspeedMode", "NoServiceBrakeOverspeedReleaseMargin",
        "NoServiceBrakeOverspeedPulseDecel", "NoServiceBrakeOverspeedPulseJerk", "NoServiceBrakeOverspeedHardCeiling")
EPS = 1e-4


def setup(mode="pulse", non_adaptive=True, nsb=True, **params):
  p = Params()
  for k in KEYS:
    p.remove(k)
  p.put_bool("NonAdaptiveLongitudinal", non_adaptive, block=True)
  p.put_bool("NoServiceBrakeHold", nsb, block=True)
  if mode is not None:
    p.put("NoServiceBrakeOverspeedMode", mode, block=True)
  for k, v in params.items():
    if isinstance(v, bool):
      p.put_bool(k, v, block=True)
    else:
      p.put(k, v, block=True)
  return p


class FakeCP:
  openpilotLongitudinalControl = True
  carFingerprint = "HONDA_CIVIC_2022"


class LegacyNoServiceBrake:
  """The pre-Task-11 backstop (no_service_brake.py at 971148479), verbatim minus params/logging: maintain must match."""
  def __init__(self, enabled, margin=DEFAULT_OVERSPEED_MARGIN, backstop=True):
    self.enabled, self.margin, self.backstop_enabled, self.backstop_active = enabled, margin, backstop, False

  def update(self, non_adaptive, v_ego, v_set, force_decel):
    clamp_wanted = self.enabled and non_adaptive and not force_decel
    if not clamp_wanted:
      self.backstop_active = False
    elif self.backstop_enabled:
      if v_ego > v_set + self.margin:
        self.backstop_active = True
      elif v_ego < v_set + self.margin * BACKSTOP_RELEASE_RATIO:
        self.backstop_active = False
    else:
      self.backstop_active = False
    return not clamp_wanted or self.backstop_active


def lead(d_rel, v_rel, a_lead=0.):
  l = log.RadarState.LeadData.new_message()
  l.present, l.dRel, l.vRel, l.aLeadK = True, float(d_rel), float(v_rel), float(a_lead)
  return l


def radar(lead_one=None):
  r = messaging.new_message('radarState').radarState
  if lead_one is not None:
    r.leadOne = lead_one
  return r


def car_state(v=25., stock_aeb=False, stock_fcw=False):
  cs = messaging.new_message('carState').carState
  cs.vEgo, cs.stockAeb, cs.stockFcw = float(v), stock_aeb, stock_fcw
  return cs


class FakeSM(dict):
  """plannerd/controlsd SubMaster stand-in: valid, logMonoTime, all_checks."""
  def __init__(self, *a, valid=True, **kw):
    super().__init__(*a, **kw)
    self.valid = dict.fromkeys(('longitudinalPlan', 'longitudinalPlanSP', 'radarState'), valid)
    self.logMonoTime = {'modelV2': 0}

  def all_checks(self, service_list=None):
    return True


class FakePM:
  def __init__(self):
    self.msgs = {}

  def send(self, name, msg):
    # over the wire, like controlsd would see it
    self.msgs[name] = getattr(messaging.log_from_bytes(msg.to_bytes(), log.Event), name)


class _Ext(ControlsExt):
  """ControlsExt with only the no-service-brake pieces (the real __init__ blocks on CarParamsSP and loads models)."""
  def __init__(self, params):  # noqa: super-init-not-called
    from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import PulseFloor as PF
    self.no_service_brake_enabled = params.get_bool("NoServiceBrakeHold")
    self.no_service_brake_floor = read_decel_floor(params)
    self.no_service_brake_pulse = PF()


class PulseSim(DownhillSim):
  """DownhillSim through the real wiring: planner publish -> serialized plan + plan SP -> ControlsExt -> LongControl."""
  def __init__(self, *a, lead_gap=None, **kw):
    super().__init__(*a, **kw)
    self.ext = _Ext(self.params)
    self.lead_gap = lead_gap  # a lead held at this gap at our speed (close, not closing)
    self.events = []

  def _sm(self):
    sm = FakeSM(super()._sm())
    if self.lead_gap is not None:
      l = lead(self.lead_gap, 0.)
      l.vLead = l.vLeadK = float(self.speed)
      l.modelProb, l.radar = 1.0, True
      sm['radarState'] = radar(l)
    return sm

  def run(self, seconds):
    for _ in range(round(seconds / DT_MDL)):
      sm = self._sm()
      self.planner.update(sm)
      self.events.append(list(self.planner.events_sp.names))
      pm = FakePM()
      self.planner.publish(sm, pm)
      csm = FakeSM({'longitudinalPlan': pm.msgs['longitudinalPlan'], 'longitudinalPlanSP': pm.msgs['longitudinalPlanSP']})
      nsb = self.planner.no_service_brake
      for _ in range(round(DT_MDL / DT_CTRL)):
        limits = self.ext.apply_no_service_brake(csm, (ACCEL_MIN, ACCEL_MAX), True, self.loc.last_output_accel)
        CS = messaging.new_message('carState').carState
        CS.vEgo, CS.aEgo = float(self.speed), float(self.a_ego)
        cmd = float(self.loc.update(True, CS, self.planner.output_a_target, self.planner.output_should_stop, limits))
        self.a_ego = cmd + self.downhill
        self.speed = max(self.speed + self.a_ego * DT_CTRL, 0.)
        self.distance += self.speed * DT_CTRL
        self.log.append({"cmd": cmd, "speed": self.speed, "over": self.speed - V_SET, "allow_brake": self.planner.allow_brake,
                         "state": nsb.state, "phase": nsb.phase, "lower_limit": limits[0]})
    return self.log


def pulses(log):
  """[(start, end)] index ranges of the PULSING state."""
  out, start = [], None
  for i, l in enumerate(log):
    if l["state"] == "pulsing" and start is None:
      start = i
    elif l["state"] != "pulsing" and start is not None:
      out.append((start, i))
      start = None
  return out


class TestPulseParams(OpenpilotTestCase):
  def test_defaults(self):
    p = setup(mode=None)
    self.assertEqual(read_overspeed_mode(p), "maintain")
    pp = read_pulse_params(p, DEFAULT_OVERSPEED_MARGIN)
    self.assertEqual(pp, PulseParams())
    self.assertEqual((pp.trigger, pp.release, pp.decel, pp.jerk, pp.ceiling), (3.6, 0.9, -1.0, 1.5, 6.7))
    nsb = NoServiceBrake(FakeCP(), p)
    self.assertEqual(nsb.mode, "maintain")
    self.assertEqual(nsb.state, "off")

  def test_mode_values(self):
    p = setup(mode="pulse")
    self.assertEqual(read_overspeed_mode(p), "pulse")
    for bad in ("", "Pulsed", "maintainx", "brake"):
      p.put("NoServiceBrakeOverspeedMode", bad, block=True)
      self.assertEqual(read_overspeed_mode(p), "maintain", bad)
    p.put("NoServiceBrakeOverspeedMode", " PULSE ", block=True)
    self.assertEqual(read_overspeed_mode(p), "pulse")

  def test_params_clamped(self):
    p = setup(NoServiceBrakeOverspeedReleaseMargin=99., NoServiceBrakeOverspeedPulseDecel=-99.,
              NoServiceBrakeOverspeedPulseJerk=99., NoServiceBrakeOverspeedHardCeiling=99.)
    pp = read_pulse_params(p, DEFAULT_OVERSPEED_MARGIN)
    self.assertEqual(pp.decel, PULSE_DECEL_RANGE[0])
    self.assertEqual(pp.jerk, PULSE_JERK_RANGE[1])
    self.assertEqual(pp.ceiling, HARD_CEILING_RANGE[1])
    self.assertAlmostEqual(pp.release, DEFAULT_OVERSPEED_MARGIN - PULSE_MIN_BAND, msg="release must stay under the trigger")
    p = setup(NoServiceBrakeOverspeedReleaseMargin=-5., NoServiceBrakeOverspeedPulseDecel=5.,
              NoServiceBrakeOverspeedPulseJerk=0., NoServiceBrakeOverspeedHardCeiling=0.)
    pp = read_pulse_params(p, DEFAULT_OVERSPEED_MARGIN)
    self.assertEqual((pp.release, pp.decel, pp.jerk, pp.ceiling),
                     (RELEASE_MARGIN_RANGE[0], PULSE_DECEL_RANGE[1], PULSE_JERK_RANGE[0], HARD_CEILING_RANGE[0]))
    for k in ("NoServiceBrakeOverspeedReleaseMargin", "NoServiceBrakeOverspeedPulseDecel", "NoServiceBrakeOverspeedPulseJerk",
              "NoServiceBrakeOverspeedHardCeiling"):
      p.put(k, float("nan"), block=True)
    self.assertEqual(read_pulse_params(p, DEFAULT_OVERSPEED_MARGIN), PulseParams())

  def test_trigger_tightened_below_ceiling(self):
    # a margin (trigger) at/above the ceiling would escalate on every pulse: the trigger comes down (earlier braking)
    p = setup(NoServiceBrakeOverspeedHardCeiling=5.)
    pp = read_pulse_params(p, 12.)
    self.assertAlmostEqual(pp.trigger, 5. - PULSE_MIN_BAND)
    self.assertLess(pp.release, pp.trigger)
    self.assertTrue(0 <= pp.release < pp.trigger < pp.ceiling)


class TestPulseStateMachine(OpenpilotTestCase):
  def make(self, **params):
    p = setup(**params)
    nsb = NoServiceBrake(FakeCP(), p)
    nsb.update(True, True, V_SET, V_SET, False)  # latch while long inactive
    return nsb

  def step(self, nsb, over, urgent=None, force_decel=False, long_inactive=False, non_adaptive=True):
    return nsb.update(long_inactive, non_adaptive, V_SET + over, V_SET, force_decel, urgent)

  def test_coast_pulse_coast(self):
    nsb = self.make()
    self.assertFalse(self.step(nsb, 0.))
    self.assertEqual(nsb.state, "coasting")
    self.assertAlmostEqual(nsb.accel_floor, -0.3)
    self.assertFalse(self.step(nsb, DEFAULT_OVERSPEED_MARGIN - 1e-6), "pulse starts only ABOVE set + trigger")
    self.assertTrue(self.step(nsb, DEFAULT_OVERSPEED_MARGIN + 0.01))
    self.assertEqual(nsb.state, "pulsing")
    self.assertAlmostEqual(nsb.accel_floor, DEFAULT_PULSE_DECEL)
    self.assertTrue(self.step(nsb, 2.0))
    self.assertTrue(self.step(nsb, DEFAULT_RELEASE_MARGIN + 0.01))
    self.assertEqual(nsb.state, "pulsing")
    self.assertFalse(self.step(nsb, DEFAULT_RELEASE_MARGIN - 1e-6))
    self.assertEqual(nsb.state, "coasting")

  def test_hysteresis_no_chatter(self):
    nsb = self.make()
    changes, prev = 0, nsb.state
    rng = random.Random(0)
    # noise inside the band never changes the state, whichever state it starts in
    for start in (0., DEFAULT_OVERSPEED_MARGIN + 0.1):
      self.step(nsb, start)
      prev = nsb.state
      for _ in range(500):
        self.step(nsb, rng.uniform(DEFAULT_RELEASE_MARGIN + 0.05, DEFAULT_OVERSPEED_MARGIN - 0.05))
        changes += nsb.state != prev
        prev = nsb.state
    self.assertEqual(changes, 0)

  def test_escalation(self):
    nsb = self.make()
    self.step(nsb, DEFAULT_OVERSPEED_MARGIN + 0.1)
    events = []
    for over in np.linspace(DEFAULT_OVERSPEED_MARGIN, DEFAULT_HARD_CEILING + 1., 40):
      self.assertTrue(self.step(nsb, over))
      events.append(nsb.escalation_event)
      if over <= DEFAULT_HARD_CEILING:
        self.assertEqual(nsb.state, "pulsing")
    self.assertEqual(nsb.state, "escalated")
    self.assertAlmostEqual(nsb.accel_floor, 0.)
    self.assertEqual(sum(events), 1, "one alert per escalation")
    # stays escalated (full braking) back down through the band until under set + release
    for over in np.linspace(DEFAULT_HARD_CEILING, DEFAULT_RELEASE_MARGIN + 0.01, 40):
      self.assertTrue(self.step(nsb, over))
      self.assertEqual(nsb.state, "escalated")
      self.assertFalse(nsb.escalation_event)
    self.assertFalse(self.step(nsb, DEFAULT_RELEASE_MARGIN - 0.01))
    self.assertEqual(nsb.state, "coasting")

  def test_escalation_from_coasting_jump(self):
    # already past the ceiling when the pulse would start (e.g. set speed lowered a lot): pulse -> escalate at once
    nsb = self.make()
    self.assertTrue(self.step(nsb, DEFAULT_HARD_CEILING + 0.5))
    self.assertEqual(nsb.state, "escalated")
    self.assertTrue(nsb.escalation_event)

  def test_disengage_ends_escalation(self):
    nsb = self.make()
    self.step(nsb, DEFAULT_OVERSPEED_MARGIN + 0.1)
    self.step(nsb, DEFAULT_HARD_CEILING + 0.1)
    self.assertEqual(nsb.state, "escalated")
    self.step(nsb, DEFAULT_HARD_CEILING + 0.1, long_inactive=True)
    self.assertEqual(nsb.phase, "coasting")
    # re-engaged that far over: pulses first, escalates again, alerts again
    self.step(nsb, DEFAULT_HARD_CEILING + 0.1)
    self.assertEqual(nsb.state, "escalated")
    self.assertTrue(nsb.escalation_event)

  def test_urgent_bypass(self):
    nsb = self.make()
    for over, phase in ((0., "coasting"), (DEFAULT_OVERSPEED_MARGIN + 0.1, "pulsing")):
      self.assertTrue(self.step(nsb, over, urgent="lead"), "urgent always allows full braking")
      self.assertEqual(nsb.state, "urgent")
      self.assertEqual(nsb.phase, phase, "the overspeed state keeps tracking underneath")
      self.assertAlmostEqual(nsb.accel_floor, 0.)
    self.assertTrue(self.step(nsb, 1.))
    self.assertEqual(nsb.state, "pulsing")

  def test_force_decel_never_limited(self):
    nsb = self.make()
    for over in (0., 2., DEFAULT_OVERSPEED_MARGIN + 0.1, 5.):
      self.assertTrue(self.step(nsb, over, force_decel=True))
      self.assertEqual(nsb.state, "off")

  def test_inert_without_non_adaptive_or_toggle(self):
    nsb = self.make()
    self.assertTrue(self.step(nsb, 5., non_adaptive=False))
    self.assertEqual(nsb.state, "off")
    nsb = self.make(nsb=False)
    self.assertTrue(self.step(nsb, 5.))
    self.assertEqual(nsb.state, "off")

  def test_backstop_off_coasts(self):
    # the mode only shapes the backstop: with the backstop OFF there is no overspeed braking at all (as maintain)
    nsb = self.make(NoServiceBrakeOverspeedBackstop=False)
    for over in (0., 5., 9., 12.):
      self.assertFalse(self.step(nsb, over))
      self.assertEqual(nsb.state, "coasting")

  def test_mode_latches_only_while_inactive(self):
    p = setup(mode="maintain")
    nsb = NoServiceBrake(FakeCP(), p)
    nsb.update(True, True, V_SET, V_SET, False)
    p.put("NoServiceBrakeOverspeedMode", "pulse", block=True)
    for _ in range(50):
      nsb.update(False, True, V_SET, V_SET, False)
    self.assertEqual(nsb.mode, "maintain")
    nsb.update(True, True, V_SET, V_SET, False)
    self.assertEqual(nsb.mode, "pulse")

  def test_overspeed_urgent(self):
    self.assertIsNone(overspeed_urgent(car_state(), False, radar()))
    self.assertEqual(overspeed_urgent(car_state(), True, radar()), "fcw_aeb")
    self.assertEqual(overspeed_urgent(car_state(stock_aeb=True), False, radar()), "fcw_aeb")
    self.assertEqual(overspeed_urgent(car_state(stock_fcw=True), False, radar()), "fcw_aeb")
    self.assertEqual(overspeed_urgent(car_state(), False, radar(lead(80., -2.))), "lead")    # closing
    self.assertEqual(overspeed_urgent(car_state(), False, radar(lead(15., 0.))), "lead")    # close
    self.assertIsNone(overspeed_urgent(car_state(), False, radar(lead(80., 1.))))           # pulling away


class TestMaintainUnchanged(OpenpilotTestCase):
  def test_maintain_matches_legacy_state_machine(self):
    rng = random.Random(1)
    for backstop in (True, False):
      setup(mode="maintain", NoServiceBrakeOverspeedBackstop=backstop)
      nsb = NoServiceBrake(FakeCP(), Params())
      nsb.update(True, True, V_SET, V_SET, False)
      ref = LegacyNoServiceBrake(True, backstop=backstop)
      v = V_SET
      for _ in range(3000):
        v = float(np.clip(v + rng.uniform(-0.4, 0.4), V_SET - 3, V_SET + 9))
        non_adaptive, force_decel = rng.random() > 0.05, rng.random() < 0.03
        urgent = rng.choice((None, None, "lead", "fcw_aeb"))
        self.assertEqual(nsb.update(False, non_adaptive, v, V_SET, force_decel, urgent), ref.update(non_adaptive, v, V_SET, force_decel))
        self.assertEqual(nsb.state, "off")

  def test_maintain_closed_loop_identical(self):
    # the real wiring in maintain mode == Task 4's direct apply_brake_floor sim, frame for frame
    for downhill in (0.6, 1.2):
      setup(mode="maintain")
      ref = DownhillSim(downhill=downhill).run(60.)
      setup(mode="maintain")
      sim = PulseSim(downhill=downhill)
      out = sim.run(60.)
      self.assertEqual([l["cmd"] for l in out], [l["cmd"] for l in ref])
      self.assertTrue(all(l["state"] == "off" for l in out))

  def test_maintain_controlsd_path_is_legacy(self):
    ext = _Ext(setup(mode="maintain"))
    sp = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP  # noServiceBrake unset: state off
    for allow in (True, False):
      plan = messaging.new_message('longitudinalPlan').longitudinalPlan
      plan.allowBrake = allow
      for valid in (True, False):
        sm = FakeSM({'longitudinalPlan': plan, 'longitudinalPlanSP': sp}, valid=valid)
        self.assertEqual(ext.apply_no_service_brake(sm, (-3.5, 2.0), True, -1.0),
                         apply_brake_floor((-3.5, 2.0), allow, valid, True, 0.3))


class TestPulseFloor(OpenpilotTestCase):
  def nsb_msg(self, state, floor, rate=DEFAULT_PULSE_JERK):
    m = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP.noServiceBrake
    m.state, m.accelFloor, m.rampRate = state, floor, rate
    return m

  def test_ramps_at_jerk_both_ways(self):
    pf = PulseFloor()
    for _ in range(30):  # engaged at 0: settles on the coast floor first
      pf.apply((ACCEL_MIN, ACCEL_MAX), True, True, True, self.nsb_msg("coasting", -0.3), 0., DT_CTRL)
    out = [pf.apply((ACCEL_MIN, ACCEL_MAX), True, True, True, self.nsb_msg("coasting", -0.3), -0.3, DT_CTRL)[0]]
    self.assertAlmostEqual(out[0], -0.3)
    for _ in range(100):
      out.append(pf.apply((ACCEL_MIN, ACCEL_MAX), True, True, True, self.nsb_msg("pulsing", -1.0), -0.3, DT_CTRL)[0])
    for _ in range(100):
      out.append(pf.apply((ACCEL_MIN, ACCEL_MAX), True, True, True, self.nsb_msg("coasting", -0.3), -1.0, DT_CTRL)[0])
    steps = np.abs(np.diff(out))
    self.assertLessEqual(steps.max(), DEFAULT_PULSE_JERK * DT_CTRL + 1e-9)
    self.assertAlmostEqual(min(out), -1.0)
    self.assertAlmostEqual(out[-1], -0.3)
    # 0.7 m/s^2 at 1.5 m/s^3 takes ~0.47 s each way
    self.assertEqual(sum(1 for x in out[1:101] if x > -1.0 + 1e-6), math.ceil(0.7 / (DEFAULT_PULSE_JERK * DT_CTRL)) - 1)

  def test_seed(self):
    pf = PulseFloor()
    # from full braking (escalated/urgent): rolls off from the last command
    self.assertAlmostEqual(pf.apply((ACCEL_MIN, ACCEL_MAX), True, True, True, self.nsb_msg("coasting", -0.3), -2.0, DT_CTRL)[0],
                           -2.0 + DEFAULT_PULSE_JERK * DT_CTRL)
    pf = PulseFloor()
    # from throttle / engage: never starts above 0
    self.assertAlmostEqual(pf.apply((ACCEL_MIN, ACCEL_MAX), True, True, True, self.nsb_msg("pulsing", -1.0), 0.8, DT_CTRL)[0],
                           -DEFAULT_PULSE_JERK * DT_CTRL)

  def test_not_applied(self):
    for state in ("off", "escalated", "urgent"):
      self.assertIsNone(PulseFloor().apply((ACCEL_MIN, ACCEL_MAX), True, True, True, self.nsb_msg(state, -1.0), 0., DT_CTRL))
    msg = self.nsb_msg("pulsing", -1.0)
    self.assertIsNone(PulseFloor().apply((ACCEL_MIN, ACCEL_MAX), False, True, True, msg, 0., DT_CTRL), "long inactive")
    self.assertIsNone(PulseFloor().apply((ACCEL_MIN, ACCEL_MAX), True, False, True, msg, 0., DT_CTRL), "invalid plan")
    self.assertIsNone(PulseFloor().apply((ACCEL_MIN, ACCEL_MAX), True, True, False, msg, 0., DT_CTRL), "toggle off")
    pf = PulseFloor()
    pf.apply((ACCEL_MIN, ACCEL_MAX), True, True, True, msg, -0.5, DT_CTRL)
    pf.apply((ACCEL_MIN, ACCEL_MAX), True, True, True, self.nsb_msg("urgent", 0.), -0.5, DT_CTRL)
    self.assertIsNone(pf.floor, "leaving the pulse path drops the ramp state")

  def test_bounded_inputs(self):
    # a corrupt target/rate from the plan can't drive the floor past MAX_DECEL_FLOOR or faster than the jerk clamp,
    # even if the incoming limits are wider than ACCEL_MIN
    pf = PulseFloor()
    out = [pf.apply((-10., ACCEL_MAX), True, True, True, self.nsb_msg("pulsing", -50., 99.), 0., DT_CTRL)[0] for _ in range(1000)]
    self.assertGreaterEqual(min(out), -3.5)
    self.assertLessEqual(np.abs(np.diff([0.] + out)).max(), PULSE_JERK_RANGE[1] * DT_CTRL + 1e-9)
    # only ever raises the lower limit, never above the upper limit, never touches the upper limit
    lo, hi = PulseFloor().apply((-0.2, -0.5), True, True, True, self.nsb_msg("coasting", -0.3), 0., DT_CTRL)
    self.assertEqual((lo, hi), (-0.5, -0.5))
    lo, hi = PulseFloor().apply((-0.1, 1.0), True, True, True, self.nsb_msg("pulsing", -1.0), -1.0, DT_CTRL)
    self.assertEqual((lo, hi), (-0.1, 1.0))

  def test_invalid_plan_sp_fails_to_braking(self):
    # pulsing publishes allowBrake True: if controlsd can't use the SP message it falls back to full braking
    ext = _Ext(setup())
    plan = messaging.new_message('longitudinalPlan').longitudinalPlan
    plan.allowBrake = True
    sp = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP
    sp.noServiceBrake.state, sp.noServiceBrake.accelFloor = "pulsing", -1.0
    sm = FakeSM({'longitudinalPlan': plan, 'longitudinalPlanSP': sp})
    sm.valid['longitudinalPlanSP'] = False
    self.assertEqual(ext.apply_no_service_brake(sm, (-3.5, 2.0), True, -0.3), (-3.5, 2.0))


class TestPulseClosedLoop(OpenpilotTestCase):
  def test_sawtooth_on_a_grade_it_can_hold(self):
    setup()
    sim = PulseSim(downhill=0.6)
    out = sim.run(120.)
    ps = pulses(out)
    self.assertGreaterEqual(len(ps), 4, "distinct pulses")
    self.assertFalse(any(l["state"] == "escalated" for l in out))
    self.assertFalse(any(EventNameSP.noServiceBrakeOverspeed in e for e in sim.events))
    for s, e in ps:
      self.assertGreater(out[s]["over"], DEFAULT_OVERSPEED_MARGIN - 0.05, "starts near set + trigger")
      self.assertLess(out[s]["over"], DEFAULT_OVERSPEED_MARGIN + 0.1)
      if e < len(out):
        self.assertLess(abs(out[e]["over"] - DEFAULT_RELEASE_MARGIN), 0.1, "releases near set + release")
    cmds = np.array([l["cmd"] for l in out])
    self.assertGreaterEqual(cmds.min(), DEFAULT_PULSE_DECEL - EPS, "bounded by the pulse decel")
    # no steps: over each planner frame the command moves no faster than the pulse jerk (roll-on/off) or the planner's
    # own cruise jerk (it updates at 20 Hz), after the engage transient
    jerk = np.abs(cmds[105:] - cmds[100:-5]) / DT_MDL
    self.assertLessEqual(jerk.max(), max(DEFAULT_PULSE_JERK, max(J_CRUISE_VALS)) + 1e-3, "rolled on and off, no steps")
    for s, _ in ps:  # the roll-on itself runs at the pulse jerk
      self.assertLessEqual(np.abs(np.diff(cmds[s:s + 40])).max(), DEFAULT_PULSE_JERK * DT_CTRL + 1e-4)
    # not a continuous hold: most of the time the brake is off (coast clamp), and between pulses the car coasts
    brake = cmds < -0.3 - EPS
    self.assertLess(brake.mean(), 0.6)
    gaps = [ps[i + 1][0] - ps[i][1] for i in range(len(ps) - 1)]
    self.assertGreater(min(gaps) * DT_CTRL, 3.0)
    overs = np.array([l["over"] for l in out])
    self.assertLess(overs.max(), DEFAULT_OVERSPEED_MARGIN + 0.3)
    self.assertGreater(overs[1000:].min(), DEFAULT_RELEASE_MARGIN - 0.5, "the roll-off undershoot stays small")

  def test_maintain_steps_where_pulse_ramps(self):
    # the A/B the mode exists for: same hill, maintain steps the command at each boundary
    setup(mode="maintain")
    out = PulseSim(downhill=0.6).run(60.)
    self.assertGreater(np.abs(np.diff([l["cmd"] for l in out])).max(), 0.5)

  def test_escalates_on_a_grade_it_cannot_hold(self):
    # a 0.6 m/s^2 pulse on a 0.9 m/s^2 hill can't slow the car; full openpilot braking can. (On this car's tune "full"
    # in non-adaptive is the cruise plan's A_CRUISE_MIN = -1.2 m/s^2: ki = 0, so nothing winds past the plan. A grade
    # steeper than that outruns maintain and escalation alike in this force-additive car model.)
    setup(NoServiceBrakeOverspeedPulseDecel=-0.6)
    sim = PulseSim(downhill=0.9)
    out = sim.run(120.)
    states = [l["state"] for l in out]
    self.assertIn("escalated", states)
    first = states.index("escalated")
    self.assertGreater(out[first]["over"], DEFAULT_HARD_CEILING)
    self.assertLess(out[first]["over"], DEFAULT_HARD_CEILING + 0.15)
    # bounded until escalation, then full braking pulls it back; never runs away
    self.assertGreaterEqual(min(l["cmd"] for l in out[:first]), -0.6 - EPS)
    self.assertLess(min(l["cmd"] for l in out[first:]), -0.6 - 0.3)
    self.assertLess(max(l["over"] for l in out), DEFAULT_HARD_CEILING + 0.3)
    # one alert per escalation episode
    episodes = sum(1 for i in range(1, len(states)) if states[i] == "escalated" and states[i - 1] != "escalated")
    alerts = sum(1 for e in sim.events if EventNameSP.noServiceBrakeOverspeed in e)
    self.assertGreaterEqual(episodes, 1)
    self.assertEqual(alerts, episodes)
    self.assertGreaterEqual(episodes, 2, "escalates again on the next cycle down the same hill")
    # while pulsing the command stays within the pulse bound (escalated braking only rolls off while coasting)
    for l in out:
      if l["state"] == "pulsing":
        self.assertGreaterEqual(l["cmd"], -0.6 - EPS)

  def test_close_lead_bypasses_the_bound(self):
    # urgent wins: a close lead at overspeed on a hill the pulse can't hold -> full braking before the ceiling
    setup(NoServiceBrakeOverspeedPulseDecel=-0.6)
    out = PulseSim(downhill=0.9, lead_gap=12.).run(30.)
    self.assertTrue(all(l["state"] == "urgent" for l in out[10:]))
    self.assertLess(min(l["cmd"] for l in out), -0.6 - 0.2, "braking past the pulse bound")
    self.assertLess(max(l["over"] for l in out), 1.5, "no coast clamp either: openpilot holds near set speed")

  def test_force_decel_not_limited(self):
    setup()
    out = PulseSim(downhill=0.6, force_decel=True).run(10.)
    self.assertTrue(all(l["allow_brake"] and l["state"] == "off" for l in out))
    self.assertLess(min(l["cmd"] for l in out), DEFAULT_PULSE_DECEL - 0.1)

  def test_alert_defined(self):
    alert = EVENTS_SP[EventNameSP.noServiceBrakeOverspeed][ET.WARNING]
    self.assertEqual(alert.alert_text_1, "Overspeed: Take Over")
