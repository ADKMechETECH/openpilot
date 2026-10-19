"""
Integration: manual low-speed handback composed with NonAdaptiveLongitudinal and NoServiceBrakeHold, each on and off,
through the same closed loop as test_manual_low_speed_handback.py (planner -> NSB floor -> handback envelope ->
LongControl, and selfdrived's events/state machine/alerts).
"""
import itertools

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import DEFAULT_HANDBACK_SPEED, DEFAULT_RAMP_TIME, \
  DISENGAGE_MARGIN, ENGAGE_HYSTERESIS
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_manual_low_speed_handback import Sim, make_cp, first, HANDBACK_ALERT

COMBOS = list(itertools.product((False, True), repeat=2))  # (NonAdaptiveLongitudinal, NoServiceBrakeHold)


def setup(non_adaptive, no_service_brake):
  p = Params()
  p.put_bool("NonAdaptiveLongitudinal", non_adaptive, block=True)
  p.put_bool("NoServiceBrakeHold", no_service_brake, block=True)
  for k in ("ManualLowSpeedHandbackSpeed", "ManualLowSpeedHandbackRamp", "NoServiceBrakeDecelFloor",
            "NoServiceBrakeOverspeedBackstop", "NoServiceBrakeOverspeedMargin"):
    p.remove(k)


def make_sim(non_adaptive, no_service_brake, **kw):
  setup(non_adaptive, no_service_brake)
  CP, CP_SP = make_cp()
  sim = Sim(CP, CP_SP, **kw)
  assert sim.planner.non_adaptive == non_adaptive, "test setup: toggle not latched"
  assert sim.ext.no_service_brake_enabled == no_service_brake
  return sim


class TestHandbackComposition(OpenpilotTestCase):
  def assert_clean_handback(self, log, msg):
    start = first(log, lambda l: l["handback"])
    off = first(log, lambda l: not l["enabled"])
    self.assertIsNotNone(start, msg)
    self.assertIsNotNone(off, msg)
    self.assertLess(log[start]["v_in"], DEFAULT_HANDBACK_SPEED, msg)
    self.assertGreater(log[start - 1]["v_in"], DEFAULT_HANDBACK_SPEED, msg)
    self.assertAlmostEqual((off - start) * DT_CTRL, DEFAULT_RAMP_TIME + DISENGAGE_MARGIN, delta=0.03, msg=msg)
    self.assertEqual(log[off - 1]["cmd"], 0., msg)
    cmds = [l["cmd"] for l in log[start - 1:off + 1]]
    max_step = max(abs(b - a) for a, b in zip(cmds, cmds[1:], strict=False))
    self.assertLess(max_step, abs(log[start - 1]["cmd"]) / (DEFAULT_RAMP_TIME / DT_CTRL) * 1.5 + 0.02, msg)
    # the prompt from the ramp start (A3.3); stock speedTooLow's own alert follows at the disengage
    shown = {l["alert"].alert_type for l in log if l["alert"].alert_type}
    self.assertEqual(shown - {"speedTooLow/immediateDisable"}, {HANDBACK_ALERT}, msg)
    self.assertFalse(any(l["enabled"] for l in log[off:]), msg)
    return start, off

  def test_uphill_slowdown_all_combos(self):
    # openpilot on the gas but can't hold set speed up a steep hill: hands back the same way in every combination
    results = {}
    for non_adaptive, nsb in COMBOS:
      msg = f"non_adaptive={non_adaptive} no_service_brake={nsb}"
      with self.subTest(msg):
        sim = make_sim(non_adaptive, nsb, speed=4., grade=-3.0, v_set=10.)
        log = sim.run(6.)
        start, off = self.assert_clean_handback(log, msg)
        self.assertGreater(log[start - 1]["cmd"], 0.3, msg)
        results[(non_adaptive, nsb)] = off
    self.assertEqual(len(set(results.values())), 1, f"handback timing must not depend on the toggles: {results}")

  def test_adaptive_lead_stop_nsb_inert(self):
    # adaptive: openpilot brakes for the stopping lead. Since Task 12 A3 a close/closing lead is Rule-5 urgency, so the
    # handback keeps the brakes available and the car may be braked to a stop behind it (no contact). NoServiceBrakeHold
    # is coupled to non-adaptive, so ON vs OFF must give the identical run (braking always allowed).
    logs = {}
    for nsb in (False, True):
      with self.subTest(no_service_brake=nsb):
        sim = make_sim(False, nsb, speed=12., lead={"pos": 45., "v": 12., "decel": 1.5})
        log = sim.run(30.)
        self.assertFalse(sim.contact)
        self.assertGreater(min(l["gap"] for l in log), 1.0)
        self.assertTrue(all(l["allow_brake"] for l in log))
        for l in log:
          if l["handback"] and l["urgency"] is not None:
            self.assertLessEqual(l["limits"][0], -3.0, "braking side at the stock limit while urgent")
        logs[nsb] = [(l["cmd"], l["enabled"]) for l in log]
    self.assertEqual(logs[False], logs[True])

  def test_non_adaptive_driver_braking_disengages_above_threshold(self):
    # non-adaptive ignores the stopping lead, so the driver brakes - and on a Honda the brake pedal disengages
    # openpilot (stock pedalPressed) long before the handback speed: the handback never runs.
    for nsb in (False, True):
      with self.subTest(no_service_brake=nsb):
        sim = make_sim(True, nsb, speed=12., lead={"pos": 60., "v": 12., "decel": 1.5}, v_set=12.)
        log, gaps, braked = [], [], None
        for i in range(round(30. / DT_CTRL)):
          gap = sim.lead["pos"] - sim.distance
          if braked is None and gap < 25.:
            sim.brake_pedal, sim.driver_brake, braked = True, 3.0, i
          log.append(sim.step())
          gaps.append(gap)
        self.assertIsNotNone(braked)
        # NSB clamps before the brake, except while non-adaptive is braking for the collision threat (A1)
        self.assertTrue(all(l["cmd"] >= -0.3 - 1e-6 or not nsb or l["collision"] for l in log[:braked]),
                        "NSB clamp active before the brake")
        off = first(log, lambda l: not l["enabled"])
        self.assertEqual(off, braked, "the brake pedal disengages on the same step")
        self.assertGreater(log[off]["v_in"], DEFAULT_HANDBACK_SPEED + 3., "disengaged by the brake, well above the threshold")
        self.assertFalse(any(l["handback"] for l in log), "the handback never ran")
        self.assertNotIn(HANDBACK_ALERT, {l["alert"].alert_type for l in log})
        self.assertEqual(log[-1]["speed"], 0.)
        self.assertGreater(min(gaps), 0.)

  def test_non_adaptive_lead_stop_without_driver_brakes_no_contact(self):
    # (replaces test_non_adaptive_lead_stop_without_driver_is_fcw_not_handback, which asserted openpilot never slowed
    # and only FCW warned — the hazard Task 12 A1 removes.) The lead stopping ahead is a collision threat: non-adaptive
    # re-admits the lead MPC, brakes, and the low-speed handback keeps braking available (A3): no contact.
    for nsb in (False, True):
      with self.subTest(no_service_brake=nsb):
        sim = make_sim(True, nsb, speed=12., lead={"pos": 60., "v": 12., "decel": 1.5}, v_set=12.)
        log = sim.run(20.)
        self.assertFalse(sim.contact)
        self.assertGreater(min(l["gap"] for l in log), 1.0)
        self.assertLess(min(l["cmd"] for l in log), -1.0, "openpilot brakes for the stopping lead")
        self.assertLess(log[-1]["speed"], DEFAULT_HANDBACK_SPEED + ENGAGE_HYSTERESIS)

  def test_nsb_floor_and_ramp_compose(self):
    # NSB floor and the handback envelope both shrink LongControl's range; the tighter bound wins, never widens
    for non_adaptive, nsb in COMBOS:
      with self.subTest(non_adaptive=non_adaptive, nsb=nsb):
        sim = make_sim(non_adaptive, nsb, speed=4., grade=-3.0, v_set=10.)
        for l in sim.run(6.):
          lo, hi = l["limits"]
          self.assertLessEqual(lo, hi)
          self.assertGreaterEqual(l["cmd"], lo - 1e-6)
          self.assertLessEqual(l["cmd"], hi + 1e-6)
          if nsb and non_adaptive and not l["allow_brake"]:
            self.assertGreaterEqual(lo, -0.3 - 1e-6)
