"""
Engine-brake learning (Task 17): collection only from brake-free coasts, the axle-torque brake check on engaged frames,
persistence through Params, and the (not applied) table proposal rule. Stage 3: the median (histogram) estimator, the
in-coast settle, and proposals that build up across drive cycles.
"""
import json
import math
import os
import tempfile
import time
from unittest import mock

from opendbc.car.structs import CLUTCH_PEDAL_FLOOR, CLUTCH_PEDAL_UP
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib import engine_brake_learning as L
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import engine_rpm_at
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import EngineBrakePreferred
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_table import STATIC_DECEL, V_MIDS, table_decel
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_manual_low_speed_handback import make_cp

G = 9.81


def frame(v=20., a=-0.45, gear=4, pedal=CLUTCH_PEDAL_UP, torque=-82., gas=False, brake=False, user_brake=-0.02,
          clutch_valid=True):
  CS = messaging.new_message('carState').carState
  CS.vEgo, CS.aEgo, CS.gasPressed, CS.brakePressed = v, a, gas, brake
  CS.deprecated.engineRpm = engine_rpm_at(gear, v) if gear else 850.
  CS.deprecated.clutchPressed = gear == 0 or pedal != CLUTCH_PEDAL_UP
  CS.deprecated.brake = user_brake
  sp = messaging.new_message('carStateSP').carStateSP
  sp.clutchSignalValid, sp.gearRaw, sp.clutchPedal, sp.engineTorqueAxle = clutch_valid, gear, pedal, torque
  return CS, sp


def stored(prev=None, timeout=2.):
  """Params writes are non-blocking: wait for the background write (a value different from prev)."""
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    v = Params().get(L.PARAM_KEY)
    if v is not None and v != prev:
      return v
    time.sleep(0.01)
  return Params().get(L.PARAM_KEY)


class Base(OpenpilotTestCase):
  def setUp(self):
    self.dir = tempfile.mkdtemp()
    Params().remove(L.PARAM_KEY)

  def learner(self, available=True):
    return L.EngineBrakeLearner(available, DT_MDL, Params(), self.dir)

  def run_frames(self, lr, seconds, enabled=False, long_active=False, sent=0., ebp_on=False, pitch=0., **kw):
    for _ in range(round(seconds / DT_MDL)):
      CS, sp = frame(**kw)
      lr.update(CS, sp, enabled, long_active, sent, ebp_on, pitch)


class TestCollection(Base):
  def test_clean_coast_is_learned(self):
    lr = self.learner()
    self.run_frames(lr, 8., pitch=-0.01, a=-0.45 + G * math.sin(0.01))
    cell = lr.store["stats"][4][L.v_bin(20.)]
    self.assertEqual(cell.n, round((8. - L.SETTLE_AFTER_GEAR - L.SETTLE_IN_COAST) / L.SAMPLE_PERIOD),
                     "everything after the gear settle + the in-coast settle")
    self.assertAlmostEqual(cell.median, 0.45, delta=L.HIST_STEP, msg="grade-corrected flat decel")
    self.assertAlmostEqual(cell.mean_e, L.K_TQ * 82., places=4, msg="the torque-based engine part")

  def test_settle_after_lift_and_gear_change(self):
    lr = self.learner()
    self.run_frames(lr, 1., gas=True)
    self.run_frames(lr, L.SETTLE_AFTER_GAS - 0.1)
    self.assertEqual(lr.coast_samples, 0)
    self.run_frames(lr, 0.1 + L.SETTLE_IN_COAST - 0.1)
    self.assertEqual(lr.coast_samples, 0, "the first second of a coast isn't learned")
    self.run_frames(lr, 1.)
    self.assertGreater(lr.coast_samples, 0)
    lr2 = self.learner()
    self.run_frames(lr2, 3., gear=3, v=15.)
    n = lr2.coast_samples
    self.run_frames(lr2, L.SETTLE_AFTER_GEAR - 0.1, gear=4, v=15.)
    self.assertEqual(lr2.coast_samples, n, "settling after the gear change")

  def test_anything_that_could_be_braking_is_not_learned(self):
    # each of these could hide service braking (or isn't a closed-throttle coast in a known gear): no sample
    cases = {"engaged": {"enabled": True}, "brake switch": {"brake": True}, "pedal pressure": {"user_brake": 0.3},
             "gas": {"gas": True}, "throttle open": {"torque": 40.}, "torque not alive": {"torque": float('nan')},
             "clutch in": {"pedal": CLUTCH_PEDAL_FLOOR}, "neutral": {"gear": 0}, "no clutch signal": {"clutch_valid": False},
             "no pitch": {"pitch": None}, "too slow": {"v": 2.5}, "phantom territory": {"gear": 6, "v": 10.}}
    for why, kw in cases.items():
      with self.subTest(why=why):
        lr = self.learner()
        self.run_frames(lr, 4., **kw)
        self.assertEqual(lr.coast_samples, 0)
        self.assertFalse(any(lr.store["stats"][g] for g in lr.store["stats"]))

  def test_engaged_frames_logged_for_review_never_learned(self):
    lr = self.learner()
    d4 = table_decel(STATIC_DECEL, 4, 20.)
    # engine braking alone: decel = engine part + road load -> not suspect
    e = L.K_TQ * 82.
    self.run_frames(lr, 2., enabled=True, long_active=True, sent=-0.3, ebp_on=True, a=-(e + L.road_load(20.)))
    self.assertGreater(lr.engaged_samples, 0)
    self.assertEqual(lr.brake_suspect, 0)
    # 0.5 m/s^2 more decel than the powertrain + road explain: brakes suspected
    self.run_frames(lr, 2., enabled=True, long_active=True, sent=-1.0, ebp_on=True, a=-(e + L.road_load(20.) + 0.5))
    self.assertGreater(lr.brake_suspect, 10)
    self.assertEqual(lr.coast_samples, 0)
    self.assertFalse(any(lr.store["stats"][g] for g in lr.store["stats"]))
    # the toggle off / long inactive / a positive request: nothing logged
    n = lr.engaged_samples
    self.run_frames(lr, 1., enabled=True, long_active=True, sent=-0.3, ebp_on=False)
    self.run_frames(lr, 1., enabled=True, long_active=False, sent=-0.3, ebp_on=True)
    self.run_frames(lr, 1., enabled=True, long_active=True, sent=0.2, ebp_on=True)
    self.assertEqual(lr.engaged_samples, n)
    self.assertGreater(d4, 0.)

  def test_samples_file(self):
    lr = self.learner()
    self.run_frames(lr, 2.)
    self.run_frames(lr, 1., enabled=True, long_active=True, sent=-0.5, ebp_on=True)
    self.assertTrue(lr.writer.drain())
    with open(os.path.join(self.dir, "samples.jsonl")) as f:
      lines = [json.loads(x) for x in f]
    self.assertEqual({x["src"] for x in lines}, {"coast", "engaged"})
    self.assertTrue(all(k in lines[0] for k in ("gear", "v", "a_ego", "pitch", "tq", "rpm", "d", "e")))
    eng = [x for x in lines if x["src"] == "engaged"][0]
    self.assertTrue(all(k in eng for k in ("sent", "extra", "brake_suspect", "pedal")))

  def test_inert_when_unavailable(self):
    lr = self.learner(available=False)
    self.run_frames(lr, 3.)
    self.assertEqual(lr.coast_samples, 0)
    self.assertIsNone(stored(timeout=0.3))


class TestPersistence(Base):
  def test_round_trip_through_params(self):
    lr = self.learner()
    self.run_frames(lr, 4.)
    lr.save()
    self.assertEqual(stored()["version"], L.VERSION)
    lr2 = self.learner()
    c1, c2 = lr.store["stats"][4][L.v_bin(20.)], lr2.store["stats"][4][L.v_bin(20.)]
    self.assertEqual(c1.to_dict(), c2.to_dict(), "survives a restart")
    self.assertEqual(lr2.store["cycles"], 2, "one drive cycle per start")

  def test_periodic_save(self):
    lr = self.learner()
    self.run_frames(lr, L.SAVE_PERIOD - 1.)
    first = stored(timeout=0.3)
    self.run_frames(lr, 2.)
    second = stored(first)
    self.assertNotEqual(first, second)
    self.assertGreater(second["stats"]["4"][str(L.v_bin(20.))]["n"], 0)

  def test_malformed_store_is_dropped_never_raised(self):
    V = L.VERSION
    ok = {"n": 3, "h": {"95": 3}, "ne": 3, "e": .2}
    for raw in (None, "", "nope", b"\x00\xff", 5, [], {"version": 99}, {"version": V, "stats": "x"},
                {"version": 1, "stats": {"4": {"4": [3, .4, .01, 3, .2]}}},   # a v1 (mean) store: dropped
                {"version": V, "stats": {"4": {"4": [1, 2]}}}, {"version": V, "stats": {"9": {"4": ok}}},
                {"version": V, "stats": {"4": {"40": ok}}}, {"version": V, "stats": {"4": {"4": {**ok, "n": float('nan')}}}},
                {"version": V, "stats": {"4": {"4": {**ok, "n": -3}}}}, {"version": V, "stats": {"4": {"4": {**ok, "n": 4}}}},
                {"version": V, "stats": {"4": {"4": {**ok, "h": {"999": 3}}}}},
                {"version": V, "stats": {"4": {"4": {**ok, "h": {"x": 3}}}}},
                {"version": V, "stats": {"4": {"4": {**ok, "h": {"95": -3}}}}}, {"version": V, "proposed": {"4": [1]}},
                {"version": V, "cycles": "many"}):
      with self.subTest(raw=raw):
        store = L.parse_store(raw)
        self.assertFalse(any(store["stats"][g] for g in store["stats"]))
        self.assertEqual(store["proposed"], {})
    good = L.parse_store({"version": V, "stats": {"4": {"4": ok, "x": [1]}}, "cycles": 3})
    self.assertEqual(good["stats"][4][4].n, 3)
    self.assertAlmostEqual(good["stats"][4][4].median, L.HIST_LO + 95.5 * L.HIST_STEP)
    self.assertEqual(good["cycles"], 3)

  def test_v1_store_is_reset_and_logged(self):
    Params().put(L.PARAM_KEY, {"version": 1, "cycles": 2, "stats": {"4": {"4": [500, 0.5, 1.0, 500, 0.2]}}}, block=True)
    with mock.patch.object(L.cloudlog, "event") as ev:
      lr = self.learner()
    self.assertFalse(any(lr.store["stats"][g] for g in lr.store["stats"]))
    self.assertEqual(lr.store["cycles"], 1)
    self.assertIn("engine_brake_learn_store_reset", [c.args[0] for c in ev.call_args_list])


def store_cell(d, n):
  """A v2 store cell holding n samples at D = d."""
  i = int((d - L.HIST_LO) / L.HIST_STEP)
  return {"n": n, "h": {str(i): n}, "ne": n, "e": 0.3}


class TestEstimator(Base):
  def test_median_ignores_the_tail(self):
    c = L.Cell()
    for i in range(1000):
      c.add(1.5 if i % 10 == 0 else 0.45, None)   # 10 % long-tail samples (descent / pitch error)
    self.assertAlmostEqual(c.median, 0.45, delta=L.HIST_STEP)
    self.assertLess(c.lower_bound(), c.median + 1e-9)
    self.assertGreater(c.lower_bound(), 0.44, "with 100 s of data the bound is near the median")

  def test_lower_bound_tightens_with_data(self):
    few, many = L.Cell(), L.Cell()
    for i in range(L.MIN_SAMPLES):
      few.add(0.3 + 0.004 * i, None)            # 0.30 .. 0.50, median ~0.40
    for i in range(2000):
      many.add(0.3 + 0.0001 * i, None)
    self.assertLess(few.lower_bound(), few.median - 0.05, "5 s of coast: a low quantile")
    self.assertGreater(many.lower_bound(), many.median - 0.02)

  def test_broken_coast_settles_again(self):
    lr = self.learner()
    self.run_frames(lr, 3.)
    n = lr.coast_samples
    self.run_frames(lr, 0.2, brake=True)
    self.run_frames(lr, L.SETTLE_IN_COAST - 0.1)
    self.assertEqual(lr.coast_samples, n, "a new coast after a brake tap settles again")
    self.run_frames(lr, 1.)
    self.assertGreater(lr.coast_samples, n)


class TestProposal(Base):
  def cell(self, n, mean, sd=0.05):
    c = L.Cell()
    for i in range(n):
      c.add(mean + sd * (1 if i % 2 else -1), None)
    return c

  def test_rules(self):
    b = L.v_bin(17.)
    s = STATIC_DECEL[4][b]
    stats = {4: {b: self.cell(200, s + 0.15)}}
    table, notes = L.propose_table(stats)
    self.assertAlmostEqual(table[4][b], s + L.MAX_STEP, msg="at most MAX_STEP per drive cycle")
    table2, _ = L.propose_table(stats, learned_prev=table)
    self.assertAlmostEqual(table2[4][b], s + 2 * L.MAX_STEP)
    for g in table:
      for i, x in enumerate(table[g]):
        if (g, i) != (4, b):
          self.assertEqual(x, STATIC_DECEL[g][i], "untouched cells stay static")
    # never above the measured lower bound
    prev = {4: [x + 0.149 if i == b else x for i, x in enumerate(STATIC_DECEL[4])]}
    t3, _ = L.propose_table(stats, learned_prev=prev)
    self.assertLessEqual(t3[4][b], stats[4][b].lower_bound() + 1e-9)
    # too few samples: no move
    t4, _ = L.propose_table({4: {b: self.cell(L.MIN_SAMPLES - 1, s + 0.15)}})
    self.assertEqual(t4[4][b], s)
    # measured below static: static stays (the floor), and it is reported for review
    t5, notes5 = L.propose_table({4: {b: self.cell(200, s - 0.1)}})
    self.assertEqual(t5[4][b], s)
    self.assertTrue(notes5 and "< static" in notes5[0])
    self.assertEqual(notes, [])

  def test_proposal_is_stored_but_never_reaches_the_clamp(self):
    b = L.v_bin(20.)
    Params().put(L.PARAM_KEY, {"version": L.VERSION, "cycles": 4, "stats": {"4": {str(b): store_cell(0.9, 500)}}},
                 block=True)
    lr = self.learner()
    self.assertGreater(lr.store["proposed"][4][b], STATIC_DECEL[4][b], "a proposal exists")
    ebp = EngineBrakePreferred(make_cp()[0], Params())
    self.assertFalse(ebp.table.apply_learned)
    self.assertEqual(ebp.table.decel(4, V_MIDS[b]), STATIC_DECEL[4][b], "the clamp still reads the static table")

  def test_proposals_build_up_across_drive_cycles(self):
    # Stage 3: each plannerd start used to restart from the static table (never more than MAX_STEP from it)
    b = L.v_bin(20.)
    s = STATIC_DECEL[4][b]
    Params().put(L.PARAM_KEY, {"version": L.VERSION, "cycles": 0, "stats": {"4": {str(b): store_cell(s + 0.2, 3000)}}},
                 block=True)
    seen = []
    for _ in range(4):
      lr = self.learner()
      seen.append(lr.store["proposed"][4][b])
      lr.save()
      stored(timeout=0.5)
      Params().put(L.PARAM_KEY, L.dump_store(lr.store), block=True)
    for k, x in enumerate(seen, 1):
      self.assertAlmostEqual(x, s + k * L.MAX_STEP, places=6, msg=f"cycle {k}")
    self.assertEqual(L.parse_store(Params().get(L.PARAM_KEY))["cycles"], 4)
    self.assertLess(seen[-1], s + 0.2, "still never above the measured bound")
