"""Session 10 (owner decisions 2026-10-21): SetSpeedCoast (gentle decel floor at highway set speeds) and CruiseAccelBoost
(higher set-speed accel ceiling) - cruise_profile.py, wired through the planner's set-speed candidate. Closed loop on the
ManualCarSim plant ("closed": the car tracks the request, as the Honda radarless path does at highway speed)."""
import numpy as np

from opendbc.car import structs
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_planner import (A_CRUISE_MAX_VALS, A_CRUISE_MIN, get_cruise_accel,
                                                                   get_max_accel)
from openpilot.sunnypilot.selfdrive.controls.lib.cruise_profile import (A_CRUISE_MAX_VALS_BOOST, COAST_DECEL_DEFAULT,
                                                                       CruiseProfile)
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import ManualCarSim, set_params

TransmissionType = structs.CarParams.TransmissionType
MPH = 0.44704


class _CP:
  def __init__(self, op_long=True):
    self.openpilotLongitudinalControl = op_long


def drop(sim, by_mph, seconds):
  """Lower the set speed by `by_mph` and run; returns the log rows of the run."""
  n0 = len(sim.log)
  sim.set(v_set=sim.v_set - by_mph * MPH)
  sim.run(seconds)
  return sim.log[n0:]


class TestCruiseProfileUnit(OpenpilotTestCase):
  def test_defaults_are_stock(self):
    set_params()
    cp = CruiseProfile(_CP())
    a_min, vals = cp.update(True, 30., 31., None)
    self.assertEqual((a_min, vals), (A_CRUISE_MIN, A_CRUISE_MAX_VALS))
    self.assertFalse(cp.coast_enabled or cp.boost_enabled or cp.shaped)

  def test_stock_ACC_never(self):
    set_params(SetSpeedCoast=True, CruiseAccelBoost=True)
    cp = CruiseProfile(_CP(op_long=False))
    self.assertEqual(cp.update(False, 30., 31., None), (A_CRUISE_MIN, A_CRUISE_MAX_VALS))
    self.assertFalse(cp.shaped)

  def test_coast_only_at_highway_set_speeds_engaged_and_without_urgency(self):
    set_params(SetSpeedCoast=True)
    cp = CruiseProfile(_CP())
    self.assertAlmostEqual(cp.update(True, 30., 31., None)[0], -COAST_DECEL_DEFAULT)
    self.assertEqual(cp.update(True, 24.0, 25., None)[0], A_CRUISE_MIN)       # set speed below 55 mph: stock
    self.assertEqual(cp.update(True, 0., 31., None)[0], A_CRUISE_MIN)         # forceDecel zeroes the target: stock
    self.assertEqual(cp.update(False, 30., 31., None)[0], A_CRUISE_MIN)       # disengaged
    for reason in ("fcw", "force_decel", "lead_closing", "invalid_radar", "hard_decel"):
      self.assertEqual(cp.update(True, 30., 31., reason)[0], A_CRUISE_MIN, reason)   # Rule 5: same frame

  def test_boost_table(self):
    set_params(CruiseAccelBoost=True)
    cp = CruiseProfile(_CP())
    self.assertEqual(cp.update(True, 30., 25., None)[1], A_CRUISE_MAX_VALS_BOOST)
    self.assertFalse(cp.shaped)   # throttle side only: no stock shadow needed
    self.assertAlmostEqual(get_max_accel(26.8, A_CRUISE_MAX_VALS_BOOST), 1.073, places=2)   # 60 mph
    self.assertAlmostEqual(get_max_accel(26.8), 0.776, places=2)                             # stock
    for v in np.linspace(0, 45, 46):
      self.assertGreaterEqual(get_max_accel(v, A_CRUISE_MAX_VALS_BOOST), get_max_accel(v) - 1e-9)

  def test_latched_only_while_disengaged(self):
    set_params()
    cp = CruiseProfile(_CP())
    cp.update(True, 30., 31., None)
    set_params(SetSpeedCoast=True, CruiseAccelBoost=True)
    for _ in range(int(3 / DT_MDL)):
      cp.update(True, 30., 31., None)
    self.assertFalse(cp.coast_enabled or cp.boost_enabled)
    cp.update(False, 30., 31., None)
    self.assertTrue(cp.coast_enabled and cp.boost_enabled)

  def test_params_clamped(self):
    set_params(SetSpeedCoast=True, SetSpeedCoastDecel=5.0, SetSpeedCoastMinSpeed=2.0)
    cp = CruiseProfile(_CP())
    self.assertAlmostEqual(cp.coast_decel, 0.3)
    self.assertAlmostEqual(cp.coast_min_speed, 20.)
    set_params(SetSpeedCoast=True, SetSpeedCoastDecel=float("nan"))
    self.assertAlmostEqual(CruiseProfile(_CP()).coast_decel, COAST_DECEL_DEFAULT)

  def test_get_cruise_accel_defaults_bit_identical(self):
    CP = type("CP", (), {"steerRatio": 15.38, "wheelbase": 2.7})()
    for v_cruise, v_ego, prev in ((30., 33., 0.), (30., 33., -1.0), (30., 25., 0.3), (10., 5., 0.)):
      a = get_cruise_accel(False, v_cruise, v_ego, prev, 0., CP, DT_MDL, 0., True)
      b = get_cruise_accel(False, v_cruise, v_ego, prev, 0., CP, DT_MDL, 0., True, (1.0, 1.0), A_CRUISE_MIN, A_CRUISE_MAX_VALS)
      self.assertEqual(a, b)


class TestSetSpeedCoastClosedLoop(OpenpilotTestCase):
  def _sim(self, v_mph, transmission=TransmissionType.manual, **params):
    sim = ManualCarSim(speed=v_mph * MPH, gear=6, v_set=v_mph * MPH, plant="closed", transmission=transmission,
                       ClutchShiftGate=False, **params)
    sim.run(3.)
    return sim

  def test_highway_drop_is_gentle_and_reaches_the_set_speed(self):
    sim = self._sim(70, SetSpeedCoast=True)
    rows = drop(sim, 5, 30.)
    self.assertGreaterEqual(min(r["a_target"] for r in rows), -COAST_DECEL_DEFAULT - 1e-3)
    self.assertGreaterEqual(min(r["sent"] for r in rows), -COAST_DECEL_DEFAULT - 0.02)
    self.assertLess(abs(sim.speed - sim.v_set), 0.6 * MPH, "reached the new set speed")
    self.assertTrue(sim.plan_sp.features.setSpeedCoast)

  def test_stock_drop_brakes_harder(self):
    sim = self._sim(70)
    rows = drop(sim, 5, 30.)
    self.assertLess(min(r["a_target"] for r in rows), -0.5)
    self.assertFalse(sim.plan_sp.features.setSpeedCoast)
    self.assertEqual(sim.plan_sp.features.aTargetStock, 0.)   # nothing new published while off

  def test_below_the_threshold_is_stock(self):
    on, off = self._sim(45, SetSpeedCoast=True), self._sim(45)
    a, b = drop(on, 5, 10.), drop(off, 5, 10.)
    self.assertEqual([r["a_target"] for r in a], [r["a_target"] for r in b])

  def test_descent_still_converges(self):
    # a 4 % descent: the car tracks the capped request, so it still slows (never a runaway)
    sim = self._sim(70, SetSpeedCoast=True)
    sim.grade = -0.04
    rows = drop(sim, 3, 40.)
    self.assertLess(sim.speed, sim.v_set + 0.6 * MPH)
    self.assertLess(max(r["v"] for r in rows), 70 * MPH + 0.5)

  def test_automatic_too(self):
    sim = self._sim(70, transmission=TransmissionType.automatic, SetSpeedCoast=True)
    rows = drop(sim, 5, 5.)
    self.assertGreaterEqual(min(r["a_target"] for r in rows), -COAST_DECEL_DEFAULT - 1e-3)

  def test_urgency_releases_the_cap_at_once(self):
    # a slow lead appears close ahead mid-coast: the lead MPC / Rule 5 brake as stock, not at the cap
    sim = self._sim(70, SetSpeedCoast=True)
    drop(sim, 5, 5.)
    sim.lead = {"pos": sim.pos + 35., "v": sim.speed - 8., "a": 0.}
    sim.run(3.)
    self.assertLess(min(r["sent"] for r in sim.log[-int(3 / 0.01):]), -0.5)


class TestCruiseAccelBoostClosedLoop(OpenpilotTestCase):
  def _time_to(self, boost):
    sim = ManualCarSim(speed=60 * MPH, gear=6, v_set=60 * MPH, plant="closed", ClutchShiftGate=False, CruiseAccelBoost=boost)
    sim.run(2.)
    sim.set(v_set=75 * MPH)
    t = 0.
    while sim.speed < 74.5 * MPH and t < 30.:
      sim.run(DT_MDL)
      t += DT_MDL
    return t, max(r["a_target"] for r in sim.log)

  def test_boost_is_faster_and_bounded(self):
    t_stock, a_stock = self._time_to(False)
    t_boost, a_boost = self._time_to(True)
    self.assertLess(t_boost, t_stock - 2.0)
    self.assertLessEqual(a_stock, 0.79)
    self.assertLessEqual(a_boost, 1.08)
    self.assertGreater(a_boost, 0.95)
