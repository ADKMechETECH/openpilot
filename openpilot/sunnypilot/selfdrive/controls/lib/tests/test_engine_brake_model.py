"""
engine_brake_model: interface, gear 0, monotonicity, grade, gear inference, and the coast-event calibration logger.
The absolute seed values are placeholders and deliberately NOT tested.
"""
import json
import math
import os
import tempfile

from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import CoastEventLogger, GEAR_RATIOS, \
  COAST_MIN_EVENT, COAST_MAX_EVENT, engine_brake_decel, engine_rpm_at, estimate_coast_decel, infer_gear, resistive_decel

GEARS = range(1, len(GEAR_RATIOS) + 1)


class TestEngineBrakeModel(OpenpilotTestCase):
  def test_interface_sane(self):
    for v in (5., 10., 20., 30.):
      for g in range(7):
        rpm = engine_rpm_at(g, v) if g else 800.
        a = estimate_coast_decel(g, rpm, v)
        self.assertIsInstance(a, float)
        self.assertTrue(math.isfinite(a))
        self.assertLess(a, 0., f"gear {g} v {v}: coasting on the flat must decelerate")
        self.assertGreater(a, -4.)
    self.assertEqual(estimate_coast_decel(3, float('nan'), 10.), 0.)

  def test_gear_zero_is_coasting_only(self):
    for v in (5., 15., 30.):
      for rpm in (0., 800., 3000.):
        self.assertEqual(engine_brake_decel(0, rpm), 0.)
        self.assertAlmostEqual(estimate_coast_decel(0, rpm, v), resistive_decel(v))
      for bogus in (7, 14, -1):  # reverse / out of range: no engine braking credited
        self.assertEqual(engine_brake_decel(bogus, 2000.), 0.)
      self.assertGreater(estimate_coast_decel(0, 800., v), estimate_coast_decel(3, engine_rpm_at(3, v), v))

  def test_higher_gear_weaker_at_same_speed(self):
    for v in (6., 12., 20., 30.):
      a = [estimate_coast_decel(g, engine_rpm_at(g, v), v) for g in GEARS]
      self.assertEqual(a, sorted(a), f"v={v}: {a}")
      self.assertEqual(len(set(a)), len(a))

  def test_higher_gear_weaker_engine_braking_at_same_rpm(self):
    for rpm in (1000., 2500., 5000.):
      a = [engine_brake_decel(g, rpm) for g in GEARS]
      self.assertEqual(a, sorted(a), f"rpm={rpm}: {a}")

  def test_higher_rpm_stronger(self):
    for g in GEARS:
      for v in (10., 20.):
        a = [estimate_coast_decel(g, rpm, v) for rpm in (1000., 2000., 3000., 4500., 6000.)]
        self.assertEqual(a, sorted(a, reverse=True), f"gear {g}: {a}")
      a = [estimate_coast_decel(g, engine_rpm_at(g, v), v) for v in (5., 10., 15., 20.)]
      self.assertEqual(a, sorted(a, reverse=True), f"gear {g}, rpm rising with speed: {a}")

  def test_grade(self):
    v, g = 20., 4
    rpm = engine_rpm_at(g, v)
    flat = estimate_coast_decel(g, rpm, v)
    self.assertLess(estimate_coast_decel(g, rpm, v, 0.05), flat)
    self.assertGreater(estimate_coast_decel(g, rpm, v, -0.05), flat)
    self.assertGreaterEqual(estimate_coast_decel(6, engine_rpm_at(6, v), v, -0.1), 0., "steep downhill: no slowing")

  def test_infer_gear(self):
    for g in GEARS:
      for v in (8., 15., 25.):
        rpm = engine_rpm_at(g, v)
        self.assertEqual(infer_gear(rpm, v), g)
        self.assertEqual(infer_gear(rpm * 1.05, v), g)
        self.assertEqual(infer_gear(rpm * 0.95, v), g)
        self.assertEqual(infer_gear(rpm, v, clutch_pressed=True), 0)
    v = 15.
    for g in GEARS:  # between gears (slipping clutch, mid-shift): no match
      self.assertEqual(infer_gear(engine_rpm_at(g, v) * 1.12, v) == g, False)
    self.assertEqual(infer_gear(800., 25.), 0, "neutral, clutch out, idling while rolling")
    self.assertEqual(infer_gear(0., 20.), 0, "no RPM signal")
    self.assertEqual(infer_gear(float('nan'), 20.), 0)
    self.assertEqual(infer_gear(engine_rpm_at(1, 2.), 2.), 0, "too slow to trust")


def coast_cs(v=20., gear=4, a_ego=-0.5, gas=False, brake=False, clutch=False, aeb=False):
  CS = messaging.new_message('carState').carState
  CS.vEgo, CS.aEgo = v, a_ego
  CS.deprecated.engineRpm = float(engine_rpm_at(gear, v)) if gear else 800.
  CS.deprecated.clutchPressed = clutch
  CS.gasPressed, CS.brakePressed, CS.stockAeb = gas, brake, aeb
  return CS


class TestCoastEventLogger(OpenpilotTestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.logger = CoastEventLogger(True, DT_MDL, log_dir=self.tmp.name)

  def tearDown(self):
    self.tmp.cleanup()

  def run_frames(self, seconds, long_active=False, fcw=False, **cs):
    v = cs.pop("v", 20.)
    for _ in range(round(seconds / DT_MDL)):
      self.logger.update(coast_cs(v=v, **cs), long_active, fcw, 0.01)
      v += cs.get("a_ego", -0.5) * DT_MDL

  def events(self):
    path = os.path.join(self.tmp.name, "coast.jsonl")
    if not os.path.exists(path):
      return []
    with open(path) as f:
      return [json.loads(line) for line in f]

  def test_records_in_gear_coast(self):
    self.run_frames(3.)
    self.run_frames(0.5, gas=True)  # ends the event
    ev = self.events()
    self.assertEqual(len(ev), 1)
    e = ev[0]
    self.assertEqual(e["gear"], 4)
    self.assertAlmostEqual(e["duration"], 3., delta=0.1)
    self.assertEqual(e["fields"], ["t", "rpm", "v_ego", "ratio", "a_ego", "pitch"])
    self.assertAlmostEqual(len(e["samples"]), 30, delta=2)  # ~10 Hz
    t, rpm, v, ratio, a, pitch = e["samples"][0]
    self.assertAlmostEqual(a, -0.5)
    self.assertAlmostEqual(pitch, 0.01)
    self.assertGreater(rpm, 0.)

  def test_disqualifiers(self):
    for kw in ({"gas": True}, {"brake": True}, {"clutch": True}, {"aeb": True}, {"gear": 0}, {"long_active": True},
               {"fcw": True}):
      with self.subTest(**kw):
        self.logger = CoastEventLogger(True, DT_MDL, log_dir=self.tmp.name)
        before = len(self.events())
        self.run_frames(3., **kw)
        self.run_frames(0.5, gas=True)
        self.assertEqual(len(self.events()), before, kw)

  def test_short_events_dropped_gear_change_splits(self):
    self.run_frames(COAST_MIN_EVENT * 0.5)
    self.run_frames(0.5, gas=True)
    self.assertEqual(self.events(), [])
    self.run_frames(2., gear=4)
    self.run_frames(2., gear=3)
    self.run_frames(0.5, brake=True)
    self.assertEqual([e["gear"] for e in self.events()], [4, 3])

  def test_long_events_chunked(self):
    self.run_frames(COAST_MAX_EVENT * 2 + 1., a_ego=0.)
    self.run_frames(0.5, gas=True)
    self.assertEqual(len(self.events()), 3)

  def test_unavailable_never_writes(self):
    self.logger = CoastEventLogger(False, DT_MDL, log_dir=self.tmp.name)
    self.run_frames(3.)
    self.run_frames(0.5, gas=True)
    self.assertEqual(self.events(), [])
