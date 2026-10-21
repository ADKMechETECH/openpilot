"""
EngineBrakePreferred (Task 17: the per-gear clamp), closed loop: LongitudinalPlanner (adaptive or non-adaptive) ->
controlsd's brake-range clamps (no-service-brake floor, engine-brake clamp) -> LongControl -> the car as Stage 2
measured it (0 = zero torque; a negative request closes the throttle and the VSA brakes only what the car doesn't
already achieve, downhill too), with a lead, a grade and a pitch-estimate error.
"""
import math
from unittest import mock

import numpy as np

from opendbc.car.honda import carstate as honda_carstate
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from opendbc.car.structs import CLUTCH_PEDAL_FLOOR, CLUTCH_PEDAL_TRAVEL, CLUTCH_PEDAL_UNKNOWN, CLUTCH_PEDAL_UP
from openpilot.cereal import log, messaging
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib import engine_brake_preferred as ebp_mod
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import braking_urgency
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import G, engine_rpm_at
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_learning import road_load
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import EngineBrakePreferred, lead_urgent, \
  trusted_gear, RELEASE_HOLD, OVERSPEED_MARGIN, HARD_FLOOR_RANGE, LEAD_TTC_RANGE, MIN_SPEED
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_table import FLOOR_MAX, MARGIN, STATIC_DECEL, \
  UNKNOWN_GEAR_FLOOR, EngineBrakeTable, brake_free_floor, table_decel
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import apply_brake_floor, read_decel_floor
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_manual_low_speed_handback import make_cp, TransmissionType

ZERO_TORQUE_BAND = 0.05   # m/s^2: Stage 2 C2 - a request this close to 0 holds ~zero axle torque (no engine braking)
BRAKE_EPS = 0.02          # m/s^2: service braking below this counts as none
ACTUATOR_TAU = 0.25       # s
DEG = math.pi / 180.


def setup(ebp=False, non_adaptive=False, nsb=False, **params):
  p = Params()
  p.put_bool("EngineBrakePreferred", ebp)
  p.put_bool("NonAdaptiveLongitudinal", non_adaptive)
  p.put_bool("NoServiceBrakeHold", nsb)
  for k in ("EngineBrakeHardFloor", "EngineBrakeLeadTtc", "EngineBrakeLeadClosingSpeed", "NoServiceBrakeDecelFloor",
            "NoServiceBrakeOverspeedBackstop"):
    p.remove(k)
  for k, v in params.items():
    p.put(k, v)


def urgency_for(CS, plan, radar, plan_ok=True, radar_ok=True, force_decel=False):
  """What controlsd's controls_urgency() computes for these messages (Rule 5)."""
  return braking_urgency(plan_ok=plan_ok, radar_ok=radar_ok, force_decel=force_decel, fcw=plan.fcw,
                         stock_aeb=CS.stockAeb or CS.stockFcw, should_stop=plan.shouldStop, a_target=plan.aTarget,
                         v_ego=CS.vEgo, radar=radar)


class ManualSim:
  """The car as Stage 2 measured it. true_scale: the car's real closed-throttle decel vs the static table's.
  pitch_bias: the calibrated pose's error (rad, + = the estimate reads more uphill than the road). gear 0 = neutral."""
  def __init__(self, speed=25., v_set=25., gear=6, lead=None, grade=0., true_scale=1.0, pitch=True, pitch_bias=0.,
               ebp_code=True, transmission=TransmissionType.manual, op_long=True, force_decel=False,
               pedal=CLUTCH_PEDAL_UP, plant="measured"):
    self.CP, self.CP_SP = make_cp(transmission, op_long)
    self.params = Params()
    self.planner = LongitudinalPlanner(self.CP, self.CP_SP, init_v=speed)
    self.loc = LongControl(self.CP, self.CP_SP)
    self.loc.long_control_state = LongCtrlState.pid  # already engaged and cruising
    self.ebp = EngineBrakePreferred(self.CP, self.params) if ebp_code else None
    self.nsb_enabled = self.params.get_bool("NoServiceBrakeHold")
    self.nsb_floor = read_decel_floor(self.params)
    self.speed, self.a_ego, self.v_set, self.gear, self.grade = speed, 0., v_set, gear, grade
    self.true_scale, self.pitch, self.pitch_bias, self.force_decel, self.pedal = true_scale, pitch, pitch_bias, \
      force_decel, pedal
    self.lead = dict(lead) if lead else None
    self.distance, self.t = 0., 0.
    self.lead_pos = self.lead["gap"] if self.lead else 0.
    self.lead_v = self.lead["v"] if self.lead else 0.
    self.log = []
    self.brake = 0.
    self.plant = plant

  def lead_decel(self):
    decel = 0.
    for t_start, d in self.lead.get("profile", ()):
      if self.t >= t_start:
        decel = d
    return decel

  def car_state(self):
    CS = messaging.new_message('carState').carState
    CS.vEgo, CS.aEgo = float(self.speed), float(self.a_ego)
    CS.vCruise = self.v_set * 3.6
    CS.deprecated.engineRpm = float(max(engine_rpm_at(self.gear, self.speed), 800.)) if self.gear else 800.
    CS.deprecated.clutchPressed = self.gear == 0 or self.pedal in (CLUTCH_PEDAL_TRAVEL, CLUTCH_PEDAL_FLOOR)
    return CS

  def pitch_estimate(self):
    return float(self.grade + self.pitch_bias) if self.pitch else None

  def radar(self):
    radar = messaging.new_message('radarState').radarState
    if self.lead:
      lead = log.RadarState.LeadData.new_message()
      lead.dRel = float(max(self.lead_pos - self.distance, 0.))
      lead.vLead = lead.vLeadK = float(self.lead_v)
      lead.vRel = float(self.lead_v - self.speed)
      lead.aLeadK = float(-self.lead_decel() if self.lead_v > 0 else 0.)
      lead.aLeadTau = float(_LEAD_ACCEL_TAU)
      lead.present, lead.modelProb, lead.radar = True, 1.0, True
      radar.leadOne = lead
      radar.leadTwo = lead
    return radar

  def planner_sm(self, radar):
    control = messaging.new_message('controlsState').controlsState
    control.longControlState = LongCtrlState.pid
    control.forceDecel = self.force_decel
    cc = messaging.new_message('carControl').carControl
    cc.longActive = True  # (not .enabled: the SCC vision controller would need a full model message)
    if self.pitch:
      cc.orientationNED = [0., float(self.grade + self.pitch_bias), 0.]
    model = messaging.new_message('modelV2').modelV2
    model.position.x = [float(x) for x in (self.speed + 0.5) * np.array(ModelConstants.T_IDXS)]
    model.velocity.x = [float(self.speed)] * len(ModelConstants.T_IDXS)
    model.acceleration.x = [0.] * len(ModelConstants.T_IDXS)
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    sm = {'radarState': radar, 'carState': self.car_state(), 'controlsState': control, 'carControl': cc, 'modelV2': model}
    for s in ('selfdriveState', 'vehicleParameters', 'carStateSP', 'liveMapDataSP', 'gpsLocation'):
      sm[s] = getattr(messaging.new_message(s), s)
    sm['carStateSP'].clutchSignalValid, sm['carStateSP'].gearRaw = True, self.gear   # live GEARBOX_ALT_2
    sm['carStateSP'].clutchPedal = self.pedal
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
    """The car as Stage 2 measured it (route 00000004--9b8c913b70): the VSA brakes whatever the car would do beyond the
    request - it tracks the request, on the flat and downhill:
      - a negative request (< -ZERO_TORQUE_BAND) closes the throttle; the car slows at the stronger of its TRUE
        closed-throttle decel and the request, the brakes making up any difference;
      - a request within the band holds ~zero axle torque (no engine braking: -0.01 -> -0.20 flat); on a descent the
        VSA brakes to it (descents, requests ~0 and small positive: held, with ~0.2-0.3 m/s^2 of braking);
      - a positive request: throttle up to it, or closed throttle + brakes if the car would exceed it.
    plant="drift" (synthetic, for the runaway backstop): no braking for requests >= -ZERO_TORQUE_BAND (the car rolls).
    In neutral there is no engine braking and no throttle. self.brake = the service braking used."""
    a_grade = -G * math.sin(self.grade)
    a_road = -road_load(self.speed) + a_grade
    d_true = table_decel(STATIC_DECEL, self.gear, self.speed) * self.true_scale if self.gear else road_load(self.speed)
    a_closed = -d_true + a_grade
    if cmd < -ZERO_TORQUE_BAND:
      self.brake = max(a_closed - cmd, 0.)
      return min(cmd, a_closed)
    a_free = a_road if cmd <= ZERO_TORQUE_BAND or not self.gear else max(cmd, a_closed)   # the powertrain alone
    if self.plant == "drift":
      self.brake = 0.
      return a_free
    self.brake = max(a_free - cmd, 0.)
    return min(cmd, a_free)

  def run(self, seconds):
    for _ in range(round(seconds / DT_MDL)):
      radar = self.radar()
      self.planner.update(self.planner_sm(radar))
      plan = self.plan_msg()
      for _ in range(round(DT_MDL / DT_CTRL)):
        CS = self.car_state()
        limits = apply_brake_floor((ACCEL_MIN, ACCEL_MAX), plan.allowBrake, True, self.nsb_enabled, self.nsb_floor)
        active = False
        urgency = urgency_for(CS, plan, radar, force_decel=self.force_decel)
        if self.ebp is not None:
          active = self.ebp.update(True, True, CS, plan, radar, urgency, self.loc.long_control_state,
                                   self.pitch_estimate(), True, self.gear, self.pedal)
          limits = self.ebp.limit_accel(limits)
        cmd = float(self.loc.update(True, CS, plan.aTarget, plan.shouldStop, limits))
        self.a_ego += (self.car_accel(cmd) - self.a_ego) * DT_CTRL / ACTUATOR_TAU
        self.speed = max(self.speed + self.a_ego * DT_CTRL, 0.)
        self.distance += self.speed * DT_CTRL
        if self.lead:
          self.lead_v = max(self.lead_v - self.lead_decel() * DT_CTRL, 0.)
          self.lead_pos += self.lead_v * DT_CTRL
        self.t += DT_CTRL
        self.log.append({"t": self.t, "cmd": cmd, "brake": self.brake, "speed": self.speed, "a_target": plan.aTarget,
                         "active": active, "floor": self.ebp.floor if self.ebp else None, "lower": limits[0],
                         "gap": self.lead_pos - self.distance if self.lead else None, "fcw": plan.fcw,
                         "reason": self.ebp.reason if self.ebp else None, "urgency": urgency,
                         "gear": self.ebp.gear if self.ebp else None})
    return self.log


def braked(log):
  return [l["brake"] for l in log if l["brake"] > BRAKE_EPS]


def brake_integral(log):
  return sum(l["brake"] for l in log) * DT_CTRL


# Scenarios
SET_DROP = {"speed": 25., "v_set": 20., "gear": 6}           # driver lowers the set speed 5 m/s: stock brakes at -1.2
TRIM = {"speed": 25., "v_set": 24.5, "gear": 5}              # a small trim
DESCENT_MILD = {"speed": 20., "v_set": 18., "gear": 4, "grade": -0.02}   # ~1.1 deg: within 4th's engine braking
DESCENT_STEEP = {"speed": 20., "v_set": 20., "gear": 6, "grade": -0.06}  # ~3.4 deg: past 6th's engine braking
FAST_CLOSING = {"speed": 25., "v_set": 25., "gear": 6, "lead": {"gap": 70., "v": 15.}}
LEAD_BRAKES = {"speed": 25., "v_set": 27., "gear": 4, "lead": {"gap": 45., "v": 25., "profile": [(2., 0.3), (8., 3.0)]}}


class TestTable(OpenpilotTestCase):
  def test_floor_math(self):
    # flat: 80 % of the measured decel; downhill the grade comes off it; capped just below 0
    d = table_decel(STATIC_DECEL, 4, 17.)
    self.assertAlmostEqual(d, STATIC_DECEL[4][3], msg="17 m/s is a bin centre")
    self.assertAlmostEqual(brake_free_floor(d, 0.), -MARGIN * d)
    self.assertAlmostEqual(brake_free_floor(d, -0.02), -MARGIN * d + G * math.sin(0.02))
    self.assertGreater(brake_free_floor(d, -0.1), FLOOR_MAX, "steeper than engine braking: not usable")
    self.assertLess(brake_free_floor(d, 0.05), -MARGIN * d, "uphill: more decel allowed (gravity helps)")
    self.assertEqual(brake_free_floor(-1., 0.), 0.)

  def test_table_interpolates_and_holds_ends(self):
    self.assertAlmostEqual(table_decel(STATIC_DECEL, 5, 19.), (STATIC_DECEL[5][3] + STATIC_DECEL[5][4]) / 2)
    self.assertAlmostEqual(table_decel(STATIC_DECEL, 3, 60.), STATIC_DECEL[3][-1])
    self.assertEqual(table_decel(STATIC_DECEL, 7, 20.), 0.)
    self.assertEqual(table_decel(STATIC_DECEL, 0, 20.), 0.)
    self.assertEqual(table_decel(STATIC_DECEL, 4, float('nan')), 0.)
    for row in STATIC_DECEL.values():
      self.assertTrue(all(0. <= x < 1.2 for x in row))

  def test_learned_never_below_static_and_off_by_default(self):
    self.assertFalse(ebp_mod.APPLY_LEARNED, "learned values are reviewed before they are applied")
    learned = {6: [x + 0.1 for x in STATIC_DECEL[6]], 4: [0.] * 8}
    off = EngineBrakeTable(learned, apply_learned=False)
    self.assertEqual(off.decel(6, 21.), table_decel(STATIC_DECEL, 6, 21.))
    on = EngineBrakeTable(learned, apply_learned=True)
    self.assertAlmostEqual(on.decel(6, 21.), table_decel(STATIC_DECEL, 6, 21.) + 0.1)
    self.assertEqual(on.decel(4, 17.), table_decel(STATIC_DECEL, 4, 17.), "static is the floor")

  def test_gear_trust(self):
    rpm4 = engine_rpm_at(4, 20.)
    self.assertEqual(trusted_gear(4, CLUTCH_PEDAL_UP, False, rpm4, 20.), 4)
    for args, why in (((0, CLUTCH_PEDAL_UP, False, rpm4, 20.), "GEAR_MT 0"),
                      ((14, CLUTCH_PEDAL_UP, False, rpm4, 20.), "reverse"),
                      ((-1, CLUTCH_PEDAL_UP, False, rpm4, 20.), "no clutch signal"),
                      ((4, CLUTCH_PEDAL_TRAVEL, False, rpm4, 20.), "pedal moving"),
                      ((4, CLUTCH_PEDAL_FLOOR, False, rpm4, 20.), "pedal down"),
                      ((4, CLUTCH_PEDAL_UNKNOWN, False, rpm4, 20.), "pedal switch not alive"),
                      ((4, CLUTCH_PEDAL_UP, True, rpm4, 20.), "decoupled (carstate phantom guard)"),
                      ((4, CLUTCH_PEDAL_UP, False, rpm4 * 1.07, 20.), "ratio 7 % off"),
                      ((5, CLUTCH_PEDAL_UP, False, rpm4, 20.), "ratio is 4th's, GEAR_MT says 5th"),
                      ((6, CLUTCH_PEDAL_UP, False, 850., 10.), "phantom territory: idle rpm, low speed"),
                      ((4, CLUTCH_PEDAL_UP, False, float('nan'), 20.), "no rpm")):
      with self.subTest(why=why):
        self.assertEqual(trusted_gear(*args), 0)
    # a real low-rpm gear above the phantom speed is trusted (6th at 14 m/s ~ 1190 rpm)
    self.assertEqual(trusted_gear(6, CLUTCH_PEDAL_UP, False, engine_rpm_at(6, 14.), 14.), 6)

  def test_phantom_constants_mirror_opendbc(self):
    self.assertEqual(ebp_mod.PHANTOM_RPM_MAX, honda_carstate.PHANTOM_RPM_MAX)
    self.assertEqual(ebp_mod.PHANTOM_MAX_SPEED, honda_carstate.PHANTOM_MAX_SPEED)


class TestEngineBrakePreferred(OpenpilotTestCase):
  def test_defaults_on(self):
    # Stage 3 (owner, 2026-10-21): ON by default. Params.get_bool ignores registered defaults, so an unset key must
    # still latch the clamp on - in controlsd and in plannerd's learner (which logs engaged frames only while it's on)
    p = Params()
    p.remove("EngineBrakePreferred")
    self.assertTrue(p.get("EngineBrakePreferred", return_default=True))
    self.assertAlmostEqual(p.get("EngineBrakeHardFloor", return_default=True), -1.5)
    CP = make_cp()[0]
    ebp = EngineBrakePreferred(CP, p)
    ebp.read_params()
    ebp._latch()
    self.assertTrue(ebp.enabled, "an unset key latches ON")
    p.put_bool("EngineBrakePreferred", False, block=True)
    ebp.read_params()
    ebp._latch()
    self.assertFalse(ebp.enabled, "OFF when the driver turns it off")
    automatic = EngineBrakePreferred(make_cp(TransmissionType.automatic)[0], p)
    p.remove("EngineBrakePreferred")
    automatic.read_params()
    automatic._latch()
    self.assertFalse(automatic.enabled, "default ON is still manual-only")

  def test_set_speed_drop_slows_on_engine_braking_without_brakes(self):
    for non_adaptive in (False, True):
      with self.subTest(non_adaptive=non_adaptive):
        setup(ebp=False, non_adaptive=non_adaptive)
        off = ManualSim(**SET_DROP).run(15.)
        self.assertGreater(brake_integral(off), 1.0, "stock brakes to the new set speed")
        setup(ebp=True, non_adaptive=non_adaptive)
        on = ManualSim(**SET_DROP).run(15.)
        self.assertTrue(any(l["active"] and l["gear"] == 6 for l in on))
        self.assertEqual(braked(on), [], "no service braking: the request never goes past 6th's engine braking")
        d6 = table_decel(STATIC_DECEL, 6, 22.)
        self.assertLess(on[-1]["speed"], SET_DROP["speed"] - 0.8 * d6 * 10., "it still slows, at engine braking")
        self.assertGreater(min(l["lower"] for l in on if l["active"]), -MARGIN * 0.45 - 1e-6)

  def test_small_trim_unchanged(self):
    # a trim weaker than engine braking was already brake-free (Task 16): the clamp doesn't change that
    setup(ebp=True, non_adaptive=True)
    on = ManualSim(**TRIM).run(10.)
    self.assertEqual(braked(on), [])
    self.assertLess(on[-1]["speed"], TRIM["speed"] - 0.2)

  def test_mild_descent_no_brakes(self):
    setup(ebp=False, non_adaptive=True)
    off = ManualSim(**DESCENT_MILD).run(15.)
    self.assertGreater(brake_integral(off), 0.5)
    setup(ebp=True, non_adaptive=True)
    on = ManualSim(**DESCENT_MILD).run(15.)
    self.assertEqual(braked(on), [], "the grade term keeps the request inside engine braking downhill")
    self.assertLess(on[-1]["speed"], DESCENT_MILD["speed"], "still slowing on engine braking")

  def test_flat_floor_alone_would_brake_downhill(self):
    # why the grade term is there: a flat-road floor on the same descent puts the request past what the closed throttle
    # achieves, and the VSA (which tracks the request) brakes the difference
    d = table_decel(STATIC_DECEL, 4, 19.)
    a_closed = -d + G * math.sin(0.02)
    self.assertLess(-MARGIN * d, a_closed - 0.1)
    self.assertGreater(brake_free_floor(d, -0.02), a_closed)

  def test_steep_descent_steps_aside(self):
    # past 6th's engine braking the brakes are needed to hold set speed whatever the request: the clamp steps aside
    setup(ebp=False, non_adaptive=True)
    off = ManualSim(**DESCENT_STEEP).run(15.)
    setup(ebp=True, non_adaptive=True)
    on = ManualSim(**DESCENT_STEEP).run(15.)
    self.assertTrue(all(not l["active"] and l["reason"] == "descent" for l in on[50:]))
    first = next(i for i, l in enumerate(on) if l["reason"] == "descent")
    self.assertLess(first, 5)
    self.assertEqual([l["cmd"] for l in on[first:]][:500], [l["cmd"] for l in off[first:]][:500], "stock")

  def test_pitch_error(self):
    # the margin is 20 % of D (~0.09 m/s^2 in 4th at 20 m/s ~ 0.5 deg): a 0.4 deg flatter-than-real estimate stays
    # brake-free; a 1 deg one brakes a little (at most the pitch error's share)
    setup(ebp=True, non_adaptive=True)
    small = ManualSim(**DESCENT_MILD, pitch_bias=0.4 * DEG).run(15.)
    self.assertEqual(braked(small), [])
    big = ManualSim(**DESCENT_MILD, pitch_bias=1.0 * DEG).run(15.)
    self.assertLess(max(braked(big), default=0.), G * math.sin(1.0 * DEG) - (1 - MARGIN) * 0.4 + 0.03)
    # the other direction (the estimate steeper than real) only makes the request weaker - or, past FLOOR_MAX, hands
    # the descent to stock: never more braking than stock
    steeper = ManualSim(**DESCENT_MILD, pitch_bias=-1.0 * DEG).run(15.)
    setup(ebp=False, non_adaptive=True)
    off = ManualSim(**DESCENT_MILD).run(15.)
    self.assertLessEqual(brake_integral(steeper), brake_integral(off) + 1e-6)

  def test_table_over_reading_brakes_lightly(self):
    # if the car's real engine braking is below 80 % of the table, the floor is past it and the VSA brakes the gap
    setup(ebp=True, non_adaptive=True)
    ok = ManualSim(**SET_DROP, true_scale=0.85).run(10.)
    self.assertEqual(braked(ok), [])
    low = ManualSim(**SET_DROP, true_scale=0.6).run(10.)
    self.assertLess(max(braked(low)), (MARGIN - 0.6) * 0.45 + 0.03)

  def test_untrusted_gear_coast_only(self):
    setup(ebp=True, non_adaptive=True)
    for kw, why in (({"pedal": CLUTCH_PEDAL_UNKNOWN}, "pedal switch not alive"),
                    ({"pedal": CLUTCH_PEDAL_FLOOR}, "clutch in"),
                    ({"gear": 0}, "neutral")):
      with self.subTest(why=why):
        log = ManualSim(**{**SET_DROP, **kw}).run(4.)
        self.assertTrue(all(l["active"] and l["gear"] == 0 and l["floor"] == UNKNOWN_GEAR_FLOOR for l in log))
        self.assertTrue(all(l["cmd"] >= UNKNOWN_GEAR_FLOOR - 1e-6 for l in log), "coast only: no decel request")
        self.assertEqual(braked(log), [])

  def test_overspeed_backstop(self):
    # neutral rolling down a descent: floor 0 can't slow it; past set + OVERSPEED_MARGIN the clamp lets go, braking
    # returns, and it re-arms only below set + half the margin
    setup(ebp=True, non_adaptive=True)
    sim = ManualSim(speed=20., v_set=20., gear=0, grade=-0.05, plant="drift")   # a car that doesn't hold 0 downhill
    log = sim.run(25.)
    first = next(i for i, l in enumerate(log) if l["reason"] == "overspeed")
    self.assertLess(log[first]["speed"], 20. + OVERSPEED_MARGIN + 0.5 * 2.5, "after OVERSPEED_TIME over the margin")
    self.assertFalse(any(l["reason"] == "overspeed" for l in ManualSim(**SET_DROP).run(10.)),
                     "5 m/s over after a set-speed drop but slowing on engine braking: not a runaway")
    self.assertGreater(sum(braked(log[first:])), 0.)
    self.assertLess(max(l["speed"] for l in log), 20. + OVERSPEED_MARGIN + 1.5)
    rearm = next((l for l in log[first:] if l["active"]), None)
    if rearm is not None:
      self.assertLess(rearm["speed"], 20. + OVERSPEED_MARGIN / 2 + 0.01)

  def test_fast_closing_lead_full_braking_passes(self):
    setup(ebp=False)
    off = ManualSim(**FAST_CLOSING).run(10.)
    setup(ebp=True)
    on = ManualSim(**FAST_CLOSING).run(10.)
    first = next((i for i, l in enumerate(on) if l["active"]), len(on))
    self.assertGreater(on[first - 1]["t"], 6., "no clamp while the lead closes fast")
    self.assertEqual([l["cmd"] for l in on[:first]], [l["cmd"] for l in off[:first]], "stock until then")
    self.assertLess(min(l["cmd"] for l in on), -1.0)

  def test_lead_starts_braking_hands_off(self):
    setup(ebp=True)
    on = ManualSim(**LEAD_BRAKES).run(14.)
    setup(ebp=False)
    off = ManualSim(**LEAD_BRAKES).run(14.)
    t_brake = LEAD_BRAKES["lead"]["profile"][1][0]
    self.assertFalse(any(l["active"] for l in on if l["t"] > t_brake + 0.1))
    first_on = next(l["t"] for l in on if l["cmd"] < -0.8)
    first_off = next(l["t"] for l in off if l["cmd"] < -0.8)
    self.assertLess(first_on - first_off, 0.3)
    self.assertGreater(min(l["gap"] for l in on), min(l["gap"] for l in off) - 1.0)

  def test_urgency_releases_in_the_same_frame(self):
    # Rule 5: every reason, the first frame it appears, leaves the lower limit alone (and a hold follows)
    setup(ebp=True, non_adaptive=True)
    sim = ManualSim(**SET_DROP)
    sim.run(2.)
    CS, radar, plan = sim.car_state(), sim.radar(), sim.plan_msg()
    ebp = sim.ebp
    self.assertTrue(ebp.update(True, True, CS, plan, radar, None, LongCtrlState.pid, 0., True, 6, CLUTCH_PEDAL_UP))
    for reason in ("fcw", "force_decel", "stock_aeb", "hard_decel", "invalid_plan", "lead_close"):
      with self.subTest(reason=reason):
        ebp.hold_timer = 0.
        self.assertTrue(ebp.update(True, True, CS, plan, radar, None, LongCtrlState.pid, 0., True, 6, CLUTCH_PEDAL_UP))
        self.assertFalse(ebp.update(True, True, CS, plan, radar, reason, LongCtrlState.pid, 0., True, 6, CLUTCH_PEDAL_UP))
        self.assertEqual(ebp.limit_accel((-3.5, 2.0)), (-3.5, 2.0))
        self.assertEqual(ebp.hold_timer, RELEASE_HOLD)
    n = 0
    while not ebp.update(True, True, CS, plan, radar, None, LongCtrlState.pid, 0., True, 6, CLUTCH_PEDAL_UP):
      n += 1
    self.assertAlmostEqual(n * DT_CTRL, RELEASE_HOLD, delta=2 * DT_CTRL)

  def test_blocked_inputs(self):
    setup(ebp=True)
    sim = ManualSim(**SET_DROP)
    sim.run(1.)
    CS, radar, plan = sim.car_state(), sim.radar(), sim.plan_msg()
    ebp = sim.ebp
    for kw, reason in (({"pitch": None}, "no_pitch"), ({"pitch": float('nan')}, "no_pitch"),
                       ({"clutch_valid": False}, "no_clutch_signal"), ({"lcs": LongCtrlState.stopping}, "stopping"),
                       ({"long_active": False}, "inactive")):
      with self.subTest(reason=reason, kw=kw):
        a = {"pitch": 0., "clutch_valid": True, "lcs": LongCtrlState.pid, "long_active": True, **kw}
        self.assertFalse(ebp.update(True, a["long_active"], CS, plan, radar, None, a["lcs"], a["pitch"],
                                    a["clutch_valid"], 6, CLUTCH_PEDAL_UP))
        self.assertEqual(ebp.reason, reason)
        self.assertEqual(ebp.limit_accel((-3.5, 2.0)), (-3.5, 2.0))
    CS.vEgo = MIN_SPEED - 0.1
    self.assertFalse(ebp.update(True, True, CS, plan, radar, None, LongCtrlState.pid, 0., True, 6, CLUTCH_PEDAL_UP))
    self.assertEqual(ebp.reason, "slow")
    # a hard decel planned within the next second rules it out
    CS.vEgo = 25.
    plan.accels = [-0.3, -0.8, -1.6] + [-2.0] * (len(ModelConstants.T_IDXS) - 3)
    self.assertFalse(ebp.update(True, True, CS, plan, radar, None, LongCtrlState.pid, 0., True, 6, CLUTCH_PEDAL_UP))
    self.assertEqual(ebp.reason, "hard_floor")
    # an upstream upper limit below the floor is never widened
    ebp.active, ebp.floor = True, -0.3
    self.assertEqual(ebp.limit_accel((-3.5, -0.5)), (-3.5, -0.5))

  def test_force_decel_never_clamped(self):
    setup(ebp=True, non_adaptive=True)
    log = ManualSim(force_decel=True, **SET_DROP).run(5.)
    self.assertFalse(any(l["active"] for l in log))

  def test_toggle_off_identical_to_stock(self):
    for scenario in (SET_DROP, DESCENT_MILD, FAST_CLOSING, LEAD_BRAKES):
      with self.subTest(scenario=scenario):
        setup(ebp=False, non_adaptive=scenario is not FAST_CLOSING)
        off = ManualSim(**scenario).run(8.)
        stock = ManualSim(ebp_code=False, **scenario).run(8.)
        self.assertEqual([(l["cmd"], l["speed"]) for l in off], [(l["cmd"], l["speed"]) for l in stock])

  def test_nsb_composition_takes_max_floor(self):
    setup(ebp=True, non_adaptive=True, nsb=True, NoServiceBrakeOverspeedBackstop=False)
    log = ManualSim(**SET_DROP).run(10.)
    self.assertTrue(any(l["active"] for l in log))
    for l in log:
      self.assertGreaterEqual(l["lower"], -0.3 - 1e-9)
      if l["active"]:
        self.assertAlmostEqual(l["lower"], max(-0.3, l["floor"]))
    self.assertEqual(braked(log), [])

  def test_unavailable_on_automatic_and_stock_acc(self):
    setup(ebp=True, non_adaptive=True)
    for kw in ({"transmission": TransmissionType.automatic}, {"op_long": False}):
      with self.subTest(**kw):
        sim = ManualSim(**kw)
        self.assertFalse(sim.ebp.available)
        self.assertFalse(sim.ebp.enabled)

  def test_latch_only_while_disengaged(self):
    setup(ebp=False, non_adaptive=True)
    sim = ManualSim(**SET_DROP)
    sim.run(1.)
    Params().put_bool("EngineBrakePreferred", True)
    sim.ebp.read_params()
    sim.run(3.)
    self.assertFalse(sim.ebp.enabled, "must not switch on mid-engagement")
    sim.ebp.update(False, False, sim.car_state(), sim.plan_msg(), sim.radar(), None, LongCtrlState.off, 0.)
    self.assertTrue(sim.ebp.enabled)

  def test_logs_binding_edges(self):
    setup(ebp=True, non_adaptive=True)
    events = []
    with mock.patch.object(ebp_mod.cloudlog, "event", side_effect=lambda name, **kw: events.append((name, kw))):
      log = ManualSim(**SET_DROP).run(15.)
    binds = [kw for name, kw in events if name == "engine_brake_clamp"]
    self.assertTrue(binds and binds[0]["binding"] and binds[0]["gear"] == 6)
    self.assertTrue(all(k in binds[0] for k in ("v_ego", "floor", "a_target", "a_ego", "pitch")))
    self.assertLessEqual(len(binds), 6, "edges only, not every frame")
    self.assertTrue(any(name == "engine_brake_preferred" and kw["active"] for name, kw in events))
    self.assertGreater(sum(l["a_target"] < l["floor"] for l in log if l["active"]), 100)

  def test_lead_urgency(self):
    lead = log.RadarState.LeadData.new_message()
    lead.present, lead.dRel, lead.vRel, lead.aLeadK = True, 50., -0.3, 0.
    self.assertFalse(lead_urgent(lead, 25., 6., 1.))
    lead.vRel = -1.2
    self.assertTrue(lead_urgent(lead, 25., 6., 1.))
    lead.vRel, lead.dRel = -0.9, 5.
    self.assertTrue(lead_urgent(lead, 25., 6., 1.))
    lead.present = False
    self.assertFalse(lead_urgent(lead, 25., 6., 1.))

  def test_params_clamped(self):
    CP, _ = make_cp()
    setup(ebp=True, EngineBrakeHardFloor=-9.0, EngineBrakeLeadTtc=0.5, EngineBrakeLeadClosingSpeed=50.)
    ebp = EngineBrakePreferred(CP, Params())
    self.assertEqual(ebp.hard_floor, HARD_FLOOR_RANGE[0])
    self.assertEqual(ebp.lead_ttc, LEAD_TTC_RANGE[0])
    self.assertEqual(ebp.lead_closing, 3.0)

  def test_coast_target_param_retired(self):
    with self.assertRaises(UnknownKeyName):
      Params().get("EngineBrakeCoastTarget")


class TestDescentHandOff(OpenpilotTestCase):
  """Owner (2026-10-20): when the gear's engine braking gives up on a descent, the no-service-brake sawtooth should take
  over, not plain service braking. Structurally it does: the clamp steps aside ("descent") and the floors compose by max,
  so NoServiceBrakeHold's coast floor / overspeed backstop is what applies. FINDING pinned here: on the car as Stage 2
  measured it, that coast floor is braked to as well (the VSA holds -0.3, and even 0), so on a descent steeper than
  engine braking the car never drifts up to the overspeed trigger - it holds its speed on the brakes. A brake-free coast
  phase needs a request at the car's own closed-throttle accel (positive there): the NSB coast-follow item."""

  def test_steep_descent_nsb_holds_on_the_brakes(self):
    for floor in (0.3, 0.0):
      with self.subTest(nsb_floor=floor):
        setup(ebp=True, non_adaptive=True, nsb=True, NoServiceBrakeDecelFloor=floor)
        log = ManualSim(**DESCENT_STEEP).run(20.)
        self.assertTrue(all(l["reason"] == "descent" for l in log[50:]), "the clamp hands the descent on")
        self.assertTrue(all(l["lower"] >= -floor - 1e-6 for l in log[50:]), "NSB's coast floor is the limit")
        self.assertLess(max(l["speed"] for l in log), DESCENT_STEEP["v_set"] + 1.0, "no drift toward the trigger")
        self.assertGreater(brake_integral(log[50:]), 2.0, "held on the brakes instead (needs coast-follow)")
    # on a car that does not hold 0 downhill the same setup drifts over set speed with the brakes off
    setup(ebp=True, non_adaptive=True, nsb=True, NoServiceBrakeDecelFloor=0.0, NoServiceBrakeOverspeedBackstop=False)
    drift = ManualSim(**DESCENT_STEEP, plant="drift").run(20.)
    self.assertGreater(drift[-1]["speed"], DESCENT_STEEP["v_set"] + 3.)
    self.assertEqual(braked(drift), [])
