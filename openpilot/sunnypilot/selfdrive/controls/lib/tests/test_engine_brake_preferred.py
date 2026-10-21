"""
EngineBrakePreferred, closed loop: LongitudinalPlanner (adaptive or non-adaptive) -> controlsd's brake-range clamps
(no-service-brake floor, engine-brake-preferred) -> LongControl -> a simple manual car that engine-brakes whenever the
throttle is closed (negative command = service brakes on top of that), with an optional lead and grade.
"""
import math

import numpy as np

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import braking_urgency
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import G, engine_brake_decel, engine_rpm_at, \
  estimate_coast_decel, resistive_decel
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import EngineBrakePreferred, lead_urgent, \
  CAPACITY_TIME, RELEASE_HOLD, DEFAULT_HARD_FLOOR, HARD_FLOOR_RANGE, LEAD_TTC_RANGE, COAST_TARGET_RANGE, DEFAULT_COAST_TARGET
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import apply_brake_floor, read_decel_floor
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_manual_low_speed_handback import make_cp, TransmissionType

ZERO_TORQUE_BAND = 0.05   # m/s^2: Stage 2 C2 - a request this close to 0 holds ~zero axle torque (no engine braking)
BRAKE_EPS = 0.02          # m/s^2: service braking below this counts as none
ACTUATOR_TAU = 0.25  # s


def setup(ebp=False, non_adaptive=False, nsb=False, **params):
  p = Params()
  p.put_bool("EngineBrakePreferred", ebp)
  p.put_bool("NonAdaptiveLongitudinal", non_adaptive)
  p.put_bool("NoServiceBrakeHold", nsb)
  for k in ("EngineBrakeHardFloor", "EngineBrakeCoastTarget", "EngineBrakeLeadTtc", "EngineBrakeLeadClosingSpeed",
            "NoServiceBrakeDecelFloor", "NoServiceBrakeOverspeedBackstop"):
    p.remove(k)
  for k, v in params.items():
    p.put(k, v)


def urgency_for(CS, plan, radar, plan_ok=True, radar_ok=True, force_decel=False):
  """What controlsd's controls_urgency() computes for these messages (Rule 5)."""
  return braking_urgency(plan_ok=plan_ok, radar_ok=radar_ok, force_decel=force_decel, fcw=plan.fcw,
                         stock_aeb=CS.stockAeb or CS.stockFcw, should_stop=plan.shouldStop, a_target=plan.aTarget,
                         v_ego=CS.vEgo, radar=radar)


def engaged(ebp, CS, plan, radar):
  """One engaged controlsd frame, LoC in pid, flat road, urgency from the messages."""
  return ebp.update(True, True, CS, plan, radar, urgency_for(CS, plan, radar), LongCtrlState.pid, 0.)


class ManualSim:
  """lead: dict(gap m, v m/s, profile [(t_start s, decel m/s^2), ...]). true_eb_scale: real engine braking vs model's.
  The car encodes the design's assumption: command 0 = closed-throttle coast (engine braking), negative = service
  brakes on top of it, a small positive command opens the throttle enough to cancel engine braking."""
  def __init__(self, speed=25., v_set=25., gear=6, lead=None, grade=0., true_eb_scale=1.0, pitch=True,
               ebp_code=True, transmission=TransmissionType.manual, op_long=True, force_decel=False, plant="measured"):
    self.CP, self.CP_SP = make_cp(transmission, op_long)
    self.params = Params()
    self.planner = LongitudinalPlanner(self.CP, self.CP_SP, init_v=speed)
    self.loc = LongControl(self.CP, self.CP_SP)
    self.loc.long_control_state = LongCtrlState.pid  # already engaged and cruising
    self.ebp = EngineBrakePreferred(self.CP, self.params) if ebp_code else None
    self.nsb_enabled = self.params.get_bool("NoServiceBrakeHold")
    self.nsb_floor = read_decel_floor(self.params)
    self.speed, self.a_ego, self.v_set, self.gear, self.grade = speed, 0., v_set, gear, grade
    self.true_eb_scale, self.pitch, self.force_decel = true_eb_scale, pitch, force_decel
    self.lead = dict(lead) if lead else None
    self.distance, self.t = 0., 0.
    self.lead_pos = self.lead["gap"] if self.lead else 0.
    self.lead_v = self.lead["v"] if self.lead else 0.
    self.log = []
    self.brake = 0.
    self.plant = plant   # "measured" (Stage 2, the default) | "force_additive" (synthetic: a car that under-delivers)

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
    CS.deprecated.engineRpm = float(engine_rpm_at(self.gear, self.speed)) if self.gear else 800.
    CS.deprecated.clutchPressed = self.gear == 0
    return CS

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
      cc.orientationNED = [0., float(self.grade), 0.]
    model = messaging.new_message('modelV2').modelV2
    model.position.x = [float(x) for x in (self.speed + 0.5) * np.array(ModelConstants.T_IDXS)]
    model.velocity.x = [float(self.speed)] * len(ModelConstants.T_IDXS)
    model.acceleration.x = [0.] * len(ModelConstants.T_IDXS)
    model.meta.disengagePredictions.gasPressProbs = [1.] * 6
    sm = {'radarState': radar, 'carState': self.car_state(), 'controlsState': control, 'carControl': cc, 'modelV2': model}
    for s in ('selfdriveState', 'vehicleParameters', 'carStateSP', 'liveMapDataSP', 'gpsLocation'):
      sm[s] = getattr(messaging.new_message(s), s)
    sm['carStateSP'].clutchSignalValid, sm['carStateSP'].gearRaw = True, self.gear   # live GEARBOX_ALT_2
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
    """The car as Stage 2 measured it (C2, route 00000004--9b8c913b70) - NOT force-additive (cmd + engine braking):
      - a request within ZERO_TORQUE_BAND of 0, or positive: the PCM holds axle torque for it, so 0 rolls on road load
        only (no engine braking: sent -0.01 -> -0.20 flat, torque -5 Nm);
      - a negative request: closed throttle; the car slows at the stronger of the TRUE engine braking and the request,
        the service brakes making up any difference (-0.17 -> -0.38 engine braking alone; -1.05 -> -1.06 with brakes;
        downhill the VSA tracks the request too).
    true_eb_scale scales only the engine part (how wrong the model is); self.brake = the service braking used."""
    a_road = resistive_decel(self.speed) - G * math.sin(self.grade)
    self.brake = 0.
    if self.plant == "force_additive":
      # the pre-Stage-2 model, kept as a SYNTHETIC car for the backstop-mechanism tests: a negative command adds service
      # braking on top of engine braking + road load (all scaled), so a coast command can leave the car short of the plan
      a_eb = estimate_coast_decel(self.gear, engine_rpm_at(self.gear, self.speed), self.speed, 0.) * self.true_eb_scale
      fade = float(np.clip(1. - max(cmd, 0.) / ZERO_TORQUE_BAND, 0., 1.))
      self.brake = max(-cmd, 0.)
      return cmd + a_eb * fade - G * math.sin(self.grade)
    if cmd >= -ZERO_TORQUE_BAND:
      return cmd + a_road
    a_closed = a_road + engine_brake_decel(self.gear, engine_rpm_at(self.gear, self.speed)) * self.true_eb_scale
    self.brake = max(a_closed - cmd, 0.)
    return min(cmd, a_closed)

  def run(self, seconds):
    for _ in range(round(seconds / DT_MDL)):
      radar = self.radar()
      self.planner.update(self.planner_sm(radar))
      plan = self.plan_msg()
      for _ in range(round(DT_MDL / DT_CTRL)):
        CS = self.car_state()
        limits = apply_brake_floor((ACCEL_MIN, ACCEL_MAX), plan.allowBrake, True, self.nsb_enabled, self.nsb_floor)
        suppress = False
        if self.ebp is not None:
          urgency = urgency_for(CS, plan, radar, force_decel=self.force_decel)
          suppress = self.ebp.update(True, True, CS, plan, radar, urgency, self.loc.long_control_state,
                                     float(self.grade) if self.pitch else None)
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
                         "suppress": suppress,
                         "gap": self.lead_pos - self.distance if self.lead else None, "fcw": plan.fcw,
                         "reason": self.ebp.reason if self.ebp else None, "lower": limits[0],
                         "urgency": urgency_for(CS, plan, radar, force_decel=self.force_decel)})
    return self.log


def brake_cmds(log):
  # frames where the (measured-car) plant used the service brakes - not "the command was negative": on this car a negative
  # request weaker than the gear's engine braking is met by closing the throttle alone (Stage 2 C2)
  return [l["brake"] for l in log if l["brake"] > BRAKE_EPS]


# Scenarios
GENTLE_TRIM = {"speed": 25., "v_set": 24.7, "gear": 6}                      # driver trims set speed: ~-0.3 m/s^2, no lead
# lead at a comfortable gap easing off gently (0.3 m/s^2 for 10 s): follow-distance trim, no urgency
GENTLE_FOLLOW = {"speed": 25., "v_set": 27., "gear": 4, "lead": {"gap": 45., "v": 25., "profile": [(2., 0.3), (12., 0.)]}}
FAST_CLOSING = {"speed": 25., "v_set": 25., "gear": 6, "lead": {"gap": 70., "v": 15.}}
# same gentle follow, then the lead brakes hard (3 m/s^2) at t=8 s
LEAD_BRAKES = {"speed": 25., "v_set": 27., "gear": 4, "lead": {"gap": 45., "v": 25., "profile": [(2., 0.3), (8., 3.0)]}}


class TestEngineBrakePreferred(OpenpilotTestCase):
  def test_defaults_off(self):
    p = Params()
    for k in ("EngineBrakePreferred", "GearAwareAccel"):
      p.remove(k)
      self.assertFalse(p.get_bool(k))
    self.assertAlmostEqual(p.get("EngineBrakeHardFloor", return_default=True), -1.5)
    self.assertAlmostEqual(p.get("EngineBrakeCoastTarget", return_default=True), DEFAULT_COAST_TARGET)

  def test_gentle_trim_coasts_on_engine_braking(self):
    setup(ebp=False, non_adaptive=True)
    off = ManualSim(**GENTLE_TRIM).run(12.)
    self.assertLess(min(l["cmd"] for l in off), -0.05, "stock should command the brakes to trim speed")
    setup(ebp=True, non_adaptive=True)
    on = ManualSim(**GENTLE_TRIM).run(12.)
    self.assertTrue(any(l["suppress"] for l in on))
    self.assertEqual(brake_cmds(on), [], "no brake-range command while engine braking covers the demand")
    self.assertLess(on[-1]["speed"], GENTLE_TRIM["speed"] - 0.2, "the car still slows (engine braking)")
    self.assertLess(on[-1]["speed"], GENTLE_TRIM["v_set"] + 0.15)

  def test_gentle_follow_distance_trim(self):
    setup(ebp=True)
    on = ManualSim(**GENTLE_FOLLOW).run(20.)
    self.assertTrue(any(l["suppress"] for l in on))
    self.assertEqual(brake_cmds(on), [])
    self.assertGreater(min(l["gap"] for l in on), 30., "follow distance maintained without the brakes")
    self.assertFalse(any(l["fcw"] for l in on))

  def test_fast_closing_lead_full_braking_passes(self):
    setup(ebp=False)
    off = ManualSim(**FAST_CLOSING).run(10.)
    setup(ebp=True)
    on = ManualSim(**FAST_CLOSING).run(10.)
    # no suppression while the lead closes >= 1 m/s or any urgency holds (+ the release hold); stock-identical until then
    first = next((i for i, l in enumerate(on) if l["suppress"]), len(on))
    self.assertEqual([l["cmd"] for l in on[:first]], [l["cmd"] for l in off[:first]], "suppression disabled while closing")
    self.assertGreater(on[first - 1]["t"], 8., "nothing suppressed through the closing phase")
    for l in on[first:]:
      if l["suppress"]:
        self.assertLess(l["speed"] - 15., 1.0, "suppressing only once the lead no longer closes fast")
        self.assertIsNone(l["urgency"])
    self.assertLess(min(l["cmd"] for l in on), -1.0)
    closing = [l for l in on if l["gap"] is not None and l["speed"] - 15. > 1.0]
    self.assertTrue(closing and all(l["reason"] in ("lead", "hard_floor", "urgency") for l in closing))

  def test_lead_starts_braking_hands_off_to_brakes(self):
    # coasting behind a comfortable lead, then the lead brakes hard: suppression must end and braking pass through
    setup(ebp=True)
    on = ManualSim(**LEAD_BRAKES).run(14.)
    setup(ebp=False)
    off = ManualSim(**LEAD_BRAKES).run(14.)
    t_brake = LEAD_BRAKES["lead"]["profile"][1][0]
    self.assertTrue(any(l["suppress"] for l in on if l["t"] < t_brake))
    first_brake_on = next(l["t"] for l in on if l["cmd"] < -0.5)
    first_brake_off = next(l["t"] for l in off if l["cmd"] < -0.5)
    self.assertLess(first_brake_on - first_brake_off, 0.3, "brake hand-off must not lag stock meaningfully")
    self.assertFalse(any(l["suppress"] for l in on if l["t"] > t_brake + 0.1))
    self.assertGreater(min(l["gap"] for l in on), min(l["gap"] for l in off) - 1.0)

  def test_demand_below_hard_floor_passes_through(self):
    CP, _ = make_cp()
    setup(ebp=True)
    ebp = EngineBrakePreferred(CP, Params())
    ebp.update(False, False, ManualSim(gear=1).car_state(), None, None, None, LongCtrlState.pid, 0.)
    sim = ManualSim(speed=12., gear=1)
    CS, radar = sim.car_state(), sim.radar()
    plan = messaging.new_message('longitudinalPlan').longitudinalPlan
    for demand in (-0.3, DEFAULT_HARD_FLOOR - 0.1, -3.0):
      ebp.reset()
      plan.aTarget = demand
      plan.accels = [demand] * len(ModelConstants.T_IDXS)
      self.assertEqual(engaged(ebp, CS, plan, radar), demand == -0.3,
                       f"demand {demand}")
      self.assertEqual(ebp.limit_accel((-3.5, 2.0)), (DEFAULT_COAST_TARGET, 2.0) if demand == -0.3 else (-3.5, 2.0))
    # a hard decel planned within the next second also rules it out, even if aTarget itself is gentle
    ebp.reset()
    plan.aTarget = -0.3
    plan.accels = [-0.3, -0.8, -1.6] + [-2.0] * (len(ModelConstants.T_IDXS) - 3)
    self.assertFalse(engaged(ebp, CS, plan, radar))
    self.assertEqual(ebp.reason, "hard_floor")

  def test_toggle_off_identical_to_stock(self):
    for scenario in (GENTLE_TRIM, GENTLE_FOLLOW, FAST_CLOSING, LEAD_BRAKES):
      with self.subTest(scenario=scenario):
        setup(ebp=False, non_adaptive=scenario is GENTLE_TRIM)
        off = ManualSim(**scenario).run(10.)
        stock = ManualSim(ebp_code=False, **scenario).run(10.)
        self.assertEqual([(l["cmd"], l["speed"]) for l in off], [(l["cmd"], l["speed"]) for l in stock])

  def test_model_overestimates_engine_braking_behind_lead(self):
    # THE risk: real engine braking is only 20% of the model's, following a slowing lead. The car coasts and closes;
    # the capacity / demand backstops must hand off to the brakes before the gap gets meaningfully worse than stock.
    setup(ebp=False)
    off = ManualSim(true_eb_scale=0.2, plant="force_additive", **GENTLE_FOLLOW).run(16.)  # stock, same (synthetic) car
    # worst case for the backstops: coast target 0 (the weakest allowed; on the real car ~zero torque, no engine braking -
    # Stage 2 C2). The default -0.2 slows the car more, so the suppression can end without any backstop (checked below)
    setup(ebp=True, EngineBrakeCoastTarget=0.0)
    sim = ManualSim(true_eb_scale=0.2, plant="force_additive", **GENTLE_FOLLOW)
    on = sim.run(16.)
    releases = [l for prev, l in zip(on, on[1:], strict=False) if prev["suppress"] and not l["suppress"]]
    self.assertTrue(releases, "a backstop must end the suppression")
    self.assertTrue({l["reason"] for l in releases} <= {"lead", "urgency", "capacity", "demand"}, releases)
    self.assertTrue(brake_cmds(on), "and real braking then passes")
    first_sup = next(l["t"] for l in on if l["suppress"])
    self.assertLess(releases[0]["t"] - first_sup, 2.0)
    # sim result at the time of writing: min gap 35.9 m vs 38.0 m stock (same car) — the cost of a 5x-optimistic model
    self.assertGreater(min(l["gap"] for l in on), min(l["gap"] for l in off) - 3.0)
    self.assertFalse(any(l["fcw"] for l in on))
    self.assertTrue(sim.ebp.capacity_failed, "and the estimate isn't trusted again this engagement")
    # the default coast target (-0.2, closed throttle) on the same over-optimistic model: never a smaller gap, no FCW
    setup(ebp=True)
    default = ManualSim(true_eb_scale=0.2, plant="force_additive", **GENTLE_FOLLOW).run(16.)
    self.assertGreaterEqual(min(l["gap"] for l in default), min(l["gap"] for l in on) - 0.05)
    self.assertFalse(any(l["fcw"] for l in default))

  def test_coast_command_that_holds_speed_is_caught(self):
    # no engine braking at all at the coast command -> the capacity backstop. Stage 2 C2 showed ACCEL_COMMAND 0 is exactly
    # that on the car (~zero torque), so this pins the coast target to 0 (still allowed) to keep testing the backstop
    setup(ebp=True, non_adaptive=True, EngineBrakeCoastTarget=0.0)
    log = ManualSim(true_eb_scale=0.0, plant="force_additive", **GENTLE_TRIM).run(8.)
    capacity = [l for l in log if l["reason"] == "capacity"]
    self.assertEqual(len(capacity), 1)
    self.assertLess(capacity[0]["t"], CAPACITY_TIME + 0.5)
    self.assertFalse(any(l["suppress"] for l in log if l["t"] > capacity[0]["t"]), "no re-entry this engagement")
    self.assertLess(log[-1]["speed"], GENTLE_TRIM["v_set"] + 0.1, "speed trim completes via the brakes")
    sim = ManualSim(true_eb_scale=0.0, plant="force_additive", **GENTLE_TRIM)
    sim.run(2.)
    self.assertTrue(sim.ebp.capacity_failed)
    sim.ebp.update(False, False, sim.car_state(), sim.plan_msg(), sim.radar(), None, LongCtrlState.off, 0.)
    self.assertFalse(sim.ebp.capacity_failed, "re-engaging clears it")

  def test_downhill_grade_is_counted(self):
    # 3% downhill: gravity eats the engine braking, so the same gentle trim needs the brakes
    setup(ebp=True, non_adaptive=True)
    sim = ManualSim(grade=-0.03, **GENTLE_TRIM)
    log = sim.run(8.)
    self.assertGreater(sim.ebp.a_eb_credited, -0.05, "gravity cancels most of 6th gear's engine braking")
    self.assertLess(min(l["cmd"] for l in log), -0.15, "the trim is braked")
    self.assertFalse(any(l["suppress"] and l["a_target"] < -0.05 for l in log))

  def test_no_pitch_or_no_gear_never_suppresses(self):
    for kw, reason in (({"pitch": False}, "no_pitch"), ({"gear": 0}, "no_gear")):
      with self.subTest(**kw):
        setup(ebp=True, non_adaptive=True)
        log = ManualSim(**{**GENTLE_TRIM, **kw}).run(5.)
        self.assertFalse(any(l["suppress"] for l in log))
        self.assertTrue(any(l["reason"] == reason for l in log))

  def test_force_decel_never_suppressed(self):
    setup(ebp=True, non_adaptive=True)
    log = ManualSim(force_decel=True, **GENTLE_TRIM).run(5.)
    self.assertFalse(any(l["suppress"] for l in log))

  def test_nsb_composition_takes_max_floor(self):
    # both on: lower limit = max(-NSB floor, coast target); never lower than either alone
    setup(ebp=True, non_adaptive=True, nsb=True, NoServiceBrakeOverspeedBackstop=False)
    log = ManualSim(**GENTLE_TRIM).run(10.)
    self.assertTrue(any(l["suppress"] for l in log))
    for l in log:
      self.assertGreaterEqual(l["lower"], -0.3 - 1e-9)
      if l["suppress"]:
        self.assertEqual(l["lower"], max(-0.3, DEFAULT_COAST_TARGET))
    self.assertEqual(brake_cmds(log), [])

  def test_unavailable_on_automatic_and_stock_acc(self):
    setup(ebp=True, non_adaptive=True)
    for kw in ({"transmission": TransmissionType.automatic}, {"op_long": False}):
      with self.subTest(**kw):
        sim = ManualSim(**kw)
        self.assertFalse(sim.ebp.available)
        self.assertFalse(sim.ebp.enabled)

  def test_latch_only_while_long_inactive(self):
    setup(ebp=False, non_adaptive=True)
    sim = ManualSim(**GENTLE_TRIM)
    sim.run(1.)
    Params().put_bool("EngineBrakePreferred", True)
    sim.ebp.read_params()
    sim.run(3.)
    self.assertFalse(sim.ebp.enabled, "must not switch on mid-engagement")
    sim.ebp.update(False, False, sim.car_state(), sim.plan_msg(), sim.radar(), None, LongCtrlState.off, 0.)
    self.assertTrue(sim.ebp.enabled)

  def test_invalid_inputs_and_stopping_never_suppress(self):
    CP, _ = make_cp()
    setup(ebp=True)
    ebp = EngineBrakePreferred(CP, Params())
    sim = ManualSim(**GENTLE_TRIM)
    CS, radar = sim.car_state(), sim.radar()
    plan = messaging.new_message('longitudinalPlan').longitudinalPlan
    plan.aTarget = -0.2
    ok = {"plan_valid": True, "radar_valid": True, "force_decel": False, "lcs": LongCtrlState.pid, "pitch": 0.,
          "clutch_valid": True}
    cases = {"urgency": [{"plan_valid": False}, {"radar_valid": False}, {"force_decel": True}],
             "stopping": [{"lcs": LongCtrlState.stopping}], "no_pitch": [{"pitch": float('nan')}, {"pitch": None}],
             "no_clutch_signal": [{"clutch_valid": False}]}
    for reason, bads in cases.items():
      for bad in bads:
        a = {**ok, **bad}
        ebp.reset()
        urgency = urgency_for(CS, plan, radar, a["plan_valid"], a["radar_valid"], a["force_decel"])
        self.assertFalse(ebp.update(True, True, CS, plan, radar, urgency, a["lcs"], a["pitch"], a["clutch_valid"]), bad)
        self.assertEqual(ebp.reason, reason, bad)
    for field in ("fcw",):
      ebp.reset()
      setattr(plan, field, True)
      self.assertFalse(engaged(ebp, CS, plan, radar))
      setattr(plan, field, False)
    for field in ("stockAeb", "stockFcw"):
      ebp.reset()
      setattr(CS, field, True)
      self.assertFalse(engaged(ebp, CS, plan, radar))
      setattr(CS, field, False)
    ebp.reset()
    self.assertTrue(engaged(ebp, CS, plan, radar), "sanity: valid case")
    # an upstream upper limit below the coast target (forced decel) is never widened
    self.assertEqual(ebp.limit_accel((-3.5, -0.5)), (-3.5, -0.5))

  def test_never_enters_on_demand_steeper_than_engine_braking(self):
    # above the hard floor but beyond what 6th gear's engine braking can supply: braking from the very first frame
    CP, _ = make_cp()
    setup(ebp=True)
    ebp = EngineBrakePreferred(CP, Params())
    sim = ManualSim(**GENTLE_TRIM)
    CS, radar = sim.car_state(), sim.radar()
    plan = messaging.new_message('longitudinalPlan').longitudinalPlan
    plan.aTarget = -1.0
    self.assertFalse(engaged(ebp, CS, plan, radar))
    self.assertLess(-1.0, ebp.a_eb_credited * 2, "sanity: demand well past the credited engine braking")
    self.assertEqual(ebp.limit_accel((-3.5, 2.0)), (-3.5, 2.0))

  def test_release_hold_prevents_chatter(self):
    CP, _ = make_cp()
    setup(ebp=True)
    ebp = EngineBrakePreferred(CP, Params())
    sim = ManualSim(**GENTLE_TRIM)
    CS, radar = sim.car_state(), sim.radar()
    plan = messaging.new_message('longitudinalPlan').longitudinalPlan
    plan.aTarget = -0.2
    self.assertTrue(engaged(ebp, CS, plan, radar))
    plan.aTarget = -1.0  # steeper than engine braking in 6th
    self.assertFalse(engaged(ebp, CS, plan, radar))
    self.assertEqual(ebp.reason, "demand")
    plan.aTarget = -0.2
    n = 0
    while not engaged(ebp, CS, plan, radar):
      n += 1
    self.assertAlmostEqual(n * DT_CTRL, RELEASE_HOLD, delta=2 * DT_CTRL)

  def test_lead_urgency(self):
    lead = log.RadarState.LeadData.new_message()
    lead.present, lead.dRel, lead.vRel, lead.aLeadK = True, 50., -0.3, 0.
    self.assertFalse(lead_urgent(lead, 25., 6., 1.))           # 2 s, slowly closing: fine
    lead.vRel = -1.2
    self.assertTrue(lead_urgent(lead, 25., 6., 1.))            # closing speed
    lead.vRel, lead.dRel = -0.9, 5.
    self.assertTrue(lead_urgent(lead, 25., 6., 1.))            # TTC
    lead.vRel, lead.dRel = 0.5, 20.
    self.assertTrue(lead_urgent(lead, 25., 6., 1.))            # too close (0.8 s) even while opening
    lead.dRel, lead.aLeadK = 60., -1.5
    self.assertTrue(lead_urgent(lead, 25., 6., 1.))            # lead braking
    lead.present = False
    self.assertFalse(lead_urgent(lead, 25., 6., 1.))

  def test_params_clamped(self):
    CP, _ = make_cp()
    setup(ebp=True, EngineBrakeHardFloor=-9.0, EngineBrakeCoastTarget=-2.0, EngineBrakeLeadTtc=0.5,
          EngineBrakeLeadClosingSpeed=50.)
    ebp = EngineBrakePreferred(CP, Params())
    self.assertEqual(ebp.hard_floor, HARD_FLOOR_RANGE[0])
    self.assertEqual(ebp.coast_target, COAST_TARGET_RANGE[0])
    self.assertEqual(ebp.lead_ttc, LEAD_TTC_RANGE[0])
    self.assertEqual(ebp.lead_closing, 3.0)


class TestCoastTargetDefault(OpenpilotTestCase):
  def test_default_coast_target_closes_the_throttle(self):
    # Stage 2 C2 (on-car): ACCEL_COMMAND 0 holds ~zero axle torque (no engine braking); a small negative closes the
    # throttle. The default must be that small negative, in the Python constant and in the compiled Params default.
    from openpilot.common.params import Params
    self.assertEqual(DEFAULT_COAST_TARGET, -0.2)
    p = Params()
    p.remove("EngineBrakeCoastTarget")
    self.assertAlmostEqual(float(p.get("EngineBrakeCoastTarget", return_default=True)), -0.2)


class TestMeasuredCarFinding(OpenpilotTestCase):
  """Stage 2 C2 on the measured-car plant (Task 16): the car already closes the throttle and engine-brakes for any request
  weaker than its engine braking, so EngineBrakePreferred changes no outcome - not brake use, not speed, not the gap -
  even when the model over-estimates engine braking 5x. Pinned so a plant / feature change that alters it is noticed."""

  def test_ebp_changes_no_outcome_on_the_measured_car(self):
    for name, sc in (("trim", GENTLE_TRIM), ("follow", GENTLE_FOLLOW), ("lead brakes", LEAD_BRAKES)):
      for scale in (1.0, 0.2):
        with self.subTest(scenario=name, true_eb_scale=scale):
          runs = []
          for on in (False, True):
            setup(ebp=on, non_adaptive=sc is GENTLE_TRIM)
            runs.append(ManualSim(true_eb_scale=scale, **sc).run(16.))
          off, on = runs
          if sc is not LEAD_BRAKES:
            self.assertTrue(any(l["suppress"] for l in on), "precondition: EBP actually suppressed")
          self.assertAlmostEqual(sum(l["brake"] for l in on), sum(l["brake"] for l in off), delta=0.05)
          self.assertAlmostEqual(on[-1]["speed"], off[-1]["speed"], delta=0.05)
          gaps_on, gaps_off = [l["gap"] for l in on if l["gap"] is not None], [l["gap"] for l in off if l["gap"] is not None]
          if gaps_on:
            self.assertAlmostEqual(min(gaps_on), min(gaps_off), delta=0.2)

  def test_small_requests_use_no_brakes_without_ebp(self):
    # the C2 numbers on the plant: 0 rolls on road load only; -0.17 / -0.45 = engine braking alone; -1.05 needs brakes
    sim = ManualSim(speed=12., gear=3)
    self.assertAlmostEqual(sim.car_accel(0.), resistive_decel(12.), places=6)
    for req, brakes in ((-0.17, False), (-0.45, False), (-1.05, True)):
      a = sim.car_accel(req)
      self.assertLessEqual(a, req + 1e-9, f"{req}: at least the request")
      self.assertEqual(sim.brake > BRAKE_EPS, brakes, f"{req}: brakes {sim.brake:.2f}")

