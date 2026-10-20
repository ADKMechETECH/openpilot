"""
Non-adaptive (set-speed only) longitudinal: with the NonAdaptiveLongitudinal toggle ON, openpilot longitudinal
holds the set speed and does not slow for leads, while the MPC (and so FCW) keeps running. With it OFF (default),
behavior is unchanged.
"""
import numpy as np

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, LongitudinalPlanSource
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant
from openpilot.sunnypilot.selfdrive.controls.lib.dec.tests.test_dec_planner_gate import E2E_ACCEL, MockDec, build_sm
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP, COLLISION_DWELL
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import ManualCarSim

V_CRUISE = 25.0  # m/s


def build_planner(op_long: bool = True, transmission=None) -> LongitudinalPlanner:
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
  CP.openpilotLongitudinalControl = op_long  # radarless Civic defaults to stock ACC unless alpha long is enabled
  CP.pcmCruise = not op_long
  if transmission is not None:
    CP.transmissionType = transmission
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC_2022)
  planner = LongitudinalPlanner(CP, CP_SP, init_v=20.0)
  planner.dec = MockDec(active=False, mode='acc')  # DEC inactive: experimental mode alone decides e2e
  return planner


def run(non_adaptive: bool, t_end: float, speed: float, lead: bool = False, d_lead: float = 200., v_lead: float = 0.,
        prob_throttle: float = 1.0, e2e: bool = False):
  Params().put_bool("NonAdaptiveLongitudinal", non_adaptive)
  plant = Plant(lead_relevancy=lead, speed=speed, distance_lead=d_lead, e2e=e2e)
  logs = []
  while plant.current_time < t_end:
    log = plant.step(v_lead=v_lead, v_cruise=V_CRUISE, prob_throttle=prob_throttle)
    log["source"] = plant.planner.mpc.source
    logs.append(log)
  return plant, logs


class TestNonAdaptiveLongitudinal(OpenpilotTestCase):
  def test_default_off(self):
    plant, _ = run(False, 0.5, V_CRUISE)
    self.assertFalse(Params().get_bool("NonAdaptiveLongitudinal"))
    self.assertFalse(plant.planner.non_adaptive)

  def test_on_slower_lead_far_ahead_not_followed(self):
    # (replaces test_on_holds_set_speed_and_ignores_lead, which closed on a 10 m/s lead from 60 m and asserted NO
    # braking — the hazard Task 12 A1 removes.) A slower car far ahead is not a threat: speed held, no braking.
    _, logs = run(True, 6.0, V_CRUISE, lead=True, d_lead=200., v_lead=20.)
    self.assertTrue(all(l["source"] == LongitudinalPlanSource.cruise for l in logs))
    self.assertGreater(min(l["acceleration"] for l in logs), -0.05)
    self.assertAlmostEqual(logs[-1]["speed"], V_CRUISE, delta=0.3)

  def test_on_fcw_still_fires(self):
    # FCW is computed from the MPC, which still runs in non-adaptive mode
    _, logs = run(True, 6.0, V_CRUISE, lead=True, d_lead=60., v_lead=0.)
    self.assertTrue(any(l["fcw"] for l in logs), "FCW must still warn when closing on a stopped lead")

  def test_on_accelerates_to_set_speed(self):
    _, logs = run(True, 15.0, 18.0)
    self.assertGreater(max(l["acceleration"] for l in logs), 0.3)
    self.assertAlmostEqual(logs[-1]["speed"], V_CRUISE, delta=0.3)

  def test_on_keeps_model_predicted_lift_cap(self):
    # Task 12 A1: stock's model-predicted-lift throttle cap (allow_throttle) is kept in non-adaptive mode — it only
    # limits throttle (coast), it never brakes
    _, logs_off = run(False, 15.0, 18.0, prob_throttle=0.0)
    _, logs_on = run(True, 15.0, 18.0, prob_throttle=0.0)
    self.assertLess(logs_off[-1]["speed"], V_CRUISE - 2.0, "precondition: OFF mode should coast, not reach set speed")
    self.assertLess(logs_on[-1]["speed"], V_CRUISE - 2.0, "the cap applies in non-adaptive mode too")
    self.assertGreater(min(l["acceleration"] for l in logs_on), -0.6, "and it only coasts: no braking from the cap")

  def test_on_ignores_e2e(self):
    # experimental mode with the model asking to brake hard: the e2e candidate must not be used when ON
    for non_adaptive, expect_e2e in ((False, True), (True, False)):
      with self.subTest(non_adaptive=non_adaptive):
        Params().put_bool("NonAdaptiveLongitudinal", non_adaptive)
        planner = build_planner()
        planner.update(build_sm(experimental_mode=True))
        self.assertEqual(planner.mpc.source == LongitudinalPlanSource.e2e, expect_e2e)
        if not expect_e2e:
          self.assertGreater(planner.output_a_target, E2E_ACCEL)

  def test_off_still_follows_lead(self):
    _, logs = run(False, 20.0, V_CRUISE, lead=True, d_lead=60., v_lead=10.)
    self.assertLess(min(l["acceleration"] for l in logs), -0.5, "lead-following must be unchanged with the toggle OFF")
    self.assertLess(logs[-1]["speed"], 12.0)
    self.assertGreater(min(l["distance_lead"] - l["distance"] for l in logs), 2.0)

  def test_available_regardless_of_transmission(self):
    # longitudinal-behavior feature: gated only on openpilot longitudinal, never on transmission or brand
    from opendbc.car.structs import CarParams
    Params().put_bool("NonAdaptiveLongitudinal", True)
    for transmission in (CarParams.TransmissionType.manual, CarParams.TransmissionType.automatic,
                         CarParams.TransmissionType.cvt):
      with self.subTest(transmission=transmission):
        planner = build_planner(op_long=True, transmission=transmission)
        planner.update(build_sm(experimental_mode=True))
        self.assertTrue(planner.non_adaptive)
        self.assertEqual(planner.mpc.source, LongitudinalPlanSource.cruise)
    # and on a non-Honda platform (Plant runs a Nidec Civic; also check a Toyota)
    from opendbc.car.toyota.interface import CarInterface as ToyotaInterface
    from opendbc.car.toyota.values import CAR as TOYOTA
    CP = ToyotaInterface.get_non_essential_params(TOYOTA.TOYOTA_RAV4_TSS2)
    CP.openpilotLongitudinalControl = True
    planner = LongitudinalPlanner(CP, ToyotaInterface.get_non_essential_params_sp(CP, TOYOTA.TOYOTA_RAV4_TSS2), init_v=20.0)
    self.assertTrue(planner.non_adaptive)

  def test_inert_without_openpilot_longitudinal(self):
    # stock ACC (pcmCruise) car: the toggle must no-op, plan identical to OFF
    Params().put_bool("NonAdaptiveLongitudinal", True)
    planner = build_planner(op_long=False)
    planner.dec = MockDec(active=False, mode='acc')
    planner.update(build_sm(experimental_mode=True))
    self.assertFalse(planner.non_adaptive_available)
    self.assertFalse(planner.non_adaptive)
    self.assertEqual(planner.mpc.source, LongitudinalPlanSource.e2e)

  def test_toggle_only_latches_while_long_inactive(self):
    plant, _ = run(False, 0.5, V_CRUISE)
    Params().put_bool("NonAdaptiveLongitudinal", True)
    for _ in range(60):  # 3 s engaged: the param is re-read, but the mode must not switch mid-engagement
      plant.step(v_cruise=V_CRUISE)
    self.assertFalse(plant.planner.non_adaptive)

    plant.enabled = False
    plant.step(v_cruise=V_CRUISE)
    plant.enabled = True
    plant.step(v_cruise=V_CRUISE)
    self.assertTrue(plant.planner.non_adaptive)


def a1_sim(plant, speed, lead, v_set=None, gear=6, **params):
  params.setdefault("NonAdaptiveLongitudinal", True)
  params.setdefault("ClutchShiftGate", False)
  return ManualCarSim(speed=speed, gear=gear, v_set=v_set, plant=plant, lead=lead, **params)


def first_t(log, cond):
  return next((r["t"] for r in log if cond(r)), None)


class TestNonAdaptiveCollisionBraking(OpenpilotTestCase):
  """Task 12 A1, closed loop through planner -> controlsd glue -> LongControl -> a manual car, both plant models. The
  lead's gap is never clamped: contact is recorded. Owner decision 2026-10-19: brake only for a real slow vehicle or
  collision, never ordinary following."""

  def test_a_stopped_car_from_far_away(self):
    for plant in ("open", "closed"):
      with self.subTest(plant=plant):
        sim = a1_sim(plant, 29., (250., 0.)).run(25.)
        self.assertFalse(sim.contact, f"contact (min gap {sim.min_gap():.1f} m)")
        self.assertGreater(sim.min_gap(), 2.0)
        t_brake = first_t(sim.log, lambda r: r["sent"] < -0.5)
        t_fcw = first_t(sim.log, lambda r: r["fcw"])
        self.assertIsNotNone(t_brake)
        if t_fcw is not None:
          self.assertLessEqual(t_brake, t_fcw, "braking must begin no later than FCW")
        self.assertTrue(any(r["collision"] for r in sim.log))

  def test_b_lead_brakes_hard_from_following_gap(self):
    # 1.5 s gap at 25 m/s, the lead brakes at 3 m/s^2 to a stop: no contact, and as good as stock adaptive. (Harder
    # than openpilot's own -3.5 m/s^2 authority, e.g. 4 m/s^2 from 1.5 s, stock adaptive makes contact in the closed
    # plant too: there the check is only "never worse than stock".)
    for plant in ("open", "closed"):
      for lead_a in (-3.0, -4.0):
        with self.subTest(plant=plant, lead_a=lead_a):
          gaps = {}
          for na in (True, False):
            # the lead brakes at t=0, so both modes start from the same state (steady following itself: test (d))
            sim = a1_sim(plant, 25., (37.5, 25.), v_set=25., NonAdaptiveLongitudinal=na)
            sim.run(15., lead_a=lead_a)
            gaps[na] = sim.min_gap()
            if lead_a == -3.0:
              self.assertFalse(sim.contact, f"contact (min gap {sim.min_gap():.1f} m)")
              self.assertLess(min(r["sent"] for r in sim.log), -2.0)
          self.assertGreater(gaps[True], gaps[False] - 0.5, "non-adaptive brakes as well as stock adaptive")

  def test_c_cut_in_close_and_closing(self):
    for plant, nsb in (("open", False), ("closed", False), ("open", True)):
      with self.subTest(plant=plant, nsb=nsb):
        sim = a1_sim(plant, 25., (15., 20.), v_set=25., NoServiceBrakeHold=nsb, NoServiceBrakeDecelFloor=0.0).run(8.)
        self.assertFalse(sim.contact, f"contact (min gap {sim.min_gap():.1f} m)")
        t_brake = first_t(sim.log, lambda r: r["sent"] < -1.0)
        self.assertIsNotNone(t_brake)
        self.assertLess(t_brake, 0.5)
        self.assertTrue(all(r["limits"][0] <= -3.0 for r in sim.log if r["collision"]), "no-service-brake floor released")
        self.assertTrue(all(r["allow_brake"] for r in sim.log if r["collision"]))

  def test_nsb_released_through_the_collision_dwell(self):
    # plannerd keeps the no-service-brake clamp released for the whole collision hold, including the dwell after the
    # Rule-5 reason has cleared (the re-admitted lead MPC may still be braking then)
    for plant in ("open", "closed"):
      with self.subTest(plant=plant):
        # closing fast from 45 m (TTC < 6 s), then the lead speeds up to our speed while still far: the Rule-5 reason
        # clears, the collision hold doesn't yet, and the MPC is still braking
        sim = a1_sim(plant, 25., (45., 17.), v_set=25., NoServiceBrakeHold=True, NoServiceBrakeDecelFloor=0.0).run(0.5)
        sim.run(2.5, lead_a=4.0)
        sim.run(4., lead_a=0.)
        dwell = [r for r in sim.log if r["collision"] and r["urgency"] is None]
        self.assertTrue(dwell, "precondition: dwell frames")
        self.assertTrue(all(r["allow_brake"] for r in sim.log if r["collision"]))
        self.assertLess(min(r["sent"] for r in dwell), -0.5, "the MPC's braking passes during the dwell")
        self.assertFalse(sim.contact)

  def test_d_steady_following_no_braking_60s(self):
    for plant in ("open", "closed"):
      with self.subTest(plant=plant):
        sim = a1_sim(plant, 25., (37.5, 25.), v_set=25.).run(60.)
        self.assertFalse(any(r["collision"] for r in sim.log))
        self.assertGreater(min(r["sent"] for r in sim.log), -0.05, "no braking")
        vs = [r["v"] for r in sim.log]
        self.assertLess(max(vs) - min(vs), 0.3, "no speed change")
        self.assertTrue(all(r["source"] == "cruise" for r in sim.log))

  def test_e_slower_lead_far_ahead(self):
    for plant in ("open", "closed"):
      with self.subTest(plant=plant):
        sim = a1_sim(plant, 25., (160., 20.), v_set=25.).run(10.)
        self.assertFalse(any(r["collision"] for r in sim.log))
        self.assertGreater(min(r["sent"] for r in sim.log), -0.05)
        self.assertAlmostEqual(sim.log[-1]["v"], 25., delta=0.3)

  def test_dwell_releases_without_a_step(self):
    # cut-in that then pulls away: the MPC stays admitted for COLLISION_DWELL after the threat clears, and the set-speed
    # candidate resumes from the collision braking (no step back up)
    sim = a1_sim("closed", 25., (14., 23.), v_set=25.).run(2.)
    self.assertTrue(any(r["collision"] for r in sim.log))
    sim.run(12., lead_a=1.5)
    t_off = first_t(sim.log, lambda r: r["t"] > 1. and not r["collision"])
    self.assertIsNotNone(t_off)
    t_last_threat = max(r["t"] for r in sim.log if r["collision"])
    self.assertGreaterEqual(t_off - 2., 0.)
    cmds = [r["sent"] for r in sim.log if abs(r["t"] - t_last_threat) < 1.]
    self.assertLess(max(abs(b - a) for a, b in zip(cmds, cmds[1:], strict=False)), 0.1, "no step at the hand-back")
    self.assertGreater(COLLISION_DWELL, 0.)

  def test_f_off_is_unchanged(self):
    # OFF: the A1 code is inert — identical frame for frame to a planner whose collision path is forced off
    # (the CI self-referenced plannerd/controlsd replays pin OFF == pre-Task-12)
    for plant in ("open", "closed"):
      with self.subTest(plant=plant):
        ref = a1_sim(plant, 25., (15., 20.), v_set=25., NonAdaptiveLongitudinal=False).run(6.)
        orig = LongitudinalPlannerSP.update_collision_braking
        try:
          LongitudinalPlannerSP.update_collision_braking = lambda *a, **k: False
          stub = a1_sim(plant, 25., (15., 20.), v_set=25., NonAdaptiveLongitudinal=False).run(6.)
        finally:
          LongitudinalPlannerSP.update_collision_braking = orig
        self.assertEqual([r["sent"] for r in ref.log], [r["sent"] for r in stub.log])
        self.assertFalse(any(r["collision"] for r in ref.log))

  def test_collision_threat_criteria(self):
    # the narrow set non-adaptive brakes on (owner decision): close only counts while closing or the lead brakes
    from openpilot.cereal import log
    from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import collision_threat, lead_reason
    lead = log.RadarState.LeadData.new_message()
    lead.present, lead.dRel, lead.vRel, lead.aLeadK = True, 20., 0., 0.        # 0.8 s at 25 m/s, matching speed
    self.assertEqual(lead_reason(lead, 25.), "lead_close", "precondition: Rule-5 calls it close")
    self.assertIsNone(collision_threat(lead, 25.), "close but not closing: not a collision threat")
    lead.vRel = -0.5
    self.assertEqual(collision_threat(lead, 25.), "lead_close")
    lead.vRel, lead.aLeadK = 0., -1.5
    self.assertEqual(collision_threat(lead, 25.), "lead_close")
    lead.dRel, lead.vRel, lead.aLeadK = 150., -2., 0.                         # slower car far ahead: TTC 75 s
    self.assertEqual(lead_reason(lead, 25.), "lead_closing")
    self.assertIsNone(collision_threat(lead, 25.))
    lead.dRel, lead.aLeadK = 100., -2.0                                        # braking, but 4 s away
    self.assertIsNone(collision_threat(lead, 25.))
    lead.dRel = 60.                                                            # braking within 3 s
    self.assertEqual(collision_threat(lead, 25.), "lead_braking")
    lead.dRel, lead.vRel = 30., -6.                                            # TTC 5 s
    self.assertEqual(collision_threat(lead, 25.), "lead_ttc")

  def test_lead_visible_only_on_threat(self):
    sim = a1_sim("closed", 25., (37.5, 25.), v_set=25.).run(1.)
    sm = {'longitudinalPlan': sim.plan, 'longitudinalPlanSP': sim.plan_sp}
    from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import FakeSM
    self.assertTrue(sim.plan.hasLead, "hasLead stays in the logs")
    self.assertFalse(sim.ext.lead_visible(FakeSM(sm)), "no ACC-follow lead on the cluster in non-adaptive")
    self.assertTrue(sim.ext.lead_visible(FakeSM(sm, invalid=('longitudinalPlanSP',))), "invalid SP -> stock")
    sim = a1_sim("closed", 25., (15., 20.), v_set=25.).run(1.)
    self.assertTrue(sim.ext.lead_visible(FakeSM({'longitudinalPlan': sim.plan, 'longitudinalPlanSP': sim.plan_sp})))
    sim = a1_sim("closed", 25., (37.5, 25.), v_set=25., NonAdaptiveLongitudinal=False).run(1.)
    self.assertTrue(sim.ext.lead_visible(FakeSM({'longitudinalPlan': sim.plan, 'longitudinalPlanSP': sim.plan_sp})))


class ForcedMpc:
  """Stand-in for LongitudinalMpc with a forced output (Task 14 B1): the planner's candidate selection, the collision
  re-admit and the cruise re-seed are pinned without the acados solver - so the A1 mutations are caught by any CI run."""

  def __init__(self, accel: float = 0.):
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS
    self.T = np.asarray(T_IDXS)
    self.accel, self.crash_cnt, self.solve_time, self.v0 = accel, 0, 0., 0.
    self.source = LongitudinalPlanSource.lead0
    self.update(None)

  def set_weights(self, *a, **kw):
    pass

  def set_cur_state(self, v, a):
    self.v0 = float(v)

  def update(self, radarstate, *a, **kw):
    self.a_solution = np.full(len(self.T), self.accel)
    self.v_solution = np.maximum(self.v0 + self.accel * self.T, 0.)
    self.j_solution = np.zeros(len(self.T) - 1)


class TestForcedMpcPlanner(OpenpilotTestCase):
  """Task 14 B1 [W2]: non-adaptive collision braking at planner level with a forced MPC candidate."""

  def _sim(self, mpc_accel, lead):
    from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import ManualCarSim
    sim = ManualCarSim(speed=18., gear=6, v_set=20., NonAdaptiveLongitudinal=True, ClutchShiftGate=False, lead=lead)
    sim.planner.mpc = ForcedMpc(mpc_accel)
    return sim

  def test_collision_threat_readmits_a_mild_mpc(self):
    # a lead inside the 6 s TTC window while the (forced) MPC asks only -0.6 (milder than MPC_NEEDED_DECEL): only
    # collision_threat() can re-admit the MPC (a1_no_threat removes it)
    sim = self._sim(-0.6, (25., 13.))   # 25 m ahead at 13 m/s, ego 18: closing 5 m/s, TTC 5 s
    sim.run(1.)
    self.assertTrue(sim.plan_sp.features.collisionBraking)
    self.assertEqual(sim.plan_sp.features.collisionReason, "lead_ttc")
    self.assertAlmostEqual(sim.plan.aTarget, -0.6, delta=0.05, msg="the MPC's braking is the plan")
    # control: no threat (the lead far away) -> the set-speed plan, not the MPC
    far = self._sim(-0.6, (200., 18.))
    far.run(1.)
    self.assertFalse(far.plan_sp.features.collisionBraking)
    self.assertGreater(far.plan.aTarget, 0., "set speed above: the cruise candidate accelerates")

  def test_cruise_resumes_from_the_collision_braking_without_a_step(self):
    # the threat clears, the 2 s dwell ends, the MPC is dropped: the cruise candidate continues from the collision
    # braking (a1_no_cruise_reseed leaves it at its pre-threat value -> a step of ~+1.5 m/s^2 in one plan)
    sim = self._sim(-1.5, (25., 13.))
    sim.run(2.)
    self.assertTrue(sim.plan_sp.features.collisionBraking)
    sim.lead = None   # threat gone; the forced MPC keeps asking -1.5 during the dwell
    targets = []
    for _ in range(round(5. / 0.05)):
      sim.run(0.05)
      targets.append(sim.plan.aTarget)
    self.assertFalse(sim.plan_sp.features.collisionBraking, "precondition: the dwell ended")
    steps = [b - a for a, b in zip(targets, targets[1:], strict=False)]
    self.assertLess(max(steps), 0.3, f"no step up when the MPC is dropped (max {max(steps):.2f} m/s^2 per plan)")
    self.assertGreater(targets[-1], -0.5, "and it does climb back toward the set-speed plan")
