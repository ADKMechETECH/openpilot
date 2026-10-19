"""
Task 12 R5 — the Rule-5 matrix and tripwire.

Matrix: every fork feature that pauses longitudinal, raises the lower accel limit or ramps braking is driven into each of
its limiting states in the closed loop (planner -> real controlsd glue -> LongControl -> manual car), then ONE control
frame is run with each braking-urgency reason injected. Within that frame the lower limit must be back at stock
(ACCEL_MIN) and the command must go negative (the plan asks for braking in every state).

Tripwire: an AST scan of the controls code. Any function that raises a lower accel limit (`max(<..>limits[0], ..)`,
`lo = max(..)`) or writes `longActive` must consult the urgency predicate itself, or be listed in REVIEWED with the
function that does it for it (which is then checked to consult it).
"""
import ast
import os

from opendbc.car.interfaces import ACCEL_MIN
from openpilot.common.basedir import BASEDIR
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import braking_urgency
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import GateState
from openpilot.sunnypilot.selfdrive.controls.lib.tests.manual_sim import FakeSM, ManualCarSim, Pose, controls_frame

REASONS = ("fcw", "force_decel", "stock_aeb", "stopping", "hard_decel", "invalid_plan", "stale_plan", "invalid_radar",
           "lead_close", "lead_closing", "lead_ttc", "lead_braking")


def inject(sim, reason):
  """The controls SubMaster for one frame with `reason` injected (sim state as it is now)."""
  common = sim.common_sm()
  plan = sim.plan.as_builder()
  invalid, stale = set(sim.invalid), set()
  if reason == "fcw":
    plan.fcw = True
  elif reason == "force_decel":
    common['driverMonitoringState'].noResponseForceDecel = True
  elif reason == "stock_aeb":
    common['carState'].stockAeb = True
  elif reason == "stopping":
    plan.shouldStop = True
  elif reason == "hard_decel":
    plan.aTarget = -2.0
  elif reason == "invalid_plan":
    invalid.add('longitudinalPlan')
  elif reason == "stale_plan":      # .valid still True, all_checks fails (not alive / below frequency)
    stale.add('longitudinalPlan')
  elif reason == "invalid_radar":
    invalid.add('radarState')
  elif reason.startswith("lead_"):
    lead = common['radarState'].leadOne
    lead.present, lead.radar, lead.modelProb = True, True, 1.0
    v = sim.speed
    d, v_rel, a_lead = {"lead_close": (max(v * 0.6, 5.), 0., 0.), "lead_closing": (150., -2., 0.),
                        "lead_ttc": (40., -10., 0.), "lead_braking": (80., 0., -2.5)}[reason]
    lead.dRel, lead.vRel, lead.vLead, lead.vLeadK, lead.aLeadK = d, v_rel, v + v_rel, v + v_rel, a_lead
  csm = FakeSM(common, invalid=tuple(invalid), stale=tuple(stale))
  csm.update({'longitudinalPlan': plan, 'longitudinalPlanSP': sim.plan_sp, '_pose': Pose(sim.grade)})
  csm.valid = {k: k not in csm.invalid for k in csm}
  return csm


def one_frame(sim, csm):
  sim.update_rpm()
  return controls_frame(sim.ext, csm, sim.loc, sim.enabled, sim.gas)


def frames_until(sim, cond, max_s, step_s=DT_MDL):
  t = 0.
  while t < max_s:
    sim.run(step_s)
    t += step_s
    if cond(sim):
      return True
  return False


def last(sim):
  return sim.log[-1]


# (name, builder) — each builder returns a sim sitting in that feature's limiting state, with the plan asking to brake
def nsb_maintain():
  sim = ManualCarSim(speed=25., gear=6, v_set=22., NonAdaptiveLongitudinal=True, NoServiceBrakeHold=True,
                     ClutchShiftGate=False)
  assert frames_until(sim, lambda s: last(s)["limits"][0] > ACCEL_MIN + 1. and last(s)["a_target"] < -0.05, 2.)
  return sim


def nsb_pulse(over):
  def build():
    sim = ManualCarSim(speed=20. + over, gear=6, v_set=20., NonAdaptiveLongitudinal=True, NoServiceBrakeHold=True,
                       NoServiceBrakeOverspeedMode="pulse", ClutchShiftGate=False, grade=-0.05)
    assert frames_until(sim, lambda s: last(s)["limits"][0] > ACCEL_MIN + 1. and last(s)["a_target"] < -0.05, 2.)
    return sim
  return build


def ebp():
  sim = ManualCarSim(speed=25., gear=6, v_set=24.7, NonAdaptiveLongitudinal=True, EngineBrakePreferred=True,
                     ClutchShiftGate=False)
  assert frames_until(sim, lambda s: s.ext.engine_brake_preferred.suppress and last(s)["limits"][0] >= 0., 6.)
  return sim


def gate(state):
  def build():
    sim = ManualCarSim(speed=25., gear=5, v_set=9., NonAdaptiveLongitudinal=True, ClutchShiftGate=True)
    sim.run(1.)
    sim.set(clutch=True)
    want = GateState.shifting if state == "shifting" else GateState.coastTimeout
    assert frames_until(sim, lambda s: s.ext.clutch_shift_gate.state == want, 8.)
    return sim
  return build


def gate_reengaging():
  sim = ManualCarSim(speed=25., gear=5, v_set=9., NonAdaptiveLongitudinal=True, ClutchShiftGate=True)
  sim.run(1.)
  sim.run(0.5, clutch=True, gear=6)
  sim.set(clutch=False)
  assert frames_until(sim, lambda s: s.ext.clutch_shift_gate.state == GateState.reengaging, 1., DT_CTRL * 5)
  return sim


def gate_gas_released():
  sim = ManualCarSim(speed=25., gear=5, v_set=9., NonAdaptiveLongitudinal=True, ClutchShiftGate=True,
                     ClutchShiftGateGasReleaseHint=True)
  sim.run(1.)
  sim.run(0.1, gas=True)   # a short override while the plan is braking toward set speed: the plan still brakes after
  sim.set(gas=False)
  assert frames_until(sim, lambda s: s.ext.clutch_shift_gate.state == GateState.gasReleased, 0.5, DT_CTRL * 5)
  return sim


def gate_flare_hold():
  # openpilot coupled in 5th, then the engine free-revs (a missed clutch): the RPM-flare backstop caps the throttle
  sim = ManualCarSim(speed=25., gear=5, v_set=9., NonAdaptiveLongitudinal=True, ClutchShiftGate=True)
  sim.run(1.)
  base = sim.update_rpm

  def flaring():
    base()
    sim.rpm_reported = sim.rpm = sim.rpm + 30. * (sim.frame - flare_start)   # +3000 rpm/s
  flare_start = sim.frame
  sim.update_rpm = flaring
  assert frames_until(sim, lambda s: s.ext.clutch_shift_gate.state == GateState.shifting, 1., DT_CTRL * 5)
  return sim


def takeover(smart):
  def build():
    sim = ManualCarSim(speed=15., gear=3, v_set=13., NonAdaptiveLongitudinal=True, ClutchShiftGate=True,
                       ClutchReengageSmart=smart, ClutchReengageMaxRamp=2.0)
    sim.run(1.)
    sim.run(0.6, clutch=True, gear=4)
    sim.set(clutch=False)
    # frame by frame: stop as soon as the takeover ramp is narrowing the lower limit
    for _ in range(round(3. / DT_CTRL)):
      sim.run(DT_MDL)
      if sim.ext.clutch_takeover.active and last(sim)["limits"][0] > ACCEL_MIN + 0.5 and last(sim)["a_target"] < -0.05:
        return sim
    raise AssertionError("takeover ramp never limited the lower limit")
  return build


def handback():
  sim = ManualCarSim(speed=3.2, gear=1, v_set=0.5, NonAdaptiveLongitudinal=False, ClutchShiftGate=False,
                     ManualLowSpeedHandbackRamp=2.0)
  assert frames_until(sim, lambda s: s.ext.low_speed_handback.active and last(s)["limits"][0] > ACCEL_MIN + 0.5
                      and last(s)["a_target"] < -0.05, 6., DT_MDL)
  return sim


STATES = {"gate_reengaging": gate_reengaging, "gate_gas_released": gate_gas_released, "gate_flare_hold": gate_flare_hold,
          "nsb_maintain": nsb_maintain, "nsb_pulse_coasting": nsb_pulse(2.0), "nsb_pulsing": nsb_pulse(5.0),
          "ebp_suppressing": ebp, "gate_shifting": gate("shifting"), "gate_coast_timeout": gate("coastTimeout"),
          "takeover_fixed": takeover(False), "takeover_smart": takeover(True), "handback_ramp": handback}


class TestRule5Matrix(OpenpilotTestCase):
  def test_predicate_reasons(self):
    # every injected reason really is a Rule-5 reason (so the matrix tests the predicate, not a side effect)
    sim = ManualCarSim(speed=20., gear=4, ClutchShiftGate=False)
    sim.run(0.5)
    for reason in REASONS:
      with self.subTest(reason=reason):
        csm = inject(sim, reason)
        from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import controls_urgency
        got = controls_urgency(csm)
        self.assertIsNotNone(got, reason)
        if reason.startswith("lead_") or reason in ("fcw", "stopping", "hard_decel", "invalid_radar", "force_decel"):
          self.assertEqual(got, reason)

  def test_matrix(self):
    for name, build in STATES.items():
      for reason in REASONS:
        with self.subTest(state=name, reason=reason):
          sim = build()
          before = last(sim)
          if name.startswith("gate_"):   # the gate caps only the throttle side; it never raises the lower limit
            self.assertLessEqual(before["limits"][1], 0., f"{name}: precondition, the throttle cap is on")
            self.assertLess(before["a_target"], 0.)
          else:
            self.assertGreater(before["limits"][0], ACCEL_MIN, f"{name}: precondition, the feature is limiting")
          long_active, limits, cmd = one_frame(sim, inject(sim, reason))
          self.assertTrue(long_active, "urgency never drops longitudinal")
          self.assertEqual(limits[0], ACCEL_MIN, "lower limit back at stock in the same frame")
          self.assertLess(cmd, 0., "the command brakes")

  def test_gate_gas_blip_is_the_panda_case(self):
    # the one exception, documented: paused with the gas pressed, the panda blocks any nonzero accel (urgent or not) -
    # exactly stock's gasPressedOverride outcome. Lower limit 0, command 0, longitudinal still active.
    sim = gate("shifting")()
    sim.set(gas=True)
    long_active, limits, cmd = one_frame(sim, inject(sim, "fcw"))
    self.assertEqual((limits, cmd), ((0., 0.), 0.))
    sim.set(gas=False)
    long_active, limits, cmd = one_frame(sim, inject(sim, "fcw"))
    self.assertEqual(limits[0], ACCEL_MIN)
    self.assertLess(cmd, 0.)

  def test_planner_side_features_step_aside(self):
    # GearAware returns stock jerk scales and the NSB plan releases (allowBrake) under urgency
    from openpilot.sunnypilot.selfdrive.controls.lib.gear_aware_accel import STOCK
    sim = ManualCarSim(speed=12., gear=2, v_set=10., NonAdaptiveLongitudinal=True, GearAwareAccel=True,
                       NoServiceBrakeHold=True, ClutchShiftGate=False)
    sim.run(1.)
    self.assertNotEqual(sim.planner.gear_aware_accel.scales, STOCK, "precondition: gear-aware shaping active")
    self.assertFalse(sim.plan.allowBrake, "precondition: NSB clamping")
    sim.fcw_override = False
    sim.force_decel = True
    sim.run(DT_MDL)
    self.assertEqual(sim.planner.gear_aware_accel.scales, STOCK)
    self.assertTrue(sim.plan.allowBrake)

  def test_plan_derived_reasons_need_a_live_plan(self):
    # while LongControl is off the planner re-seeds from aEgo, so its aTarget/shouldStop are the car's own decel (a
    # hill, the driver braking): not a braking request. FCW, leads, forceDecel, invalid inputs always count.
    from openpilot.cereal import messaging
    radar = messaging.new_message('radarState').radarState
    kw = {"plan_ok": True, "radar_ok": True, "force_decel": False, "fcw": False, "stock_aeb": False, "v_ego": 1., "radar": radar}
    self.assertEqual(braking_urgency(should_stop=True, a_target=-3., plan_live=True, **kw), "stopping")
    self.assertEqual(braking_urgency(should_stop=False, a_target=-3., plan_live=True, **kw), "hard_decel")
    self.assertIsNone(braking_urgency(should_stop=True, a_target=-3., plan_live=False, **kw))
    self.assertEqual(braking_urgency(should_stop=False, a_target=float('nan'), plan_live=False, **kw), "hard_decel")
    self.assertEqual(braking_urgency(should_stop=False, a_target=0., plan_live=False, **{**kw, "fcw": True}), "fcw")
    self.assertEqual(braking_urgency(should_stop=False, a_target=0., plan_live=False, **{**kw, "plan_ok": False}),
                     "invalid_plan")


# -- tripwire -----------------------------------------------------------------------------------------------------------
SCAN = ("sunnypilot/selfdrive/controls", "selfdrive/controls")
# function -> (the function that consults urgency for it, why)
REVIEWED = {
  "engine_brake_preferred.py:EngineBrakePreferred.limit_accel":
    ("engine_brake_preferred.py:EngineBrakePreferred.update", "suppress is only set by update(), Rule 5 first"),
  "no_service_brake.py:apply_brake_floor":
    ("controlsd_ext.py:ControlsExt.apply_no_service_brake", "only caller; returns before it under urgency"),
  "no_service_brake.py:PulseFloor.apply":
    ("controlsd_ext.py:ControlsExt.apply_no_service_brake", "only caller; returns before it under urgency"),
  "controlsd.py:Controls.state_control":
    ("controlsd_ext.py:ControlsExt.update_low_speed_handback", "stock overrides + handback_done, which is Rule-5 gated"),
}


def _functions(path):
  tree = ast.parse(open(path).read())
  for node in ast.walk(tree):
    if isinstance(node, ast.ClassDef):
      for f in node.body:
        if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef)):
          yield f"{node.name}.{f.name}", f
  for f in tree.body:
    if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef)):
      yield f.name, f


def _raises_lower_limit(fn) -> bool:
  for node in ast.walk(fn):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "max" and node.args:
      first = node.args[0]
      if isinstance(first, ast.Subscript) and isinstance(first.slice, ast.Constant) and first.slice.value == 0 and \
         "limits" in ast.unparse(first.value):
        return True
    if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "lo" for t in node.targets) and \
       isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) and node.value.func.id == "max":
      return True
    if isinstance(node, ast.Assign) and any(isinstance(t, ast.Attribute) and t.attr == "longActive" for t in node.targets):
      return True
    # returning limits with a constant lower bound (e.g. `return 0., 0.`)
    if "limit" in fn.name and isinstance(node, ast.Return) and isinstance(node.value, ast.Tuple) and node.value.elts and \
       isinstance(node.value.elts[0], ast.Constant):
      return True
  return False


def _consults_urgency(fn) -> bool:
  return "urgen" in ast.unparse(fn)


class TestRule5Tripwire(OpenpilotTestCase):
  def scan(self):
    found, index = {}, {}
    for rel in SCAN:
      for root, _, files in os.walk(os.path.join(BASEDIR, "openpilot", rel)):
        if "/tests" in root or root.endswith("tests"):
          continue
        for f in files:
          if f.endswith(".py"):
            for name, fn in _functions(os.path.join(root, f)):
              index[f"{f}:{name}"] = fn
              if _raises_lower_limit(fn):
                found[f"{f}:{name}"] = fn
    return found, index

  def test_every_limiter_consults_the_predicate(self):
    found, index = self.scan()
    self.assertIn("clutch_shift_gate.py:ClutchShiftGate.limit_accel", found, "sanity: the scan sees the known sites")
    self.assertIn("manual_low_speed_handback.py:LowSpeedHandback.limit_accel", found)
    for key, fn in found.items():
      with self.subTest(site=key):
        if _consults_urgency(fn):
          continue
        msg = (f"{key} raises a lower accel limit / drops longActive without consulting braking_urgency(): " +
               "call it, or add a REVIEWED entry naming the function that does")
        self.assertIn(key, REVIEWED, msg)
        gate_fn, _ = REVIEWED[key]
        self.assertIn(gate_fn, index, f"REVIEWED gate {gate_fn} not found")
        self.assertTrue(_consults_urgency(index[gate_fn]), f"{gate_fn} must consult the urgency predicate")

  def test_reviewed_entries_are_live(self):
    found, _ = self.scan()
    for key in REVIEWED:
      self.assertIn(key, found, f"stale REVIEWED entry {key}: remove it")
