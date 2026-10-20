"""
Task 12 R5 — the Rule-5 matrix and tripwire.

Matrix: every fork feature that pauses longitudinal, raises the lower accel limit or ramps braking is driven into each of
its limiting states in the closed loop (planner -> real controlsd glue -> LongControl -> manual car), then ONE control
frame is run with each braking-urgency reason injected. Within that frame the lower limit must be back at stock
(ACCEL_MIN) and the command must go negative (the plan asks for braking in every state).

The low-speed handback is the one limiter with a narrower carve-out (Task 13 A1, like non-adaptive's collision_threat):
it steps aside on braking_urgency.handback_urgency() — every non-lead reason except "stopping", plus an imminent lead —
and must NOT freeze for the plan's stop request or a lead that is merely close / closing / braking somewhere ahead
(HANDBACK_CONTINUES); that is what held a manual at a standstill in ordinary traffic.

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
           "lead_close", "lead_closing", "lead_ttc", "lead_braking", "lead_imminent")
# Task 13 A1: reasons the low-speed handback deliberately does not step aside for (not imminent from where it stands)
HANDBACK_CONTINUES = ("stopping", "lead_close", "lead_closing", "lead_braking")


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
                        "lead_ttc": (40., -10., 0.), "lead_braking": (80., 0., -2.5),
                        "lead_imminent": (3., -v, 0.)}[reason]   # a stopped car 3 m ahead
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
        if (reason.startswith("lead_") and reason != "lead_imminent") or reason in ("fcw", "stopping", "hard_decel", "invalid_radar", "force_decel"):
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
          if name == "handback_ramp" and reason in HANDBACK_CONTINUES:
            # Task 13 A1: not an emergency for the handback — the ramp keeps going (the driver takes the stop)
            self.assertFalse(sim.ext.low_speed_handback.urgent, f"{reason} must not freeze the handback")
            self.assertGreater(limits[0], ACCEL_MIN, "the handback keeps ramping the braking side")
            continue
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

  def _gear_aware_shaping(self):
    """1st gear, lifting off after a set-speed drop: GearAware's fall scaling holds the plan above the stock plan."""
    sim = ManualCarSim(speed=10., gear=1, v_set=14., NonAdaptiveLongitudinal=True, GearAwareAccel=True,
                       ClutchShiftGate=False)
    sim.run(2.)
    sim.set(v_set=sim.speed - 0.5)
    assert frames_until(sim, lambda s: s.plan.aTarget - s.plan_sp.features.aTargetStock > 0.1, 4.)
    return sim

  def test_gear_aware_steps_aside_in_one_control_frame(self):
    # Task 13 A3: GearAware shapes only the plannerd set-speed candidate; under any Rule-5 reason controlsd runs the
    # stock-jerk plan in the SAME control frame (not up to 50 ms later at the next plan)
    sim = self._gear_aware_shaping()
    plan, stock = sim.plan.aTarget, sim.plan_sp.features.aTargetStock
    one_frame(sim, inject(sim, "none"))
    self.assertEqual(sim.ext.last_a_target, plan, "no urgency: the shaped plan runs")
    for reason in REASONS:
      with self.subTest(reason=reason):
        sim = self._gear_aware_shaping()
        plan, stock = sim.plan.aTarget, sim.plan_sp.features.aTargetStock
        csm = inject(sim, reason)
        one_frame(sim, csm)
        self.assertIsNotNone(sim.ext.urgency)
        want = min(csm['longitudinalPlan'].aTarget, stock)   # hard_decel injects a lower plan: that one wins
        self.assertAlmostEqual(sim.ext.last_a_target, want, places=5, msg="stock plan within the frame")
        self.assertLess(sim.ext.last_a_target, plan - 0.1)

  def test_gear_aware_planner_steps_down_to_stock(self):
    # and plannerd's own next frame under urgency: the shaped candidate steps down to the stock one (not just stock rates)
    sim = self._gear_aware_shaping()
    sim.force_decel = True
    sim.run(DT_MDL)
    self.assertLessEqual(sim.plan.aTarget, sim.plan_sp.features.aTargetStock + 1e-6)

  def test_gear_aware_off_publishes_nothing_new(self):
    sim = ManualCarSim(speed=10., gear=1, v_set=14., NonAdaptiveLongitudinal=True, GearAwareAccel=False,
                       ClutchShiftGate=False)
    sim.run(2.)
    sim.set(v_set=sim.speed - 0.5)
    sim.run(1.)
    self.assertEqual(sim.plan_sp.features.aTargetStock, 0.)
    self.assertFalse(sim.plan_sp.features.gearAware)
    one_frame(sim, inject(sim, "fcw"))
    self.assertEqual(sim.ext.last_a_target, sim.plan.aTarget)

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
# the urgency predicate: calling one of these (and acting on the result) is "consulting" it
PREDICATES = {"braking_urgency", "controls_urgency", "handback_urgency", "collision_threat", "lead_urgent", "lead_reason",
              "handback_lead_reason"}
LOGGING = {"cloudlog", "print", "str", "repr", "format", "warning", "info", "debug", "event", "error"}
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
# functions that don't consult urgency on purpose: (the test in this file that pins why, why)
EXEMPT = {
  "clutch_shift_gate.py:ClutchShiftGate.limit_accel":
    ("test_gate_gas_blip_is_the_panda_case", "never raises the lower limit; the gas-pressed (0, 0) is the panda's rule"),
}


def _functions(path):
  """(qualified name, function node, class node or None) for every function/method in the file."""
  tree = ast.parse(open(path).read())
  for node in ast.walk(tree):
    if isinstance(node, ast.ClassDef):
      for f in node.body:
        if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef)):
          yield f"{node.name}.{f.name}", f, node
  for f in tree.body:
    if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef)):
      yield f.name, f, None


def _is_lower_limit(node) -> bool:
  return isinstance(node, ast.Subscript) and isinstance(node.slice, ast.Constant) and node.slice.value == 0 and \
         "limits" in ast.unparse(node.value)


def _raises_lower_limit(fn) -> bool:
  for node in ast.walk(fn):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "max" and node.args:
      if _is_lower_limit(node.args[0]):
        return True
    # Task 14 W1-N7: np.clip(limits[0], lo, hi) / clip(...) raises it too
    if isinstance(node, ast.Call) and node.args and _is_lower_limit(node.args[0]) and \
       ((isinstance(node.func, ast.Attribute) and node.func.attr == "clip") or
        (isinstance(node.func, ast.Name) and node.func.id == "clip")):
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


def _call_name(call: ast.Call) -> str:
  f = call.func
  return f.id if isinstance(f, ast.Name) else f.attr if isinstance(f, ast.Attribute) else ""


def _is_predicate_call(node) -> bool:
  return isinstance(node, ast.Call) and _call_name(node) in PREDICATES


def _tainted(expr, names: set[str], attrs: set[str]) -> bool:
  """expr carries an urgency value: a predicate call, a tainted local / parameter, or a tainted self.<attr>."""
  for n in ast.walk(expr):
    if _is_predicate_call(n):
      return True
    if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load) and n.id in names:
      return True
    if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Load) and isinstance(n.value, ast.Name) and \
       n.value.id == "self" and n.attr in attrs:
      return True
  return False


def _local_taint(fn, attrs: set[str]) -> set[str]:
  """Flow-insensitive: urgency parameters, then every local assigned from a tainted expression (to a fixpoint)."""
  args = fn.args
  names = {a.arg for a in args.args + args.kwonlyargs + args.posonlyargs if "urgen" in a.arg}
  changed = True
  while changed:
    changed = False
    for n in ast.walk(fn):
      targets, value = [], None
      if isinstance(n, ast.Assign):
        targets, value = n.targets, n.value
      elif isinstance(n, (ast.AnnAssign, ast.AugAssign)) and n.value is not None:
        targets, value = [n.target], n.value
      elif isinstance(n, ast.NamedExpr):
        targets, value = [n.target], n.value
      if value is None or not _tainted(value, names, attrs):
        continue
      for t in targets:
        for e in ast.walk(t):
          if isinstance(e, ast.Name) and isinstance(e.ctx, ast.Store) and e.id not in names:
            names.add(e.id)
            changed = True
  return names


def _class_taint(cls) -> set[str]:
  """self.<attr> assigned (in any method) from an urgency value, to a fixpoint."""
  attrs: set[str] = set()
  if cls is None:
    return attrs
  methods = [f for f in cls.body if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef))]
  changed = True
  while changed:
    changed = False
    for fn in methods:
      names = _local_taint(fn, attrs)
      for n in ast.walk(fn):
        if isinstance(n, ast.Assign) and _tainted(n.value, names, attrs):
          for t in n.targets:
            for e in ast.walk(t):
              if isinstance(e, ast.Attribute) and isinstance(e.value, ast.Name) and e.value.id == "self" and \
                 e.attr not in attrs:
                attrs.add(e.attr)
                changed = True
  return attrs


def _acts_on_urgency(fn, cls_attrs: set[str]) -> bool:
  """Task 13 C2: the function calls the predicate (or takes / reads a value derived from it) AND acts on it: the value
  reaches a decision (if / ternary / while / assert test), a return, or another (non-logging) call. A docstring or
  comment that names the predicate, an unused `urgency` parameter, or a predicate call whose result is dropped do not."""
  names = _local_taint(fn, cls_attrs)
  for n in ast.walk(fn):
    if isinstance(n, (ast.If, ast.IfExp, ast.While, ast.Assert)) and _tainted(n.test, names, cls_attrs):
      return True
    if isinstance(n, ast.Return) and n.value is not None and _tainted(n.value, names, cls_attrs):
      return True
    if isinstance(n, ast.Call) and not _is_predicate_call(n) and _call_name(n) not in LOGGING and \
       not (isinstance(n.func, ast.Attribute) and isinstance(n.func.value, ast.Name) and n.func.value.id in LOGGING) and \
       any(_tainted(a, names, cls_attrs) for a in n.args + [k.value for k in n.keywords]):
      return True
  return False


def scan(roots):
  """found: {key: (fn, class attrs)} for every function that raises a lower limit / writes longActive; index: all."""
  found, index = {}, {}
  for root_dir in roots:
    for root, _, files in os.walk(root_dir):
      if "/tests" in root or root.endswith("tests"):
        continue
      for f in files:
        if f.endswith(".py"):
          classes = {}
          for name, fn, cls in _functions(os.path.join(root, f)):
            if cls is not None and id(cls) not in classes:
              classes[id(cls)] = _class_taint(cls)
            entry = (fn, classes.get(id(cls), set()) if cls is not None else set())
            index[f"{f}:{name}"] = entry
            if _raises_lower_limit(fn):
              found[f"{f}:{name}"] = entry
  return found, index


def violations(found, index, reviewed=None, exempt=None):
  reviewed = REVIEWED if reviewed is None else reviewed
  exempt = EXEMPT if exempt is None else exempt
  out = []
  for key, (fn, attrs) in found.items():
    if key in exempt or _acts_on_urgency(fn, attrs):
      continue
    if key not in reviewed:
      out.append(f"{key} raises a lower accel limit / drops longActive without calling the urgency predicate and acting " +
                 "on it: call it (or take `urgency` and branch on it), or add a REVIEWED entry naming the function that does")
      continue
    gate = reviewed[key][0]
    if gate not in index:
      out.append(f"REVIEWED gate {gate} not found")
    elif not _acts_on_urgency(*index[gate]):
      out.append(f"{gate} (REVIEWED gate for {key}) must call the urgency predicate and act on it")
  return out


PLANTED = {
  "comment_only.py": '''
def limit_accel(accel_limits, urgency=None):
  """Rule 5: consults braking_urgency() (it doesn't)."""
  # braking_urgency / controls_urgency / urgent
  return max(accel_limits[0], -0.3), accel_limits[1]
''',
  "dropped_call.py": '''
from braking_urgency import braking_urgency
def limit_accel(sm, accel_limits):
  braking_urgency(plan_ok=True)
  return max(accel_limits[0], -0.3), accel_limits[1]
''',
  "stored_never_used.py": '''
class Feature:
  def update(self, sm):
    self.urgent = controls_urgency(sm)
  def limit_accel(self, accel_limits):
    """urgent"""
    return max(accel_limits[0], -0.3), accel_limits[1]
''',
  "other_attr.py": '''
class Feature:
  def update(self, sm):
    self.urgent = controls_urgency(sm)
    self.active = True
  def limit_accel(self, accel_limits):
    lo = max(accel_limits[0], -0.3) if self.active else accel_limits[0]
    return lo, accel_limits[1]
''',
  "np_clip.py": '''
import numpy as np
def limit_accel(accel_limits, urgency=None):
  """clamps via np.clip; mentions braking_urgency in the docstring only"""
  return float(np.clip(accel_limits[0], -0.3, 0.)), accel_limits[1]
''',
  "ok_np_clip.py": '''
import numpy as np
def limit_accel(accel_limits, urgency):
  if urgency is not None:
    return accel_limits
  return float(np.clip(accel_limits[0], -0.3, 0.)), accel_limits[1]
''',
  "long_active.py": '''
def state_control(CC, urgency):
  """drops longActive; urgency is ignored"""
  CC.longActive = False
  cloudlog.event("x", urgency=urgency)
''',
  "ok_calls.py": '''
def limit_accel(sm, accel_limits):
  if controls_urgency(sm) is not None:
    return accel_limits
  return max(accel_limits[0], -0.3), accel_limits[1]
''',
  "ok_param.py": '''
def limit_accel(accel_limits, urgency):
  lo = accel_limits[0] if urgency is not None else max(accel_limits[0], -0.3)
  return lo, accel_limits[1]
''',
  "ok_class.py": '''
class Feature:
  def update(self, urgency):
    urgent = urgency is not None
    self.urgent = urgent and True
  def limit_accel(self, accel_limits):
    return (accel_limits[0] if self.urgent else max(accel_limits[0], -0.3)), accel_limits[1]
''',
}


class TestRule5Tripwire(OpenpilotTestCase):
  def scan(self):
    return scan([os.path.join(BASEDIR, "openpilot", rel) for rel in SCAN])

  def test_every_limiter_consults_the_predicate(self):
    found, index = self.scan()
    self.assertIn("clutch_shift_gate.py:ClutchShiftGate.limit_accel", found, "sanity: the scan sees the known sites")
    self.assertIn("manual_low_speed_handback.py:LowSpeedHandback.limit_accel", found)
    self.assertEqual(violations(found, index), [])

  def test_reviewed_and_exempt_entries_are_live(self):
    found, _ = self.scan()
    for key in list(REVIEWED) + list(EXEMPT):
      self.assertIn(key, found, f"stale entry {key}: remove it")
    for key, (test, _) in EXEMPT.items():
      self.assertTrue(any(test in dir(c) for c in (TestRule5Matrix,)), f"{key}: exemption test {test} missing")

  def test_planted_modules(self):
    # Task 13 C2 [review2 N4]: a substring match ("urgen" in a docstring, an unused parameter) used to pass
    import tempfile
    with tempfile.TemporaryDirectory() as d:
      for name, src in PLANTED.items():
        with open(os.path.join(d, name), "w") as f:
          f.write(src)
      found, index = scan([d])
    for name in PLANTED:
      with self.subTest(module=name):
        keys = [k for k in found if k.startswith(name + ":")]
        self.assertTrue(keys, f"{name}: the scan must see its limiter")
        bad = violations({k: found[k] for k in keys}, index, reviewed={}, exempt={})
        if name.startswith("ok_"):
          self.assertEqual(bad, [], name)
        else:
          self.assertTrue(bad, f"{name} must FAIL the tripwire")
