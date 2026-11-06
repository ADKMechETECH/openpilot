"""
fork-only (civic-cruise) Phase 2: closed-loop proof that non-ACC mode ignores leads.

Uses the stock longitudinal-maneuver Plant. A lead decelerates from set speed to a stop ahead of the ego:
- car in ACC mode (stock): the ego brakes to follow the lead.
- car in its conventional-cruise mode (carState.cruiseState.nonAdaptive = non-ACC): the ego holds its set speed and
  never brakes for the lead (the driver would).
"""

from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant

CRUISE = 20.0  # m/s set speed


def _run(non_adaptive: bool):
  Plant.car_non_adaptive = non_adaptive
  try:
    man = Maneuver(
      "lead decelerates to a stop ahead",
      duration=6.0,
      initial_speed=CRUISE,
      lead_relevancy=True,
      initial_distance_lead=90.0,
      breakpoints=[0.0, 4.0, 6.0],
      speed_lead_values=[CRUISE, 0.0, 0.0],
      cruise_values=[CRUISE, CRUISE, CRUISE],
    )
    _, logs = man.evaluate()
  finally:
    Plant.car_non_adaptive = False
  return logs[:, 3], logs[:, 5]  # ego speed, ego acceleration


class TestNonAccPlanner:
  def test_adaptive_brakes_for_lead(self):
    speed, accel = _run(False)
    assert speed.min() < 10.0, f"stock should brake to follow the stopping lead, min speed {speed.min():.1f}"
    assert accel.min() < -0.8, f"stock should brake, min accel {accel.min():.2f}"

  def test_nonadaptive_ignores_lead(self):
    speed, accel = _run(True)
    assert speed.min() > 18.0, f"non-ACC should hold set speed, min speed {speed.min():.1f}"
    assert accel.min() > -0.3, f"non-ACC should not brake for the lead, min accel {accel.min():.2f}"


class _FlooredPlanner:
  """The real planner with controlsd's ACC-mode coast floor between its output and the car: what the plant drives on is
  the plan's accel clipped at the NonAccController's lower limit, exactly as LongControl does it. The planner itself
  never sees the floor (as on the car) - only the car's resulting speed."""
  GEAR = 5

  def __init__(self, planner, dt):
    from opendbc.car.honda.values import CAR
    from opendbc.car.honda.interface import CarInterface
    from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import controller as C
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP.openpilotLongitudinalControl = True
    self._p, self._c, self._last, self.reasons = planner, C.NonAccController(CP, dt), 0.0, []

  def update(self, sm):
    self._p.update(sm)
    cs = sm['carState']
    lower = self._c.update(True, float(cs.vEgo), float(cs.vCruise) / 3.6, self.GEAR, 0.0, False, -3.5, False,
                           last_accel=self._last, lead_accel_limit=float(self._p.coast_floor_bound))
    self._last = max(float(self._p.output_a_target), lower)
    self.reasons.append(self._p.lead_brake_gate.reason)

  @property
  def output_a_target(self):
    return self._last

  def __getattr__(self, k):
    return getattr(self._p, k)


def run_acc(monkeypatch, floored: bool, **kw):
  """A maneuver in the car's ACC mode on the real planner; floored = with the rebuilt coast floor in the loop.
  Returns (time, gap to the lead, ego speed, ego accel, why braking was allowed per step)."""
  import numpy as np
  made = []
  orig = Plant.__init__

  def init(self, *a, **k):
    orig(self, *a, **k)
    if floored:
      self.planner = _FlooredPlanner(self.planner, self.ts)
      made.append(self.planner)

  monkeypatch.setattr(Plant, "__init__", init)
  _, logs = Maneuver("acc-mode engine-brake priority", **kw).evaluate()
  monkeypatch.setattr(Plant, "__init__", orig)
  logs = np.array(logs)
  return logs[:, 0], logs[:, 2] - logs[:, 1], logs[:, 3], logs[:, 5], (made[0].reasons if made else [])


class TestAccModeEngineBrakePriority:
  """Owner 2026-11-06: in adaptive mode "prioritize engine braking only unless we enter the [approach-warning]
  threshold ... anticipate the decel requirement at our normal minimum follow distance and coast on engine brake. If we
  continue to close the gap and enter our margin, we can brake." And: "stopped car should always cause brakes."
  Closed loop on the real planner (lead MPC) with the rebuilt controller's floor between the plan and the car."""

  def test_a_car_stopping_ahead_is_braked_for_like_stock(self, monkeypatch):
    kw = {"duration": 14.0, "initial_speed": CRUISE, "lead_relevancy": True, "initial_distance_lead": 90.0,
              "breakpoints": [0.0, 4.0, 14.0], "speed_lead_values": [CRUISE, 0.0, 0.0], "cruise_values": [CRUISE] * 3}
    _, gap0, v0, a0, _ = run_acc(monkeypatch, False, **kw)
    _, gap1, v1, a1, why = run_acc(monkeypatch, True, **kw)
    assert gap1.min() > 2.0 and v1[-1] < 0.5, f"must stop behind the stopped car (closest {gap1.min():.1f} m)"
    assert gap1.min() > gap0.min() - 3.0, f"stops about where stock does ({gap1.min():.1f} m vs {gap0.min():.1f} m)"
    assert a1.min() > a0.min() - 0.8, f"without much harder braking than stock ({a1.min():.2f} vs {a0.min():.2f})"
    assert "stopped" in why or "hard" in why

  def test_a_slower_car_is_closed_on_engine_braking_until_the_margin(self, monkeypatch):
    # 67 mph coming up on a car doing 49: stock brakes early and gently; the rebuilt car lifts off, coasts, and brakes
    # only once inside the approach margin - and still never gets dangerously close
    from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.approach import inside
    kw = {"duration": 30.0, "initial_speed": 30.0, "lead_relevancy": True, "initial_distance_lead": 140.0,
              "breakpoints": [0.0, 30.0], "speed_lead_values": [22.0, 22.0], "cruise_values": [30.0, 30.0]}
    _, gap0, v0, a0, _ = run_acc(monkeypatch, False, **kw)
    t, gap1, v1, a1, why = run_acc(monkeypatch, True, **kw)
    braking = a1 < -0.9
    assert braking.any(), "it should need the brakes eventually in this one"
    first = int(braking.argmax())
    assert why[first] in ("margin", "hard"), why[first]
    assert (a1[:first] > -0.9).all()                                  # engine braking only before that
    if why[first] == "margin":
      assert inside(float(v1[first]), float(gap1[first]) - 1.0, 22.0)   # and the lead really was inside the margin
    assert gap1.min() > 0.55 * 22.0, f"closest approach {gap1.min():.1f} m is under 0.55 s behind the lead"
    assert abs(v1[-1] - 22.0) < 1.5                                   # and it ends up following

  def test_a_slightly_slower_car_needs_no_brakes_at_all(self, monkeypatch):
    kw = {"duration": 40.0, "initial_speed": 30.0, "lead_relevancy": True, "initial_distance_lead": 120.0,
              "breakpoints": [0.0, 40.0], "speed_lead_values": [28.0, 28.0], "cruise_values": [30.0, 30.0]}
    _, gap1, v1, a1, why = run_acc(monkeypatch, True, **kw)
    assert a1.min() > -0.9 and gap1.min() > 20.0, f"min accel {a1.min():.2f}, closest {gap1.min():.1f} m"


def _run_bound(monkeypatch, **kw):
  """Run a maneuver in the car's ACC mode and record, per planner step, the plan's accel and the bound it publishes
  for controlsd's ACC-mode coast floor (coastFloorBound)."""
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  rec = []
  orig = LongitudinalPlanner.update

  def update(self, sm):
    orig(self, sm)
    rec.append((float(self.coast_floor_bound), float(self.output_a_target)))

  monkeypatch.setattr(LongitudinalPlanner, "update", update)
  Maneuver("acc-mode coast bound", **kw).evaluate()
  import numpy as np
  r = np.array(rec)
  return r[:, 0], r[:, 1]


class TestCoastFloorBound:
  def test_a_lowered_set_speed_on_a_free_road_leaves_room_to_coast(self, monkeypatch):
    bound, a_target = _run_bound(monkeypatch, duration=6.0, initial_speed=CRUISE, lead_relevancy=False, initial_distance_lead=200.0,
                                 breakpoints=[0.0, 1.0, 1.1, 6.0], speed_lead_values=[CRUISE] * 4, cruise_values=[CRUISE, CRUISE, CRUISE - 5.0, CRUISE - 5.0])
    slowing = a_target < -0.5                       # stock ACC brakes for the tap
    assert slowing.any()
    assert (bound[slowing] > 1.0).all(), "with no lead nothing needs the brakes: the coast floor acts in full"

  def test_bound_passes_braking_once_the_lead_is_stopped(self, monkeypatch):
    bound, a_target = _run_bound(monkeypatch, duration=6.0, initial_speed=CRUISE, lead_relevancy=True, initial_distance_lead=90.0,
                                 breakpoints=[0.0, 4.0, 6.0], speed_lead_values=[CRUISE, 0.0, 0.0], cruise_values=[CRUISE, CRUISE, CRUISE])
    hard = a_target < -2.0
    assert hard.any()
    assert (bound[hard] <= a_target[hard] + 1e-6).all(), "hard braking for a stopping car must pass the floor in full"
