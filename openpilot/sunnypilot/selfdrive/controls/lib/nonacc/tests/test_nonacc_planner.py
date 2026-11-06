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


def _run_bound(monkeypatch, **kw):
  """Run a maneuver in the car's ACC mode and record, per planner step, the plan's accel and the bound it publishes
  for controlsd's ACC-mode coast floor (accelNoSetSpeed)."""
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  rec = []
  orig = LongitudinalPlanner.update

  def update(self, sm):
    orig(self, sm)
    rec.append((float(self.accel_no_set_speed), float(self.output_a_target)))

  monkeypatch.setattr(LongitudinalPlanner, "update", update)
  Maneuver("acc-mode coast bound", **kw).evaluate()
  import numpy as np
  r = np.array(rec)
  return r[:, 0], r[:, 1]


class TestAccModeCoastBound:
  """Owner 2026-11-06: adaptive cruise coasts to a lowered set speed and brakes only if necessary. The planner publishes
  what the plan asks for with the set speed left out; controlsd's floor is never raised above it."""

  def test_braking_for_a_lead_is_never_floored(self, monkeypatch):
    bound, a_target = _run_bound(monkeypatch, duration=6.0, initial_speed=CRUISE, lead_relevancy=True, initial_distance_lead=90.0,
                                 breakpoints=[0.0, 4.0, 6.0], speed_lead_values=[CRUISE, 0.0, 0.0], cruise_values=[CRUISE, CRUISE, CRUISE])
    braking = a_target < -0.5
    assert braking.any()
    assert (bound[braking] <= a_target[braking] + 1e-6).all(), "a floor at the bound would have taken braking away from the lead plan"

  def test_a_lowered_set_speed_on_a_free_road_leaves_room_to_coast(self, monkeypatch):
    bound, a_target = _run_bound(monkeypatch, duration=6.0, initial_speed=CRUISE, lead_relevancy=False, initial_distance_lead=200.0,
                                 breakpoints=[0.0, 1.0, 1.1, 6.0], speed_lead_values=[CRUISE] * 4, cruise_values=[CRUISE, CRUISE, CRUISE - 5.0, CRUISE - 5.0])
    slowing = a_target < -0.5                       # stock ACC brakes for the tap
    assert slowing.any()
    assert (bound[slowing] > -0.1).all(), "with no lead the plan needs no braking: the coast floor may act"

  def test_bound_is_no_floor_until_the_planner_has_spoken(self):
    from opendbc.car.honda.values import CAR
    from opendbc.car.honda.interface import CarInterface
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    p = LongitudinalPlannerSP(CP, CP_SP, LongitudinalMpc())
    assert p.accel_no_set_speed <= -3.5             # default: the minimum accel = no floor in ACC mode
    p.update_accel_no_set_speed([0.4, -0.2], False, -1.2, 2.0)
    assert p.accel_no_set_speed == -0.2             # the most demanding of the lead / e2e candidates
    p.update_accel_no_set_speed([0.4], True, -1.2, 2.0)
    assert p.accel_no_set_speed == -1.2             # a stop is wanted: no floor
    p.update_accel_no_set_speed([], False, -1.2, 2.0)
    assert p.accel_no_set_speed == -1.2             # non-ACC: no candidates, the bound is not used
