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
