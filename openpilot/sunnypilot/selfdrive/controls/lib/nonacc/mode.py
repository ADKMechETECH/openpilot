"""
fork-only (civic-cruise) non-ACC cruise mode.

A self-contained read of the non-ACC ("set-speed only") mode. Kept deliberately thin so it ports to a structurally
different fork by rewiring only the call sites that consume it (the longitudinal planner's candidate selection,
controlsd's accel-limit floor / sawtooth, and the car-event filter).

The switch is the CAR'S OWN ACC <-> Cruise mode (owner decision 2026-11-05, session 12): a ~1.2 s distance-button hold,
kept by the stock camera and read by the car interface as carState.cruiseState.nonAdaptive. Non-ACC is active exactly
when the car is in its conventional-cruise mode. It replaced the NonAdaptiveLongitudinal param and the fork's own hold
toggle, which fired on the same hold as the car's switch: openpilot's stock wrongCruiseMode then refused to engage
whenever non-ACC was armed. There is no param and no write on the driving path; the car remembers the mode.

Owner decisions:
- Non-ACC ignores leads entirely (no emergency collision backstop) - FCW still warns, the driver brakes.
- The mode flips LIVE, engaged or not (no latch). The ACC<->non-ACC transition is smoothed by the consumers, not here.
- Only on a manual with openpilot longitudinal (the fork's scope). Everywhere else the mode is never active and the
  stock wrongCruiseMode refusal stays.
- A missing / invalid camera flag reads as ACC (lead following), the stock-safe side.
"""
from opendbc.car import structs


def nonacc_available(CP) -> bool:
  """True where the car's conventional-cruise mode means the fork's non-ACC mode (instead of 'wrong cruise mode')."""
  return bool(CP.openpilotLongitudinalControl) and CP.transmissionType == structs.CarParams.TransmissionType.manual


class NonAccMode:
  def __init__(self, CP, dt: float):
    self._dt = dt
    self.available = nonacc_available(CP)
    self.active = False

  def update(self, car_non_adaptive: bool) -> bool:
    """Follow the car's mode live (no latch). Call once per frame with carState.cruiseState.nonAdaptive."""
    self.active = bool(self.available and car_non_adaptive)
    return self.active
