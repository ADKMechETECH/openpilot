"""
fork-only (civic-cruise) non-ACC cruise mode.

A self-contained, platform-agnostic read of the non-ACC ("set-speed only") mode. Kept deliberately thin so it ports to a
structurally different fork by rewiring only the two call sites that consume it (the longitudinal planner's candidate
selection, and controlsd's accel-limit floor / sawtooth).

Owner decisions (2026-11-02):
- Non-ACC ignores leads entirely (no emergency collision backstop) - FCW still warns, the driver brakes.
- The mode flips LIVE, engaged or not (no latch). The ACC<->non-ACC transition is smoothed by the consumers, not here.
- Only meaningful when openpilot owns longitudinal (otherwise the camera runs cruise and there is nothing to reshape).
"""
from openpilot.common.params import Params

PARAM = "NonAdaptiveLongitudinal"
READ_HZ = 1.0   # the param is re-read at this rate; a deliberate button/menu flip tolerates <= 1 s latency


class NonAccMode:
  def __init__(self, CP, dt: float):
    self._params = Params()
    self._dt = dt
    self._frame = 0
    self._read_period = max(1, int(round(1.0 / (READ_HZ * dt))))
    self.available = bool(CP.openpilotLongitudinalControl)
    self.active = self.available and self._read()

  def _read(self) -> bool:
    # NonAdaptiveLongitudinal defaults OFF, so get_bool (which ignores registered defaults) reads an unset key as OFF.
    return bool(self._params.get_bool(PARAM))

  def update(self) -> bool:
    """Re-read the mode live (no latch). Call once per planner frame. Returns the current active state."""
    if self.available and (self._frame % self._read_period == 0):
      self.active = self._read()
    elif not self.available:
      self.active = False
    self._frame += 1
    return self.active
