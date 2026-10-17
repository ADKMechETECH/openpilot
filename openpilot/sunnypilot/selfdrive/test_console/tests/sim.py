"""Synthetic 100 Hz Sample streams for the test console tests (a manual Civic in a chosen gear, no physics)."""
import math
from dataclasses import replace

from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import engine_rpm_at
from openpilot.sunnypilot.selfdrive.test_console.signals import Sample

DT = 0.01


class Sim:
  """Holds the current Sample; step() advances time and applies field overrides. Gear 1-6 keeps rpm locked to speed."""

  def __init__(self, **kw):
    self.s = Sample(t=100., v_ego=10., pitch=0., v_cruise=10., plan_valid=True, allow_brake=True, gear_mt=3, rpm=0.)
    self.locked = True       # engine rpm follows the wheel speed through gear_mt (clutch engaged)
    self.idle_rpm = 800.
    self.events: list = []
    self.apply(**kw)

  def apply(self, **kw) -> None:
    self.s = replace(self.s, **kw)

  def _sync(self) -> None:
    s = self.s
    g = s.gear_mt if 1 <= s.gear_mt <= 6 else 0
    if self.locked and g and not s.clutch:
      s.rpm = engine_rpm_at(g, s.v_ego)
    elif not self.locked or s.clutch:
      s.rpm = s.rpm if s.rpm > 0 else self.idle_rpm
    from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import infer_gear, measured_ratio
    s.gear = infer_gear(s.rpm, s.v_ego, s.clutch)
    s.ratio = measured_ratio(s.rpm, s.v_ego)

  def step(self, **kw) -> Sample:
    self.s = replace(self.s, t=self.s.t + DT, events=tuple(self.events), **kw)
    self.events = []
    self.s.v_ego = max(self.s.v_ego + self.s.a_ego * DT, 0.)
    self._sync()
    if math.isfinite(self.s.a_coast):
      self.s.excess = self.s.a_ego - self.s.a_coast
    return self.s

  def run(self, seconds: float, **kw):
    for _ in range(int(round(seconds / DT))):
      yield self.step(**kw)


def feed(analyzer, samples, ok=True):
  notes = []
  for s in samples:
    notes += analyzer.feed(s, ok(s) if callable(ok) else ok)
  return notes
