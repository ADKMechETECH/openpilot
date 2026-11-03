"""
fork-only (civic-cruise) gas-override re-surge guard (Task 16 port): on a manual, after the driver blips/lifts the gas
(as in a rev-matched shift), openpilot must not re-open the throttle it just closed.
"""
from types import SimpleNamespace

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP as SP


class _Stub:
  manual = True
  gas_override_reset = False


def _sm(enabled, gas):
  return {'selfdriveState': SimpleNamespace(enabled=enabled), 'carState': SimpleNamespace(gasPressed=gas)}


def test_manual_override_coasts_never_positive():
  s = _Stub()
  s.gas_override_reset = True
  assert SP.reset_accel(s, 1.8) == 0.0      # never re-opens throttle after a lift
  assert SP.reset_accel(s, -0.4) == -0.4    # keeps engine braking / braking


def test_automatic_is_stock():
  s = _Stub()
  s.manual = False
  s.gas_override_reset = True
  assert SP.reset_accel(s, 1.8) == 1.8


def test_no_override_is_stock():
  s = _Stub()
  s.gas_override_reset = False
  assert SP.reset_accel(s, 1.8) == 1.8


def test_sets_on_engaged_gas_clears_when_long_returns():
  s = _Stub()
  SP.update_gas_override(s, _sm(True, True), True)      # engaged + gas + resetting -> set
  assert s.gas_override_reset
  SP.update_gas_override(s, _sm(True, True), False)     # planner no longer resetting (long back) -> cleared
  assert not s.gas_override_reset


def test_not_set_when_disengaged():
  s = _Stub()
  SP.update_gas_override(s, _sm(False, True), True)
  assert not s.gas_override_reset
