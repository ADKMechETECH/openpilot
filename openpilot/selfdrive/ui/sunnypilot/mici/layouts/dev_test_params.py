"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The dev-test UI's ONLY Param write path, and its vehicle profile (DEV-TEST BUILD ONLY). Every write from the dev test
panels goes through write_param(), which refuses any key outside test_console.sessions.ui_writable(profile): the
console's own TestConsole* keys plus the feature Params the active vehicle profile allows (Task 9 Part C — a deliberate,
bounded widening; the daemon itself still writes nothing). No profile for this car -> only the console's own keys.
Feature Params are written only while openpilot is disengaged and the car is stopped (or offroad) — Task 12 D2; the
features latch only while disengaged, so a write here applies at the next engagement. The core caps every profile's
allowlist (sessions.CORE_MAX_ALLOW / CORE_DENY).
"""
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.test_console.profile import inactive_reason, select_profile
from openpilot.sunnypilot.selfdrive.test_console.sessions import ui_writable, write_refusal

_NO_PROFILE = object()


class ParamNotAllowed(ValueError):
  pass


class ParamWriteRefused(ValueError):
  """A permitted Param, but not now (Task 12 D2: feature Params only while disengaged and stopped / offroad)."""


class _ProfileCache:
  """The vehicle profile for the car the UI knows (CarParamsPersistent), re-selected when that changes."""

  def __init__(self):
    self._cp = _NO_PROFILE
    self.profile = None
    self.reason = inactive_reason(None)

  def get(self):
    from openpilot.selfdrive.ui.ui_state import ui_state
    cp = ui_state.CP
    if cp is not self._cp:
      self._cp = cp
      self.profile = select_profile(cp)
      self.reason = inactive_reason(cp)
    return self.profile


profile_cache = _ProfileCache()


def ui_profile():
  return profile_cache.get()


def _car_state():
  """(onroad, openpilot engaged, standstill). Engaged = selfdriveState.enabled: what the features latch on."""
  from openpilot.selfdrive.ui.ui_state import ui_state
  if not ui_state.started:
    return False, False, True
  return True, bool(ui_state.sm['selfdriveState'].enabled), bool(ui_state.sm['carState'].standstill)


def write_param(params, key: str, value, profile=_NO_PROFILE, car_state=None) -> None:
  """car_state: (onroad, engaged, standstill); default = the UI's live state."""
  profile = ui_profile() if profile is _NO_PROFILE else profile
  if key not in ui_writable(profile):
    cloudlog.error(f"dev test UI: refused to write non-allowlisted param {key}")
    raise ParamNotAllowed(key)
  reason = write_refusal(key, *(car_state if car_state is not None else _car_state()))
  if reason is not None:
    raise ParamWriteRefused(reason)
  if isinstance(value, bool):
    params.put_bool(key, value)
  else:
    params.put(key, value)
