"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The dev-test UI's ONLY Param write path, and its vehicle profile (DEV-TEST BUILD ONLY). Every write from the dev test
panels goes through write_param(), which refuses any key outside test_console.sessions.ui_writable(profile): the
console's own TestConsole* keys plus the feature Params the active vehicle profile allows (Task 9 Part C — a deliberate,
bounded widening; the daemon itself still writes nothing). No profile for this car -> only the console's own keys.
Setting a feature toggle here is the same as setting it in settings: the features latch it only while openpilot
longitudinal is inactive, so a change made while engaged applies at the next disengage/engage.
"""
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.test_console.profile import inactive_reason, select_profile
from openpilot.sunnypilot.selfdrive.test_console.sessions import ui_writable

_NO_PROFILE = object()


class ParamNotAllowed(ValueError):
  pass


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


def write_param(params, key: str, value, profile=_NO_PROFILE) -> None:
  profile = ui_profile() if profile is _NO_PROFILE else profile
  if key not in ui_writable(profile):
    cloudlog.error(f"dev test UI: refused to write non-allowlisted param {key}")
    raise ParamNotAllowed(key)
  if isinstance(value, bool):
    params.put_bool(key, value)
  else:
    params.put(key, value)
