"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The dev-test UI's ONLY Param write path (DEV-TEST BUILD ONLY). Every write from the dev test panels goes through
write_param(), which refuses any key outside test_console.sessions.UI_WRITABLE: the console's own TestConsole* keys
plus the feature toggles a session needs (Task 9 Part C — a deliberate, bounded widening; the daemon itself still
writes nothing). Setting a feature toggle here is the same as setting it in settings: the features latch it only while
openpilot longitudinal is inactive, so a change made while engaged applies at the next disengage/engage.
"""
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.test_console.sessions import UI_WRITABLE


class ParamNotAllowed(ValueError):
  pass


def write_param(params, key: str, value) -> None:
  if key not in UI_WRITABLE:
    cloudlog.error(f"dev test UI: refused to write non-allowlisted param {key}")
    raise ParamNotAllowed(key)
  if isinstance(value, bool):
    params.put_bool(key, value)
  else:
    params.put(key, value)
