"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console file IPC (shared by the daemon and the UI; light imports only).

  UI -> daemon   Params TestConsoleSelected (test id) / TestConsoleArmed (bool, cleared on manager start) /
                 TestConsoleBackground. Schema-light on purpose: no custom.capnp ordinal for throwaway tooling.
  daemon -> UI   <dir>/state.json   (live status, ~5 Hz, atomic replace)
                 <dir>/results.json (last result per test; persists across drives, drives the prereq hints)
  logs           <dir>/markers.jsonl (rotating) + cloudlog "test_marker" events in the rlog
                 <dir>/captures/*.jsonl (qualifying samples per capture window)
"""
import json
import os

from openpilot.common.hardware.hw import Paths

DIR_ENV = "TEST_CONSOLE_DIR"
STATE_FILE = "state.json"
RESULTS_FILE = "results.json"
MARKERS_FILE = "markers.jsonl"
STALE_AFTER = 2.0   # s: older state means the daemon isn't running (offroad) or is stuck

# Params (registered in common/params_keys.h on the dev-test branch only)
P_SELECTED = "TestConsoleSelected"
P_ARMED = "TestConsoleArmed"
P_BACKGROUND = "TestConsoleBackground"
P_BUTTON = "TestConsoleStartButton"
P_PRESS = "TestConsoleStartPress"
CONSOLE_PARAMS = (P_SELECTED, P_ARMED, P_BACKGROUND, P_BUTTON, P_PRESS)


def console_dir() -> str:
  # next to swaglog's directory (/data/log -> /data/test_console on device), like the Task 6 coast log
  return os.environ.get(DIR_ENV) or os.path.join(os.path.dirname(Paths.swaglog_root().rstrip("/")), "test_console")


def write_json_atomic(path: str, obj) -> None:
  tmp = f"{path}.tmp"
  with open(tmp, "w") as f:
    json.dump(obj, f, separators=(",", ":"))
  os.replace(tmp, path)


def read_json(path: str) -> dict:
  try:
    with open(path) as f:
      d = json.load(f)
    return d if isinstance(d, dict) else {}
  except (OSError, ValueError):
    return {}


# --- nested-type validation (Task 12 D1): a hand-edited / truncated / older file must never crash a consumer -----------
_STR, _BOOL, _NUM = (str,), (bool,), (int, float)
STATE_TYPES = {"selected": _STR, "armed": _BOOL, "state": _STR, "qualifying": _BOOL, "background": _BOOL,
               "progress": _STR, "frac": _NUM, "status": _STR, "detail": _STR, "stamp": _NUM, "mono": _NUM,
               "exited": _BOOL, "window": _NUM, "elapsed": _NUM, "version": _NUM, "error": _STR, "errors": _NUM,
               "inert": _BOOL, "dropped": _NUM}
RESULT_TYPES = {"status": _STR, "complete": _BOOL, "progress": _STR, "frac": _NUM, "detail": _STR, "runs": _NUM,
                "window": _NUM, "background": _BOOL, "coast_fit": (dict,), "dropped": _NUM}


def _typed(d: dict, types: dict) -> dict:
  out = {}
  for k, v in d.items():
    want = types.get(k)
    if want is None:
      out[k] = v
    elif isinstance(v, want) and not (want is _NUM and isinstance(v, bool)):
      out[k] = v
  return out


def sanitize_state(d) -> dict:
  if not isinstance(d, dict):
    return {}
  out = _typed(d, STATE_TYPES)
  conds = d.get("conditions")
  out["conditions"] = [(str(c[0]), bool(c[1])) for c in conds if isinstance(c, (list, tuple)) and len(c) == 2] \
                      if isinstance(conds, list) else []
  for k in ("bg", "button", "live"):
    out[k] = d[k] if isinstance(d.get(k), dict) else {}
  out["bg"] = {str(k): str(v) for k, v in out["bg"].items()}
  return out


def sanitize_results(d) -> dict:
  if not isinstance(d, dict):
    return {}
  return {k: _typed(v, RESULT_TYPES) for k, v in d.items() if isinstance(k, str) and isinstance(v, dict)}


def read_state() -> dict:
  return sanitize_state(read_json(os.path.join(console_dir(), STATE_FILE)))


def read_results() -> dict:
  return sanitize_results(read_json(os.path.join(console_dir(), RESULTS_FILE)))


class StatusPoller:
  """UI side: re-reads state/results at most every `period` seconds (cheap; the UI calls it every frame)."""

  def __init__(self, period: float = 0.25):
    self.period = period
    self._t = -1e9
    self.state: dict = {}
    self.results: dict = {}

  def update(self, now: float) -> None:
    if now - self._t < self.period:
      return
    self._t = now
    self.state = read_state()
    self.results = read_results()

  def live(self, now: float) -> bool:
    """The daemon is running and its status is fresh (now: time.monotonic(), same clock as the daemon's stamp)."""
    return bool(self.state) and not self.state.get("exited") and now - float(self.state.get("stamp", -1e9)) < STALE_AFTER


def prereq_hint(prereqs, results: dict) -> str:
  """Soft hint only ('recommended complete first'); never blocks arming."""
  missing = []
  for p in prereqs:
    r = results.get(p, {})
    if r.get("status") == "FAIL":
      missing.append(f"{p} FAILED")
    elif not r.get("complete"):
      missing.append(p)
  return "prereqs done" if not missing else "recommended first: " + ", ".join(missing)


def result_line(entry: dict) -> str:
  if not entry:
    return "not run"
  status = entry.get("status")
  if entry.get("complete"):
    return status or "complete"
  return f"{status + ' • ' if status else ''}partial {entry.get('progress', '')}".strip()
