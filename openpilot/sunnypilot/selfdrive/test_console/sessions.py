"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Session view state (DEV-TEST BUILD ONLY): pure functions from the registry's SESSIONS, the daemon's state.json /
results.json and the current Param values to what the session UI shows — tile status (done / current / locked),
the ready check (required toggles vs current -> ready / set N), test pills, the calibration card, the status chip.
No I/O and no writes here; the UI's only write path is selfdrive/ui/sunnypilot/mici/layouts/dev_test_params.py,
limited to UI_WRITABLE below.
"""
from collections.abc import Callable
from dataclasses import dataclass

from openpilot.sunnypilot.selfdrive.test_console.registry import CAL, OP_LONG_PARAM, SESSIONS, SESSIONS_BY_ID, TESTS_BY_ID, Session
from openpilot.sunnypilot.selfdrive.test_console.status_io import CONSOLE_PARAMS

# The only Params the session UI may write (Task 9 Part C: a deliberate, bounded widening of the UI's write scope).
# Feature toggles the separately-validated, default-OFF, gated features read, plus the console's own keys. Notably NOT
# AlphaLongitudinalEnabled (openpilot longitudinal: disables stock AEB on this car) — that stays in the Developer panel.
# Task 11 added NoServiceBrakeOverspeedMode (maintain / pulse: the NSB2 A/B in session 8).
UI_WRITABLE_TOGGLES = ("NonAdaptiveLongitudinal", "NoServiceBrakeHold", "NoServiceBrakeDecelFloor",
                       "NoServiceBrakeOverspeedBackstop", "NoServiceBrakeOverspeedMargin", "NoServiceBrakeOverspeedMode",
                       "ClutchShiftGate", "ClutchShiftGateGasReleaseHint", "ClutchReengageSmart", "EngineBrakePreferred",
                       "GearAwareAccel", "DisengageOnAccelerator")
UI_WRITABLE = frozenset(UI_WRITABLE_TOGGLES) | frozenset(CONSOLE_PARAMS)

SHORT = {OP_LONG_PARAM: "op long", "NonAdaptiveLongitudinal": "non-adaptive", "NoServiceBrakeHold": "NSB",
         "NoServiceBrakeDecelFloor": "floor", "NoServiceBrakeOverspeedBackstop": "backstop",
         "NoServiceBrakeOverspeedMargin": "margin", "NoServiceBrakeOverspeedMode": "mode",
         "ClutchShiftGate": "gate", "ClutchShiftGateGasReleaseHint": "gas hint",
         "ClutchReengageSmart": "smart", "EngineBrakePreferred": "EBP", "GearAwareAccel": "gear-aware",
         "DisengageOnAccelerator": "DOA"}


def fmt_value(v) -> str:
  if isinstance(v, bool):
    return "ON" if v else "OFF"
  if isinstance(v, str):
    return v
  if v is None:
    return "unset"
  return f"{float(v):.1f}"


def same_value(want, have) -> bool:
  if isinstance(want, bool):
    return bool(have) == want
  if isinstance(want, str):
    return isinstance(have, str) and have.strip().lower() == want
  try:
    return abs(float(have) - float(want)) < 1e-6
  except (TypeError, ValueError):
    return False


@dataclass(frozen=True)
class ToggleState:
  param: str
  want: object          # required value (None for an optional toggle)
  have: object
  ok: bool              # matches the requirement (always True for optional ones)
  settable: bool        # in UI_WRITABLE
  required: bool
  choices: tuple = ()

  @property
  def label(self) -> str:
    return f"{SHORT.get(self.param, self.param)} {fmt_value(self.have)}"


def toggle_states(session: Session, read: Callable[[str], object]) -> list[ToggleState]:
  """read(param) -> current value (Params.get(..., return_default=True))."""
  out = []
  for param, want in session.required_toggles:
    have = read(param)
    have = bool(have) if isinstance(want, bool) else have
    out.append(ToggleState(param, want, have, same_value(want, have), param in UI_WRITABLE, True))
  for param, choices in session.optional_toggles:
    have = read(param)
    have = bool(have) if isinstance(choices[0], bool) else have
    out.append(ToggleState(param, None, have, True, param in UI_WRITABLE, False, tuple(choices)))
  return out


def ready_check(states: list[ToggleState]) -> tuple[bool, int]:
  """(ready, how many required toggles still need setting)."""
  n = sum(1 for t in states if t.required and not t.ok)
  return n == 0, n


def ready_text(states: list[ToggleState]) -> str:
  ready, n = ready_check(states)
  return "ready ✓" if ready else f"set {n}"


def next_value(t: ToggleState):
  """What a tap on this switch writes: the requirement if unmet, else the next choice (optional), else a flip."""
  if t.required and not t.ok:
    return t.want
  if t.choices:
    idx = next((i for i, c in enumerate(t.choices) if same_value(c, t.have)), -1)
    return t.choices[(idx + 1) % len(t.choices)]
  return not bool(t.have) if isinstance(t.want, bool) else t.want


# --- results -> status ------------------------------------------------------------------------------------------------
def test_done(results: dict, test_id: str) -> bool:
  r = results.get(test_id, {})
  return bool(r.get("complete")) and r.get("status") != "FAIL"


def required_tests(session: Session) -> list[str]:
  return [t for t in session.test_ids if not TESTS_BY_ID[t].optional]


@dataclass(frozen=True)
class SessionStatus:
  state: str            # "done" | "current" | "locked"
  done: int
  total: int
  gating: tuple[str, ...]  # sessions not done yet that this one recommends first (locked)
  failed: tuple[str, ...]


def session_done(session: Session, results: dict) -> bool:
  return all(test_done(results, t) for t in required_tests(session))


def session_status(session: Session, results: dict) -> SessionStatus:
  req = required_tests(session)
  done = sum(test_done(results, t) for t in req)
  failed = tuple(t for t in session.test_ids if results.get(t, {}).get("status") == "FAIL")
  if done == len(req):
    return SessionStatus("done", done, len(req), (), failed)
  gating = tuple(p for p in session.prereqs if not session_done(SESSIONS_BY_ID[p], results))
  return SessionStatus("locked" if gating else "current", done, len(req), gating, failed)


def next_session(results: dict) -> str | None:
  """The recommended next drive: the first non-card session that is current."""
  return next((s.id for s in SESSIONS if not s.card and session_status(s, results).state == "current"), None)


def tile_line(session: Session, st: SessionStatus, calib: tuple[float, str] | None = None) -> str:
  if session.card and calib is not None:
    return calib[1]
  if st.state == "done":
    return "✓ done" + (f" • {len(st.failed)} FAIL" if st.failed else "")
  if st.state == "locked":
    return "after " + ", ".join(st.gating)
  return f"{st.done}/{st.total} tests" + (f" • {len(st.failed)} FAIL" if st.failed else "")


def calibration_progress(results: dict, state: dict) -> tuple[float, str]:
  """The Cal card: mean completion of its tests (results.json, or the live background progress text)."""
  cal = SESSIONS_BY_ID[CAL]
  fracs = []
  for t in cal.test_ids:
    r = results.get(t, {})
    fracs.append(1. if r.get("complete") else float(r.get("frac", 0.) or 0.))
  frac = sum(fracs) / len(fracs)
  live = bool(state.get("bg")) and any(t in state["bg"] for t in cal.test_ids)
  return frac, f"{frac:.0%}" + (" • collecting" if live else " • background")


# --- tests / chip -----------------------------------------------------------------------------------------------------
def pill_state(test_id: str, results: dict, state: dict, live: bool) -> str:
  """'rec' | 'armed' | 'PASS' | 'FAIL' | 'CAPTURED' | 'partial' | 'not run'."""
  if live and state.get("armed") and state.get("selected") == test_id:
    return "rec" if state.get("state") == "capturing" else "armed"
  r = results.get(test_id, {})
  if r.get("complete"):
    return r.get("status") or "CAPTURED"
  if r.get("status") == "FAIL":
    return "FAIL"
  return "partial" if r else "not run"


PILL_MARK = {"rec": "●", "armed": "○", "PASS": "✓", "FAIL": "×", "CAPTURED": "✓", "partial": "•", "not run": ""}


def default_test(session: Session, results: dict, selected: str) -> str:
  """Pill selected on opening a session: the armed/selected test if it's here, else the first one not done."""
  if selected in session.test_ids:
    return selected
  return next((t for t in session.test_ids if not test_done(results, t)), session.test_ids[0])


def chip_text(state: dict, live: bool, test_id: str, button: str = "gap", press: str = "short") -> tuple[str, str]:
  """(text, color key) for the session page's start-button chip."""
  armed_here = live and state.get("armed") and state.get("selected") == test_id
  if not armed_here:
    if live and state.get("armed"):
      return f"{state.get('selected')} is armed • ARM {test_id} to switch", "idle"
    return f"ARM {test_id}, then {press}-press {button} to START", "idle"
  st, progress = state.get("state"), state.get("progress", "")
  if st == "capturing":
    return f"STOP • ● REC  {progress}".strip(), "capturing"
  if st == "stopped":
    return f"PAUSED {progress} • press to resume".strip(), "stopped"
  if st == "complete":
    status = state.get("status") or "DONE"
    return f"{status} • press to re-run", status if status in ("PASS", "FAIL") else "CAPTURED"
  return f"armed • {press}-press {button} to START", "armed"
