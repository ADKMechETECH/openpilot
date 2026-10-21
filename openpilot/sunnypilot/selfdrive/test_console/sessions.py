"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Session view state (vehicle-agnostic): pure functions from a plan's sessions (registry.Registry), the daemon's
state.json / results.json and the current Param values to what the session UI shows — tile status (done / current /
locked), the ready check (required toggles vs current -> ready / set N), test pills, the calibration card, the status
chip. No I/O and no writes here; the UI's only write path is selfdrive/ui/sunnypilot/mici/layouts/dev_test_params.py,
limited to ui_writable(profile) below.
"""
from collections.abc import Callable
from dataclasses import dataclass

from openpilot.sunnypilot.selfdrive.test_console.registry import OP_LONG_PARAM, Registry, Session
from openpilot.sunnypilot.selfdrive.test_console.status_io import CONSOLE_PARAMS


# Task 12 D1: the core's MAXIMUM allowlist - a profile can narrow it, never widen it. The fork's longitudinal feature
# Params (plus DisengageOnAccelerator, which the clutch gate needs OFF); nothing that tunes lateral/longitudinal control,
# switches openpilot or its longitudinal on, or changes the driving model.
CORE_MAX_ALLOW = frozenset({
  "NonAdaptiveLongitudinal", "NoServiceBrakeHold", "NoServiceBrakeDecelFloor", "NoServiceBrakeOverspeedBackstop",
  "NoServiceBrakeOverspeedMargin", "NoServiceBrakeOverspeedMode", "ClutchShiftGate", "ClutchShiftGateGasReleaseHint",
  "ClutchReengageSmart", "EngineBrakePreferred", "GearAwareAccel", "DisengageOnAccelerator",
})
# ...and, explicitly, never these (documentation + belt and braces against an edit of the set above)
CORE_DENY = frozenset({OP_LONG_PARAM, "ExperimentalMode", "OpenpilotEnabledToggle", "LongitudinalPersonality",
                       "EnforceTorqueControl", "TorqueControlTune", "NeuralNetworkLateralControl", "Mads",
                       "IsLdwEnabled", "ShowDebugInfo"})
# feature Params: written only while openpilot is disengaged AND the car is stopped (or offroad) - D2
FEATURE_WRITE_RULE = "disengaged + stopped"


def ui_writable(profile) -> frozenset[str]:
  """The only Params the session UI may write: the profile's allowlist (feature Params its sessions need), capped by
  CORE_MAX_ALLOW, plus the console's own keys. Never AlphaLongitudinalEnabled (openpilot longitudinal: disables stock
  AEB) or anything in CORE_DENY — even if a profile listed it. No profile (console inactive) -> only the console's keys."""
  allow = frozenset(profile.allowlist) & CORE_MAX_ALLOW if profile is not None else frozenset()
  return (allow | frozenset(CONSOLE_PARAMS)) - CORE_DENY


def write_refusal(key: str, onroad: bool, engaged: bool, standstill: bool) -> str | None:
  """Task 12 D2: why a write must be refused right now, or None. The console's own keys are always writable; a feature
  Param only while openpilot is disengaged and the car is stopped (or offroad) - the features latch only while
  disengaged anyway, and this makes "applies when disengaged" literally true."""
  if key in CONSOLE_PARAMS:
    return None
  if engaged:
    return "disengage first"
  if onroad and not standstill:
    return "stop the car first"
  return None


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
  settable: bool        # in ui_writable(profile)
  required: bool
  choices: tuple = ()

  @property
  def label(self) -> str:
    return f"{SHORT.get(self.param, self.param)} {fmt_value(self.have)}"


def toggle_states(session: Session, read: Callable[[str], object], writable: frozenset[str]) -> list[ToggleState]:
  """read(param) -> current value (Params.get(..., return_default=True)); writable = ui_writable(profile)."""
  out = []
  for param, want in session.required_toggles:
    have = read(param)
    have = bool(have) if isinstance(want, bool) else have
    out.append(ToggleState(param, want, have, same_value(want, have), param in writable, True))
  for param, choices in session.optional_toggles:
    have = read(param)
    have = bool(have) if isinstance(choices[0], bool) else have
    out.append(ToggleState(param, None, have, True, param in writable, False, tuple(choices)))
  return out


def ready_check(states: list[ToggleState]) -> tuple[bool, int]:
  """(ready, how many required toggles still need setting)."""
  n = sum(1 for t in states if t.required and not t.ok)
  return n == 0, n


def ready_text(states: list[ToggleState]) -> str:
  ready, n = ready_check(states)
  return "ready ✓" if ready else f"set {n}"


def next_value(t: ToggleState):
  """What a tap on this switch writes: the requirement if unmet, else the next choice (optional). A satisfied
  requirement is NOT flipped off by a tap (Task 12 D2): None = write nothing."""
  if t.required:
    return None if t.ok else t.want
  if t.choices:
    idx = next((i for i, c in enumerate(t.choices) if same_value(c, t.have)), -1)
    return t.choices[(idx + 1) % len(t.choices)]
  return not bool(t.have)


# --- results -> status ------------------------------------------------------------------------------------------------
def test_done(results: dict, test_id: str) -> bool:
  r = results.get(test_id, {})
  return bool(r.get("complete")) and r.get("status") != "FAIL"


def required_tests(reg: Registry, session: Session) -> list[str]:
  return [t for t in session.test_ids if not reg.tests_by_id[t].optional]


@dataclass(frozen=True)
class SessionStatus:
  state: str            # "done" | "current" | "locked"
  done: int
  total: int
  gating: tuple[str, ...]  # sessions not done yet that this one recommends first (locked)
  failed: tuple[str, ...]


def session_done(reg: Registry, session: Session, results: dict) -> bool:
  return all(test_done(results, t) for t in required_tests(reg, session))


def session_status(reg: Registry, session: Session, results: dict) -> SessionStatus:
  req = required_tests(reg, session)
  done = sum(test_done(results, t) for t in req)
  failed = tuple(t for t in session.test_ids if results.get(t, {}).get("status") == "FAIL")
  if done == len(req):
    return SessionStatus("done", done, len(req), (), failed)
  gating = tuple(p for p in session.prereqs if not session_done(reg, reg.sessions_by_id[p], results))
  return SessionStatus("locked" if gating else "current", done, len(req), gating, failed)


def next_session(reg: Registry, results: dict) -> str | None:
  """The recommended next drive: the first non-card session that is current."""
  return next((s.id for s in reg.sessions if not s.card and session_status(reg, s, results).state == "current"), None)


def tile_line(session: Session, st: SessionStatus, calib: tuple[float, str] | None = None) -> str:
  if session.card and calib is not None:
    return calib[1]
  if st.state == "done":
    return "✓ done" + (f" • {len(st.failed)} FAIL" if st.failed else "")
  if st.state == "locked":
    return "after " + ", ".join(st.gating)
  return f"{st.done}/{st.total} tests" + (f" • {len(st.failed)} FAIL" if st.failed else "")


def calibration_progress(reg: Registry, results: dict, state: dict) -> tuple[float, str]:
  """The calibration card: mean completion of its tests (results.json, or the live background progress text)."""
  cal = reg.card
  if cal is None or not cal.test_ids:
    return 0., "no calibration card"
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
    return f"● REC  {progress} • press to pause".strip(), "capturing"
  if st == "stopped":
    return f"PAUSED {progress} • press to resume".strip(), "stopped"
  if st == "complete":
    status = state.get("status") or "DONE"
    return f"{status} • press to re-run", status if status in ("PASS", "FAIL") else "CAPTURED"
  return f"armed • {press}-press {button} to START", "armed"
