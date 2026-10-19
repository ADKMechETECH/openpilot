"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console registry engine (vehicle-agnostic): the shapes of a test plan and the lookups the core evaluates them with.
The plans themselves (the tests, phases and sessions for one car) live in its profile (profiles/<car>/).

Each TestSpec row names its pieces instead of holding code, so a test is added by adding a row — no rewiring:
  conditions  names in conditions.CONDITIONS (or the profile's own); a sample QUALIFIES only while all hold (+ toggles)
  toggles     Params the test expects ON/OFF. Read-only checks: the daemon never sets a toggle
  analyzer    "segments" (qualifying time, optionally binned) | "episodes" (detected maneuvers) | a custom analyzer name
  goal        the analyzer's parameters (what "complete" means)
  result      a name in the profile's results (the core's generic ones too): PASS / FAIL / CAPTURED + a one-line detail
  passes      the pass criterion in words ("" = characterization only: CAPTURED, no PASS/FAIL)
  fields      Sample fields written per qualifying sample to the capture file
  prereqs     tests recommended complete first (a soft hint — never a block; driver judgment)
  session     the ~30-minute drive session it belongs to (a card session = filled by background telemetry)
  background  False = never captured by background telemetry (deliberate, driver-performed tests)

Two kinds of test:
  TELEMETRY  passive, no toggle, order-independent: collect opportunistically on any drive (also in the background)
  FEATURE    needs openpilot long and/or a toggle: follow the phase order
"""
from dataclasses import dataclass, field
from enum import StrEnum

OP_LONG_PARAM = "AlphaLongitudinalEnabled"  # shown as a requirement; NEVER settable from the console UI (Developer panel)
CAL = "Cal"                                 # conventional id of the calibration card session


class Kind(StrEnum):
  TELEMETRY = "telemetry"
  FEATURE = "feature"


@dataclass(frozen=True)
class Phase:
  num: int
  title: str
  setup: str   # how the car must be set up for this phase
  gates: str   # what this phase gates


@dataclass(frozen=True)
class TestSpec:
  id: str
  phase: int
  kind: Kind
  label: str                      # short: fits a mici button
  how: tuple[str, ...]            # what the driver does (one card per entry)
  conditions: tuple[str, ...]
  analyzer: str
  goal: dict
  result: str
  passes: str
  fields: tuple[str, ...]
  prereqs: tuple[str, ...] = ()
  toggles: dict[str, bool] = field(default_factory=dict)
  first_run: bool = False         # part of the first-run characterization on a new car
  optional: bool = False
  sample_hz: int = 20             # capture-file rate for qualifying samples (analysis always runs at 100 Hz)
  session: str = ""
  background: bool = True         # telemetry only: may be captured by background telemetry


@dataclass(frozen=True)
class Session:
  id: str
  name: str                                        # short: fits a home-grid tile
  objective: str                                   # one line
  required_toggles: tuple[tuple[str, object], ...]  # (param, value) the session needs; the UI shows ready / set N
  test_ids: tuple[str, ...]
  prereqs: tuple[str, ...] = ()                    # sessions recommended done first (soft hint, never a block)
  optional_toggles: tuple[tuple[str, tuple], ...] = ()  # (param, choices): switches offered, not required (A/B, floors)
  card: bool = False                               # a progress card filled by background telemetry, not a drive


class Registry:
  """One car's plan: ordered tests, their phases and sessions, with the lookups the core uses."""

  def __init__(self, phases: dict[int, Phase], tests: tuple[TestSpec, ...], sessions: tuple[Session, ...],
               first_run_note: str = "Run the first-run characterization tests first."):
    self.phases, self.tests, self.sessions = phases, tests, sessions
    self.first_run_note = first_run_note   # shown on the phase view's "first run" card
    self.tests_by_id: dict[str, TestSpec] = {t.id: t for t in tests}
    self.sessions_by_id: dict[str, Session] = {s.id: s for s in sessions}
    assert len(self.tests_by_id) == len(tests), "duplicate test id"
    assert len(self.sessions_by_id) == len(sessions), "duplicate session id"
    assert all(t.session in self.sessions_by_id for t in tests), "every test needs a session"
    assert all(tid in self.tests_by_id for s in sessions for tid in s.test_ids), "sessions list known tests"

  def tests_in_phase(self, phase: int) -> list[TestSpec]:
    return [t for t in self.tests if t.phase == phase]

  def tests_in_session(self, session_id: str) -> list[TestSpec]:
    return [self.tests_by_id[t] for t in self.sessions_by_id[session_id].test_ids]

  @property
  def card(self) -> Session | None:
    """The calibration card (the first card session), if the plan has one."""
    return next((s for s in self.sessions if s.card), None)
