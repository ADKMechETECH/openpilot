"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Session view (DEV-TEST BUILD ONLY) — the default "dev test" entry. The active vehicle profile's test plan re-cut into
~30-minute drive sessions (no profile for this car -> a "no test-console profile" card). Task 16: built from the stock
mici settings components (NavScroller + Big* cards, the same size as stock's settings), one thing per card, white on
dark, readable at arm's length:
  - home: a status card, one card per session (done ✓ / N of M / after X), the calibration card, and the phase view;
  - session page: what the session is for, then ONE CARD PER TOGGLE (a large on/off switch with the toggle's name, its
    current value and what the session needs), ONE CARD PER TEST (status at a glance; tap for the detail page with its
    ARM switch, how-to, conditions and result), and the start-button status in large monospace type.
Toggle taps write through dev_test_params.write_param (the profile's allowlist, capped by the core; feature Params only
while openpilot is disengaged and the car is stopped - Task 12 D2); a refusal shows why on the card. A satisfied
requirement is not flipped off by a tap. openpilot longitudinal itself is shown but can't be set here (Developer panel).
A swipe never fires a card (stock NavScroller invalidates touches while scrolling - D-09). Every render / input /
callback path is guarded: a console bug logs once and never takes the ui process down (D1).
"""
import time


from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigToggle
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test import (DevTestLayoutMici, InfoCard, StatusCard, TestDetailPage,
                                                                    contrast, is_armed, poller, _refresh)
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import ParamNotAllowed, ParamWriteRefused, profile_cache, \
  ui_profile, write_param
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.test_console.sessions import (SHORT, calibration_progress, chip_text, default_test, fmt_value,
                                                                  next_session, next_value, pill_state, ready_check,
                                                                  session_status, tile_line, toggle_states, ui_writable)
from openpilot.sunnypilot.selfdrive.test_console.guard import guarded
from openpilot.sunnypilot.selfdrive.test_console.status_io import P_BUTTON, P_PRESS, result_line
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller

TITLE_SIZE = 44        # card titles (stock BigButton: 48 / 42)
VALUE_SIZE = 32        # card values (stock: 36) - at least 24 everywhere on these screens
PARAMS_PERIOD = 0.5    # s between Param re-reads (file reads; the UI renders at 60 fps)
FLASH_S = 3.0          # s a refusal / note stays on its card
PILL_WORD = {"rec": "● REC", "armed": "○ armed", "PASS": "✓ PASS", "CAPTURED": "✓ captured", "FAIL": "× FAIL",
             "partial": "• partial", "not run": "not run"}


class _ParamCache:
  def __init__(self):
    self._t = -1e9
    self.values: dict = {}

  def read(self, param: str):
    if time.monotonic() - self._t > PARAMS_PERIOD:
      self._t = time.monotonic()
      self.values = {}
    if param not in self.values:
      self.values[param] = ui_state.params.get(param, return_default=True)
    return self.values[param]

  def invalidate(self) -> None:
    self._t = -1e9


params_cache = _ParamCache()


def _live() -> bool:
  return poller.live(time.monotonic())


class _Flash:
  """A short message shown on a card in place of its value (a refused write, 'already set', ...)."""

  def __init__(self):
    self.text, self.until = "", 0.

  def set(self, text: str) -> None:
    self.text, self.until = text, time.monotonic() + FLASH_S

  def get(self) -> str:
    return self.text if time.monotonic() < self.until else ""


def tap_toggle(t, flash: _Flash) -> None:
  """What a toggle card tap does: write next_value(t) through write_param, or say why not."""
  if not t.settable:
    flash.set("set this in Developer settings")
    return
  value = next_value(t)
  if value is None:
    flash.set("already set as this session needs")
    return
  try:
    write_param(ui_state.params, t.param, value)
  except ParamWriteRefused as e:
    flash.set(f"not set: {e}")
    return
  except (ParamNotAllowed, TypeError, ValueError):
    flash.set("not set: can't be written here")
    return
  params_cache.invalidate()


def toggle_value_text(t) -> str:
  """Second line of a toggle card: the current value and what the session needs, in words."""
  now = fmt_value(t.have)
  if t.required:
    need = fmt_value(t.want)
    state = "✓ ready" if t.ok else f"needs {need}"
    return f"now {now} • {state}" + ("" if t.settable else " • Developer")
  if t.choices:
    return f"now {now} • optional • tap: {fmt_value(next_value(t))}"
  return f"now {now} • optional"


class _ToggleCardMixin:
  """Shared by both toggle cards: the session's ToggleState for this Param, the tap, and the value line."""

  def _setup(self, page: "SessionPage", param: str) -> None:
    contrast(self, VALUE_SIZE)
    self._page, self.param, self._flash = page, param, _Flash()

  def state(self):
    return next((t for t in self._page.states() if t.param == self.param), None)

  def _tap(self) -> None:
    t = self.state()
    if t is not None:
      tap_toggle(t, self._flash)

  def _refresh_value(self, t) -> None:
    value = self._flash.get() or toggle_value_text(t)
    if value != self.value:
      self.set_value(value)


class ToggleCard(_ToggleCardMixin, BigToggle):
  """An on/off session toggle: its name, a large stock switch, the current value and what the session needs. The
  switch always shows the Param's real value - a tap writes (or is refused), it never flips the switch by itself."""

  def __init__(self, page: "SessionPage", param: str):
    BigToggle.__init__(self, SHORT.get(param, param), "", font_size=TITLE_SIZE)
    self._setup(page, param)
    self._toggle_callback = guarded(lambda _checked: self._tap())

  @guarded
  def _update_state(self):
    super()._update_state()
    t = self.state()
    if t is not None:
      self.set_checked(bool(t.have))
      self._refresh_value(t)


class ChoiceCard(_ToggleCardMixin, BigButton):
  """A multi-value session setting (e.g. a floor 0.3 / 0.0, a mode): name, current value, tap = the next choice."""

  def __init__(self, page: "SessionPage", param: str):
    BigButton.__init__(self, SHORT.get(param, param), "", font_size=TITLE_SIZE)
    self._setup(page, param)
    self.set_click_callback(guarded(self._tap))

  @guarded
  def _update_state(self):
    super()._update_state()
    t = self.state()
    if t is not None:
      self._refresh_value(t)


def toggle_card(page: "SessionPage", param: str, sample) -> BigButton:
  """The stock switch for a bool setting, a value card for anything else (sample = its required value or a choice)."""
  return ToggleCard(page, param) if isinstance(sample, bool) else ChoiceCard(page, param)


class TestCard(BigButton):
  """One test of the session: id + name, and its state at a glance; tap opens the detail page (ARM is there)."""

  def __init__(self, spec, profile, details: dict):
    super().__init__(f"{spec.id} {spec.label}", "not run", font_size=TITLE_SIZE)
    contrast(self, VALUE_SIZE)
    self.spec, self._profile, self._details = spec, profile, details
    self.set_click_callback(guarded(self._open))

  def _open(self) -> None:
    key = (self._profile.name, self.spec.id)
    if key not in self._details:
      self._details[key] = TestDetailPage(self.spec, self._profile)
    gui_app.push_widget(self._details[key])

  @guarded
  def _update_state(self):
    super()._update_state()
    results, state, live = poller.results, poller.state, _live()
    st = pill_state(self.spec.id, results, state, live)
    if st in ("rec", "armed"):
      value = f"{PILL_WORD[st]} {state.get('progress', '')}".strip()
    else:
      res = results.get(self.spec.id, {})
      value = PILL_WORD[st] if st != "partial" else result_line(res)
    if is_armed(self.spec.id) and st not in ("rec", "armed"):
      value = "○ armed • " + value
    if value != self.value:
      self.set_value(value)


class SessionPage(NavScroller):
  """One drive session: header, then one card per toggle, one card per test, and the start-button status."""

  def __init__(self, session, profile, details: dict):
    super().__init__()
    self.session, self.profile = session, profile
    self._header = InfoCard(f"{session.id}  {session.name}", "")
    self._goal = InfoCard("", session.objective)   # untitled: the objective gets the whole card (up to 5 lines)
    self._status = StatusCard("start button", "")
    self._calib = InfoCard("calibration", "") if session.card else None
    reg = profile.registry
    self.toggle_cards = [toggle_card(self, p, want) for p, want in session.required_toggles] + \
                        [toggle_card(self, p, choices[0]) for p, choices in session.optional_toggles]
    self.test_cards = [TestCard(reg.tests_by_id[tid], profile, details) for tid in session.test_ids]
    items = [self._header, self._goal] + ([self._calib] if self._calib else []) + self.toggle_cards + self.test_cards + \
            [self._status]
    self._scroller.add_widgets(items)

  def states(self):
    return toggle_states(self.session, params_cache.read, ui_writable(self.profile))

  def first_unfinished(self) -> str:
    sel = poller.state.get("selected", "") if _live() else ui_state.params.get("TestConsoleSelected") or ""
    return default_test(self.session, poller.results, sel)

  @guarded
  def _update_state(self):
    super()._update_state()
    _refresh()
    s, results, state, live = self.session, poller.results, poller.state, _live()
    states = self.states()
    if states:
      ready, n = ready_check(states)
      head = "✓ toggles ready" if ready else f"{n} toggle{'s' if n != 1 else ''} to set"
    else:
      head = "telemetry only: no toggles"
    note = "\nset them disengaged + stopped" if ui_state.started and any(t.settable for t in states) else ""
    self._header.update_text(f"{s.id}  {s.name}", head + note)
    if self._calib is not None:
      frac, text = calibration_progress(self.profile.registry, results, state)
      self._calib.update_text(f"calibration {frac:.0%}", text)
    button = self.profile.buttons.label(params_cache.read(P_BUTTON))
    text, _ = chip_text(state, live, self.first_unfinished(), button, params_cache.read(P_PRESS) or "short")
    self._status.update_text("start button", text)


class SessionCard(BigButton):
  def __init__(self, home: "SessionHomeLayoutMici", idx: int, session, profile):
    super().__init__(f"{session.id}  {session.name}", "", font_size=TITLE_SIZE)
    contrast(self, VALUE_SIZE)
    self._home, self.idx, self.session, self._profile = home, idx, session, profile
    self.set_click_callback(guarded(lambda: home.open_session(idx)))

  @guarded
  def _update_state(self):
    super()._update_state()
    reg, results = self._profile.registry, poller.results
    st = session_status(reg, self.session, results)
    calib = calibration_progress(reg, results, poller.state) if self.session.card else None
    value = tile_line(self.session, st, calib)
    if self.session.id == next_session(reg, results):
      value = "▶ next • " + value
    if value != self.value:
      self.set_value(value)


class SessionHomeLayoutMici(NavScroller):
  """dev test home: status, one card per session, the phase view."""

  def __init__(self):
    super().__init__()
    self._phases = DevTestLayoutMici()
    self._status = StatusCard("dev test", "")
    self._phase_card = BigButton("phase view", "every test, by phase", font_size=TITLE_SIZE)
    contrast(self._phase_card, VALUE_SIZE)
    self._phase_card.set_click_callback(guarded(lambda: gui_app.push_widget(self._phases)))
    self._details: dict = {}
    self.pages: dict = {}
    self._profile = object()
    self._build(ui_profile())

  @property
  def sessions(self):
    profile = ui_profile()
    return profile.registry.sessions if profile is not None else ()

  def _build(self, profile) -> None:
    self._profile = profile
    self._scroller._items.clear()
    if profile is None:
      self._scroller.add_widgets([InfoCard("dev test • inactive", profile_cache.reason +
                                           ". Nothing is armed, captured or written on this car.")])
      return
    cards = [SessionCard(self, i, s, profile) for i, s in enumerate(profile.registry.sessions)]
    self._scroller.add_widgets([self._status, *cards, self._phase_card])

  def page(self, idx: int) -> SessionPage:
    profile = ui_profile()
    s = profile.registry.sessions[idx % len(profile.registry.sessions)]
    key = (profile.name, s.id)
    if key not in self.pages:
      self.pages[key] = SessionPage(s, profile, self._details)
    return self.pages[key]

  def open_session(self, idx: int) -> None:
    gui_app.push_widget(self.page(idx))

  @guarded
  def _update_state(self):
    super()._update_state()
    _refresh()
    profile = ui_profile()
    if profile is not self._profile:
      self._build(profile)
    if profile is None:
      return
    results, state, live = poller.results, poller.state, _live()
    if live and state.get("armed"):
      status = f"{state.get('selected')} " + {"capturing": "● REC", "complete": state.get("status") or "done"}.get(
        state.get("state"), state.get("state", ""))
    else:
      status = "live" if live else "offroad"
    nxt = next_session(profile.registry, results)
    self._status.update_text("dev test", status + (f"\nnext: session {nxt}" if nxt else "\nall sessions done"))


def dev_test_home() -> SessionHomeLayoutMici:
  return SessionHomeLayoutMici()
