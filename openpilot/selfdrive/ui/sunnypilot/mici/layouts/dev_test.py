"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

"dev test" phase view (DEV-TEST BUILD ONLY): the on-car test console's ordered validation plan, grouped by phase —
the secondary view; the session view (dev_test_sessions.py) is the default entry. The plan is the active vehicle
profile's (rebuilt when the car changes; no profile -> a "no test-console profile" card). Select a test, arm it; the
steering-wheel button starts/stops its capture. This view writes only the console's own TestConsole* Params, through
dev_test_params.write_param (the session view may also set its sessions' allowlisted feature toggles there).
Arming is never blocked: the prereq line is a recommendation, the driver decides.
"""
import time

import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigMultiToggle, BigToggle, GreyBigButton
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import profile_cache, ui_profile, write_param
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.test_console.button import PRESS_MODES
from openpilot.sunnypilot.selfdrive.test_console.conditions import Context, condition_label, toggle_label
from openpilot.sunnypilot.selfdrive.test_console.registry import TestSpec
from openpilot.sunnypilot.selfdrive.test_console.status_io import (P_ARMED, P_BACKGROUND, P_PRESS, P_SELECTED, StatusPoller,
                                                                     prereq_hint, result_line)
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller

poller = StatusPoller()

STATE_TEXT = {"idle": "not armed", "armed": "ARMED: press to start", "capturing": "● REC", "stopped": "PAUSED", "complete": "DONE"}


def _refresh() -> None:
  poller.update(time.monotonic())


def set_armed(test_id: str, armed: bool) -> None:
  """Arm test_id (disarming whatever was armed), or disarm it if it is the selected one."""
  if armed:
    write_param(ui_state.params, P_SELECTED, test_id)
    write_param(ui_state.params, P_ARMED, True)
  elif ui_state.params.get(P_SELECTED) == test_id:
    write_param(ui_state.params, P_ARMED, False)


def is_armed(test_id: str) -> bool:
  return ui_state.params.get(P_SELECTED) == test_id and ui_state.params.get_bool(P_ARMED)


def _active(spec: TestSpec) -> dict:
  """Live daemon status if this is the selected test, else {}."""
  st = poller.state
  return st if poller.live(time.monotonic()) and st.get("selected") == spec.id else {}


class InfoCard(GreyBigButton):
  """Non-interactive text card; smaller type so a few lines of guidance fit the 240 px screen."""

  def __init__(self, title: str, text: str = "", font_size: int = 28):
    super().__init__(title, text)
    self._label.set_font_size(32)
    self._sub_label.set_font_size(font_size)

  def update_text(self, title: str, text: str) -> None:
    if title != self.text:
      self.set_text(title)
      self._label.set_font_size(32)
    if text != self.value:
      self.set_value(text)


def condition_lines(conditions: list, max_lines: int = 5) -> str:
  """Failing conditions first (what the driver must fix), then met ones while they fit."""
  bad = [f"× {label}" for label, ok in conditions if not ok]
  good = [f"✓ {label}" for label, ok in conditions if ok]
  if not bad:
    return f"✓ all {len(good)} met"
  room = max_lines - len(bad)
  return "\n".join(bad + (good if len(good) <= room else good[:max(room - 1, 0)] + [f"✓ +{len(good) - max(room - 1, 0)} more"]))


def condition_text(profile, name: str) -> str:
  return condition_label(name, Context(profile=profile))


class TestDetailPage(NavScroller):
  def __init__(self, spec: TestSpec, profile):
    super().__init__()
    self.spec, self.profile = spec, profile
    phase = profile.registry.phases[spec.phase]
    self._arm = BigToggle("armed", initial_state=self._is_armed(), toggle_callback=self._on_arm)
    self._status = InfoCard(f"{spec.id} {spec.label}", "")
    self._result = InfoCard("result", "", font_size=24)
    self._conds = InfoCard("", "", font_size=24)  # untitled: room for 5 condition lines
    self._button = InfoCard("start/stop", "")
    self._prereq = InfoCard("prereqs", "")
    how = [InfoCard(f"how {i + 1}/{len(spec.how)}", text) for i, text in enumerate(spec.how)]
    toggles = ", ".join(toggle_label(k, v) for k, v in spec.toggles.items()) or "none (telemetry)"
    first = " • FIRST-RUN characterization" if spec.first_run else ""
    self._scroller.add_widgets([
      self._arm, self._status, self._conds, self._result, self._button, self._prereq, *how,
      InfoCard(f"phase {phase.num}: {phase.title.lower()}", phase.setup + first),
      InfoCard("toggles this test expects", toggles + " • set them in its session page (or settings)"),
      InfoCard("pass if", spec.passes or "characterization only: CAPTURED, no pass/fail"),
    ])

  def _is_armed(self) -> bool:
    return is_armed(self.spec.id)

  def _on_arm(self, state: bool) -> None:
    set_armed(self.spec.id, state)

  def show_event(self):
    super().show_event()
    self._arm.set_checked(self._is_armed())

  def _update_state(self):
    super()._update_state()
    _refresh()
    spec, st = self.spec, _active(self.spec)
    res = poller.results.get(spec.id, {})
    if st.get("armed"):
      state = st.get("state", "idle")
      title = f"{spec.id}: {STATE_TEXT.get(state, state)}"
      text = st.get("progress", "")
      result = f"{st['status']}: {st.get('detail', '')}" if st.get("status") else "nothing scored yet"
      conds = condition_lines(st.get("conditions", []))
      btn = st.get("button", {})
      button = f"{btn.get('name', '?')}, {btn.get('mode', '?')} press • seen {btn.get('seen', 0)}x this drive"
    else:
      title = f"{spec.id}: {result_line(res)}"
      text = res.get("progress", "") if res else "arm it, then drive"
      result = f"{res.get('status')}: {res.get('detail', '')}" if res.get("status") else "no result yet"
      needs = [toggle_label(k, v) for k, v in spec.toggles.items()] + [condition_text(self.profile, c) for c in spec.conditions]
      conds = "conditions: " + ", ".join(needs)
      button = "live once armed and onroad (shows whether the wheel button decodes)"
    self._status.update_text(title, text)
    self._result.update_text("result", result)
    self._button.update_text("start/stop button", button)
    self._conds.update_text("", conds)
    self._prereq.update_text("prereqs", prereq_hint(spec.prereqs, poller.results) if spec.prereqs else "none")


class TestButton(BigButton):
  def __init__(self, spec: TestSpec, profile):
    super().__init__(f"{spec.id} {spec.label}", "not run", font_size=40)
    self.spec = spec
    self._sub_label.set_font_size(30)
    self._page = TestDetailPage(spec, profile)
    self.set_click_callback(lambda: gui_app.push_widget(self._page))

  def _update_state(self):
    super()._update_state()
    spec, st = self.spec, _active(self.spec)
    if st.get("armed"):
      state = st.get("state", "idle")
      value = STATE_TEXT.get(state, state) + (f" {st.get('progress', '')}" if state in ("capturing", "stopped") else "")
      if state == "complete":
        value = st.get("status") or "DONE"
    else:
      value = result_line(poller.results.get(spec.id, {}))
      if spec.prereqs and value == "not run" and prereq_hint(spec.prereqs, poller.results) != "prereqs done":
        value += " • prereqs ×"
      elif spec.first_run and value == "not run":
        value += " • first run"
    if value != self.value:
      self.set_value(value)


class DevTestLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._live = InfoCard("dev test • phases", "")
    self._background = BigToggle("background telemetry", initial_state=ui_state.params.get_bool(P_BACKGROUND),
                                 toggle_callback=lambda v: write_param(ui_state.params, P_BACKGROUND, v))
    current = ui_state.params.get(P_PRESS, return_default=True)
    self._press = BigMultiToggle("start press", list(PRESS_MODES), select_callback=self._on_press)
    self._press.set_value(current if current in PRESS_MODES else PRESS_MODES[0])
    self._profile = object()
    self._build(ui_profile())

  def _build(self, profile) -> None:
    """The plan of the active vehicle profile (or the no-profile card)."""
    self._profile = profile
    self._scroller._items.clear()
    if profile is None:
      self._scroller.add_widgets([self._live, InfoCard("no test-console profile", profile_cache.reason +
                                                       ". The console stays inactive on this car (writes nothing).")])
      return
    reg = profile.registry
    items = [
      self._live,
      InfoCard("phase view", "Sets no feature toggles here (the session view can), no driving authority. Arm a test, " +
                             "then press the wheel button to start/stop. Select/arm stopped or via a passenger."),
      InfoCard("first run on the C4", reg.first_run_note),
      self._background, self._press,
    ]
    for num, phase in reg.phases.items():
      items.append(InfoCard(f"phase {num}: {phase.title.lower()}", phase.setup + (f" • gates {phase.gates}" if phase.gates else "")))
      items += [TestButton(spec, profile) for spec in reg.tests_in_phase(num)]
    self._scroller.add_widgets(items)
    assert sum(isinstance(i, TestButton) for i in items) == len(reg.tests)

  def _on_press(self, value: str) -> None:
    write_param(ui_state.params, P_PRESS, value)

  def _update_state(self):
    super()._update_state()
    _refresh()
    if ui_profile() is not self._profile:
      self._build(ui_profile())
    st = poller.state
    if poller.live(time.monotonic()):
      sel = st.get("selected") or "-"
      text = f"{sel} {STATE_TEXT.get(st.get('state', 'idle'), '')}" if st.get("armed") else "running, nothing armed"
      bg = st.get("bg", {})
      if bg:
        text += f" • background: {len(bg)} tests"
    else:
      sel = ui_state.params.get(P_SELECTED) or ""
      text = "offline (offroad)" + (f" • {sel} armed" if sel and ui_state.params.get_bool(P_ARMED) else "")
    self._live.update_text("dev test • phases", text)


def dev_test_button_icon() -> rl.Texture:
  return gui_app.texture("icons_mici/settings/developer_icon.png", 64, 60)
