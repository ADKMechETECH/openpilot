"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Session view (DEV-TEST BUILD ONLY) — the default "dev test" entry. The active vehicle profile's test plan re-cut into
~30-minute drive sessions (its registry's sessions; no profile for this car -> a "no test-console profile" screen):
  - home: a grid of sessions, each done ✓ / current / locked (with the gating session), plus the calibration card
    (background progress) and a link to the phase view (dev_test.py, kept as the secondary view);
  - session page (swipe left/right, ◀ ▶ edge taps, dot indicators): header + objective, the session's toggles as live
    switches with a ready ✓ / set N indicator, its tests as pills, the selected test's progress / conditions / ARM,
    and the start-button status chip (STOP • ● REC while capturing). "how ▶" opens the per-test detail page.
Switch taps write through dev_test_params.write_param (the profile's allowlist only). openpilot longitudinal itself is
shown but can't be set here (Developer panel). Features latch toggles only while longitudinal is inactive: the page says
so.
"""
import time

import pyray as rl

from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test import (DevTestLayoutMici, TestDetailPage, is_armed, poller,
                                                                    set_armed, _refresh)
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import profile_cache, ui_profile, write_param
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.test_console.sessions import (PILL_MARK, calibration_progress, chip_text, default_test,
                                                                  next_session, next_value, pill_state, ready_check,
                                                                  ready_text, session_status, tile_line, toggle_states,
                                                                  ui_writable)
from openpilot.sunnypilot.selfdrive.test_console.status_io import P_BUTTON, P_PRESS, result_line
from openpilot.system.ui.lib.application import FontWeight, MousePos, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.nav_widget import NavWidget

MARGIN = 30            # px each side: room for the ◀ ▶ edge taps
SWIPE_PX = 50
PARAMS_PERIOD = 0.5    # s between Param re-reads (file reads; the UI renders at 60 fps)

WHITE = rl.Color(255, 255, 255, 230)
GREY = rl.Color(170, 170, 170, 255)
DIM = rl.Color(110, 110, 110, 255)
GREEN = rl.Color(30, 150, 70, 255)
ORANGE = rl.Color(220, 120, 20, 255)
RED = rl.Color(200, 40, 40, 255)
BLUE = rl.Color(30, 120, 150, 255)
TILE = {"done": rl.Color(25, 70, 40, 255), "current": rl.Color(30, 55, 80, 255), "locked": rl.Color(38, 38, 38, 255)}
PILL = {"rec": RED, "armed": rl.Color(70, 90, 120, 255), "PASS": GREEN, "CAPTURED": BLUE, "FAIL": ORANGE,
        "partial": rl.Color(120, 100, 30, 255), "not run": rl.Color(55, 55, 55, 255)}
CHIP = {"idle": rl.Color(50, 50, 50, 255), "armed": rl.Color(70, 70, 70, 255), "capturing": RED,
        "stopped": rl.Color(200, 140, 20, 255), "PASS": GREEN, "CAPTURED": BLUE, "FAIL": ORANGE}


def _font(bold: bool):
  return gui_app.font(FontWeight.BOLD if bold else FontWeight.MEDIUM)


def text_w(text: str, size: int, bold: bool = False) -> float:
  return measure_text_cached(_font(bold), text, size).x


def fit(text: str, size: int, max_w: float, bold: bool = False) -> str:
  while len(text) > 4 and text_w(text, size, bold) > max_w:
    text = text[:-4] + "..."
  return text


def wrap(text: str, size: int, max_w: float, lines: int, bold: bool = True) -> list[str]:
  """Greedy word wrap into at most `lines` lines (the last one is fitted with ... by draw_text)."""
  out: list[str] = []
  for word in text.split():
    if out and len(out) <= lines and text_w(f"{out[-1]} {word}", size, bold) <= max_w:
      out[-1] = f"{out[-1]} {word}"
    elif len(out) < lines:
      out.append(word)
    else:
      out[-1] = f"{out[-1]} {word}"
  return out


def draw_text(text: str, x: float, y: float, size: int, color: rl.Color, bold: bool = False, max_w: float | None = None) -> float:
  if max_w is not None:
    text = fit(text, size, max_w, bold)
  rl.draw_text_ex(_font(bold), text, rl.Vector2(x, y), size, 0, color)
  return text_w(text, size, bold)


def draw_pill(rect: rl.Rectangle, color: rl.Color, outline=None) -> None:
  rl.draw_rectangle_rounded(rect, 0.5, 8, color)
  if outline is not None:
    rl.draw_rectangle_rounded_lines_ex(rect, 0.5, 8, 2, outline)


class _Hits:
  """Tap targets registered while drawing (render runs before this frame's mouse events are processed)."""

  def __init__(self):
    self.items: list[tuple[rl.Rectangle, object]] = []

  def clear(self) -> None:
    self.items = []

  def add(self, rect: rl.Rectangle, action) -> None:
    self.items.append((rect, action))

  def find(self, pos: MousePos):
    return next((a for r, a in self.items if rl.check_collision_point_rec(rl.Vector2(pos.x, pos.y), r)), None)


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


class SessionPager(NavWidget):
  def __init__(self):
    super().__init__()
    self.idx = 0
    self._selected: dict[str, str] = {}
    self._details: dict[tuple[str, str], TestDetailPage] = {}
    self._hits = _Hits()
    self._press: MousePos | None = None
    self._hint = ("", 0.)

  @property
  def sessions(self):
    profile = ui_profile()
    return profile.registry.sessions if profile is not None else ()

  def open(self, idx: int) -> None:
    self.idx = idx % max(len(self.sessions), 1)

  @property
  def session(self):
    return self.sessions[self.idx % len(self.sessions)]

  def selected_test(self) -> str:
    s = self.session
    sel = self._selected.get(s.id)
    if sel not in s.test_ids:
      sel = default_test(s, poller.results, poller.state.get("selected", "") if _live() else ui_state.params.get("TestConsoleSelected") or "")
    return sel

  def _flash(self, text: str) -> None:
    self._hint = (text, time.monotonic() + 2.5)

  # -- input --------------------------------------------------------------------------------------------------------
  def _handle_mouse_event(self, mouse_event) -> None:
    super()._handle_mouse_event(mouse_event)
    if mouse_event.left_pressed:
      self._press = mouse_event.pos

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    super()._handle_mouse_release(mouse_pos)
    press, self._press = self._press, None
    if press is not None and abs(mouse_pos.x - press.x) > SWIPE_PX and abs(mouse_pos.x - press.x) > abs(mouse_pos.y - press.y):
      self.idx = (self.idx + (1 if mouse_pos.x < press.x else -1)) % max(len(self.sessions), 1)
      return
    action = self._hits.find(mouse_pos)
    if action is not None:
      action()

  def _tap_toggle(self, t) -> None:
    if not t.settable:
      self._flash("op long: set it in Developer settings")
      return
    write_param(ui_state.params, t.param, next_value(t))
    params_cache.invalidate()
    if ui_state.engaged:
      self._flash("set • applies when disengaged")

  def _tap_arm(self, test_id: str) -> None:
    set_armed(test_id, not is_armed(test_id))

  def _tap_detail(self, test_id: str) -> None:
    profile = ui_profile()
    key = (profile.name, test_id)
    if key not in self._details:
      self._details[key] = TestDetailPage(profile.registry.tests_by_id[test_id], profile)
    gui_app.push_widget(self._details[key])

  # -- draw ---------------------------------------------------------------------------------------------------------
  def _update_state(self):
    super()._update_state()
    _refresh()

  def _render(self, rect: rl.Rectangle):
    self._hits.clear()
    profile = ui_profile()
    if profile is None:
      draw_no_profile(rect)
      return
    reg = profile.registry
    s = self.session
    x0, w, y0 = rect.x + MARGIN, rect.width - 2 * MARGIN, rect.y
    results, state, live = poller.results, poller.state, _live()
    states = toggle_states(s, params_cache.read, ui_writable(profile))

    # header: id • name, ready indicator
    if s.card:
      frac, cal_text = calibration_progress(reg, results, state)
      right, right_c = f"{frac:.0%}", BLUE
    elif states:
      ready, _ = ready_check(states)
      right, right_c = ready_text(states), GREEN if ready else ORANGE
    else:
      right, right_c = "no toggles", GREY
    rw = text_w(right, 24, True)
    draw_text(f"{s.id} • {s.name}", x0, y0 + 12, 28, WHITE, True, w - rw - 12)
    draw_text(right, x0 + w - rw, y0 + 14, 24, right_c, True)
    # features latch toggles only while longitudinal is inactive: say so wherever a mid-drive change is possible
    note = "  applies when disengaged" if ui_state.started and any(t.settable for t in states) else ""
    nw = text_w(note, 17) if note else 0
    draw_text(s.objective, x0, y0 + 44, 18, GREY, max_w=w - nw)
    if note:
      draw_text(note, x0 + w - nw, y0 + 45, 17, DIM)

    # switches (up to two rows)
    y, x = y0 + 68, x0
    if s.card:
      bar = rl.Rectangle(x0, y + 4, w, 20)
      draw_pill(bar, rl.Color(50, 50, 50, 255))
      draw_pill(rl.Rectangle(x0, y + 4, max(w * frac, 20), 20), BLUE)
      draw_text(f"{cal_text} • fills on any drive with background telemetry on (phase view)", x0, y + 34, 17, GREY,
                max_w=w)
    elif not states:
      draw_text("telemetry only • parked tests: engine idling", x0, y + 6, 18, GREY, max_w=w)
    for t in states:
      label = t.label
      cw = text_w(label, 19, True) + 34
      if x + cw > x0 + w and x > x0:
        x, y = x0, y + 32
      r = rl.Rectangle(x, y, cw, 28)
      fill = rl.Color(30, 70, 40, 255) if t.required and t.ok else rl.Color(90, 50, 15, 255) if t.required else rl.Color(45, 45, 45, 255)
      draw_pill(r, fill, None if t.settable else DIM)
      rl.draw_circle(int(x + 14), int(y + 14), 5, GREEN if t.ok and t.required else ORANGE if t.required else GREY)
      draw_text(label, x + 24, y + 4, 19, WHITE, True)
      self._hits.add(r, lambda t=t: self._tap_toggle(t))
      x += cw + 6

    # tests as pills
    sel = self.selected_test()
    x, y = x0, y0 + 134
    for tid in s.test_ids:
      st = pill_state(tid, results, state, live)
      label = f"{tid}{'?' if reg.tests_by_id[tid].optional else ''} {PILL_MARK[st]}".strip()
      pw = text_w(label, 20, True) + 24
      r = rl.Rectangle(x, y, pw, 30)
      draw_pill(r, PILL[st], WHITE if tid == sel else None)
      draw_text(label, x + 12, y + 4, 20, WHITE, True)
      self._hits.add(r, lambda tid=tid: self._selected.__setitem__(s.id, tid))
      x += pw + 8
    spec = reg.tests_by_id[sel]

    # selected test: progress + conditions on the left, ARM + how on the right
    y = y0 + 170
    arm_w, how_w = 92, 72
    armed = is_armed(sel)
    here = live and state.get("armed") and state.get("selected") == sel
    if here:
      progress = state.get("progress", "") or state.get("state", "")
      bad = [label for label, ok in state.get("conditions", []) if not ok]
      conds = ("× " + ", ".join(bad)) if bad else f"✓ all {len(state.get('conditions', []))} conditions met"
    else:
      progress = result_line(results.get(sel, {}))
      conds = spec.passes or "characterization: CAPTURED, no pass/fail"
    left_w = w - arm_w - how_w - 16
    draw_text(f"{spec.label} • {progress}", x0, y, 20, WHITE, True, left_w)
    draw_text(conds, x0, y + 24, 17, GREY if not here or conds.startswith("✓") else ORANGE, max_w=left_w)
    ra = rl.Rectangle(x0 + w - arm_w - how_w - 8, y - 2, arm_w, 42)
    draw_pill(ra, GREEN if armed else rl.Color(70, 70, 70, 255))
    at = "ARMED" if armed else "ARM"
    draw_text(at, ra.x + (arm_w - text_w(at, 22, True)) / 2, ra.y + 9, 22, WHITE, True)
    self._hits.add(ra, lambda: self._tap_arm(sel))
    rh = rl.Rectangle(x0 + w - how_w, y - 2, how_w, 42)
    draw_pill(rh, rl.Color(55, 55, 55, 255))
    draw_text("how ▶", rh.x + 10, rh.y + 10, 20, WHITE, True)
    self._hits.add(rh, lambda: self._tap_detail(sel))

    # start-button chip + dots
    hint, until = self._hint
    if hint and time.monotonic() < until:
      text, key = hint, "stopped"
    else:
      button = profile.buttons.label(params_cache.read(P_BUTTON))
      text, key = chip_text(state, live, sel, button, params_cache.read(P_PRESS) or "short")
    dots_w = len(reg.sessions) * 10
    cw = min(text_w(text, 18, True) + 24, w - dots_w - 12)
    rc = rl.Rectangle(x0, y0 + 214, cw, 24)
    draw_pill(rc, CHIP.get(key, CHIP["idle"]))
    draw_text(text, x0 + 12, y0 + 216, 18, WHITE, True, cw - 20)
    dx = x0 + w - dots_w + 5
    for i in range(len(reg.sessions)):
      rl.draw_circle(int(dx + i * 10), int(y0 + 226), 4 if i == self.idx else 2.5, WHITE if i == self.idx else DIM)

    # edge arrows
    for side, glyph in ((0, "◀"), (1, "▶")):
      ex = rect.x + (4 if side == 0 else rect.width - 24)
      draw_text(glyph, ex, y0 + 108, 22, GREY)
      self._hits.add(rl.Rectangle(rect.x + (0 if side == 0 else rect.width - MARGIN), y0 + 60, MARGIN, 140),
                     lambda d=(-1 if side == 0 else 1): self.open(self.idx + d))


class SessionHomeLayoutMici(NavWidget):
  COLS, ROWS = 4, 3

  def __init__(self):
    super().__init__()
    self._pager = SessionPager()
    self._phases = DevTestLayoutMici()
    self._hits = _Hits()

  def _update_state(self):
    super()._update_state()
    _refresh()

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    super()._handle_mouse_release(mouse_pos)
    action = self._hits.find(mouse_pos)
    if action is not None:
      action()

  def _open(self, idx: int) -> None:
    self._pager.open(idx)
    gui_app.push_widget(self._pager)

  def _render(self, rect: rl.Rectangle):
    self._hits.clear()
    profile = ui_profile()
    if profile is None:
      draw_no_profile(rect)
      return
    reg = profile.registry
    assert len(reg.sessions) <= self.COLS * self.ROWS
    results, state, live = poller.results, poller.state, _live()
    x0, y0 = rect.x + 8, rect.y
    nxt = next_session(reg, results)
    if live and state.get("armed"):
      status = f"{state.get('selected')} " + {"capturing": "● REC", "complete": state.get("status") or "done"}.get(
        state.get("state"), state.get("state", ""))
    else:
      status = "live" if live else "offroad"
    head = f"sessions • {status}" + (f" • next {nxt}" if nxt else "")
    link = "phases ▶"
    lw = text_w(link, 20, True)
    draw_text(head, x0 + 4, y0 + 16, 22, WHITE, True, rect.width - lw - 44)
    rl_link = rl.Rectangle(rect.x + rect.width - lw - 24, y0 + 10, lw + 16, 32)
    draw_pill(rl_link, rl.Color(55, 55, 55, 255))
    draw_text(link, rl_link.x + 8, rl_link.y + 6, 20, WHITE, True)
    self._hits.add(rl_link, lambda: gui_app.push_widget(self._phases))

    gap, top = 6, y0 + 46
    tw = (rect.width - 16 - gap * (self.COLS - 1)) / self.COLS
    th = (rect.height - 46 - 4 - gap * (self.ROWS - 1)) / self.ROWS
    calib = calibration_progress(reg, results, state)
    for i, s in enumerate(reg.sessions):
      c, r = i % self.COLS, i // self.COLS
      tile = rl.Rectangle(x0 + c * (tw + gap), top + r * (th + gap), tw, th)
      st = session_status(reg, s, results)
      key = "current" if s.card and st.state != "done" else st.state
      draw_pill(tile, TILE[key], WHITE if s.id == nxt else None)
      line = tile_line(s, st, calib if s.card else None)
      if s.card:
        draw_text(s.name, tile.x + 8, tile.y + 4, 18, WHITE, True, tw - 16)
        draw_text(line, tile.x + 8, tile.y + 24, 14, GREY, max_w=tw - 16)
        bar = rl.Rectangle(tile.x + 8, tile.y + th - 13, tw - 16, 7)
        draw_pill(bar, rl.Color(60, 60, 60, 255))
        draw_pill(rl.Rectangle(bar.x, bar.y, max(bar.width * calib[0], 7), 7), BLUE)
      else:
        idw = draw_text(s.id, tile.x + 7, tile.y + 4, 22, WHITE, True)
        name_x, name_w = tile.x + 7 + idw + 5, tw - idw - 17
        for k, part in enumerate(wrap(s.name, 15, name_w, 2)):
          draw_text(part, name_x, tile.y + 3 + 16 * k, 15, WHITE, True, name_w)
        color = GREEN if st.state == "done" else GREY if st.state == "locked" else WHITE
        draw_text(line, tile.x + 7, tile.y + th - 19, 14, color, max_w=tw - 14)
      self._hits.add(tile, lambda i=i: self._open(i))


def draw_no_profile(rect: rl.Rectangle) -> None:
  """Safe default: the car has no test-console profile — the console is inactive (the daemon subscribes to and writes
  nothing) and nothing here can be armed or set."""
  x0, w = rect.x + MARGIN, rect.width - 2 * MARGIN
  draw_text("dev test • inactive", x0, rect.y + 16, 26, WHITE, True, w)
  for k, line in enumerate(wrap(profile_cache.reason + ". Nothing is armed, captured or written on this car; add a " +
                                "vehicle profile to use the console here.", 20, w, 5, bold=False)):
    draw_text(line, x0, rect.y + 60 + 26 * k, 20, GREY, max_w=w)


def dev_test_home() -> SessionHomeLayoutMici:
  return SessionHomeLayoutMici()
