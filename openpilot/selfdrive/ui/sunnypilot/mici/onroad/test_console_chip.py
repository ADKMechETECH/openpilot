"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Onroad status chip for the test console (DEV-TEST BUILD ONLY): glanceable mid-maneuver. Shown only while a test is
armed and the daemon is live, and hidden while openpilot shows an alert (Task 12 D-08: never over an alert). Guarded:
a bug here logs once and never takes the ui process down (D1). Bottom-center, clear of the wheel (left) and model-source (right) icons. Read-only.
"""
import time

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.test_console.guard import guarded
from openpilot.sunnypilot.selfdrive.test_console.status_io import StatusPoller
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

FONT_SIZE = 24
PAD_X, PAD_Y, LINE_GAP = 14, 6, 2
SIDE_CLEARANCE = 110        # px kept free at each side for the wheel / model icons
BG = {
  "armed": rl.Color(70, 70, 70, 210),
  "capturing": rl.Color(200, 40, 40, 225),
  "stopped": rl.Color(200, 140, 20, 225),
  "PASS": rl.Color(30, 150, 70, 225),
  "CAPTURED": rl.Color(30, 120, 150, 225),
  "FAIL": rl.Color(220, 90, 20, 235),
}


def chip_lines(st: dict) -> tuple[list[str], str]:
  """(lines, color key) for a live, armed status dict."""
  test, state, progress = st.get("selected", ""), st.get("state", ""), st.get("progress", "")
  failing = next((label for label, ok in st.get("conditions", []) if not ok), None)
  if state == "capturing":
    return [f"{test} ● REC", progress] + ([f"× {failing}"] if failing else []), "capturing"
  if state == "stopped":
    return [f"{test} PAUSED", progress], "stopped"
  if state == "complete":
    status = st.get("status") or "DONE"
    return [f"{test} {status}", st.get("detail", "")[:60]], status if status in BG else "CAPTURED"
  lines = [f"{test} armed: press to start"]
  if failing:
    lines.append(f"× {failing}")
  return lines, "armed"


def alert_showing() -> bool:
  try:
    return ui_state.started and ui_state.sm['selfdriveState'].alertSize != 0   # 0 = AlertSize.none
  except (KeyError, AttributeError):
    return False


class TestConsoleChip:
  def __init__(self):
    self._poller = StatusPoller()
    self._font = gui_app.font(FontWeight.BOLD)

  @guarded
  def render(self, rect: rl.Rectangle) -> None:
    self._poller.update(time.monotonic())
    st = self._poller.state
    if not st.get("armed") or not self._poller.live(time.monotonic()) or alert_showing():
      return
    lines, key = chip_lines(st)
    max_w = rect.width - 2 * SIDE_CLEARANCE - 2 * PAD_X
    fitted = []
    for line in lines:
      while len(line) > 4 and measure_text_cached(self._font, line, FONT_SIZE).x > max_w:
        line = line[:-4] + "..."
      fitted.append(line)
    w = max(measure_text_cached(self._font, line, FONT_SIZE).x for line in fitted) + 2 * PAD_X
    h = len(fitted) * (FONT_SIZE + LINE_GAP) + 2 * PAD_Y
    x = rect.x + (rect.width - w) / 2
    y = rect.y + rect.height - 10 - h
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), 0.35, 8, BG.get(key, BG["armed"]))
    for i, line in enumerate(fitted):
      rl.draw_text_ex(self._font, line, rl.Vector2(x + PAD_X, y + PAD_Y + i * (FONT_SIZE + LINE_GAP)), FONT_SIZE, 0, rl.WHITE)
