"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

SHIFT_TEXT_SIZE = 32
SHIFT_TEXT_COLOR = rl.Color(255, 255, 255, 200)


def manual_hud_text(sm) -> str:
  """Task 12 A2.9: a low-priority HUD state (not an alert, never competes with FCW) while the clutch gate caps the
  throttle for a shift, or when the gate refused to turn on."""
  try:
    if sm.recv_frame['carControlSP'] < ui_state.started_frame:
      return ""
    ml = sm['carControlSP'].manualLongitudinal
  except (KeyError, AttributeError):
    return ""
  if ml.gatePaused:
    return "Shift: coasting"
  if ml.gateBlocked:
    return "Shift gate off"
  return ""


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self._manual_text = ""

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    self._manual_text = manual_hud_text(ui_state.sm) if ui_state.started else ""

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    if self._manual_text:
      font = gui_app.font(FontWeight.MEDIUM)
      w = measure_text_cached(font, self._manual_text, SHIFT_TEXT_SIZE).x
      pos = rl.Vector2(rect.x + (rect.width - w) / 2, rect.y + 20)
      rl.draw_text_ex(font, self._manual_text, pos, SHIFT_TEXT_SIZE, 0, SHIFT_TEXT_COLOR)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
