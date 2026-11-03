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

# fork-only (civic-cruise): non-ACC mode status overlay (top-centre). The persistent text tells the driver leads are
# ignored; the cluster's hidden lead-car icon is the dashboard tell.
NONACC_TEXT_SIZE = 32
NONACC_WHITE = rl.Color(255, 255, 255, 255)
NONACC_ORANGE = rl.Color(255, 146, 0, 255)


def nonacc_hud(sm):
  """Returns (text, color) for the non-ACC overlay, or None when it should not show."""
  try:
    if sm.recv_frame['carControlSP'] < ui_state.started_frame:
      return None
    na = sm['carControlSP'].nonAcc
  except (KeyError, AttributeError):
    return None
  if not na.active:
    return None
  if na.pulsing:
    return "NON-ACC  •  BRAKING", NONACC_ORANGE
  return "NON-ACC", NONACC_WHITE


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self._nonacc = None

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    self._nonacc = nonacc_hud(ui_state.sm) if ui_state.started else None

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    if self._nonacc is not None:
      text, color = self._nonacc
      font = gui_app.font(FontWeight.MEDIUM)
      w = measure_text_cached(font, text, NONACC_TEXT_SIZE).x
      pos = rl.Vector2(rect.x + (rect.width - w) / 2, rect.y + 20)
      rl.draw_text_ex(font, text, pos, NONACC_TEXT_SIZE, 0, color)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
