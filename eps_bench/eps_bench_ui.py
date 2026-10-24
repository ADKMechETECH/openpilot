#!/usr/bin/env python3
"""EPS bench - Deliverable 3 (preferred): on-screen two-button UI + STOP.

Minimal raylib app on the device screen:
  * STEER RAMP TEST  -> start/stop the injector (status: IDLE/RUNNING/DONE/ABORTED)
  * CAN LOG          -> start/stop the logger (status: IDLE/LOGGING/SAVED)
  * STOP / ABORT     -> graceful ramp-to-0 abort of the injector + stop logger (red)

To use the screen, the stock `ui` process must not be running (it owns the
framebuffer). Either stop it, or add "ui"/"soundd" to the EPS_BENCH disable set in
process_config.py, then launch this app. The SSH text menu (eps_menu.py) needs no
screen and is the validated fallback. On-screen layout is on-car-verify (TODO).

Run on the device (screen free):  python3 eps_bench/eps_bench_ui.py
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

import pyray as rl  # noqa: E402
from openpilot.system.ui.lib.application import gui_app, FontWeight  # noqa: E402
from openpilot.system.ui.widgets.button import Button, ButtonStyle  # noqa: E402
from eps_bench import common  # noqa: E402
from eps_bench.eps_menu import Worker, INJECTOR, LOGGER, fmt_status  # noqa: E402


def main() -> None:
  cfg = common.load_config()
  inj = Worker(INJECTOR, "steer_injector.py")
  log = Worker(LOGGER, "can_logger.py")

  def toggle_inj():
    inj.stop(cfg) if inj.running() else inj.start(cfg)

  def toggle_log():
    log.stop(cfg) if log.running() else log.start(cfg)

  def abort_all():
    inj.stop(cfg, graceful=True)
    log.stop(cfg, graceful=True)

  gui_app.init_window("EPS BENCH")
  btn_steer = Button("STEER RAMP TEST", toggle_inj, button_style=ButtonStyle.NORMAL)
  btn_log = Button("CAN LOG", toggle_log, button_style=ButtonStyle.NORMAL)
  btn_stop = Button("STOP / ABORT", abort_all, button_style=ButtonStyle.DANGER)

  try:
    for _ in gui_app.render():
      W, H = gui_app.width, gui_app.height
      m = int(W * 0.04)
      bw = W - 2 * m
      bh = int((H - 5 * m) / 3)

      btn_steer.set_text(("STOP  " if inj.running() else "START ") + "STEER RAMP TEST")
      btn_log.set_text(("STOP  " if log.running() else "START ") + "CAN LOG")

      btn_steer.render(rl.Rectangle(m, m, bw, bh))
      btn_log.render(rl.Rectangle(m, 2 * m + bh, bw, bh))
      btn_stop.render(rl.Rectangle(m, 3 * m + 2 * bh, bw, bh))

      # status lines
      font = gui_app.font(FontWeight.NORMAL)
      s_inj = fmt_status(cfg, INJECTOR)
      s_log = fmt_status(cfg, LOGGER)
      rl.draw_text_ex(font, s_inj[:60], rl.Vector2(m + 10, m + bh - 34), 22, 0,
                      rl.Color(255, 220, 120, 255))
      rl.draw_text_ex(font, s_log[:60], rl.Vector2(m + 10, 2 * m + 2 * bh - 34), 22, 0,
                      rl.Color(120, 220, 255, 255))
  finally:
    abort_all()


if __name__ == "__main__":
  main()
