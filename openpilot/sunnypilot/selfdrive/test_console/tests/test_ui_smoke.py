"""
Task 12 D5: a render smoke test over the whole dev-test UI - the session home, every session page, the phase view, a
test detail page and the onroad chip - with well-formed AND malformed state.json / results.json, onroad + engaged
(writes refused) and offroad. Nothing may raise, and the D1 guard must not have swallowed anything either (a guard
that hides a crash would make this test vacuous). Needs a display (WSLg / X); skipped without one.
"""
import json
import os
import tempfile
import time

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.test_console import guard

MALFORMED_RESULTS = {"S1": "garbage", "S2": [1, 2], "S3": {"status": 5, "complete": "yes", "frac": "x"}, "": None,
                     "G2": {"status": "PASS", "complete": True, "progress": "6/6"}}
MALFORMED_STATE = {"selected": 3, "armed": "yes", "state": None, "conditions": "nope", "progress": 7, "frac": "x",
                   "stamp": "now", "bg": [1], "button": "x", "live": 5}


def good_state(armed=True):
  return {"version": 1, "stamp": time.monotonic(), "selected": "G2", "armed": armed, "state": "capturing",
          "progress": "shifts 2/6", "conditions": [["manual transmission", True], ["op engaged", False]],
          "bg": {"E0c": "g1: 3/3s"}, "button": {"name": "gapAdjustCruise", "mode": "triple", "seen": 4,
                                                "note": "gap press also steps the follow distance (triple = net 0)"}}


class TestDevTestUiSmoke(OpenpilotTestCase):
  def test_every_page_and_the_chip_render(self):
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
      self.skipTest("needs a display")
    d = tempfile.mkdtemp()
    os.environ["TEST_CONSOLE_DIR"] = d
    import pyray as rl
    from opendbc.car.honda.interface import CarInterface
    from opendbc.car.honda.values import CAR
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts import dev_test_sessions as DS
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad import test_console_chip as CHIP
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.system.ui.lib.application import MouseEvent, MousePos, gui_app

    def write(results, state):
      for name, body in (("results.json", results), ("state.json", state)):
        with open(os.path.join(d, name), "w") as f:
          json.dump(body, f)

    cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
    cp.transmissionType = "manual"
    ui_state.CP = cp
    guard._logged.clear()
    gui_app.init_window("dev test smoke")
    try:
      home = DS.SessionHomeLayoutMici()
      gui_app.push_widget(home)
      pager = home._pager
      chip = CHIP.TestConsoleChip()
      sessions = pager.sessions
      drawn = []
      real_draw = CHIP.rl.draw_rectangle_rounded

      def frames(n, events=(), files=None):
        k = 0
        for _ in gui_app.render():
          if files is not None:
            write(*files())
          if k == 0 and events:
            with gui_app._mouse._lock:
              gui_app._mouse._events.extend(events)
          chip.render(rl.Rectangle(0, 0, 536, 240))
          k += 1
          if k >= n:
            return

      def tap(rect):
        pos, t = MousePos(rect.x + rect.width / 2, rect.y + rect.height / 2), time.monotonic()
        return [MouseEvent(pos, 0, True, False, True, t), MouseEvent(pos, 0, False, True, False, t + 0.05)]

      for label, files in (("malformed", lambda: (MALFORMED_RESULTS, MALFORMED_STATE)),
                           ("good", lambda: (MALFORMED_RESULTS, good_state()))):
        for onroad in (False, True):
          with self.subTest(files=label, onroad=onroad):
            ui_state.started = onroad
            frames(5, files=files)                      # the home grid
            gui_app.push_widget(pager)
            for i in range(len(sessions)):
              pager.open(i)
              frames(4, files=files)
              for rect, _ in list(pager._hits.items)[:8]:   # every switch / pill / ARM / how on the page: tap it
                frames(2, tap(rect), files=files)
                while gui_app._nav_stack[-1] is not pager:   # "how ▶" opened a detail page: render it, go back
                  frames(3, files=files)
                  gui_app.pop_widget()
            gui_app.pop_widget()
            gui_app.push_widget(home._phases)
            frames(5, files=files)
            gui_app.pop_widget()
      # D-08: the chip never draws over an alert
      write({}, good_state())
      CHIP.alert_showing = lambda: True
      CHIP.rl.draw_rectangle_rounded = lambda *a: drawn.append(a)
      chip._poller._t = -1e9
      chip.render(rl.Rectangle(0, 0, 536, 240))
      self.assertEqual(drawn, [], "chip hidden while an alert shows")
      CHIP.alert_showing = lambda: False
      chip._poller._t = -1e9
      chip.render(rl.Rectangle(0, 0, 536, 240))
      self.assertTrue(drawn, "sanity: drawn without an alert")
    finally:
      CHIP.rl.draw_rectangle_rounded = real_draw
      gui_app.close()
      del os.environ["TEST_CONSOLE_DIR"]
    self.assertEqual(guard._logged, set(), "the guard swallowed an exception somewhere in the UI")
