"""
Task 12 D5 (rebuilt for the Task 16 stock-card layout): a render smoke test over the whole dev-test UI - the session
home, every session page card by card, every test detail page, the phase view and the onroad chip - with well-formed AND
malformed state.json / results.json, onroad + engaged (writes refused) and offroad. Nothing may raise, and the D1 guard
must not have swallowed anything either (a guard that hides a crash would make this test vacuous).
Task 16 adds the in-car readability rules (every card at least stock size, no text under 24 px, white text) and D-09 on
the real widgets (a swipe across a switch never writes). Needs a display (WSLg / X); skipped without one.
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
MIN_TEXT = 24          # px: nothing on these screens is smaller (Task 16)
STOCK_CARD = (402, 180)


def good_state(armed=True):
  return {"version": 1, "stamp": time.monotonic(), "selected": "G2", "armed": armed, "state": "capturing",
          "progress": "shifts 2/6", "conditions": [["manual transmission", True], ["op engaged", False]],
          "bg": {"E0c": "g1: 3/3s"}, "button": {"name": "gapAdjustCruise", "mode": "triple", "seen": 4,
                                                "note": "gap press also steps the follow distance (triple = net 0)"}}


class _Ui:
  """The real UI in a window: one persistent render generator (leaving gui_app.render() mid-frame overflows raylib's
  matrix stack), event injection, taps / swipes on widget rects."""

  def __init__(self, console_dir):
    import pyray as rl
    from opendbc.car.honda.interface import CarInterface
    from opendbc.car.honda.values import CAR
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts import dev_test_sessions as DS
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad import test_console_chip as CHIP
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.system.ui.lib.application import MouseEvent, MousePos, gui_app
    self.rl, self.DS, self.CHIP, self.ui_state, self.gui_app = rl, DS, CHIP, ui_state, gui_app
    self.MouseEvent, self.MousePos, self.dir = MouseEvent, MousePos, console_dir
    from openpilot.common.params import Params
    cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
    cp.transmissionType = "manual"
    ui_state.CP = cp
    ui_state.params = Params()   # ui_state is a module singleton: rebind it to this test's Params prefix
    gui_app.init_window("dev test smoke")
    self.home = DS.SessionHomeLayoutMici()
    gui_app.push_widget(self.home)
    self.chip = CHIP.TestConsoleChip()
    self.files = lambda: ({}, good_state())
    self._render = gui_app.render()

  def write(self):
    results, state = self.files()
    for name, body in (("results.json", results), ("state.json", state)):
      with open(os.path.join(self.dir, name), "w") as f:
        json.dump(body, f)

  def frames(self, n, events=()):
    for k in range(n):
      next(self._render)
      self.write()
      if k == 0 and events:
        with self.gui_app._mouse._lock:
          self.gui_app._mouse._events.extend(events)
      self.chip.render(self.rl.Rectangle(0, 0, 536, 240))

  def show(self, page, item, settle=6):
    sc = page._scroller
    sc.scroll_to(item.rect.x - sc.rect.x - 30)
    self.frames(settle)

  def tap(self, widget):
    r, t = widget.rect, time.monotonic()
    pos = self.MousePos(r.x + r.width / 2, r.y + r.height / 2)
    self.frames(3, [self.MouseEvent(pos, 0, True, False, True, t), self.MouseEvent(pos, 0, False, True, False, t + 0.05)])

  def swipe_across(self, widget, steps=8):
    r = widget.rect
    y = r.y + r.height / 2
    xs = [r.x + r.width * (0.8 - 0.6 * k / steps) for k in range(steps + 1)]
    self.frames(1, [self.MouseEvent(self.MousePos(xs[0], y), 0, True, False, True, time.monotonic())])
    for x in xs[1:]:
      self.frames(1, [self.MouseEvent(self.MousePos(x, y), 0, False, False, True, time.monotonic())])
    self.frames(20, [self.MouseEvent(self.MousePos(xs[-1], y), 0, False, True, False, time.monotonic())])

  def back_to(self, widget):
    while self.gui_app._nav_stack[-1] is not widget:
      self.gui_app.pop_widget()
    self.frames(3)

  def close(self):
    self._render.close()
    self.gui_app.close()


class TestDevTestUiSmoke(OpenpilotTestCase):
  def setup_method(self):
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
      self.skipTest("needs a display")
    self.dir = tempfile.mkdtemp()
    os.environ["TEST_CONSOLE_DIR"] = self.dir
    guard._logged.clear()
    self.ui = _Ui(self.dir)

  def teardown_method(self):
    self.ui.close()
    os.environ.pop("TEST_CONSOLE_DIR", None)

  def test_every_page_and_the_chip_render(self):
    ui, home = self.ui, self.ui.home
    for label, files in (("malformed", lambda: (MALFORMED_RESULTS, MALFORMED_STATE)),
                         ("good", lambda: (MALFORMED_RESULTS, good_state()))):
      for onroad in (False, True):
        with self.subTest(files=label, onroad=onroad):
          ui.files, ui.ui_state.started = files, onroad
          ui.frames(5)
          for item in home._scroller._items:
            ui.show(home, item)
          for i in range(len(home.sessions)):
            page = home.page(i)
            ui.gui_app.push_widget(page)
            ui.frames(4)
            for item in list(page._scroller._items):
              ui.show(page, item)
              ui.tap(item)                       # every toggle / choice / test card on the page: tap it
              if ui.gui_app._nav_stack[-1] is not page:   # a test card opened its detail page: walk it, go back
                detail = ui.gui_app._nav_stack[-1]
                if label == "good" and not onroad:        # every detail card once is enough (it's the slow part)
                  for card in detail._scroller._items:
                    ui.show(detail, card, settle=2)
                ui.frames(3)
                ui.back_to(page)
            ui.back_to(home)
          ui.gui_app.push_widget(home._phases)
          ui.frames(5)
          ui.back_to(home)
    # D-08: the chip never draws over an alert
    CHIP, drawn = ui.CHIP, []
    real_draw = CHIP.rl.draw_rectangle_rounded
    try:
      ui.files = lambda: ({}, good_state())
      ui.write()
      CHIP.alert_showing = lambda: True
      CHIP.rl.draw_rectangle_rounded = lambda *a: drawn.append(a)
      ui.chip._poller._t = -1e9
      ui.chip.render(ui.rl.Rectangle(0, 0, 536, 240))
      self.assertEqual(drawn, [], "chip hidden while an alert shows")
      CHIP.alert_showing = lambda: False
      ui.chip._poller._t = -1e9
      ui.chip.render(ui.rl.Rectangle(0, 0, 536, 240))
      self.assertTrue(drawn, "sanity: drawn without an alert")
    finally:
      CHIP.rl.draw_rectangle_rounded = real_draw
    self.assertEqual(guard._logged, set(), "the guard swallowed an exception somewhere in the UI")

  def test_readable_at_arms_length(self):
    # Task 16: every card at least stock size (the whole card is the touch target), no text under MIN_TEXT, white text
    ui, home = self.ui, self.ui.home
    ui.frames(5)
    cards = list(home._scroller._items)
    for i in range(len(home.sessions)):
      page = home.page(i)
      ui.gui_app.push_widget(page)
      ui.frames(3)
      cards += page._scroller._items
      for tc in page.test_cards:
        tc._open()
        ui.frames(3)
        cards += ui.gui_app._nav_stack[-1]._scroller._items
        ui.back_to(page)
      ui.back_to(home)
    self.assertGreater(len(cards), 50)
    for card in cards:
      name = f"{type(card).__name__} '{card.text}'"
      self.assertGreaterEqual((card.rect.width, card.rect.height), STOCK_CARD, name)
      self.assertGreaterEqual(card._sub_label._font_size, MIN_TEXT, name)
      if card.text:
        self.assertGreaterEqual(card._label._font_size, MIN_TEXT, name)
      for label in (card._label, card._sub_label):
        c = label._text_color
        self.assertEqual((c.r, c.g, c.b, c.a), (255, 255, 255, 255), f"{name}: white text (no stock grey)")

  def test_toggle_cards_on_the_real_widgets(self):
    ui, home = self.ui, self.ui.home
    p = ui.ui_state.params   # the UI's own Params (bound when ui_state was first imported: the same store it writes)
    p.put_bool("AlphaLongitudinalEnabled", True)
    p.put_bool("ClutchShiftGate", True)
    ui.ui_state.started = False
    ids = [s.id for s in home.sessions]
    page = home.page(ids.index("3"))            # baseline: wants the gate OFF
    ui.gui_app.push_widget(page)
    ui.frames(10)
    cards = {c.param: c for c in page.toggle_cards}
    gate, op_long = cards["ClutchShiftGate"], cards["AlphaLongitudinalEnabled"]
    ui.show(page, gate, settle=40)
    ui.swipe_across(gate)
    self.assertTrue(p.get_bool("ClutchShiftGate"), "D-09: a swipe across the switch writes nothing")
    ui.show(page, gate, settle=40)   # let the swipe's scroll settle (a still-moving scroller invalidates touches)
    ui.tap(gate)
    ui.frames(5)
    self.assertFalse(p.get_bool("ClutchShiftGate"), "a tap writes what the session needs")
    self.assertFalse(gate._checked, "the switch shows the Param")
    ui.tap(gate)
    ui.frames(5)
    self.assertFalse(p.get_bool("ClutchShiftGate"), "a satisfied requirement is not flipped back")
    self.assertFalse(gate._checked, "and the switch didn't flip by itself")
    self.assertIn("already set", gate.value)
    ui.show(page, op_long, settle=40)
    ui.tap(op_long)
    ui.frames(5)
    self.assertTrue(p.get_bool("AlphaLongitudinalEnabled"), "op long can't be set here")
    self.assertTrue(op_long._checked, "the switch still shows ON")
    self.assertIn("Developer", op_long.value)
    self.assertEqual(guard._logged, set())
