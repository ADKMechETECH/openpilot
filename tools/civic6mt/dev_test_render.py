#!/usr/bin/env python3
"""Headless renders of the dev-test UI at the comma four's 536x240, with synthetic console state (Task 9; rebuilt for the
Task 16 stock-card layout).

Writes PNGs, one per card the driver sees (the session view is stock NavScroller pages of Big* cards now): the home
(status card + the first sessions), session 3 card by card (header, each toggle, a test, the start-button status),
session 8 (choice cards: NSB floor, overspeed mode), the Cal page, a test detail page, the phase view, the onroad chip,
and the no-profile card. RENDER_ALL=1 also writes the first view of every session page.
Then drives the real widgets with injected touches and checks the Params: tap a switch (written), tap the non-settable
op-long card (refused, reason on the card), tap a satisfied requirement (not flipped off), a swipe across a switch (no
write - stock NavScroller invalidates touches while scrolling, D-09), open a test and ARM it, cycle a choice card, and a
car with no profile (RAV4: refused). Uses a throwaway Params prefix and console dir. Needs a display (WSLg works).

  cd ~/civic-cruise/openpilot && source .venv/bin/activate && export PYTHONPATH="$(pwd)"
  python tools/civic6mt/dev_test_render.py <outdir>
"""
import json
import os
import sys
import tempfile
import time

outdir = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "dev_test_renders")
os.makedirs(outdir, exist_ok=True)
console_dir = tempfile.mkdtemp()
os.environ["TEST_CONSOLE_DIR"] = console_dir
os.environ.setdefault("SCALE", "2.0")  # force the render-texture path (and a crisper PNG to look at)

from openpilot.common.prefix import OpenpilotPrefix

with OpenpilotPrefix():
  import pyray as rl
  from openpilot.common.params import Params
  from openpilot.system.ui.lib.application import MouseEvent, MousePos, gui_app

  params = Params()
  params.put_bool("AlphaLongitudinalEnabled", True)
  params.put_bool("ClutchShiftGate", True)          # session 3 wants it OFF -> "set 1"
  params.put("TestConsoleSelected", "G2")
  params.put_bool("TestConsoleArmed", True)

  results = {t: {"status": s, "complete": True, "progress": p, "frac": 1.0} for t, s, p in (
    ("S1", "PASS", "done"), ("S7", "CAPTURED", "90/90 s"), ("S2", "PASS", "10/10 shifts"), ("S3", "PASS", "60/60 s"),
    ("S4", "PASS", "10/10 shifts"), ("S5", "PASS", "5/5 shifts"), ("D1", "PASS", "300/300 s"), ("G1", "PASS", "5/5 shifts"))}
  results["C1"] = {"status": "FAIL", "complete": True, "progress": "5/5 segments", "frac": 1.0}
  results["E0c"] = {"status": None, "complete": False, "progress": "g1: 3/3s • g2: 1/3s", "frac": 0.4}
  results["E0b"] = {"status": None, "complete": False, "progress": "", "frac": 0.15}
  with open(os.path.join(console_dir, "results.json"), "w") as f:
    json.dump(results, f)

  def write_state():
    state = {"version": 1, "stamp": time.monotonic(), "selected": "G2", "armed": True, "state": "capturing",
             "progress": "shifts 2/6 (up 1/3, down 1/3)", "conditions": [["manual transmission", True], ["op engaged", True]],
             "bg": {"E0c": "g1: 3/3s"}, "button": {"name": "gapAdjustCruise", "mode": "triple", "seen": 4}}
    with open(os.path.join(console_dir, "state.json"), "w") as f:
      json.dump(state, f)

  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_sessions import ChoiceCard, SessionHomeLayoutMici, TestCard, \
    ToggleCard
  from openpilot.selfdrive.ui.sunnypilot.mici.onroad.test_console_chip import TestConsoleChip
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.registry import SESSIONS

  civic_cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
  civic_cp.transmissionType = "manual"
  ui_state.CP = civic_cp
  from opendbc.car.toyota.interface import CarInterface as ToyotaInterface
  from opendbc.car.toyota.values import CAR as TOYOTA
  rav4_cp = ToyotaInterface.get_non_essential_params(TOYOTA.TOYOTA_RAV4)   # no test-console profile

  gui_app.init_window("dev test render")
  home = SessionHomeLayoutMici()
  gui_app.push_widget(home)
  ids = [s.id for s in SESSIONS]
  chip = TestConsoleChip()

  # ONE render generator for the whole run, advanced frame by frame: leaving gui_app.render() at a yield leaves that
  # frame's scale matrix pushed, and doing it once per card overflows raylib's matrix stack
  render = gui_app.render()

  def frames(n, events=(), each=None):
    for k in range(n):
      next(render)
      if k == 0 and events:
        with gui_app._mouse._lock:
          gui_app._mouse._events.extend(events)
      write_state()
      if each is not None:
        each()

  def snap(name, draw=None):
    """Capture the next frame to <name>.png (draw: extra drawing on that frame, e.g. the onroad chip)."""
    def grab():
      rl.rl_draw_render_batch_active()
      if gui_app._render_texture is not None:
        img = rl.load_image_from_texture(gui_app._render_texture.texture)
        rl.image_flip_vertical(img)
      else:
        img = rl.load_image_from_screen()
      path = os.path.join(outdir, f"{name}.png")
      rl.export_image(img, path)
      rl.unload_image(img)
      print("wrote", path)
    frames(1, each=lambda: (draw() if draw is not None else None, grab()))

  def show(page, item):
    """Bring a card of a NavScroller page into view (instant scroll) and let the layout settle."""
    sc = page._scroller
    sc.scroll_to(item.rect.x - sc.rect.x - 30)
    frames(40)

  def tap(widget):
    r = widget.rect
    pos, t = MousePos(r.x + r.width / 2, r.y + r.height / 2), time.monotonic()
    return [MouseEvent(pos, 0, True, False, True, t), MouseEvent(pos, 0, False, True, False, t + 0.05)]

  def swipe_across(widget, steps=8):
    """A real swipe: press on the card, drag across it over several frames, release on it (one event per frame)."""
    r = widget.rect
    y = r.y + r.height / 2
    xs = [r.x + r.width * (0.8 - 0.6 * k / steps) for k in range(steps + 1)]
    frames(1, [MouseEvent(MousePos(xs[0], y), 0, True, False, True, time.monotonic())])
    for x in xs[1:]:
      frames(1, [MouseEvent(MousePos(x, y), 0, False, False, True, time.monotonic())])
    frames(1, [MouseEvent(MousePos(xs[-1], y), 0, False, True, False, time.monotonic())])

  def pop_to(widget):
    while gui_app._nav_stack[-1] is not widget:
      gui_app.pop_widget()
    frames(20)

  # --- renders --------------------------------------------------------------------------------------------------
  frames(60)
  snap("home")
  cards = home._scroller._items
  show(home, cards[3])
  snap("home_sessions")

  page3 = home.page(ids.index("3"))
  gui_app.push_widget(page3)
  frames(60)
  snap("session_3_header")
  for i, item in enumerate(page3._scroller._items[1:], 1):
    show(page3, item)
    snap(f"session_3_card{i}_{type(item).__name__}")

  page8 = home.page(ids.index("8"))
  pop_to(home)
  gui_app.push_widget(page8)
  frames(40)
  for item in page8.toggle_cards:
    if isinstance(item, ChoiceCard):
      show(page8, item)
      snap(f"session_8_{item.param}")
  pop_to(home)
  gui_app.push_widget(home.page(ids.index("Cal")))
  frames(60)
  snap("session_Cal")
  if os.environ.get("RENDER_ALL") == "1":
    for sid in ids:
      pop_to(home)
      gui_app.push_widget(home.page(ids.index(sid)))
      frames(40)
      snap(f"session_{sid}")
  pop_to(home)
  page5 = home.page(ids.index("5"))
  gui_app.push_widget(page5)
  frames(40)
  g2 = next(c for c in page5.test_cards if c.spec.id == "G2")
  show(page5, g2)
  frames(10, tap(g2))
  frames(40)
  snap("detail_G2")
  pop_to(home)
  gui_app.push_widget(home._phases)
  frames(60)
  snap("phases")
  pop_to(home)
  ui_state.started = True   # the chip draws only onroad (and never over an alert); on top of whatever is on screen here
  chip._poller._t = -1e9
  frames(5, each=lambda: chip.render(rl.Rectangle(0, 0, 536, 240)))
  snap("onroad_chip", lambda: chip.render(rl.Rectangle(0, 0, 536, 240)))
  ui_state.started = False

  # --- interaction on the real widgets ----------------------------------------------------------------------------
  checks = []
  gui_app.push_widget(page3)
  frames(40)
  by_param = {c.param: c for c in page3.toggle_cards}
  gate = by_param["ClutchShiftGate"]
  show(page3, gate)
  swipe_across(gate)
  frames(30)
  checks.append(("swipe across the gate switch -> no write (D-09)", params.get_bool("ClutchShiftGate") is True))
  show(page3, gate)
  frames(10, tap(gate))
  frames(10)
  checks.append(("tap 'gate' -> ClutchShiftGate OFF (session 3 wants it off)", params.get_bool("ClutchShiftGate") is False))
  frames(10, tap(gate))
  frames(10)
  checks.append(("tap it again -> satisfied requirement not flipped back on", params.get_bool("ClutchShiftGate") is False
                 and "already set" in gate.value))
  op_long = next(c for p, c in by_param.items() if p == "AlphaLongitudinalEnabled")
  show(page3, op_long)
  frames(10, tap(op_long))
  frames(10)
  checks.append(("tap 'op long' -> refused, reason on the card", params.get_bool("AlphaLongitudinalEnabled") is True
                 and "Developer" in op_long.value))
  d1 = next(c for c in page3.test_cards if isinstance(c, TestCard) and c.spec.id == "D1")
  show(page3, d1)
  frames(10, tap(d1))
  frames(40)
  detail = gui_app._nav_stack[-1]
  frames(10, tap(detail._arm))
  frames(10)
  checks.append(("open D1, tap ARM -> D1 selected + armed", params.get("TestConsoleSelected") == "D1" and
                 params.get_bool("TestConsoleArmed")))
  pop_to(home)
  gui_app.push_widget(page8)
  frames(40)
  floor = next(c for c in page8.toggle_cards if isinstance(c, ChoiceCard) and c.param == "NoServiceBrakeDecelFloor")
  before = params.get("NoServiceBrakeDecelFloor", return_default=True)
  show(page8, floor)
  frames(10, tap(floor))
  frames(10)
  checks.append(("tap the NSB floor card -> next choice", params.get("NoServiceBrakeDecelFloor", return_default=True) != before))
  checks.append(("every card is at least stock size (402 x 180)", all(c.rect.width >= 402 and c.rect.height >= 180
                 for c in page3._scroller._items + page8._scroller._items)))
  checks.append(("bool toggles use the stock switch", all(isinstance(c, ToggleCard) for c in page3.toggle_cards)))
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import ParamNotAllowed, write_param
  ui_state.CP = rav4_cp
  try:
    write_param(params, "ClutchShiftGate", True)
    refused = False
  except ParamNotAllowed:
    refused = True
  checks.append(("no profile (RAV4): feature toggle write refused", refused and params.get_bool("ClutchShiftGate") is False))
  pop_to(home)
  frames(40)
  snap("no_profile")
  for label, ok in checks:
    print(("PASS " if ok else "FAIL ") + label)
  render.close()
  gui_app.close()
  sys.exit(0 if all(ok for _, ok in checks) else 1)
