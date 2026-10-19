#!/usr/bin/env python3
"""Headless renders of the dev-test session UI (Task 9) at the comma four's 536x240, with synthetic console state.

Writes PNGs: the session home grid, and session pages 1 (telemetry, no toggles), 3 (baseline, toggles not ready),
5 (clutch gate, G2 armed and capturing, onroad), 8 (no-service-brake, floor + overspeed-mode choice) and Cal (calibration
card); RENDER_ALL=1 renders every session page (Task 10 used it for the before/after pixel comparison).
Then drives the real widgets with injected touches (tap a switch, tap the non-settable op-long chip, ARM, swipe) and
checks the Params. Task 10: the UI's vehicle profile comes from ui_state.CP (a 6MT CIVIC_2022 here); a last shot
switches to a car with no profile (RAV4) -> no_profile.png, and a feature write is refused. Uses a throwaway Params
prefix and console dir, so nothing on the machine is touched. Needs a display (WSLg works).

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
  from openpilot.system.ui.lib.application import gui_app

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
             "bg": {"E0c": "g1: 3/3s"}, "button": {"name": "gapAdjustCruise", "mode": "short", "seen": 4}}
    with open(os.path.join(console_dir, "state.json"), "w") as f:
      json.dump(state, f)

  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_sessions import SessionHomeLayoutMici
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.registry import SESSIONS

  # the UI picks the vehicle profile from the car it knows (ui_state.CP <- CarParamsPersistent): a 6MT CIVIC_2022
  civic_cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
  civic_cp.transmissionType = "manual"
  ui_state.CP = civic_cp
  from opendbc.car.toyota.interface import CarInterface as ToyotaInterface
  from opendbc.car.toyota.values import CAR as TOYOTA
  rav4_cp = ToyotaInterface.get_non_essential_params(TOYOTA.TOYOTA_RAV4)   # no test-console profile

  gui_app.init_window("dev test render")
  home = SessionHomeLayoutMici()
  gui_app.push_widget(home)

  only = os.environ.get("RENDER_ALL") == "1"
  shots = [("home", None)] + [(f"session_{sid}", sid) for sid in
                              ([s.id for s in SESSIONS] if only else ("1", "3", "5", "8", "Cal"))]
  if only:
    shots += [("phases", "PHASES"), ("detail_G2", "DETAIL")]
  shots.append(("no_profile", "NOPROFILE"))
  ids = [s.id for s in SESSIONS]
  # one render loop for every shot: leaving gui_app.render() at a yield would leave that frame's scale matrix pushed
  frame, shot = 0, 0
  for _ in gui_app.render():
    write_state()
    frame += 1
    name, sid = shots[shot]
    if frame == 1:
      ui_state.started = sid == "5"  # one onroad shot: shows the "applies when disengaged" note
    if frame == 1 and sid == "NOPROFILE":
      ui_state.CP = rav4_cp
      while gui_app._nav_stack[-1] is not home:
        gui_app.pop_widget()
    elif frame == 1 and sid == "PHASES":
      gui_app.push_widget(home._phases)
    elif frame == 1 and sid == "DETAIL":
      home._pager._tap_detail("G2")
    elif frame == 1 and sid is not None:
      home._pager.open(ids.index(sid))
      if gui_app._nav_stack[-1] is not home._pager:
        gui_app.push_widget(home._pager)
    if frame == 90:
      rl.rl_draw_render_batch_active()  # flush raylib's pending draw batch into the target first
      if gui_app._render_texture is not None:
        # at this point the frame is drawn into the app's render texture (PC: scaled window), not yet to the screen
        img = rl.load_image_from_texture(gui_app._render_texture.texture)
        rl.image_flip_vertical(img)
      else:
        img = rl.load_image_from_screen()
      path = os.path.join(outdir, f"{name}.png")
      rl.export_image(img, path)
      rl.unload_image(img)
      print("wrote", path)
      frame, shot = 0, shot + 1
      if shot == len(shots):
        break

  # interaction check on the real widgets: injected touches on session 3 (baseline: gate ON is the one unmet switch)
  from openpilot.system.ui.lib.application import MouseEvent, MousePos
  pager = home._pager
  ui_state.CP = civic_cp
  gui_app.push_widget(pager)
  pager.open(ids.index("3"))

  def frames(n, events=()):
    k = 0
    for _ in gui_app.render():
      if k == 0 and events:
        with gui_app._mouse._lock:
          gui_app._mouse._events.extend(events)
      write_state()
      k += 1
      if k >= n:
        return

  def tap(rect):
    pos = MousePos(rect.x + rect.width / 2, rect.y + rect.height / 2)
    t = time.monotonic()
    return [MouseEvent(pos, 0, True, False, True, t), MouseEvent(pos, 0, False, True, False, t + 0.05)]

  def swipe(x0, x1, y=150.):
    t = time.monotonic()
    return [MouseEvent(MousePos(x0, y), 0, True, False, True, t), MouseEvent(MousePos((x0 + x1) / 2, y), 0, False, False, True, t + 0.05),
            MouseEvent(MousePos(x1, y), 0, False, True, False, t + 0.1)]

  frames(60)
  chips = pager._hits.items[:6]          # op long, non-adaptive, gate, NSB, EBP, gear-aware (drawn in this order)
  checks = []
  frames(10, tap(chips[2][0]))
  checks.append(("tap 'gate ON' -> ClutchShiftGate OFF", params.get_bool("ClutchShiftGate") is False))
  frames(10, tap(chips[0][0]))
  checks.append(("tap 'op long' -> refused (no write), hint shown", params.get_bool("AlphaLongitudinalEnabled") is True
                 and "Developer" in pager._hint[0]))
  arm = next(r for r, _ in pager._hits.items if r.width == 92)
  frames(10, tap(arm))
  checks.append(("tap ARM -> D1 selected + armed", params.get("TestConsoleSelected") == "D1" and params.get_bool("TestConsoleArmed")))
  frames(10, swipe(400, 150))
  checks.append(("swipe left -> session 4", SESSIONS[pager.idx].id == "4"))
  # a car with no test-console profile: the UI can't set a feature toggle
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import ParamNotAllowed, write_param
  ui_state.CP = rav4_cp
  try:
    write_param(params, "ClutchShiftGate", False)
    refused = False
  except ParamNotAllowed:
    refused = True
  checks.append(("no profile (RAV4): feature toggle write refused", refused and params.get_bool("ClutchShiftGate") is False))
  for label, ok in checks:
    print(("PASS " if ok else "FAIL ") + label)
  gui_app.close()
