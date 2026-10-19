#!/usr/bin/env python3
"""Task 7 test console (dev-test branch): behavioral proof that it is non-actuating, plus an end-to-end run on real logs.

  replay <refdir> [CAR,...]
      For each CI segment, replays controlsd and plannerd TOGETHER WITH the real test_consoled process (multi-process
      process replay: the console receives controlsd's/plannerd's live outputs), with the console armed, a test
      selected and background capture on. controlsd/plannerd outputs are compared against a self-referenced capture
      taken at the feature branch (tools/civic6mt/selfref_replay.py capture ... at sp-honda-dev-202608) -> expect 0 diffs.
      Also fails if test_consoled dies on any frame (replay asserts the process is alive every step).
      Task 10: the CI segments are not 6MT Civics, so the console would select no profile and stay inactive; the replay
      forces the Civic profile (TEST_CONSOLE_PROFILE=civic_6mt) to keep the proof as strong as before. (The inactive,
      no-profile path can't be process-replayed: the harness steps a process by waiting for it to receive, and an
      inactive console subscribes to nothing. tests/test_profiles.py covers it.)

  offline
      Drives the daemon's own classes (SampleBuilder -> StartButton -> Console) over the HONDA CI segment's rlog with
      an injected gap-button press, re-labelled as a manual with injected RPM (as in engine_brake_replay.py): checks
      that captures/markers/results are written, the button starts the capture, and reports per-frame cost.

Run from the openpilot checkout on the civic-6mt-dev-test branch (needs network for the CI segments):
  cd ~/civic-cruise/openpilot && source .venv/bin/activate && export PYTHONPATH="$(pwd)"
  git switch sp-honda-dev-202608 && python tools/civic6mt/selfref_replay.py capture controlsd /tmp/ref \
    && python tools/civic6mt/selfref_replay.py capture plannerd /tmp/ref && git switch civic-6mt-dev-test
  python tools/civic6mt/test_console_replay.py replay /tmp/ref
  python tools/civic6mt/test_console_replay.py offline
"""
import json
import os
import sys
import tempfile
import time

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.selfdrive.test.process_replay.compare_logs import compare_logs
from openpilot.selfdrive.test.process_replay.process_replay import CONFIGS, MessageBasedRcvCallback, ProcessConfig, \
  get_car_params_callback, replay_process
from openpilot.selfdrive.test.process_replay.test_processes import get_log_data, segments
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import engine_rpm_at
from openpilot.tools.lib.logreader import LogReader

CONSOLE_PARAMS = {"TestConsoleSelected": "D1", "TestConsoleArmed": True, "TestConsoleBackground": True}
CONSOLE_CFG = ProcessConfig(
  proc_name="test_consoled",
  pubs=["carState", "carControl", "carOutput", "longitudinalPlan", "radarState", "selfdriveState", "pandaStates"],
  subs=["uiDebug"],  # publishes nothing; the replay harness just needs a socket to drain
  ignore=[],
  init_callback=get_car_params_callback,
  should_recv_callback=MessageBasedRcvCallback("carState"),
)


def replay(refdir: str, cars: list[str]) -> bool:
  os.environ["TEST_CONSOLE_PROFILE"] = "civic_6mt"
  ok = True
  for car, segment in segments:
    if car not in cars:
      continue
    _, dat = get_log_data(segment)
    for proc in ("controlsd", "plannerd"):
      cfg = next(c for c in CONFIGS if c.proc_name == proc)
      out = replay_process([cfg, CONSOLE_CFG], LogReader.from_bytes(dat), custom_params=CONSOLE_PARAMS, disable_progress=True)
      msgs = [m for m in out if m.which() in cfg.subs]
      with open(os.path.join(refdir, f"{car}_{proc}.raw"), "rb") as f:
        ref = list(LogReader.from_bytes(f.read()))
      diff = compare_logs(ref, msgs, cfg.ignore, [], cfg.tolerance)
      print(f"{car} {proc} with test_consoled running (Civic profile, armed, background on): {len(diff)} diffs over {len(msgs)} msgs")
      for d in diff[:5]:
        print("   ", d)
      ok &= len(diff) == 0 and len(msgs) > 0
  return ok


def offline() -> bool:
  tmp = tempfile.mkdtemp(prefix="test_console_offline_")
  os.environ["TEST_CONSOLE_DIR"] = tmp
  from openpilot.sunnypilot.selfdrive.test_console.button import StartButton
  from openpilot.sunnypilot.selfdrive.test_console.capture import Console, Store
  from openpilot.sunnypilot.selfdrive.test_console.conditions import Context
  from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE
  from openpilot.sunnypilot.selfdrive.test_console.signals import LogEventTap, SampleBuilder

  _, dat = get_log_data(dict(segments)["HONDA"])
  msgs = list(LogReader.from_bytes(dat))
  CP = next(m.carParams for m in msgs if m.which() == "carParams")
  schedule = [(4.5, 1), (8., 2), (12., 3), (17., 4), (23., 5)]
  ctx = Context(profile=PROFILE, manual=True, op_long=CP.openpilotLongitudinalControl,
                toggles=dict.fromkeys(("NonAdaptiveLongitudinal", "ClutchShiftGate", "NoServiceBrakeHold", "EngineBrakePreferred",
                                       "GearAwareAccel"), False))
  store = Store(tmp)
  console = Console(store, ctx)
  console.set_control("D1", True, True)
  # "short", as the TestConsoleStartPress default (without a mode the gap button resolves to its recommended triple)
  builder, log_tap, button = SampleBuilder(PROFILE), LogEventTap(), StartButton(None, "short", buttons=PROFILE.buttons)
  latest = {w: messaging.new_message(w) for w in ("carControl", "carOutput", "longitudinalPlan", "radarState", "selfdriveState")}
  t0, n, cost = None, 0, 0.
  for m in msgs:
    w = m.which()
    if w in latest:
      latest[w] = m
    if w != "carState":
      continue
    t = m.logMonoTime / 1e9
    t0 = t if t0 is None else t0
    cs = m.carState.as_builder()
    gear = next((g for v_up, g in schedule if cs.vEgo < v_up), 6)
    cs.deprecated.engineRpm = float(engine_rpm_at(gear, cs.vEgo))
    events = []
    if abs(t - t0 - 2.0) < 0.005:      # gap button press at +2.00 s
      events = [structs.CarState.ButtonEvent(pressed=True, type=structs.CarState.ButtonEvent.Type.gapAdjustCruise)]
    elif abs(t - t0 - 2.2) < 0.005:    # ...released 0.2 s later = short press
      events = [structs.CarState.ButtonEvent(pressed=False, type=structs.CarState.ButtonEvent.Type.gapAdjustCruise)]
    start = time.perf_counter()
    s = builder.build(t, cs, latest["carControl"].carControl, latest["carOutput"].carOutput, latest["longitudinalPlan"].longitudinalPlan,
                      latest["longitudinalPlan"].valid, latest["radarState"].radarState, latest["selfdriveState"].selfdriveState, 0,
                      None, log_tap, ())
    console.update(s, button.update(t, events))
    cost += time.perf_counter() - start
    n += 1
  status = console.status(s, {})
  console.shutdown()
  with open(os.path.join(tmp, "markers.jsonl")) as f:
    markers = [json.loads(line) for line in f]
  results = json.load(open(os.path.join(tmp, "results.json")))
  caps = sorted(os.listdir(os.path.join(tmp, "captures")))
  print(f"offline: {n} carState frames, {cost / n * 1e6:.0f} us/frame (armed D1 + background telemetry)")
  print(f"  D1 state {status['state']}: {status['progress']} -> {status['status']}: {status['detail']}")
  print(f"  markers: {[mk['action'] for mk in markers if not mk.get('background')][:6]} ... ({len(markers)} total)")
  print(f"  background results: { {k: v['progress'] for k, v in results.items() if v.get('background')} }")
  print(f"  capture files: {len(caps)} in {tmp}/captures")
  started = any(mk["action"] == "start" and mk["test"] == "D1" for mk in markers)
  return started and status["state"] in ("capturing", "complete") and len(caps) >= 1


if __name__ == "__main__":
  mode = sys.argv[1]
  if mode == "replay":
    good = replay(sys.argv[2], sys.argv[3].split(",") if len(sys.argv) > 3 else ["HONDA", "HONDA2", "TOYOTA", "HYUNDAI"])
  else:
    good = offline()
  print("OK" if good else "FAILED")
  sys.exit(0 if good else 1)
