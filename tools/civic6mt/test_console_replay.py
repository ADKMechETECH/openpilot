#!/usr/bin/env python3
"""Task 7 test console (dev-test branch): behavioral proof that it is non-actuating, plus an end-to-end run on real logs.

  replay <refdir> [CAR,...]
      For each CI segment, replays controlsd and plannerd TOGETHER WITH the real test_consoled process (multi-process
      process replay: the console receives controlsd's/plannerd's live outputs), with the console armed, a test
      selected and background capture on. controlsd/plannerd outputs are compared against a self-referenced capture
      taken at the feature branch (tools/civic6mt/selfref_replay.py capture ... at sp-honda-dev-202608) -> expect 0 diffs.
      Task 13 B4: the console now also receives the features' PUBLISHED state (carControlSP / longitudinalPlanSP: the
      gate state, latched toggles), and the run fails unless the console's own state.json shows it ran clean - 0 caught
      errors, not inert, frames processed. ("Fails if test_consoled dies" alone proved nothing once the guard caught
      every exception.)

  isolation <refdir> [CAR,...]
      Task 13 B4: the containment, proven instead of assumed. (1) The same multi-process replay with a fault injected into
      every console frame (TEST_CONSOLE_FAULT_INJECT=frame): controlsd / plannerd still 0 diffs vs the reference, and the
      console's state.json says it went INERT (>= Guard.MAX_CONSECUTIVE errors). Without the guard the console process
      dies and the replay raises -> FAILED. (2) selfdrived replayed with test_consoled reported dead in managerState
      (running False, shouldBeRunning True) gives output identical to test_consoled running; a control run with a
      non-ignored process dead must differ (processNotRunning), proving the comparison can see it. Without selfdrived's
      ignored_processes entry (2) FAILS.
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
  python tools/civic6mt/test_console_replay.py isolation /tmp/ref HONDA
  python tools/civic6mt/test_console_replay.py offline
"""
import json
import os
import signal
import sys
import tempfile
import time

from opendbc.car import structs
from openpilot.cereal import log, messaging
from openpilot.selfdrive.test.process_replay.compare_logs import compare_logs
from openpilot.selfdrive.test.process_replay.migration import migrate_all
from openpilot.selfdrive.test.process_replay.process_replay import CONFIGS, MessageBasedRcvCallback, ProcessConfig, \
  _replay_multi_process, get_car_params_callback, replay_process
from openpilot.selfdrive.test.process_replay.test_processes import get_log_data, segments
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import engine_rpm_at
from openpilot.tools.lib.logreader import LogReader

CONSOLE_PARAMS = {"TestConsoleSelected": "D1", "TestConsoleArmed": True, "TestConsoleBackground": True}
CONSOLE_CFG = ProcessConfig(
  proc_name="test_consoled",
  pubs=["carState", "carControl", "carOutput", "longitudinalPlan", "radarState", "selfdriveState", "pandaStates",
        "carControlSP", "longitudinalPlanSP"],   # Task 13 B4: the published feature state the console reads (Task 12)
  subs=["uiDebug"],  # publishes nothing; the replay harness just needs a socket to drain
  ignore=[],
  init_callback=get_car_params_callback,
  should_recv_callback=MessageBasedRcvCallback("carState"),
)


def console_state(console_dir: str) -> dict:
  try:
    with open(os.path.join(console_dir, "state.json")) as f:
      return json.load(f)
  except (OSError, ValueError):
    return {}


def replay_with_console(car: str, dat: bytes, proc: str, refdir: str, fault: str = "") -> tuple[bool, dict]:
  """controlsd or plannerd + the real test_consoled, multi-process. Returns (0 diffs vs the reference, console state)."""
  console_dir = tempfile.mkdtemp(prefix=f"tc_replay_{car}_{proc}_")
  os.environ["TEST_CONSOLE_DIR"] = console_dir
  os.environ["TEST_CONSOLE_FAULT_INJECT"] = fault
  cfg = next(c for c in CONFIGS if c.proc_name == proc)
  out = replay_process([cfg, CONSOLE_CFG], LogReader.from_bytes(dat), custom_params=CONSOLE_PARAMS, disable_progress=True)
  msgs = [m for m in out if m.which() in cfg.subs]
  with open(os.path.join(refdir, f"{car}_{proc}.raw"), "rb") as f:
    ref = list(LogReader.from_bytes(f.read()))
  diff = compare_logs(ref, msgs, cfg.ignore, [], cfg.tolerance)
  st = console_state(console_dir)
  print(f"{car} {proc} with test_consoled ({'fault: ' + fault if fault else 'Civic profile, armed, background on'}): " +
        f"{len(diff)} diffs over {len(msgs)} msgs; console: state {st.get('state')!r}, errors {st.get('errors', 0)}, " +
        f"inert {st.get('inert', False)}, mono {st.get('mono')}")
  for d in diff[:5]:
    print("   ", d)
  return len(diff) == 0 and len(msgs) > 0, st


def replay(refdir: str, cars: list[str]) -> bool:
  os.environ["TEST_CONSOLE_PROFILE"] = "civic_6mt"
  ok = True
  for car, segment in segments:
    if car not in cars:
      continue
    _, dat = get_log_data(segment)
    for proc in ("controlsd", "plannerd"):
      same, st = replay_with_console(car, dat, proc, refdir)
      # Task 13 B4: the console must have RUN, clean - not merely "not crashed"
      ran = bool(st) and st.get("armed") and st.get("selected") == CONSOLE_PARAMS["TestConsoleSelected"] and \
            float(st.get("mono") or 0.) > 0.
      clean = not st.get("errors") and not st.get("inert")
      if not (ran and clean):
        print(f"    console did not run clean: ran={ran} clean={clean}")
      ok &= same and ran and clean
  return ok


def with_process(msgs, name: str, running: bool) -> list:
  """The log with one more managerState process entry (shouldBeRunning) in every managerState message."""
  out = []
  for m in msgs:
    if m.which() == "managerState":
      d = m.managerState.to_dict()
      d["processes"] = list(d.get("processes", [])) + [{"name": name, "running": running, "shouldBeRunning": True}]
      out.append(log.Event.new_message(logMonoTime=m.logMonoTime, valid=m.valid, managerState=d).as_reader())
    else:
      out.append(m)
  return out


REPLAY_TIMEOUT = 300   # s per replay; a clean one takes ~1 min


def _timeout(*_):
  raise TimeoutError(f"replay did not finish in {REPLAY_TIMEOUT} s")


def isolation(refdir: str, cars: list[str]) -> bool:
  from openpilot.sunnypilot.selfdrive.test_console.guard import Guard
  os.environ["TEST_CONSOLE_PROFILE"] = "civic_6mt"
  ok = True
  for car, segment in segments:
    if car not in cars:
      continue
    _, dat = get_log_data(segment)
    # (1) a console failing on every frame: driving processes untouched, the console reports inert
    for proc in ("controlsd", "plannerd"):
      signal.signal(signal.SIGALRM, _timeout)
      signal.alarm(REPLAY_TIMEOUT)   # a dead console (no guard) leaves the harness waiting on it forever
      try:
        same, st = replay_with_console(car, dat, proc, refdir, fault="frame")
      except Exception as e:   # the console process died (no guard): the replay can't continue / timed out
        print(f"{car} {proc}: replay raised with the console fault injected ({type(e).__name__}: {e}) - containment broken")
        return False   # stop here: the dead console's leftovers can hang the next replay
      finally:
        signal.alarm(0)
      inert = bool(st.get("inert")) and int(st.get("errors") or 0) >= Guard.MAX_CONSECUTIVE and st.get("state") == "inert"
      if not inert:
        print(f"    console did not report inert: {st}")
      ok &= same and inert
    # (2) selfdrived: a dead test_consoled is ignored; a dead non-ignored process is not (control)
    cfg = next(c for c in CONFIGS if c.proc_name == "selfdrived")
    # replay_process's own migration rewrites every managerState to "all running" - migrate first, then inject
    msgs = migrate_all(LogReader.from_bytes(dat), manager_states=True, panda_states=True,
                       camera_states=len(cfg.vision_pubs) != 0)
    n_ms = sum(1 for m in msgs if m.which() == "managerState")
    outs = {}
    for label, name, running in (("alive", "test_consoled", True), ("dead", "test_consoled", False),
                                 ("control", "tc_isolation_control", False)):
      out = _replay_multi_process([cfg], with_process(msgs, name, running), None, None, None, None, True)
      outs[label] = [m for m in out if m.which() in cfg.subs]
    same = compare_logs(outs["alive"], outs["dead"], cfg.ignore, [], cfg.tolerance)
    control = compare_logs(outs["alive"], outs["control"], cfg.ignore, [], cfg.tolerance)
    print(f"{car} selfdrived over {n_ms} managerState msgs: test_consoled dead vs alive {len(same)} diffs; " +
          f"control (a non-ignored process dead) {len(control)} diffs")
    for d in same[:5]:
      print("   ", d)
    ok &= n_ms > 0 and len(same) == 0 and len(control) > 0 and len(outs["alive"]) > 0
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
  cars = sys.argv[3].split(",") if len(sys.argv) > 3 else ["HONDA", "HONDA2", "TOYOTA", "HYUNDAI"]
  if mode == "replay":
    good = replay(sys.argv[2], cars)
  elif mode == "isolation":
    good = isolation(sys.argv[2], cars)
  else:
    good = offline()
  print("OK" if good else "FAILED")
  sys.exit(0 if good else 1)
