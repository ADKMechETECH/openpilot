#!/usr/bin/env python3
"""Self-referenced process replay: capture a process's output on the CI segments at one commit,
then compare a later commit against that capture. Proves "no behavior change" without relying on
upstream sunnypilot refs (this fork has diverged from them).

  capture <proc> <outdir> [CAR,...]   -> writes <outdir>/<car>_<proc>.raw
  compare <proc> <outdir> [CAR,...]   -> exits 1 on any diff
  Set NON_ADAPTIVE=1 / CLUTCH_GATE=1 / CLUTCH_SMART=1 / NO_SERVICE_BRAKE=1 / ENGINE_BRAKE=1 / GEAR_AWARE=1 to turn the
  corresponding Params toggle on for the run, and PULSE=1 for NoServiceBrakeOverspeedMode=pulse (Task 11).
  Set SUMMARY=1 to print a per-field histogram of the diffs.

Usage (from the openpilot checkout; downloads the CI segments, needs network):
  python tools/civic6mt/selfref_replay.py capture plannerd /tmp/replay_base   # at the old commit
  python tools/civic6mt/selfref_replay.py compare plannerd /tmp/replay_base   # at the new commit
"""
import os
import sys

from openpilot.selfdrive.test.process_replay.compare_logs import compare_logs
from openpilot.selfdrive.test.process_replay.process_replay import CONFIGS, replay_process
from openpilot.selfdrive.test.process_replay.test_processes import get_log_data, segments
from openpilot.tools.lib.logreader import LogReader, save_log

mode, proc, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
cars = sys.argv[4].split(",") if len(sys.argv) > 4 else ["HONDA", "HONDA2", "TOYOTA", "HYUNDAI"]
cfg = next(c for c in CONFIGS if c.proc_name == proc)
os.makedirs(outdir, exist_ok=True)
toggles = {"NonAdaptiveLongitudinal": "NON_ADAPTIVE", "ClutchShiftGate": "CLUTCH_GATE",
           "NoServiceBrakeHold": "NO_SERVICE_BRAKE", "EngineBrakePreferred": "ENGINE_BRAKE",
           "GearAwareAccel": "GEAR_AWARE", "ClutchReengageSmart": "CLUTCH_SMART"}

failed = False
for car, segment in segments:
  if car not in cars:
    continue
  _, dat = get_log_data(segment)
  # replay_process runs each process in its own OpenpilotPrefix, so toggles must go through custom_params
  custom_params = {key: True for key, env in toggles.items() if os.environ.get(env) == "1"}
  if os.environ.get("PULSE") == "1":
    custom_params["NoServiceBrakeOverspeedMode"] = "pulse"
  msgs = replay_process(cfg, LogReader.from_bytes(dat), custom_params=custom_params or None, disable_progress=True)

  path = os.path.join(outdir, f"{car}_{proc}.raw")
  if mode == "capture":
    save_log(path, msgs, compress=False)
    print(f"captured {car}: {len(msgs)} msgs")
  else:
    with open(path, "rb") as f:
      ref = list(LogReader.from_bytes(f.read()))
    diff = compare_logs(ref, msgs, cfg.ignore, [], cfg.tolerance)
    n_out = sum(1 for m in msgs if m.which() in cfg.subs)
    print(f"{car}: {len(diff)} diffs over {n_out} output msgs")
    for d in diff[:5]:
      print("   ", d)
    if os.environ.get("SUMMARY") == "1":
      from collections import Counter
      fields = Counter(d[1] if isinstance(d[1], str) else ".".join(map(str, d[1])) for d in diff)
      print("    fields:", dict(fields))
    failed |= len(diff) > 0

sys.exit(1 if failed else 0)
