#!/usr/bin/env python3
"""EPS bench - Deliverable 3 (fallback): SSH text menu + orchestrator.

Two single-key toggles plus a prominent abort, matching the on-screen UI:
  [A] STEER RAMP TEST  -> start/stop the injector
  [B] CAN LOG          -> start/stop the logger
  [S] STOP / ABORT     -> graceful ramp-to-0 abort of the injector + stop logger
  [Q] QUIT             -> abort everything, exit

Launches the workers as cereal client subprocesses (no exclusive panda access).
Status is read from the workers' JSON status files. The abort is a graceful
ramp-to-0 (abort file), never an instant cut.

Run on the device:  python3 eps_bench/eps_menu.py
"""
from __future__ import annotations

import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

from eps_bench import common  # noqa: E402

PY = sys.executable
INJECTOR = "steer_injector"
LOGGER = "can_logger"


def _env() -> dict:
  env = dict(os.environ)
  env["PYTHONPATH"] = REPO + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
  return env


class Worker:
  def __init__(self, name: str, script: str):
    self.name = name
    self.script = os.path.join(HERE, script)
    self.proc: subprocess.Popen | None = None

  def running(self) -> bool:
    return self.proc is not None and self.proc.poll() is None

  def start(self, cfg: dict):
    if self.running():
      return
    common.clear_abort(cfg, self.name)
    self.proc = subprocess.Popen([PY, self.script], env=_env(), cwd=REPO)

  def stop(self, cfg: dict, graceful: bool = True):
    if not self.running():
      return
    if graceful:
      common.request_abort(cfg, self.name)  # injector ramps to 0; logger flushes
    try:
      self.proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
      self.proc.terminate()
      try:
        self.proc.wait(timeout=3)
      except subprocess.TimeoutExpired:
        self.proc.kill()


def fmt_status(cfg: dict, name: str) -> str:
  st = common.read_status(cfg, name)
  if not st:
    return "IDLE"
  age = time.time() - st.get("ts", 0)
  stale = " (stale)" if age > 2 else ""
  s = st.get("state", "?")
  if name == INJECTOR:
    return f"{s} cmd={st.get('cmd')} set={st.get('setpoint')} v={st.get('speed_kph')}kph {st.get('reason','')}{stale}"
  return f"{s} frames={st.get('frames')} {os.path.basename(st.get('file',''))}{stale}"


def main() -> None:
  cfg = common.load_config()
  inj = Worker(INJECTOR, "steer_injector.py")
  log = Worker(LOGGER, "can_logger.py")

  print("=" * 64)
  print(" EPS PARKED BENCH - keep hands clear of wheel spokes; kill reachable")
  print(f" inject 0x0E4 bus {cfg['BUS']} | TARGET {cfg['TARGET']} | MAX_CMD {cfg['MAX_CMD']}"
        f" | speed-abort {cfg['SPEED_ABORT_KPH']} kph")
  print("=" * 64)

  try:
    while True:
      a = "STOP " if inj.running() else "START"
      b = "STOP " if log.running() else "START"
      print()
      print(f" [A] {a} STEER RAMP TEST : {fmt_status(cfg, INJECTOR)}")
      print(f" [B] {b} CAN LOG         : {fmt_status(cfg, LOGGER)}")
      print(f" [S] STOP / ABORT (ramp to 0)   [Q] QUIT")
      try:
        c = input(" > ").strip().lower()
      except EOFError:
        c = "q"
      if c == "a":
        inj.stop(cfg) if inj.running() else inj.start(cfg)
      elif c == "b":
        log.stop(cfg) if log.running() else log.start(cfg)
      elif c == "s":
        print(" ABORT: ramping injector to 0 ...")
        inj.stop(cfg, graceful=True)
        log.stop(cfg, graceful=True)
      elif c == "q":
        break
      time.sleep(0.3)
  finally:
    inj.stop(cfg, graceful=True)
    log.stop(cfg, graceful=True)
    print("stopped.")


if __name__ == "__main__":
  main()
