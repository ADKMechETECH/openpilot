#!/usr/bin/env python3
"""Bench test for the low-speed-band state machine (directive s8, stage 1).

PURE / NO MOTION / NO CAR: drives lowspeed.LowSpeedController with synthetic
0x158 speed + clutch sequences and asserts ARM -> START -> bounded ramp ->
ramp-to-0 and every abort path. Run on the PC or on the device bench:
    python3 eps_bench/test_lowspeed.py
Exit 0 = all pass.

Also unit-tests common.parse_clutch bit extraction against the fork DBC layout.
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from eps_bench import common  # noqa: E402
from eps_bench.lowspeed import (  # noqa: E402
  LowSpeedConfig, LowSpeedController, IDLE, ARMED, RUNNING, STOPPING, DONE,
)

HZ = 100.0
DT = 1.0 / HZ
FAILS: list[str] = []


def check(cond: bool, msg: str) -> None:
  if not cond:
    FAILS.append(msg)
    print(f"  FAIL: {msg}")


def cfg(**kw) -> LowSpeedConfig:
  base = dict(v_min=12.0, v_max=20.0, v_start_margin=1.0, v_drop=2.0, t_max=1.5,
              target=300.0, max_cmd=600.0, max_rate=400.0, ramp_s=1.0, rate_hz=HZ,
              direction=1, speed_stale_s=0.5, session_timeout_s=60.0,
              arm_requires_clutch=True)
  base.update(kw)
  return LowSpeedConfig(**base)


def run(c: LowSpeedController, frames, dt=DT):
  """frames: list of (speed, clutch_up, ui_abort). Returns list of Step, advancing
  the clock by dt each frame from the controller's t0."""
  out = []
  t = c.t0
  for speed, clutch_up, ui in frames:
    t += dt
    out.append(c.update(t, speed, clutch_up, ui))
  return out


# -- a helper: hold a value for N frames --
def hold(val, n):
  return [val] * n


def test_clutch_parse():
  print("test_clutch_parse")
  # byte 6 bit 2 = UP (DBC bit 50), byte 6 bit 3 = FLOOR (bit 51)
  up = bytes([0, 0, 0, 0, 0, 0, 0b00000100, 0])
  floor = bytes([0, 0, 0, 0, 0, 0, 0b00001000, 0])
  neither = bytes([0] * 8)
  both = bytes([0, 0, 0, 0, 0, 0, 0b00001100, 0])
  check(common.parse_clutch(0x130, 0, up) == (True, False), "UP bit decodes")
  check(common.parse_clutch(0x130, 0, floor) == (False, True), "FLOOR bit decodes")
  check(common.parse_clutch(0x130, 0, neither) == (False, False), "neither bit set")
  check(common.parse_clutch(0x130, 0, both) == (True, True), "both bits (inconsistent) returned raw")
  check(common.parse_clutch(0x130, 2, up) is None, "wrong bus ignored")
  check(common.parse_clutch(0x158, 0, up) is None, "wrong addr ignored")
  check(common.parse_clutch(0x130, 0, b"\x00\x00") is None, "short frame ignored")


def test_happy_path():
  print("test_happy_path: ARM -> START -> bounded ramp -> t_max -> ramp-to-0 -> DONE")
  c = LowSpeedController(cfg(), t0=0.0)
  frames = []
  # 10 frames idle, clutch pressed (down)
  frames += [(0.0, False, False)] * 10
  # clutch released rising edge -> ARM; speed still 0 (not in band)
  frames += [(0.0, True, False)] * 10
  # speed climbs into band -> START at 13 (v_min+margin)
  frames += [(13.0, True, False)] * 300   # 3 s in band -> run should last t_max=1.5s then stop
  steps = run(c, frames)

  states = [s.state for s in steps]
  check(ARMED in states, "reached ARMED")
  check(RUNNING in states, "reached RUNNING")
  check(STOPPING in states, "reached STOPPING")
  check(states[-1] == DONE, f"ended DONE (got {states[-1]})")
  # ARM edge exactly once, START edge exactly once
  check(sum(s.armed_edge for s in steps) == 1, "exactly one ARM edge")
  check(sum(s.started_edge for s in steps) == 1, "exactly one START edge")
  # peak command reaches TARGET (300) and never exceeds MAX_CMD (600)
  peak = max(abs(s.cmd) for s in steps)
  check(290 <= peak <= 300.5, f"peak cmd ~= TARGET 300 (got {peak:.1f})")
  check(all(abs(s.cmd) <= 600.0 + 1e-6 for s in steps), "never exceeds MAX_CMD")
  # never a step: successive cmd delta bounded by the stop rate (4.0/frame)
  max_jump = max(abs(steps[i].cmd - steps[i-1].cmd) for i in range(1, len(steps)))
  check(max_jump <= 4.0 + 1e-6, f"no step; max per-frame delta {max_jump:.2f} <= 4.0")
  # final command zero and request released
  check(steps[-1].cmd == 0.0 and steps[-1].request is False, "final cmd 0, request 0")
  # run duration ~ t_max: frames in RUNNING ~ 150
  nrun = sum(1 for s in steps if s.state == RUNNING)
  check(145 <= nrun <= 156, f"RUNNING ~ t_max*hz=150 frames (got {nrun})")
  check(steps[-1].reason == "t_max", f"stop reason t_max (got {steps[-1].reason})")


def test_no_start_on_clutch_alone():
  print("test_no_start_on_clutch_alone: clutch released but speed never in band -> never RUNNING")
  c = LowSpeedController(cfg(), t0=0.0)
  frames = [(0.0, False, False)] * 5 + [(5.0, True, False)] * 200  # armed, creeping at 5 km/h
  steps = run(c, frames)
  check(ARMED in [s.state for s in steps], "armed")
  check(RUNNING not in [s.state for s in steps], "never started below band")
  check(all(s.cmd == 0.0 for s in steps), "no command issued")


def test_disarm_on_clutch_in():
  print("test_disarm_on_clutch_in: clutch pressed again before start -> back to IDLE")
  c = LowSpeedController(cfg(), t0=0.0)
  frames = [(0.0, False, False)] * 3 + [(0.0, True, False)] * 3 + [(0.0, False, False)] * 3
  steps = run(c, frames)
  check(steps[3].state == ARMED or any(s.state == ARMED for s in steps[:6]), "armed on release")
  check(steps[-1].state == IDLE, f"disarmed back to IDLE (got {steps[-1].state})")


def test_abort_speed_above_band():
  print("test_abort_speed_above_band: speed > V_MAX mid-run -> STOPPING speed_above_band")
  c = LowSpeedController(cfg(), t0=0.0)
  frames = [(0.0, False, False)] * 3 + [(0.0, True, False)] * 3
  frames += [(13.0, True, False)] * 20     # start + run briefly
  frames += [(25.0, True, False)] * 200    # overspeed -> abort
  steps = run(c, frames)
  check(any(s.reason == "speed_above_band" for s in steps), "aborted on overspeed")
  check(steps[-1].state == DONE and steps[-1].cmd == 0.0, "ramped to 0 and DONE")


def test_abort_speed_below_band():
  print("test_abort_speed_below_band: speed < V_MIN - V_DROP mid-run -> STOPPING speed_below_band")
  c = LowSpeedController(cfg(), t0=0.0)
  frames = [(0.0, True, False)] * 3 + [(13.0, True, False)] * 20 + [(9.0, True, False)] * 200
  steps = run(c, frames)
  check(any(s.reason == "speed_below_band" for s in steps), "aborted on underspeed (9 < 12-2)")
  check(steps[-1].state == DONE and steps[-1].cmd == 0.0, "ramped to 0 and DONE")


def test_abort_clutch_in_during_run():
  print("test_abort_clutch_in_during_run: clutch pressed while RUNNING -> STOPPING clutch_in")
  c = LowSpeedController(cfg(), t0=0.0)
  frames = [(0.0, True, False)] * 3 + [(13.0, True, False)] * 20 + [(13.0, False, False)] * 200
  steps = run(c, frames)
  check(any(s.reason == "clutch_in" for s in steps), "aborted on clutch-in")
  check(steps[-1].state == DONE and steps[-1].cmd == 0.0, "ramped to 0 and DONE")


def test_abort_stale_speed_during_run():
  print("test_abort_stale_speed_during_run: speed None mid-run -> STOPPING speed_unknown_or_stale")
  c = LowSpeedController(cfg(), t0=0.0)
  frames = [(0.0, True, False)] * 3 + [(13.0, True, False)] * 20 + [(None, True, False)] * 200
  steps = run(c, frames)
  check(any(s.reason == "speed_unknown_or_stale" for s in steps), "aborted on stale speed")
  check(steps[-1].state == DONE and steps[-1].cmd == 0.0, "ramped to 0 and DONE")


def test_ui_abort():
  print("test_ui_abort: STOP during run -> STOPPING ui_abort")
  c = LowSpeedController(cfg(), t0=0.0)
  frames = [(0.0, True, False)] * 3 + [(13.0, True, False)] * 20 + [(13.0, True, True)] * 200
  steps = run(c, frames)
  check(any(s.reason == "ui_abort" for s in steps), "ui abort honored")
  check(steps[-1].state == DONE and steps[-1].cmd == 0.0, "ramped to 0 and DONE")


def test_session_timeout():
  print("test_session_timeout: watchdog -> STOPPING session_timeout")
  c = LowSpeedController(cfg(session_timeout_s=0.5), t0=0.0)
  # armed + running, then run past 0.5 s session cap (stops before t_max=1.5)
  frames = [(0.0, True, False)] * 3 + [(13.0, True, False)] * 300
  steps = run(c, frames)
  check(any(s.reason == "session_timeout" for s in steps), "session timeout fired")
  check(steps[-1].state == DONE, "reached DONE")


def test_ramp_out_no_step_from_target():
  print("test_ramp_out_no_step: abort at full TARGET ramps down smoothly, no step")
  c = LowSpeedController(cfg(t_max=0.5), t0=0.0)
  frames = [(0.0, True, False)] * 3 + [(13.0, True, False)] * 300
  steps = run(c, frames)
  # find the STOPPING phase and check monotone bleed to 0, bounded step
  stopping = [s.cmd for s in steps if s.state in (STOPPING, DONE)]
  deltas = [abs(stopping[i] - stopping[i-1]) for i in range(1, len(stopping))]
  check(all(d <= 4.0 + 1e-6 for d in deltas), "ramp-out every step <= 4.0 (no cut)")
  check(stopping[-1] == 0.0, "ends at 0")


def test_negative_direction():
  print("test_negative_direction: DIRECTION=-1 commands negative torque")
  c = LowSpeedController(cfg(direction=-1), t0=0.0)
  frames = [(0.0, True, False)] * 3 + [(13.0, True, False)] * 160
  steps = run(c, frames)
  trough = min(s.cmd for s in steps)
  check(-300.5 <= trough <= -290, f"peak negative ~= -TARGET (got {trough:.1f})")
  check(all(s.cmd <= 1e-6 for s in steps), "never commands positive in -1 direction")


def test_arm_requires_clutch_false():
  print("test_arm_requires_clutch_false: auto-arm (bench/debug), starts on speed alone")
  c = LowSpeedController(cfg(arm_requires_clutch=False), t0=0.0)
  frames = [(13.0, None, False)] * 160   # no clutch signal at all
  steps = run(c, frames)
  check(RUNNING in [s.state for s in steps], "auto-armed + started without clutch")


def main():
  tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
  for t in tests:
    t()
  print()
  if FAILS:
    print(f"RESULT: {len(FAILS)} FAILURE(S)")
    for f in FAILS:
      print(f"  - {f}")
    sys.exit(1)
  print(f"RESULT: all {len(tests)} tests passed")


if __name__ == "__main__":
  main()
