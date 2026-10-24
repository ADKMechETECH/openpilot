#!/usr/bin/env python3
"""EPS bench - Deliverable 1: rate-limited parked steering injector.

Publishes STEERING_CONTROL (0x0E4) to the `sendcan` cereal service at ~100 Hz as
the SOLE steer source (controlsd is not running on the bench build, so there is
no command fight). pandad TXes it to the panda; with the panda in SAFETY_ALLOUTPUT
(relay OPEN) the frame reaches the EPS on bus 0 and the camera's bus-2 0x0E4 is
isolated.

Command profile: 0 -> +TARGET -> dwell -> 0 -> -TARGET -> dwell -> 0, x N_CYCLES.
The opendbc Honda DBC + CANPacker are used ONLY to format the frame (bit layout,
rolling COUNTER, CHECKSUM) - not for any EPS behavior.

SAFETY (all mandatory, see config.json):
  * HARD speed interlock - commands only when 0x158 speed <= SPEED_ABORT_KPH and
    fresh; exceeding it (or stale/unknown speed) LATCHES an abort and ramps to 0.
  * Rate limit (anti-slam) - per-frame delta capped; never a step.
  * Hard magnitude ceiling MAX_CMD - never exceeded, independent of TARGET.
  * Global session timeout - self-stops.
  * Instant abort (UI / SIGTERM / any exception) - ramps to 0, sets
    STEER_TORQUE_REQUEST=0, stops publishing. Never an instant cut.

Run:   python3 eps_bench/steer_injector.py
Stop:  Ctrl-C / SIGTERM, or `touch <LOG_DIR>/steer_injector.abort`, or UI STOP.
"""
from __future__ import annotations

import os
import signal
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # repo root

from opendbc.can import CANPacker  # noqa: E402
from openpilot.cereal import messaging  # noqa: E402
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp  # noqa: E402
from eps_bench import common  # noqa: E402

NAME = "steer_injector"


def build_segments(cfg: dict) -> list[tuple[float, float, float]]:
  """(duration_s, from_cmd, to_cmd) piecewise-linear setpoint segments."""
  T = float(cfg["TARGET"])
  ramp = float(cfg["RAMP_S"])
  dwell = float(cfg["DWELL_S"])
  pause = float(cfg["PAUSE_S"])
  segs: list[tuple[float, float, float]] = []
  for _ in range(int(cfg["N_CYCLES"])):
    segs += [(ramp, 0.0, T), (dwell, T, T), (ramp, T, 0.0), (pause, 0.0, 0.0)]
    segs += [(ramp, 0.0, -T), (dwell, -T, -T), (ramp, -T, 0.0), (pause, 0.0, 0.0)]
  return segs


def setpoint_at(segs: list[tuple[float, float, float]], t: float) -> tuple[float, bool]:
  """Desired setpoint at elapsed time t. Returns (setpoint, profile_done)."""
  acc = 0.0
  for dur, a, b in segs:
    if t < acc + dur:
      frac = (t - acc) / dur if dur > 0 else 1.0
      return a + (b - a) * frac, False
    acc += dur
  return 0.0, True


def main() -> None:
  cfg = common.load_config()
  common.clear_abort(cfg, NAME)
  d = common.log_dir(cfg)

  bus = int(cfg["BUS"])
  rate = float(cfg["RATE_HZ"])
  dt = 1.0 / rate
  max_step = float(cfg["MAX_RATE"]) / rate         # anti-slam, per frame
  max_cmd = abs(float(cfg["MAX_CMD"]))             # hard ceiling
  speed_abort = float(cfg["SPEED_ABORT_KPH"])
  speed_stale = float(cfg["SPEED_STALE_S"])
  timeout = float(cfg["SESSION_TIMEOUT_S"])
  down_to_zero = bool(cfg.get("SET_STEER_DOWN_TO_ZERO", False))

  packer = CANPacker(cfg["DBC_NAME"])
  pm = messaging.PubMaster(["sendcan"])
  can_sock = messaging.sub_sock("can", timeout=5)

  segs = build_segments(cfg)

  # abort latch
  abort = {"flag": False, "reason": ""}

  def latch_abort(reason: str):
    if not abort["flag"]:
      abort["flag"] = True
      abort["reason"] = reason

  def _sig(*_a):
    latch_abort("signal")

  signal.signal(signal.SIGINT, _sig)
  signal.signal(signal.SIGTERM, _sig)
  if hasattr(signal, "SIGHUP"):
    signal.signal(signal.SIGHUP, _sig)  # SSH drop -> graceful ramp-to-0, not an instant cut

  cmd = 0.0                 # actual commanded STEER_TORQUE (rate-limited)
  last_speed = None
  last_speed_t = 0.0

  stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
  cmdlog_path = os.path.join(d, f"cmdlog_{stamp}.csv")
  cmdlog = open(cmdlog_path, "w", buffering=1)
  cmdlog.write("monotonic_ts,phase,setpoint,cmd,request,speed_kph,note\n")

  def send(cmd_val: float, request: bool, phase: str, setpoint: float, note: str = ""):
    values = {"STEER_TORQUE": int(round(cmd_val)), "STEER_TORQUE_REQUEST": 1 if request else 0}
    if down_to_zero:
      values["STEER_DOWN_TO_ZERO"] = 1 if request else 0
    addr, data, _ = packer.make_can_msg(cfg["STEER_MSG"], bus, values)
    pm.send("sendcan", can_list_to_can_capnp([(addr, data, bus)], msgtype="sendcan"))
    sp = f"{last_speed:.2f}" if last_speed is not None else "nan"
    cmdlog.write(f"{time.monotonic():.6f},{phase},{setpoint:.1f},{cmd_val:.1f},"
                 f"{1 if request else 0},{sp},{note}\n")

  t_start = time.monotonic()
  common.write_status(cfg, NAME, {"state": "RUNNING", "cmd": 0, "cycle": 0,
                                  "file": cmdlog_path, "reason": ""})
  phase = "RUNNING"

  try:
    next_t = time.monotonic()
    while True:
      now = time.monotonic()
      elapsed = now - t_start

      # --- refresh speed from the can stream (non-blocking drain) ---
      for nanos, can_list in can_capnp_to_list(messaging.drain_sock_raw(can_sock), "can"):
        for addr, data, src in can_list:
          sp = common.parse_speed_kph(addr, src, bytes(data))
          if sp is not None:
            last_speed = sp
            last_speed_t = now

      # --- HARD interlocks ---
      if not abort["flag"]:
        if elapsed > timeout:
          latch_abort("session_timeout")
        elif common.abort_requested(cfg, NAME):
          latch_abort("ui_abort")
        elif last_speed is None or (now - last_speed_t) > speed_stale:
          latch_abort("speed_unknown_or_stale")
        elif last_speed > speed_abort:
          latch_abort("speed_exceeded")

      # --- desired setpoint ---
      if abort["flag"]:
        setpoint, done = 0.0, False
        phase = f"ABORT:{abort['reason']}"
      else:
        setpoint, done = setpoint_at(segs, elapsed)
        phase = "RUNNING"
        if done:
          phase = "FINISHING"

      # --- rate limit toward setpoint (never a step) ---
      delta = setpoint - cmd
      if delta > max_step:
        delta = max_step
      elif delta < -max_step:
        delta = -max_step
      cmd += delta
      # --- hard ceiling ---
      if cmd > max_cmd:
        cmd = max_cmd
      elif cmd < -max_cmd:
        cmd = -max_cmd

      finished = (abort["flag"] or done) and abs(cmd) < 1.0
      request = not finished  # keep REQUEST=1 until cmd has reached 0
      send(cmd, request, phase, setpoint, abort["reason"])

      common.write_status(cfg, NAME, {
        "state": phase, "cmd": round(cmd, 1), "setpoint": round(setpoint, 1),
        "speed_kph": round(last_speed, 2) if last_speed is not None else None,
        "elapsed": round(elapsed, 1), "file": cmdlog_path, "reason": abort["reason"],
      })

      if finished:
        break

      next_t += dt
      sleep = next_t - time.monotonic()
      if sleep > 0:
        time.sleep(sleep)
      else:
        next_t = time.monotonic()  # fell behind; resync

  except Exception as e:  # any failure -> ramp to 0 below
    latch_abort(f"exception:{type(e).__name__}")
    # bounded ramp-to-zero on the way out
    guard = int(rate * 5)
    while abs(cmd) >= 1.0 and guard > 0:
      cmd += max_step if cmd < 0 else -max_step
      if abs(cmd) < max_step:
        cmd = 0.0
      send(cmd, abs(cmd) >= 1.0, f"ABORT:{abort['reason']}", 0.0, "ramp_out")
      time.sleep(dt)
      guard -= 1
    raise
  finally:
    # final zero frames: STEER_TORQUE=0, STEER_TORQUE_REQUEST=0 (explicit release)
    for _ in range(5):
      send(0.0, False, "STOPPED", 0.0, "zero")
      time.sleep(dt)
    end_state = "ABORTED" if abort["flag"] and abort["reason"] not in ("signal", "ui_abort") else \
                ("ABORTED" if abort["flag"] else "DONE")
    common.write_status(cfg, NAME, {"state": end_state, "cmd": 0, "reason": abort["reason"],
                                    "file": cmdlog_path})
    common.clear_abort(cfg, NAME)
    cmdlog.close()
    print(f"[steer_injector] {end_state} (reason={abort['reason'] or 'none'}) -> {cmdlog_path}")


if __name__ == "__main__":
  main()
