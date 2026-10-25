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
from eps_bench import lowspeed  # noqa: E402

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


def run_lowspeed(cfg: dict) -> None:
  """Low-speed-band mode (closed course). The INVERSE of the parked interlock:
  commands ONLY inside [V_MIN, V_MAX], armed by the clutch-released signal and a
  speed-gated start. Pure decisions live in lowspeed.LowSpeedController; this just
  wires cereal to it and logs. See CLOSED_COURSE_TEST_DIRECTIVE_2026-10-25.md.
  """
  d = common.log_dir(cfg)
  bus = int(cfg["BUS"])
  rate = float(cfg["RATE_HZ"])
  dt = 1.0 / rate
  speed_stale = float(cfg.get("SPEED_STALE_S", 0.5))
  clutch_stale = float(cfg.get("CLUTCH_STALE_S", 0.3))
  down_to_zero = bool(cfg.get("SET_STEER_DOWN_TO_ZERO", False))

  lscfg = lowspeed.LowSpeedConfig.from_dict(cfg)
  packer = CANPacker(cfg["DBC_NAME"])
  pm = messaging.PubMaster(["sendcan"])
  can_sock = messaging.sub_sock("can", timeout=5)

  abort = {"flag": False}

  def _sig(*_a):
    abort["flag"] = True

  signal.signal(signal.SIGINT, _sig)
  signal.signal(signal.SIGTERM, _sig)
  if hasattr(signal, "SIGHUP"):
    signal.signal(signal.SIGHUP, _sig)

  last_speed = None
  last_speed_t = 0.0
  last_clutch = None          # (up, floor)
  last_clutch_t = 0.0

  stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
  cmdlog_path = os.path.join(d, f"cmdlog_lowspeed_{stamp}.csv")
  cmdlog = open(cmdlog_path, "w", buffering=1)
  cmdlog.write("monotonic_ts,state,setpoint,cmd,request,speed_kph,clutch_up,reason,marker\n")

  def drain():
    nonlocal last_speed, last_speed_t, last_clutch, last_clutch_t
    now = time.monotonic()
    for nanos, can_list in can_capnp_to_list(messaging.drain_sock_raw(can_sock), "can"):
      for addr, data, src in can_list:
        b = bytes(data)
        sp = common.parse_speed_kph(addr, src, b)
        if sp is not None:
          last_speed, last_speed_t = sp, now
          continue
        cl = common.parse_clutch(addr, src, b)
        if cl is not None:
          last_clutch, last_clutch_t = cl, now

  def fresh_speed(now):
    if last_speed is None or (now - last_speed_t) > speed_stale:
      return None
    return last_speed

  def fresh_clutch_up(now):
    # unknown if stale or both bits set (inconsistent frame)
    if last_clutch is None or (now - last_clutch_t) > clutch_stale:
      return None
    up, floor = last_clutch
    if up and floor:
      return None
    return up

  def send(step, marker=""):
    values = {"STEER_TORQUE": int(round(step.cmd)),
              "STEER_TORQUE_REQUEST": 1 if step.request else 0}
    if down_to_zero:
      values["STEER_DOWN_TO_ZERO"] = 1 if step.request else 0
    addr, data, _ = packer.make_can_msg(cfg["STEER_MSG"], bus, values)
    pm.send("sendcan", can_list_to_can_capnp([(addr, data, bus)], msgtype="sendcan"))
    now = time.monotonic()
    spk = fresh_speed(now)
    cu = fresh_clutch_up(now)
    sp_str = "" if spk is None else f"{spk:.2f}"
    cu_str = "" if cu is None else ("1" if cu else "0")
    cmdlog.write(f"{now:.6f},{step.state},{step.setpoint:.1f},{step.cmd:.1f},"
                 f"{1 if step.request else 0},{sp_str},{cu_str},{step.reason},{marker}\n")

  t0 = time.monotonic()
  ctrl = lowspeed.LowSpeedController(lscfg, t0)
  common.write_status(cfg, NAME, {"state": "IDLE", "mode": "lowspeed", "cmd": 0,
                                  "file": cmdlog_path, "reason": ""})
  print(f"[steer_injector] lowspeed mode armed-by-clutch; band "
        f"[{lscfg.v_min},{lscfg.v_max}] km/h start>={lscfg.v_min+lscfg.v_start_margin} "
        f"TARGET {lscfg.target} MAX_CMD {lscfg.max_cmd} T_MAX {lscfg.t_max}s -> {cmdlog_path}")

  try:
    next_t = time.monotonic()
    while True:
      drain()
      now = time.monotonic()
      ui = abort["flag"] or common.abort_requested(cfg, NAME)
      step = ctrl.update(now, fresh_speed(now), fresh_clutch_up(now), ui)

      marker = ""
      if step.armed_edge:
        marker = "ARM"
      elif step.started_edge:
        marker = "START"
      send(step, marker)

      common.write_status(cfg, NAME, {
        "state": step.state, "mode": "lowspeed", "cmd": round(step.cmd, 1),
        "setpoint": round(step.setpoint, 1),
        "speed_kph": round(last_speed, 2) if last_speed is not None else None,
        "clutch_up": (None if fresh_clutch_up(now) is None else bool(fresh_clutch_up(now))),
        "elapsed": round(now - t0, 1), "file": cmdlog_path, "reason": step.reason,
      })

      if step.state == lowspeed.DONE:
        break

      next_t += dt
      sleep = next_t - time.monotonic()
      if sleep > 0:
        time.sleep(sleep)
      else:
        next_t = time.monotonic()

  except Exception as e:
    # bounded ramp-to-zero then release
    reason = f"exception:{type(e).__name__}"
    guard = int(rate * 5)
    while abs(ctrl.cmd) >= 1.0 and guard > 0:
      step = ctrl.update(time.monotonic(), fresh_speed(time.monotonic()),
                         fresh_clutch_up(time.monotonic()), True)
      send(step, "exc")
      time.sleep(dt)
      guard -= 1
    common.write_status(cfg, NAME, {"state": "ABORTED", "mode": "lowspeed", "cmd": 0,
                                    "reason": reason, "file": cmdlog_path})
    raise
  finally:
    for _ in range(5):
      values = {"STEER_TORQUE": 0, "STEER_TORQUE_REQUEST": 0}
      if down_to_zero:
        values["STEER_DOWN_TO_ZERO"] = 0
      addr, data, _ = packer.make_can_msg(cfg["STEER_MSG"], bus, values)
      pm.send("sendcan", can_list_to_can_capnp([(addr, data, bus)], msgtype="sendcan"))
      cmdlog.write(f"{time.monotonic():.6f},STOPPED,0.0,0.0,0,,,{ctrl.reason},zero\n")
      time.sleep(dt)
    end = "ABORTED" if ctrl.reason and ctrl.reason not in ("ui_abort", "signal") else "DONE"
    common.write_status(cfg, NAME, {"state": end, "mode": "lowspeed", "cmd": 0,
                                    "reason": ctrl.reason, "file": cmdlog_path})
    common.clear_abort(cfg, NAME)
    cmdlog.close()
    print(f"[steer_injector] lowspeed {end} (reason={ctrl.reason or 'none'}) -> {cmdlog_path}")


def main() -> None:
  cfg = common.load_config()
  common.clear_abort(cfg, NAME)

  if str(cfg.get("MODE", "parked")).lower() == "lowspeed":
    run_lowspeed(cfg)
    return

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

  # Wait for the first 0x158 speed frame before starting the ramp clock. A freshly created
  # subscriber gets nothing on its first drain, which latched speed_unknown_or_stale at t=0.
  # Nothing is sent while waiting; if no speed arrives in 3 s the interlock below aborts as before.
  t_wait = time.monotonic()
  while last_speed is None and time.monotonic() - t_wait < 3.0:
    for nanos, can_list in can_capnp_to_list(messaging.drain_sock_raw(can_sock), "can"):
      for addr, data, src in can_list:
        sp = common.parse_speed_kph(addr, src, bytes(data))
        if sp is not None:
          last_speed = sp
          last_speed_t = time.monotonic()
    time.sleep(0.01)
  t_start = time.monotonic()

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
