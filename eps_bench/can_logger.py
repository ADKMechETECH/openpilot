#!/usr/bin/env python3
"""EPS bench - Deliverable 2: raw CAN logger.

Subscribes to the `can` cereal service (pandad publishes every frame from all
buses) and writes ALL buses to a timestamped CSV. No exclusive panda access, no
contention with the injector (both are cereal clients).

CSV line:  monotonic_ts,bus,addr_hex,len,data_hex
Header carries wall-clock start + the watch-list IDs the analysis focuses on.

Run:   python3 eps_bench/can_logger.py
Stop:  Ctrl-C / SIGTERM, or `touch <LOG_DIR>/can_logger.abort`, or the UI STOP.
"""
from __future__ import annotations

import os
import signal
import sys
import time
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # repo root, for `import eps_bench.*`

from openpilot.cereal import messaging  # noqa: E402
from openpilot.selfdrive.pandad import can_capnp_to_list  # noqa: E402
from eps_bench import common  # noqa: E402

NAME = "can_logger"

# IDs the firmware-RE analysis focuses on (for the header only - we log EVERYTHING).
INJECT_ID = 0x0E4        # STEERING_CONTROL (injected)
SENSORS_ID = 0x094       # STEERING_SENSORS (hand-wheel angle/torque)
SPEED_ID = 0x158         # speed (also the injector's interlock source)
STEER_STATUS_ID = 0x18F  # STEER_STATUS - delivered-torque candidate / LKAS-active / low_speed_lockout
EPS_TX_IDS = [0x324, 0x326, 0x328, 0x32F, 0x33D, 0x374, 0x3A1, 0x516, 0x78E]  # EPS-TX status set


def main() -> None:
  cfg = common.load_config()
  common.clear_abort(cfg, NAME)
  d = common.log_dir(cfg)

  stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
  path = os.path.join(d, f"canlog_{stamp}.csv")

  stop = {"flag": False}

  def _stop(*_a):
    stop["flag"] = True

  signal.signal(signal.SIGINT, _stop)
  signal.signal(signal.SIGTERM, _stop)
  if hasattr(signal, "SIGHUP"):
    signal.signal(signal.SIGHUP, _stop)  # SSH drop -> flush + close the log cleanly

  can_sock = messaging.sub_sock("can", timeout=100)

  frames = 0
  nbytes = 0
  t0 = time.monotonic()

  with open(path, "w", buffering=1) as f:
    f.write(f"# EPS bench CAN log\n")
    f.write(f"# wall_clock_start={datetime.now(timezone.utc).isoformat()}\n")
    f.write(f"# analysis_ids: inject=0x{INJECT_ID:X} handwheel=0x{SENSORS_ID:X} "
            f"speed=0x{SPEED_ID:X} steer_status=0x{STEER_STATUS_ID:X} "
            f"eps_tx={','.join(f'0x{i:X}' for i in EPS_TX_IDS)}\n")
    f.write(f"# delivered-torque candidates: 0x{STEER_STATUS_ID:X} + the EPS-TX set - watch for "
            f"saturation vs the rising 0x{INJECT_ID:X} (firmware cap +/-0x2b4e = 34%)\n")
    f.write("# NOTE: all buses are logged in full; the IDs above are the analysis focus only\n")
    f.write("monotonic_ts,bus,addr_hex,len,data_hex\n")

    common.write_status(cfg, NAME, {"state": "LOGGING", "file": path, "frames": 0, "bytes": 0})

    last_status = 0.0
    while not stop["flag"] and not common.abort_requested(cfg, NAME):
      msgs = messaging.drain_sock_raw(can_sock, wait_for_one=True)
      if not msgs:
        continue
      for nanos, can_list in can_capnp_to_list(msgs, "can"):
        ts = nanos / 1e9
        for addr, data, src in can_list:
          data = bytes(data)
          f.write(f"{ts:.6f},{src},{addr:X},{len(data)},{data.hex()}\n")
          frames += 1
          nbytes += len(data)

      now = time.monotonic()
      if now - last_status > 0.5:
        common.write_status(cfg, NAME, {"state": "LOGGING", "file": path,
                                        "frames": frames, "bytes": nbytes,
                                        "elapsed": round(now - t0, 1)})
        last_status = now

  common.write_status(cfg, NAME, {"state": "SAVED", "file": path,
                                  "frames": frames, "bytes": nbytes,
                                  "elapsed": round(time.monotonic() - t0, 1)})
  common.clear_abort(cfg, NAME)
  print(f"[can_logger] SAVED {frames} frames / {nbytes} bytes -> {path}")


if __name__ == "__main__":
  main()
