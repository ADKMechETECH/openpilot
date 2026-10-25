"""Shared helpers for the EPS parked-bench harness.

Config loading, log-dir paths, JSON status/abort files (the IPC between the UI
and the two worker processes), and the firmware-verified 0x158 speed parse used
by the injector's HARD speed interlock.

Nothing here touches the panda directly or carries any EPS *behavior* assumption;
the 0x158 byte layout is the firmware-team-verified one from the directive.
"""
from __future__ import annotations

import json
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "config.json")

# ---- 0x158 speed (firmware-verified layout) -------------------------------
SPEED_ADDR = 0x158
SPEED_BUS = 0

# ---- 0x130 clutch pedal (fork-discovered layout; low-speed mode trigger) ----
# GAS_PEDAL_2 (0x130 = 304), bus 0. CLUTCH_PEDAL_UP = DBC bit 50, CLUTCH_PEDAL_FLOOR
# = bit 51 (both 1-bit, so endianness is moot). Discovered on the 2024 Civic 6MT
# Bosch-radarless car and defined by name in the SEPARATE cruise fork's
# _honda_common.dbc; here we read the raw bits DBC-free (same approach as the speed
# interlock) so the bench harness stays independent of that fork. DBC bit N -> byte
# N//8, bit N%8: bit 50 -> byte 6 bit 2; bit 51 -> byte 6 bit 3.
CLUTCH_ADDR = 0x130
CLUTCH_BUS = 0
_CLUTCH_UP_BYTE, _CLUTCH_UP_BIT = 50 // 8, 50 % 8        # (6, 2)
_CLUTCH_FLOOR_BYTE, _CLUTCH_FLOOR_BIT = 51 // 8, 51 % 8  # (6, 3)


def load_config() -> dict:
  with open(CONFIG_PATH) as f:
    cfg = json.load(f)
  # strip the _README / _KEY documentation entries
  return {k: v for k, v in cfg.items() if not k.startswith("_")}


def log_dir(cfg: dict) -> str:
  d = cfg.get("LOG_DIR", "/data/eps_bench")
  os.makedirs(d, exist_ok=True)
  return d


def _f(cfg: dict, d: dict | None = None) -> str:
  return log_dir(cfg)


# ---- status / abort IPC (plain JSON files; no cereal schema needed) -------
def status_path(cfg: dict, name: str) -> str:
  return os.path.join(log_dir(cfg), f"{name}.status.json")


def abort_path(cfg: dict, name: str) -> str:
  # presence of this file == a graceful abort request for that worker
  return os.path.join(log_dir(cfg), f"{name}.abort")


def write_status(cfg: dict, name: str, status: dict) -> None:
  status = dict(status)
  status["ts"] = time.time()
  tmp = status_path(cfg, name) + ".tmp"
  with open(tmp, "w") as f:
    json.dump(status, f)
  os.replace(tmp, status_path(cfg, name))  # atomic


def read_status(cfg: dict, name: str) -> dict | None:
  try:
    with open(status_path(cfg, name)) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def request_abort(cfg: dict, name: str) -> None:
  with open(abort_path(cfg, name), "w") as f:
    f.write(str(time.time()))


def abort_requested(cfg: dict, name: str) -> bool:
  return os.path.exists(abort_path(cfg, name))


def clear_abort(cfg: dict, name: str) -> None:
  try:
    os.remove(abort_path(cfg, name))
  except OSError:
    pass


def parse_speed_kph(addr: int, src: int, data: bytes) -> float | None:
  """Firmware-verified: 0x158 on bus 0, bytes 4-5 big-endian, ~0.01 kph.

  Returns speed in kph, or None if this frame isn't the speed frame.
  Kept deliberately DBC-free so the safety interlock has no decode dependency.
  """
  if addr != SPEED_ADDR or src != SPEED_BUS:
    return None
  if len(data) < 6:
    return None
  raw = (data[4] << 8) | data[5]
  return raw * 0.01


def parse_clutch(addr: int, src: int, data: bytes) -> tuple[bool, bool] | None:
  """Raw read of 0x130 clutch bits. Returns (pedal_up, pedal_floor), or None if
  this frame isn't 0x130 on bus 0 / is too short.

  pedal_up   = CLUTCH_PEDAL_UP (bit 50): clutch fully released (armable).
  pedal_floor= CLUTCH_PEDAL_FLOOR (bit 51): clutch on the floor.
  DBC-free on purpose (see the constants above). If both bits are set the frame is
  inconsistent -> the caller should treat clutch as unknown.
  """
  if addr != CLUTCH_ADDR or src != CLUTCH_BUS:
    return None
  if len(data) <= _CLUTCH_UP_BYTE:
    return None
  up = bool((data[_CLUTCH_UP_BYTE] >> _CLUTCH_UP_BIT) & 1)
  floor = bool((data[_CLUTCH_FLOOR_BYTE] >> _CLUTCH_FLOOR_BIT) & 1)
  return up, floor
