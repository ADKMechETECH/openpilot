#!/usr/bin/env python3
"""
Run the real opendbc Honda CarState over a recorded route's CAN (all segments, in order, the way card feeds it) and
summarize the manual-gearbox signals it produces: engineRpm, clutchPressed (= drivetrain decoupled), the clutch pedal
switches (CarStateSP.clutchPedal), the phantom-gear guard and the 0x1DD liveness.

  tools/civic6mt/carstate_route_replay.py <route dir>          # a directory of segment dirs (<n>/ or <route>--<n>/)
  tools/civic6mt/carstate_route_replay.py <route dir> --gate   # + drive the real ClutchShiftGate (ON, engaged) from it

Proves the Task 15 carstate changes against the car's own bus (Stage 1 route 00000002--6c1a3bb7d4): rpm comes from
POWERTRAIN_DATA, the pedal switch decodes, every phantom gear report is flagged, and no in-gear driving above the
phantom guard's thresholds is read as decoupled. Read-only; the route stays where it is (no route data in the repo).
"""
import argparse
import glob
import os
import sys
from collections import Counter

import numpy as np

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.structs import CLUTCH_PEDAL_UNKNOWN, CLUTCH_PEDAL_UP, CLUTCH_PEDAL_TRAVEL, CLUTCH_PEDAL_FLOOR
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.tools.lib.logreader import LogReader

PEDAL_NAMES = {CLUTCH_PEDAL_UNKNOWN: "unknown", CLUTCH_PEDAL_UP: "up", CLUTCH_PEDAL_TRAVEL: "travel",
               CLUTCH_PEDAL_FLOOR: "floor"}


def segment_rlogs(route_dir: str) -> list[str]:
  segs = [d for d in os.listdir(route_dir) if os.path.isdir(os.path.join(route_dir, d))]
  segs.sort(key=lambda d: int(d.rsplit("--", 1)[-1]))
  out = []
  for d in segs:
    logs = sorted(glob.glob(os.path.join(route_dir, d, "rlog*")))
    if logs:
      out.append(logs[0])
  return out


def fingerprint_of(rlogs: list[str]) -> dict:
  fp = gen_empty_fingerprint()
  for m in LogReader(rlogs[0]):
    if m.which() == "can":
      for c in m.can:
        if c.src in fp and c.address < 0x800:
          fp[c.src][c.address] = len(c.dat)
  return fp


class GateParams:
  """Params stand-in for the gate: ClutchShiftGate ON (and ClutchReengageSmart if asked), everything else default."""
  def __init__(self, smart: bool):
    self.on = {"ClutchShiftGate"} | ({"ClutchReengageSmart"} if smart else set())

  def get_bool(self, key):
    return key in self.on

  def get(self, key, return_default=False):
    return None


def replay(rlogs: list[str], gate_smart: bool | None = None):
  fp = fingerprint_of(rlogs)
  CP = CarInterface.get_params(CAR.HONDA_CIVIC_2022, fp, [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.HONDA_CIVIC_2022, fp, [], False, False, False)
  CI = CarInterface(CP, CP_SP)
  assert CP.transmissionType == structs.CarParams.TransmissionType.manual, CP.transmissionType
  rows = []
  gate, engaged = None, False
  if gate_smart is not None:
    from opendbc.car.structs import car
    from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import ClutchShiftGate
    cp_capnp = car.CarParams.new_message(transmissionType="manual")
    gate = ClutchShiftGate(cp_capnp, GateParams(gate_smart))
    gate.update(structs.CarState(), False, -1, True)   # latch the toggle while "disengaged"
  for f in rlogs:
    for m in LogReader(f):
      if m.which() != "can":
        continue
      frames = [(c.address, bytes(c.dat), c.src) for c in m.can]
      CS, CS_SP = CI.update([(m.logMonoTime, frames)])
      gstate = -1
      if gate is not None:
        valid = bool(CS_SP.clutchSignalValid)
        # "engage" one frame after the clutch signal is alive: the gate latches its toggle only while disengaged, and a
        # dead signal at that moment (or while engaged) leaves it off - as on the car
        gstate = int(gate.update(CS, engaged, CS_SP.gearRaw if valid else -1, valid, CS_SP.clutchPedal if valid else 0))
        engaged = engaged or valid
      rows.append((m.logMonoTime * 1e-9, CS.vEgo, CS.deprecated.engineRpm, CS.deprecated.clutchPressed, CS_SP.clutchPedal,
                   CS_SP.gearRaw, CS_SP.clutchSignalValid, CI.CS.phantom_gear_guard.untrusted, CS.canValid, gstate,
                   CS.gasPressed))
  return np.array(rows, dtype=float)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("route_dir")
  ap.add_argument("--gate", action="store_true", help="also drive the real ClutchShiftGate (ON, engaged throughout)")
  ap.add_argument("--smart", action="store_true", help="with --gate: ClutchReengageSmart ON")
  args = ap.parse_args()
  rlogs = segment_rlogs(args.route_dir)
  if not rlogs:
    sys.exit(f"no rlogs under {args.route_dir}")
  R = replay(rlogs, args.smart if args.gate else None)
  t, v, rpm, decoupled, pedal, gear, valid, phantom, can_valid, gstate, gas = R.T
  t -= t[0]
  dt = np.median(np.diff(t))
  print(f"segments {len(rlogs)}  frames {len(t)}  duration {t[-1]:.0f} s")
  print(f"canValid false: {(can_valid == 0).mean() * 100:.2f} %   clutchSignalValid false: {(valid == 0).mean() * 100:.2f} %")
  print(f"engineRpm: min {rpm.min():.0f} max {rpm.max():.0f}  (0 for {(rpm == 0).mean() * 100:.2f} % of frames)")
  print("clutchPedal:", {PEDAL_NAMES[int(k)]: f"{n * dt:.1f} s" for k, n in sorted(Counter(pedal.astype(int)).items())})
  in_gear = (gear >= 1) & (gear <= 6)
  read_decoupled = (in_gear & (pedal == CLUTCH_PEDAL_UP) & (decoupled == 1)).sum() * dt
  guarded = (in_gear & (phantom == 1)).sum() * dt
  print(f"in gear (GEAR_MT 1-6), pedal up, read decoupled: {read_decoupled:.1f} s  (of which phantom-guarded {guarded:.1f} s)")
  hi = in_gear & (pedal == CLUTCH_PEDAL_UP) & (rpm >= 1250) & (decoupled == 1)
  print(f"in gear, pedal up, rpm >= 1250, read decoupled (must be 0): {hi.sum() * dt:.2f} s")
  # the phantom episodes: start time, gear, duration, rpm, speed
  starts = np.nonzero(np.diff(np.concatenate([[0], phantom])) == 1)[0]
  ends = np.nonzero(np.diff(np.concatenate([phantom, [0]])) == -1)[0]
  print(f"phantom-guard episodes: {len(starts)}")
  for s, e in zip(starts, ends, strict=True):
    print(f"  t={t[s]:7.2f}  GEAR_MT={int(gear[s])}  {t[e] - t[s]:5.2f} s  rpm {rpm[s]:5.0f}  v {v[s]:5.1f}  pedal {PEDAL_NAMES[int(pedal[s])]}")
  # pedal lead: frames where the pedal is off its top switch while GEAR_MT still reports a gear
  lead = in_gear & ((pedal == CLUTCH_PEDAL_TRAVEL) | (pedal == CLUTCH_PEDAL_FLOOR))
  print(f"pedal pressed while GEAR_MT still in gear (switch leads GEAR_MT): {lead.sum() * dt:.1f} s")
  neutral = (pedal == CLUTCH_PEDAL_UP) & ~in_gear & (gear != 14) & (v > 3)
  print(f"neutral with the clutch out, rolling > 3 m/s (pedal up, GEAR_MT 0): {neutral.sum() * dt:.1f} s")
  if args.gate:
    gate_report(t, v, rpm, decoupled, pedal, gear, gstate, gas, dt)


def gate_report(t, v, rpm, decoupled, pedal, gear, gstate, gas, dt):
  from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import GateState
  names = {int(s): s.name for s in GateState}
  print("\ngate (ClutchShiftGate ON, engaged the whole drive - a what-if, not what the car did):")
  print("  time in state:", {names[int(k)]: f"{n * dt:.1f} s" for k, n in sorted(Counter(gstate.astype(int)).items())})
  # every stretch the gate was not idle: why it started, how it ended, and how long the resume took after the
  # drivetrain read coupled again
  busy = gstate != int(GateState.idle)
  starts = np.nonzero(np.diff(np.concatenate([[0], busy.astype(int)])) == 1)[0]
  ends = np.nonzero(np.diff(np.concatenate([busy.astype(int), [0]])) == -1)[0]
  resume_lags, no_clutch_starts, rows = [], [], []
  for s, e in zip(starts, ends, strict=True):
    seq = [names[int(x)] for x in gstate[s:e + 1]]
    path = [seq[0]] + [b for a, b in zip(seq, seq[1:], strict=False) if a != b]
    coupled_at = None
    for k in range(e, s - 1, -1):     # last frame the drivetrain read decoupled within the stretch
      if decoupled[k]:
        coupled_at = k + 1
        break
    lag = (e + 1 - coupled_at) * dt if coupled_at is not None else None
    if lag is not None and "reengaging" in path:
      resume_lags.append(lag)
    if not decoupled[s]:
      no_clutch_starts.append(round(t[s], 2))
    rows.append((t[s], t[e] - t[s] + dt, " > ".join(path), lag, v[s]))
  print(f"  episodes: {len(rows)}   started without a decoupled drivetrain (flare / hint): {no_clutch_starts}")
  if resume_lags:
    lg = np.array(resume_lags)
    print(f"  resume after the drivetrain read coupled: median {np.median(lg):.2f} s, max {lg.max():.2f} s (n={len(lg)})")
  # throttle the gate would have allowed into a decoupled drivetrain: never (limit_accel caps it), but count the
  # frames the gate was idle while the drivetrain read decoupled (it must be 0: idle means the cap is off)
  idle_decoupled = (gstate == int(GateState.idle)) & (decoupled == 1)
  print(f"  idle while the drivetrain reads decoupled (must be 0): {idle_decoupled.sum()} frames")
  print(f"  neutral -> coastTimeout before MaxCoast: {sum('coastTimeout' in r[2] for r in rows)} episodes")
  print("  longest episodes:")
  for r in sorted(rows, key=lambda r: -r[1])[:8]:
    print(f"    t={r[0]:7.2f}  {r[1]:5.2f} s  v {r[4]:4.1f}  {r[2]}" + (f"  resume +{r[3]:.2f} s" if r[3] is not None else ""))


if __name__ == "__main__":
  main()
