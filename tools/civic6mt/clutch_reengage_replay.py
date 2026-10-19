#!/usr/bin/env python3
"""Integration check for the clutch/shift gate + takeover (Phase 3, Task 8, Task 12 A2) in controlsd, on a real CI segment.

The HONDA (Nidec Civic) process-replay segment with GEARBOX_CVT (0x191) stripped from bus 0 fingerprints as a manual.
It is a ~27 m/s highway drive with openpilot longitudinal active from ~frame 250 to ~4400. carState is rewritten with a
synthetic gear/RPM trace and shifts inside that window, and a carStateSP (live clutch signal, GEAR_MT) is published
after every carState:

  A  off-throttle upshift 5th -> 6th   clutch 1000-1060, slip 0.25 s, then locked in 6th
  B  downshift 6th -> 4th + blip       clutch 2000-2060 (throttle blip while the clutch is open), RPM rises into 4th
     as the clutch bites, engine braking afterwards (aEgo forced to -0.8 m/s^2 for 1.5 s)
  C  on-throttle upshift 4th -> 5th    gas held 2950-3150 through the shift (clutch 3000-3050), released at 3150
  D  (timeout run only) neutral coast  clutch/GEAR_MT 0 from 3400 to the end of the active window

controlsd only, open loop (the logged car doesn't respond to the command; onroadEvents come from the log, so the
injected gas raises no override). radarState and carStateSP are added to the published services.

  capture <file>   at the pre-change commit: ClutchShiftGate OFF -> saves carControl
  check <file>     at the new commit:
    gate OFF                        bit-identical to the capture (OFF = stock)
    gate ON, smart OFF and ON       Task 12 A2: longActive never drops for a shift; while the gate is paused the command
                                    is <= the coast accel (0), exactly 0 with the gas pressed; after the resume the command
                                    changes at most at the takeover rate (or its catch-up) per frame
    D, smart OFF and ON             coastTimeout after ClutchShiftGateMaxCoast (5 s); command <= 0 from the clutch-in on;
                                    longActive never drops

Usage (from the openpilot checkout; downloads the CI segment, needs network):
  python tools/civic6mt/clutch_reengage_replay.py capture /tmp/gate_ref.raw   # at the old commit
  python tools/civic6mt/clutch_reengage_replay.py check /tmp/gate_ref.raw     # at the new commit
"""
import dataclasses
import sys

from openpilot.cereal import messaging
from openpilot.selfdrive.test.process_replay.process_replay import CONFIGS, replay_process
from openpilot.selfdrive.test.process_replay.test_processes import get_log_data, segments
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import engine_rpm_at
from openpilot.tools.lib.logreader import LogReader, save_log

IDLE_RPM = 750.
A_CLUTCH, A_LEN = 1000, 60
B_CLUTCH, B_LEN = 2000, 60
C_GAS, C_CLUTCH, C_LEN, C_GAS_OFF = 2950, 3000, 50, 3150
D_CLUTCH = 3400
SLIP = 25                     # frames for the clutch to bite after GEAR_MT reports the gear
B_EB_ACCEL, B_EB_LEN = -0.8, 150


def gear_trace(i, neutral_coast):
  """(clutch_open, gas, gear, rpm_fn) for carState frame i; rpm_fn(v) -> rpm; gear = what GEAR_MT reports."""
  def locked(g):
    return lambda v: engine_rpm_at(g, v)

  def blend(g_from_rpm, g, k, n):
    return lambda v: g_from_rpm(v) + (engine_rpm_at(g, v) - g_from_rpm(v)) * min(k / n, 1.)

  if neutral_coast and i >= D_CLUTCH:
    return True, False, 0, lambda v: IDLE_RPM
  if i < A_CLUTCH:
    return False, False, 5, locked(5)
  if i < A_CLUTCH + A_LEN:  # clutch in: engine falls from 5th toward the 6th-gear speed
    return True, False, 0, blend(locked(5), 6, i - A_CLUTCH, A_LEN)
  if i < A_CLUTCH + A_LEN + SLIP:
    return False, False, 6, blend(lambda v: engine_rpm_at(6, v) * 0.85, 6, i - A_CLUTCH - A_LEN, SLIP)
  if i < B_CLUTCH:
    return False, False, 6, locked(6)
  if i < B_CLUTCH + B_LEN:  # clutch in: blip in the middle of the shift (rev-match to ~4th)
    k = i - B_CLUTCH
    blip = 20 <= k < 35
    return True, blip, 0, (lambda v: engine_rpm_at(4, v)) if blip else locked(6)
  if i < B_CLUTCH + B_LEN + SLIP:  # bite: RPM pulled up from the rev-matched level into 4th
    return False, False, 4, blend(lambda v: engine_rpm_at(4, v) * 0.9, 4, i - B_CLUTCH - B_LEN, SLIP)
  if i < C_CLUTCH:
    return False, C_GAS <= i, 4, locked(4)
  if i < C_CLUTCH + C_LEN:
    return True, True, 0, blend(locked(4), 5, i - C_CLUTCH, C_LEN)
  if i < C_CLUTCH + C_LEN + SLIP:
    return False, True, 5, blend(lambda v: engine_rpm_at(5, v) * 1.1, 5, i - C_CLUTCH - C_LEN, SLIP)
  return False, i < C_GAS_OFF, 5, locked(5)


def build(neutral_coast=False):
  _, dat = get_log_data(dict(segments)["HONDA"])
  msgs, i = [], 0
  for m in LogReader.from_bytes(dat):
    sp = None
    if m.which() == "can":
      b = m.as_builder()
      keep = [(c.address, c.dat, c.src) for c in m.can if not (c.address == 0x191 and c.src == 0)]
      lst = b.init("can", len(keep))
      for k, (addr, d, src) in enumerate(keep):
        lst[k].address, lst[k].dat, lst[k].src = addr, d, src
      m = b.as_reader()
    elif m.which() == "carState":
      b = m.as_builder()
      clutch, gas, gear, rpm = gear_trace(i, neutral_coast)
      b.carState.deprecated.clutchPressed = clutch
      b.carState.deprecated.engineRpm = float(rpm(m.carState.vEgo))
      b.carState.gasPressed = bool(gas or m.carState.gasPressed)
      if B_CLUTCH + B_LEN <= i < B_CLUTCH + B_LEN + B_EB_LEN:
        b.carState.aEgo = B_EB_ACCEL
      m = b.as_reader()
      sp = messaging.new_message('carStateSP', logMonoTime=m.logMonoTime, valid=True)
      sp.carStateSP.clutchSignalValid, sp.carStateSP.gearRaw = True, gear
      sp = sp.as_reader()
      i += 1
    msgs.append(m)
    if sp is not None:
      msgs.append(sp)
  return msgs


cfg = next(c for c in CONFIGS if c.proc_name == "controlsd")
cfg = dataclasses.replace(cfg, pubs=cfg.pubs + ["radarState", "carStateSP"], subs=cfg.subs + ["carControlSP"])


def run(msgs, **params):
  return replay_process(cfg, msgs, custom_params=params or None, disable_progress=True)


def frames(out):
  """[(carControl, manualLongitudinal)] per controlsd frame."""
  ccs = [m.carControl for m in out if m.which() == "carControl"]
  mls = [m.carControlSP.manualLongitudinal for m in out if m.which() == "carControlSP"]
  return list(zip(ccs, mls, strict=False))


def states_per_frame(msgs):
  res, cs = [], None
  for m in msgs:
    if m.which() == "carState":
      cs = m.carState
    elif m.which() == "selfdriveState":
      res.append(cs)
  return res


def main():
  mode, path = sys.argv[1], sys.argv[2]
  shifts = build()
  if mode == "capture":
    save_log(path, [m for m in run(shifts) if m.which() == "carControl"], compress=False)
    print(f"captured {path}")
    return 0

  from openpilot.sunnypilot.selfdrive.controls.lib.clutch_reengage import CATCHUP_RATE, DEFAULT_MAX_COAST, takeover_rate
  with open(path, "rb") as f:
    ref = [m.carControl for m in LogReader.from_bytes(f.read())]
  ok = True

  off = [cc for cc, _ in frames(run(shifts))]
  same = len(off) == len(ref) and all(a.longActive == b.longActive and a.actuators.accel == b.actuators.accel and
                                      a.actuators.longControlState == b.actuators.longControlState
                                      for a, b in zip(off, ref, strict=True))
  print(f"gate OFF vs capture: {'bit-identical' if same else 'DIFFERS'} ({len(off)} frames)")
  ok &= same

  css = states_per_frame(shifts)
  active_lo, active_hi = A_CLUTCH - 100, D_CLUTCH - 50
  for label, extra in (("smart OFF", {}), ("smart ON", {"ClutchReengageSmart": True})):
    fr = frames(run(shifts, ClutchShiftGate=True, **extra))
    n = min(len(fr), len(css))
    dropped = [i for i in range(active_lo, active_hi) if off[i].longActive and not fr[i][0].longActive]
    paused = [i for i in range(n) if fr[i][1].gatePaused]
    over_coast = [i for i in paused if fr[i][0].actuators.accel > 1e-6]
    gas_nonzero = [i for i in paused if css[i].gasPressed and css[i].deprecated.clutchPressed and fr[i][0].actuators.accel != 0.]
    print(f"  gate ON, {label}: paused frames {len(paused)}, longActive dropped {len(dropped)}, " +
          f"paused frames with throttle {len(over_coast)}, paused+gas with nonzero accel {len(gas_nonzero)}")
    ok &= bool(paused) and not dropped and not over_coast and not gas_nonzero
    resumes = [i for i in range(1, n) if fr[i - 1][1].gatePaused and not fr[i][1].gatePaused]
    for i in resumes:
      cs = css[i]
      cap = max(takeover_rate(cs.vEgo, 6), CATCHUP_RATE) * 0.01 + 1e-6
      steps = [abs(fr[k][0].actuators.accel - fr[k - 1][0].actuators.accel) for k in range(i + 1, min(i + 150, n))]
      print(f"    resume @{i}: first cmd {fr[i][0].actuators.accel:+.3f} (aEgo {cs.aEgo:+.3f}), " +
            f"max per-frame change {max(steps):.4f} (cap {cap:.4f})")
      ok &= max(steps) <= cap

  for label, extra in (("smart OFF", {}), ("smart ON", {"ClutchReengageSmart": True})):
    msgs = build(neutral_coast=True)
    fr = frames(run(msgs, ClutchShiftGate=True, **extra))
    expect = D_CLUTCH + round(DEFAULT_MAX_COAST * 100)
    t_out = next((i for i in range(D_CLUTCH, len(fr)) if fr[i][1].gateState == "coastTimeout"), None)
    end = min(len(fr), D_CLUTCH + 900)
    throttle = [i for i in range(D_CLUTCH + 2, end) if fr[i][0].actuators.accel > 1e-6]
    dropped = [i for i in range(D_CLUTCH, end) if off[i].longActive and not fr[i][0].longActive]
    print(f"  D neutral coast ({label}): coastTimeout @{t_out} (expected ~{expect}); frames with throttle {len(throttle)}; " +
          f"longActive dropped {len(dropped)}")
    ok &= t_out is not None and abs(t_out - expect) <= 3 and not throttle and not dropped

  print("PASS" if ok else "FAIL")
  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
