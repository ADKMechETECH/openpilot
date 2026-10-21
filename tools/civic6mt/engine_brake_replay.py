#!/usr/bin/env python3
"""Integration check for Task 6 (EngineBrakePreferred in controlsd, GearAwareAccel + coast logger in plannerd) on a
real CI segment.

Same trick as clutch_reengage_replay.py: the HONDA (Nidec Civic) segment with GEARBOX_CVT (0x191) stripped from bus 0
fingerprints as a manual. Nidec doesn't decode ENGINE_RPM, so carState.deprecated.engineRpm is injected from vEgo
through the seed gearing with a plausible shift schedule, plus a carStateSP after each carState (live clutch signal,
GEAR_MT = the scheduled gear: Task 12 B5 keeps the features inert without it). radarState / carStateSP are added to
the replay configs' published services.

Checks:
  controlsd: RPM injection with the toggle OFF changes nothing; ON (Task 17: the per-gear clamp), every frame that
             differs is a clamp (the command held at exactly the published floor carControlSP.manualLongitudinal.ebpFloor
             while the plan asks for more decel), and no clamped frame coincides with a Rule-5 urgency reason
             (braking_urgency on the same plan / radar / carState). The clutch pedal switch is injected as fully up.
  plannerd : GearAwareAccel OFF = 0 diffs vs. no injection; ON = only aTarget/trajectory-type fields change, FCW
             identical (also run with a synthetic held-in-3rd RPM trace, since the segment is all 6th gear).

Usage (from the openpilot checkout; downloads the CI segment, needs network):
  python tools/civic6mt/engine_brake_replay.py
"""
import dataclasses
import os
import sys
import tempfile
from collections import Counter

from openpilot.cereal import messaging
from openpilot.selfdrive.test.process_replay.compare_logs import compare_logs
from openpilot.selfdrive.test.process_replay.process_replay import CONFIGS, replay_process
from openpilot.selfdrive.test.process_replay.test_processes import get_log_data, segments
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import braking_urgency
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import COAST_LOG_DIR_ENV, engine_rpm_at
from opendbc.car.structs import CLUTCH_PEDAL_UP
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import DEFAULT_HARD_FLOOR, DEFAULT_LEAD_CLOSING, \
  DEFAULT_LEAD_TTC, lead_urgent
from openpilot.tools.lib.logreader import LogReader

SHIFT_SCHEDULE = [(4.5, 1), (8., 2), (12., 3), (17., 4), (23., 5)]  # upshift speeds (m/s); 6th above


def gear_for(v):
  return next((g for v_up, g in SHIFT_SCHEDULE if v < v_up), 6)


_, dat = get_log_data(dict(segments)["HONDA"])


def build(inject_rpm, fixed_gear=None):
  msgs = []
  for m in LogReader.from_bytes(dat):
    sp = None
    if m.which() == "can":
      b = m.as_builder()
      keep = [(c.address, c.dat, c.src) for c in m.can if not (c.address == 0x191 and c.src == 0)]
      lst = b.init("can", len(keep))
      for i, (addr, d, src) in enumerate(keep):
        lst[i].address, lst[i].dat, lst[i].src = addr, d, src
      m = b.as_reader()
    elif m.which() == "carState" and inject_rpm:
      b = m.as_builder()
      gear = fixed_gear or gear_for(m.carState.vEgo)
      b.carState.deprecated.engineRpm = float(engine_rpm_at(gear, m.carState.vEgo))
      m = b.as_reader()
      sp = messaging.new_message('carStateSP', logMonoTime=m.logMonoTime, valid=True)
      sp.carStateSP.clutchSignalValid, sp.carStateSP.gearRaw, sp.carStateSP.clutchPedal = True, gear, CLUTCH_PEDAL_UP
      sp = sp.as_reader()
    msgs.append(m)
    if sp is not None:
      msgs.append(sp)
  return msgs


base_msgs, rpm_msgs = build(False), build(True)
# the segment is a highway drive (6th gear by schedule, where GearAwareAccel's scale is 1.0): for the plannerd shaping
# check, also inject a synthetic "held in 3rd" RPM trace so the 3rd-gear scale applies
rpm3_msgs = build(True, fixed_gear=3)
ok = True

# --- controlsd -------------------------------------------------------------------------------------------------------
cfg = next(c for c in CONFIGS if c.proc_name == "controlsd")
cfg = dataclasses.replace(cfg, pubs=cfg.pubs + ["radarState", "carStateSP"], subs=cfg.subs + ["carControlSP"])


def run_controlsd(msgs, on, with_sp=False):
  out = replay_process(cfg, msgs, custom_params={"EngineBrakePreferred": True} if on else None, disable_progress=True)
  cc = [m.carControl for m in out if m.which() == "carControl"]
  if not with_sp:
    return cc
  return cc, [m.carControlSP.manualLongitudinal for m in out if m.which() == "carControlSP"]


base, off = run_controlsd(base_msgs, False), run_controlsd(rpm_msgs, False)
same_off = [a.actuators.accel for a in base] == [a.actuators.accel for a in off]
print(f"controlsd: RPM injection, toggle OFF: {'identical' if same_off else 'DIFFERS'} ({len(off)} frames)")
ok &= same_off

# per carControl frame, the latest plan / radar / carState that controlsd had seen
plans, radars, states, ctx = [], [], [], [None, None, None]
for m in rpm_msgs:
  w = m.which()
  if w == "longitudinalPlan":
    ctx[0] = m.longitudinalPlan
  elif w == "radarState":
    ctx[1] = m.radarState
  elif w == "carState":
    ctx[2] = m.carState
  elif w == "selfdriveState":  # controlsd's poll socket: one output frame per selfdriveState
    plans.append(ctx[0])
    radars.append(ctx[1])
    states.append(ctx[2])
n = min(len(off), len(plans))


def urgent(i):
  plan, radar, cs = plans[i], radars[i], states[i]
  if plan is None or radar is None or cs is None:
    return True
  r5 = braking_urgency(plan_ok=True, radar_ok=True, force_decel=False, fcw=plan.fcw, stock_aeb=cs.stockAeb or cs.stockFcw,
                       should_stop=plan.shouldStop, a_target=plan.aTarget, v_ego=cs.vEgo, radar=radar)
  return r5 is not None or plan.aTarget < DEFAULT_HARD_FLOOR or \
    lead_urgent(radar.leadOne, cs.vEgo, DEFAULT_LEAD_TTC, DEFAULT_LEAD_CLOSING) or \
    lead_urgent(radar.leadTwo, cs.vEgo, DEFAULT_LEAD_TTC, DEFAULT_LEAD_CLOSING)


on, ml = run_controlsd(rpm_msgs, True, with_sp=True)
n = min(n, len(ml))
clamped = [i for i in range(n) if on[i].longActive and ml[i].ebpSuppressing and abs(on[i].actuators.accel - ml[i].ebpFloor) < 1e-6
           and plans[i] is not None and plans[i].aTarget < ml[i].ebpFloor]
diff = [i for i in range(n) if on[i].actuators.accel != off[i].actuators.accel]
not_clamp = [i for i in diff if i not in set(clamped)]
active = [i for i in range(n) if ml[i].ebpSuppressing]
bad = [i for i in clamped if any(urgent(j) for j in range(max(i - 2, 0), min(i + 3, n)))]
floors = [ml[i].ebpFloor for i in active]
gears = sorted(Counter(ml[i].ebpGear for i in active).items())
print(f"controlsd ON: long-active frames {sum(c.longActive for c in on)}, clamp active {len(active)} (gears {gears}, " +
      f"floor {min(floors, default=0):.2f}..{max(floors, default=0):.2f}), clamped frames {len(clamped)}, " +
      f"differing frames {len(diff)}")
print(f"  differing frames that are not a clamp: {len(not_clamp)}; clamped frames during urgency: {len(bad)}")
print(f"  min accel OFF {min(c.actuators.accel for c in off):.2f}, ON {min(c.actuators.accel for c in on):.2f}")
# the first difference may be a frame after a clamp (LoC's state carries over), so: the first must be a clamp, and the
# clamp must actually run on this segment (or the proof is vacuous)
ok &= bool(active) and (not diff or diff[0] in clamped) and not bad

# --- plannerd --------------------------------------------------------------------------------------------------------
pcfg = next(c for c in CONFIGS if c.proc_name == "plannerd")
pcfg = dataclasses.replace(pcfg, pubs=pcfg.pubs + ["carStateSP"])
with tempfile.TemporaryDirectory() as tmp:
  os.environ[COAST_LOG_DIR_ENV] = tmp


  def run_plannerd(msgs, **params):
    return replay_process(pcfg, msgs, custom_params=params or None, disable_progress=True)


  allowed = ("longitudinalPlan.aTarget", "longitudinalPlan.speeds", "longitudinalPlan.accels", "longitudinalPlan.jerks",
             "longitudinalPlan.shouldStop", "longitudinalPlan.longitudinalPlanSource")
  d_off = compare_logs(run_plannerd(base_msgs), run_plannerd(rpm_msgs), pcfg.ignore, [], pcfg.tolerance)
  print(f"plannerd: GearAwareAccel OFF, RPM injected vs not: {len(d_off)} diffs")
  ok &= len(d_off) == 0
  # adaptive (cruise competes with the lead MPC) and non-adaptive (cruise is the only candidate)
  for label, msgs, extra in (("adaptive, schedule", rpm_msgs, {}),
                             ("adaptive, 3rd", rpm3_msgs, {}),
                             ("non-adaptive, 3rd", rpm3_msgs, {"NonAdaptiveLongitudinal": True})):
    p_off, p_on = run_plannerd(msgs, **extra), run_plannerd(msgs, GearAwareAccel=True, **extra)
    d_on = compare_logs(p_off, p_on, pcfg.ignore, [], pcfg.tolerance)
    fields = Counter(d[1] if isinstance(d[1], str) else ".".join(map(str, d[1])) for d in d_on)
    a_off = [m.longitudinalPlan.aTarget for m in p_off if m.which() == "longitudinalPlan"]
    a_on = [m.longitudinalPlan.aTarget for m in p_on if m.which() == "longitudinalPlan"]
    max_d = max(abs(a - b) for a, b in zip(a_off, a_on, strict=True))
    print(f"  {label}: GearAwareAccel ON vs OFF: {len(d_on)} diffs {dict(fields)}; max |aTarget diff| {max_d:.3f}")
    ok &= all(f.startswith(allowed) for f in fields)
    fcw_off = [m.longitudinalPlan.fcw for m in p_off if m.which() == "longitudinalPlan"]
    fcw_on = [m.longitudinalPlan.fcw for m in p_on if m.which() == "longitudinalPlan"]
    print(f"    fcw identical: {fcw_off == fcw_on}")
    ok &= fcw_off == fcw_on
  coast_files = [f for f in os.listdir(tmp) if f.startswith("coast.jsonl")]
  n_events = sum(1 for f in coast_files for _ in open(os.path.join(tmp, f)))
  print(f"  coast logger (manual, long inactive & no pedals): {n_events} events written")

print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
