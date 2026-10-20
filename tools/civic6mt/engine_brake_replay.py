#!/usr/bin/env python3
"""Integration check for Task 6 (EngineBrakePreferred in controlsd, GearAwareAccel + coast logger in plannerd) on a
real CI segment.

Same trick as clutch_reengage_replay.py: the HONDA (Nidec Civic) segment with GEARBOX_CVT (0x191) stripped from bus 0
fingerprints as a manual. Nidec doesn't decode ENGINE_RPM, so carState.deprecated.engineRpm is injected from vEgo
through the seed gearing with a plausible shift schedule, plus a carStateSP after each carState (live clutch signal,
GEAR_MT = the scheduled gear: Task 12 B5 keeps the features inert without it). radarState / carStateSP are added to
the replay configs' published services.

Checks:
  controlsd: RPM injection with the toggle OFF changes nothing; ON, the only change is the brake-range clamp (frames
             where the command is held at exactly the coast target while the plan asks for decel), and no clamped frame
             coincides with a Rule-5 urgency reason (braking_urgency on the same plan / radar / carState).
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
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import DEFAULT_COAST_TARGET, DEFAULT_HARD_FLOOR, \
  DEFAULT_LEAD_CLOSING, DEFAULT_LEAD_TTC, lead_urgent
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
      sp.carStateSP.clutchSignalValid, sp.carStateSP.gearRaw = True, gear
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
cfg = dataclasses.replace(cfg, pubs=cfg.pubs + ["radarState", "carStateSP"])


def run_controlsd(msgs, on, coast_target=None):
  params = {"EngineBrakePreferred": True} if on else {}
  if coast_target is not None:
    params["EngineBrakeCoastTarget"] = coast_target
  out = replay_process(cfg, msgs, custom_params=params or None, disable_progress=True)
  return [m.carControl for m in out if m.which() == "carControl"]


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


# Task 16: the default coast target is -0.2 (Stage 2 C2), which this segment's gentle trims rarely go below, so the same
# check also runs at 0.0 (the Task 6 setting) - that run must clamp something, or the proof would be vacuous
for target, label, must_clamp in ((DEFAULT_COAST_TARGET, f"default {DEFAULT_COAST_TARGET:+.1f}", False), (0.0, "target 0.0", True)):
  on = run_controlsd(rpm_msgs, True, None if target == DEFAULT_COAST_TARGET else target)
  clamped = [i for i in range(n) if on[i].longActive and on[i].actuators.accel == target and
             plans[i] is not None and plans[i].aTarget < target]
  diff = [i for i in range(n) if on[i].actuators.accel != off[i].actuators.accel]
  neg_off = [i for i in range(n) if off[i].longActive and off[i].actuators.accel < 0.]
  bad = [i for i in clamped if any(urgent(j) for j in range(max(i - 2, 0), min(i + 3, n)))]
  first_diff_is_clamp = not diff or diff[0] in clamped
  print(f"controlsd ON ({label}): long-active frames {sum(c.longActive for c in on)}, OFF brake-range frames {len(neg_off)}, " +
        f"clamped frames {len(clamped)}, differing frames {len(diff)}")
  print(f"  first difference is a clamp: {first_diff_is_clamp}; clamped frames during urgency: {len(bad)}")
  print(f"  min accel OFF {min(c.actuators.accel for c in off):.2f}, ON {min(c.actuators.accel for c in on):.2f}; " +
        f"gears seen {sorted(Counter(gear_for(s.vEgo) for s in states if s).items())}")
  ok &= first_diff_is_clamp and not bad and (len(clamped) > 0 or not must_clamp)

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
