"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Civic 6MT test console profile: the plan's pass rules — turn each test's measurements (core analyzers) into PASS / FAIL /
CAPTURED + a one-line detail in this car's terms (GEAR_MT, 0x17C, PR #2050, test ids). Analysis only.

Thresholds marked "provisional" are first guesses for the car; the raw captures are kept so they can be re-scored.
"""
import math

from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import FLARE_RPM_RATE, GAS_RELEASE_WINDOW, REENGAGE_DELAY
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import GEAR_RATIOS, overall_ratio
from openpilot.sunnypilot.selfdrive.test_console.analyzers import CAPTURED, FAIL, PASS, BitHuntAnalyzer, EpisodeAnalyzer, \
  GearRawMapAnalyzer, NonAdaptiveAnalyzer, SegmentAnalyzer, _fmt, _pct, health as _health

RESUME_JERK_MAX = 3.0        # m/s^3, provisional "smooth resume" bound
HANDBACK_JERK_MAX = 2.5      # m/s^3, provisional "no lurch" bound
PULSE_HOLD_MAX = 20.         # s, provisional: a single overspeed pulse longer than this is a brake hold, not a pulse
PULSE_MIN_COAST = 1.0        # s, provisional: the coast between two pulses must last at least this
BAND_TOL = 0.5               # m/s: pulse start / release vs set + trigger / set + release


def r_brake_strength(a: SegmentAnalyzer):
  if not a.segments:
    return None, ""
  cmd, acc = a.pooled("accel_cmd"), a.pooled("a_ego")
  coast = a.pooled("a_coast")
  ratio = acc / cmd if cmd < -1e-3 else math.nan
  assist = sum(g.assist_on for g in a.segments) / max(1, sum(g.assist_n for g in a.segments))
  detail = (f"achieves {ratio:.0%} of request (cmd {cmd:.2f}, a_ego {acc:.2f}, coast est {_fmt(coast)}); " +
            f"brake-assist bit {assist:.0%}")
  return (PASS if ratio >= 0.5 else FAIL), detail + ("" if ratio >= 0.5 else " - weak/no braking (PR #2050)")


def r_cmd_semantics(a: SegmentAnalyzer):
  parts = []
  for key, name in (("zero", "cmd 0"), ("small_neg", "small neg")):
    if not a.by_key(key):
      continue
    acc, coast, cmd = a.pooled("a_ego", key), a.pooled("a_coast", key), a.pooled("accel_cmd", key)
    excess = acc - coast
    if key == "zero":
      verdict = "HOLDS speed (accel semantics)" if abs(acc) < 0.15 and excess > 0.15 else \
                "COASTS (closed throttle)" if abs(excess) < 0.15 else "unclear"
    else:
      verdict = "uses SERVICE BRAKES" if excess < -0.15 else "fuel-cut/coast only"
    parts.append(f"{name}: a {acc:.2f} vs coast est {_fmt(coast)} (cmd {cmd:.2f}) -> {verdict}")
  if not parts:
    return None, ""
  return CAPTURED, "; ".join(parts) + " [coast est uncalibrated: compare E0b]"


def r_rpm_noise(a: SegmentAnalyzer):
  if not a.segments:
    return None, ""
  secs = sum(g.dur for g in a.segments)
  flares = sum(g.flares for g in a.segments)
  rate = sum(g.rpm_changes for g in a.segments) / secs if secs else 0.
  detail = (f"{flares} false flares in {secs:.0f}s steady; RPM changes ~{rate:.0f}/s; " +
            f"max rate {_fmt(a.extreme('rpm_rate', max), 0)} rpm/s (threshold {FLARE_RPM_RATE:.0f}). Check tach by eye.")
  return (PASS if flares == 0 else FAIL), detail


def r_gear_mt_map(a: GearRawMapAnalyzer):
  if not (a.total or a.stop_hist or a.rev_hist):
    return None, ""
  bad, parts = [], []
  for g in range(1, 7):
    tot = a.total.get(g, 0.)
    if tot <= 0.:
      continue
    pct = a.agree[g] / tot
    parts.append(f"{g}:{pct:.0%}")
    if tot >= a.need_s and pct < 0.95:
      inferred = max(a.inferred_when[g].items(), key=lambda kv: kv[1])[0]
      bad.append(f"GEAR_MT {g} reads as inferred {inferred}")
  odd = sorted(k for k in a.total if not 1 <= k <= 6)
  if odd:
    parts.append("rolling non-gear values " + ",".join(map(str, odd)))
  rev = sorted(a.rev_hist)
  if rev and rev != [14]:
    bad.append(f"reverse reads {rev}")
  stop = ",".join(f"{k}:{v:.0f}s" for k, v in sorted(a.stop_hist.items()))
  detail = f"agree {' '.join(parts) or '-'}; R={rev or '-'}; at stop {stop or '-'}"
  if bad:
    return FAIL, "; ".join(bad) + " | " + detail
  return (PASS if a.complete else None), detail


def r_bite_lag(a: EpisodeAnalyzer):
  lags = [e["bite_lag"] for e in a.episodes if e["bite_lag"] is not None]
  dec = [e["decouple_lag"] for e in a.episodes if e["decouple_lag"] is not None]
  if not lags:
    return None, ""
  p90 = _pct(lags, 0.9)
  detail = (f"bite lag median {_pct(lags, 0.5):.2f}s p90 {p90:.2f}s (REENGAGE_DELAY {REENGAGE_DELAY}s); " +
            f"GEAR_MT->0 vs decouple median {_fmt(_pct(dec, 0.5))}s (+ = GEAR_MT lags)")
  return (PASS if p90 <= REENGAGE_DELAY else FAIL), detail


def r_lift_to_clutch(a: EpisodeAnalyzer):
  d = [e["gas_release_dt"] for e in a.episodes]
  if not d:
    return None, ""
  p90 = _pct(d, 0.9)
  return (PASS if p90 <= GAS_RELEASE_WINDOW else FAIL), \
         f"lift->clutch median {_pct(d, 0.5):.2f}s p90 {p90:.2f}s (GAS_RELEASE_WINDOW {GAS_RELEASE_WINDOW}s; only matters with the hint on)"


def r_blip_trace(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  flick = sum(e["flickers"] for e in a.episodes)
  leaks = sum(1 for e in a.episodes if "shifting" in e["gate_states"] and
              any(st != "shifting" for st in e["gate_states"][e["gate_states"].index("shifting"):]))
  gate_seen = any(e["gate_states"] for e in a.episodes)
  rates = [e["rpm_rate_max_open"] for e in a.episodes]
  detail = (f"{len(a.episodes)} blips: GEAR_MT flickers {flick}, gate leaks {leaks}" +
            f"{'' if gate_seen else ' (gate off: leak not exercised)'}; blip rpm rate max {max(rates):.0f} rpm/s")
  return (PASS if flick == 0 and leaks == 0 else FAIL), detail


def r_gate_accept(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  bad = [e for e in a.episodes if e["long_active_open"] or not e["enabled_min"] or e["acc_fault"]]
  detail = f"{len(a.episodes) - len(bad)}/{len(a.episodes)} presses paused, stayed engaged, no ACC fault"
  if bad:
    e = bad[0]
    detail += f"; first bad: long_active={e['long_active_open']} engaged={e['enabled_min']} fault={e['acc_fault']}"
  health = _health(a)
  return (FAIL, detail + "; " + "; ".join(health)) if bad or health else (PASS, detail)


def r_gate_shifts(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  throttle = [e for e in a.episodes if e["long_active_open"] or e["accel_out_max_open"] > 0.05]
  jerky = [e for e in a.episodes if e["resume_jerk_max"] > RESUME_JERK_MAX]
  flares = a.detector.false_flares
  aligned = [e["gate_shift_dt"] for e in a.episodes if e["gate_shift_dt"] is not None]
  detail = (f"{len(a.episodes)} shifts: throttle-with-clutch-in {len(throttle)}, jerky resumes {len(jerky)} " +
            f"(max {max(e['resume_jerk_max'] for e in a.episodes):.1f} m/s^3), false flares {flares}; " +
            f"gate 'shifting' after GEAR_MT->0: median {_fmt(_pct(aligned, 0.5))}s")
  return (PASS if not throttle and not jerky and flares == 0 else FAIL), detail


def r_gate_revmatch(a: EpisodeAnalyzer):
  if not a.episodes and not a.extra_episodes:
    return None, ""
  bad = [e for e in a.episodes if not e["enabled_min"] or e["long_active_open"]]
  brakes = [e["disengage_dt"] for e in a.extra_episodes]
  slow = [d for d in brakes if d is None or d > 0.3]
  detail = (f"blips {len(a.episodes) - len(bad)}/{len(a.episodes)} stayed engaged+paused; " +
            f"brake disengage {', '.join(_fmt(d) + 's' for d in brakes) or '-'}")
  return (PASS if not bad and not slow and a.complete else FAIL if bad or slow else None), detail


def r_eb_downshift(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  added = [e for e in a.episodes if e["post_cmd_max"] > 0.1]
  mx = max(e["post_cmd_max"] for e in a.episodes)
  return CAPTURED, f"throttle added after re-engage in {len(added)}/{len(a.episodes)} downshifts (max cmd {_fmt(mx)})"


def r_non_adaptive(a: NonAdaptiveAnalyzer):
  if a.hold_s <= 0. and a.lead_s <= 0.:
    return None, ""
  mean_err = a.err_sum / a.hold_s if a.hold_s else math.nan
  p95 = a.err_p95()
  hold_ok = abs(mean_err) < 0.5 and p95 < 1.5
  lead_ok = a.lead_cmd_min >= -0.1
  detail = (f"hold err mean {_fmt(mean_err)} p95 {_fmt(p95, 1)} m/s; behind slower lead min cmd " +
            f"{_fmt(a.lead_cmd_min if a.lead_s else math.nan)}; FCW x{a.fcw_edges}")
  if not a.complete:
    return (FAIL if (a.hold_s >= a.hold_need and not hold_ok) or (a.lead_s >= a.lead_need and not lead_ok) else None), detail
  return (PASS if hold_ok and lead_ok else FAIL), detail


def r_overset_brake(a: SegmentAnalyzer):
  if not a.segments:
    return None, ""
  assist = sum(g.assist_on for g in a.segments) / max(1, sum(g.assist_n for g in a.segments))
  return CAPTURED, (f"over set by {a.pooled('v_ego') - a.pooled('v_cruise'):.1f} m/s: cmd {a.pooled('accel_cmd'):.2f}, " +
                    f"sent {_fmt(a.pooled('accel_sent'))}, a_ego {a.pooled('a_ego'):.2f}, excess vs coast " +
                    f"{_fmt(a.pooled('excess'))}, brake-assist bit {assist:.0%}")


def r_nsb_floor(a: SegmentAnalyzer):
  parts, ok = [], True
  for key in a.keys:
    if not a.by_key(key):
      continue
    floor = float(key)
    out_min, exc = a.extreme("accel_out", min, key), a.pooled("excess", key)
    good = out_min >= -floor - 0.01 and not (exc < -0.3)
    ok &= good
    parts.append(f"floor {key}: min out {out_min:.2f}, excess {_fmt(exc)} {'✓' if good else '×'}")
  if not parts:
    return None, ""
  return (PASS if ok and a.complete else FAIL if not ok else None), "; ".join(parts)


def r_backstop(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  pulses = [e for e in a.episodes if e.get("mode") == "pulse"]
  if pulses:
    return r_pulse(a, pulses)
  e = a.episodes[-1]
  at_margin = abs(e["v_over"] - e["margin"]) <= 0.5
  braked = e["a_min"] <= -0.3
  detail = f"triggered at set+{e['v_over']:.1f} m/s (margin {e['margin']:.1f}); min a_ego in 3 s {e['a_min']:.2f}, min cmd {e['cmd_min']:.2f}"
  return (PASS if at_margin and braked else FAIL), detail + ("" if braked else " - backstop too weak (see C1)")


def pulse_faults(e: dict) -> list[str]:
  """Why one overspeed pulse (Task 11) is not clean. Escalated pulses are scored on the band + their alert only."""
  bad = []
  if abs(e["v_over"] - e["trigger"]) > BAND_TOL:
    bad.append(f"started at set+{e['v_over']:.1f} (trigger {e['trigger']:.1f})")
  if e["v_over_end"] is None or abs(e["v_over_end"] - e["release"]) > BAND_TOL:
    bad.append(f"released at set+{_fmt(e['v_over_end'], 1)} (release {e['release']:.1f})")
  if e["escalated"]:
    if e["alerts"] != 1:
      bad.append(f"escalated with {e['alerts']} alerts (want 1)")
    return bad
  if e["cmd_min"] < e["decel"] - 0.05 or e["cmd_max"] > 0.05:
    bad.append(f"cmd {e['cmd_min']:.2f}..{e['cmd_max']:.2f} outside [{e['decel']:.1f}, 0]")
  if e["dur"] > PULSE_HOLD_MAX:
    bad.append(f"{e['dur']:.0f}s pulse (brake hold)")
  if e["coast_before"] is not None and (e["coast_before"] < PULSE_MIN_COAST or e["coast_cmd_min"] < -e["floor"] - 0.05):
    bad.append(f"no coast before it ({e['coast_before']:.1f}s, min cmd {_fmt(e['coast_cmd_min'])})")
  return bad


def r_pulse(a: EpisodeAnalyzer, pulses: list[dict]):
  faults = [(e, pulse_faults(e)) for e in pulses]
  bad = [f for _, f in faults if f]
  esc = [e for e in pulses if e["escalated"]]
  e0 = pulses[-1]
  detail = (f"pulse: {len(pulses) - len(bad)}/{len(pulses)} clean; started set+{_pct([e['v_over'] for e in pulses], 0.5):.1f} " +
            f"(trigger {e0['trigger']:.1f}), released set+{_pct([e['v_over_end'] for e in pulses if e['v_over_end'] is not None], 0.5):.1f} " +
            f"(release {e0['release']:.1f}), cmd >= {min(e['cmd_min'] for e in pulses if not e['escalated']) if len(esc) < len(pulses) else math.nan:.2f} " +
            f"(bound {e0['decel']:.1f}), longest {max(e['dur'] for e in pulses):.0f}s")
  if esc:
    detail += (f"; ESCALATED {len(esc)}x at set+{max(e['v_over_esc'] for e in esc):.1f} (alerts {sum(e['alerts'] for e in esc)}) - " +
               "the bounded pulse can't hold this grade (see C1)")
  if bad:
    detail += "; first fault: " + ", ".join(bad[0])
  return (FAIL if bad else PASS if a.complete else None), detail


def r_handback_feel(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  bad = [e for e in a.episodes if e["alerts"] != 1 or e["jerk_max"] > HANDBACK_JERK_MAX]
  detail = "; ".join(f"{e['v'] * 2.237:.1f}mph g{e['gear']} jerk {e['jerk_max']:.1f} alerts {e['alerts']}" for e in a.episodes[-3:])
  return (PASS if not bad and a.complete else FAIL if bad else None), f"{len(a.episodes) - len(bad)}/{len(a.episodes)} ok | {detail}"


def r_stop_hold(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  rolled = [e for e in a.episodes if e["rolled"]]
  hills = len(a.keyed("hill"))
  detail = (f"{len(a.episodes)} stops ({hills} hill): rollback {len(rolled)}; brake hold seen " +
            f"{sum(e['brake_hold'] for e in a.episodes)}x; max roll {max(e['roll_v'] for e in a.episodes):.2f} m/s")
  return (FAIL if rolled else PASS if a.complete else None), detail


def r_neutral_coast(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  e = a.episodes[-1]
  hist = ",".join(f"{k}:{v}" for k, v in sorted(e["gear_raw"].items()))
  return CAPTURED, (f"{len(a.episodes)} coasts; last {e['dur']:.0f}s: GEAR_MT {hist}, clutchPressed {e['clutch_n'] / max(1, e['n']):.0%}, " +
                    f"long active {e['long_n'] / max(1, e['n']):.0%}, gate {'/'.join(e['gate_states']) or '-'}")


def r_uphill_handback(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  return CAPTURED, "; ".join(f"pitch {e['pitch']:.3f}: decel after ramp {_fmt(e['a_after_mean'])}, " +
                             f"v {_fmt(e['v_ramp_end'])}->{_fmt(e['v_end'])} m/s" for e in a.episodes)


def r_ratio_check(a: SegmentAnalyzer):
  parts, bad = [], []
  for g in a.keys:
    if not a.by_key(g):
      continue
    r = a.pooled("ratio", g)
    err = r / overall_ratio(g) - 1.
    parts.append(f"{g}:{err:+.1%} (FD~{r / GEAR_RATIOS[g - 1]:.2f})")
    if abs(err) > 0.03:
      bad.append(g)
  if not parts:
    return None, ""
  detail = "ratio vs seed " + " ".join(parts) + " [FD assumes the seed tire radius]"
  return (FAIL if bad else PASS if a.complete else None), detail + (f"; off: gears {bad}" if bad else "")


def r_coast_fit(a: SegmentAnalyzer):
  parts = []
  for g in a.keys:
    if not a.by_key(g):
      continue
    parts.append(f"g{g} a {a.pooled('a_ego', g):.2f} (model {_fmt(a.pooled('a_coast', g))}, pitch {a.pooled('pitch', g):+.3f})")
  return (CAPTURED, "; ".join(parts)) if parts else (None, "")


def r_ebp_trims(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  bad = [e for e in a.episodes if e["capacity"] or e["out_min"] < a.ctx.ebp_coast_target - 0.01 or e["excess_min"] < -0.3]
  detail = (f"{len(a.episodes) - len(bad)}/{len(a.episodes)} clean; capacity releases {sum(e['capacity'] for e in a.episodes)}; " +
            f"reasons {','.join(str(e['reason']) for e in a.episodes[-3:])}")
  return (FAIL if bad else PASS if a.complete else None), detail


def r_ebp_handoff(a: EpisodeAnalyzer):
  on, off = a.keyed("ebp_on"), a.keyed("ebp_off")
  if not on and not off:
    return None, ""
  gap_on = min((e["gap_min"] for e in on), default=math.nan)
  gap_off = min((e["gap_min"] for e in off), default=math.nan)
  fcw = any(e["fcw"] for e in a.episodes)
  detail = f"min gap EBP on {_fmt(gap_on, 1)} m vs off {_fmt(gap_off, 1)} m; FCW {'yes' if fcw else 'no'}"
  if not (on and off):
    return (FAIL if fcw else None), detail
  return (PASS if gap_on >= gap_off - 2. and not fcw else FAIL), detail


def r_ebp_downhill(a: SegmentAnalyzer):
  if not a.segments:
    return None, ""
  over = max((g.maxs.get("v_ego", 0.) - g.mean("v_cruise")) for g in a.segments)
  detail = f"max over set {over:.1f} m/s; a_ego {a.pooled('a_ego'):.2f}, pitch {a.pooled('pitch'):+.3f}"
  return (FAIL if over > 3.6 else PASS if a.complete else None), detail


def r_jerk_by_gear(a: SegmentAnalyzer):
  parts = []
  for g in a.keys:
    segs = a.by_key(g)
    if segs:
      n = sum(s.n for s in segs)
      parts.append(f"g{g} jerk mean {sum(s.jerk_sum for s in segs) / max(1, n):.2f} max {max(s.jerk_max for s in segs):.2f}")
  return (CAPTURED, "; ".join(parts)) if parts else (None, "")


def r_accel_by_gear(a: SegmentAnalyzer):
  parts = [f"g{g} a {a.pooled('a_ego', g):.2f} @pitch {a.pooled('pitch', g):+.3f} rpm {a.pooled('rpm', g):.0f}"
           for g in a.keys if a.by_key(g)]
  return (CAPTURED, "; ".join(parts)) if parts else (None, "")


def r_bit_hunt(a: BitHuntAnalyzer):
  parts = []
  sig = a.signature_candidates()
  if sig:
    parts.append("signature (DBC bit, N): " + ", ".join(f"{b} N={n}" for b, n in sig[:5]))
  if a.edges:
    c = a.candidates()
    parts.append("vs GEAR_MT (DBC bit, hit rate, toggles): " + ", ".join(f"{b} {h:.0%} {n}" for b, h, n in c[:5]) if c else
                 f"no bit tracks GEAR_MT->0 across {len(a.edges)} edges")
  if not parts and a.window_ok <= 0.:
    return None, ""
  return CAPTURED, "; ".join(parts) or f"no 0x17C bit matched the signature in {a.window_ok:.0f}s (scrub cabana)"


def r_window(a: SegmentAnalyzer):
  """S7: a marked capture window for scrubbing every bus in cabana (the volume buttons are likely on B-CAN)."""
  secs = sum(g.dur for g in a.segments)
  if secs <= 0.:
    return None, ""
  return CAPTURED, f"{secs:.0f}s window marked (test_marker start/stop): scrub every logged bus in cabana for the 4/5 rhythm"


RESULTS = {
  "brake_strength": r_brake_strength, "cmd_semantics": r_cmd_semantics, "rpm_noise": r_rpm_noise,
  "gear_mt_map": r_gear_mt_map, "bite_lag": r_bite_lag, "lift_to_clutch": r_lift_to_clutch, "blip_trace": r_blip_trace,
  "gate_accept": r_gate_accept, "gate_shifts": r_gate_shifts, "gate_revmatch": r_gate_revmatch, "eb_downshift": r_eb_downshift,
  "non_adaptive": r_non_adaptive, "overset_brake": r_overset_brake, "nsb_floor": r_nsb_floor, "backstop": r_backstop,
  "handback_feel": r_handback_feel, "stop_hold": r_stop_hold, "neutral_coast": r_neutral_coast,
  "uphill_handback": r_uphill_handback, "ratio_check": r_ratio_check, "coast_fit": r_coast_fit, "ebp_trims": r_ebp_trims,
  "ebp_handoff": r_ebp_handoff, "ebp_downhill": r_ebp_downhill, "jerk_by_gear": r_jerk_by_gear,
  "accel_by_gear": r_accel_by_gear, "bit_hunt": r_bit_hunt, "window": r_window,
}
