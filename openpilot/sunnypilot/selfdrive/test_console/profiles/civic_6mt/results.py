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
COAST_CAL_TEST = "E0b"       # flat coasts per gear: the coast model is FIT to these (Task 13 B1), else it is the seed
FIT_MIN_SAMPLES = 300        # stats samples (~3 s) per gear for a usable per-gear fit
SENT_TOL = 0.25              # m/s^2: the ACCEL_COMMAND seen on the bus must be within this of the command (C1)
RESUME_BOUND = REENGAGE_DELAY + 1.0   # s: openpilot must resume within this after the clutch closes (G2, Task 12 D3)
COVERAGE_MIN = 0.8           # S2: share of shifts per gear whose bite was actually measured


def coast_fit(ctx) -> dict[int, float] | None:
  """Task 13 B1 [D-03, D-12]: the per-gear coast-model correction (a_ego - seed, m/s^2) FIT from the E0b capture — not
  merely "E0b complete" (finishing E0b only captures; nothing refit the seed). Task 14 C1 [N13-1]: per gear - the gears
  whose own E0b capture is done and has a usable fit; None if there is none. (It used to need all six, so one sparse gear
  blocked every braking verdict.) The rules gate on the gears their own samples used (coast_calibrated)."""
  entry = ctx.results.get(COAST_CAL_TEST, {})
  fit = entry.get("coast_fit")
  gears = fit.get("gears") if isinstance(fit, dict) else None
  if not isinstance(gears, dict):
    return None
  v1 = fit.get("version") == 1 and entry.get("complete")   # Task 13 files: written only on completion = every gear done
  out = {}
  for g in range(1, len(GEAR_RATIOS) + 1):
    e = gears.get(str(g))
    if isinstance(e, dict) and isinstance(e.get("bias"), (int, float)) and math.isfinite(e["bias"]) and \
       isinstance(e.get("n"), (int, float)) and e["n"] >= FIT_MIN_SAMPLES and (e.get("done") is True or v1):
      out[g] = float(e["bias"])
  return out or None


def coast_calibrated(ctx, gears=None) -> bool:
  """A usable fit for every gear in `gears` (the gears a test's samples used; gear 0 never has one), or any fit at all."""
  fit = coast_fit(ctx)
  return fit is not None and (gears is None or (bool(gears) and all(g in fit for g in gears)))


def _unfit(a, fit_gears) -> str:
  fit = coast_fit(a.ctx) or {}
  missing = sorted(g for g in fit_gears if g not in fit)
  return f" [provisional: coast model not fit ({COAST_CAL_TEST}) for gear(s) {missing or 'any'}]"


def fit_offset(a, key=None) -> float:
  """The fit's correction averaged over a segment analyzer's samples (by inferred gear); NaN if any sample was in a gear
  without a fit (gear 0: clutch in / neutral / no rpm) — then the coast estimate for those samples isn't known."""
  fit = coast_fit(a.ctx)
  counts = a.gear_counts(key)
  n = sum(counts.values())
  if fit is None or not n or any(g not in fit for g in counts):
    return math.nan
  return sum(fit[g] * c for g, c in counts.items()) / n


def _assist_sent(a) -> str:
  segs = a.segments if hasattr(a, "segments") else []
  n = sum(g.assist_n for g in segs)
  assist = f"{sum(g.assist_on for g in segs) / n:.0%}" if n else "n/a"
  sent = a.pooled("accel_sent") if segs else math.nan
  return f"brake-assist bit {assist}, sent accel {_fmt(sent)}"


def r_brake_strength(a: SegmentAnalyzer):
  """Task 12 D3 [D-03]: the share of the requested decel BEYOND coasting that the car delivered,
  (a_ego - a_coast) / (cmd - a_coast) - a raw a_ego / cmd counted engine braking and gravity as braking (a 3 % uphill in
  3rd "passed" with zero service braking).
  Task 13 B1: a_coast is the coast model FIT to the E0b capture (seed + per-gear correction); with no fit it is the seed
  and the result is CAPTURED (a 20 % seed error near the coast level read as 80 % "delivered" with zero service braking).
  The sent ACCEL_COMMAND (raw tap) is part of the verdict: PASS needs the request seen on the bus; a request that isn't
  there is a FAIL of its own. The brake-assist bit is reported (on this Civic it is the idle-stop-allow bit, not a brake
  flag - see CLAUDE.md), not scored."""
  if not a.segments:
    return None, ""
  cmd, acc, seed, sent = a.pooled("accel_cmd"), a.pooled("a_ego"), a.pooled("a_coast"), a.pooled("accel_sent")
  coast = seed + fit_offset(a)
  want = cmd - coast
  ratio = (acc - coast) / want if math.isfinite(coast) and want < -0.05 else math.nan
  detail = (f"delivered {_fmt(ratio * 100, 0)}% of the braking beyond coasting (cmd {cmd:.2f}, a_ego {acc:.2f}, " +
            f"coast fit {_fmt(coast)}, seed {_fmt(seed)}); {_assist_sent(a)}")
  used = set(a.gear_counts())   # Task 14 C1: gate on this test's own gears
  if not coast_calibrated(a.ctx, used):
    return CAPTURED, detail + _unfit(a, used)
  if not math.isfinite(sent):
    return CAPTURED, detail + " [sent accel not seen: can't confirm the request reached the bus]"
  if sent > cmd + SENT_TOL:
    return FAIL, detail + " - the braking request was not on the bus as commanded"
  if not math.isfinite(ratio):
    return CAPTURED, detail + " [no fitted coast estimate for these gears / request within coasting]"
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
  if not a.episodes:
    return None, ""
  # Task 12 D3 [D-13]: report coverage; a gear whose bites mostly weren't measured (e.g. its ratio is off, so the lock
  # never matched) can't pass on the others' numbers
  cover = {}
  for e in a.episodes:
    g = e["gear_after"]
    n, m = cover.get(g, (0, 0))
    cover[g] = (n + 1, m + (e["bite_lag"] is not None))
  low = sorted(g for g, (n, m) in cover.items() if m / n < COVERAGE_MIN)
  cov = " ".join(f"{g}:{m}/{n}" for g, (n, m) in sorted(cover.items()))
  if not lags:
    return (FAIL if a.complete else None), f"no bite measured in {len(a.episodes)} shifts (coverage {cov})"
  p90 = _pct(lags, 0.9)
  detail = (f"bite lag median {_pct(lags, 0.5):.2f}s p90 {p90:.2f}s (REENGAGE_DELAY {REENGAGE_DELAY}s); " +
            f"GEAR_MT->0 vs decouple median {_fmt(_pct(dec, 0.5))}s (+ = GEAR_MT lags); measured per gear {cov}")
  if low:
    return (FAIL if a.complete else None), detail + f"; < {COVERAGE_MIN:.0%} measured in gear(s) {low}"
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
  # Task 12 D3 [D-04]: a GEAR_MT "close" that re-opens within ~1 s is a leak - not just a sub-0.15 s flicker.
  # Task 13 B2: counted per clutch event (the leak id), whether the glitch came before or after the blip started; the
  # gas still pressed at the real release is NOT a leak (heel-toe: legal) - reported only
  gm_leaks = len({e["leak"] for e in a.episodes if e.get("leak") is not None})
  close_gas = sum(1 for e in a.episodes if e.get("close_gas") and e.get("leak") is None)
  gate_leaks = sum(1 for e in a.episodes if "shifting" in e["gate_states"] and
                   any(st != "shifting" for st in e["gate_states"][e["gate_states"].index("shifting"):]))
  gate_seen = any(e["gate_states"] for e in a.episodes)
  rates = [e["rpm_rate_max_open"] for e in a.episodes]
  detail = (f"{len(a.episodes)} blips: GEAR_MT flickers {flick}, GEAR_MT leaks {gm_leaks}, gate leaks {gate_leaks}" +
            f"{'' if gate_seen else ' (gate off: gate leak not exercised)'}; blip rpm rate max {max(rates):.0f} rpm/s" +
            (f"; released with gas {close_gas}x (heel-toe, ok)" if close_gas else ""))
  return (PASS if flick == 0 and gm_leaks == 0 and gate_leaks == 0 else FAIL), detail


def r_gate_accept(a: EpisodeAnalyzer):
  """Task 12 A2: longitudinal STAYS active through the press (braking available); the gate caps the throttle. Every
  press: the gate published 'paused', no throttle sent with the clutch in, stayed engaged, no ACC fault."""
  if not a.episodes:
    return None, ""
  bad = [e for e in a.episodes if not e.get("paused_open") or e["accel_out_max_open"] > 0.05 or not e["enabled_min"] or
         e["acc_fault"]]
  detail = f"{len(a.episodes) - len(bad)}/{len(a.episodes)} presses: throttle capped, stayed engaged, no ACC fault"
  if bad:
    e = bad[0]
    detail += (f"; first bad: paused={e.get('paused_open')} max out {_fmt(e['accel_out_max_open'])} " +
               f"engaged={e['enabled_min']} fault={e['acc_fault']}")
  health = _health(a)
  return (FAIL, detail + "; " + "; ".join(health)) if bad or health else (PASS, detail)


def r_gate_shifts(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  throttle = [e for e in a.episodes if e["accel_out_max_open"] > 0.05]
  jerky = [e for e in a.episodes if e["resume_jerk_max"] > RESUME_JERK_MAX]
  # Task 12 D3 [D-06]: every engaged shift must show the gate 'shifting' and openpilot must take back over within
  # REENGAGE_DELAY + 1 s of the clutch closing (a gate that never resumes is a FAIL, not a quiet pass)
  no_gate = [e for e in a.episodes if "shifting" not in e["gate_states"]]
  no_resume = [e for e in a.episodes if e["enabled_open"] and (e["resume_dt"] is None or e["resume_dt"] > RESUME_BOUND)]
  flares = a.detector.false_flares
  aligned = [e["gate_shift_dt"] for e in a.episodes if e["gate_shift_dt"] is not None]
  resumes = [e["resume_dt"] for e in a.episodes if e["resume_dt"] is not None]
  detail = (f"{len(a.episodes)} shifts: throttle-with-clutch-in {len(throttle)}, no 'shifting' {len(no_gate)}, " +
            f"late/no resume {len(no_resume)} (median {_fmt(_pct(resumes, 0.5))}s, bound {RESUME_BOUND:.1f}s), " +
            f"jerky resumes {len(jerky)} (max {max(e['resume_jerk_max'] for e in a.episodes):.1f} m/s^3), false flares " +
            f"{flares}; gate 'shifting' after GEAR_MT->0: median {_fmt(_pct(aligned, 0.5))}s")
  ok = not throttle and not jerky and not no_gate and not no_resume and flares == 0
  return (PASS if ok else FAIL), detail


def r_gate_revmatch(a: EpisodeAnalyzer):
  if not a.episodes and not a.extra_episodes:
    return None, ""
  # Task 12 A2: through every blip openpilot stays engaged, the gate is paused and the command is exactly 0 (the panda
  # allows nothing else while the gas is pressed)
  bad = [e for e in a.episodes if not e["enabled_min"] or not e.get("paused_open") or e.get("blip_cmd_abs", 0.) > 1e-3]
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
            f"{_fmt(a.lead_cmd_min if a.lead_s else math.nan)}; FCW x{a.fcw_edges}" +
            (" (FCW not exercised)" if a.fcw_edges == 0 else ""))
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
  parts, cmd_ok, coast_ok, sent_ok, known = [], True, True, True, True
  for key in a.keys:
    if not a.by_key(key):
      continue
    floor = float(key)
    out_min, sent_min = a.extreme("accel_out", min, key), a.extreme("accel_sent", min, key)
    exc = a.pooled("excess", key) - fit_offset(a, key)   # Task 13 B1: vs the FIT coast model (NaN without a fit)
    # the bus carried braking past the floor (raw tap), or the command did: no calibration needed to call either
    good_cmd = out_min >= -floor - 0.01 and not (sent_min < -floor - 0.01)
    good_coast = not (exc < -0.3)
    cmd_ok &= good_cmd
    coast_ok &= good_coast
    sent_ok &= math.isfinite(sent_min)
    known &= math.isfinite(exc)   # a gear without a fit (or no rpm) -> no coast estimate -> unknown, never "clean"
    parts.append(f"floor {key}: min out {out_min:.2f}, min sent {_fmt(sent_min)}, excess vs fit {_fmt(exc)} " +
                 f"{'✓' if good_cmd and good_coast else '×'}")
  if not parts:
    return None, ""
  detail = "; ".join(parts) + f"; {_assist_sent(a)}"
  if not cmd_ok:
    return FAIL, detail
  # Task 12 D3 [D-11] / Task 13 B1: "stayed off the brakes" is judged from a_ego vs the FIT coast model, and needs the
  # sent accel seen on the bus - provisional (CAPTURED) until both exist
  used = set(a.gear_counts())   # Task 14 C1: gate on this test's own gears
  if not coast_calibrated(a.ctx, used):
    return (CAPTURED if a.complete else None), detail + _unfit(a, used)
  if not sent_ok:
    return (CAPTURED if a.complete else None), detail + " [sent accel not seen on the bus]"
  if coast_ok and not known:
    return (CAPTURED if a.complete else None), detail + " [no fitted coast estimate for these samples' gears]"
  return (PASS if coast_ok and a.complete else FAIL if not coast_ok else None), detail


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
  """E0a / E0b characterization. Task 13 B1: when E0b (flat) completes, it FITS the coast model: per gear, the mean of
  a_ego - seed over its flat-coast samples, saved with the result (coast_fit) - what C1 / NSB1 / E1 then judge against."""
  parts = []
  for g in a.keys:
    if not a.by_key(g):
      continue
    parts.append(f"g{g} a {a.pooled('a_ego', g):.2f} (model {_fmt(a.pooled('a_coast', g))}, pitch {a.pooled('pitch', g):+.3f})")
  a.data.pop("coast_fit", None)
  if a.spec.id == COAST_CAL_TEST:
    # Task 14 C1: fit each gear whose own capture is done (its seconds collected), not only when every gear is
    gears = {}
    for g in a.keys:
      n = sum(seg.cnts.get("excess", 0) for seg in a.by_key(g))
      bias = a.pooled("excess", g)
      if a._live(g)[0] >= a.seconds and n >= FIT_MIN_SAMPLES and math.isfinite(bias):
        gears[str(g)] = {"bias": round(bias, 4), "n": n, "done": True}
    if gears:
      a.data["coast_fit"] = {"version": 2, "gears": gears}
      parts.append("fit " + " ".join(f"g{g} {v['bias']:+.2f}" for g, v in gears.items()))
  return (CAPTURED, "; ".join(parts)) if parts else (None, "")


def r_ebp_trims(a: EpisodeAnalyzer):
  if not a.episodes:
    return None, ""
  fit = coast_fit(a.ctx)
  sent = [e.get("sent_min", math.nan) for e in a.episodes]
  # Task 17: a command (or the sent accel, raw tap) below the clamp's published floor while it holds braking off is a
  # fault of the clamp - a FAIL of its own; braking beyond coasting (the fit coast model) is judged below
  hard = [e for e in a.episodes if e.get("under_floor", 0) > 0]
  # judged against the FIT coast model (the episode's gear); a gear without a fit -> unknown (provisional), not clean
  unknown = [e for e in a.episodes if fit is None or e.get("gear") not in fit]
  soft = [e for e in a.episodes if e not in unknown and e["excess_min"] - fit[e["gear"]] < -0.3]
  detail = (f"{len(a.episodes) - len(hard) - len(soft)}/{len(a.episodes)} clean; below the clamp floor in {len(hard)}; " +
            f"gears {','.join(str(e['gear']) for e in a.episodes[-5:])}; reasons {','.join(str(e['reason']) for e in a.episodes[-3:])}; " +
            f"min sent accel {_fmt(min(sent, default=math.nan))}, brake-assist seen {sum(e.get('assist', 0) for e in a.episodes)}x")
  if hard:
    return FAIL, detail
  if fit is None:   # Task 12 D3 [D-11] / Task 13 B1: provisional until the coast model is fit
    return (CAPTURED if a.complete else None), detail + f" [provisional: coast model not fit ({COAST_CAL_TEST})]"
  if soft:
    return FAIL, detail
  if unknown:
    return (CAPTURED if a.complete else None), detail + " [no fitted coast estimate for the episode's gear]"
  if not all(math.isfinite(v) for v in sent):
    return (CAPTURED if a.complete else None), detail + " [sent accel not seen on the bus]"
  return (PASS if a.complete else None), detail


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
  """S7 / S8: a marked capture window; the analysis runs offline on the full logs, every bus (S7: the volume rhythm,
  likely B-CAN; S8: a brake-pressure field)."""
  secs = sum(g.dur for g in a.segments)
  if secs <= 0.:
    return None, ""
  return CAPTURED, f"{secs:.0f}s window marked (test_marker start/stop) - analysed offline on the full logs"


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
