#!/usr/bin/env python3
"""Mutation check for the dev-test console (Task 12 Part D) - same runner as mutate.py, its own manifest.

  python tools/civic6mt/mutate_devtest.py [--only ID[,ID..]]     (dev-test branch only)
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutate import run_one

TC = "openpilot/sunnypilot/selfdrive/test_console"
R = f"{TC}/profiles/civic_6mt/results.py"
UI = "openpilot/selfdrive/ui/sunnypilot/mici/layouts"
T = [f"{TC}/tests/test_test_console.py", f"{TC}/tests/test_task12_console.py", f"{TC}/tests/test_sessions.py"]
SMOKE = [f"{TC}/tests/test_ui_smoke.py"]
T13 = [f"{TC}/tests/test_task13_console.py"]


# Task 14 C1 added a per-gear gate in front of the fitted estimate: the old D3 / B1 bugs remove both layers now
_C1_LAYERS = "\n".join([
  '  coast = seed + fit_offset(a)',
  '  want = cmd - coast',
  '  ratio = (acc - coast) / want if math.isfinite(coast) and want < -0.05 else math.nan',
  '  detail = (f"delivered {_fmt(ratio * 100, 0)}% of the braking beyond coasting (cmd {cmd:.2f}, a_ego {acc:.2f}, " +',
  '            f"coast fit {_fmt(coast)}, seed {_fmt(seed)}); {_assist_sent(a)}")',
  "  used = set(a.gear_counts())   # Task 14 C1: gate on this test's own gears",
  '  if not coast_calibrated(a.ctx, used):',
  '    return CAPTURED, detail + _unfit(a, used)',
  '',
])
_C1_NO_LAYERS = "\n".join([
  '  coast = seed',
  '  want = cmd - coast',
  '  ratio = (acc - coast) / want if math.isfinite(coast) and want < -0.05 else math.nan',
  '  detail = (f"delivered {_fmt(ratio * 100, 0)}% of the braking beyond coasting (cmd {cmd:.2f}, a_ego {acc:.2f}, " +',
  '            f"coast fit {_fmt(coast)}, seed {_fmt(seed)}); {_assist_sent(a)}")',
  '',
])
_NSB1_LAYERS = "\n".join([
  '    known &= math.isfinite(exc)   # a gear without a fit (or no rpm) -> no coast estimate -> unknown, never "clean"',
  '    parts.append(f"floor {key}: min out {out_min:.2f}, min sent {_fmt(sent_min)}, excess vs fit {_fmt(exc)} " +',
  '                 f"{\'✓\' if good_cmd and good_coast else \'×\'}")',
  '  if not parts:',
  '    return None, ""',
  '  detail = "; ".join(parts) + f"; {_assist_sent(a)}"',
  '  if not cmd_ok:',
  '    return FAIL, detail',
  '  # Task 12 D3 [D-11] / Task 13 B1: "stayed off the brakes" is judged from a_ego vs the FIT coast model, and needs the',
  '  # sent accel seen on the bus - provisional (CAPTURED) until both exist',
  "  used = set(a.gear_counts())   # Task 14 C1: gate on this test's own gears",
  '  if not coast_calibrated(a.ctx, used):',
  '    return (CAPTURED if a.complete else None), detail + _unfit(a, used)',
  '',
])
_NSB1_NO_LAYERS = "\n".join([
  '    known &= True   # a gear without a fit (or no rpm) -> no coast estimate -> unknown, never "clean"',
  '    parts.append(f"floor {key}: min out {out_min:.2f}, min sent {_fmt(sent_min)}, excess vs fit {_fmt(exc)} " +',
  '                 f"{\'✓\' if good_cmd and good_coast else \'×\'}")',
  '  if not parts:',
  '    return None, ""',
  '  detail = "; ".join(parts) + f"; {_assist_sent(a)}"',
  '  if not cmd_ok:',
  '    return FAIL, detail',
  '  # Task 12 D3 [D-11] / Task 13 B1: "stayed off the brakes" is judged from a_ego vs the FIT coast model, and needs the',
  '  # sent accel seen on the bus - provisional (CAPTURED) until both exist',
  "  used = set(a.gear_counts())   # Task 14 C1: gate on this test's own gears",
  '',
])


def m(mid, finding, file, old, new, tests=T):
  return {"id": mid, "finding": finding, "file": file, "old": old, "new": new, "tests": tests}


MUTATIONS = [
  m("d1_status_nan", "D1 D-01", f"{TC}/capture.py", '"rpm": _num(s.rpm, 0)', '"rpm": round(s.rpm)'),
  m("d1_results_unsanitized", "D1 D-17", f"{TC}/status_io.py",
    "  return {k: _typed(v, RESULT_TYPES) for k, v in d.items() if isinstance(k, str) and isinstance(v, dict)}", "  return d"),
  m("d1_guard_raises", "D1 D-17", f"{TC}/guard.py", "      except Exception:\n        log_once(site)\n        return default",
    "      except ZeroDivisionError:\n        log_once(site)\n        return default"),
  m("d1_guard_never_inert", "D1", f"{TC}/guard.py", "      if n >= self.MAX_CONSECUTIVE:", "      if False:"),
  m("d1_allowlist_uncapped", "D1", f"{TC}/sessions.py", "frozenset(profile.allowlist) & CORE_MAX_ALLOW if", "frozenset(profile.allowlist) if"),
  m("d1_selfdrived_watches_console", "D1 D-01", "openpilot/selfdrive/selfdrived/selfdrived.py",
    "{'mapd', 'test_consoled'}", "{'mapd', }"),
  m("d2_write_while_engaged", "D2 D-02", f"{TC}/sessions.py", "  if engaged:\n    return \"disengage first\"\n", ""),
  m("d2_write_while_rolling", "D2 D-02", f"{TC}/sessions.py", "  if onroad and not standstill:", "  if False:"),
  m("d2_tap_flips_satisfied", "D2 D-02", f"{TC}/sessions.py", "    return None if t.ok else t.want", "    return (not t.have) if t.ok else t.want"),
  m("d2_conditions_use_params", "D2 D-02", f"{TC}/conditions.py", "toggle_value(k, s, ctx) == want", "ctx.toggles.get(k, False) == want"),
  m("d2_tap_any_release", "D2 D-09", f"{UI}/dev_test_sessions.py",
    "    return action if rl.check_collision_point_rec(rl.Vector2(release.x, release.y), rect) else None", "    return action"),
  m("d3_pass_before_complete", "D3", f"{TC}/analyzers.py",
    "    return (None if status == PASS and not self.complete else status), detail", "    return status, detail"),
  m("d3_c1_raw_ratio", "D3 D-03", R, "  ratio = (acc - coast) / want if math.isfinite(coast) and want < -0.05 else math.nan",
    "  ratio = acc / cmd if cmd < -1e-3 else math.nan"),
  m("d3_c1_uncalibrated_pass", "D3 D-03 / T13 B1 / T14 C1", R, _C1_LAYERS, _C1_NO_LAYERS, T + T13),
  m("d3_s5_reopen_not_leak", "D3 D-04", f"{TC}/analyzers.py",
    '        ep["reopened"] = s.t - ep["t_close"] <= self.REOPEN_LEAK', '        ep["reopened"] = False'),
  m("d3_s5_close_gas_ok", "D3 D-04", f"{TC}/analyzers.py", '          ep["close_gas"] = self._gas_at_close', '          ep["close_gas"] = False'),
  m("d3_g2_no_resume_ok", "D3 D-06", R,
    '  no_resume = [e for e in a.episodes if e["enabled_open"] and (e["resume_dt"] is None or e["resume_dt"] > RESUME_BOUND)]',
    "  no_resume = []"),
  m("d3_g2_no_shifting_ok", "D3 D-06", R, '''  no_gate = [e for e in a.episodes if "shifting" not in e["gate_states"]]''', "  no_gate = []"),
  m("d3_s2_no_coverage", "D3 D-13", R, "  if low:\n", "  if False:\n"),
  m("d3_n1_fcw_note", "D3 D-12", R, '(" (FCW not exercised)" if a.fcw_edges == 0 else "")', '""'),
  m("d4_bit_hunt_uncached", "D4 D-07", f"{TC}/analyzers.py", '    return self._cached("edges", self._candidates)',
    "    return self._candidates()"),
  m("d4_signature_no_tail", "D4 D-19", f"{TC}/analyzers.py", "    if frac >= 1. and self.t_now - self.t_last_edge < self.TAIL:",
    "    if False:"),
  m("d4_prune_only_on_open", "D4 D-21", f"{TC}/capture.py",
    '    if s.t - self._t_pruned > PRUNE_PERIOD and (self.state == "capturing" or self.bg_runs):', "    if False:"),
  m("d4_tap_never_stale", "D4 D-16", f"{TC}/profiles/civic_6mt/tap.py", "    return t_ns > 0 and self.t_bus - t_ns < STALE_NS",
    "    return t_ns > 0"),
  m("d4_cancel_allowed", "D4 D-10", f"{TC}/profiles/civic_6mt/__init__.py",
    'allowed=("gapAdjustCruise", "accelCruise", "decelCruise", "lkas"),',
    'allowed=("gapAdjustCruise", "accelCruise", "decelCruise", "cancel", "mainCruise", "lkas"),'),
  m("d4_gap_long_allowed", "D4 D-10", f"{TC}/profiles/civic_6mt/__init__.py", 'modes={"gapAdjustCruise": ("triple", "short")},',
    'modes={"gapAdjustCruise": ("short", "triple", "long")},'),
  m("d4_name_unresolved", "D4 D-15", f"{TC}/button.py", "    self.name = self.buttons.resolve(name)",
    "    self.name = name if name else self.buttons.default"),
  m("d4_chip_over_alert", "D4 D-08", "openpilot/selfdrive/ui/sunnypilot/mici/onroad/test_console_chip.py",
    "or not self._poller.live(time.monotonic()) or alert_showing():", "or not self._poller.live(time.monotonic()):", SMOKE),
  m("d5_ui_reads_unsanitized", "D5 D-05", f"{TC}/status_io.py",
    "  return sanitize_state(read_json(os.path.join(console_dir(), STATE_FILE)))",
    "  return read_json(os.path.join(console_dir(), STATE_FILE))", SMOKE),
  m("d3_g1_old_semantics", "D3 / A2", R, 'bad = [e for e in a.episodes if not e.get("paused_open") or e["accel_out_max_open"] > 0.05 or',
    'bad = [e for e in a.episodes if e["accel_out_max_open"] > 0.05 or'),
  # --- Task 13 Part B ------------------------------------------------------------------------------------------------
  m("t13_b1_seed_not_fit", "T13 B1 D-03 D-12", R, "  coast = seed + fit_offset(a)\n", "  coast = seed\n", T13),
  m("t13_b1_sent_not_in_verdict", "T13 B1", R, "  if sent > cmd + SENT_TOL:\n", "  if False:\n", T13),
  m("t13_b1_sent_unseen_passes", "T13 B1", R,
    '  if not math.isfinite(sent):\n    return CAPTURED, detail + " [sent accel not seen',
    '  if False:\n    return CAPTURED, detail + " [sent accel not seen', T13),
  m("t13_b1_nsb_nan_is_clean", "T13 B1 / T14 C1", R, _NSB1_LAYERS, _NSB1_NO_LAYERS, T13),
  m("t13_b1_ebp_sent_ignored", "T13 B1", R,
    '          e.get("sent_min", math.nan) < a.ctx.ebp_coast_target - 0.01]', '          False]', T13),
  m("t13_b1_fit_not_saved", "T13 B1", R, '      a.data["coast_fit"] = {"version": 2, "gears": gears}\n', "", T13),
  m("t13_b2_pre_blip_unlinked", "T13 B2 D-04", f"{TC}/analyzers.py", '               "leak": self._leak_pending}',
    '               "leak": None}'),
  m("t13_b2_gas_at_release_is_leak", "T13 B2 D-04", R,
    '  gm_leaks = len({e["leak"] for e in a.episodes if e.get("leak") is not None})',
    '  gm_leaks = len({e["leak"] for e in a.episodes if e.get("leak") is not None}) + sum(bool(e.get("close_gas")) for e in a.episodes)'),
  m("t13_b3_dropped_still_passes", "T13 B3 review2 p3", f"{TC}/capture.py", "    if self.dropped:\n      if status == PASS:",
    "    if self.dropped:\n      if False:", T13),
  m("t13_b3_errors_not_counted", "T13 B3", f"{TC}/test_consoled.py",
    "  if guard.errors != before or guard.inert:", "  if False:", T13),
  m("t13_b3_bookkeeping_resets_streak", "T13 B3", f"{TC}/test_consoled.py",
    "(inert would never come)\n      console.note_dropped()\n", "(inert would never come)\n      guard.run('d', console.note_dropped)\n",
    T13),
  m("t13_b3_inert_status_skipped", "T13 B3", f"{TC}/guard.py", "    if self.inert and not force:", "    if self.inert:", T13),
  m("t13_b3_errors_not_shown", "T13 B3", "openpilot/selfdrive/ui/sunnypilot/mici/onroad/test_console_chip.py",
    "  if st.get(\"errors\"):\n", "  if False:\n", T13),
  m("t13_b4_guard_global_streak", "T13 B4", f"{TC}/guard.py", "      self.consecutive[site] = 0\n",
    "      self.consecutive = dict.fromkeys(self.consecutive, 0)\n", T13),
  m("t14_c1_all_six_gears", "T14 C1 N13-1", R,
    "  return fit is not None and (gears is None or (bool(gears) and all(g in fit for g in gears)))",
    "  return fit is not None and len(fit) == 6", T13),
  m("t14_c1_fit_only_when_complete", "T14 C1 N13-1", R, "      if a._live(g)[0] >= a.seconds and n >= FIT_MIN_SAMPLES",
    "      if a.complete and n >= FIT_MIN_SAMPLES", T13),
  m("t14_c1_undone_gear_counts", "T14 C1", R, ' and (e.get("done") is True or v1):', ":", T13),
  m("t13_b3_inert_chip_vanishes", "T13 B3", "openpilot/selfdrive/ui/sunnypilot/mici/onroad/test_console_chip.py",
    "  if st.get(\"inert\"):\n", "  if False:\n", T13),
]


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--only", default="")
  args = ap.parse_args()
  todo = [x for x in MUTATIONS if not args.only or x["id"] in args.only.split(",")]
  missed = 0
  for x in todo:
    caught, why = run_one(x)
    missed += not caught
    print(f"{'CAUGHT' if caught else 'MISSED'}  {x['id']:<34} {why}", flush=True)
  print(f"{len(todo) - missed}/{len(todo)} caught")
  sys.exit(1 if missed else 0)


if __name__ == "__main__":
  main()
