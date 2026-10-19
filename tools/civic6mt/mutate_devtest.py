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


def m(mid, finding, file, old, new, tests=T):
  return {"id": mid, "finding": finding, "file": file, "old": old, "new": new, "tests": tests}


MUTATIONS = [
  m("d1_status_nan", "D1 D-01", f"{TC}/capture.py", '"rpm": _num(s.rpm, 0)', '"rpm": round(s.rpm)'),
  m("d1_results_unsanitized", "D1 D-17", f"{TC}/status_io.py",
    "  return {k: _typed(v, RESULT_TYPES) for k, v in d.items() if isinstance(k, str) and isinstance(v, dict)}", "  return d"),
  m("d1_guard_raises", "D1 D-17", f"{TC}/guard.py", "      except Exception:\n        log_once(site)\n        return default",
    "      except ZeroDivisionError:\n        log_once(site)\n        return default"),
  m("d1_guard_never_inert", "D1", f"{TC}/guard.py", "      if self.consecutive >= self.MAX_CONSECUTIVE:", "      if False:"),
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
  m("d3_c1_uncalibrated_pass", "D3 D-03", R, '''  if not coast_calibrated(a.ctx):
    return CAPTURED, detail + f" [provisional: {COAST_CAL_TEST} coast calibration not complete]"''', ""),
  m("d3_s5_reopen_not_leak", "D3 D-04", f"{TC}/analyzers.py",
    '        ep["reopened"] = s.t - ep["t_close"] <= self.REOPEN_LEAK', '        ep["reopened"] = False'),
  m("d3_s5_close_gas_ok", "D3 D-04", f"{TC}/analyzers.py", '          ep["close_gas"] = self._gas_at_close', '          ep["close_gas"] = False'),
  m("d3_g2_no_resume_ok", "D3 D-06", R,
    '  no_resume = [e for e in a.episodes if e["enabled_open"] and (e["resume_dt"] is None or e["resume_dt"] > RESUME_BOUND)]',
    "  no_resume = []"),
  m("d3_g2_no_shifting_ok", "D3 D-06", R, '''  no_gate = [e for e in a.episodes if "shifting" not in e["gate_states"]]''', "  no_gate = []"),
  m("d3_s2_no_coverage", "D3 D-13", R, "  if low:\n", "  if False:\n"),
  m("d3_n1_fcw_note", "D3 D-12", R, '(" (FCW not exercised)" if a.fcw_edges == 0 else "")', '""'),
  m("d3_g1_old_semantics", "D3 / A2", R, 'bad = [e for e in a.episodes if not e.get("paused_open") or e["accel_out_max_open"] > 0.05 or',
    'bad = [e for e in a.episodes if e["accel_out_max_open"] > 0.05 or'),
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
