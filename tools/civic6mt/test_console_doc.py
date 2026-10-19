#!/usr/bin/env python3
"""Regenerate the generated part of docs/TEST_CONSOLE.md (the Civic 6MT profile's signal map, the session table and the
per-phase registry) from the dev-test profile, in place: everything from the "## Civic 6MT profile (generated" heading
(or, in an older file, "## Sessions (generated") to the end of the file is replaced.

  cd ~/civic-cruise/openpilot && source .venv/bin/activate && export PYTHONPATH="$(pwd)"   # on civic-6mt-dev-test
  python tools/civic6mt/test_console_doc.py ~/civic-cruise/docs/TEST_CONSOLE.md
"""
import sys

from openpilot.sunnypilot.selfdrive.test_console.conditions import Context, condition_label, toggle_label
from openpilot.sunnypilot.selfdrive.test_console.profile import ROLES
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.registry import PHASES, SESSIONS, TESTS, TESTS_BY_ID
from openpilot.sunnypilot.selfdrive.test_console.registry import CAL, OP_LONG_PARAM, Kind
from openpilot.sunnypilot.selfdrive.test_console.sessions import fmt_value, ui_writable

CTX = Context(profile=PROFILE)
UI_WRITABLE = ui_writable(PROFILE)
tests_in_phase = PROFILE.registry.tests_in_phase

MARK = "## Civic 6MT profile (generated"
OLD_MARK = "## Sessions (generated"
NOUN = {"shift": "shift", "handback": "handback", "stop_hold": "stop hold", "neutral_coast": "neutral coast",
        "backstop": "backstop", "ebp_suppress": "ebp suppress", "lead_approach": "lead approach", "brake_press": "brake press"}


def goal_text(t) -> str:
  g = t.goal
  if t.analyzer == "gear_raw_map":
    return f"{g['seconds_per_gear']:.0f} s rolling per GEAR_MT 1–6, {g['stop_seconds']:.0f} s stopped, reverse seen"
  if t.analyzer == "bit_hunt":
    return f"{g['presses']} clutch presses (rolling) or a {g['seconds']:.0f} s signature window (N = {g['signature_n']})"
  if t.analyzer == "non_adaptive":
    return f"{g['hold_seconds']:.0f} s hold + {g['lead_seconds']:.0f} s behind a slower lead"
  if t.analyzer == "episodes":
    out = f"{g['count']} {NOUN[g['detector']]}"
    if "filter" in g:
      out += f" [{g['filter'].replace('_', ' ')}]"
    if "need" in g:
      out += " (" + ", ".join(f"{k} ≥ {v}" for k, v in g["need"].items()) + ")"
    if "extra" in g:
      out += f" + {g['extra']['count']} {NOUN[g['extra']['detector']]}"
    if "count_by" in g:
      attr, table = g["count_by"]
      out += " (" + ", ".join(f"{v} with {attr.replace('nsb_', '')} {k}" for k, v in table.items()) + ")"
    return out
  if g.get("count"):
    return f"{g['count']} segments ≥ {g['min_segment']} s"
  if "bin" in g:
    keys = ", ".join(map(str, g["keys"]))
    out = f"{g['seconds']:.0f} s per {g['bin'].replace('_any', '').replace('_', ' ')} {keys}"
    if "required" in g:
      out += " (required " + ", ".join(map(str, g["required"])) + ")"
    return out
  return f"{g['seconds']:.0f} s"


def kind_text(t) -> str:
  out = t.kind.value + (" ★" if t.first_run else "") + (" (optional)" if t.optional else "")
  return out + (" • not background" if t.kind == Kind.TELEMETRY and not t.background else "")


def cond_text(t) -> str:
  conds = ", ".join(condition_label(c, CTX) for c in t.conditions) or "none (a marked window)"
  if t.toggles:
    conds += " + " + ", ".join(toggle_label(k, v) for k, v in t.toggles.items())
  return conds


def toggle_cell(pairs, optional=False) -> str:
  if not pairs:
    return "—"
  out = []
  for k, v in pairs:
    val = "/".join(fmt_value(c) for c in v) if optional else fmt_value(v)
    out.append(f"`{k}` {val}" + ("" if k in UI_WRITABLE else " (Developer panel; not settable here)"))
  return ", ".join(out)


def profile_section() -> list[str]:
  p, pt, b = PROFILE, PROFILE.powertrain, PROFILE.buttons
  lines = [f"{MARK} from `profiles/civic_6mt`)", "",
           f"Applies to: `{'`, `'.join(p.fingerprints)}` with a manual transmission (a CVT Civic gets no profile: inactive). "
           f"Powertrain: ratios {', '.join(f'{r:.3f}' for r in pt.gear_ratios)}, final drive {pt.final_drive:.2f}, tire radius "
           f"{pt.tire_radius:.3f} m, inference ±{pt.match_tol:.0%} (the console calls the features' own `engine_brake_model` "
           f"functions). Start button: `{b.default}` by default; `TestConsoleStartButton` may pick "
           f"{', '.join(f'`{n}`' for n in b.allowed)} ({b.note}). UI allowlist: "
           f"{', '.join(f'`{k}`' for k in p.allowlist)} + the console's own `TestConsole*` keys.", "",
           "| Role | Sample field | Log column | Name on this car | Source on this car |", "|---|---|---|---|---|"]
  for role, field in ROLES.items():
    lines.append(f"| `{role}` | {f'`{field}`' if field else '—'} | {f'`{p.column(field)}`' if field else '—'} | "
                 f"{p.signals.labels.get(role, '—')} | {p.signals.sources.get(role, '—')} |")
  return lines + [""]


def generate() -> str:
  lines = profile_section() + ["## Sessions (generated from the Civic 6MT profile's registry)", "",
           "The plan re-cut into ~30-minute outings. Every test belongs to exactly one session or to the **Cal** card. "
           "\"Requires\" is what the session page's ready check compares (✓ / set N); \"Offers\" are extra switches "
           "(A/B, floors) that never count against ready. Prereqs are soft hints (locked tiles), never a block.", "",
           "| Session | Name | Objective | Tests | Requires | Offers | After |",
           "|---|---|---|---|---|---|---|"]
  for s in SESSIONS:
    tests = ", ".join(f"{t}{' (optional)' if TESTS_BY_ID[t].optional else ''}" for t in s.test_ids)
    name = s.name + (" (progress card, background)" if s.card else "")
    lines.append(f"| {s.id} | {name} | {s.objective} | {tests} | {toggle_cell(s.required_toggles)} | "
                 f"{toggle_cell(s.optional_toggles, optional=True)} | {', '.join(s.prereqs) or '—'} |")
  lines += ["", f"`{OP_LONG_PARAM}` (openpilot longitudinal, which disables stock AEB on this car) is shown as a "
            "requirement but can only be set in the Developer panel.", "",
            "## The registry (generated from `profiles/civic_6mt/registry.py`)", "",
            "★ = first-run characterization. Conditions must all hold for a sample to count; toggles are checked by the "
            "daemon, never set by it (the session UI can set them — see above).", ""]
  for num, phase in PHASES.items():
    gates = f" — gates {phase.gates}" if phase.gates else ""
    lines += [f"**Phase {num} — {phase.title}** ({phase.setup}){gates}", "",
              "| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |",
              "|---|---|---|---|---|---|---|---|"]
    for t in tests_in_phase(num):
      lines.append(f"| {t.id} | {t.session} | {kind_text(t)} | {t.label} | {cond_text(t)} | {goal_text(t)} | "
                   f"{t.passes or 'characterization (CAPTURED)'} | {', '.join(t.prereqs) or '—'} |")
    lines.append("")
  assert sum(1 for line in lines if line.startswith("| ") and line.split(" | ")[0][2:] in TESTS_BY_ID) == len(TESTS)
  assert CAL in {s.id for s in SESSIONS}
  return "\n".join(lines)


if __name__ == "__main__":
  path = sys.argv[1]
  with open(path) as f:
    text = f.read()
  head = next(text.split(m)[0] for m in (MARK, OLD_MARK, "## The registry (generated") if m in text)
  with open(path, "w") as f:
    f.write(head + generate())
  print(f"regenerated {path}: {len(SESSIONS)} sessions, {len(TESTS)} tests")
