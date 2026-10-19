#!/usr/bin/env python3
"""Mutation check for the civic-6mt safety code: apply each mutation in mutations.py (one at a time), run its tests,
expect at least one failure, restore the file. Prints CAUGHT/MISSED with the first failing test, exits 1 on any MISSED.

  cd ~/civic-cruise/openpilot && source .venv/bin/activate && export PYTHONPATH="$(pwd)"
  python tools/civic6mt/mutate.py [--only ID[,ID..]] [--list]

The file is always restored (try/finally); run it on a clean tree so a crash can't leave a mutation behind
(`git status` afterwards should be clean).
"""
import argparse
import os
import pathlib
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutations import MUTATIONS

ROOT = pathlib.Path(__file__).resolve().parents[2]   # the openpilot checkout


def run_one(m) -> tuple[bool, str]:
  path = ROOT / m["file"]
  src = path.read_text()
  n = src.count(m["old"])
  if n != 1:
    return False, f"PATTERN MATCHES {n}x (manifest out of date)"
  path.write_text(src.replace(m["old"], m["new"]))
  try:
    r = subprocess.run([sys.executable, "-m", "pytest", "-q", "-x", "-p", "no:cacheprovider", "-W", "ignore::RuntimeWarning",
                        *[str(ROOT / t) for t in m["tests"]]], capture_output=True, text=True, cwd=str(ROOT))
  finally:
    path.write_text(src)
  failed = [ln for ln in r.stdout.splitlines() if ln.startswith(("FAILED", "ERROR", "SUBFAILED"))]
  return r.returncode != 0, (failed[0].split(" - ")[0] if failed else r.stdout.strip().splitlines()[-1:])


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--only", default="")
  ap.add_argument("--list", action="store_true")
  args = ap.parse_args()
  todo = [m for m in MUTATIONS if not args.only or m["id"] in args.only.split(",")]
  if args.list:
    for m in todo:
      print(f"{m['id']:<34} {m['finding']:<26} {m['file']}")
    return
  missed = 0
  for m in todo:
    caught, why = run_one(m)
    missed += not caught
    print(f"{'CAUGHT' if caught else 'MISSED'}  {m['id']:<34} {why}", flush=True)
  print(f"{len(todo) - missed}/{len(todo)} caught")
  sys.exit(1 if missed else 0)


if __name__ == "__main__":
  main()
