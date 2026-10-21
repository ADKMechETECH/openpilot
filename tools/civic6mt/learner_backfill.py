#!/usr/bin/env python3
"""
Rebuild an EngineBrakeLearnedTable store (the current VERSION) from the learner's own samples.jsonl files.

Stage 3 changed the store to v2 (median histograms); a v1 store (mean cells) is dropped when plannerd starts, which
would lose the coasts already collected on the car. Their raw samples are still in /data/engine_brake_learn/samples.jsonl
(+ rotated .1 .. .10): this replays the "coast" rows through the same Cell, with the same in-coast settle (rows written
before the settle existed are grouped into coasts by their 10 Hz timestamps and gear). Engaged rows are never learned.

  tools/civic6mt/learner_backfill.py samples.jsonl [samples.jsonl.1 ...] [--cycles N] > store.json

Telemetry only, like the learner: nothing here reaches the clamp (APPLY_LEARNED stays off).
"""
import argparse
import json
import sys

from openpilot.sunnypilot.selfdrive.controls.lib import engine_brake_learning as L

RUN_GAP = 0.15   # s between 10 Hz samples of one unbroken coast


def build(paths: list[str], cycles: int = 0) -> tuple[dict, dict]:
  store = L.parse_store(None)
  store["cycles"] = cycles
  rows = []
  for path in paths:
    with open(path) as f:
      for line in f:
        try:
          r = json.loads(line)
        except ValueError:
          continue
        if isinstance(r, dict) and r.get("src") == "coast":
          rows.append(r)
  rows.sort(key=lambda r: r["t"])
  stats = {"rows": len(rows), "learned": 0, "settling": 0, "out_of_range": 0}
  run, prev = 0., None
  for r in rows:
    if prev is not None and (r["t"] - prev["t"] > RUN_GAP or r["gear"] != prev["gear"] or r["t"] < prev["t"]):
      run = 0.
    learned = r["learned"] if "learned" in r else run >= L.SETTLE_IN_COAST - 1e-6
    prev = r
    run += L.SAMPLE_PERIOD
    b = L.v_bin(float(r["v"]))
    if r.get("gear") not in L.GEARS or b is None:
      stats["out_of_range"] += 1
      continue
    if not learned:
      stats["settling"] += 1
      continue
    store["stats"][r["gear"]].setdefault(b, L.Cell()).add(float(r["d"]), r.get("e"))
    stats["learned"] += 1
  return store, stats


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("samples", nargs="+")
  ap.add_argument("--cycles", type=int, default=0, help="drive cycles already counted (the old store's 'cycles')")
  args = ap.parse_args()
  store, stats = build(args.samples, args.cycles)
  print(json.dumps(L.dump_store(store)))
  cells = {f"{g}/{L.V_MIDS[b]:g}": (c.n, round(c.median, 3)) for g, bins in store["stats"].items() for b, c in bins.items()}
  print(f"{stats}; cells (n, median): {cells}", file=sys.stderr)


if __name__ == "__main__":
  main()
