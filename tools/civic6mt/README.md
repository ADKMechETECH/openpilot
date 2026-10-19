# civic-6mt proof tools

Evidence for the claims in the civic-6mt docs, runnable from this checkout. Set up first:
`cd ~/civic-cruise/openpilot && source .venv/bin/activate && export PYTHONPATH="$(pwd)"`.
The replays download the process-replay CI segments, so they need network access.

| Tool | One-line usage | What it proves |
| --- | --- | --- |
| `selfref_replay.py` | `python tools/civic6mt/selfref_replay.py capture\|compare <plannerd\|controlsd> <dir>` | A process is bit-identical to a capture taken at an earlier commit, with toggles set by env (`NON_ADAPTIVE=1`, `NO_SERVICE_BRAKE=1`, …). It is used for the "OFF = 0 diffs" claims. |
| `clutch_reengage_replay.py` | `python tools/civic6mt/clutch_reengage_replay.py capture\|check <file>` | The clutch gate and takeover in controlsd, on a re-fingerprinted manual CI segment with synthetic shifts. Gate OFF is identical to the capture. Gate ON never drops `longActive`, never adds throttle while paused, and keeps the resume rate-bounded. The neutral coast ends in `coastTimeout` at 5 s. |
| `engine_brake_replay.py` | `python tools/civic6mt/engine_brake_replay.py` | EngineBrakePreferred (controlsd) and GearAwareAccel and the coast logger (plannerd), on the same segment. OFF is identical. ON changes only clamps outside every Rule-5 urgency frame, and FCW is identical. |
| `mutate.py` + `mutations.py` | `python tools/civic6mt/mutate.py [--only ID,..] [--list]` | Each listed mutation breaks one safety property, and the test named in the manifest must catch it. The tool exits 1 on any miss. Run it on a clean tree. |

`clutch_gate_replay.py` (Phase 3) is retired. Its claim, "a pause drops `longActive` for the shift", is no longer
true after Task 12 A2, and `clutch_reengage_replay.py` covers the gate now.
