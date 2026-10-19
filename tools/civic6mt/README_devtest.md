# civic-6mt proof tools: dev-test only (the test console)

These tools need the `civic-6mt-dev-test` branch. The feature-branch tools are in `README.md`. Set up the same way:
`cd ~/civic-cruise/openpilot && source .venv/bin/activate && export PYTHONPATH="$(pwd)"`.

| Tool | One-line usage | What it proves |
| --- | --- | --- |
| `test_console_replay.py` | `python tools/civic6mt/test_console_replay.py replay <ref>` (`<ref>` = `selfref_replay.py capture` of controlsd + plannerd at the feature tip), `... isolation <ref> HONDA`, `... offline` | **Zero driving authority.** controlsd and plannerd replayed together with the real `test_consoled` (armed, background on, fed `carControlSP` / `longitudinalPlanSP`) show 0 diffs against the feature branch, and the console's own state.json must show it ran clean. `isolation` (Task 13 B4): a fault on every console frame leaves controlsd / plannerd at 0 diffs and the console reports INERT; selfdrived with `test_consoled` dead matches it alive while a non-ignored dead process doesn't. It FAILS if the guard or selfdrived's ignore entry is removed. `offline` runs the daemon's classes over a CI rlog end to end and reports the cost per frame. |
| `test_console_equiv.py` | `python tools/civic6mt/test_console_equiv.py <outdir>` | Before/after behavior-preservation harness for console refactors. Run it at two commits and `diff -r` the outputs. Task 12 changed console behavior on purpose, so expect differences across it. Since Task 13 B4 it exits 1 unless every armed test really captured (triple presses; published gate state with Task 12 A2 semantics). |
| `test_console_doc.py` | `python tools/civic6mt/test_console_doc.py ~/civic-cruise/docs/TEST_CONSOLE.md` | Regenerates the generated part of TEST_CONSOLE.md from the profile. |
| `dev_test_render.py` | `python tools/civic6mt/dev_test_render.py <outdir>` | Headless renders of the session UI (PNGs) plus injected-touch checks on the real widgets. Needs a display. The CI-style version is `test_console/tests/test_ui_smoke.py`. |
| `mutate_devtest.py` | `python tools/civic6mt/mutate_devtest.py [--only ID,..]` | The 45 console mutations (Task 12 Part D + Task 13 Part B). Each must be caught by its test. |
