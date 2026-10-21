# On-car test console (Tasks 7, 7b, 9, 10; NSB2 pulse scoring Task 11; Task 12 Part D; Task 13 Part B) — dev-test build only

**Branch:** `civic-6mt-dev-test` in `ADKMechETECH/openpilot` (off the feature branch `sp-honda-dev-202608`; opendbc
unchanged). **Flash the car from this branch. Never merge it back** — see `FORK-RESILIENCE.md` §8.

> ## ★ This is the first thing to run on the comma four
> The car is an unknown until it is characterized. Order on the C4: mount + install → flash the **dev-test** build →
> run the console's characterization tests (★ below: phases 0, 1, 2 and the phase 7 calibration captures E0c, E0a,
> E0b) → *then* the feature phases in order. Don't validate a feature before its prerequisites pass (the console
> hints, it never blocks — driver judgment).

## What it is (and is not)

Telemetry + a steering-wheel button read + a UI. **Zero driving authority.** `test_consoled` subscribes to
`carState`, `carControl`, `carOutput`, `longitudinalPlan`, `radarState`, `selfdriveState`, `pandaStates`,
`carControlSP` + `longitudinalPlanSP` (Task 12: what the features actually latched and are doing, as controlsd / plannerd
publish it), `logMessage` (the feature modules' cloudlog events) and — when the car's vehicle profile has a raw tap (on the
Bosch-radarless Civic: `GEAR_MT` value, `PEDAL_GAS`, the `POWERTRAIN_DATA` payload, `ACC_CONTROL` as sent) —
`can`/`sendcan`. On a car with **no vehicle profile** it is inactive: it subscribes to nothing and writes nothing. It publishes no cereal
message, sends no CAN, writes no Param, never touches a plan/actuator, and never changes a feature toggle (each test
lists the toggles it expects). Subscribing can't slow a publisher (msgq is a ring buffer).

**It can't take anything down (Task 12 D1):** selfdrived ignores `test_consoled` (a console crash is never a "process
not running" disengage); the daemon's params read, socket drain, per-frame body and status run under a guard (an
exception is logged once to the rlog and the daemon keeps running; 100 consecutive failures → inert for the drive, and
`state.json` says so); every dev-test UI render / input / callback path and the onroad chip are guarded the same way
(the ui process never dies from console code); `state.json` / `results.json` are type-validated wherever they are read
(a truncated or hand-edited file can't crash a reader). **Caught errors are not silent (Task 13 B3):** `state.json`
carries the error count; the onroad chip adds "! N console errors, M frames lost" and the detail page shows it; a run that
lost frames to a caught error never PASSes (its PASS becomes CAPTURED "[provisional: N frame(s) lost to console
errors]", and `results.json` keeps `dropped`); once inert the daemon keeps writing a minimal status, so the chip reads
"test console INERT — N errors — see the rlog; results provisional" (even with nothing armed) instead of vanishing.

**The UI may set feature toggles (Task 9 — a deliberate, bounded widening; the daemon is unchanged and still writes
nothing).** The session page's switches write through one function, `dev_test_params.write_param`, which refuses any
key outside the active vehicle profile's allowlist — capped by the core's maximum allowlist (`sessions.CORE_MAX_ALLOW`,
the fork's longitudinal feature Params + DOA) and denylist (`CORE_DENY`: `AlphaLongitudinalEnabled`, `ExperimentalMode`,
`OpenpilotEnabledToggle`, personality, lateral tuning, …), so a profile can narrow but never widen — plus the
`TestConsole*` keys (`sessions.ui_writable(profile)`); on
the Civic 6MT: `NonAdaptiveLongitudinal`, `NoServiceBrakeHold`, `NoServiceBrakeDecelFloor`,
`NoServiceBrakeOverspeedBackstop`, `NoServiceBrakeOverspeedMargin`, `NoServiceBrakeOverspeedMode` (Task 11),
`ClutchShiftGate`, `ClutchShiftGateGasReleaseHint`, `ClutchReengageSmart`, `EngineBrakePreferred`, `GearAwareAccel`,
`DisengageOnAccelerator`. No profile -> only the `TestConsole*` keys. Everything else is rejected — notably `AlphaLongitudinalEnabled` (openpilot longitudinal, which disables stock AEB on this car): the
session pages show it as a requirement but you set it in the Developer panel. The console doesn't actuate anything: it
changes config that the separately validated, default-OFF, gated features read. **Since Task 12 D2 a feature Param is
written only while openpilot is disengaged AND the car is stopped (or offroad)** — anything else is refused with the
reason on the page ("disengage first" / "stop the car first"); the `TestConsole*` keys stay writable. The features latch
only while openpilot is disengaged (Task 12 B3), so a write applies at the next engagement; onroad the page says "set
when disengaged + stopped". A satisfied requirement is not flipped off by a tap. The test conditions check the value the
feature actually **latched** (published by controlsd / plannerd), not the Param, wherever one is published — a Param
changed while engaged, or a gate that refused to enable, shows as not met. `DisengageOnAccelerator` is read live by
selfdrived.

Proven by: structural tests (no `PubMaster`/`pub_sock`/`send`/Params writes in the daemon; no direct Param write or
Param-bound widget anywhere in the UI files — `write_param` is the single writer — and `write_param` rejects
non-allowlisted keys) and a **multi-process process replay** of controlsd and plannerd with the real `test_consoled`
running armed + background on all four CI segments — **0 diffs** vs a capture at the feature branch
(`tools/civic6mt/test_console_replay.py replay` in the openpilot fork; re-run for Tasks 9, 11, 10, 12 and 13; since Task 10
the replay forces the Civic profile, as the CI segments aren't 6MT Civics; the no-profile path subscribes to nothing, so
it is covered by a unit test instead of a replay). *(Task 13 B4 correction: the Task 12 note said the daemon was "now also
reading `carControlSP` / `longitudinalPlanSP`" in that replay — the daemon subscribed, but the replay's console config
didn't feed those topics. Since B4 it does, and the replay also fails unless the console's own `state.json` shows it ran
clean (armed, frames processed, 0 caught errors, not inert) — "fails if test_consoled dies" proved nothing once the
guard caught everything. `isolation` mode: a fault injected into every console frame must leave controlsd / plannerd
at 0 diffs with the console reporting INERT, and selfdrived with `test_consoled` reported dead must match it alive while
a non-ignored dead process must not. Results below, "Task 13 run".)* Task 12 D5: `tests/test_ui_smoke.py` renders the whole UI (every session
page, tapping every switch / pill / ARM / how, the phase view, detail pages, the chip) with well-formed and malformed
files, onroad and offroad: nothing raises and the guard swallows nothing. `tools/civic6mt/mutate_devtest.py`: 48
console mutations (D1–D5 + Task 13 B1–B4 + Task 14 C1), all caught.

**Task 13 run (2026-10-19, PC):** `replay` 0 diffs controlsd + plannerd on all 4 segments, console fed the published
state and clean (armed, 0 errors, not inert); `isolation` (HONDA) OK — fault on every console frame: controlsd / plannerd 0
diffs, console INERT after 100 errors; selfdrived with `test_consoled` dead = alive, control (non-ignored dead process)
23 585 diffs; **and it FAILS with isolation removed** (no ignore entry: 23 585 diffs; no guard: the console dies, "containment
broken"). `offline` OK; `test_console_equiv` every armed test captured (start marker + rows); `dev_test_render` PASS. Found
by the run and fixed: the guard's failure streak was global (a succeeding drain reset a failing frame, so it never went
inert) — now per site; the isolation check's dead process was erased by the replay harness's `managerState` migration.

## Architecture: vehicle-agnostic core + per-vehicle profile (Task 10)

The console is a **core** that knows no car plus one **vehicle profile** per car, selected by fingerprint. It lives only
on the dev-test overlay branch, which tracks production (feature → dev-test); production never has it.

| Core (`sunnypilot/selfdrive/test_console/*.py`, the dev-test UI) | Profile (`…/test_console/profiles/<car>/`) |
|---|---|
| capture state machine, markers, capture files, results (`capture.py`) | **signal map** — how the car provides each named role (`SignalMap`: clutch / rpm readers, the raw-CAN tap, display names, log column names) |
| detectors + analyzers: shifts, handbacks, stops, neutral coasts, the NSB backstop/pulse, EBP trims, lead approaches, brake presses, raw-gear map, payload bit hunt (`analyzers.py`) | **powertrain** — gear ratios, final drive, tire radius, inference tolerance (optionally the car's own inference/coast functions) |
| the registry / session engine (`registry.py`: `TestSpec`, `Session`, `Registry`; `sessions.py`), conditions over the roles (`conditions.py`) | **button map** — the start/stop button (default + allowed) and its native side effects |
| the start button's gestures (`button.py`), file IPC (`status_io.py`), the daemon (`test_consoled.py`) | **registry** — the tests (conditions over the roles), phases, sessions, required toggles, and **results** (the plan's pass rules and wording) |
| the UI framework: session home/pages, phase view, detail page, onroad chip; the single writer `write_param` | **allowlist** — the feature Params the UI may set for this car / fork |
| the non-actuating guarantee (structural tests + 0-diff replay) | **applicability** — `applies(CP)` + the fingerprints it serves |

The core reads vehicle signals **only through the named roles** (`profile.ROLES`: `vehicle_speed`, `gas`, `brake`,
`clutch_coupled`, `engine_rpm`, `gear` (inferred), `gear_raw`, `gas_raw`, `raw_payload`, `accel_command_sent`,
`brake_request_sent`, `cruise_buttons`, `start_button`) — a test (`tests/test_profiles.py`) fails if a core file names
a platform (`honda`, `civic`, `GEAR_MT`, `0x1DD`, `gapAdjustCruise`, a CAN id, …). Selection
(`profile.select_profile`): the first registered profile whose `applies(CP)` is true; none → inactive, and the UI shows
"no test-console profile for this platform (FINGERPRINT)" and can arm and set nothing. The daemon re-checks
`CarParams` every second and re-selects on a change. `TEST_CONSOLE_PROFILE=<name>` forces one (replays/bench only).

The Civic's outputs were unchanged by the Task 10 split (Task 12 then changed some on purpose: the published gate state,
the verdict rules below): `tools/civic6mt/test_console_equiv.py` drove the console before and after over
the same inputs (the HONDA CI segment re-labelled a 6MT + a scripted 6MT drive, every test armed, background on, NSB2 in
both modes) — markers, results, capture files, status snapshots and every Sample identical in all 32 runs; every
session page, the phase view and a detail page render pixel-identical. The Civic keeps its log column names
(`gear_mt`, `raw_17c`, `pedal_gas`) through its profile's column map, so old captures stay re-scorable.

### Adding a car = writing a profile

1. `profiles/<car>/__init__.py`: a `VehicleProfile` — `applies(CP)` (fingerprint, transmission), a `SignalMap` (how
   this car gives clutch / rpm; a raw tap class with `available`, `update(can, sendcan)`, `fill(sample)` if it has raw
   signals worth tapping; labels for the UI; column names only if you need legacy spellings), a `Powertrain` (ratios,
   final drive, tire radius, tolerance — or point it at the car's own functions), a `ButtonMap`, the allowlist, the
   watched toggles.
2. `profiles/<car>/registry.py`: the plan — `TestSpec` rows (conditions from `conditions.CONDITIONS` or your own),
   phases, sessions → `Registry(...)`. Reuse the Civic's rows/results where the feature tests apply unchanged.
3. `profiles/<car>/results.py` (if it needs its own pass rules or wording); the core's generic `baseline` is always there.
4. Add it to `profiles.PROFILES`. Nothing in the core changes. `tests/stub_profile.py` is a complete minimal example
   (a test-only 5-speed with two tests) that the test suite runs through the unchanged core.

## Using it — the session view (default)

Task 16 rebuilt these screens for the car: they are the **stock mici settings components** (a sideways-scrolling row of
`Big*` cards, the same 402×180 cards as stock's settings), **one thing per card**, white text on dark, 44 px titles and
32 px values (nothing under 24 px). Swipe sideways through a page's cards; the back gesture returns, as in stock settings.
Read it at a stoplight, not while driving.

Settings → **dev test** opens the **session home**:
- a status card (live / offroad, the armed test and its state, the next recommended session);
- **one card per session** (the ~30-minute sessions, table below, and **Calibration**): `id  name`, then `✓ done`,
  `n/m tests` (+ FAIL count), `after 3, Cal` (the sessions recommended first; a hint, never a block) or the calibration
  progress; the recommended next one starts with `▶ next`;
- **phase view** — the original phase-grouped list (background telemetry and start-press settings live there).

Tap a session card → its **session page**:
- the session's name and its toggle status in words (`✓ toggles ready` / `2 toggles to set`; onroad: "set them
  disengaged + stopped"), then the **objective** on its own card;
- **one card per toggle**: the name, the stock on/off switch (a value card for multi-value settings: tap = the next
  choice — session 8's NSB floor 0.3 ↔ 0.0 and overspeed mode `maintain` ↔ `pulse`), and a line saying the current
  value and what the session needs (`now ON • needs OFF`, `now OFF • ✓ ready`, `now OFF • optional`). A tap sets it
  to the requirement; a tap on a satisfied one does nothing and says `already set` (Task 12 D2); writes happen only
  while disengaged and stopped — a refusal is shown on the card. `op long` is shown but not settable here (Developer);
  the switch always shows the Param's real value;
- **one card per test**: `id  name` and its state (`✓ PASS`, `× FAIL`, `✓ captured`, `• partial n/m`, `○ armed`,
  `● REC` + progress). Tap → the **test detail page**: the ARM switch, live status, conditions (failing first), result,
  the start button, prereqs, how-to cards and the pass rule;
- the **start-button status** in large monospace: "ARM G2, then triple-press gap to START" → "armed • …" →
  **STOP • ● REC** + progress while capturing → PAUSED / PASS / FAIL / CAPTURED.
- A swipe never fires a card (stock's scroller invalidates touches while it scrolls — Task 12 D-09, now checked on the
  real widgets with a real swipe).

The **onroad chip** (only while a test is armed and the daemon is live; never over an alert) is 28 px monospace on an
opaque background.

## Using it — the phase view (Task 7, secondary)

1. Tests are grouped by phase in the recommended order; each button shows
   its status (`not run` / `partial …` / `PASS` / `FAIL` / `CAPTURED`, `• prereqs ×` when a prerequisite isn't done,
   `• first run` for the ★ set).
2. Open a test: how-to cards, the toggles it expects, the pass criterion, live conditions (✓/×), the prereq hint.
   Turn **armed** on (stopped, or via a passenger). One test is armed at a time.
3. Drive into the maneuver and press the **gap/distance button**: capture starts (onroad chip turns red `● REC`). The
   chip shows progress and the first unmet condition. Press again to pause; again to resume (progress is kept). It
   completes by itself at the goal (chip turns green/blue/orange: `PASS` / `CAPTURED` / `FAIL`). A press after
   completion starts a fresh run.
4. **Background telemetry** (panel toggle, default off): captures the telemetry tests opportunistically on any drive
   without arming (never the feature tests, and never the S6/S7 signature hunts — they need you to perform the rhythm).

## Signature hunts (S6, S7 — Task 7b)

Both hunt an unknown bit on an unknown bus, so you press a distinctive, asymmetric rhythm that pops out of bus chatter
in cabana, parked & idling (bus quiet), with the count **N** identifying the input:
`~5 s still • N quick (<1 s apart) • ~5 s pause • N slow (~2 s apart) • ~5 s pause • N quick • ~5 s still` —
**clutch = 3, volume-up = 4, volume-down = 5.** Brief taps (holding volume auto-repeats and smears the pattern).
- **S6 clutch-bit hunt** (N = 3, in neutral): scored automatically on `POWERTRAIN_DATA` (0x17C, the visible bus the
  console taps) — any bit whose toggles form 3 groups of 2N edges is reported as `DBC bit N=3`. Rolling in gear still
  works too (presses scored against GEAR_MT → 0). Completes at 9 GEAR_MT presses or a 45 s window.
- **S7 volume-button hunt** (N = 4, then N = 5): a 90 s marked window only. Honda audio buttons are almost certainly on
  **B-CAN**, which the harness doesn't tap, so they likely show on no logged bus; scrub every bus in cabana between the
  start/stop markers. If a bit does change in that rhythm, note its ID + bit: it can replace the gap button as
  `TestConsoleStartButton` (a carState button event + the name in the Civic profile's `ButtonMap.allowed`). No automatic multi-bus scan (it would mean parsing
  every CAN frame on the C4 for a likely-absent signal).

**Start button** (`TestConsoleStartButton`, default = the profile's default, on the Civic `gapAdjustCruise`; a name the
profile doesn't allow falls back to it, and the resolved name is what is stored and shown — Task 12 D-15; allowed on the
Civic: gap, accel/decel cruise, LKAS — not cancel / main cruise, whose native function ends the test drive's
engagement; `TestConsoleStartPress`). Each button's native side effect is shown on the chip and the detail page (D-10):
- `triple` (**recommended on the gap button**): three presses within 1.5 s — three personality steps = net unchanged.
- `short` (the Param's default): one press. Side effect: sunnypilot steps the following-distance personality on every
  release — matters only with lead following (adaptive mode).
- `long`: ≥ 0.5 s hold — **not allowed on the gap button** (with openpilot long it also toggles Experimental mode); the
  selector offers only the modes the chosen button allows.
The panel shows "seen N×" for the button: confirm it decodes (`SCM_BUTTONS.CRUISE_SETTING`) before relying on it.

## Files (on device)

| Path | What |
|---|---|
| `/data/test_console/state.json` | live status (~5 Hz), read by the UI |
| `/data/test_console/results.json` | last result per test; persists across drives; drives the prereq hints |
| `/data/test_console/markers.jsonl` (+ rotations, 1 MB × 5) | `arm` / `start` / `sample` (milestones) / `stop` / `complete` / `abort` / `disarm`, with test id, phase, progress, result, key signals, monotonic + wall time |
| `/data/test_console/captures/*.jsonl` | one file per capture window: header (`fields`, rate) + one row per qualifying sample; ≤ 20 MB/file, ≤ 300 MB total (oldest pruned when a file opens and every 60 s while capturing — Task 12 D-21) |
| rlog `logMessage` | the same markers as cloudlog `test_marker` events — line up with cabana/plotjuggler |

Params (dev-test `params_keys.h` only): `TestConsoleSelected` (string), `TestConsoleArmed` (bool, cleared on manager
start), `TestConsoleBackground` (bool), `TestConsoleStartButton` (string), `TestConsoleStartPress` (string).

## Code map (openpilot, `civic-6mt-dev-test`)

| File | Role |
|---|---|
| `sunnypilot/selfdrive/test_console/profile.py` | core: the profile interface (`VehicleProfile`, `SignalMap`, `Powertrain`, `ButtonMap`, the named `ROLES`), `select_profile` |
| `…/profiles/__init__.py` | `PROFILES`: the registered cars |
| `…/profiles/civic_6mt/` | the Civic 6MT profile: `__init__.py` (signal map, powertrain = `engine_brake_model`, buttons, allowlist, applicability), `registry.py` (the plan: one row per test — add a test = add a row; phases; sessions), `results.py` (pass rules), `tap.py` (Honda raw-CAN tap) |
| `…/registry.py` | core: `TestSpec`, `Phase`, `Session`, `Registry` (the plan's shape + lookups) |
| `…/conditions.py` | core: `Context` (profile + the fork's feature params), named per-sample predicates over the roles + bin functions |
| `…/signals.py` | core: `Sample` (one per carState frame, role fields), `SampleBuilder` (through the profile), `LogEventTap` |
| `…/analyzers.py` | core: `SegmentAnalyzer`, `EpisodeAnalyzer` + detectors (shift, handback, stop-hold, neutral coast, backstop/pulse, EBP trim, lead approach, brake press), raw-gear map / payload bit hunt / non-adaptive analyzers, generic `baseline` result |
| `…/capture.py` | capture state machine, markers, capture files, results, background runs |
| `…/button.py` | start/stop gesture |
| `…/status_io.py` | file IPC + UI poller; `sanitize_state` / `sanitize_results` (Task 12 D1) |
| `…/guard.py` | Task 12 D1: `guarded` (UI paths) and `Guard` (the daemon loop) - log once, never raise; per-site failure streak, `force` for the status writer (Task 13) |
| `…/test_consoled.py` | the daemon (`only_onroad`, default priority) |
| `…/sessions.py` | session view state: tile status, ready check, pills, chip, calibration card; `ui_writable(profile)` (capped by `CORE_MAX_ALLOW` / `CORE_DENY`) and `write_refusal` (pure, no I/O) |
| `selfdrive/ui/sunnypilot/mici/layouts/dev_test_sessions.py` | session home grid + swipeable session pages (default entry) |
| `…/layouts/dev_test_params.py` | `write_param`: the UI's only Param write path, allowlist-enforced; the UI's profile (from `CarParamsPersistent`) |
| `selfdrive/ui/sunnypilot/mici/layouts/dev_test.py`, `…/onroad/test_console_chip.py` | phase view + per-test page + onroad chip (comma four "mici" UI — the fork's UI is Python/raylib, not Qt) |

## Notes and limits (TODO(on-car))

- **Brake-active indicator:** no confirmed radarless "service brake applied" signal. C1/C2/N2/NSB1/E1 use
  `excess = aEgo − coast estimate` plus the sent `COMPUTER_BRAKE_ASSIST` bit and the sent `ACCEL_COMMAND`. **Since Task 12
  D3 the console enforces it:** C1 scores `(aEgo − a_coast) / (cmd − a_coast)` (the braking delivered beyond coasting — a
  raw aEgo/cmd let a 3 % uphill in 3rd "pass" with zero service braking) and NSB1 / E1 judge "stayed off the brakes"
  against the same model. **Task 13 B1: against a FIT model, not the seed.** "E0b complete" used to count as calibrated,
  but E0b only captured — with a seed 20 % low on coast decel and zero service braking C1 still PASSed. Now E0b's
  completion **fits** a per-gear correction (mean of `aEgo − seed` over its flat coasts, ≥ 300 samples per gear, saved in
  `results.json` as `coast_fit`); C1 / NSB1 / E1 score against seed + correction (weighted by the samples' inferred gears)
  and stay **CAPTURED (provisional) until a fit exists for the gear(s) the test's own samples used** (Task 14 C1: per
  gear — E0b fits each gear once that gear's own 10 s is in, whether or not E0b is complete, so one sparse gear no longer
  blocks verdicts in the others); a sample with no fitted estimate (gear 0) is unknown, never "clean". The **sent accel is part of the verdict**: C1 PASS needs the request seen on the bus (missing
  from the bus = FAIL; tap not live = CAPTURED); NSB1 / E1 FAIL on a sent accel past the floor / coast target and stay
  CAPTURED while it isn't seen. The brake-assist bit stays reported (on this Civic it's the idle-stop bit). A command
  that breaks the NSB floor still FAILs at once. The raw rows are kept to re-score; re-arming E0b replaces the fit.
- **Verdict rules (Task 12 D3):** no rule PASSes before its test is complete (an early FAIL is fine). S2 reports
  per-gear coverage and needs ≥ 80 % of each gear's shifts measured. S5: a GEAR_MT close that re-opens within 1 s is a
  leak — **Task 13 B2:** counted once per clutch event whether the glitch came before or after the blip started (a
  pre-blip glitch used to split the event and the blipped half looked clean); the gas still pressed at the real release
  is no longer a leak (a clean heel-toe can be on the gas as the clutch comes up — reported only). N1's detail says
  "FCW not exercised" when no FCW happened.
- **Gate tests after Task 12 A2** (a shift no longer drops longitudinal): the shift detector reads the gate's
  *published* state (`carControlSP.manualLongitudinal.gatePaused` / `gateState`). G1 = paused + no throttle with the
  clutch in + still engaged + no ACC fault (`long_active` staying on is now correct). G2 = every engaged shift shows
  'shifting', openpilot resumes within `REENGAGE_DELAY` + 1 s, no throttle with the clutch in, smooth resume, no false
  flares. G3 = paused and an exactly-0 command through every blip, and the brake still disengages.
- **Raw roles go stale (Task 12 D-16):** a raw role whose CAN source is quiet for 0.2 s reads as unavailable (−1 / NaN),
  not its last value (e.g. `accel_sent` once openpilot stops sending ACC_CONTROL).
- **S6 signature window (D-19):** completes on the press count plus a 1 s tail; the bit scorers are cached (re-run on new
  edges or a window close, ≤ 0.5 Hz — D-07).
- **Fork-specific reads still in the core (Task 12 D-18, listed instead of moved):** `test_consoled.read_context` reads
  `NoServiceBrakeDecelFloor` / `…Margin` / `…Mode` + the pulse params (via `no_service_brake.read_pulse_params`),
  `ManualLowSpeedHandbackRamp`, `EngineBrakeCoastTarget`; `signals.WATCHED_EVENTS` names the fork's cloudlog events
  (`clutch_shift_gate`, `engine_brake_preferred`, `no_service_brake_backstop` / `_pulse`, `manual_low_speed_handback`,
  `engine_brake_coast`) and the `manualLowSpeedHandback` / `noServiceBrakeOverspeed` alerts; `signals.published_state`
  maps the fork's `carControlSP.manualLongitudinal` / `longitudinalPlanSP.features` / `.noServiceBrake` fields;
  `sessions.CORE_MAX_ALLOW` lists the fork's Params. A second fork would move these behind the profile.
- **Clutch pedal timing:** there is no pedal signal (that's what S6 hunts for). S2 measures GEAR_MT against the
  engine/wheel ratio (decouple on clutch-in, ratio lock = bite on clutch-out), which is what sizes `REENGAGE_DELAY`.
- **Neutral vs clutch held** (H3) can't be told apart in data unless `GEAR_MT` differs — that is the question; do the
  maneuver deliberately and use the start button to mark the window.
- Thresholds marked provisional (resume jerk 3 m/s³, handback jerk 2.5 m/s³, rollback 0.3 m/s, hill 0.03 rad).
- CPU: ~110 µs/frame on the PC at 100 Hz with a test armed + background on; check `top` on the C4 (default
  priority, never competes with controls). Raw CAN parsing runs only while a test is selected or background is on.
- Progress is per drive (a capture's analyzer lives in memory); results persist.
- **NSB2 in pulse mode (Task 11):** scored per brake pulse from the planner's `no_service_brake_pulse` phase events —
  start within 0.5 m/s of set + trigger, release within 0.5 m/s of set + release, command within [pulse decel, 0] while
  bounded, ≥ 1 s of coast on the clamp between pulses, no pulse > 20 s (provisional: a brake hold); needs 2 pulses.
  An escalation ("Overspeed: Take Over") is flagged in the result and must show exactly one alert. Run `maintain`
  first (the original backstop), then set the mode to `pulse` in session 8 while stopped for the A/B.
- Session view (Task 9): renders verified headless at 536×240 (`tools/civic6mt/dev_test_render.py`); confirm on the
  device that the layout reads at the real screen and that switch taps set the toggles (stopped, disengaged); regroup
  sessions if a 30-minute outing proves too full. Tables below are generated:
  `python tools/civic6mt/test_console_doc.py ~/civic-cruise/docs/TEST_CONSOLE.md`.

## Civic 6MT profile (generated from `profiles/civic_6mt`)

Applies to: `HONDA_CIVIC_2022` with a manual transmission (a CVT Civic gets no profile: inactive). Powertrain: ratios 3.643, 2.080, 1.361, 1.024, 0.830, 0.686, final drive 4.10, tire radius 0.315 m, inference ±8% (the console calls the features' own `engine_brake_model` functions). Start button: `gapAdjustCruise` by default; `TestConsoleStartButton` may pick `gapAdjustCruise`, `accelCruise`, `decelCruise`, `lkas` (distance button: a short press also steps the following-distance personality on release; a >= 0.5 s hold toggles Experimental mode (sunnypilot's distance-button long press)). UI allowlist: `NonAdaptiveLongitudinal`, `NoServiceBrakeHold`, `NoServiceBrakeDecelFloor`, `NoServiceBrakeOverspeedBackstop`, `NoServiceBrakeOverspeedMargin`, `NoServiceBrakeOverspeedMode`, `ClutchShiftGate`, `ClutchShiftGateGasReleaseHint`, `ClutchReengageSmart`, `EngineBrakePreferred`, `GearAwareAccel`, `DisengageOnAccelerator` + the console's own `TestConsole*` keys.

| Role | Sample field | Log column | Name on this car | Source on this car |
|---|---|---|---|---|
| `vehicle_speed` | `v_ego` | `v_ego` | — | carState.vEgo |
| `gas` | `gas` | `gas` | — | carState.gasPressed (POWERTRAIN_DATA.PEDAL_GAS) |
| `brake` | `brake` | `brake` | — | carState.brakePressed |
| `clutch_coupled` | `clutch` | `clutch` | — | carState.deprecated.clutchPressed (GAS_PEDAL_2 clutch-up switch, GEAR_MT 0, phantom gear), inverted; also the ratio lock |
| `engine_rpm` | `rpm` | `rpm` | — | carState.deprecated.engineRpm (POWERTRAIN_DATA.ENGINE_RPM) |
| `gear` | `gear` | `gear` | — | inferred: rpm / wheel speed vs the seed ratios (engine_brake_model.infer_gear, +-8 %) |
| `gear_raw` | `gear_raw` | `gear_mt` | GEAR_MT | GEARBOX_ALT_2.GEAR_MT (0x1DD, bus 0): 0 clutch/neutral, 1-6, 14 reverse (raw tap) |
| `gas_raw` | `gas_raw` | `pedal_gas` | PEDAL_GAS | POWERTRAIN_DATA.PEDAL_GAS (0x17C, raw tap) |
| `raw_payload` | `raw_payload` | `raw_17c` | 0x17C | POWERTRAIN_DATA (0x17C) payload, for the clutch-switch bit hunt (raw tap) |
| `accel_command_sent` | `accel_sent` | `accel_sent` | ACCEL_COMMAND | ACC_CONTROL.ACCEL_COMMAND (0x1C8) as sent on sendcan (raw tap) |
| `brake_request_sent` | `brake_assist` | `brake_assist` | COMPUTER_BRAKE_ASSIST | ACC_CONTROL.COMPUTER_BRAKE_ASSIST as sent (raw tap; the PR #2050 bit) |
| `cruise_buttons` | — | — | — | carState.buttonEvents from SCM_BUTTONS (CRUISE_BUTTONS, CRUISE_SETTING) |
| `start_button` | — | — | — | gapAdjustCruise (the distance button) |

## Sessions (generated from the Civic 6MT profile's registry)

The plan re-cut into ~30-minute outings. Every test belongs to exactly one session or to the **Cal** card. "Requires" is what the session page's ready check compares (✓ / set N); "Offers" are extra switches (A/B, floors) that never count against ready. Prereqs are soft hints (locked tiles), never a block.

| Session | Name | Objective | Tests | Requires | Offers | After |
|---|---|---|---|---|---|---|
| 1 | Signals • parked | Decode GEAR_MT, then the clutch / volume signature hunts, engine idling | S1, S6 (optional), S7 (optional) | — | — | — |
| 2 | Signals • shifts | Real shifts: GEAR_MT lag, RPM noise, lift-to-clutch, rev-match traces | S2, S3, S4, S5 | — | — | 1 |
| 3 | Baseline | 5 min engaged on op long with every feature OFF: no faults, no blocked frames | D1 | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here), `NonAdaptiveLongitudinal` OFF, `ClutchShiftGate` OFF, `NoServiceBrakeHold` OFF, `EngineBrakePreferred` OFF, `GearAwareAccel` OFF | — | — |
| 4 | Command authority | Does a negative ACCEL_COMMAND brake, and does 0 coast or hold speed? | C1, C2 | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here) | — | 3, Cal |
| 5 | Clutch gate | Gate frame acceptance first, then real up/down shifts | G1, G2 | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here), `ClutchShiftGate` ON, `DisengageOnAccelerator` OFF | `ClutchReengageSmart` OFF/ON, `ClutchShiftGateGasReleaseHint` OFF/ON | 2, 3 |
| 6 | Clutch feel | Rev-match / heel-toe and engine-brake downshifts with the gate | G3, G4 | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here), `ClutchShiftGate` ON, `DisengageOnAccelerator` OFF | `ClutchReengageSmart` OFF/ON, `ClutchShiftGateGasReleaseHint` OFF/ON | 5 |
| 7 | Non-adaptive | Hold set speed, no braking for a slower lead; over set downhill | N1, N2 | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here), `NonAdaptiveLongitudinal` ON | — | 4 |
| 8 | No-service-brake | Coast over set speed downhill (floors 0.3 and 0.0), then the overspeed backstop | NSB1, NSB2 | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here), `NonAdaptiveLongitudinal` ON, `NoServiceBrakeHold` ON, `NoServiceBrakeOverspeedBackstop` ON | `NoServiceBrakeDecelFloor` 0.3/0.0, `NoServiceBrakeOverspeedMode` maintain/pulse | 7 |
| 9 | Low-speed handback | Handback feel, hold at a stop / hill, neutral while rolling, uphill handback | H1, H2, H3, H4 | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here), `ClutchShiftGate` ON, `DisengageOnAccelerator` OFF | — | 5, 7 |
| 10 | Engine-brake pref. | Gentle trims on engine braking; downhill; lead hand-off only with a helper car | E1, E3, E2 (optional) | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here), `EngineBrakePreferred` ON | — | 4, Cal |
| 11 | Gear-aware accel | Throttle-on / lift-off feel per gear; uphill accel per gear | GA1, A7 | `AlphaLongitudinalEnabled` ON (Developer panel; not settable here), `GearAwareAccel` ON | — | Cal |
| Cal | Calibration (progress card, background) | Background: accel sweep + flat/downhill coasts per gear fill this on any drive | E0c, E0a, E0b | — | — | 2 |

`AlphaLongitudinalEnabled` (openpilot longitudinal, which disables stock AEB on this car) is shown as a requirement but can only be set in the Developer panel.

## The registry (generated from `profiles/civic_6mt/registry.py`)

★ = first-run characterization. Conditions must all hold for a sample to count; toggles are checked by the daemon, never set by it (the session UI can set them — see above).

**Phase 0 — Signal decode** (parked -> low speed, engine running; console labels + cabana) — gates every manual feature (gear inference, clutch gate)

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| S1 | 1 | telemetry ★ | GEAR_MT decode | manual transmission, GEAR_MT tap live | 2 s rolling per GEAR_MT 1–6, 5 s stopped, reverse seen | every gear 1-6 agrees with the RPM/speed ratio >= 95% of rolling samples; reverse reads 14 | — |
| S2 | 2 | telemetry ★ | GEAR_MT lag | manual transmission, GEAR_MT tap live, speed > 3 m/s | 10 shift | p90 bite lag <= REENGAGE_DELAY (0.15 s) | S1 |
| S3 | 2 | telemetry ★ | Engine RPM sanity | manual transmission, speed > 3 m/s, GEAR_MT 1-6, clutch engaged, steady speed | 60 s | no false RPM flares (> 1500 rpm/s while steady in gear) | — |
| S4 | 2 | telemetry ★ | Gas-lift to clutch | manual transmission, speed > 3 m/s | 10 shift [after gas] | p90 lift->clutch <= GAS_RELEASE_WINDOW (0.6 s) | S1 |
| S5 | 2 | telemetry ★ | Rev-match trace | manual transmission, GEAR_MT tap live, speed > 3 m/s | 5 shift [blip] | GEAR_MT stays 0 through every blip and the gate never leaves 'shifting' during it | S1 |
| S6 | 1 | telemetry ★ (optional) • not background | Clutch-bit hunt | manual transmission, GEAR_MT tap live | 9 clutch presses (rolling) or a 45 s signature window (N = 3) | characterization (CAPTURED) | S1 |
| S7 | 1 | telemetry ★ (optional) • not background | Volume-button hunt | none (a marked window) | 90 s | characterization (CAPTURED) | S1 |

**Phase 1 — Baseline drive** (openpilot long (alpha) engaged, all feature toggles OFF) — gates trusting any on-road feature test

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| D1 | 3 | feature ★ | Baseline op-long drive | op long (alpha) on, op long active + NonAdaptiveLongitudinal OFF, ClutchShiftGate OFF, NoServiceBrakeHold OFF, EngineBrakePreferred OFF, GearAwareAccel OFF | 300 s | no CRUISE_FAULT, no panda-blocked TX frames, no fault alerts | — |

**Phase 2 — Command authority** (openpilot long engaged, flat empty road) — gates no-service-brake, overspeed backstop, engine-brake-preferred, the coast target

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| C1 | 4 | feature ★ | Brake strength | op long (alpha) on, op long active, off gas & brake, speed > 3 m/s, cmd <= -0.5 | 5 segments ≥ 1.5 s | achieves >= 50% of the requested decel | D1 |
| C2 | 4 | feature ★ | Cmd 0 / small negative | op long (alpha) on, op long active, off gas & brake, speed > 3 m/s, in gear (rpm/speed), clutch engaged, flat road | 10 s per cmd band zero, small_neg | characterization (CAPTURED) | D1, E0c |

**Phase 3 — Clutch/shift gate** (ClutchShiftGate ON, DisengageOnAccelerator OFF) — gates real road use of the longitudinal features

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| G1 | 5 | feature | Gate frame acceptance | manual transmission, op long (alpha) on, op engaged + ClutchShiftGate ON, DisengageOnAccelerator OFF | 5 shift [engaged open] | every press: paused (no long), stayed engaged, no ACC fault | S1, S2, D1 |
| G2 | 5 | feature | Gate on real shifts | manual transmission, op long (alpha) on, op engaged + ClutchShiftGate ON, DisengageOnAccelerator OFF | 6 shift [engaged shift] (up ≥ 3, down ≥ 3) | every shift paused with no throttle while the clutch was in; resume jerk < 3 m/s^3; 0 false flares | G1 |
| G3 | 6 | feature | Gate + rev-match | manual transmission, op long (alpha) on, op engaged + ClutchShiftGate ON, DisengageOnAccelerator OFF | 5 shift [engaged blip] + 1 brake press | stayed engaged and paused through every blip; brake disengaged within 0.3 s | G2, S5 |
| G4 | 6 | feature | Engine-brake downshift | manual transmission, op long (alpha) on, op engaged + ClutchShiftGate ON, DisengageOnAccelerator OFF | 5 shift [engaged downshift no pedal] | characterization (CAPTURED) | G2 |

**Phase 4 — Non-adaptive set-speed** (NonAdaptiveLongitudinal ON) — gates phases 5 and 6

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| N1 | 7 | feature | Non-adaptive hold | op long (alpha) on, op long active, speed > 3 m/s + NonAdaptiveLongitudinal ON | 60 s hold + 5 s behind a slower lead | set speed held (|mean err| < 0.5, p95 < 1.5 m/s) and no braking for a slower lead | D1 |
| N2 | 7 | feature | Over set, downhill | op long (alpha) on, op long active, above set speed, downhill, off gas & brake + NonAdaptiveLongitudinal ON | 20 s | characterization (CAPTURED) | N1, C1 |

**Phase 5 — No-service-brake coast** (NonAdaptiveLongitudinal + NoServiceBrakeHold ON (needs C1))

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| NSB1 | 8 | feature | Coast over set speed | op long (alpha) on, op long active, above set speed, off gas & brake, no-brake clamp on + NonAdaptiveLongitudinal ON, NoServiceBrakeHold ON | 30 s per nsb floor 0.3, 0.0 | command never below -floor and no service-brake decel beyond the coast estimate | N1, C1, C2 |
| NSB2 | 8 | feature | Overspeed backstop | op long (alpha) on, op long active + NonAdaptiveLongitudinal ON, NoServiceBrakeHold ON, NoServiceBrakeOverspeedBackstop ON | 1 backstop (2 with mode pulse) | maintain: braking resumed within 0.5 m/s of set+margin and reached -0.3 m/s^2 within 3 s; pulse: every pulse starts within 0.5 m/s of set+trigger and releases within 0.5 m/s of set+release, command within [pulse decel, 0], coasts >= 1 s between pulses, no pulse > 20 s (provisional); an escalation is flagged and must show exactly one alert | NSB1, C1 |

**Phase 6 — Low-speed handback** (manual, always on (Task 5); needs phase 4 working)

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| H1 | 9 | feature | Handback feel | manual transmission, op long (alpha) on | 5 handback | one alert per handback and peak jerk < 2.5 m/s^3 (provisional) | N1, G1 |
| H2 | 9 | feature | Hold at stop / hill | manual transmission | 3 stop hold (hill ≥ 1) | no rollback (> 0.3 m/s with brake and gas released) on any stop | H1 |
| H3 | 9 | feature | Neutral while rolling | manual transmission, op long (alpha) on, op engaged + ClutchShiftGate ON | 2 neutral coast | characterization (CAPTURED) | S1, G1 |
| H4 | 9 | feature | Uphill handback | manual transmission, op long (alpha) on | 2 handback [uphill on gas] | characterization (CAPTURED) | H1 |

**Phase 7 — Engine-brake model** (calibration: no toggle; validation: EngineBrakePreferred ON) — gates phase 8

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| E0c | Cal | telemetry ★ | Accel sweep per gear | manual transmission, speed > 3 m/s, on the gas, clutch engaged, accelerating | 3 s per gear 1, 2, 3, 4, 5, 6 | every gear's measured ratio within 3% of the seed (inference tolerance is 8%) | S1, S3 |
| E0a | Cal | telemetry ★ | Downhill coast per gear | manual transmission, speed > 3 m/s, in gear (rpm/speed), clutch engaged, off gas & brake, op not on long, downhill | 6 s per gear 1, 2, 3, 4, 5, 6 | characterization (CAPTURED) | E0c |
| E0b | Cal | telemetry ★ | Flat coast per gear | manual transmission, speed > 3 m/s, in gear (rpm/speed), clutch engaged, off gas & brake, op not on long, flat road | 10 s per gear 1, 2, 3, 4, 5, 6 | characterization (CAPTURED) | E0c |
| E1 | 10 | feature | EBP gentle trims | manual transmission, op long (alpha) on, op long active, no lead, flat road + EngineBrakePreferred ON | 5 ebp suppress | no capacity release; command held at the coast target; no service-brake decel | E0a, E0b, E0c, C2 |
| E2 | 10 | feature (optional) | EBP lead hand-off | manual transmission, op long (alpha) on, op long active, lead car | 2 lead approach (ebp_on ≥ 1, ebp_off ≥ 1) | min gap with EBP ON >= OFF - 2 m, and no FCW | E1 |
| E3 | 10 | feature | EBP downhill | manual transmission, op long (alpha) on, op long active, downhill + EngineBrakePreferred ON | 30 s | never more than 3.6 m/s over set speed | E1 |

**Phase 8 — Gear-aware accel** (GearAwareAccel ON; needs E0c (gear inference))

| ID | Session | Kind | Test | Conditions (+ expected toggles) | Goal | Pass criterion | Prereqs |
|---|---|---|---|---|---|---|---|
| GA1 | 11 | feature | Gear-aware feel | manual transmission, op long (alpha) on, op long active, in gear (rpm/speed) + GearAwareAccel ON | 20 s per gear 1, 2, 3 | characterization (CAPTURED) | E0c |
| A7 | 11 | telemetry | Uphill accel per gear | manual transmission, speed > 3 m/s, on the gas, clutch engaged, uphill | 5 s per gear 1, 2, 3, 4, 5, 6 (required 1, 2, 3, 4) | characterization (CAPTURED) | E0c |
