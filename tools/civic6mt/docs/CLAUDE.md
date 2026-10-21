# Civic 6MT Non-Adaptive Cruise — Project Context

Context for Claude Code sessions working in this project. Read this first.

## Goal
Build a non-adaptive ("classic") cruise control for a **2024 Honda Civic Sport
Hatchback, 6-speed manual**, running **sunnypilot** on a **comma four**. It should:
- Hold a driver-set speed on throttle.
- Ignore lead cars (no adaptive following).
- NOT apply service brakes to hold speed on downhills — let the car drift slightly
  over set speed, like old throttle-only cruise.
- Pause longitudinal during gear changes (clutch-in / shift) instead of fighting them.

## Vehicle / platform
- 2024 Civic Sport Hatchback, 2.0L NA (K20C2), 6MT.
- openpilot fingerprint: `HONDA_CIVIC_2022`. Harness: Honda **"Bosch B" (radarless)**.
  The camera/ADAS unit is the longitudinal authority (no radar).
- Build target: **sunnypilot**, specifically **mvl-boston's `sp-honda` fork family**
  (the only active 11th-gen / Bosch-radarless Honda work). Base branch TBD — see Open Questions.

## Architecture facts established (why the approach is what it is)
- On this car, BOTH adaptive and conventional cruise run **inside the camera/ADAS
  unit**. There is no standalone throttle-only cruise governor in the engine ECU
  (unlike pre-Honda-Sensing cars). Conventional "Cruise mode" = ACC with
  lead-following off; the camera still computes and sends the deceleration.
- The camera sends `ACC_CONTROL` (CAN `0x1C8`) with a signed `ACCEL_COMMAND`
  (~ -3.5 .. +2.0 m/s²). AEB/CMBS rides as **separate flag bits in the SAME
  message** (`AEB_STATUS`, `AEB_PREPARE`, `AEB_BRAKING`, `FCW_1/2`). The VSA
  modulator executes it and faults (`CRUISE_FAULT`, ~0.3 s) if the `0x1C8` stream stops.
- Because cruise decel and AEB share one message from one sender, anything that edits
  the camera's message in-flight sits in the AEB path.

## Chosen approach (and the one rejected)
**CHOSEN:** sunnypilot longitudinal, customized to be **non-adaptive** (set-speed only,
ignore leads), with a **clutch/shift gate** to pause longitudinal during gear changes.
Stays inside openpilot's existing longitudinal safety envelope (panda accel/jerk limits,
FCW). Does NOT modify the panda safety firmware.
- **CONSEQUENCE (must be understood):** running openpilot longitudinal on this car
  REPLACES the camera's message and therefore **DISABLES stock AEB/CMBS while engaged**.
  openpilot does not provide its own AEB. This is a known, disclosed openpilot tradeoff
  (identical to enabling experimental longitudinal), not a hidden change. Keep openpilot's
  own longitudinal limits and FCW intact.

**REJECTED (out of scope):** intercepting/trimming the camera's `ACC_CONTROL` in the panda
to suppress only the downhill cruise-brake while keeping stock AEB. Though cruise-decel and
AEB are logically separable by flag, the implementation requires modifying the **panda
brake-actuation safety guard** and rests on an AEB flag/timing relationship comma itself has
not fully characterized (TODO in carstate). Not pursued in this project.

## Key code locations
- **Lead-following (disable for set-speed-only):** *(corrected 2026-10-15 against the actual
  `sp-honda-dev-202608` code — the earlier "[lead0, lead1, cruise] obstacle stack" description was
  wrong for this fork.)* `long_mpc.py` (`openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/`)
  stacks only `[lead0, lead1]` — there is no cruise column. Set speed is a separate `a_cruise`
  candidate (`get_cruise_accel`) in `LongitudinalPlanner.update()`
  (`openpilot/selfdrive/controls/lib/longitudinal_planner.py`), and the output accel is
  `min(MPC-lead, a_cruise, e2e)`. Non-adaptive mode therefore gates the *candidates*, not the MPC.
- **Manual gear state:** `honda/carstate.py` currently only sets `gearShifter = reverse/drive`
  from `REVERSE_LIGHT` (`SCM_FEEDBACK`) — BUT the full state is already on the bus and decoded:
  **`GEARBOX_ALT_2` (0x1DD) `GEAR_MT`** = `0` Clutch, `1–6` gears, `14` reverse (bus 0, comma taps
  it; CANParser already decodes it, carstate just ignores it for manuals). So numeric gear AND
  clutch are a one-signal read *(Task 12 correction: not "no new plumbing" — 0x1DD had to be registered, with NaN
  frequency, plus a liveness flag; Task 13 A3 added counter/checksum checks)*. `REVERSE_LIGHT` default came via opendbc PR #3245
  (jyoung8607, 2026-03-28).
- **Radarless longitudinal braking defect:** opendbc PR #2050 (mvl-boston, UNMERGED) —
  without it radarless Hondas "won't brake, only accel/coast." Relevant both to base behavior
  and to the no-downhill-brake goal (the unfixed state is coincidentally close to what we want,
  but don't design on a defect).

## Clutch/shift gate (the safety-improving piece) — SIGNALS CONFIRMED
Pause/relax longitudinal on clutch-in so the controller doesn't flare RPM or fight the shift.
Signals verified against opendbc master `06ade56` (2026-10-14); full detail in
`docs/manual_clutch_rpm_signals.md`:
- **Clutch/gear:** `GEARBOX_ALT_2` (0x1DD) `GEAR_MT` → `0` = Clutch, `1–6` = gears, `14` = reverse.
  Already decoded on bus 0; carstate ignores it for manuals. `GEAR_MT == 0` = clutch open.
- **Throttle:** `PEDAL_GAS` (`POWERTRAIN_DATA` 0x17C) — drives `gasPressed`; continuous `gas` deprecated/unused.
- **RPM:** `ENGINE_RPM` in `ENGINE_DATA` (0x158) and `POWERTRAIN_DATA` (0x17C) — present, undecoded.
  **On-car (Stage 1, 2026-10-20): 0x158's `ENGINE_RPM` is always 0 on this car; only 0x17C carries RPM.** The fork's
  carstate read 0x158 for CIVIC_2022 → `engineRpm` = 0 → EBP / GearAware / ratio lock / flare backstop inert. **Fixed in
  Task 15** (every radarless manual reads 0x17C).

**On-car Stage 1 (2026-10-20, stock SP, route `00000002--6c1a3bb7d4`) — full detail `docs/stage1-findings-2026-10-20.md`:**
0x1DD 50 Hz, counter +1 on every frame, 0 checksum faults; GEAR_MT ∈ {0, 1–6, 14}; 0 with clutch in while in gear AND
in neutral clutch-out. GEAR_MT is **ratio-derived**: it reports a gear only when the clutch is fully up and the ratio
already matches (no bite lag — `REENGAGE_DELAY` is extra coast), and it **falsely reports a gear in neutral at idle**
when speed matches a gear's idle ratio (seen 6th, then 5th). **Clutch pedal switches found:** `GAS_PEDAL_2` (0x130)
byte 6 bit 50 = clutch fully up, bit 51 = clutch on the floor (undefined in the DBC). Gear ratios = seed within 0.4 %.
0x130 `ENGINE_TORQUE_ESTIMATE` reads as axle torque; closed-throttle crank drag 23–27 Nm (seed 27–38) — the seed
over-predicts flat-road engine braking by 0–0.26 m/s² (worst in 2nd).
- capnp slots already exist (deprecated group): `clutchPressed @28`, `engineRpm @46`, `gas @3` —
  populate these in the manual branch.

Detection order (fast → slow): (1) `PEDAL_GAS` release (driver lifts before clutch), (2) `GEAR_MT == 0`
(clutch open), (3) RPM-flare / RPM-decoupled-from-wheelspeed backstop (lags — flare is after clutch in).
Behavior: **relax to coast, do NOT disengage** (mimic stock PCM fuel-cut); resume on re-engagement.

Optional earlier edge: a raw clutch-switch bit may live in `POWERTRAIN_DATA` reserved fields
(`BOH_17C` bits 33–37, `BOH2_17C`, `BOH3_17C`) — find with cabana (press clutch, watch which bit
toggles 1:1) if wanted.

Implementation hooks: set `ret.clutchPressed` / `ret.engineRpm` in `honda/carstate.py` manual branch;
relax longitudinal in the planner / `controlsd` (note `longitudinal_planner.py` lives in the openpilot
repo, not opendbc). Honda gas output clamp point: `carcontroller.py` `new_actuators.gas`. Closest
existing precedent: `disengageOnAccelerator` / `gasPressed`.

## Rule 5 / urgency predicate (Task 12 — applies to every longitudinal feature)
**One predicate, used everywhere:** `openpilot/sunnypilot/selfdrive/controls/lib/braking_urgency.py`
`braking_urgency(...) -> reason | None` (controlsd: `controls_urgency(sm, plan_live)`, computed once per control frame
in `ControlsExt.update_sp_state`; plannerd: `LongitudinalPlannerSP.update_urgency`). Reasons, most severe first:
`invalid_plan` / `invalid_radar` (via `sm.all_checks` — stale counts as invalid, never `.valid` alone), `force_decel`
(DM no-response or soft-disabling), `fcw`, `stock_aeb` (stock AEB/FCW; **dead on the radarless Civic while openpilot
long runs** — the panda blocks 0x1C8 — kept for other platforms), `stopping` / `hard_decel` (plan.shouldStop / aTarget
< −1.5; **only from a live plan** — while LongControl is off the planner re-seeds from aEgo, so those two would just echo
the car's own decel), and `lead_ttc` / `lead_close` / `lead_braking` / `lead_closing` (TTC < 6 s, < max(10 m, 1 s),
lead braking < −1 m/s², closing ≥ 1 m/s at any distance).
- **The rule:** any code path that pauses longitudinal, raises the lower accel limit or rate-limits braking steps aside
  **within the same control frame** when the predicate returns a reason — the lower limit goes back to stock and the
  plan's braking passes. Call sites: clutch gate (it never raises the lower limit at all since A2), takeover ramp (ends),
  low-speed handback (freezes, lower limit = incoming — on its own narrower set, see below), NSB maintain + pulse
  (plannerd allowBrake AND controlsd ignores the floor), EBP (backstop "urgency" first), GearAware (stock scales, and since
  Task 13 A3 controlsd runs the stock-jerk plan `longitudinalPlanSP.features.aTargetStock` in the same control frame). The only exception is physical: a paused shift
  with the gas pressed must send exactly 0 (the panda blocks any other 0x1C8 accel with gas pressed — stock's override
  outcome); it has its own test.
- **Non-adaptive is different on purpose (owner decision 2026-10-19):** it does NOT re-admit lead following on the broad
  lead reasons (that would make it follow traffic). `collision_threat()` is the narrow set: FCW, stock AEB, TTC < 6 s, a
  close lead that is also closing > 0.3 m/s or braking, a lead braking hard within 3 s, or the lead MPC itself needing
  < −1.0 m/s² with a lead present; held 2 s.
- **The low-speed handback is narrower too (Task 13 A1, finding N-1):** the broad lead reasons froze it on any lead
  closing ≥ 1 m/s at any distance, so ordinary stop-and-go held a manual engaged at a standstill. `handback_urgency()`
  keeps FCW, stock AEB, forceDecel (aborts), a hard (< −1.5) plan request and invalid/stale inputs; drops `stopping` and
  the broad lead reasons; adds one kinematic lead test (`lead_imminent`: a driver braking at 2 m/s² only after ramp +
  0.1 s + 1.5 s reaction couldn't stop 2 m short; since Task 14 A4 a downhill grade from the calibrated pose enlarges
  it, uphill / no pose = level). **The latch (as fixed in Task 14):** it SETS on that imminence test itself, whatever
  reason `handback_urgency` returns (A2 — FCW / hard_decel used to hide it); it HOLDS while some lead is still closing
  within the test + 2 m; it RELEASES 0.5 s after none is (A1 — a lead that drove off used to leave openpilot engaged and
  frozen), and the handback then completes as if the lead had never been there. **Standstill resolution:** stopped 0.5 s
  while urgent → the handback completes anyway; FCW / stock AEB are never resolved away, also after a resolution (A3).
- **Enforcement:** `tests/test_rule5.py` — the matrix (12 limiting states × 13 injected reasons, closed loop, one frame;
  the handback's `HANDBACK_CONTINUES` set is asserted NOT to freeze it) and an AST tripwire that fails if a function raises
  a lower accel limit / writes `longActive` without calling the predicate and acting on it (Task 13 C2: a taint pass —
  a predicate call, an `*urgen*` parameter or a class attribute assigned from one must reach a decision; a docstring, an
  unused parameter or a dropped call no longer passes), or a REVIEWED / EXEMPT entry (the gate's panda gas-0 case). `tools/civic6mt/mutations.py` removes each call site
  in turn; some test must fail.
- **Adding a feature:** take `urgency` as an argument (never re-derive it), step aside on any reason, add a matrix state
  and a mutation. Comfort features may use stricter thresholds of their own *on top*, never looser.

## Design decisions (implementation log)
Feature work is **SIM/TEST-ONLY** until the comma four is installed: new behavior is behind
`Params` toggles that default OFF, and every on-car unknown is marked `# TODO(on-car): verify ...`.
`grep -rn "TODO(on-car)"` across both forks lists the open items.

**Phase 1 — signal plumbing** (opendbc `honda/carstate.py`, manual branch):
- `clutchPressed` / `engineRpm` live in `CarState`'s capnp **`deprecated` group**, so they are
  written/read as `ret.deprecated.clutchPressed` / `CS.deprecated.engineRpm` — not `ret.clutchPressed`.
  Reusing these existing slots avoids any cereal schema change.
- Gated on `HondaFlags.BOSCH_RADARLESS`: `GEARBOX_ALT_2` exists only in the radarless DBC, so other
  manual Hondas (e.g. Nidec Civic) must not read it. Auto/CVT paths untouched.
- Not toggle-gated: it is telemetry only (no consumer until Phase 3), so it cannot change driving.
  *(Task 12 correction [A-F-M6]: "no new plumbing / nothing new subscribed" was wrong — `GEARBOX_ALT_2` was registered
  with a frequency check, so a missing 0x1DD could have invalidated CAN. It is now registered with NaN frequency and
  `CarStateSP.clutchSignalValid` / `gearRaw` gate every consumer.)*
- Numeric gear (`GEAR_MT` 1–6) is **not** exposed: it would need a new field in sunnypilot's
  `custom.capnp` (fork-only schema change, ordinal-collision risk on future upstream rebases) with no
  consumer, and raw `GEAR_MT` is already in rlogs for cabana.

**Phase 2 — non-adaptive longitudinal** (openpilot `longitudinal_planner.py` + SP mixin), toggle
`NonAdaptiveLongitudinal` (default OFF, registered in `openpilot/common/params_keys.h`):
- When ON, the output accel uses **only the `a_cruise` (set-speed) candidate**; the lead-following MPC
  candidate and the e2e/experimental-model candidate are dropped. `long_mpc.py` is untouched and the
  MPC still runs every cycle, so **FCW (`mpc.crash_cnt`) is unchanged** — note it remains an
  emergency-threshold warning (fires when even max braking can't avoid the lead), not an early
  "closing" warning, and in this mode openpilot will NOT brake for traffic; the driver must.
- *(Superseded by Task 12 A1: non-adaptive now keeps emergency collision braking and keeps stock's `allow_throttle`
  cap — see Task 12.)* Also ignores the model's predicted-lift throttle cap (`allow_throttle`), which is a soft traffic
  response that would otherwise drop below set speed. Accel/jerk limits (`get_cruise_accel`, PID
  limits, panda) unchanged. `forceDecel` (driver-monitoring) still works. Sunnypilot's own opt-in
  set-speed modifiers (SCC vision/map, speed-limit assist) are left alone.
- The toggle is re-read at 1 Hz but only **latches while longitudinal is inactive** — the mode never
  flips mid-engagement. *(Task 12 B3 correction: "longitudinal inactive" included gas overrides and shifts, so it could
  flip mid-engagement; it now latches only while `selfdriveState.enabled` is false.)*
- Still brakes when **above** set speed (`a_cruise` floor −1.2 m/s²), i.e. it does NOT yet meet the
  "no service brake downhill" goal — that is still the open PR #2050 / brake-authority question.
- Verified by: planner closed-loop tests (`Plant` simulator) and a **self-referenced process replay**
  (plannerd on CI segments, captured at the pre-change commit): toggle OFF = 0 diffs; toggle ON changes
  only planSource/aTarget/trajectories/allowThrottle — never fcw/hasLead/shouldStop/allowBrake.
  Upstream sunnypilot process-replay refs are not usable — this fork has diverged from them.

**Phase 3 — clutch/shift gate** (`openpilot/sunnypilot/selfdrive/controls/lib/clutch_shift_gate.py`,
wired via `ControlsExt.get_long_paused()` into `CC.longActive` in `controlsd.py`), toggle
`ClutchShiftGate` (default OFF), only available when `CP.transmissionType == manual`:
- *(Superseded by Task 12 A2: a shift no longer drops `longActive` — braking stays, only the throttle is capped.)*
  **Mechanism = the existing longitudinal-override path** (same as a gas override): `longActive=False`
  while `enabled` stays True → `LoC` resets, accel 0, `cruiseControl.override` set; the Honda radarless
  `ACC_CONTROL` keeps `CONTROL_ON` (tied to `enabled`) with no gas and no brake → coast, no disengage.
  On resume the planner re-seeds from `aEgo` and LoC restarts from zero (Task 8 adds an opt-in ratio-lock / seeded
  takeover — "bumpless" was overclaimed, see the Task 12 B4 correction — and a neutral-coast timeout). No cereal/event schema change.
- States: `idle → gasReleased` (0.6 s hold after a gas-override release, waiting for a clutch) /
  `shifting` (clutch in, or RPM-flare backstop) `→ reengaging` (0.5 s after the clutch is back in gear,
  since GEAR_MT may report a gear before the clutch fully bites) `→ idle`.
- RPM-flare backstop = RPM rising >1500 rpm/s while `aEgo < 0.5` and `vEgo > 2`. It can only **start**
  a pause from idle/gasReleased; it never extends a shift in progress, because the RPM jump as the
  clutch bites on a **downshift** looks like a flare but is the engine re-coupling.
- *(Superseded by Task 2 below: the gas-release cue is now opt-in, default OFF.)* With it ON it adds a
  0.6 s coast after *every* gas-override release, delaying openpilot braking for a lead by up to 0.6 s
  when lead-following is active. During a real shift the gate stops any openpilot braking (as stock
  fuel-cut would) — the driver is in the loop by definition. All failure modes bias toward coasting
  (no gas, no brake), never toward unwanted throttle.
- Toggle changes apply only while the gate is idle (never cuts a shift short). State transitions are
  logged via `cloudlog.event("clutch_shift_gate", ...)` for on-car review.
- Verified by: 13 state-machine unit tests (synthetic carState sequences, incl. downshift RPM jump,
  backstop, gas-release window, toggle latch); an integration replay of controlsd on the real
  Nidec-Civic CI segment fingerprinted as a manual with an injected 0.8 s clutch press (paused exactly
  130 frames = 0.8 s + 0.5 s, enabled stayed True, accel 0, then resumed; a second 0.6 s pause matched
  the one real gas release in the log); and self-referenced controlsd/plannerd replays (0 diffs, OFF and
  ON — CI segments are automatics, where the gate is unavailable).
- Replay tooling lives in this repo under `tools/` (`selfref_replay.py`, `clutch_gate_replay.py`). *(Task 12 E1: the
  proof tools now live in the openpilot fork under `tools/civic6mt/` — every `tools/<name>.py` mentioned in this file is
  there now (dev-test-only ones on dev-test); `~/civic-cruise/tools/` only has a pointer README. `clutch_gate_replay.py`
  is retired — its "pause drops longActive" check no longer holds after A2.)*

**Architecture principle:** clutch/transmission-sensing features (Phase 1 signals, Phase 3
clutch/shift gate + Task 8 re-engagement, Task 5 manual low-speed handback, Task 6 engine-brake features) are **manual-only**. Longitudinal-*behavior*
features (Phase 2 non-adaptive set-speed, Task 4 no-service-brake) are **transmission- and
platform-agnostic**, gated only on "openpilot longitudinal is active" (`CP.openpilotLongitudinalControl`).
They are validated only on the Civic — user-facing toggle reference and platform/AEB warning: `docs/TOGGLES.md`.

**Task 1 — stock panda safety limits restored** (opendbc `4d8fed351`, bump `12c01a9a6`; tightening only):
- `HONDA_BOSCH_LONG_LIMITS.max_gas` 2200 → **2000**; the Nidec `ACC_HUD` gas-pressed exemption (which
  let *any* PCM_GAS through during an override, even > MAX_GAS) removed.
- Commanding side reverted to match, because a frame the panda blocks drops `ACC_CONTROL` and latches
  `CRUISE_FAULT`: RDX 3G / Pilot 4G gas lookups capped at 2000; the Nidec pedal mirror on `ACC_HUD`
  replaced by stock zeroing (cost: MVL's Nidec slow-release fix is gone — Nidec only, not our car).
- **Our car was not in either path:** the radarless `ACC_CONTROL` (0x1C8) has no panda gas check at
  all, and Bosch sends an all-zero `ACC_HUD` whenever `longActive` is false. Phases 1–3 don't depend
  on the loosened limits.
- The loosening hid because `test_gas_safety_check` couples accel = gas/1000 (gas > 2000 always trips
  the accel limit first). New tests pin the gas limit independently, assert all-zero `ACC_HUD` while gas
  is pressed, and check every `BOSCH_GAS_LOOKUP_V` against `max_gas` parsed from `honda.h`.

**Task 2 — rev-match / heel-toe** (openpilot `0a12d2319`; gate refinement, still behind `ClutchShiftGate`):
- **Clutch-open (`GEAR_MT == 0`) is the authoritative trigger.** While it's open, gas is ignored for
  longitudinal purposes; once seen it also cancels/never arms the RPM backstop (a blip on the first
  clutch frame used to arm a 1 s backstop hold that outlived the shift — found and fixed by a test).
- Gas-release "anticipate" hold → new param `ClutchShiftGateGasReleaseHint`, **default OFF**; never
  arms while the clutch is open. This also removes the 0.6 s braking delay after plain gas overrides.
- `gasPressedOverride` is dropped from onroad events while the clutch is open (`car_specific.py`,
  manual + toggle only), so a blip isn't treated as a driver accel override. Brake events are never
  touched: heel-toe braking still disengages exactly as stock. The panda still disallows longitudinal
  while gas is pressed, so this is cosmetic/state-machine only, not a safety relaxation.
- **Requires `DisengageOnAccelerator` OFF** (the default). With it ON the *panda* drops controls on any
  gas rising edge, so a rev-match blip disengages openpilot; suppressing that would need a panda change
  (forbidden) or produce `controlsMismatch`. The gate logs a warning if both are on.
- Resume re-seeds from `aEgo` (planner reset on long-off) — now pinned by a test.
- `TODO(on-car)`: engine-brake downshifts with long active and no brake — openpilot may add throttle to
  hold set speed after the re-engage delay.

**Task 3 — non-adaptive generalized** (openpilot `13db13835`): it was never manual/Honda-gated, but it
also wasn't gated on openpilot owning longitudinal — on stock-ACC cars it was reshaping a plan that
doesn't actuate (CI replay: Accord/Sonata segments changed by 56537/162 diffs with the toggle ON). Now
available only when `CP.openpilotLongitudinalControl`; with the toggle ON only the op-long CI segment
changes. Tests cover manual/automatic/CVT, a Toyota, and the stock-ACC no-op.

**Task 4 — no-service-brake ("coast over set speed")** (openpilot `b62c24698`), toggle `NoServiceBrakeHold` (default OFF) +
`NoServiceBrakeDecelFloor` (0.3 m/s²; 0.0 = throttle-only), `NoServiceBrakeOverspeedBackstop` (**ON**),
`NoServiceBrakeOverspeedMargin` (3.6 m/s ≈ 8 mph). Code: `sunnypilot/.../no_service_brake.py`.
- Platform-agnostic; acts only on openpilot's own commanded accel — not the camera/AEB path, not panda.
- **plannerd is the single authority** on whether braking is allowed and publishes it in the existing
  (previously always-True, unconsumed) `longitudinalPlan.allowBrake`. **controlsd** applies the floor to
  `LongControl`'s accel limits — the final command, all LoC states, PID anti-windup. Clamping only the
  planner's aTarget would not work: downhill, the PID adds braking on its own to track it.
- Clamp active only when: toggle latched on (only while long inactive — *Task 12 B3: now only while disengaged*) **and non-adaptive actually
  active** (safety coupling) **and** openpilot owns longitudinal (pcmCruise → no-op) **and not**
  `forceDecel` (driver-monitoring decel is never limited) **and not** in overspeed backstop.
- Fail-safe: capnp's default for `allowBrake` is False, so controlsd ignores it unless the plan is valid
  and its own toggle read is on — any doubt means braking is allowed.
- Backstop: braking re-allowed above set + margin, released below set + margin/2 (hysteresis, logged) — the
  `maintain` mode; Task 11 added the bounded brake-pulse mode (`NoServiceBrakeOverspeedMode`, below).
  **Only as strong as openpilot's radarless braking** — braking on this car is the camera-path negative
  `ACCEL_COMMAND`; how hard the VSA actually brakes on it is the open PR #2050 question, unverified.
  *(Corrected 2026-10-16: this used to say it hinged on "`0x1BE` in the fingerprint". `0x1BE` is the
  alternate user-brake-pedal message, not brake authority, and is not fingerprinted on CIVIC_2022.)*
- Caught by a probe before shipping: `Params.get_bool()` ignores registered defaults, so the "default ON"
  backstop read as OFF; now read via `get(..., return_default=True)` and pinned by a test.
- Platform warning: only validated on `HONDA_CIVIC_2022` (cloudlog warning elsewhere); stock-AEB
  behavior with openpilot long differs per platform — see `docs/TOGGLES.md`.
- Tests: closed-loop planner → floor → LongControl → downhill vehicle sim (12 tests, 6 safety
  mutations all caught).

**Task 5 — manual low-speed longitudinal handback** (openpilot `a00cd3ae7`; opendbc unchanged). *Revised
design (2026-10-16), replaces the earlier "brake to stop + standstill hold" (Option 2) draft, which was
discarded uncommitted.* Code: `sunnypilot/.../manual_low_speed_handback.py`, wired in `controlsd.py` /
`controlsd_ext.py` and `car_specific.py` (selfdrived).
- **Behavior:** manual + openpilot long, below `ManualLowSpeedHandbackSpeed` (default 2.24 m/s ≈ 5 mph,
  clamped 1.5–8.0 so it can't be tuned into a brake-to-stop), openpilot hands longitudinal back — in BOTH
  adaptive and non-adaptive modes. It never brakes to a stop, never commands a standstill hold, never
  launches. **Assumption:** the driver takes the stop (downshift → neutral → own braking) and the car's own
  **Auto Brake Hold** (driver-selectable) and **automatic hill-hold** cover holding at a stop and anti-rollback.
- **Intended consequence (adaptive mode):** the driver takes the final ~5 mph of braking behind a stopping
  lead themselves — matches how this driver already stops.
- **Sequence** (latched once below the threshold, until disengaged): controlsd shrinks LongControl's accel
  range linearly from the last command to 0 over `ManualLowSpeedHandbackRamp` (default 0.5 s, 0–2 s) — no step,
  no PID windup — then sets `longActive=False` (no gas/brake/stop request); selfdrived disengages 0.1 s later
  (`DISENGAGE_MARGIN`) with the stock `speedTooLow` event. Engaging below threshold + 0.5 m/s is refused with
  stock `belowEngageSpeed` → no engage-at-standstill-and-pull-away.
- **Alerts:** stock `speedTooLow`/`belowEngageSpeed` only *drive* the state machine (sunnypilot events can't);
  their texts are outranked by two new SP alerts — `manualLowSpeedHandback` ("Low Speed: Take Over",
  orange prompt, prompt chime, Priority.HIGHEST, fires once at the disengage) and `manualLowSpeedNoEntry`
  ("Manual: Drive Above N mph to Engage"; stock text would read "-1 mph" because `minEnableSpeed = -1`).
  **Fork-only `custom.capnp` ordinals `@26`/`@27`** — may collide on a future sunnypilot rebase.
- **Cross-process timing:** controlsd and selfdrived each run the same latch/timer on the same carState with
  params that only change while disengaged, so they agree within a frame or two; the margin keeps the
  disengage after the ramp. Fail-safe: if the disengage never came, controlsd stays at `longActive=False`.
  *(Superseded by Task 12 A3: controlsd alone owns the latch/timer and publishes it; selfdrived acts on the published
  state. The prompt now shows from the ramp start at Priority.HIGH, below FCW. Under any Rule-5 reason the ramp freezes
  and braking stays at the stock limit — the car may be braked to a stop.)* *(Task 13 A1: only the handback's own narrow
  set freezes it now — an ordinary approach to slow/stopped traffic hands back at the handback speed as with no lead;
  an imminent lead brakes to the stop and the handback completes 0.5 s after standstill. It never holds a manual.)*
- **Not a toggle** (deliberate exception to "new behavior default OFF", like Task 1): without it openpilot
  brakes a manual to a stop and auto-launches it. Only the threshold/ramp are tunable. `autoResumeSng` in
  opendbc is left as-is (metadata only — nothing consumes it; the handback structurally prevents a launch).
- On Honda, `speedTooLow`/`belowEngageSpeed` also end/block MADS lateral at that speed (MADS only strips them
  for non-Honda brands) — consistent with stock Honda low-speed behavior. **Kept as-is for now; flagged as a
  feature improvement** (see Open questions: "Lateral at the low-speed handback"). *(Corrected 2026-10-21: wrong for
  the handback disengage — MADS runs after selfdrived and MVL's all-speed-MADS branch strips both once longitudinal is
  off; Stage 3 logs: `latActive` stayed True through both handbacks. The one real coupling, a UEM SET press below the
  handback speed with lateral off, is fixed — see Session 10 prep.)*
- **Composition with Phase 2 / Task 4 (integration tests, all four `NonAdaptiveLongitudinal` ×
  `NoServiceBrakeHold` combos):** no conflicts. Handback timing and result are identical in every combo; the
  NSB floor and the handback envelope intersect (tighter bound wins, never widens). Interactions found:
  (1) **non-adaptive never slows for a lead**, so it reaches the threshold only when something else slows the
  car (hill, gas override release); behind a stopping lead the driver brakes, and on a Honda the **brake pedal
  disengages openpilot immediately (stock `pedalPressed`) far above the threshold** — the handback never runs
  (tested: disengage at 12 m/s ≈ 27 mph, same step as the pedal). Without the driver, FCW is the only warning.
  (2) **Uphill:** if openpilot is on the gas when it crosses the threshold, the ramp takes the throttle away
  over 0.5 s while still climbing — the car slows quickly; the driver must be ready with gas/clutch.
  (3) NSB is inert in adaptive mode (coupling), so with lead-following its ON/OFF runs are bit-identical.
  (4) A clutch press at low speed (gate ON) already zeroes the command, so a handback starting then has
  nothing to ramp.
- `TODO(on-car)`: GEAR_MT reads 0 for clutch-pressed AND neutral-with-clutch-out; the handback sidesteps this
  at stops. Coasting in neutral **while rolling** with openpilot engaged used to make the clutch gate coast
  indefinitely — now bounded by Task 8's `ClutchShiftGateMaxCoast` (8 s); verify on-car (H3).
- Verified by: 10 closed-loop tests (planner → handback → LongControl + CarSpecificEventsSP → StateMachine →
  AlertManager): smooth ramp, one alert, disengage at ramp + margin while rolling, no stopping state/standstill
  hold ever commanded, engage at standstill refused, non-adaptive handback, automatic unchanged (still
  stops/holds), pcmCruise no-op, params clamped/latched, zero ramp; 5 integration tests (toggle matrix);
  6 safety mutations all caught; self-referenced controlsd + selfdrived replay: 0 diffs on all CI segments.

**Task 6 — engine-brake-aware longitudinal** (openpilot `18f5f59bf`; no opendbc code change; the separate 0x1BE
doc-correction commits are openpilot `d609dd301` / opendbc `02ad781f6`). Plan + stage only, NOT road-applied. Brief: `.../CommaAI/prompts/task6-engine-brake-planner.md`. Three pieces, manual + openpilot long only:
- **6b `engine_brake_model.py`** (`sunnypilot/selfdrive/controls/lib/`): `estimate_coast_decel(gear, rpm, v_ego, grade)`
  = closed-throttle engine-brake torque (seed: 20→58 N·m over 700→6500 rpm) × overall ratio / wheel radius ÷ effective
  mass (engine inertia reflected through the gearing) + rolling + aero − g·sin(grade). Gear 0 → rolling/aero/grade only.
  **UNCALIBRATED SEED.** Ratios 3.643/2.080/1.361/1.024/0.830/0.686 from Honda's 2022 Civic Hatchback 6MT spec sheet;
  final drive 4.10 is `TODO(on-car)` (sources say 4.10 or 4.11 for the 2.0 L), tire radius 0.315 m guessed.
  - **Gear is inferred from RPM ÷ wheel speed** (`infer_gear`, ±8 % match), *not* `GEAR_MT`: Phase 1 deliberately kept
    numeric gear out of the schema, and the ratio is what sets engine braking anyway. It also fails safe: clutch in,
    neutral-with-clutch-out (where `GEAR_MT` reads 0 too), a slipping clutch, no RPM, or wrong seed ratios → gear 0 →
    nothing credited → brake. Exposing `GEAR_MT` would be a `CarStateSP` field + one carstate line if ever wanted.
  - **Calibration hook** `CoastEventLogger` (plannerd, 10 Hz): in-gear coasts (clutch engaged, no gas/brake, openpilot
    NOT commanding long — its gas feedforward would contaminate the coast — no stock AEB/FCW, no FCW, ≥ 1 s, one gear
    per event, ≤ 30 s chunks) → JSON lines `[t, rpm, v_ego, ratio, a_ego, pitch]` in `/data/engine_brake_coast/coast.jsonl`
    (RotatingFileHandler, 1 MB × 10) + `engine_brake_coast` cloudlog summary. Raw ratio logged so the fit can re-bin gears.
    Not toggle-gated (telemetry, like Phase 1). `TODO(on-car)`: fit a per-gear/rpm table from these logs, replace the seed.
- **6a `EngineBrakePreferred`** *(superseded by Task 17: replaced by the per-gear clamp; the coast-target design below
  is history)* (toggle, default OFF; `engine_brake_preferred.py`, wired in `controlsd_ext.py` /
  `controlsd.py`): like Task 4, a raise of **LongControl's lower accel limit** (final command, all LoC states, PID
  anti-windup) to `EngineBrakeCoastTarget` (0.0) while `a_eb_credited ≤ aTarget < 0`, where `a_eb_credited` = 0.7 ×
  (engine + road estimate) + gravity in full, capped at the hard floor. **controlsd alone decides** (unlike Task 4's
  plannerd authority): everything it needs is already in controlsd (carState, `radarState`, `longitudinalPlan`, the
  calibrated pose for pitch, forceDecel), and the decision belongs next to the final command.
  - Backstops (any → no suppression, and a 1.5 s hold after a release or urgent trigger): lead closing ≥ 1.0 m/s,
    TTC < 6 s, < 1.0 s headway / 10 m, or braking harder than 1 m/s² (leadOne and leadTwo); aTarget or any planned accel
    in the next 1 s < hard floor (−1.5); planner FCW, stock AEB/FCW; forceDecel; shouldStop / LoC not in pid; invalid
    plan/radar; no pitch; < 5 m/s; gear 0; demand steeper than the credit (hard hand-off); **capacity** — while
    suppressing, `aEgo > aTarget + 0.15` for 0.75 s or an integrated shortfall > 0.25 m/s → release **and disable for
    the rest of the engagement** (the estimate isn't trusted again until re-engaged; avoids a suppress/brake cycle).
  - Composition: NSB and EBP both only raise the lower limit → max of the floors, order-independent; the handback
    then shrinks what's left. An upstream upper limit below the coast target is never widened.
  - **Radarless semantics caveat (found while building — the #1 on-car question):** radarless `ACC_CONTROL` carries only
    `ACCEL_COMMAND` (no `GAS_COMMAND`); the car arbitrates throttle vs. brake from that accel request. The brief assumes
    command 0 = closed-throttle coast. If 0 actually *holds speed* (PCM adds throttle), suppression can't slow the car:
    the capacity backstop catches it within ~1 s and turns the feature off for the engagement (safe, but useless), and
    the coast target would need to follow the engine-brake level instead. If the car already uses fuel-cut for small
    negative requests, the feature is unnecessary. Also entangled with PR #2050 (radarless brake strength).
- **6c `GearAwareAccel`** (toggle, default OFF; `gear_aware_accel.py`, planner): `get_cruise_accel` gained
  `j_scale=(rise, fall)` (default (1.0, 1.0) = bit-identical stock). Scales 1st..6th = 0.5/0.65/0.8/0.9/1.0/1.0, all ≤ 1
  (never looser than stock — the brief's "looser in high gears" is read as "back to stock"). Rise always scaled; fall
  scaled only through the power→coast span (a_cruise above the credited engine-brake level and the target within it),
  stock when braking is needed / FCW / urgent lead. Only the **cruise candidate** is shaped: the lead MPC (its jerk cost,
  and therefore FCW) is untouched, so lead-following braking is exactly stock. `TODO(on-car)`: the scales by feel.
- **THE RISK (must validate on-car with margin before relying on it):** suppressing ACC braking in favor of engine
  braking changes follow-distance maintenance. If the model **over-estimates** engine braking, the car coasts and
  **closes on a lead before the hand-off to the brakes**. The lead-aware + capacity backstops are what keep it safe.
  Sim numbers (simple car model, lead easing off at 0.3 m/s²): true engine braking 100 % of model → same gap as stock;
  50 % → no braking needed, settles at the MPC's desired gap (37.5 m vs. 42.5 m stock); **20 % → min gap 35.9 m vs. 38.0 m
  stock on the same car**, braking resumes (lead closing-speed trigger, then capacity), no FCW. Until validated: OFF.
- Verified by: 40 new tests — 20 closed-loop EBP (planner → NSB floor → EBP → LoC → manual car that engine-brakes on a
  closed throttle; gentle set-speed trim and gentle follow trim coast with **zero** brake-range commands where stock
  brakes; fast-closing lead and a lead that brakes hard are **bit-identical to stock**; demand past the floor/plan horizon
  passes; toggle OFF bit-identical to the code removed; over-estimate behind a lead; "0 holds speed" caught; downhill
  grade; no pitch/no gear/forceDecel/FCW/AEB/invalid inputs/stopping; NSB composition; latch; hold; params clamped),
  12 model/logger, 8 GearAwareAccel; **24 safety mutations all caught** (against a green baseline); self-referenced
  plannerd + controlsd replays: toggles OFF **0 diffs**, EBP+GAA ON 0 diffs (CI segments are automatics); manual
  integration replay `tools/engine_brake_replay.py` (Nidec Civic segment re-fingerprinted as a manual, RPM injected):
  OFF identical; EBP ON changes exactly 53 frames, every one a clamp to the coast target, none during any urgency
  condition; GAA (synthetic 3rd-gear trace) changes only aTarget/trajectories (≤ 0.22 m/s²), FCW identical.
- Deferred to the car (do NOT do before then): calibrate the model from coast logs; tune thresholds, coast target and
  jerk scales by feel; road-validate the backstops — **especially the lead-approach hand-off** — with wide margin.

**Task 8 — clutch-gate smart re-engagement** (openpilot `971148479` on the feature branch, merged into dev-test
`93de97ce3`; opendbc unchanged). Brief: `.../CommaAI/prompts/task8-clutch-reengagement.md`. Code:
`sunnypilot/.../clutch_reengage.py` (RatioLock, TakeoverRamp), `clutch_shift_gate.py`, wired via
`ControlsExt.apply_clutch_takeover` / `post_clutch_takeover` in `controlsd.py` (after the NSB/EBP floors, before the
handback). Manual + `ClutchShiftGate` ON; `DisengageOnAccelerator` OFF as before.
- **`ClutchReengageSmart`** (default OFF; OFF = the fixed `REENGAGE_DELAY`, bit-identical — the manual replay proves it):
  - **8a ratio lock** replaces the 0.5 s guess: coupled = `infer_gear(rpm, v, clutch, ClutchReengageRatioTol)` on one
    gear for 3 frames **and** the measured ratio steady (≤ 1 % spread). The steadiness rule was added after a test
    showed a slipping clutch sweeping through the **adjacent** gear's ±8 % band (85 % of 4th's rpm reads as 5th).
    `GEAR_MT` reporting a gear stays required (pre-cue): rolling in neutral at idle passes through a speed where idle
    rpm matches 6th's ratio (~7.5–8.8 m/s). Below max(3 m/s, handback speed) the fixed delay is the fallback. A locked
    ratio also blocks the RPM-flare backstop (found by the replay: the smart takeover no longer waits out a
    downshift's re-coupling, and the tail of that RPM rise re-paused the gate from idle).
  - **8c one takeover rule:** coupled AND gas released for `ClutchReengageGasDebounce` (0.15 s). Off-throttle /
    downshift → immediate at the bite; on-throttle upshift → waits for the lift (the gate stays in `reengaging`
    instead of going idle on gas). At the takeover the command is **seeded from `aEgo`** (clipped to the already
    NSB/EBP-narrowed limits; a positive seed is capped at `max(aTarget, 0)` — never throttle the plan doesn't want;
    a negative seed after a downshift is kept — no throttle forced on against engine braking) and **slewed** to
    LongControl's own output at planner set-speed jerk × `JERK_SCALE_BY_GEAR` (toggle-independent), escalating
    linearly to 3 m/s³ by `ClutchReengageMaxRamp` (1.0 s) so it catches a plan that keeps moving (first design closed
    the remaining gap as gap/remaining-time — that spiked to 20 m/s³ at the end; a test caught it). Applied as accel
    limits, so the PID anti-windup holds the integrator; **the integrator is deliberately not seeded** (seeding
    `i = seed − aTarget` would make the PID hold the seed and unwind at the ki rate). Ends on convergence, max ramp,
    long inactive, LoC not in pid, or **urgent** (close/closing lead, FCW, stock AEB/FCW, forceDecel, aTarget < −1.5,
    shouldStop, invalid plan/radar → needed braking passes at once).
- **8b `ClutchShiftGateMaxCoast`** (8 s, 2–20, active whenever the gate is on, not a feel knob): uncoupled longer than
  that → new state `coastTimeout`: **not paused** (openpilot can brake again), command capped ≤ 0 while `GEAR_MT`
  reads 0 (throttle would only rev a disconnected engine), back to `reengaging` when the clutch re-engages. `GEAR_MT`
  in gear but no ratio lock (smart; wrong seed ratios / slipping clutch) → idle + warning. `clutch_shift_gate_timeout`
  cloudlog. A real shift never gets near it, so the smart-OFF path is unchanged in every replay.
- *(Task 12 B4 correction: "bumpless / no step at resume" was claimed from `cmd ≈ aEgo`; the rewritten test measures the
  plant instead — peak plant jerk around the resume, open plant: fixed 2.05, smart-aego 3.38, smart-coast 2.75 m/s³;
  closed plant: 1.41 / 1.52 / 1.34 m/s³. Open + aego is the downshift brake-tap case on-car test C2 decides.)*
- **What the old path actually does (honest baseline):** in closed loop with the real planner, the fixed-delay resume
  is already close to bumpless in non-adaptive cruise — the planner re-seeds `a_cruise` from `aEgo` while LoC is off,
  so the first command is within ~0.05 m/s² of `aEgo` (sim downshift: −0.425 vs −0.472). The OFF "+0.80 step" in the
  controlsd-only replay is a replay artifact (the logged plan isn't re-seeded). Smart's real gains: takeover at the
  bite instead of 0.2–0.5 s later (no uncommanded coast hole), the on-throttle rule (no grab/yield flutter), exact
  seed continuity regardless of which plan candidate wins (MPC/e2e aren't jerk-limited from `aEgo`), bounded command
  rate through the bridge, and the neutral-coast bound.
- `TODO(on-car)`: C2 decides the downshift seed — if a negative `ACCEL_COMMAND` adds service brakes on top of engine
  braking (instead of meaning "decelerate like this"), the downshift hand-back feels like a brake tap → seed from
  max(aEgo, coast estimate) or 0. Also: ratio steadiness vs real ENGINE_RPM / wheel-speed noise (S3); a slow,
  near-synced slip can read as locked slightly before the bite completes (sim: ≤ 0.25 s early); tune tolerance,
  debounce, max coast, max ramp and the per-gear rates by feel; A/B against OFF.
- Verified by: 42 unit/closed-loop tests (ratio lock incl. adjacent-gear sweep, takeover rule, flutter, low-speed
  fallback, timeout/coastTimeout, ramp seed/slew/max-ramp/urgent/floors, controlsd wiring, shift sim with the real
  planner + NSB/EBP); **23 safety mutations all caught**; self-referenced controlsd + plannerd replays **0 diffs**
  (OFF, gate ON, gate+smart ON — CI segments are automatics); manual integration replay
  `tools/clutch_reengage_replay.py` (re-fingerprinted Nidec Civic, synthetic up/down/on-throttle shifts + a neutral
  coast): smart OFF bit-identical to a pre-Task-8 capture; ON seeds at aEgo, ≤ 0.02 m/s² per frame after resume,
  downshift resumes at −0.80 and ramps (no throttle into the engine braking), on-throttle waits for gas-clear +
  0.15 s exactly; neutral coast times out at 8 s with the command ≤ 0. Phase 3 and Task 6 replays unchanged.
- **Test console (G1–G4, H3):** run G1–G4 with `ClutchReengageSmart` **OFF first** (validates the placeholder gate
  the frame-acceptance result depends on), then repeat G2/G4 with it ON for the A/B — G2's resume-jerk and G4's
  "throttle added after re-engage" are the comparison numbers; S2's bite lag shows how much earlier the ratio lock
  hands back. H3 should now end in `coastTimeout` after 8 s instead of coasting indefinitely.

**Task 11 — no-service-brake overspeed brake pulse** (openpilot `7b3c6a77c` on the feature branch, merged into dev-test
`f892a32b4`; NSB2 console scoring dev-test `49c3de5de`; opendbc unchanged). Brief:
`.../CommaAI/prompts/task11-overspeed-brake-pulse.md`. Code: `no_service_brake.py` (`NoServiceBrake` pulse state machine,
`PulseFloor`), the planner SP mixin (urgency, alert, publish), `ControlsExt.apply_no_service_brake`. Transmission- and
platform-agnostic, like the rest of NSB. **Timing per the brief: tune only after C1 and the `maintain` NSB2 run.**
- **`NoServiceBrakeOverspeedMode`** = `maintain` (default: the Task 4 backstop, bit-identical — controlsd never reads the
  new message in this mode; pinned by a verbatim copy of the old state machine, the closed-loop sim and replays) |
  `pulse`. Params (clamped; release < trigger < ceiling enforced by tightening the trigger): trigger = the existing
  `NoServiceBrakeOverspeedMargin` (3.6 m/s; *Task 13 A2: capped at a fixed 6.7 m/s and 0.45 m/s under the ceiling, in both
  modes — it used to follow the ceiling param up to 11*), `…ReleaseMargin` 0.9 m/s (0–3.6), `…PulseDecel` −1.0 m/s² (−2.5…−0.3),
  `…PulseJerk` 1.5 m/s³ (0.5–3.0), `…HardCeiling` 6.7 m/s above set (4.5–11). Mode latched only while long inactive.
- **Pulse:** COASTING (the coast clamp) until set + trigger → PULSING: controlsd moves LongControl's lower accel limit at
  the jerk down to the pulse decel, until set + release → the limit rolls back up at the same jerk → COASTING. Past set +
  ceiling while pulsing → ESCALATED: full openpilot braking *(Task 12 correction: in non-adaptive this is the cruise
  plan's −1.2 m/s² cap, not "full" braking; and the alert now repeats every 4 s while still over and rising)* + a one-shot **"Overspeed: Take Over"** alert (new
  `OnroadEventSP.noServiceBrakeOverspeed`, fork-only **`custom.capnp` @28**) until under set + release or disengaged.
  Close/closing lead (EBP's `lead_urgent`: TTC < 6 s, closing ≥ 1 m/s, < 1 s / 10 m, lead braking > 1 m/s²), planner FCW
  or stock AEB/FCW → **urgent**: full braking (pulse mode lifts the coast clamp too; *Task 12: stock AEB/FCW is dead on the
  radarless Civic while openpilot long runs — kept for other platforms; urgency is now the shared Rule-5 predicate*); forceDecel turns the whole clamp
  off as before; an invalid radarState invalidates the plan → braking allowed. Transitions logged as
  `no_service_brake_pulse` (phase, prev, overspeed, v_set) — what NSB2 scores.
- **Placement (Task 4's split):** plannerd decides the state + target floor and publishes them in a new fork-only
  **`longitudinalPlanSP.noServiceBrake` (`custom.capnp` @8)**; controlsd (now subscribed to `longitudinalPlanSP`) ramps
  its floor (`PulseFloor`, seeded at min(last command, 0) so an escalation/urgent brake rolls off instead of stepping).
  PULSING publishes `allowBrake` True, so a missing/invalid plan-SP message falls back to **full braking**, never the
  coast clamp. Both ordinals may collide on a future sunnypilot rebase (like Task 5's @26/@27).
- **What it does and doesn't buy (sim, force-additive car, 160 s descents) — honest A/B:** `maintain` already cycles on a
  moderate grade (0.6 m/s²: set+1.8 ↔ set+3.6) but **steps** the command ~0.9 m/s² at each boundary (18 m/s³); on a
  grade steeper than its −1.2 can hold it rides the brakes continuously at set+3.6. `pulse`: command jerk ≤ 1.5 m/s³,
  about half the brake events, longer coasts — but brake **duty is not lower** in this model (0.6 grade 43 % vs 34 %;
  0.9 grade 91 % vs 66 %), because the bound is −1.0 vs the plan's −1.2 and it brakes down to +0.9 instead of +1.8. If
  the car's brake controller tracks the requested decel (the Honda VSA likely does), a −1.0 pulse gives −1.0 actual
  whatever the grade → short, distinct pulses; that's the on-car question. "Full openpilot braking" in non-adaptive is
  the set-speed plan's **A_CRUISE_MIN −1.2 m/s²** (this car's PID has ki = 0, nothing winds past the plan) — the same
  as `maintain`, so `…PulseDecel` stronger than −1.2 has no extra effect, and a grade steeper than −1.2 outruns both
  modes in the sim (escalation still alerts).
- `TODO(on-car)`: tune band / decel / jerk after C1 + NSB2 `maintain`; does a −1.0 command actually decelerate at −1.0
  downhill?; lowering the set speed by more than the ceiling while over it escalates at once with the alert (nuisance?).
- Verified by: 29 tests (state machine, hysteresis/no chatter, escalation + exactly one alert per episode, urgent bypass,
  forceDecel, latch, clamps, `PulseFloor` ramp/seed/bounds/fail-safe, wiring through serialized plan messages, downhill
  sims: sawtooth that starts at +3.61 and releases at +0.88–0.90 with no steps; escalation on a grade the pulse can't
  hold); **27 safety mutations all caught**; self-referenced controlsd + plannerd replays **0 diffs** in `maintain` (toggles
  OFF and non-adaptive + NSB ON); `pulse` on the CI segments changes 4 plannerd frames on the op-long one, all the urgent
  (close lead) bypass. Console: NSB2 extended for pulse mode (see `docs/TEST_CONSOLE.md`); session 8 offers the mode.

**Task 7 — on-car test console** (openpilot `dfc8a0ed2`, **dev-test branch `civic-6mt-dev-test` ONLY** — never on the
feature branch, never merged back, never upstreamed; opendbc unchanged). Operator guide + the full generated registry:
`docs/TEST_CONSOLE.md`. Brief: `.../CommaAI/prompts/task7-test-console.md`.
- **★ It is the FIRST thing to run on the comma four:** mount + install → flash **dev-test** → run the characterization
  tests (phases 0, 1, 2 + the phase 7 calibration captures E0c/E0a/E0b) → only then the feature phases, in order.
- **Zero driving authority** (same safety class as the Task 6 coast logger): `test_consoled` (`only_onroad`, default
  priority) subscribes and writes files; it publishes no cereal message, sends no CAN, writes no Param, changes no
  toggle (tests list the toggles they expect; the driver sets them). Proof: structural tests + a **multi-process
  process replay of controlsd and plannerd with the real `test_consoled` running armed + background on all 4 CI
  segments: 0 diffs** vs a capture at `sp-honda-dev-202608` (`tools/test_console_replay.py replay`).
- **Pieces** (`openpilot/sunnypilot/selfdrive/test_console/`; split into a vehicle-agnostic core + the Civic profile by
  Task 10 — the Civic plan now lives in `profiles/civic_6mt/`): `registry.py` (the plan as data: id, phase, kind, how,
  conditions, expected toggles, analyzer + goal, result rule, pass text, recorded fields, prereqs — add a test by adding
  a row), `conditions.py`, `signals.py` (`Sample` per carState frame; raw CAN tap on Bosch-radarless only: `GEAR_MT`
  value, `PEDAL_GAS`, `POWERTRAIN_DATA` payload, `ACC_CONTROL` as sent; feature cloudlog events via `logMessage`),
  `analyzers.py` (segments / episodes: shift, handback, stop-hold, neutral coast, backstop, EBP trim, lead approach,
  brake press; PASS / FAIL / CAPTURED rules), `capture.py` (arm → button start/stop/resume → complete; markers =
  cloudlog `test_marker` + rotating `/data/test_console/markers.jsonl`; per-window capture JSONL; `results.json`
  persists across drives), `button.py`, `status_io.py`, `test_consoled.py`. UI (comma four = "mici", Python/raylib —
  the brief said Qt, this fork has none): settings **dev test** panel grouped by phase with soft prereq hints (never
  blocks arming), per-test page (how, toggles to set, pass rule, live ✓/× conditions), onroad status chip.
- **IPC (schema-light, no `custom.capnp` ordinal):** Params `TestConsoleSelected` / `TestConsoleArmed` (cleared on
  manager start) / `TestConsoleBackground` / `TestConsoleStartButton` (default `gapAdjustCruise`) /
  `TestConsoleStartPress` (`short` default | `triple` | `long`); status in `/data/test_console/state.json`.
- **Start button:** reads `carState.buttonEvents` only, native function untouched — a short gap press also steps the
  following-distance personality (irrelevant in non-adaptive; use `triple` = net-unchanged for adaptive tests like E2);
  a ≥ 0.5 s hold also toggles Experimental mode on this car (so `long` is not recommended).
- **Ordered registry (29 tests at Task 7; 30 with Task 9's S7; ★ first-run):** P0 signals ★ S1 GEAR_MT decode, S2 GEAR_MT lag (bite vs
  `REENGAGE_DELAY`), S3 RPM sanity/flare noise, S4 gas-lift→clutch (vs `GAS_RELEASE_WINDOW`), S5 rev-match trace, S6
  clutch-bit hunt (optional) · P1 ★ D1 baseline op-long drive (Task 1 regression: no CRUISE_FAULT / blocked TX / fault
  alerts) · P2 ★ C1 brake strength (PR #2050), ★ C2 command-0 / small-negative semantics · P3 (gate ON, DOA OFF) G1
  frame acceptance (FIRST), G2 real shifts (no throttle with clutch in, smooth resume, no false flares), G3 rev-match +
  brake still disengages, G4 engine-brake downshift (tuning) · P4 N1 non-adaptive hold / no braking for a slower lead,
  N2 over set downhill · P5 NSB1 coast over set (floors 0.3 and 0.0), NSB2 overspeed backstop · P6 H1 handback feel,
  H2 Auto Brake Hold / hill-hold (no rollback), H3 GEAR_MT neutral-while-rolling, H4 uphill handback · P7 ★ E0c accel
  sweep (gear-ratio clustering = HARD prereq for Task 6), ★ E0a downhill coast, ★ E0b flat coast, E1 EBP gentle trims,
  E2 EBP lead hand-off (optional, needs a helper car), E3 EBP downhill · P8 GA1 gear-aware feel, A7 uphill accel per
  gear. Telemetry tests (no toggle) are order-independent and can run in the background.
- Verified: 43 unit tests (predicates fire only when they should, progress/goal and PASS/FAIL logic, markers/captures/
  results, default-inert, button gestures, real swaglog parsing, raw CAN tap on packed CIVIC_2022 frames, structural
  no-actuation, UI glyph coverage) + the 0-diff multi-process replay + an offline run of the daemon's classes over the
  HONDA CI log with an injected button press (`tools/test_console_replay.py offline`, ~110 µs/frame on the PC);
  headless renders of the panel/page/chip at 536×240.
- `TODO(on-car)`: the gap button decodes (panel shows "seen N×"); CPU on the C4; no confirmed radarless
  "service brake applied" signal — C1/C2/N2/NSB1/E1 judge braking from `aEgo − coast estimate` (uncalibrated until
  E0b) + the sent `COMPUTER_BRAKE_ASSIST` bit, so those verdicts are provisional (raw rows are kept to re-score);
  provisional thresholds (resume jerk 3, handback jerk 2.5 m/s³, rollback 0.3 m/s). *(Task 13 B1: "uncalibrated until E0b"
  became "until E0b's capture is FIT": E0b now fits a per-gear coast correction and the verdicts judge against it.)*

**Task 9 — session-based console UI + plan re-cut** (openpilot `fed215ab3`, **dev-test branch `civic-6mt-dev-test`
ONLY**; includes the Task 7b signature hunts, which hadn't landed and Task 9 depends on; opendbc unchanged). Briefs:
`.../CommaAI/prompts/task9-session-ui.md`, `task7b-signal-hunt-patterns.md`. Operator guide + generated tables:
`docs/TEST_CONSOLE.md` (regenerate with `tools/test_console_doc.py`).
- **Sessions** (`registry.SESSIONS`, since Task 10 `profiles/civic_6mt/registry.py`; every `TestSpec` has `session`): 1 Signals parked (S1 S6 S7) · 2 Signals shifts
  (S2–S5) · 3 Baseline (D1) · 4 Command authority (C1 C2) · 5 Clutch gate (G1 G2) · 6 Clutch feel (G3 G4) · 7 Non-adaptive
  (N1 N2) · 8 No-service-brake (NSB1 NSB2) · 9 Low-speed handback (H1–H4) · 10 Engine-brake pref. (E1 E3, E2 optional)
  · 11 Gear-aware accel (GA1 A7) · **Cal** (E0c E0a E0b — a progress card filled by background telemetry). Required
  toggles per the brief (session 8 also requires the backstop ON, which NSB2 checks; sessions 5/6 *offer*
  `ClutchReengageSmart` / gas-release hint for the Task 8 A/B; session 8 offers the NSB floor 0.3 ↔ 0.0). Session prereqs
  are tested against the tests' own prereqs (transitive closure): 2←1, 4←3+Cal, 5←2+3, 6←5, 7←4, 8←7, 9←5+7, 10←4+Cal,
  11←Cal, Cal←2. Soft hints only (locked tiles), never a block.
- **7b:** S6 = clutch signature N=3 parked in neutral, auto-scored on 0x17C bits (3 groups of 2N toggles) plus the old
  GEAR_MT correlation when rolling; completes at 9 presses or a 45 s window. S7 = volume signature N=4 then N=5, a 90 s
  marked window (likely B-CAN → scrub cabana; no multi-bus auto-scan — CPU on the C4 for a likely-absent signal).
  S6/S7 are `background=False` (one-line filter in `Console._update_background`; a background run would "complete" them
  on any idle). 30 tests now.
- **UI** (`dev_test_sessions.py`, raylib, 536×240): session home grid (done ✓ / current / locked with gating session,
  next-session ring, Cal card, "phases ▶" → the Task 7 phase view, still there) and swipeable session pages (◀ ▶ edge
  taps, dots): header + objective, switches with live state + **ready ✓ / set N**, test pills, selected test's progress /
  failing conditions / ARM / "how ▶" (Task 7 per-test page), start-button chip (**STOP • ● REC** while capturing). Pure
  derivations in `test_console/sessions.py` (unit-tested).
- **UI may set toggles (Part C):** one writer, `dev_test_params.write_param`, refuses anything outside
  `sessions.UI_WRITABLE` = the brief's 11 feature toggles + `TestConsole*` (Task 11 added `NoServiceBrakeOverspeedMode`;
  since Task 10 the list is the vehicle profile's, `sessions.ui_writable(profile)`); **not** `AlphaLongitudinalEnabled` (shown as a
  requirement, set in Developer — enabling op long disables stock AEB). Structural test: no direct Param write or
  Param-bound widget in any dev-test UI file. "applies when disengaged" shown onroad (features latch only while long is
  inactive; `DisengageOnAccelerator` is read live by selfdrived). The daemon still writes nothing. *(Task 12 D2
  correction: that note wasn't true — latching followed "long inactive", and writes were allowed while engaged. Now
  features latch only while disengaged, the UI writes feature Params only while disengaged + stopped, and the note
  reads "set when disengaged + stopped".)*
- Verified by: 31 new tests (+ updated Task 7 tests: 30 tests, UI write-path structure, glyph coverage); dev-test suite
  501 passed; **daemon multi-process replay 0 diffs** (controlsd + plannerd with `test_consoled` armed + background, all
  4 CI segments, vs the feature-branch capture) and the offline run (115 µs/frame); headless renders of the home grid
  and session pages 1/3/5/8/Cal at 536×240 plus injected-touch checks on the real widgets (switch tap writes, op-long tap
  refused, ARM, swipe) — `tools/dev_test_render.py`.
- `TODO(on-car)`: toggle-set-from-UI on the device; layout at the real screen; regroup sessions if a 30-minute outing
  proves too full; S6 signature detection against real 0x17C chatter.

**Task 10 — test console: vehicle-agnostic core + per-vehicle profile** (openpilot `0a551a870`, **dev-test branch
`civic-6mt-dev-test` ONLY**; opendbc unchanged). Brief: `.../CommaAI/prompts/task10-platform-agnostic-console.md`.
Behavior-preserving refactor; architecture below ("Test console architecture"). *Timing:* the brief says to run it after
the single-car build is validated on-car so the harness isn't restructured mid-test; run now because no on-car testing
has started (nothing is mid-test) and the Civic's outputs are proven identical.
- **Core** (`test_console/*.py` + the dev-test UI) reads vehicle signals only through named roles (`profile.ROLES`);
  `Sample` fields renamed to roles (`gear_mt`→`gear_raw`, `raw_17c`→`raw_payload`, `pedal_gas`→`gas_raw`). Gear
  inference / coast model via the profile's `Powertrain`; condition labels take the profile's signal names
  ("{gear_raw} tap live" → "GEAR_MT tap live"); the plan's pass rules (`results`) are the profile's (the core keeps the
  generic `baseline`). The core imports no feature module that carries Civic data (two clutch-gate constants are mirrored
  and pinned by a test).
- **Civic profile** (`profiles/civic_6mt/`): signal map (clutch/rpm readers, the Honda raw-CAN tap `tap.py`, labels,
  sources, **column map** so capture files / markers keep `gear_mt` / `raw_17c` / `pedal_gas` — old captures stay
  re-scorable), powertrain = the features' own `engine_brake_model` functions, button map (default `gapAdjustCruise`,
  allowed = every button the Honda carstate emits), the plan (`registry.py`) + pass rules (`results.py`), allowlist,
  applicability (`HONDA_CIVIC_2022` **and manual**).
- **Selection** (`select_profile(CP)`): first registered profile whose `applies(CP)` → else **inactive**: the daemon
  subscribes to and writes nothing, the UI shows "no test-console profile for this platform (FP)" and can set only
  `TestConsole*` keys. Daemon re-checks `CarParams` every second and re-selects; the UI re-selects when
  `CarParamsPersistent` changes. `TEST_CONSOLE_PROFILE=<name>` forces one (replays/bench).
- **Deliberate behavior changes (non-Civic only):** a car with no profile used to run the Civic plan; now inactive. A
  `TestConsoleStartButton` name outside the profile's allowed list now falls back to its default (was: any ButtonEvent).
- **Stub profile** (`tests/stub_profile.py`, test-only, never registered): a 5-speed with no tap, `accelCruise` start
  button, two tests — runs the unchanged core (in a fresh interpreter that never imports the Civic profile or its model).
- Verified by: `tools/test_console_equiv.py` before/after on identical inputs (HONDA CI segment re-labelled a 6MT + a
  scripted 6MT drive; every test armed + background, NSB2 in both modes): markers, results, capture files, status
  snapshots and every Sample **identical in all 32 runs**; every session page + the phase view + a detail page
  **pixel-identical** (`dev_test_render.py`, `RENDER_ALL=1`); the 67 Task 7/9/11 console tests pass through the Civic
  profile (changes are mechanical only: imports, the Civic `ctx()`, the renamed role fields, the renamed condition
  `raw_can`→`raw_gear_live`, a header check through the column map; the structural no-actuation / glyph checks now also
  scan the profiles); 14 new tests (purity grep, selection, stub, per-profile allowlist, inactive daemon, CarParams
  change); daemon multi-process replay **0 diffs** (forced Civic profile — the CI segments aren't 6MT Civics; the inactive
  path can't be process-replayed since it subscribes to nothing, so a unit test covers it); offline 115–121 µs/frame
  (unchanged; label/condition lookups cached per context).

**Task 12 — review fix pack** (brief `.../CommaAI/prompts/task12-review-fix-pack.md`; an independent four-reviewer
review of every brief and changed line, refs feature `7b3c6a7` / dev-test `0a551a8` / opendbc `02ad781`). Headline it
fixed: three features could remove braking while engaged with no emergency release, and several tests encoded the same
wrong assumption. Feature branch `37d6f74d5`…`b2db15b12` (9 commits, one per item group + a tool fix; the suites pass at the
series tip, not at every intermediate commit), opendbc `0751f8e2a` (B5/A2.7) + `828d13b10` (README). Dev-test: see Part D below.
- **R5 — one urgency predicate** (see "Rule 5 / urgency predicate" above): `braking_urgency.py`, consulted by every
  limiter; matrix + AST tripwire in `test_rule5.py`.
- **A1 — non-adaptive keeps emergency collision braking.** Owner decision 2026-10-19 (now a requirement): *"have it
  slow only when the model would actually need to for real slow vehicles or collisions."* The lead MPC is re-admitted
  (plan = min(MPC, cruise), braking allowed, NSB released) on `collision_threat` / FCW / stock AEB / the MPC needing
  < −1.0 m/s² with a lead; 2 s dwell; the cruise candidate resumes from the collision braking. *Implemented differently*
  from the suggestion (re-admit on any R5 lead reason): those broad criteria would make it follow traffic. Stock
  `allow_throttle` kept; cluster `leadVisible` False without a threat (`hasLead` unchanged).
- **A2 — a shift never removes braking.** `longActive` no longer drops; only the throttle is capped
  (`ClutchShiftGateCoastAccel`, 0.0, −0.5…0); never positive with the clutch in; paused + gas = exactly 0 (panda);
  MaxCoast ≤ 5 s; RPM flare = 3 new samples, no gas, cool-down; smart no-lock fallback 1 s; selfdrived drops the gas
  override only on the published `gatePaused`; GEAR_MT ∉ {1–6, 14} = decoupled; DOA ON / no clutch signal → refuses
  (`gateBlocked`, HUD "Shift gate off"); HUD "Shift: coasting"; ramp never on the transition into coastTimeout, ends on a
  step brake request; the fixed path also ramps (from the command that ran through the shift).
- **A3 — the handback keeps emergency braking.** forceDecel blocks/aborts it; under urgency the ramp freezes with the
  lower limit at stock (may brake to a stop); prompt from the ramp start at Priority.HIGH (below FCW); controlsd owns and
  publishes the state, selfdrived only acts on it. A3.6 MADS keep-lateral investigated, not done (Open questions).
- **B1/B2 — NSB:** never caps a lower SCC-V/SCC-M/SLA target (overspeed vs the effective target); Rule 5 in both modes
  and both processes; margin ≤ hard ceiling; overspeed alert repeats every 4 s while rising, in both modes; 2 s grace
  after a set-speed drop. −0.3 floor (C2) and maintain's brake step left as-is, documented.
- **B3 — latching:** every feature toggle/param latches only while `selfdriveState.enabled` is false (not "long
  inactive"); EBP's capacity lockout lasts the engagement; latched values published (`longitudinalPlanSP.features` /
  `.noServiceBrake`, `carControlSP.manualLongitudinal`).
- **B4 — smart re-engagement:** lock gear must equal GEAR_MT (`CarStateSP.gearRaw`); lock tolerance 0.02 (0.01–0.04),
  separate from the ±8 % gear ID; steady 0.12 s on filtered signals; `ClutchReengageSeed` aego (default) | coast — C2
  decides. Seed-continuity test rewritten to measure plant jerk/decel (numbers in the A2+B4 commit / TOGGLES).
- **B5 — plumbing:** opendbc `GEARBOX_ALT_2` NaN frequency + `clutchSignalValid`/`gearRaw`, Integra RPM from
  `POWERTRAIN_DATA`; features inert without the signal; `all_checks` for plan/radar; coast-logger I/O on a writer
  thread; GearAware stock with no pitch. **Found by a test during Task 12:** `card` converts `carControlSP` to the opendbc
  dataclass every frame, so the new field would have crashed card on the car — `convert_carControlSP` now passes only
  known fields.
- **Implemented differently, with why (beyond A1):** R5's plan-derived reasons (`stopping`, `hard_decel`) count only
  from a live plan (LongControl not off), because while long is off the planner re-seeds from aEgo and a coasting car's
  own decel would freeze the handback forever. The gate's `limit_accel` takes urgency but changes nothing with it: it
  never raises the lower limit, and the gas-pressed 0 is the panda's rule.
- **C — disclosure:** README (both forks) says CMBS is off for the *entire drive* with openpilot long, and restores
  "whether the car even has brake authority". GitHub default branch: see the wrap-up below. Claims corrected inline above
  (Phase 1 "no new plumbing", latching "never flips mid-engagement", stock AEB/FCW bypasses dead on radarless, "full
  braking" on escalation = −1.2 cap, "no step at resume", the retired `clutch_gate_replay`).
- **E1 — evidence in the repo:** `tools/civic6mt/` in the openpilot fork (selfref / clutch_reengage / engine_brake
  replays, `mutate.py` + `mutations.py`, README). **The mutation counts quoted for Tasks 4–11 above came from ad-hoc
  scratch scripts that were never committed** — they are superseded by the committed manifest (49 mutations, every R5
  call site; 49/49 caught at the Task 12 tip).
- **Part D — dev-test console** (`civic-6mt-dev-test`: merge `15907fc8b`, then `a6c6a1566` D1+D2, `f272a6205` D3,
  `b9cb7a16a` D4+D5, `1eba6a205` + `274737927` tools; details in `docs/TEST_CONSOLE.md`):
  - **D1** selfdrived ignores `test_consoled`; daemon loop + every UI render/input path guarded (log once, keep running;
    daemon inert after 100 consecutive failures); NaN rpm crashed `status()` in every test (fixed); state/results JSON
    type-validated; core max allowlist + denylist cap every profile; allowlist keys checked against `params_keys.h`.
  - **D2** feature Param writes only while disengaged AND stopped (or offroad), refusal shown; a satisfied requirement
    isn't flipped; conditions use the PUBLISHED latched values (daemon subscribes `carControlSP` / `longitudinalPlanSP`);
    same-target taps < 20 px; session 9 requires the gate for H3.
  - **D3** no PASS before complete; C1 = braking beyond coasting, CAPTURED until E0b; NSB1/E1 provisional until E0b;
    S5 re-open / close-with-gas leaks; G2 needs 'shifting' + resume ≤ REENGAGE_DELAY + 1 s; N1 "FCW not exercised";
    S2 per-gear coverage ≥ 80 %. **A2 knock-on:** the shift analyzers read the published gate state (G1/G2/G3 rules
    rewritten — `long_active` staying on during a shift is now correct).
  - **D4** bit-hunt cache + 1 s tail, chip hidden over alerts, gap button triple/short only (triple recommended), no
    cancel/mainCruise start button, resolved button names, raw roles stale after 0.2 s, periodic capture prune; D-18 =
    the remaining fork reads in the core are listed in TEST_CONSOLE.md. **D5** `test_ui_smoke.py`.
  - Verified: dev-test suites 872 passed; `test_console_replay` 0 diffs on all 4 CI segments (controlsd + plannerd with
    the daemon armed + background, vs feature-tip captures); offline OK (115 µs/frame); `dev_test_render` 5/5;
    `mutate_devtest.py` 30/30.
- **Wrap-up:** both forks pushed (opendbc `828d13b10`, feature `b2db15b12`, dev-test `274737927`); **GitHub default
  branch switched to `sp-honda-dev-202608` on both forks** (`gh repo edit --default-branch`; the brief's preferred option
  — no README-only commit on `master`); bundles regenerated (7th regen, `backups/README.md`); docs/recovery re-mirrored.
  Nothing flashed; nothing merged toward any base.
- **Always-on changes (the exceptions to "OFF = stock"), listed for E2:** the low-speed handback's A3 changes (manuals
  with op long only); opendbc's NaN-frequency 0x1DD registration + the two CarStateSP fields (radarless manuals only;
  telemetry); `convert_carControlSP` filtering; Plant publishing `selfdriveState.enabled` (test harness). Everything
  else is behind a default-OFF toggle. Verified: self-referenced plannerd + controlsd replays with every toggle OFF,
  **0 diffs on all 4 CI segments** vs captures at `7b3c6a7`.

**Task 13 — fix-ups from the Task 12 verification** (brief `.../CommaAI/prompts/task13-verification-fixups.md`; an
independent verification of Task 12 confirmed the three braking-loss fixes and R5, and listed what was left open).
Feature branch `2627b961d`…`d05a17c70` (A1, A1 follow-up, A2, A3, C2), opendbc `434871715` (A3); dev-test merge
`5e8c0f000`, then `7c5153e01` B1, `780b46dea` B2, `c3f236056` B3, `68624b153` A2 knock-on, `505559d82` B4,
`e359cf749` B4 follow-up (per-site guard streak, isolation replay fixed — both found by C3), `3b04bc3dd` tools README;
then the feature → dev-test merge of the docs mirror.
- **A1 — the handback never holds a manual at a standstill in ordinary traffic [N-1, review2 N6].** Repro: a stopped
  car 40 m ahead at 3 m/s → the handback started at 10.4 m / 2.2 m/s, froze on `lead_closing`, openpilot braked to a stop
  and held the manual engaged for good; a 1 m/s lead kept openpilot crawling a manual. Now `handback_urgency()` (see
  Rule 5): FCW / stock AEB / forceDecel / hard request / invalid pass through; `stopping` and the broad lead reasons
  don't; an *imminent* lead (kinematic: a driver at 2 m/s² after ramp + 0.1 s + 1.5 s couldn't stop 2 m short) does and
  latches until the stop; **standstill resolution** — stopped 0.5 s while urgent → hand back (FCW/AEB excepted).
  Implemented with a kinematic test instead of the suggested "TTC AND gap" so it scales with the handback speed (≤ 8 m/s).
  Tests assert end states (handed back, never engaged at standstill, stopped short with the modelled driver, no contact).
  The R5 matrix gained `lead_imminent` and the documented `HANDBACK_CONTINUES` set. Changed on purpose because A1 changes
  the requirement: `test_throttle_start_then_lead_stops` (its 6 m-at-0.85 m/s lead isn't imminent; now 4 m at ~2 m/s),
  `test_adaptive_lead_stop_nsb_inert` (the handback's own urgency), `test_neutral_coast_toward_stopped_lead_adaptive`
  (hands back 8.9–12.3 m short at 2.2 m/s; ManualCarSim gained the post-handback driver model).
- **A2 — overspeed margin capped at a fixed 6.7 m/s [N-2, N3]:** `min(configured, 6.7, ceiling − 0.45)` in both modes;
  closed-loop test with margin and ceiling at their max: braking begins by set + 6.7.
- **A3 — robustness [review2 N2, N5, GearAware frame timing]:** (1) opendbc: a 0x1DD with a bad counter or checksum
  reads `clutchSignalValid` False (counter checked in carstate, the parser's counter check off for 0x1DD so it can never
  invalidate CAN; parser `last_rejected_nanos` for checksum drops, 100 ms hold); end-to-end test through the real opendbc
  carstate: canValid True throughout, gate/EBP/GearAware inert. (2) GearAware: plannerd keeps the stock-jerk shadow of
  the set-speed candidate and publishes the stock plan (`features.aTargetStock` @5, only while on); controlsd
  (`ControlsExt.plan_a_target`) runs `min(aTarget, aTargetStock)` in any control frame with a Rule-5 reason — implemented
  differently from a controlsd clamp because GearAware has no limit to release. (3) NSB broad release: documented, not
  changed (fail-safe direction; TOGGLES).
- **C2 — R5 tripwire parses the call [review2 N4]:** taint pass (predicate call / `*urgen*` parameter / class attribute
  assigned from one, reaching a decision); planted modules (docstring + unused parameter, dropped call, stored-never-read,
  other attribute, logging-only) FAIL. Its first version tainted `self` itself — found by the tripwire-only mutations.
  `ClutchShiftGate.limit_accel` is now an explicit EXEMPT entry pinned to the panda gas-0 test.
- **Part B (dev-test):** **B1** E0b fits a per-gear coast correction; C1/NSB1/E1 judge against it and can't PASS without
  it; the sent accel is in the verdict (a NaN-as-clean bug in NSB1 found on the way). **B2** S5 links both halves of a
  split clutch event (pre-blip leaks FAIL); gas at release is legal. **B3** caught errors visible (state, chip, detail),
  runs that lost frames can't PASS, inert says so (a bookkeeping call that reset the failure streak — found by a test).
  **A2 knock-on** the console reads the NSB values through `read_nsb_params`. **B4** `test_console_equiv` triple presses +
  published state + A2 shift semantics + exits 1 without real captures (it was also broken by Task 12's lint commit);
  `test_console_replay` feeds the console `carControlSP`/`longitudinalPlanSP`, asserts it ran clean, `isolation` mode.
- **C1 — docs checkable:** CLAUDE.md / TOGGLES.md / TEST_CONSOLE.md mirrored into the forks (`tools/civic6mt/docs/`,
  TEST_CONSOLE on dev-test only) and in the bundles; remaining uncorrected Task 12 claims fixed inline.
- **C3 — run, with numbers:** run 2026-10-19 on the PC (WSL2), not on a device and not in CI — numbers below. *The "767 / 872" counts quoted for
  Task 12 came from a suite definition that was never recorded; they are superseded by these, with the exact sets:*
  `CORE` = `openpilot/sunnypilot openpilot/selfdrive/controls openpilot/selfdrive/selfdrived opendbc/car/honda opendbc/can`,
  `WIDE` = `opendbc/car opendbc/safety/tests/test_honda.py openpilot/selfdrive/car/tests`.
  - Feature tip: CORE **874 passed**, 41 skipped, **1 failed** — `sunnypilot/models/tests/test_default_chestnut.py`
    (`UnicodeDecodeError` in the model-downloader test; code Task 13 never touched; it fails identically at the Task 12
    tip — pre-existing, environment-specific); WIDE **2196 passed**, 994 skipped. Dev-test tip: CORE **997 passed** (same 1
    pre-existing failure), WIDE 2196 passed.
  - Self-referenced replays, every toggle OFF, Task 13 feature tip vs captures at the Task 12 tip `b2db15b12`:
    **plannerd 0 diffs and controlsd 0 diffs on all 4 CI segments** (HONDA, HONDA2, TOYOTA, HYUNDAI).
  - `clutch_reengage_replay check` vs a Task 12 capture: PASS; `engine_brake_replay`: PASS.
  - **Mutation sweep, real MPC: `mutate.py` 62/62 caught** — the 49 Task 12 entries (incl. `a1_no_threat`,
    `a1_no_cruise_reseed`, `a3_force_decel_ignored`, which the verifier's static harness could not confirm) + 13 Task 13.
    Dev-test `mutate_devtest.py`: 45/45 after the fixes below (the first sweep was 43/44: `d3_c1_uncalibrated_pass` had a
    stale pattern after B1; re-pointed).
  - Console (dev-test): `test_console_replay replay` **0 diffs** controlsd + plannerd on all 4 segments with the console
    fed `carControlSP`/`longitudinalPlanSP`, and its state.json clean (armed, 0 errors, not inert); `offline` OK;
    `test_console_equiv` every armed test captured; `dev_test_render` PASS. `isolation` (HONDA): fault on every console
    frame → controlsd 0 diffs / plannerd 0 diffs, console INERT after 100 errors; selfdrived with test_consoled dead = alive
    (0 diffs), control with a non-ignored dead process 23 585 diffs. **Fails if isolation is removed:** without the
    `test_consoled` ignore → FAILED (23 585 diffs); without the guard → the console dies and the replay times out → FAILED.
  - **Found by C3 (fixed):** the daemon guard's failure streak was global, so the drain step succeeding every loop reset a
    frame step failing every frame — a persistently broken console never went inert on the car (the B3 claim). Now per
    site. And the first isolation selfdrived check couldn't fail: `replay_process`'s migration rewrites every
    `managerState` to "all running", erasing the injected dead process (control 0 diffs) — the tool now injects after it.
- **Wrap-up:** pushed opendbc `434871715`, feature `f96bb93af`, dev-test `cf3d8d82d` (merges one way, feature → dev-test;
  the fork's `.gitignore` ignores `CLAUDE.md`, so the mirror is force-added — `06175554f` had silently missed it; the fork
  copy lags this wrap-up line only); bundles regenerated (8th regen, `backups/README.md`), Dropbox copies + `docs/`
  re-mirrored. No environment change (`recovery/` unchanged). Nothing flashed; nothing merged toward any base.
- **New on-car unknowns:** how often a 0x1DD counter/checksum fault drops `clutchSignalValid` on the real bus; whether the
  handback's imminent-lead thresholds (2 m/s², 1.5 s, 2 m) match this driver's stops; the E0b per-gear additive fit may
  need an rpm term once real coasts are logged.

**Task 14 — handback latch fixes + verification follow-ups** (brief `.../CommaAI/prompts/task14-handback-latch-fixes.md`;
a three-reviewer, execution-based verification of Task 13 confirmed the standstill-hold regression gone and A2/A3/C2/the
console fixed, and found the Task 13 imminent-lead latch defective both ways). Feature `09f209292` A2, `390a3a9b5` A1,
`f8274b1be` A3, `267c8fe8f` A4, `4346060a0` B1, `87472bfbb` W1-N7; dev-test merge `ef161db38`, `ea9bfb2c8` C1.
- **A1 [W1-N1] latch releases when the lead is gone:** held only while a lead is closing within the imminent test + 2 m;
  0.5 s without → released, handback completes in ramp + margin. Repro (Task 13): a latched slow lead pulls away → still
  engaged, rolling 0.6–1.2 m/s after 60 s.
- **A2 [W1-N2] latch sets independently of the returned reason:** `handback_lead_imminent()` every frame. Repro: FCW held
  until 0.6–1.0 m/s → Task 13 ramped at 0.58–0.96 m/s, 3.7–4.3 m behind a stopped car (one contact, no driver in that sim).
- **A3 [W1-N3]:** FCW / stock AEB urgent even after a standstill resolution.
- **A4 [W1-N4]:** a downhill grade (calibrated pose) lengthens the imminent distance; uphill / no pose = level. The
  discriminating proof is the unit test — closed-loop sweeps are identical with and without it.
- **B1 [W1-N8, W2]:** `t13_a1_no_imminent_latch` now fails the hard-braking latch test (and the Task 13 latched test);
  `ForcedMpc` stand-in pins the non-adaptive re-admit and cruise re-seed at planner level (`t14_b1_*_forced` run only that
  class — no solver). **C1 [N13-1]** (dev-test): the coast-fit gate is per gear (E0b fits each gear once its own 10 s is
  in; verdicts gate on their own gears). **W1-N7:** the tripwire recognises `np.clip(limits[0], …)`.
- **Found while verifying (owner questions answered):** the handback test plant does NOT double-count grade (exactly one
  −g·sin(pitch) term — measured, and the `grade` / `pitch` knobs are now exclusive and pinned). On that correct plant the
  downhill contact survives — and it is **not** the handback or the non-adaptive cap: identical in all four of manual /
  automatic × adaptive / non-adaptive. It is braking authority (Open questions: "Downhill braking authority"); A4 changes
  neither outcome (48/48 force plant, 2/48 tracking plant, on and off). Not fixed here (authority change, needs C1 data).
- **Manifest hygiene:** `t13_a1_broad_urgency` / `t13_a1_imminent_ignored` had gone stale with A2 (fixed in A4's commit);
  `t13_a1_fcw_resolved`, `d3_c1_uncalibrated_pass`, `t13_b1_nsb_nan_is_clean` met a second protective layer and now remove
  both (the actual bug); every pattern in both manifests is checked to match exactly once.
- **Run (PC, 2026-10-19):** OFF replays vs the Task 13 tip `f96bb93af`: plannerd + controlsd **0 diffs on all 4 CI
  segments**. Feature CORE **883 passed** (+ the pre-existing `test_default_chestnut` failure), WIDE 2196 passed; dev-test
  CORE **1010 passed** (same 1), WIDE 2196. **`mutate.py` 70/70, `mutate_devtest.py` 48/48.** Console: replay 0 diffs on
  all 4 segments (console clean), isolation OK, offline OK, equiv every armed test captured, render PASS; clutch / engine-
  brake replays PASS.
- **Wrap-up:** pushed feature `4329294f2`, dev-test `1da48b74e` (opendbc unchanged, `434871715`); bundles regenerated (9th regen); docs re-mirrored (fork copies + Dropbox). Nothing flashed;
  nothing merged toward any base. **The low-speed handback is ready for stop-and-go-with-traffic testing** (after the
  empty-lot handback and Phase 0 signals); the downhill braking question is C1's.

**Task 15 — first pass from Stage 1 on-car data** (brief: the owner's Task 15 message, 2026-10-20; data:
`docs/stage1-findings-2026-10-20.md`, route `00000002--6c1a3bb7d4`). Goal: the fork ready for Stage 2 (op-long ON, every
custom feature OFF). opendbc `ec94b772a` (1), `d5af6598b` (2), `0ca828dde` (3); feature `dc4d4eec2` (schema + bump),
`2729be830` (4), `83f8e6173` (tools), `34604c568` (docs). **Pushed 2026-10-20** (opendbc `0ca828dde`, feature `aff9ce1d4`, dev-test `1a23dc9b3`);
the fork copy of CLAUDE.md lags this line only. Nothing flashed by Claude (owner installs dev-test). Every custom toggle still defaults OFF; non-adaptive untouched (Stage 4).
- **1 RPM source (blocker):** every Bosch-radarless manual reads `POWERTRAIN_DATA.ENGINE_RPM` (0x17C) — `ENGINE_DATA`'s
  is 0 on the car. Before this the ratio lock, flare backstop, EBP and GearAware were inert on the car.
- **2 clutch pedal switches:** `GAS_PEDAL_2` (0x130) `CLUTCH_PEDAL_UP` (bit 50, opens as the pedal starts to move) /
  `CLUTCH_PEDAL_FLOOR` (bit 51) added to `_honda_common.dbc` (comment: seen on this car only). New fork-only
  `CarStateSP.clutchPedal` (`custom.capnp` @3): **0 unknown** (the default, so an unset field never reads "pedal up"),
  1 up, 2 travel, 3 floor; unknown while 0x130 is missing / > 250 ms old or both bits are on. `deprecated.clutchPressed`
  **keeps its meaning for every consumer — "drivetrain decoupled"** — and now opens on the pedal switch first: pedal
  off the top OR `GEAR_MT` ∉ {1–6, 14} OR phantom. *Implemented differently from the brief's "wire into clutchPressed":*
  pointing clutchPressed at the pedal alone would make neutral-with-the-clutch-out read "coupled" (EBP would credit
  engine braking in neutral); `clutchPedal` is what separates clutch-in from neutral. The trigger is the top switch, not
  the floor switch (owner, 2026-10-20). Floor is decoded for telemetry only.
- **3 phantom-gear guard** (`PhantomGearGuard`, opendbc carstate): a gear first reported < 1100 rpm and < 13 m/s is
  untrusted (decoupled) until the rpm follows the speed at that ratio (expected Δ ≥ 80 rpm, actual within 35 %) or
  rises ≥ 1250; every new value judged afresh (sweeps go 2 → 1 without a 0). *Implemented differently from "rpm near
  idle AND speed low → don't trust":* that alone would also distrust real low-rpm engagements forever (openpilot can't
  throttle while untrusted, so it couldn't raise the rpm out of the band); the kinematic test releases those. A
  pedal-time rule was tried on the data and rejected (phantoms 0.02–0.8 s after pedal-up, real ones up to 0.98 s).
- **4 gate:** `REENGAGE_DELAY` 0.5 → **0.15 s** (settle debounce, restarted on a gear change while re-engaging);
  `NEUTRAL_CONFIRM` 0.5 s: decoupled with the pedal fully up = neutral → `coastTimeout` at once (no switch → MaxCoast
  only); `FLARE_HOLDOFF` 0.5 s: no flare may start within 0.5 s of decoupled (the old 0.5 s delay covered a downshift's
  re-coupling rpm rise implicitly — found by `test_downshift_rpm_jump_does_not_extend_shift`). **Knock-on:** smart
  re-engage (lock + 0.12 s steady + 0.15 s gas debounce) is now *slower* than the fixed path on this car; kept (OFF) for
  the on-throttle rule and the seeded ramp; its test now pins "at the lock, not the fallback".
- **5 coast model:** not refit (Stage 4). `EngineBrakePreferred` default OFF confirmed (`params_keys.h`); TOGGLES says
  keep it OFF until the refit.
- **Proof on the car's own bus:** `tools/civic6mt/carstate_route_replay.py <route dir> [--gate]` runs the real opendbc
  CarState (and optionally the real gate, ON + engaged as a what-if) over every segment. Stage 1 route: rpm never 0;
  pedal decodes (up 644 s / travel 93 s / floor 89 s); **all 29 phantom episodes flagged, 0.00 s of in-gear driving above
  idle read as decoupled**; gate: 81 episodes all started by a real decoupling (no flare / false start), resume median
  0.15 s / max 0.21 s after coupled, 15 neutral coasts to coastTimeout at once.
- **Run (PC, 2026-10-20):** CORE **902 passed** (+ the pre-existing `test_default_chestnut` failure), WIDE **2209
  passed**; ruff clean; mutations: 9 new Task 15 entries + 2 re-pointed (`b5_odbc_unknown_gear_coupled`,
  `b5_odbc_integra_rpm`) — **`mutate.py` 79/79**. Self-referenced replays, every toggle OFF, vs the Task 14 tip
  `4329294f2`: **controlsd, plannerd and card 0 diffs on all 4 CI segments** (card added because it runs the opendbc
  carstate; the CI segments are automatics, so the manual branch is exercised only by the route replay above).
- **Dev-test** (merge `f953cf842`, then `243747467` console follow-ups, `42fded473` S2 coverage test): S2's bite tracking started
  only after the 0.15 s clutch-close debounce, so a drivetrain already locked when the gear is reported (this car) read
  ≥ 0.16 s and **S2 could never pass against the new 0.15 s bound** — the lock is now tracked through the debounce
  (mutation `t15_s2_bite_from_debounce_end`); profile rpm / clutch source labels; S2 / G4 text 0.15 s. Found by the
  sweep: `d3_s2_no_coverage` had gone vacuous (its "good" gear's 0.3 s lag now fails the bound by itself) — re-pinned.
  Dev-test CORE **1029 passed** (+ the same 1), WIDE 2209; `test_console_replay` 0 diffs controlsd + plannerd on all 4
  segments (console armed, 0 errors), isolation OK, offline OK, equiv every armed test captured, render PASS;
  **`mutate_devtest.py` 49/49**.
- **New on-car unknowns:** cold idle must stay < 1100 rpm (phantom guard); S2 on the car reads the console's rpm-based
  lock, which may lag the real bite by a frame or two (Stage 1 analysis: 0.00–0.07 s); the pedal switch bits confirmed with the pedal
  moved slowly in cabana; does a 0.15 s resume feel abrupt (the takeover ramp still rate-limits it); `NEUTRAL_CONFIRM`
  vs real slow releases (pedal up 0.5–1.2 s before GEAR_MT reported was seen — those go through coastTimeout briefly, no
  throttle either way).

**Task 16 — Stage 2 fixes + in-car UI** (owner brief 2026-10-20; data `docs/stage2-findings-2026-10-20.md`, route
`00000004--9b8c913b70`). Feature `d3451cf1b` (1), `515f83aad` (2), `f5edfca91` (mutations), `0c0dd5ade` (tests/tools follow
the target), `f5dc25587` (HUD contrast); dev-test merges `2c87fe532`, `215d315e3`, UI `d9f2e597a`.
- **1 gas-override surge (SAFETY, always on for manuals, no toggle — owner decision):** while the gas overrides openpilot
  the planner re-seeds from aEgo every frame (stock); after the lift it resumed from the driver's +1.8 m/s^2 and the PCM
  re-opened the throttle (~1.5 s re-surge to 5.5 mph over set). `LongitudinalPlannerSP.update_gas_override` flags a
  reset that is a driver gas override while engaged (kept through the lift frame); `reset_accel` seeds `min(aEgo, 0)`
  for it on manuals; automatics and every other reset are stock. Only ever lowers the seed (less throttle, never less
  braking). Tests (`test_gas_override_seed.py`, closed loop): no throttle requested or sent while above set after the
  lift, no re-surge; below set it coasts first then resumes; automatic unchanged. *The brief's "never positive" is
  checked as "never positive while above set": after a lift over set the car may dip below set and the plan then
  correctly asks for throttle again.*
- **2 `EngineBrakeCoastTarget` 0.0 → −0.2** (constant + `params_keys.h`, compiled): C2 showed 0 = zero torque (no engine
  braking), a small negative = closed throttle. **It is the engine-braking request, not a brake threshold** — the brakes
  join past the gear's engine braking (≈ −0.5 in 3rd), which is EBP's separate credited estimate. EBP stays default OFF.
  Tests that hardcoded 0.0 now key on `DEFAULT_COAST_TARGET`; the two backstop tests (capacity, lockout) pin 0.0, their
  worst case; `engine_brake_replay` runs EBP at the default (0 clamps on its segment) AND at 0.0 (must clamp: 53, as in
  Task 6). **The EBP test plant now models the measured car** (feature `b10830106`): 0 = zero torque (road load only), a
  negative request = closed throttle, the car slows at the stronger of true engine braking and the request, brakes only for
  the difference; the old force-additive model is kept as a labelled synthetic car for the two backstop-mechanism tests.
  **FINDING: on the measured car EngineBrakePreferred changes no outcome** — brake use, speed and gap identical ON vs OFF in
  every scenario, even with a 5x over-estimated model — because the car already engine-brakes first (the Task 6 caveat
  "if the car already uses fuel-cut for small negative requests, the feature is unnecessary", now confirmed). Pinned by
  `TestMeasuredCarFinding`. Recommendation: drop session 10 from Stage 4 (owner decision). **Implication for NSB (not
  changed):** downhill, any negative request the engine can't meet is braked to (the VSA tracks it), so floor −0.3 brakes
  gently on a steep grade, and floor 0.0 is a zero-torque roll (no engine braking) — Stage 4 NSB should be tested with
  that in mind. Found by the full CORE run (the Rule 5 matrix's EBP
  state and the B3 lockout test failed their preconditions); the 36 EBP / R5 / latch / Task 16 mutations re-run clean.
- **3 UI (dev-test):** the session view was custom-drawn (14–28 px text, 24–42 px targets). Rebuilt on the **stock mici
  settings components** (NavScroller + `Big*` cards at stock size 402×180, 44 px titles / 32 px values, nothing < 24 px,
  white on dark — `contrast()` pins full white because stock re-applies a 90 % title color every draw and greys values):
  home = status + one card per session + phase view; session page = name/toggle status, objective card, **one card per
  toggle** (stock switch; multi-value = a card that cycles), **one card per test** (detail page with ARM), start-button
  status in monospace (`UNIFONT`). Chip 28 px monospace, opaque; HUD shift text full white. Kept: single write path, no
  Param-bound widget, D2 refusals on the card, satisfied requirement never flipped, D1 guards. D-09 is now stock's
  scroller touch invalidation — checked on the real widgets with a real multi-frame swipe (a one-frame "teleport" swipe
  does fire a card; real fingers span frames). `test_ui_smoke` rebuilt (every card + detail page, readability, real-widget
  toggles); `dev_test_render` per-card PNGs + 9 interaction checks; mutations: `d2_tap_any_release` re-pointed at the
  stock scroller, 4 new UI ones.
- **Run (PC, 2026-10-20):** feature CORE **908 passed** (+ the pre-existing chestnut failure), WIDE 2209; selfref
  controlsd / plannerd / card **0 diffs** on all 4 CI segments vs the Task 15 baseline (toggles OFF; CI segments are
  automatics); `clutch_reengage_replay` PASS (gate OFF bit-identical on the re-fingerprinted manual); `engine_brake_replay`
  PASS (both targets); `mutate.py` 84/84 (EBP/R5 subset 36/36 re-run after the test fixes). Dev-test: CORE **1035 passed** + the chestnut failure + **`test_locationd::test_params_gps`, a
  pre-existing flake** (a "written within 5 s" timing test: fail/pass/fail here, fail/pass/pass at the Task 15 tip —
  untouched code); WIDE 2209; `test_console_replay` 0 diffs on all 4 segments (console armed, 0 errors), isolation OK,
  offline OK, equiv every armed test captured; `dev_test_render` 9/9; **`mutate_devtest.py` 53/53**.
- **On-car next (Stage 3, test card):** 3a surge fix (lifts over and under set), 3b gate feel at 0.15 s (ClutchShiftGate ON),
  3c low-speed handback in an empty lot. Update via `git pull` on the device (dev branch, not prebuilt); the first start
  rebuilds (params_keys.h changed). Prebuilt not rebuilt (owner: test on-device first).

**Task 17 — EngineBrakePreferred = per-gear engine-brake clamp + learning scaffold** (owner brief 2026-10-20, with
three owner confirmations: reuse the toggle, the gear-trust rule, learn only from brake-free data). opendbc `c54d823b9`;
feature `7fb66ecc4` (clamp), `2a1772a73` (learner), `689b63f73` (mutations), `da041c61e` (plant + hand-off
finding); dev-test merges `59a3941b3` / `238f67f5a`, console `7e5fff9df`. Data: Stage 1 coasts (`analysis/stage1/eb_table.py`), Stage 2 torque fit, S8.
- **Premise (Stage 2):** one `ACCEL_COMMAND`; any negative request closes the throttle and the VSA brakes only what the
  car doesn't already achieve (on the flat AND downhill). So a request no stronger than the closed-throttle decel in the
  current gear never uses the brakes. Task 16's EBP (coast target only while the plan was within the credited engine
  braking) was a no-op for exactly that reason; the clamp is different: when the plan wants MORE than engine braking (a
  lowered set speed, a descent, a lead in adaptive mode) it does not hand off to the brakes.
- **Static table** `engine_brake_table.py`: flat-road closed-throttle decel D(gear, v), 8 speed bins, from the Stage 1
  disengaged coasts — min(median measured, 0x130 axle-torque engine part + fitted road load 0.225 + 1.25e-4·v²) where
  ≥ 1 s of data, that model elsewhere (1st: 23 N·m crank drag), 0 outside 900–6500 rpm. ~0.4–0.5 in 3rd–6th, 0.7 in 2nd.
- **Clamp** (`engine_brake_preferred.py`, controlsd, LongControl's lower limit): `floor = −0.8·D − g·sin(pitch)`.
  *Implemented differently from the brief:* (1) **the grade term** — the brief's flat table would brake on every descent
  (the VSA tracks the request downhill); (2) **descent beyond engine braking** (floor weaker than `FLOOR_MAX` −0.15) →
  steps aside to stock: holding speed there needs the brakes whatever the request; capping instead used as much brake in
  the sim with a higher peak (it alternated between a zero-torque roll and braking). −0.15 because −0.01 holds zero
  torque, −0.17 closes the throttle (Stage 2); (3) **runaway backstop** = > set + 3.6 m/s for 2 s while NOT slowing — a
  plain overspeed backstop tripped on every set-speed drop > 8 mph (found by a test) and handed it back to the brakes;
  (4) the clamp lives in controlsd (Rule 5, PID anti-windup), not the Honda carcontroller.
  Gear (owner rule): `GEAR_MT`, trusted only with `clutchPedal` UP, not decoupled (incl. the carstate phantom guard),
  rpm/speed within 5 % of that gear, not < 1100 rpm below 13 m/s; else floor **0 (coast only)**. Never clamped: any
  Rule-5 reason (same frame), EBP's own lead thresholds, a plan < hard floor within 1 s (these hold it off 1.5 s),
  descent, runaway, no pitch, no clutch signal, stopping, < 5 m/s, long inactive. Logs `engine_brake_preferred` (edges +
  reason), `engine_brake_clamp` (binding edges: gear, v, floor, target, aEgo, pitch), `…_overspeed`. Published:
  `manualLongitudinal.ebpSuppressing` (= clamp applies), new `ebpFloor @17` / `ebpGear @18` / `ebpBinding @19`;
  `ebpLockedOut` always false (capacity lockout removed: it encoded "never under-deliver the plan"). `EngineBrakeCoastTarget`
  **removed** (params_keys.h). GearAwareAccel keeps the old 0.7 credit as its own `CREDIT_MARGIN` (unchanged behavior).
- **Learning** (`engine_brake_learning.py`, plannerd, 10 Hz, manuals with op long; telemetry only): learns ONLY from
  brake-free data (owner): disengaged closed-throttle coasts (no gas, brake switch off, `USER_BRAKE` < 0.2, trusted gear,
  axle torque < 0, settled 0.5 s after the lift / 1 s after a gear change) → per cell Welford stats of D and the torque
  engine part E. Engaged frames (toggle latched on, long active, negative request) are logged for review with the torque
  brake check `extra = D − (E + road)` > 0.3 = brakes suspected (Stage 2: 0.00 median on pure coasts, 0.6–1.1 at −1.2),
  never learned from. *Why not slowdown-vs-request (the brief's signal):* Stage 2 frames that slowed MORE than asked were
  still braked 80 % of the time (slow brake release after firm requests). Store = Params `EngineBrakeLearnedTable`
  (JSON, PERSISTENT), saved ≤ every 60 s; samples `/data/engine_brake_learn/samples.jsonl`. `propose_table` (pure, tested):
  ≥ 50 samples, ≤ 0.02 m/s² per drive cycle (plannerd start), toward the mean − 2 SE, never above it, never below static;
  a measurement below static is noted. **Proposals are stored and logged, NOT applied** (`APPLY_LEARNED = False`).
- **Torque plumbing:** opendbc `CarStateSP.engineTorqueAxle` (`GAS_PEDAL_2.ENGINE_TORQUE_ESTIMATE`, NaN while 0x130
  isn't alive), `custom.capnp` @4.
- **Tests:** `test_engine_brake_preferred.py` rewritten (28: table / floor math, gear trust, measured-car closed loop —
  set-speed drop and mild descent brake-free where stock brakes, pitch error ±1°, table over-reading, untrusted gear =
  coast only, steep descent = stock, runaway, fast-closing / braking lead = stock, Rule 5 same frame + hold, latch,
  composition with NSB, logging), `test_engine_brake_learning.py` (11), Rule-5 EBP matrix state now a binding clamp;
  `test_ebp_lockout_survives…` removed with the lockout. Mutations: 19 new `t17_*`, `b3_ebp_latch` re-pointed,
  `b3_ebp_lockout_cleared` / `t16_coast_target_zero` removed.
- **Descent hand-off (owner, 2026-10-20): "engine braking giving up shouldn't default to service brake — that's when
  the sawtooth should take over."** Structurally it does: on a descent the clamp steps aside and the floors compose by
  max, so `NoServiceBrakeHold`'s coast floor / overspeed backstop / pulse sawtooth is what applies (with NSB off: stock).
  **FINDING (Stage 2 descents, re-analysed):** the VSA also holds requests of ~0 and small positive ones downhill (closed
  throttle, ~0.2–0.3 m/s² of braking), not only negative ones — so NSB's coast floor (−0.3, and 0.0) is braked to as well:
  on a descent steeper than the gear's engine braking the car holds its speed on the brakes and **never drifts up to the
  sawtooth's trigger**. Pinned by `TestDescentHandOff` (EBP test plant now models it; a synthetic "drift" car keeps the
  runaway-backstop test). A brake-free coast phase needs the request at the car's own closed-throttle accel (positive on
  such a descent) with the 0x130 torque as the guard against throttle = **the NSB coast-follow item, now the
  prerequisite for the sawtooth on this car** (owner decides when).
- **Run (PC, 2026-10-20):** feature: CORE 926 passed + the pre-existing `test_default_chestnut` failure (controls lib
  342 after the plant fix), WIDE 2210; selfref controlsd / plannerd / card **0 diffs** on all 4 CI segments (vs the Task
  14 captures; toggles OFF); `clutch_reengage_replay` PASS; `engine_brake_replay` PASS (the clamp runs on 1352 frames in
  6th, floor −0.73…−0.15, never binds — openpilot never asks below −0.11 on that segment — so ON = OFF); **`mutate.py`
  101/101** (t15 pedal-staleness entry re-pointed after the GAS_PEDAL_2 freshness refactor). Dev-test: CORE **1055 passed** (+ the same chestnut failure), WIDE 2210; `test_console_replay` **0 diffs** controlsd + plannerd on all 4 segments (console armed, 0 errors), isolation OK, offline OK, equiv every armed test captured, `dev_test_render` 9/9; **`mutate_devtest.py` 55/55** (the first sweep was 54/55: nothing tested that E1 reads the PUBLISHED floor — `TestPublishedClampFloor` added). Pushed 2026-10-20 (see wrap-up).
- **Wrap-up:** pushed opendbc `c54d823b9`, feature `fe4ebb0b1`, dev-test `0d02346cf` (the fork copy of CLAUDE.md lags this
  line only); bundles regenerated (11th regen); docs re-mirrored (fork copies + Dropbox). Nothing flashed. The device
  update rebuilds on first start (`params_keys.h` and `custom.capnp` changed). `EngineBrakePreferred` stays default OFF;
  the learner collects on every drive (telemetry). Coast-follow: after Stage 3 (owner), gated on the Stage 3 card 3d check.
- **On-car (session 10, after Stage 3):** set-speed drops on the flat with the cluster's mini-car brake lights off; a
  mild descent; does a −0.15 request close the throttle (FLOOR_MAX); does a small positive request below the car's own
  accel keep the throttle closed on a descent (would make it brake-free on any grade); pitch-estimate error on the
  car's usual descents; review the learner's samples + proposals before `APPLY_LEARNED`.

**Stage 3 — on-car (2026-10-21): coast-down table drive + first mixed op-long drive** (brief: Dropbox
`CommaAI/prompts/stage3-analysis-prompt.md`; full write-up `docs/stage3-findings-2026-10-21.md`; scripts
`analysis/stage3/`). Routes `00000006--65015c3735` (Drive 1, 30 min, never engaged) and `00000007--001552d195` (Drive 2,
434 s engaged), pulled over the phone hotspot (`comma@10.69.162.42`), rlog + qcamera only → Dropbox `CommaAI/routes/`.
**Every custom toggle was OFF in Drive 2** (published latched state): it ran stock SP longitudinal + the Task 16 surge
seed + the handback. Feature `sp-honda-dev-202608` + dev-test: table + tool commits (see the doc §4).
- **Engine-brake table rebuilt** (`engine_brake_table.STATIC_DECEL`) from ~850 s / 276 brake-free coasts (Stage 1 +
  Drive 1 + Drive 2's disengaged coasts), settled frames only (≥ 1 s into a coast — the first second reads ~0.05 light).
  **Stage 1 was not conservative:** 3rd–6th ~0.03 high; 1st 0.95 → 0.62 (Stage 1 had no 1st-gear coasts — crank-drag
  model); 2nd at 5 m/s 0.70 → 0.33. Learner `ROAD_LOAD` → 0.188 + 1.52e-4·v². Gaps: 6th 60–70 mph, 3rd > 50 mph, 5th/6th
  < 35 mph, 1st/2nd above their measured speeds. The learner had already collected both drives on-device (nothing to
  feed); found: its mean runs above the settled median (tail + no settle), and **proposals never accumulate**
  (`_new_cycle` doesn't pass `learned_prev` → never more than 0.02 from static) — fix at the APPLY_LEARNED review.
- **Surge fix confirmed:** 17 gas-override releases, 0 s of positive command while over set (Stage 2, same analysis: 7
  of 10 over-set releases, up to 1.47 s, +2.6 mph). All 21 `longActive` restarts followed the gas (17) or the engagement.
- **Shifts:** "no flare" with the gate OFF = the PCM ignores ACC throttle with the clutch in (up to +0.32 requested). 0
  faults. Gate what-if (`carstate_route_replay.py --gate`, fixed to engage after 5 s of a live clutch signal — 0x1DD
  blips at start-up had blocked it) on all three on-car routes: 298 episodes, 0 false starts, resume median 0.15 s.
- **Handback:** both at 5.0 mph, smooth, no stall — but **5 mph in 2nd = 575 rpm, below idle**: the ECU's idle control
  pushed +150 Nm (the "idle creep", ~4.7 mph) until the clutch went in 1 s later. Proposal (owner): gear-aware handback
  speed (≥ ~1000 rpm in the current gear: 2nd 8.7 mph, 3rd 13 mph).
- **Follow distance / sluggish re-accel = stock.** `long_mpc.py` byte-identical to MVL base / sunnypilot / comma;
  aggressive = 1.25 s·v **+ 6 m** → 1.48 s desired, 1.65 s actual at 62 mph. Plan trails the lead's accel by 1.15 s in
  stop-and-go (lead MPC). The surge seed can only act after a gas override (code + logs).
- **ACCEL_COMMAND at highway (new):** the PCM delivers the requested *acceleration* incl. road load — a steady 0 holds
  speed at 60–80 mph (+160 Nm), throttle closes only near −0.3. Stage 2's "0 = zero torque" was at ~30 mph.
- **Card 3d not exercised** (8.5 s engaged steeper than −1.7°, no small positive requests) — coast-follow stays gated.
- **Cold idle beats the phantom guard:** 1,240–1,700 rpm for the first 30 s, GEAR_MT reported 1 at a standstill in
  neutral → read coupled for 32 s (harmless at 0 mph). Proposal: GEAR_MT 1–6 below 0.5 m/s = neutral; idle-relative rpm.
- **Flare cool-down counts shifts:** clutch-in/slipping rpm rises (blips, re-coupling) can't start a cap but count toward
  the 2-in-10 s cool-down (3×/drive). Minor; proposal: count only flares that could start a cap.
- **Run (PC, 2026-10-21):** controls lib 342 passed; CORE 927 + the pre-existing chestnut failure; Task 17 mutations
  20/20; `engine_brake_replay` PASS; dev-test CORE 1056 + chestnut, console + controls 471 passed. Pushed feature
  `6db16d0e0` (`e5cb0b2c9` table, `c4eafd826` replay tool), dev-test `d733926dd` (opendbc unchanged, `c54d823b9`); the fork
  copy of CLAUDE.md lags this line only. Nothing flashed; Python-only (dev-test `git pull`; prebuilt needs the script, no
  recompile).

**Session 10 prep (2026-10-21)** (brief: Dropbox `CommaAI/session10-prep-prompt.md`; feature `sp-honda-dev-202608` →
dev-test). Three changes, all from the Stage 3 findings:
- **Gear-aware handback speed** (`manual_low_speed_handback.py` `GEAR_HANDBACK_SPEED`, always on like the handback):
  threshold = max(`ManualLowSpeedHandbackSpeed`, the GEAR_MT gear's value): **2nd 8.7 mph, 3rd 13.0 mph** (~1000 rpm);
  1st and **4th–6th use the setting — owner decision** (the brief's "1000 rpm in 4th ≈ 3.5 mph" was inverted: it is
  17.6 mph in 4th / 26.4 in 6th; asked, and the owner kept 5 mph there since a driver clutches/downshifts first and an
  18–26 mph handback would take Honda lateral with it). Gear = GEAR_MT while `clutchSignalValid`, else the setting
  (clutch in / neutral too). Checked every frame until it latches; never undone by a gear change. The published
  `handbackSpeed` is now the gear-aware value (capnp comment updated), so selfdrived's engage refusal and its "Drive
  Above N mph" text follow the gear. A phantom 2nd in neutral at idle only makes a neutral coast hand back sooner.
- **Learner v2** (`engine_brake_learning.py`, still telemetry only, `APPLY_LEARNED` off): per-cell D **histogram →
  median** (bound = the quantile 2 binomial SEs below 0.5, n_eff = coast seconds); **1 s in-coast settle**
  (`SETTLE_IN_COAST`); **proposals build up** (`_new_cycle` passes `learned_prev`). Store VERSION 2: a v1 store is dropped
  at start (`engine_brake_learn_store_reset`); `tools/civic6mt/learner_backfill.py` rebuilds v2 from `samples.jsonl`
  (the device's Stage 3 samples: 6,933 rows → 5,126 learnable, 24 cells; first proposals within +0.02 of static).
- **`EngineBrakePreferred` default ON** (`params_keys.h` "1", owner). Both readers now use `get(…, return_default=True)`
  (`get_bool` ignores registered defaults — an unset key would have stayed OFF). A stored 0 still wins: **the car's
  param file held 0**, so the default alone wouldn't turn it on there. Closed-loop sims keep their EBP-OFF baseline
  (`manual_sim.set_params`); `test_defaults_on` covers the default (unset → ON, stored 0 → OFF, automatics never).
  **Always-on exception list (E2) gains: EBP on manuals with op long (default), and the gear-aware handback speed.**
- **Lateral stays independent of the handback (owner, 2026-10-21, after a side note flagged that 2nd/3rd now hand back at
  8.7/13 mph):** the Stage 3 logs show the handback never ended lateral with MADS on (`latActive` True through both
  handbacks; MADS runs after selfdrived and MVL's all-speed-MADS branch strips `speedTooLow`/`belowEngageSpeed` once
  longitudinal is off). The one coupling — a SET press (Unified Engagement Mode) below the handback speed with lateral
  off was refused for lateral too — is fixed: `mads.py` strips both on manual-handback cars (`handback_available`)
  whenever longitudinal is disengaged; stock-ACC Hondas / automatics unchanged. `test_mads_manual_handback.py`, mutation
  `s3_mads_refusal_blocks_lateral`. MADS OFF = stock openpilot (lateral tied to engagement).
- `clutch_reengage_replay.py` now pins EBP OFF (its "gate OFF = capture" check differed only because EBP turned on by
  default on the re-fingerprinted manual; pinned → bit-identical).
- **Run (PC, 2026-10-21):** feature: controls lib 352 passed; CORE 937 + the pre-existing chestnut failure; WIDE 2210;
  **`mutate.py` 110/110** (9 new `s3_*`; + `s3_mads_refusal_blocks_lateral` caught → 111; CORE 940 after the MADS fix); self-referenced plannerd + controlsd vs captures at `6db16d0e0`: **0 diffs on all
  4 CI segments** (automatics: none of the three changes reaches them); `clutch_reengage_replay` PASS;
  `engine_brake_replay` PASS. Dev-test: CORE 1066 + chestnut (after rebuilding `libparams_c.so` on that branch — the
  feature build lacks the `TestConsole*` keys); **`mutate_devtest.py` 55/55**; `dev_test_render` 9/9. `test_console_replay`
  not rerun (no console code changed).
- **Pushed:** feature `862bd713a`, dev-test `cafc0dab6` (incl. the MADS fix; opendbc unchanged, `c54d823b9`); the fork copy of
  CLAUDE.md lags this line only. Dev-test CORE 1069 + chestnut after the MADS fix.
- **Device updated 2026-10-21 (offroad, hotspot `10.69.162.42`):** `git pull` → `cafc0dab6`; `EngineBrakePreferred` set to 1
  (the stored 0 overrode the new default); learner store rebuilt as v2 on the device from its own `samples.jsonl` (5,126
  samples, 24 cells, cycles 2; the v1 store kept at `/data/engine_brake_learn/EngineBrakeLearnedTable.v1.json`); rebooted →
  `build.py` rebuilt `libparams_c.so` (registered EBP default now True), manager up. **Device Python = `/usr/local/venv/bin/python`
  with `PYTHONPATH=/data/openpilot`** (system `python3` lacks zmq/capnp). Ready for session 10.

**Session 10 — on-car (2026-10-21): first EBP-ON drive + lateral investigation** (brief: Dropbox
`CommaAI/session10-analysis-prompt.md`; full write-up `docs/session10-findings-2026-10-21.md`; scripts `analysis/session10/`).
Route `00000008--044cd426e8` (48 min, 537 s engaged, mostly 6th at 60–76 mph behind traffic), pulled over home WiFi
(`comma@192.168.0.150`), rlog + qcamera → Dropbox `CommaAI/routes/`. Device on dev-test `0b69935b7`. No code changed.
- **EBP was ON** (stored 1, `ebpEnabled` latched every engaged frame) and works: binding 8.3 s (plan to −1.20 held at the
  engine-braking floor, torque check 7 % braked). But it ran only 15 % of the negative-plan time: **72 % was lead following**,
  where Rule 5 / its lead thresholds (closing ≥ 1 m/s at any distance + 1.5 s hold) hand braking to stock by design.
  Suspected service brake over the whole engaged drive: ~15 s of 508. What is felt in a slow-down is mostly closed-throttle
  engine braking (≈ 0.4–0.5 m/s²), which EBP never removes. The calibrated pitch matched GPS grade (median 0.04°).
- **Throttle boundary at highway speed (new):** 6th, 56–78 mph: a −0.05…−0.15 request gets −0.14 on part throttle
  (+114 Nm); −0.15…−0.25 closes it. Below ~55 mph in 4th, any request < −0.05 closes it. So `FLOOR_MAX` −0.15 sits on the
  boundary at highway speed, and a gentle "coast to set speed" cap is feasible at highway speed only (proposal, owner).
- **Set-speed drops:** 4 of 16 had no lead and overspeed; all slowed mostly on engine braking (≤ 0.9 s braked each; the
  braked one was a descent step-aside). NonAdaptive does not change set-speed reductions (it keeps exactly `a_cruise`).
  `ACCEL_COMMAND` 0 is not a coast on this car (zero-torque roll at 30 mph, holds speed at highway).
- **Re-accel:** stock. Both highway 15 mph drops = following a slower lead (aggressive); Brian took over both times. When
  openpilot re-accelerates it rides `get_max_accel` (0.78 at 60 mph → 60→75 ≥ 9.2 s) and, behind a close lead, holds ≤ 0
  ~3 s to re-open the gap. openpilot kept braking through a lane change (model kept the old lead 2.5 s). Proposal: opt-in
  higher `A_CRUISE_MAX` above 25 m/s (~1.1/0.9); 6th delivered 0.9–1.2 m/s² for the driver.
- **Long-press = Experimental mode** (stock SP `cruise_helpers.py`, ≥ 0.5 s while main cruise is on), not NonAdaptive
  (no button handler exists; only the dev-test console writes it). Brian's holds toggled Experimental 4× (net off) and a
  hold with main cruise off + a tap stepped **personality aggressive → relaxed → standard** (still standard on the device).
  Wiring proposal: CruiseHelper handler, hold → NonAdaptive while disengaged (option b), new EventNameSP + alert.
- **Lateral: our fork changed nothing lateral.** Differences vs SP/comma come from the MVL base: **STEER_MAX 5120**
  (`2c62d127b`; SP/comma 4096), rate-limiter reset on `STEER_CONTROL_ACTIVE` drop (0.1 s of drops on this drive — not a
  factor), widened calibration limits (calibration here is fine: 100 %, pitch +1.15°, yaw −2.14°), torque override
  3.1/1.5 (torque/NNLC only). Model = SP default **CD210** (`ModelManager_ActiveBundle` unset); NNLC off and the mici UI has
  no steering panel. Measured: the car tracks openpilot's desired curvature (gyro/desired 0.94–1.04, 0.1 s lag); the
  failures are tight 35–40 mph bends where the model ramps late to ~1.7 m/s², openpilot sends 5120 for ~2.5 s and gets
  ~half (2 "turn exceeds limit"); 62 s at the 5120 cap over the drive. Whether the EPS honors > 4096 is unknown
  (`steeringTorqueEps` reads 0). **Laneless:** the plan weaves 2–2.5× more with a weak lane line; the car follows it →
  model issue; no laneless tuning in this base. Proposals: read the 3X's model/NNLC settings, A/B NNLC (param over SSH),
  A/B a release model, then (only if needed) a Param-gated STEER_MAX 4096/5120 A/B.
- **Owner decisions (same day) — done:** (1) `SetSpeedCoast` (default OFF): set speed ≥ 55 mph → the set-speed candidate
  decelerates ≤ 0.12 m/s² (stock −1.2); Rule 5 = stock the same planner frame + the stock shadow (`features.aTargetStock`,
  the GearAware mechanism) for controlsd. (2) `CruiseAccelBoost` (default OFF): `get_max_accel` [1.6, 1.2, 1.1, 0.9]
  (60→75 mph 6.5 s vs 9.2). Both in `sunnypilot/.../cruise_profile.py`, any transmission with op long, latched while
  disengaged. (3) `GapHoldNonAdaptive` (**default ON**, manual + op long): the ≥ 0.5 s distance hold toggles
  `NonAdaptiveLongitudinal` while disengaged (engaged: alert only), never steps the personality (also with main off);
  automatics stock (`cruise_helpers.py`). Fork-only ordinals `LongitudinalFeatures` @6–@8, `OnroadEventSP` @29–@31.
  (4) **NNLC enabled on the device** (param, offroad) instead of a STEER_MAX A/B; open-loop pre-flight
  `analysis/session10/nnlc_offline.py` (bounded ±1, sign agreement 88–99 %, less time at the cap). Caveat: the NN expects
  the stock 4096 scale, this base sends 5120 — if NNLC overshoots, A/B STEER_MAX 4096 next. Feature `de178ceb5`; 8/8 new
  mutations caught. **Always-on exception list (E2) gains `GapHoldNonAdaptive`** (button behavior only).

## Test console architecture (core + vehicle profile)
The on-car test console is a **vehicle-agnostic core** — capture state machine, detectors/analyzers, the registry/session
engine, the UI framework, file IPC, the start button's gestures, and the non-actuating guarantee — **plus one vehicle
profile per car** (signal map → named roles, powertrain params, button map, the test plan: tests/phases/sessions/required
toggles and their pass rules, the UI's toggle allowlist, applicability), **selected by fingerprint**; no profile → the
console is inactive. It lives only on the dev-test overlay branch that tracks production (feature → dev-test, never
back); production never carries it. **Adding a car = writing a profile** (`profiles/<car>/` + one line in
`profiles.PROFILES`); the core is reused unchanged. Console work should keep platform assumptions out of the core — a
test greps the core for platform tokens. Details + "adding a car": `docs/TEST_CONSOLE.md`. (Recorded as the target in
Task 10 Step 1 and implemented by the rest of Task 10.)

## Scope / safety guardrails
- Build within openpilot's safety envelope. Do NOT modify the panda safety firmware / AEB guard.
- Keep openpilot's own longitudinal limits and FCW.
- Prioritize the clutch gate before any real road use.
- This is experimental manual longitudinal on an under-tested platform: validate carefully —
  low-speed, empty road first, ready to take over.

## On-car verification checklist (before the first road test)
All features (Phases 1–3, Tasks 2–6, 8, 11–14) are SIM/TEST-ONLY. Nothing below has been observed on the real car.
**Run this list through the Task 7/9/10 test console** (flash `civic-6mt-dev-test`; Settings → dev test opens the session
view — work through sessions 1→11 + the Cal card): every item below is a registry test
with live conditions, capture and a pass rule — S1–S7, D1, C1–C2, G1–G4, N1–N2, NSB1–NSB2, H1–H4, E0a–E3, GA1, A7
(`docs/TEST_CONSOLE.md`).
Prerequisite: the toggles only matter with openpilot longitudinal (alpha) enabled, which **disables stock
AEB/CMBS** on this car. Keep `DisengageOnAccelerator` OFF if using the clutch gate.

Signals — cabana, engine running (parked, then rolling slowly):
- [ ] `GEARBOX_ALT_2.GEAR_MT`: 0 with clutch down in gear, 1–6 per gear, 14 in reverse; what it reads in
      true neutral at a stop with the clutch out.
- [x] `GEAR_MT` lag: how long after the clutch pedal moves (in and out) it changes — sets whether
      `REENGAGE_DELAY` (0.5 s) and the gas-release cue are right-sized. *(Stage 1: reports the gear at full engagement,
      ratio already locked → Task 15 cut the delay to 0.15 s.)*
- [ ] `ENGINE_DATA.ENGINE_RPM` matches the tach with no extra scaling; its update rate/noise is clean
      enough for the 1500 rpm/s flare threshold.
- [ ] `PEDAL_GAS` → clutch timing in real shifts (how long between lift and clutch) — only matters if
      `ClutchShiftGateGasReleaseHint` is turned on (`GAS_RELEASE_WINDOW` 0.6 s).
- [ ] Rev-match / heel-toe downshift trace: `GEAR_MT` stays 0 through the blip; RPM flare during the blip
      doesn't leak into the gate; how fast `GEAR_MT` reports the new gear as the clutch bites.
- [ ] Optional: hunt for a raw clutch-switch bit in `POWERTRAIN_DATA` `BOH*_17C` (S6 signature, N=3), and the
      volume buttons as a start button (S7, N=4/5 — likely B-CAN, not visible).

Behavior — empty road, low speed, ready to take over, one toggle at a time:
- [ ] **Clutch gate frame acceptance:** VSA/PCM accept `ACC_CONTROL` with `CONTROL_ON` and a coast/brake command
      (since Task 12 A2 longitudinal stays active through the shift: throttle capped, braking passes) with the clutch in
      for the length of a shift — no `CRUISE_FAULT`, no cruise cancel. Test this first.
- [ ] **(Task 12) Clutch signal liveness:** `CarStateSP.clutchSignalValid` true on the car (0x1DD alive); the gate
      shows "Shift gate off" if it isn't. `GEAR_MT` values other than 0–6/14 ever seen? (they count as decoupled).
      **(Task 13 A3)** it now also drops on a bad 0x1DD counter (for as long as the counter score is non-zero) or checksum
      (100 ms): how often does that happen on the real bus? Occasional single-frame drops are expected; frequent ones
      would make the gate / EBP / GearAware flicker inert. **(Task 14, W1-N6)** the check also needs the 2-bit counter to
      step by exactly 1 every frame at > 4 Hz; any other pattern on the real car (e.g. a counter that steps by 2, or a
      slow message) leaves those features permanently inert (fail-safe) — confirm in cabana.
- [ ] **(Task 12) `ClutchShiftGateCoastAccel` vs C2:** 0.0 is right only if `ACCEL_COMMAND` 0 coasts; if 0 holds
      speed (PCM adds throttle), a small negative cap may be needed — and then whether that uses the service brakes.
- [ ] Clutch gate on real up- and downshifts: coasts through the shift, no RPM flare, smooth resume, no
      false flares in normal driving (hills, downshifts, engine braking). Check the
      `clutch_shift_gate` cloudlog events line up with the actual shifts.
- [ ] Non-adaptive: holds set speed; does not brake for a slower lead far ahead or for steady following; FCW still
      alerts on a closing stopped/slow car. **(Task 12 A1)** it DOES brake for a collision threat — validate with wide
      margin (a cooperating car, empty road): when the lead MPC is re-admitted (`non_adaptive_collision_braking`
      cloudlog), how early vs. FCW, and that ordinary traffic (merges at a normal gap, a slower car far ahead) does not
      trigger it. The 2 s dwell and the thresholds (TTC 6 s, closing 0.3 m/s when close, MPC need −1.0 m/s²) are
      provisional.
- [ ] **Radarless brake strength:** with openpilot long, does a negative `ACCEL_COMMAND` actually brake,
      and how hard vs. the request? Without PR #2050, radarless openpilot long reportedly "won't brake,
      only accel/coast" (or brakes weakly) — check before relying on any openpilot braking, and decide
      deliberately vs. the no-downhill-brake goal. *(Corrected 2026-10-16: this item used to ask whether
      the fingerprint includes `0x1BE`. `0x1BE` (`BRAKE_MODULE`) is the alternate **user brake-pedal**
      message — fingerprinted on ACCORD / HRV_3G / RDX_3G / MDX_4G / ADX / Bosch CANFD, not CIVIC_2022 —
      and is not what lets openpilot brake; braking authority is the camera's negative `ACCEL_COMMAND`.
      Side note: MVL's `hondacan.py` uses `BOSCH_ALT_BRAKE` (set from `0x1BE`) as the condition for
      sending `COMPUTER_BRAKE_ASSIST` while braking, so on our Civic that bit stays in its
      idle-stop-allow role; whether that matters for brake strength is part of the PR #2050 question.)*
- [ ] Non-adaptive above set speed downhill: what the radarless brake path actually does (PR #2050).
- [ ] **Task 1 stock limits:** a normal drive with openpilot long shows no new faults/blocked frames (our
      radarless car has no panda gas check; this is a regression check for the Nidec/Bosch revert).
- [ ] **Clutch gate + rev-match:** real heel-toe/rev-matched downshifts with the gate on: stays coasting,
      no override/disengage from the blip (DisengageOnAccelerator OFF), brake still disengages as stock.
- [ ] Engine-brake downshift with long active and no brake: does openpilot add throttle after the (0.15 s since Task 15)
      re-engage delay and fight the engine braking? (tuning item) Then A/B with `ClutchReengageSmart` ON: takeover
      at the bite, seeded at the engine-braking decel — no lurch/hole, no brake tap (C2 decides the seed).
- [ ] **No-service-brake:** with non-adaptive ON and floor 0.3 — does `ACCEL_COMMAND` ≥ −0.3 actually stay
      off the brakes (VSA decides)? Try floor 0.0 too. Coasts over set speed downhill as intended.
- [ ] **Overspeed backstop:** on a long descent, braking resumes at set + 8 mph — only as effective as the
      radarless brake-strength item above; if the negative `ACCEL_COMMAND` barely brakes, the backstop is too.
- [ ] **Overspeed brake pulse (Task 11, `NoServiceBrakeOverspeedMode` pulse — after the `maintain` run above):** distinct
      pulses that start at set + 8 mph and release at set + 2 mph, rolled on/off (no step), coasting in between; does a
      −1.0 command decelerate at −1.0 downhill (short pulses) or ride on (long ones)? Past set + 15 mph: one "Overspeed:
      Take Over" alert (repeats every 4 s while still over and rising) and set-speed braking (the −1.2 m/s² cruise-plan
      cap in non-adaptive, not "full" braking). NSB2 scores it.
- [ ] **Low-speed handback (Task 5):** tune `ManualLowSpeedHandbackSpeed` + `ManualLowSpeedHandbackRamp` for a
      smooth, non-jerky handoff in 1st gear (5 mph in 1st is near idle — the point may need to be higher).
      One prompt from the ramp start ("openpilot is handing longitudinal back to you"), no lurch; check the
      `manual_low_speed_handback` cloudlog events.
- [ ] **(Task 12 A3) Handback under urgency:** behind a car stopping close ahead, openpilot keeps braking through the
      threshold and may brake to a stop — the driver must clutch in (a stall is the alternative). Check how that feels,
      and that the prompt never masks FCW.
- [ ] **(Task 13 A1) Handback in ordinary traffic:** behind slow or stopping traffic tens of metres ahead the handback
      runs at the handback speed exactly as with no lead (prompt, ramp, disengage) — openpilot no longer brakes the manual
      to a stop and holds it there. With a genuinely close car (`lead_imminent`: a driver braking at 2 m/s² after the ramp +
      1.5 s couldn't stop 2 m short) it brakes to the stop, then hands back 0.5 s after standstill
      (`manual_low_speed_handback_standstill` cloudlog). Do the thresholds match how this driver actually stops?
- [ ] **(Task 14) Handback latch in stop-and-go:** a slow car close ahead that then drives off → openpilot hands back
      within ~1 s of it pulling away (`manual_low_speed_handback_latch_release`), not engaged-and-rolling; a hard stop
      behind a car (FCW or hard braking) → it brakes to the stop and only then hands back.
- [ ] **(Task 14, W1-N5) Standstill resolution on a slope:** after openpilot brakes a manual to a stop for a close car,
      the resolution releases the brakes 0.5 s later. On a −6 % grade in the sim the car (clutch in) rolled into the lead
      during the modelled 1.5 s driver reaction. Does Honda Auto Brake Hold engage after an *openpilot* stop without a
      driver brake press? Test it on a gentle slope first, foot over the brake; it decides whether this needs code.
- [ ] **(Task 14, open finding) Downhill braking authority — part of C1:** command a steady −1.0 m/s² on a ~6 % descent:
      is −1.0 actually delivered (the VSA tracks the request), or does gravity eat it? Above 3 m/s openpilot sends the
      plan's accel uncompensated (see Open questions). Wide margin, empty road.
- [ ] **Auto Brake Hold / hill-hold assumption:** confirm the car's Auto Brake Hold and automatic hill-hold
      hold the car at a stop and prevent rollback after openpilot hands back — **especially on a hill**.
- [ ] **GEAR_MT in neutral while rolling:** what it reads when coasting in neutral (clutch out) with openpilot
      engaged; the gate should reach `coastTimeout` after `ClutchShiftGateMaxCoast` (5 s since Task 12; braking was never
      paused).
- [ ] **Smart re-engagement (Task 8, `ClutchReengageSmart`):** ratio lock fires at the real bite (not mid-slip, not
      never — no lock within 1 s → fixed-delay fallback + `clutch_reengage_no_lock`: check ratios / the 2 % lock tolerance
      `ClutchReengageRatioTol` vs real rpm/speed noise); on-throttle upshift waits for the lift; feathering the gas
      doesn't grab/yield; measure the resume jerk (sim peak 1.3–3.4 m/s³ depending on plant + seed).
- [ ] **(Task 12 B4) Takeover seed `ClutchReengageSeed`:** `aego` vs `coast` after a downshift into engine braking — C2
      decides (if a negative command adds service brakes on top of engine braking, `aego` is a brake tap).
- [ ] **(Task 12 A2) RPM-flare thresholds** against real ENGINE_DATA: 3 new samples > 1500 rpm/s, no gas; cool-down
      after 2 flares in 10 s. Zero false flares over a normal drive (sim: 10 min of σ 30 rpm noise at 25 Hz).
- [ ] Uphill handback: crossing the threshold on the gas uphill — how fast the car slows once the ramp ends.
- [x] **(Task 16) Gas-override surge fix:** engaged over set, press the gas, lift: no re-acceleration (the command must
      not go positive while over set); lifting below set coasts briefly then resumes smoothly. *(Stage 3: 17 releases,
      0 s positive while over set.)*
- [ ] **(Task 16) Console UI on the device:** cards readable at arm's length in the mount, swipes never flip a switch,
      a refused write says why on the card.
- [ ] **(Task 17, before coast-follow — Stage 3 card 3d) Descent throttle boundary:** mild descent (~3–5 %), 6th, engaged
      ~2 mph under set, off the pedals: at a small positive `ACCEL_COMMAND` (+0.05…+0.1) does `ENGINE_TORQUE_ESTIMATE`
      (0x130) stay at closed-throttle drag (≈ −65 Nm in 6th → coast-follow works) or rise (the PCM opens the throttle →
      a different strategy)? One observation, not a sweep (owner, 2026-10-20). Coast-follow stays after Stage 3.
- [ ] **(Task 17) Engine-brake clamp (session 10):** `EngineBrakePreferred` ON, flat, 4th–6th: set-speed drops of 5 mph
      slow the car on engine braking with the cluster's mini-car brake lights OFF; `engine_brake_clamp` events line up;
      clutch in / neutral = no decel request; a lead closing ≥ 1 m/s or braking → normal braking at once. Then a mild
      descent (a little braking possible: pitch error). Check that −0.15 really closes the throttle (`FLOOR_MAX`).
- [ ] **(Task 15) Clutch pedal switch:** in cabana, move the clutch pedal slowly: `GAS_PEDAL_2.CLUTCH_PEDAL_UP` (bit 50)
      opens at the first movement, `CLUTCH_PEDAL_FLOOR` (bit 51) closes only at the floor; `CarStateSP.clutchPedal` reads
      1 / 2 / 3 accordingly and never 0 once running.
- [x] **(Task 15) Phantom guard at cold idle:** right after a cold start, rolling in neutral: is idle under 1100 rpm
      (`PHANTOM_RPM_MAX`)? If not, a phantom gear at cold idle would be trusted. *(Stage 3: **no** — 1,240–1,700 rpm for
      the first 30 s; GEAR_MT 1 at a standstill was trusted. Fix proposed, not done — see Stage 3.)*
- [ ] **(Task 15) 0.15 s resume:** gate ON, real up/downshifts: the resume 0.15 s after the clutch is fully out — no lurch
      (the takeover ramp still rate-limits it); `clutch_shift_gate_timeout reason=neutral` on a neutral coast with the
      clutch out, not on a slow clutch release.

Task 6 (engine-brake features) — only after the items above, toggles one at a time:
- [ ] **Coast logging first (no toggle needed):** drive normally with coasts in each gear; confirm
      `/data/engine_brake_coast/coast.jsonl` fills, the logged rpm/speed ratios cluster at the seed gear ratios
      (confirms final drive 4.10 vs 4.11 and tire radius — if they don't match, gear inference reads 0 and both features
      stay inert), and fit the per-gear/rpm decel table to replace the seed.
- [ ] **What a coast-target command does (before enabling `EngineBrakePreferred`):** with openpilot long engaged, in
      gear, on a flat empty road: does `ACCEL_COMMAND` 0 coast on engine braking, or hold speed? Does a small negative
      request (−0.2…−0.5) use fuel-cut only, or the service brakes? This decides whether the feature is useful at all
      and what `EngineBrakeCoastTarget` should be (see Task 6 caveat).
- [ ] `EngineBrakePreferred` gentle trims (set-speed −1/−2 mph, no lead): no brake application, car slows on engine
      braking; `engine_brake_preferred` cloudlog events; no `capacity` releases on a flat road.
- [ ] **Lead-approach hand-off, wide margin:** follow a cooperating car that eases off, then brakes; confirm the
      hand-off to braking happens early (closing-speed/TTC triggers) and the gap is never worse than with the toggle off.
- [ ] Downhill: suppression should mostly not apply (gravity cancels engine braking); no runaway.
- [ ] `GearAwareAccel`: throttle-on / lift-off feel in 1st–3rd; tune `JERK_SCALE_BY_GEAR`.

## Open questions
- ~~**FEATURE IMPROVEMENT — lateral at the low-speed handback**~~ — **resolved 2026-10-21** (owner: lateral must stay
  independent of the handback): with MADS on, the handback never ended lateral (see the Task 5 correction), and MADS
  now also ignores the handback's refusal when lateral is off (Session 10 prep). Lateral stays on its own MADS/stock
  logic at any speed. With MADS OFF, stock openpilot ties lateral to engagement — no decoupling there. Original text:
  today the
  Task 5 handback disengages via stock `speedTooLow`, and on Honda MADS honors it, so lateral drops with
  longitudinal below ~5 mph and can't be engaged below ~6 mph (`belowEngageSpeed`). Decided 2026-10-16: leave
  as-is, revisit. *(Task 12 A3.6 [B-L5] looked again: not clean to do without the car — sunnypilot's `mads.py` keeps
  `speedTooLow`/`belowEngageSpeed` for Honda while engaged on purpose, so either option changes that Honda path, and the
  EPS low-speed behavior must be checked on-car first. Still open.)* Options: (a) a toggle (e.g. `ManualLowSpeedHandbackKeepLateral`, default OFF) that has MADS
  strip `speedTooLow`/`belowEngageSpeed` for this case the way `mads.py` already does for non-Honda brands,
  so only longitudinal hands back; (b) make that MADS behavior unconditional for the manual handback when MADS
  is enabled. Either way: check Honda EPS low-speed/standstill steering cutoffs and `steerAtStandstill`, that
  the handback alert still fires once, that engage refusal still blocks *longitudinal* below the threshold,
  and on-car that lateral stays engaged through stops without nags or faults.
- ~~Stop-and-go on a manual with openpilot long~~ — **resolved by Task 5 (low-speed handback)**: openpilot
  hands back below ~5 mph and refuses to engage below it, so it never brakes a manual to a stop or launches it.
- Base branch: forks are on MVL's `sp-honda-dev-202608`, pinned at tag `mvl-base/sp-honda-dev-202608/2026-10-13`
  (openpilot `66598eda1`, opendbc `cf4611742`). *Correction 2026-10-16:* it was originally chosen for a
  "longitudinal-toggle VSA-fault fix" — that fix (opendbc PR #802) targets Bosch **radar** cars, is
  still unmerged, and is **not** in our base. The dev branch is still the right choice for a different
  reason: MVL's radarless LKAS-auto-disable / `SCM_BUTTONS` takeover (stops the touch-steering nag
  forcing an ACC disengage) is dev-only. Side effect of the half-landed #802: the alpha-long toggle
  flipped *while onroad* applies on the next drive. See `docs/FORK-RESILIENCE.md`.
  When the comma arrives, install the fork's **`civic-6mt-dev-test`** branch (feature + Task 7 test console) and confirm it.
- **NEXT after the Stage 3 road tests — NSB "coast-follow" (owner, 2026-10-20):** on this car any negative request the
  engine can't meet is braked to (the VSA tracks the request; inferred from Stage 2), so NSB's fixed floor (−0.3 / 0.0)
  either brakes gently downhill or rolls with no engine braking. Instead, while NSB holds the brakes off, request the car's
  own closed-throttle coasting accel (filtered aEgo), confirmed at closed-throttle drag by the live 0x130 axle torque, leaning
  slightly below to avoid throttle; overspeed backstop unchanged. Default OFF, sim on the measured-car plant, one targeted
  downhill test (watch torque / decel / the cluster's mini-car brake lights). Do it after Stage 3, not before.
  *(Task 17: EngineBrakePreferred's per-gear clamp now does the 'request within closed-throttle decel' part for any
  slow-down request; coast-follow is still the NSB-specific 'drift over set' piece — and, per the Task 17 descent finding,
  the prerequisite for the overspeed sawtooth to happen at all on this car: the VSA holds a 0 request downhill too.)*
- **Brake-applied signal (2026-10-20):** none on the buses the harness sees — a hunt over the Stage 2 drive (every byte /
  16-bit field / bit on bus 0 and in sendcan vs driver and openpilot braking) found no field that tracks braking; the
  cluster's mini-car brake lights (which do light for ACC braking) are most likely fed over B-CAN via the gateway. Options:
  the cluster icon as a human-observed indicator (phone video synced to the logs), a second CAN tool on B-CAN, or UDS
  reads from the VSA (proprietary DIDs; parked first). Until then braking is inferred (decel vs 0x130 torque + road load).
  **S8 (parked press test, route `00000005`, 2026-10-20):** `VSA_STATUS.USER_BRAKE` (0x1A4 bytes 0–1, already in the DBC,
  `carState.deprecated.brake`) is the **driver's pedal pressure** — rest −0.02, light ≈ 0.5, medium 1.6–3, hard ≈ 6, back at
  rest on release. It stays at rest while openpilot brakes (Stage 2: −1.2 m/s² requests, decel up to −1.5 m/s², no
  change), so it doesn't show the VSA's own braking. `COMPUTER_BRAKING` (bit 23) isn't brake-applied either: on ~50 % of
  engaged time whatever the request, also on throttle. Analysis: `analysis/stage1/brake_press_hunt.py`;
  `docs/s8-brake-pressure-2026-10-20.md`.
- **Downhill braking authority (Task 14 finding, beyond the handback):** in the sims openpilot cannot stop for a stopped
  car on a 3–6 % descent in ANY mode (manual / automatic, adaptive / non-adaptive, handback or not) when the car's brake
  delivers force rather than tracking the requested decel: openpilot's own loop has no grade term on this car (`kiV = 0`)
  and the Honda Bosch path sends `ACCEL_COMMAND` = the plan's accel uncompensated above 3 m/s (`hill_brake` feeds only the
  gas side; MVL's braking-only `brake_pid` closes the loop below 3 m/s). With a brake that tracks the request the same
  sweeps are clean. So downhill braking rests entirely on the VSA tracking `ACCEL_COMMAND` — unverified (C1, now with a
  grade). Not changed in code: adding grade to the braking command is an authority change that needs on-car data.
- Whether to include PR #2050's radarless brake-strength fix or intentionally run with weak/no openpilot
  braking for the "coast downhill" behavior. (Braking authority itself is the negative `ACCEL_COMMAND`,
  not `0x1BE` — corrected 2026-10-16.)
- Confirm `GEAR_MT`/`PEDAL_GAS`/`ENGINE_RPM` decode correctly on this specific car with a cabana
  log once the comma is installed (values match actual gear/clutch/RPM).
- **Lateral on the comma four (session 10, 2026-10-21):** weaker / wider than the owner's earlier stock-SP 3X. Not our
  fork (see Session 10). Open: what the 3X ran (model, NNLC/torque); NNLC and model A/Bs; whether the Civic EPS honors
  commands above 4096 (MVL's 5120). Score each A/B with `analysis/session10/lateral.py` on the same roads/bends.
- ~~Owner decisions pending (session 10)~~ — decided 2026-10-21: coast cap, accel boost and long-press → NonAdaptive
  built (default OFF / OFF / ON); NNLC enabled on the device instead of the STEER_MAX A/B (that A/B stays the fallback
  if NNLC overshoots). On-car next: NNLC on the same bends (`lateral.py`), then the two toggles.

## Device install: source vs prebuilt (`docs/PREBUILT.md`)
The car runs the fork from either **`civic-6mt-dev-test`** (source; builds on the device) or **`civic-6mt-prebuilt`**
(flattened, compiled on the device for **AGNOS 19.7**, published from the PC by `scripts/rebuild-prebuilt.sh`).
Install prebuilt: custom software `ADKMechETECH/civic-6mt-prebuilt`, or
`git clone -b civic-6mt-prebuilt --depth 1 https://github.com/ADKMechETECH/openpilot.git` (no submodules in it).
- **Pure Python / `.dbc` changes:** push to dev-test; a dev-test install just `git pull`s. *A prebuilt install only sees
  what was last published — rerun the script (no recompile, quick).*
- **Recompile → `scripts/rebuild-prebuilt.sh`:** new/renamed **Params keys (`params_keys.h`)** — nearly every feature
  task — C/C++, Cython `.pyx`, panda firmware / `opendbc/safety`, cereal `.capnp` / services, the Python EKF generators
  (`locationd/models/*_kf.py`, rednose), models (`.onnx`), SConstruct/SConscript.
- **AGNOS version change:** full rebuild on the device after it updates.
- Binaries embed `/data/openpilot`: always built there (the script swaps a build checkout in and puts the live install
  back). The device has no GitHub credentials: publishing is from the PC.

## Build environment
Linux via WSL2 (Ubuntu 24.04). Clone into the WSL home (`~/`), not `/mnt/e` (slow across the
mount). Keep this project folder (on E:) for docs/context; keep the code clone in WSL.
See `recovery/bootstrap.sh` and `recovery/MANIFEST.md` to rebuild this environment from scratch.

## Environment recovery — standing rule
`recovery/` (`bootstrap.sh` + `MANIFEST.md`) must stay a true, re-runnable record of how to
rebuild this dev environment from a bare Ubuntu 24.04 box. On **every** future environment
change (new apt package, new Python dep, new system tool, changed build step, etc.), in the
same step:
1. Update `recovery/bootstrap.sh` to perform the change idempotently.
2. Update `recovery/MANIFEST.md` (ordered log + tool versions + apt list + freeze files) to
   describe it.
3. Re-mirror `recovery/` to `/mnt/e/Dropbox/Family Drive/Brian/Projects/CommaAI/recovery/`.
4. Whenever `docs/` changes, also re-mirror it to `/mnt/e/Dropbox/Family Drive/Brian/Projects/CommaAI/docs/`
   (`rsync -a --delete ~/civic-cruise/docs/ ".../CommaAI/docs/"`). The older originals in
   `.../CommaAI/Civic-6MT-NonAdaptive-Cruise/docs/` are left untouched.

## Upstream chain / fork resilience
**Branch topology (openpilot fork):** `sp-honda-dev-202608` = the clean, upstream-able **feature branch** (Tasks 1–6;
every feature commit and promotion cherry-pick comes from here). `civic-6mt-dev-test` = feature + Task 7
instrumentation (test console) — **the branch the car is flashed from**. One-way flow only: feature → dev-test
(`git switch civic-6mt-dev-test && git merge --no-ff sp-honda-dev-202608` after feature work). **Never** merge or
cherry-pick dev-test into the feature branch or any base. opendbc has no dev-test branch (no change needed); if one is
ever needed, same rule on a matching opendbc branch. Details: `docs/FORK-RESILIENCE.md` §8.

MVL tracks sunnypilot, which tracks comma. Both repos have `origin`/`upstream`(MVL)/`sunnypilot`/`comma`
remotes; the base is pinned by tag; our features cherry-pick onto sunnypilot master. Lineage, the
MVL-delta inventory (what to carry if MVL vanishes) and the promotion playbook are in
`docs/FORK-RESILIENCE.md`. Offline bundles: `backups/`. Fetch with `--no-recurse-submodules`.

Do not defer any of these three to a later commit.
