# Feature toggles — what they do, defaults, and warnings

All features are **SIM/TEST-ONLY until verified on the car** (see the on-car checklist in CLAUDE.md).
Every toggle defaults to the stock behavior. The feature branch has no UI for them; set them on the device with
the Params store, e.g. `echo -n 1 > /data/params/d/NonAdaptiveLongitudinal` (or, on the dev-test build, from the test
console's session pages — see below). They take effect the next time **openpilot is disengaged** (Task 12 B3: every
toggle and param latches only while `selfdriveState.enabled` is false — a gas override, a shift or the low-speed
handback no longer count as "not engaged"), and the latched values are published (`longitudinalPlanSP.features` /
`.noServiceBrake`, `carControlSP.manualLongitudinal`) so the test console checks what is actually active.

**Rule 5 (Task 12): one braking-urgency predicate** (`braking_urgency.py`) — FCW, driver-monitoring force-decel,
stock AEB/FCW (dead on the radarless Civic while openpilot long runs; kept for other platforms), a stop or hard
(< −1.5 m/s²) plan request, an invalid **or stale** plan / radarState, or a close / closing / braking lead. Every
feature below that pauses longitudinal, raises the lower accel limit or ramps braking steps aside **in the same control
frame** when it reports a reason: the lower limit is back at stock and the plan's braking passes through. The one
exception is physical: during a clutch-gate pause with the gas pressed the panda allows only a 0 command (the same
outcome as stock's gas override). Two features use a narrower set **on purpose**: non-adaptive re-admits lead braking
only for a collision threat, and the low-speed handback freezes only for its own emergencies (Task 13 A1, below).

> ## ⚠ Read first
> - **These features act on openpilot's own longitudinal control.** Enabling openpilot longitudinal on the
>   Honda Civic (radarless) **disables the car's stock AEB/collision-mitigation braking for the entire drive** —
>   engaged or not — because the panda statically blocks the camera's `ACC_CONTROL` (0x1C8) that carries it.
>   That consequence is **platform-specific**: some other cars keep stock AEB with openpilot longitudinal,
>   others don't. Nothing here was validated on any car except the 2022+ Civic, and nothing here has been
>   road-tested yet. Verify the specific car's openpilot-longitudinal braking and AEB behavior before using
>   any of this anywhere else.
> - With **Non-adaptive** on, openpilot **does not follow traffic** — it holds the set speed, and you slow down for
>   ordinary traffic. Since Task 12 (owner decision 2026-10-19) it **does brake for a real collision threat**: FCW, a
>   lead inside 6 s time-to-collision, a lead closer than max(10 m, 1 s) that is closing or braking, a lead braking
>   hard within 3 s, or the lead planner itself needing more than 1 m/s² of braking re-admits openpilot's normal lead
>   braking (held 2 s after the threat clears, then back to set speed without a step). A slower car far ahead or steady
>   following is not a threat. FCW is unchanged; it is an emergency-threshold alert, not a gentle "closing" warning.
> - On a **manual**, openpilot hands longitudinal back below ~5 mph (adaptive and non-adaptive), with a prompt from
>   the start of the hand-back ramp. **In ordinary traffic you take the stop** (Task 13 A1): approaching slow or stopped
>   cars tens of metres ahead, it hands back at that speed exactly as with no lead. **Emergency braking is never handed
>   back:** with FCW, a hard braking request, invalid inputs, or a car so close that you couldn't stop for it yourself
>   (braking at 2 m/s² after the hand-back plus a 1.5 s reaction, 2 m short — a downhill makes that distance longer), the
>   brakes stay available and openpilot **may brake the car to a stop** — be ready with the clutch. If that car drives
>   off before you've stopped, openpilot hands back about a second later instead of staying in control (Task 14). It
>   does not hold the car at a stop: 0.5 s after it stops it hands back anyway (the car's Auto Brake Hold / hill-hold and you hold it), and it never launches. Driver-monitoring
>   force-decel blocks the handback (stock stops the car; a stall is acceptable then).
> - With **No-service-brake** on, openpilot **will not brake to hold your set speed** on downhills — the
>   car runs over set speed until the overspeed backstop (if on) lets it brake. It never caps braking for a curve-speed
>   (SCC-V), map (SCC-M) or speed-limit (SLA) target below the set speed, or for any Rule-5 reason. On the radarless Civic
>   the backstop is only as strong as openpilot's radarless braking (negative `ACCEL_COMMAND`; brake
>   strength is the open PR #2050 question — unverified). *(Corrected 2026-10-16: previously said this
>   hinged on `0x1BE` in the fingerprint — that is the alternate user-brake-pedal message, not brake
>   authority, and isn't fingerprinted on CIVIC_2022.)* In `NoServiceBrakeOverspeedMode` **pulse**, past set +
>   15 mph you get **"Overspeed: Take Over"**: the bounded pulse couldn't hold the grade — slow the car yourself. The
>   alert repeats every 4 s while you are still over that and speeding up, in both modes.
> - With **Engine-brake-preferred** on, openpilot **holds off the service brakes for gentle decel** and relies on
>   engine braking in your current gear, using an **uncalibrated** physics estimate of how much the engine brakes.
>   If that estimate is too optimistic the car coasts and **closes on a lead** before a backstop (closing/close lead,
>   decel steeper than the hard floor, FCW, "car isn't slowing as planned") hands back to the brakes. In the sim, a 5×
>   over-estimate cost ~2 m of gap vs. stock. Not validated on-car — leave it off until the on-car checklist says so.

| Param | Type / default | Scope | What it does |
|---|---|---|---|
| `NonAdaptiveLongitudinal` | bool, **off** | any car with openpilot longitudinal | Set-speed only: ignore lead cars for following and the experimental-mode model; hold the set speed. **Collision braking stays** (Task 12 A1, see Read first): on a real threat the lead MPC is re-admitted, braking is allowed and no-service-brake is released. Keeps stock's model-predicted-lift throttle cap (`allow_throttle`): it only limits throttle (coasts), it never brakes. The cluster's lead icon is hidden unless openpilot is braking for a collision (`hasLead` stays in the logs). FCW unchanged. No-op with stock ACC. |
| `NoServiceBrakeHold` | bool, **off** | any car with openpilot longitudinal; **only acts while `NonAdaptiveLongitudinal` is active** | Coast over set speed: openpilot won't command more than `NoServiceBrakeDecelFloor` of decel to hold set speed. Released at once (both plannerd and controlsd, never debounced) for any Rule-5 reason, for non-adaptive collision braking, and whenever SCC-V / SCC-M / SLA lowers the target below the set speed (overspeed is measured against that effective target). Driver-monitoring force-decel is never limited. |
| `NoServiceBrakeDecelFloor` | float m/s², **0.3** | with the above | Most decel openpilot may command while clamped. `0.0` = pure throttle-only classic cruise. The 0.3 default stays until on-car test C2 shows whether a small negative `ACCEL_COMMAND` uses the service brakes. |
| `NoServiceBrakeOverspeedBackstop` | bool, **on** | with the above | Overspeed backstop: openpilot brakes again once speed exceeds set speed + margin. How it brakes is `NoServiceBrakeOverspeedMode`. OFF = never brakes for overspeed (pure coast). |
| `NoServiceBrakeOverspeedMargin` | float m/s, **3.6** (≈ 8 mph), clamped 0.5…6.7 | with the above | Backstop margin; in pulse mode the pulse **trigger**. Task 13 A2: at most a **fixed 6.7 m/s** (≈ 15 mph) in both modes, whatever `…HardCeiling` is set to (raising the ceiling to 11 used to move the braking point to ~25 mph over), and always ≥ 0.45 m/s under the ceiling, so braking starts before the "Overspeed: Take Over" point. |
| `NoServiceBrakeOverspeedMode` | string, **`maintain`** \| `pulse` | with the backstop | `maintain` (the original backstop, unchanged): above set + margin normal speed-hold braking is allowed until back under set + margin/2 — on a grade steeper than that braking can hold, it **rides the brakes** at set + margin, and it steps the brake on/off at each boundary. `pulse` (Task 11): a hysteretic, **bounded brake pulse** — coast (no-brake clamp) up to set + margin, then roll a decel on at `…PulseJerk` down to at most `…PulseDecel`, hold it until back at set + `…ReleaseMargin`, roll it off at the same jerk and coast again: distinct braking events, never a continuous hold. If the bounded pulse can't stop the speed rising (steep grade / weak brakes) and the car passes set + `…HardCeiling`, it **escalates** to openpilot's set-speed braking (in non-adaptive that is the cruise plan's −1.2 m/s² cap, not "full" braking) with an **"Overspeed: Take Over"** alert (repeats every 4 s while still over the ceiling and speeding up; not for 2 s after you lower the set speed) until back under set + release (or disengaged). `maintain` gets the same alert above the ceiling. Any Rule-5 reason, a collision threat or a lowered SCC/SLA target bypasses the pulse (no clamp); driver-monitoring force-decel is never limited. `maintain` keeps its brake step at each boundary on purpose (`pulse` is the comfort fix); apart from Rule 5 and the alert it is bit-identical to the original backstop. Latched only while openpilot is disengaged. Tune only after C1 (brake strength) and the `maintain` feel (NSB2) are known. |
| `NoServiceBrakeOverspeedReleaseMargin` | float m/s, **0.9** (≈ 2 mph), clamped 0–3.6 | pulse mode | A pulse ends at set + this. Kept ≥ 0.45 m/s under the trigger. |
| `NoServiceBrakeOverspeedPulseDecel` | float m/s², **−1.0**, clamped −2.5…−0.3 | pulse mode | The pulse's decel bound. In non-adaptive mode the plan itself never asks for more than −1.2 m/s² (the set-speed planner's floor; this car's PID adds nothing), so a bound stronger than −1.2 has no extra effect there. |
| `NoServiceBrakeOverspeedPulseJerk` | float m/s³, **1.5**, clamped 0.5–3.0 | pulse mode | Roll-on and roll-off rate of each pulse (`maintain` steps the command by up to ~0.9 m/s² instead). |
| `NoServiceBrakeOverspeedHardCeiling` | float m/s above set, **6.7** (≈ 15 mph), clamped 4.5–11 | backstop (both modes) | Pulse mode: escalation threshold (set-speed braking + "Overspeed: Take Over"); that braking is the set-speed planner's (−1.2 m/s² floor in non-adaptive) — the same strength `maintain` uses; only as strong as the radarless brake authority (C1). Both modes: the overspeed alert threshold. It can only pull the margin down (margin ≤ ceiling − 0.45), never raise it past 6.7. |
| `ManualLowSpeedHandbackSpeed` | float m/s, **2.24** (≈ 5 mph), clamped 1.5–8.0 | manual transmissions with openpilot longitudinal | **Always on (not a toggle).** Below this speed openpilot ramps its command to 0 and disengages, with the prompt "openpilot is handing longitudinal back to you" from the start of the ramp (priority below FCW — it never masks "BRAKE!"); it won't engage below it + 0.5 m/s. Non-urgent — **including an ordinary approach to slow or stopped traffic** (Task 13 A1) — openpilot never brakes a manual to a stop, holds it, or launches it — you and the car's Auto Brake Hold / hill-hold do. **Urgent (the handback's own set: FCW, stock AEB, a plan asking for > 1.5 m/s², invalid/stale inputs, or an *imminent* lead — one a driver braking at 2 m/s² after the ramp + 0.1 s + 1.5 s couldn't stop 2 m short of; a downhill grade from the calibrated pose lengthens that, uphill / no pose count as level): the ramp freezes, braking stays at the stock limit and openpilot may brake to a stop**. An imminent lead latches (Task 14): set by the imminence test whatever else is urgent (FCW, a hard request), held while a lead is still closing within that distance + 2 m, released 0.5 s after none is — a lead that drives off doesn't leave openpilot engaged. The ramp restarts from the current command once the urgency clears, or **0.5 s after the car has stopped** (standstill resolution — never a held stop); FCW / stock AEB are never resolved away, even after a resolution. The plan's own "stop" request and a lead merely closing somewhere ahead do not freeze it. Driver-monitoring force-decel: no handback (stock stops the car). controlsd owns the state and the latched threshold/ramp and publishes them; selfdrived only acts on what it published. |
| *(no Param)* gas-override exit | — | manual transmissions with openpilot longitudinal | **Always on (Task 16, not a toggle).** When the driver lifts off the gas after overriding openpilot, the plan restarts from `min(current accel, 0)` (coast first, then the plan) instead of the driver's own acceleration — on a manual the throttle closes the instant the driver lifts, and resuming at e.g. +1.8 m/s² made the car re-accelerate for ~1.5 s to 5.5 mph over set (Stage 2). Automatics unchanged. |
| `ManualLowSpeedHandbackRamp` | float s, **0.5**, 0–2 | with the above | How long the throttle (and, non-urgent, braking) side ramps to 0 before the disengage (smoothness). |
| `ClutchShiftGate` | bool, **off** | manual transmissions only | During a shift (clutch open = drivetrain decoupled: since Task 15 the clutch pedal leaving its top switch — `GAS_PEDAL_2` bit 50, the earliest cue, as the pedal starts to move — or any `GEAR_MT` value other than 1–6 / 14, or a phantom gear `GEAR_MT` reports at idle in neutral) **openpilot stays engaged and keeps braking**: only the throttle side is capped at `ClutchShiftGateCoastAccel` (Task 12 A2 — it used to drop longitudinal, i.e. no brakes, for the whole shift). Never a positive command with the clutch in. A rev-match / heel-toe blip during the shift is not an override and the command is exactly 0 then (what the panda requires with the gas pressed); a blip outside a paused shift (e.g. after the shift timeout) is a normal gas override. Requires `DisengageOnAccelerator` **off** and a live clutch signal (`GEARBOX_ALT_2`): with either missing it refuses to turn on (HUD "Shift gate off"); if the signal dies mid-engagement it goes inert. HUD "Shift: coasting" while it caps the throttle. RPM-flare backstop: 3 consecutive new RPM samples rising > 1500 rpm/s (filtered), gas not pressed, not within 0.5 s of a decoupled drivetrain (Task 15 hold-off); rests 10 s after 2 flares in 10 s. Resume (Task 15): 0.15 s after the drivetrain reads coupled again (restarted if the reported gear changes) — on this car `GEAR_MT` reports the new gear only once the clutch is fully out and the rpm already matches, so the old 0.5 s was extra coast. Needs real RPM: until Task 15 the fork read `ENGINE_DATA` RPM, which is 0 on this car, so the backstop never fired. |
| `ClutchShiftGateGasReleaseHint` | bool, **off** | with the above | Also cap the throttle briefly after lifting off the gas, in case a clutch press follows. Delays the resume after every gas override. |
| `ClutchShiftGateCoastAccel` | float m/s², **0.0**, clamped −0.5…0.0 | with `ClutchShiftGate` | The throttle cap during a shift. The right value depends on on-car test C2 (does `ACCEL_COMMAND` 0 coast, or hold speed?). |
| `ClutchShiftGateMaxCoast` | float s, **5.0**, clamped 2–5 | with `ClutchShiftGate` (always active with it) | Shift timeout: clutch in longer than this isn't a shift but coasting in neutral → `coastTimeout`: no throttle while `GEAR_MT` reads 0, braking as normal (it is never paused anyway since A2). A real shift never gets near 5 s. **Task 15:** with a live clutch pedal switch, neutral with the clutch out (decoupled, pedal fully up for 0.5 s) goes to `coastTimeout` at once; a clutch held down is still a shift until this timeout. |
| `ClutchReengageSmart` | bool, **off** | with `ClutchShiftGate` | Smart re-engagement (Task 8, corrected in Task 12 B4). OFF = the fixed re-engage delay (0.15 s since Task 15, was 0.5 s) after the drivetrain reads coupled; since Task 12 the resume is still rate-limited from the command running through the shift (no step). ON: (1) re-engage on **ratio lock**: the gear inferred from rpm ÷ speed must be the gear `GEAR_MT` reports, within `ClutchReengageRatioTol` of its ratio, steady (≤ 1 %) for 0.12 s on filtered rpm/speed — no lock within 1 s falls back to the fixed delay (logged); below ~3 m/s / the handback speed the fixed delay is used; (2) **one takeover rule**: coupled **and** gas released for `ClutchReengageGasDebounce`; (3) **takeover ramp** from the seed (`ClutchReengageSeed`) to the plan at the gear's rate, escalating to ≤ 3 m/s³ within `ClutchReengageMaxRamp`, inside the NSB/EBP floors. Any Rule-5 reason, or the plan dropping by > 0.5 m/s² (a step brake request), ends the ramp at once. For A/B on the car against OFF. **Task 15:** the fixed delay is now shorter than lock + steady + gas debounce, so ON no longer takes over *earlier* on this car; what it still adds is waiting for the gas lift and the seeded ramp. |
| `ClutchReengageGasDebounce` | float s, **0.15**, clamped 0–0.5 | with the above | How long the gas must be released before openpilot takes over after a shift (stops grab/yield flutter when feathering the throttle). |
| `ClutchReengageRatioTol` | float, **0.02**, clamped 0.01–0.04 | with the above | The **lock** tolerance (Task 12 B4), separate from and much tighter than the ±8 % gear identification. |
| `ClutchReengageSeed` | string, **`aego`** \| `coast` | with the above | Where the takeover command starts: `aego` = the car's measured accel (the Task 8 behavior), `coast` = aEgo minus the engine-brake coast estimate. **On-car test C2 decides**: if a negative `ACCEL_COMMAND` adds service brakes on top of engine braking, `aego` after a downshift feels like a brake tap and `coast` is the right seed. |
| `ClutchReengageMaxRamp` | float s, **1.0**, clamped 0.3–2.0 | with the above | Longest the takeover ramp may last; at the end it hands over to the normal controller. |
| `EngineBrakePreferred` | bool, **off** | manual transmissions with openpilot longitudinal | For gentle, non-urgent decel that engine braking in the current gear can cover (estimate × 0.7, grade included), command the coast target instead of the brakes; anything steeper or urgent passes through unchanged (hard hand-off). Backstops: any Rule-5 reason (incl. a stale plan/radar), close/closing/braking lead with its own thresholds, demand (now or in the next 1 s of plan) past the hard floor, not in gear, no pitch, no live clutch signal, and "car isn't slowing as planned" (which also turns it off **for the rest of the engagement** — a shift or a gas override does not clear it, only a disengage). Works in adaptive and non-adaptive mode; composes with `NoServiceBrakeHold` (higher floor wins). **Keep OFF until the coast model is refit (Stage 4):** Stage 1 (2026-10-20) measured the seed over-predicting flat-road engine braking by up to 0.26 m/s² in 2nd (crank drag 23–27 Nm vs the seed's 27–38); the 0.7 credit only just covers 2nd. |
| `EngineBrakeHardFloor` | float m/s², **−1.5**, clamped −2.0…−0.3 | with the above | Demand steeper than this always goes to the brakes. |
| `EngineBrakeCoastTarget` | float m/s², **−0.2** (Task 16; was 0.0), clamped −0.3…0.0 | with the above | Lowest command while the brakes are held off — the request that gets **engine braking**, not a brake threshold. Stage 2 C2 (on the car): a command of 0 holds ~zero engine torque (the car rolls, no engine braking); a small negative closes the throttle (fuel cut: −0.17 gave −0.38 m/s² in 3rd, no brakes). The brakes join only past the current gear's engine braking (≈ −0.5 in 3rd, stronger in lower gears) — and that hand-off is the separate credited-engine-braking estimate, not this value. |
| `EngineBrakeLeadTtc` | float s, **6.0**, clamped 3–15 | with the above | A lead with time-to-collision under this disables the feature. |
| `EngineBrakeLeadClosingSpeed` | float m/s, **1.0**, clamped 0.2–3.0 | with the above | A lead closing faster than this disables the feature (also: any lead under 1.0 s headway / 10 m, or braking harder than 1 m/s²). |
| `GearAwareAccel` | bool, **off** | manual transmissions with openpilot longitudinal | Gear-indexed scaling of the set-speed accel-rate (jerk) limit: 1st ×0.5, 2nd ×0.65, 3rd ×0.8, 4th ×0.9, 5th/6th ×1.0. Applies to throttle-on, and to lift-off while the target stays within what engine braking covers; braking gets the stock rate. Fully stock (both directions) under any Rule-5 reason, with no pitch, or with no live clutch signal. Task 13 A3: under a Rule-5 reason controlsd runs the stock-jerk plan (`longitudinalPlanSP.features.aTargetStock`, published only while this is on) **in the same control frame** — not up to 50 ms later at the next plan — and plannerd steps the shaped set-speed candidate down to the stock one. Never looser than stock; the lead-following MPC (and FCW) is untouched. |

Dev-test build only (`civic-6mt-dev-test`): the **on-car test console** (Tasks 7/9/10, `docs/TEST_CONSOLE.md`) has its own
`TestConsole*` Params (selected test, armed, background capture, start button/gesture; `TestConsoleStartButton` empty
or unknown = the vehicle profile's default button). Its daemon has no driving authority and writes nothing. Since
Task 9 its **session view can set the toggles above** for the session you're on — a deliberate, bounded widening,
listed per car in its vehicle profile (Task 10); on the Civic 6MT only `NonAdaptiveLongitudinal`, `NoServiceBrakeHold`,
`NoServiceBrakeDecelFloor`, `NoServiceBrakeOverspeedBackstop`, `NoServiceBrakeOverspeedMargin`,
`NoServiceBrakeOverspeedMode` (Task 11), `ClutchShiftGate`, `ClutchShiftGateGasReleaseHint`, `ClutchReengageSmart`,
`EngineBrakePreferred`, `GearAwareAccel`, `DisengageOnAccelerator` (plus `TestConsole*`); any other key is refused, and
on a car with no profile everything but `TestConsole*` is (openpilot longitudinal itself always stays in the Developer
panel). It's the same as setting them here:
the features still latch only while openpilot is disengaged, and since Task 12 D2 the console refuses to write a toggle
while openpilot is engaged at all. Not present on the feature branch.

Not a toggle: **coast-event logging** for calibrating the engine-brake model. On manuals, in-gear coasts (clutch
engaged, no pedals, openpilot not commanding longitudinal, no AEB/FCW, ≥ 1 s) are written as JSON lines to
`/data/engine_brake_coast/coast.jsonl` (rotating, ≤ ~11 MB) plus an `engine_brake_coast` cloudlog event. The file I/O
runs on a background thread (a full queue drops an event and logs it; plannerd never blocks). Logging only — it can't
change driving.

Not a toggle either (Task 12 B5, opendbc): on Bosch-radarless manuals `GEARBOX_ALT_2` (0x1DD) is registered with NaN
frequency, so its absence never invalidates CAN, and `CarStateSP.clutchSignalValid` / `gearRaw` say whether it is alive
and what `GEAR_MT` reads. The clutch gate, smart re-engagement, engine-brake-preferred and gear-aware accel are inert
without it. Task 13 A3: "alive" also means clean — a bad 0x1DD counter or checksum reads not-live too, and (the
parser's own counter check being off for 0x1DD) a bad counter can never invalidate CAN for the whole car.

Why the coupling: clamping braking while lead-following could stop openpilot braking for a slower car, so
`NoServiceBrakeHold` does nothing unless non-adaptive (which doesn't follow leads) is active — and even then the
non-adaptive collision braking and every Rule-5 reason release it. That release uses the **broad** lead reasons (any
lead closing ≥ 1 m/s, at any distance), wider than the handback's imminent-lead test — kept on purpose (Task 13 A3,
documented): it only gives back openpilot's own set-speed braking (−1.2 m/s² at most in non-adaptive), the fail-safe
direction; the cost is less coasting with traffic ahead on a descent. Narrowing it waits for on-car data (C1, NSB1/2).

Platform split: clutch/transmission features are manual-only; longitudinal-behavior features
(non-adaptive, no-service-brake) are transmission- and platform-agnostic and gated only on openpilot
longitudinal being active. Engine-brake-preferred and gear-aware accel are manual-only: they need the gear, which
is inferred from engine RPM ÷ wheel speed (clutch in, neutral, a slipping clutch or no RPM signal all read as "no
gear", which turns both off).
