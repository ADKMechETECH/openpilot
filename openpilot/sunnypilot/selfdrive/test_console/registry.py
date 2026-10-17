"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console registry: the complete, ordered on-car validation plan for the non-ACC controls (Tasks 1-6).

Pure data (the UI imports it too). Each row names its pieces instead of holding code, so a test is added by adding
a row here — no rewiring:
  conditions  names in conditions.CONDITIONS; a sample QUALIFIES only while all hold (plus the toggle checks)
  toggles     Params the test expects ON/OFF. Read-only checks: the console never sets a toggle
  analyzer    "segments" (qualifying time, optionally binned) | "episodes" (detected maneuvers) | a custom name
  goal        the analyzer's parameters (what "complete" means)
  result      names in analyzers.RESULTS: turns the capture into PASS / FAIL / CAPTURED + a one-line detail
  passes      the pass criterion in words ("" = characterization only: CAPTURED, no PASS/FAIL)
  fields      Sample fields written per qualifying sample to the capture file
  prereqs     tests recommended complete first (a soft hint — never a block; driver judgment)

Two kinds of test:
  TELEMETRY  passive, no toggle, order-independent: collect opportunistically on any drive (also in the background
             with TestConsoleBackground on)
  FEATURE    needs openpilot long and/or a toggle: follow the phase order

Phases 0, 1, 2 and the Phase 7 calibration captures (first_run=True) are the FIRST thing to run on the comma four:
they turn the car from unknown to known, and every feature's tuning and trust depends on them.
"""
from dataclasses import dataclass, field
from enum import StrEnum


class Kind(StrEnum):
  TELEMETRY = "telemetry"
  FEATURE = "feature"


@dataclass(frozen=True)
class Phase:
  num: int
  title: str
  setup: str   # how the car must be set up for this phase
  gates: str   # what this phase gates


PHASES: dict[int, Phase] = {p.num: p for p in (
  Phase(0, "Signal decode", "parked -> low speed, engine running; console labels + cabana",
        "every manual feature (gear inference, clutch gate)"),
  Phase(1, "Baseline drive", "openpilot long (alpha) engaged, all feature toggles OFF",
        "trusting any on-road feature test"),
  Phase(2, "Command authority", "openpilot long engaged, flat empty road",
        "no-service-brake, overspeed backstop, engine-brake-preferred, the coast target"),
  Phase(3, "Clutch/shift gate", "ClutchShiftGate ON, DisengageOnAccelerator OFF",
        "real road use of the longitudinal features"),
  Phase(4, "Non-adaptive set-speed", "NonAdaptiveLongitudinal ON", "phases 5 and 6"),
  Phase(5, "No-service-brake coast", "NonAdaptiveLongitudinal + NoServiceBrakeHold ON (needs C1)", ""),
  Phase(6, "Low-speed handback", "manual, always on (Task 5); needs phase 4 working", ""),
  Phase(7, "Engine-brake model", "calibration: no toggle; validation: EngineBrakePreferred ON", "phase 8"),
  Phase(8, "Gear-aware accel", "GearAwareAccel ON; needs E0c (gear inference)", ""),
)}

GEARS = (1, 2, 3, 4, 5, 6)


@dataclass(frozen=True)
class TestSpec:
  id: str
  phase: int
  kind: Kind
  label: str                      # short: fits a mici button
  how: tuple[str, ...]            # what the driver does (one card per entry)
  conditions: tuple[str, ...]
  analyzer: str
  goal: dict
  result: str
  passes: str
  fields: tuple[str, ...]
  prereqs: tuple[str, ...] = ()
  toggles: dict[str, bool] = field(default_factory=dict)
  first_run: bool = False         # part of the first-run characterization on the comma four
  optional: bool = False
  sample_hz: int = 20             # capture-file rate for qualifying samples (analysis always runs at 100 Hz)


BASE_FIELDS = ("t", "v_ego", "a_ego", "gas", "brake", "clutch", "gear_mt", "gear", "rpm")
LONG_FIELDS = ("enabled", "long_active", "accel_cmd", "accel_out", "accel_sent", "a_target", "v_cruise")
COAST_FIELDS = ("t", "rpm", "v_ego", "ratio", "a_ego", "pitch", "gear", "gear_mt")  # Task 6 coast-log layout + gears

GATE_ON = {"ClutchShiftGate": True, "DisengageOnAccelerator": False}
NON_ADAPTIVE_ON = {"NonAdaptiveLongitudinal": True}
NSB_ON = {"NonAdaptiveLongitudinal": True, "NoServiceBrakeHold": True}
EBP_ON = {"EngineBrakePreferred": True}

TESTS: tuple[TestSpec, ...] = (
  # --- Phase 0: signal decode ------------------------------------------------------------------------------------
  TestSpec("S1", 0, Kind.TELEMETRY, "GEAR_MT decode",
           ("Parked, engine on: clutch down in gear, each of 1-6, reverse, then true neutral with the clutch OUT.",
            "Then drive through gears 1-6. Agreement with RPM/speed is scored per gear.",
            "Use the start button to mark the parked neutral/clutch-out window for cabana."),
           ("manual", "raw_can"), "gear_mt_map",
           {"seconds_per_gear": 2.0, "stop_seconds": 5.0},
           "gear_mt_map", "every gear 1-6 agrees with the RPM/speed ratio >= 95% of rolling samples; reverse reads 14",
           BASE_FIELDS + ("ratio", "reverse", "standstill"), first_run=True),
  TestSpec("S2", 0, Kind.TELEMETRY, "GEAR_MT lag",
           ("Normal up- and downshifts above 10 mph.",
            "Measures how long after GEAR_MT reports a gear the clutch actually bites (RPM locks to wheel speed),",
            "and whether GEAR_MT->0 leads or lags the engine decoupling on clutch-in."),
           ("manual", "raw_can", "rolling"), "episodes",
           {"detector": "shift", "count": 10},
           "bite_lag", "p90 bite lag <= REENGAGE_DELAY (0.5 s)",
           BASE_FIELDS + ("ratio",), prereqs=("S1",), first_run=True, sample_hz=100),
  TestSpec("S3", 0, Kind.TELEMETRY, "Engine RPM sanity",
           ("Cruise steady in gears 3-6 on a flat road. Compare the chip RPM with the tach.",
            "Scores RPM noise against the 1500 rpm/s flare threshold."),
           ("manual", "rolling", "gear_mt_in_gear", "clutch_out", "steady"), "segments",
           {"seconds": 60.0, "min_segment": 2.0},
           "rpm_noise", "no false RPM flares (> 1500 rpm/s while steady in gear)",
           BASE_FIELDS + ("rpm_rate", "ratio"), first_run=True),
  TestSpec("S4", 0, Kind.TELEMETRY, "Gas-lift to clutch",
           ("Normal shifts: lift off the gas, clutch in, shift.",
            "Measures lift -> GEAR_MT=0 time. Only matters if ClutchShiftGateGasReleaseHint is on."),
           ("manual", "rolling"), "episodes",
           {"detector": "shift", "count": 10, "filter": "after_gas"},
           "lift_to_clutch", "p90 lift->clutch <= GAS_RELEASE_WINDOW (0.6 s)",
           BASE_FIELDS, prereqs=("S1",), first_run=True, sample_hz=100),
  TestSpec("S5", 0, Kind.TELEMETRY, "Rev-match trace",
           ("Rev-matched / heel-toe downshifts: blip the throttle with the clutch in.",
            "GEAR_MT must stay 0 through the blip; the flare must not leak into the gate."),
           ("manual", "raw_can", "rolling"), "episodes",
           {"detector": "shift", "count": 5, "filter": "blip"},
           "blip_trace", "GEAR_MT stays 0 through every blip and the gate never leaves 'shifting' during it",
           BASE_FIELDS + ("rpm_rate", "gate"), prereqs=("S1",), first_run=True, sample_hz=100),
  TestSpec("S6", 0, Kind.TELEMETRY, "Clutch-bit hunt",
           ("Optional. Parked or rolling: press and release the clutch 10x, slowly.",
            "Looks for a raw clutch-switch bit in POWERTRAIN_DATA (0x17C) that toggles with GEAR_MT=0."),
           ("manual", "raw_can"), "bit_hunt",
           {"presses": 10},
           "bit_hunt", "", ("t", "gear_mt", "clutch", "raw_17c"),
           prereqs=("S1",), first_run=True, optional=True, sample_hz=100),

  # --- Phase 1: baseline -----------------------------------------------------------------------------------------
  TestSpec("D1", 1, Kind.FEATURE, "Baseline op-long drive",
           ("openpilot long (alpha) on, every feature toggle OFF. Drive normally, engaged, for 5 minutes.",
            "Task 1 regression: no ACC fault, no blocked frames, no fault alerts."),
           ("op_long", "long_active"), "segments",
           {"seconds": 300.0, "min_segment": 0.0},
           "baseline", "no CRUISE_FAULT, no panda-blocked TX frames, no fault alerts",
           BASE_FIELDS + LONG_FIELDS, first_run=True,
           toggles={"NonAdaptiveLongitudinal": False, "ClutchShiftGate": False, "NoServiceBrakeHold": False,
                    "EngineBrakePreferred": False, "GearAwareAccel": False}),

  # --- Phase 2: command authority --------------------------------------------------------------------------------
  TestSpec("C1", 2, Kind.FEATURE, "Brake strength",
           ("Flat empty road ~35 mph, engaged, no lead: drop the set speed 5+ mph at once so openpilot asks to brake.",
            "Repeat 5x. Compares the achieved decel with the request (PR #2050: may 'only accel/coast')."),
           ("op_long", "long_active", "no_pedals", "rolling", "cmd_brake"), "segments",
           {"count": 5, "min_segment": 1.5, "settle": 0.5},
           "brake_strength", "achieves >= 50% of the requested decel",
           BASE_FIELDS + LONG_FIELDS + ("a_coast", "excess", "brake_assist", "pitch"), prereqs=("D1",), first_run=True),
  TestSpec("C2", 2, Kind.FEATURE, "Cmd 0 / small negative",
           ("Flat road, engaged, in gear. Hold set speed (command ~0) for 10 s+,",
            "then nudge the set speed down 1-2 mph so openpilot asks for a small decel (-0.15..-0.5).",
            "Answers: does command 0 coast or hold speed; does a small negative use fuel-cut or the brakes?"),
           ("op_long", "long_active", "no_pedals", "rolling", "in_gear", "clutch_out", "flat"), "segments",
           {"bin": "cmd_band", "keys": ["zero", "small_neg"], "seconds": 10.0, "min_segment": 1.0, "settle": 0.5},
           "cmd_semantics", "", BASE_FIELDS + LONG_FIELDS + ("a_coast", "excess", "brake_assist", "pitch"),
           prereqs=("D1", "E0c"), first_run=True),

  # --- Phase 3: clutch/shift gate --------------------------------------------------------------------------------
  TestSpec("G1", 3, Kind.FEATURE, "Gate frame acceptance",
           ("FIRST gate test. Engaged above 10 mph, flat empty road: clutch in for 1-2 s WITHOUT shifting, then out.",
            "Repeat 5x. ACC_CONTROL keeps CONTROL_ON with no gas/brake for the whole press:",
            "must be no CRUISE_FAULT, no cruise cancel, no disengage."),
           ("manual", "op_long", "engaged"), "episodes",
           {"detector": "shift", "count": 5, "filter": "engaged_open"},
           "gate_accept", "every press: paused (no long), stayed engaged, no ACC fault",
           BASE_FIELDS + LONG_FIELDS + ("gate", "acc_faulted"), prereqs=("S1", "S2", "D1"), toggles=GATE_ON,
           sample_hz=50),
  TestSpec("G2", 3, Kind.FEATURE, "Gate on real shifts",
           ("Engaged: real upshifts and downshifts (3+ each), plus normal driving on hills.",
            "Checks: coasts through each shift, no throttle with the clutch in, smooth resume, no false flares."),
           ("manual", "op_long", "engaged"), "episodes",
           {"detector": "shift", "count": 6, "filter": "engaged_shift", "need": {"up": 3, "down": 3}},
           "gate_shifts", "every shift paused with no throttle while the clutch was in; resume jerk < 3 m/s^3; 0 false flares",
           BASE_FIELDS + LONG_FIELDS + ("gate", "rpm_rate"), prereqs=("G1",), toggles=GATE_ON, sample_hz=50),
  TestSpec("G3", 3, Kind.FEATURE, "Gate + rev-match",
           ("Engaged: 5 rev-matched / heel-toe downshifts. Then one plain brake press while engaged.",
            "The blip must not override or disengage; the brake must still disengage as stock."),
           ("manual", "op_long", "engaged"), "episodes",
           {"detector": "shift", "count": 5, "filter": "engaged_blip", "extra": {"detector": "brake_press", "count": 1}},
           "gate_revmatch", "stayed engaged and paused through every blip; brake disengaged within 0.3 s",
           BASE_FIELDS + LONG_FIELDS + ("gate",), prereqs=("G2", "S5"), toggles=GATE_ON, sample_hz=50),
  TestSpec("G4", 3, Kind.FEATURE, "Engine-brake downshift",
           ("Engaged, no lead: downshift to engine-brake WITHOUT touching the brake. 5x.",
            "Tuning: does openpilot add throttle after the 0.5 s re-engage delay and fight the engine braking?"),
           ("manual", "op_long", "engaged"), "episodes",
           {"detector": "shift", "count": 5, "filter": "engaged_downshift_no_pedal"},
           "eb_downshift", "", BASE_FIELDS + LONG_FIELDS + ("gate",), prereqs=("G2",), toggles=GATE_ON),

  # --- Phase 4: non-adaptive -------------------------------------------------------------------------------------
  TestSpec("N1", 4, Kind.FEATURE, "Non-adaptive hold",
           ("Engaged: hold the set speed 60 s+. Then follow behind a slower car (5 s+) - openpilot must NOT brake",
            "for it (you do). FCW should still alert on a closing stopped car: don't stage that, just note any FCW."),
           ("op_long", "long_active", "rolling"), "non_adaptive",
           {"hold_seconds": 60.0, "lead_seconds": 5.0},
           "non_adaptive", "set speed held (|mean err| < 0.5, p95 < 1.5 m/s) and no braking for a slower lead",
           BASE_FIELDS + LONG_FIELDS + ("has_lead", "lead_d", "lead_vrel", "fcw"), prereqs=("D1",),
           toggles=NON_ADAPTIVE_ON),
  TestSpec("N2", 4, Kind.FEATURE, "Over set, downhill",
           ("Engaged on a downhill where the car runs above set speed. 20 s.",
            "Records what the radarless brake path actually does (ties to C1)."),
           ("op_long", "long_active", "over_set", "downhill", "no_pedals"), "segments",
           {"seconds": 20.0, "min_segment": 1.0},
           "overset_brake", "", BASE_FIELDS + LONG_FIELDS + ("a_coast", "excess", "brake_assist", "pitch"),
           prereqs=("N1", "C1"), toggles=NON_ADAPTIVE_ON),

  # --- Phase 5: no-service-brake ---------------------------------------------------------------------------------
  TestSpec("NSB1", 5, Kind.FEATURE, "Coast over set speed",
           ("Engaged on a downhill above set speed, 30 s at NoServiceBrakeDecelFloor 0.3,",
            "then (stopped) set the floor to 0.0 and repeat. The command must stay >= -floor and the car off the brakes."),
           ("op_long", "long_active", "over_set", "no_pedals", "clamp"), "segments",
           {"bin": "nsb_floor", "keys": ["0.3", "0.0"], "seconds": 30.0, "min_segment": 1.0},
           "nsb_floor", "command never below -floor and no service-brake decel beyond the coast estimate",
           BASE_FIELDS + LONG_FIELDS + ("a_coast", "excess", "allow_brake", "pitch"), prereqs=("N1", "C1", "C2"),
           toggles=NSB_ON),
  TestSpec("NSB2", 5, Kind.FEATURE, "Overspeed backstop",
           ("Long descent, engaged, NoServiceBrakeOverspeedBackstop ON: let the car run to set + 8 mph.",
            "Braking must resume there - only as strong as C1."),
           ("op_long", "long_active"), "episodes",
           {"detector": "backstop", "count": 1},
           "backstop", "braking resumed within 0.5 m/s of set+margin and reached -0.3 m/s^2 within 3 s",
           BASE_FIELDS + LONG_FIELDS + ("allow_brake", "a_coast", "excess", "pitch"), prereqs=("NSB1", "C1"),
           toggles={**NSB_ON, "NoServiceBrakeOverspeedBackstop": True}),

  # --- Phase 6: low-speed handback -------------------------------------------------------------------------------
  TestSpec("H1", 6, Kind.FEATURE, "Handback feel",
           ("Engaged, slow down below ~5 mph (in 1st/2nd) so openpilot hands back. 5x.",
            "One 'Low Speed: Take Over' alert, no lurch. Tune ManualLowSpeedHandbackSpeed / Ramp."),
           ("manual", "op_long"), "episodes",
           {"detector": "handback", "count": 5},
           "handback_feel", "one alert per handback and peak jerk < 2.5 m/s^3 (provisional)",
           BASE_FIELDS + LONG_FIELDS + ("pitch", "alert"), prereqs=("N1", "G1"), sample_hz=50),
  TestSpec("H2", 6, Kind.FEATURE, "Hold at stop / hill",
           ("After a handback, stop (neutral, own braking), then release the brake.",
            "Auto Brake Hold / hill-hold must hold the car - no rollback. 3 stops, at least 1 on a hill."),
           ("manual",), "episodes",
           {"detector": "stop_hold", "count": 3, "need": {"hill": 1}},
           "stop_hold", "no rollback (> 0.3 m/s with brake and gas released) on any stop",
           BASE_FIELDS + ("pitch", "brake_hold"), prereqs=("H1",)),
  TestSpec("H3", 6, Kind.FEATURE, "Neutral while rolling",
           ("Engaged, flat road, 15+ mph: shift to NEUTRAL, clutch OUT, coast 5 s+. 2x.",
            "Records what GEAR_MT reads and whether the gate coasts the whole time."),
           ("manual", "op_long", "engaged"), "episodes",
           {"detector": "neutral_coast", "count": 2},
           "neutral_coast", "", BASE_FIELDS + LONG_FIELDS + ("gate",), prereqs=("S1", "G1")),
  TestSpec("H4", 6, Kind.FEATURE, "Uphill handback",
           ("Engaged uphill, openpilot on the gas, slowing through the handback speed. 2x.",
            "Measures how fast the car slows after the ramp - be ready with gas/clutch."),
           ("manual", "op_long"), "episodes",
           {"detector": "handback", "count": 2, "filter": "uphill_on_gas"},
           "uphill_handback", "", BASE_FIELDS + LONG_FIELDS + ("pitch",), prereqs=("H1",), sample_hz=50),

  # --- Phase 7: engine-brake model -------------------------------------------------------------------------------
  TestSpec("E0c", 7, Kind.TELEMETRY, "Accel sweep per gear",
           ("Accelerate firmly in each gear 1-6 (clutch fully out), 3 s+ each.",
            "Confirms gear inference (ratios cluster) and settles final drive x tire radius.",
            "HARD PREREQ: if ratios don't cluster, gear inference reads 0 and Task 6 stays inert."),
           ("manual", "rolling", "gas", "clutch_out", "accelerating"), "segments",
           {"bin": "gear_any", "keys": list(GEARS), "seconds": 3.0, "min_segment": 0.5},
           "ratio_check", "every gear's measured ratio within 3% of the seed (inference tolerance is 8%)",
           COAST_FIELDS + ("gas", "pedal_gas"), prereqs=("S1", "S3"), first_run=True),
  TestSpec("E0a", 7, Kind.TELEMETRY, "Downhill coast per gear",
           ("Downhill, openpilot NOT on long: coast in gear (clutch out, off both pedals) in each gear, 6 s each.",
            "Feeds the engine-brake model fit with grade."),
           ("manual", "rolling", "in_gear", "clutch_out", "no_pedals", "long_off", "downhill"), "segments",
           {"bin": "gear", "keys": list(GEARS), "seconds": 6.0, "min_segment": 1.0},
           "coast_fit", "", COAST_FIELDS, prereqs=("E0c",), first_run=True),
  TestSpec("E0b", 7, Kind.TELEMETRY, "Flat coast per gear",
           ("Flat road, openpilot NOT on long: coast in gear in each gear, 10 s each.",
            "Separates engine braking from gravity."),
           ("manual", "rolling", "in_gear", "clutch_out", "no_pedals", "long_off", "flat"), "segments",
           {"bin": "gear", "keys": list(GEARS), "seconds": 10.0, "min_segment": 1.0},
           "coast_fit", "", COAST_FIELDS, prereqs=("E0c",), first_run=True),
  TestSpec("E1", 7, Kind.FEATURE, "EBP gentle trims",
           ("Flat road, no lead, engaged: lower the set speed 1-2 mph. 5x.",
            "No brake application: the car slows on engine braking; no 'capacity' releases."),
           ("manual", "op_long", "long_active", "no_lead", "flat"), "episodes",
           {"detector": "ebp_suppress", "count": 5},
           "ebp_trims", "no capacity release; command held at the coast target; no service-brake decel",
           BASE_FIELDS + LONG_FIELDS + ("a_coast", "excess", "ebp", "pitch"), prereqs=("E0a", "E0b", "E0c", "C2"),
           toggles=EBP_ON),
  TestSpec("E2", 7, Kind.FEATURE, "EBP lead hand-off",
           ("Only if running EBP with lead following. Helper car, WIDE margin: follow it, it eases off, then brakes.",
            "Run once with EngineBrakePreferred ON and once OFF (baseline). Hand-off must be early; gap never worse."),
           ("manual", "op_long", "long_active", "lead"), "episodes",
           {"detector": "lead_approach", "count": 2, "need": {"ebp_on": 1, "ebp_off": 1}},
           "ebp_handoff", "min gap with EBP ON >= OFF - 2 m, and no FCW",
           BASE_FIELDS + LONG_FIELDS + ("has_lead", "lead_d", "lead_vrel", "ebp", "fcw"), prereqs=("E1",),
           optional=True),
  TestSpec("E3", 7, Kind.FEATURE, "EBP downhill",
           ("Engaged on a downhill, 30 s. Suppression should mostly not apply (gravity cancels engine braking).",
            "No runaway."),
           ("manual", "op_long", "long_active", "downhill"), "segments",
           {"seconds": 30.0, "min_segment": 1.0},
           "ebp_downhill", "never more than 3.6 m/s over set speed",
           BASE_FIELDS + LONG_FIELDS + ("ebp", "a_coast", "pitch"), prereqs=("E1",), toggles=EBP_ON),

  # --- Phase 8: gear-aware accel ---------------------------------------------------------------------------------
  TestSpec("GA1", 8, Kind.FEATURE, "Gear-aware feel",
           ("Engaged in 1st-3rd: throttle-on and lift-off (set-speed changes). 20 s per gear.",
            "Tune JERK_SCALE_BY_GEAR by feel; jerk is recorded per gear."),
           ("manual", "op_long", "long_active", "in_gear"), "segments",
           {"bin": "gear", "keys": [1, 2, 3], "seconds": 20.0, "min_segment": 1.0},
           "jerk_by_gear", "", BASE_FIELDS + LONG_FIELDS, prereqs=("E0c",), toggles={"GearAwareAccel": True}),
  TestSpec("A7", 8, Kind.TELEMETRY, "Uphill accel per gear",
           ("Uphill, accelerate in each gear (clutch out), 5 s each. Feeds GA1 and the load behavior.",),
           ("manual", "rolling", "gas", "clutch_out", "uphill"), "segments",
           {"bin": "gear", "keys": list(GEARS), "required": [1, 2, 3, 4], "seconds": 5.0, "min_segment": 1.0},
           "accel_by_gear", "", COAST_FIELDS + ("gas", "pedal_gas"), prereqs=("E0c",)),
)

TESTS_BY_ID: dict[str, TestSpec] = {t.id: t for t in TESTS}
assert len(TESTS_BY_ID) == len(TESTS), "duplicate test id"


def tests_in_phase(phase: int) -> list[TestSpec]:
  return [t for t in TESTS if t.phase == phase]
