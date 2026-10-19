"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console profile: 2024 Honda Civic Sport Hatchback 6MT (HONDA_CIVIC_2022, Bosch radarless). Everything the console
knows about this car lives here (package): the signal map and raw-CAN tap (tap.py), the powertrain (the same gear
inference code the Task 6/8 features run), the start button, the test plan (registry.py) and its pass rules (results.py),
and the feature Params the session UI may set.
"""
from openpilot.sunnypilot.selfdrive.controls.lib import engine_brake_model as ebm
from openpilot.sunnypilot.selfdrive.test_console.profile import ButtonMap, Powertrain, SignalMap, VehicleProfile
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.registry import REGISTRY
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.results import RESULTS
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt.tap import RawCanTap

FINGERPRINTS = ("HONDA_CIVIC_2022",)


def applies(CP) -> bool:
  # the 6MT plan: a CIVIC_2022 with a CVT gets no profile (console inactive)
  return CP.carFingerprint in FINGERPRINTS and CP.transmissionType == "manual"


def _clutch(CS) -> bool:
  return bool(CS.deprecated.clutchPressed)    # Phase 1: GEAR_MT == 0 (clutch pedal down, or neutral)


def _rpm(CS) -> float:
  return float(CS.deprecated.engineRpm)        # Phase 1: ENGINE_DATA.ENGINE_RPM


SIGNALS = SignalMap(
  clutch=_clutch,
  engine_rpm=_rpm,
  raw_tap=RawCanTap,
  gear_raw_reverse=14,
  labels={"gear_raw": "GEAR_MT", "raw_payload": "0x17C", "gas_raw": "PEDAL_GAS", "accel_command_sent": "ACCEL_COMMAND",
          "brake_request_sent": "COMPUTER_BRAKE_ASSIST"},
  # capture files and markers keep the column names the Civic captures always had (old captures stay re-scorable)
  columns={"gear_raw": "gear_mt", "raw_payload": "raw_17c", "gas_raw": "pedal_gas"},
  sources={
    "vehicle_speed": "carState.vEgo",
    "gas": "carState.gasPressed (POWERTRAIN_DATA.PEDAL_GAS)",
    "brake": "carState.brakePressed",
    "clutch_coupled": "carState.deprecated.clutchPressed = GEARBOX_ALT_2.GEAR_MT == 0 (0x1DD), inverted; also the ratio lock",
    "engine_rpm": "carState.deprecated.engineRpm (ENGINE_DATA.ENGINE_RPM)",
    "gear": "inferred: rpm / wheel speed vs the seed ratios (engine_brake_model.infer_gear, +-8 %)",
    "gear_raw": "GEARBOX_ALT_2.GEAR_MT (0x1DD, bus 0): 0 clutch/neutral, 1-6, 14 reverse (raw tap)",
    "gas_raw": "POWERTRAIN_DATA.PEDAL_GAS (0x17C, raw tap)",
    "raw_payload": "POWERTRAIN_DATA (0x17C) payload, for the clutch-switch bit hunt (raw tap)",
    "accel_command_sent": "ACC_CONTROL.ACCEL_COMMAND (0x1C8) as sent on sendcan (raw tap)",
    "brake_request_sent": "ACC_CONTROL.COMPUTER_BRAKE_ASSIST as sent (raw tap; the PR #2050 bit)",
    "cruise_buttons": "carState.buttonEvents from SCM_BUTTONS (CRUISE_BUTTONS, CRUISE_SETTING)",
    "start_button": "gapAdjustCruise (the distance button)",
  },
)

POWERTRAIN = Powertrain(
  gear_ratios=ebm.GEAR_RATIOS, final_drive=ebm.FINAL_DRIVE, tire_radius=ebm.WHEEL_RADIUS, match_tol=ebm.GEAR_MATCH_TOL,
  min_speed=ebm.INFER_MIN_SPEED, min_rpm=ebm.INFER_MIN_RPM,
  # the console validates the exact code the features run (Task 6 EBP / gear-aware accel, Task 8 ratio lock)
  infer_fn=ebm.infer_gear, measured_ratio_fn=ebm.measured_ratio, overall_ratio_fn=ebm.overall_ratio,
  coast_fn=ebm.estimate_coast_decel,
)

# TODO(on-car): confirm in cabana that SCM_BUTTONS.CRUISE_SETTING == 3 (DISTANCE) decodes on this car, i.e. the console
#  shows "button seen" after a press.
BUTTONS = ButtonMap(
  default="gapAdjustCruise",
  # every button the Honda carstate emits (TestConsoleStartButton may pick any; each keeps its native function)
  allowed=("gapAdjustCruise", "accelCruise", "decelCruise", "cancel", "mainCruise", "lkas"),
  short={"gapAdjustCruise": "gap"},
  note="distance button: a short press also steps the following-distance personality on release; a >= 0.5 s hold " +
       "toggles Experimental mode (sunnypilot's distance-button long press)",
)

# Feature Params the session UI may set on this car (Task 9 Part C + Task 11). Notably NOT AlphaLongitudinalEnabled.
ALLOWLIST = ("NonAdaptiveLongitudinal", "NoServiceBrakeHold", "NoServiceBrakeDecelFloor", "NoServiceBrakeOverspeedBackstop",
             "NoServiceBrakeOverspeedMargin", "NoServiceBrakeOverspeedMode", "ClutchShiftGate", "ClutchShiftGateGasReleaseHint",
             "ClutchReengageSmart", "EngineBrakePreferred", "GearAwareAccel", "DisengageOnAccelerator")

# All Params a registry row may check. Read-only (1 Hz); the daemon never writes any of them.
WATCHED_TOGGLES = ("NonAdaptiveLongitudinal", "ClutchShiftGate", "ClutchShiftGateGasReleaseHint", "DisengageOnAccelerator",
                   "NoServiceBrakeHold", "NoServiceBrakeOverspeedBackstop", "EngineBrakePreferred", "GearAwareAccel")

PROFILE = VehicleProfile(
  name="civic_6mt",
  title="Civic 6MT",
  applies=applies,
  fingerprints=FINGERPRINTS,
  signals=SIGNALS,
  powertrain=POWERTRAIN,
  buttons=BUTTONS,
  registry=REGISTRY,
  allowlist=ALLOWLIST,
  watched_toggles=WATCHED_TOGGLES,
  results=RESULTS,
)
