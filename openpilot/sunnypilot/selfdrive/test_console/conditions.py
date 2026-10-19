"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console condition library: named per-sample predicates (each shown as a ✓/× line in the UI) and the bin
functions the segment analyzer uses. Registry rows refer to these by name.
"""
import math
from collections.abc import Callable
from dataclasses import dataclass, field

from openpilot.sunnypilot.selfdrive.test_console.signals import Sample

# All Params a registry row may check. Read-only (1 Hz); the console never writes any of them.
WATCHED_TOGGLES = ("NonAdaptiveLongitudinal", "ClutchShiftGate", "ClutchShiftGateGasReleaseHint", "DisengageOnAccelerator",
                   "NoServiceBrakeHold", "NoServiceBrakeOverspeedBackstop", "EngineBrakePreferred", "GearAwareAccel")


@dataclass
class Context:
  manual: bool = False
  op_long: bool = False
  toggles: dict[str, bool] = field(default_factory=dict)
  nsb_floor: float = 0.3
  nsb_margin: float = 3.6
  # NoServiceBrakeOverspeedMode and the pulse band, as the feature clamps them (no_service_brake.read_pulse_params)
  nsb_mode: str = "maintain"
  nsb_trigger: float = 3.6
  nsb_release: float = 0.9
  nsb_pulse_decel: float = -1.0
  nsb_pulse_jerk: float = 1.5
  nsb_ceiling: float = 6.7
  handback_ramp: float = 0.5
  ebp_coast_target: float = 0.0


FLAT_PITCH = 0.01       # rad (~1 % grade)
HILL_PITCH = 0.02       # rad (~2 % grade)

Cond = Callable[[Sample, Context], bool]

CONDITIONS: dict[str, tuple[str, Cond]] = {
  "manual": ("manual transmission", lambda s, c: c.manual),
  "op_long": ("op long (alpha) on", lambda s, c: c.op_long),
  "raw_can": ("GEAR_MT tap live", lambda s, c: s.gear_mt >= 0),
  "engaged": ("op engaged", lambda s, c: s.enabled),
  "long_active": ("op long active", lambda s, c: s.long_active),
  "long_off": ("op not on long", lambda s, c: not s.long_active),
  "rolling": ("speed > 3 m/s", lambda s, c: s.v_ego > 3.),
  "in_gear": ("in gear (rpm/speed)", lambda s, c: s.gear >= 1),
  "gear_mt_in_gear": ("GEAR_MT 1-6", lambda s, c: 1 <= s.gear_mt <= 6 if s.gear_mt >= 0 else s.gear >= 1),
  "clutch_out": ("clutch engaged", lambda s, c: not s.clutch),
  "no_pedals": ("off gas & brake", lambda s, c: not s.gas and not s.brake),
  "gas": ("on the gas", lambda s, c: s.gas),
  "accelerating": ("accelerating", lambda s, c: s.a_ego > 0.2),
  "steady": ("steady speed", lambda s, c: abs(s.a_ego) < 0.3),
  "flat": ("flat road", lambda s, c: math.isfinite(s.pitch) and abs(s.pitch) < FLAT_PITCH),
  "downhill": ("downhill", lambda s, c: math.isfinite(s.pitch) and s.pitch < -HILL_PITCH),
  "uphill": ("uphill", lambda s, c: math.isfinite(s.pitch) and s.pitch > HILL_PITCH),
  "no_lead": ("no lead", lambda s, c: not s.has_lead),
  "lead": ("lead car", lambda s, c: s.has_lead),
  "cmd_brake": ("cmd <= -0.5", lambda s, c: s.accel_cmd <= -0.5),
  "over_set": ("above set speed", lambda s, c: math.isfinite(s.v_cruise) and s.v_ego > s.v_cruise + 0.3),
  "clamp": ("no-brake clamp on", lambda s, c: s.plan_valid and not s.allow_brake),
}


def toggle_label(key: str, want: bool) -> str:
  return f"{key} {'ON' if want else 'OFF'}"


def check_conditions(names: tuple[str, ...], toggles: dict[str, bool], s: Sample, ctx: Context) -> list[tuple[str, bool]]:
  out = [(toggle_label(k, want), ctx.toggles.get(k, False) == want) for k, want in toggles.items()]
  for name in names:
    label, fn = CONDITIONS[name]
    out.append((label, bool(fn(s, ctx))))
  return out


# --- bin functions (segment analyzer): key for a qualifying sample, None = doesn't count -------------------------
def _cmd_band(s: Sample, c: Context):
  if abs(s.accel_cmd) < 0.05:
    return "zero"
  if -0.5 <= s.accel_cmd <= -0.15:
    return "small_neg"
  return None


def _gear_any(s: Sample, c: Context):
  # raw GEAR_MT when the tap is live (that's what E0c validates the inference against), else the nearest seed gear
  # within 20 % (wider than the 8 % inference window, so wrong seed ratios still bin and show up as an error)
  if s.gear_mt >= 0:
    return s.gear_mt if 1 <= s.gear_mt <= 6 else None
  from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import GEAR_RATIOS, overall_ratio
  if s.ratio <= 0.:
    return None
  errs = [abs(s.ratio / overall_ratio(g) - 1.) for g in range(1, len(GEAR_RATIOS) + 1)]
  g = min(range(len(errs)), key=errs.__getitem__)
  return g + 1 if errs[g] < 0.2 else None


BINS: dict[str, Callable[[Sample, Context], object]] = {
  "gear": lambda s, c: s.gear if s.gear >= 1 else None,
  "gear_any": _gear_any,
  "cmd_band": _cmd_band,
  "nsb_floor": lambda s, c: f"{c.nsb_floor:.1f}",
}
