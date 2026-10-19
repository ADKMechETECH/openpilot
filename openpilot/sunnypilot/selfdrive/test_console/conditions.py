"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console condition library (vehicle-agnostic): named per-sample predicates over the Sample roles (each shown as a
✓/× line in the UI) and the bin functions the segment analyzer uses. Registry rows refer to these by name; a profile may
add its own (VehicleProfile.conditions). A label may name a role in braces ("{gear_raw} tap live"): it is shown with the
profile's name for that signal.
"""
import math
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import cached_property

from openpilot.sunnypilot.selfdrive.test_console.profile import Powertrain, VehicleProfile
from openpilot.sunnypilot.selfdrive.test_console.signals import Sample


@dataclass
class Context:
  profile: VehicleProfile | None = None
  manual: bool = False
  op_long: bool = False
  toggles: dict[str, bool] = field(default_factory=dict)
  # the fork's feature parameters the plans score against (read by the daemon, 1 Hz)
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
  label_cache: dict[str, str] = field(default_factory=dict, repr=False, compare=False)   # condition name -> its line

  @property
  def powertrain(self) -> Powertrain:
    return self.profile.powertrain if self.profile is not None else Powertrain()

  def label(self, role: str, default: str = "") -> str:
    return self.profile.label(role, default) if self.profile is not None else (default or role)

  def column(self, sample_field: str) -> str:
    return self.profile.column(sample_field) if self.profile is not None else sample_field

  @cached_property
  def conditions(self) -> dict:
    """The core's conditions plus the profile's own (merged once; checked every frame)."""
    return {**CONDITIONS, **(self.profile.conditions if self.profile is not None else {})}

  @property
  def watched_toggles(self) -> tuple[str, ...]:
    return self.profile.watched_toggles if self.profile is not None else ()


FLAT_PITCH = 0.01       # rad (~1 % grade)
HILL_PITCH = 0.02       # rad (~2 % grade)

Cond = Callable[[Sample, Context], bool]


def gear_raw_in_gear(s: Sample, c: Context) -> bool:
  """The raw gear signal reads a forward gear (falls back to the inferred gear when the car has no raw gear signal)."""
  return 1 <= s.gear_raw <= c.powertrain.num_gears if s.gear_raw >= 0 else s.gear >= 1


CONDITIONS: dict[str, tuple[str, Cond]] = {
  "manual": ("manual transmission", lambda s, c: c.manual),
  "op_long": ("op long (alpha) on", lambda s, c: c.op_long),
  "raw_gear_live": ("{gear_raw} tap live", lambda s, c: s.gear_raw >= 0),
  "engaged": ("op engaged", lambda s, c: s.enabled),
  "long_active": ("op long active", lambda s, c: s.long_active),
  "long_off": ("op not on long", lambda s, c: not s.long_active),
  "rolling": ("speed > 3 m/s", lambda s, c: s.v_ego > 3.),
  "in_gear": ("in gear (rpm/speed)", lambda s, c: s.gear >= 1),
  "raw_gear_in_gear": ("{gear_raw} 1-{num_gears}", gear_raw_in_gear),
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


def condition_label(name: str, ctx: Context) -> str:
  """A condition's ✓/× line with the profile's signal names filled in."""
  label = ctx.conditions[name][0]
  if "{" not in label:
    return label
  return label.format(gear_raw=ctx.label("gear_raw", "raw gear"), num_gears=ctx.powertrain.num_gears)



def toggle_label(key: str, want: bool) -> str:
  return f"{key} {'ON' if want else 'OFF'}"


def check_conditions(names: tuple[str, ...], toggles: dict[str, bool], s: Sample, ctx: Context) -> list[tuple[str, bool]]:
  out = [(toggle_label(k, want), ctx.toggles.get(k, False) == want) for k, want in toggles.items()]
  conds, labels = ctx.conditions, ctx.label_cache
  for name in names:
    label = labels.get(name)
    if label is None:
      label = labels[name] = condition_label(name, ctx)
    out.append((label, bool(conds[name][1](s, ctx))))
  return out


# --- bin functions (segment analyzer): key for a qualifying sample, None = doesn't count -------------------------
def _cmd_band(s: Sample, c: Context):
  if abs(s.accel_cmd) < 0.05:
    return "zero"
  if -0.5 <= s.accel_cmd <= -0.15:
    return "small_neg"
  return None


def _gear_any(s: Sample, c: Context):
  # the raw gear signal when the tap is live (that's what the ratio check validates the inference against), else the
  # nearest powertrain gear within 20 % (wider than the inference window, so wrong ratios still bin and show as an error)
  pt = c.powertrain
  if s.gear_raw >= 0:
    return s.gear_raw if 1 <= s.gear_raw <= pt.num_gears else None
  if s.ratio <= 0. or not pt.num_gears:
    return None
  errs = [abs(s.ratio / pt.overall_ratio(g) - 1.) for g in range(1, pt.num_gears + 1)]
  g = min(range(len(errs)), key=errs.__getitem__)
  return g + 1 if errs[g] < 0.2 else None


BINS: dict[str, Callable[[Sample, Context], object]] = {
  "gear": lambda s, c: s.gear if s.gear >= 1 else None,
  "gear_any": _gear_any,
  "cmd_band": _cmd_band,
  "nsb_floor": lambda s, c: f"{c.nsb_floor:.1f}",
}
