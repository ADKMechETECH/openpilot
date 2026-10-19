"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console vehicle profiles: the seam between the vehicle-agnostic core and one car (DEV-TEST BUILD ONLY).

The core (capture state machine, detectors/analyzers, the registry/session engine, the UI framework, file IPC, the
start button's gestures, the non-actuating guarantee) reads vehicle signals only through the named roles below. A
VehicleProfile supplies everything car-specific:
  signals     how this car provides each role (carState readers, a raw-CAN tap, display names, log column names)
  powertrain  gear ratios, final drive, tire radius, inference tolerance (and optionally the car's own inference code)
  buttons     which steering-wheel button + press mode starts/stops a capture (default + allowed)
  registry    the test plan: tests (conditions over the roles), phases, sessions, required toggles; results = pass rules
  allowlist   the feature Params the session UI may set for this car / fork
  applies     which fingerprint(s) / platform it serves
select_profile(CP) picks the first registered profile whose applies(CP) is true; none -> the console is inactive (does
nothing, writes nothing but an "inactive" status line). Adding a car = writing a profile + listing it in
profiles.PROFILES. TEST_CONSOLE_PROFILE=<name> forces one (replays / bench only).
"""
import math
import os
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Protocol

PROFILE_ENV = "TEST_CONSOLE_PROFILE"

# Named roles -> the Sample field that carries each (signals.Sample). The core uses the fields; a profile maps its car's
# raw signals onto them (SignalMap) and names them for people (labels) and log files (columns).
ROLES: Mapping[str, str] = MappingProxyType({
  "vehicle_speed": "v_ego",
  "gas": "gas",
  "brake": "brake",
  "clutch_coupled": "clutch",           # Sample.clutch = clutch OPEN (not coupled)
  "engine_rpm": "rpm",
  "gear": "gear",                       # inferred from rpm / speed through the profile's powertrain (0 = none)
  "gear_raw": "gear_raw",               # the car's own manual-gear signal, -1 = unavailable
  "gas_raw": "gas_raw",                 # raw gas pedal position, NaN = unavailable
  "raw_payload": "raw_payload",         # one raw CAN payload the bit hunt scans, -1 = unavailable
  "accel_command_sent": "accel_sent",   # the longitudinal command as actually sent on the bus, NaN = unavailable
  "brake_request_sent": "brake_assist", # the brake-request bit as sent, -1 = unavailable
  "cruise_buttons": "",                 # carState.buttonEvents (read by the start button)
  "start_button": "",                   # ButtonMap.default
})


class RawTap(Protocol):
  """A car's raw-CAN reader (never sends). available=False -> the raw roles stay unavailable."""
  available: bool

  def update(self, can_list, sendcan_list) -> None: ...

  def fill(self, sample) -> None: ...


def _no_clutch(CS) -> bool:
  return False


def _no_rpm(CS) -> float:
  return 0.


def _no_tap(CP) -> RawTap | None:
  return None


@dataclass(frozen=True)
class SignalMap:
  clutch: Callable[[Any], bool] = _no_clutch          # carState -> clutch open (role clutch_coupled, inverted)
  engine_rpm: Callable[[Any], float] = _no_rpm        # carState -> rpm
  raw_tap: Callable[[Any], RawTap | None] = _no_tap   # CarParams -> the raw-CAN tap for gear_raw/gas_raw/raw_payload/...
  gear_raw_reverse: int | None = None                 # gear_raw value that means reverse
  labels: Mapping[str, str] = field(default_factory=dict)   # role -> what people call it on this car
  columns: Mapping[str, str] = field(default_factory=dict)  # Sample field -> its name in capture files / markers
  sources: Mapping[str, str] = field(default_factory=dict)  # role -> where it comes from on this car (docs)


@dataclass(frozen=True)
class Powertrain:
  """Gear inference. Generic ratio math from the numbers; a profile may point the console at the exact functions its
  features run instead (infer_fn etc.), so the console validates the code that drives."""
  gear_ratios: tuple[float, ...] = ()
  final_drive: float = 1.
  tire_radius: float = 0.3
  match_tol: float = 0.08
  min_speed: float = 3.0
  min_rpm: float = 500.
  infer_fn: Callable[..., int] | None = None
  measured_ratio_fn: Callable[[float, float], float] | None = None
  overall_ratio_fn: Callable[[int], float] | None = None
  coast_fn: Callable[[int, float, float, float], float] | None = None   # (gear, rpm, v, grade) -> m/s^2 coast model

  @property
  def num_gears(self) -> int:
    return len(self.gear_ratios)

  def overall_ratio(self, gear: int) -> float:
    if self.overall_ratio_fn is not None:
      return self.overall_ratio_fn(gear)
    return self.gear_ratios[gear - 1] * self.final_drive if 1 <= gear <= self.num_gears else 0.

  def measured_ratio(self, rpm: float, v_ego: float) -> float:
    if self.measured_ratio_fn is not None:
      return self.measured_ratio_fn(rpm, v_ego)
    return 0. if v_ego < 0.1 else rpm * 2. * math.pi / 60. * self.tire_radius / v_ego

  def infer_gear(self, rpm: float, v_ego: float, clutch_pressed: bool = False) -> int:
    if self.infer_fn is not None:
      return self.infer_fn(rpm, v_ego, clutch_pressed)
    if clutch_pressed or not self.num_gears or not (math.isfinite(rpm) and math.isfinite(v_ego)) or \
       v_ego < self.min_speed or rpm < self.min_rpm:
      return 0
    ratio = self.measured_ratio(rpm, v_ego)
    errors = [abs(ratio / self.overall_ratio(g) - 1.) for g in range(1, self.num_gears + 1)]
    best = min(range(len(errors)), key=errors.__getitem__)
    return best + 1 if errors[best] <= self.match_tol else 0

  def coast_decel(self, gear: int, rpm: float, v_ego: float, grade: float) -> float:
    return self.coast_fn(gear, rpm, v_ego, grade) if self.coast_fn is not None else math.nan


@dataclass(frozen=True)
class ButtonMap:
  default: str                                   # carState.ButtonEvent.Type name that starts/stops a capture
  allowed: tuple[str, ...] = ()                  # names TestConsoleStartButton may select (empty = default only)
  short: Mapping[str, str] = field(default_factory=dict)   # UI label per name
  note: str = ""                                 # its native function / side effects on this car
  modes: Mapping[str, tuple[str, ...]] = field(default_factory=dict)   # name -> allowed press modes, recommended first
  side_effect: Mapping[str, str] = field(default_factory=dict)         # name -> one short line for the chip (D-10)

  def resolve(self, name: str | None) -> str:
    """The button actually used for a TestConsoleStartButton value ("" / unknown / not allowed -> the default)."""
    return name if name and (name == self.default or name in self.allowed) else self.default

  def label(self, name: str | None) -> str:
    name = self.resolve(name)
    return self.short.get(name, name)

  def resolve_mode(self, name: str | None, mode: str | None) -> str:
    """The press mode actually used: an allowed one for this button (its recommended one otherwise)."""
    allowed = self.modes.get(self.resolve(name))
    if not allowed:
      return mode if mode in ("short", "triple", "long") else "short"
    return mode if mode in allowed else allowed[0]


@dataclass(frozen=True)
class VehicleProfile:
  name: str                                      # short id, e.g. for TEST_CONSOLE_PROFILE
  title: str                                     # what the UI shows
  applies: Callable[[Any], bool]                 # CarParams -> does this profile serve this car
  fingerprints: tuple[str, ...]                  # documentation of applies()
  signals: SignalMap
  powertrain: Powertrain
  buttons: ButtonMap
  registry: Any                                  # registry.Registry
  allowlist: tuple[str, ...]                     # feature Params the session UI may set (plus the console's own keys)
  watched_toggles: tuple[str, ...]               # Params the registry checks (read-only, 1 Hz)
  conditions: Mapping[str, tuple] = field(default_factory=dict)   # extra named per-sample predicates
  results: Mapping[str, Callable] = field(default_factory=dict)   # result rules by name (the plan's pass criteria)

  def label(self, role: str, default: str = "") -> str:
    return self.signals.labels.get(role, default or role)

  def column(self, sample_field: str) -> str:
    return self.signals.columns.get(sample_field, sample_field)


def select_profile(CP, profiles: Iterable[VehicleProfile] | None = None) -> VehicleProfile | None:
  """The profile serving this car, or None (console inactive). TEST_CONSOLE_PROFILE forces one by name."""
  if profiles is None:
    from openpilot.sunnypilot.selfdrive.test_console.profiles import PROFILES
    profiles = PROFILES
  profiles = tuple(profiles)
  forced = os.environ.get(PROFILE_ENV)
  if forced:
    return next((p for p in profiles if p.name == forced), None)
  if CP is None:
    return None
  for p in profiles:
    try:
      if p.applies(CP):
        return p
    except Exception:
      continue
  return None


def inactive_reason(CP) -> str:
  fp = getattr(CP, "carFingerprint", "") if CP is not None else ""
  return f"no test-console profile for this platform ({fp})" if fp else "no test-console profile: car not identified yet"
