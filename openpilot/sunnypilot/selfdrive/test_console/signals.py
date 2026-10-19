"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console signal layer (vehicle-agnostic): one flat Sample per carState frame (100 Hz), built from the published
messages, the vehicle profile's signal map + raw-CAN tap (profile.py: the named roles) and the feature modules' cloudlog
events. Read-only.
"""
import json
import math
from dataclasses import dataclass, field, fields

from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL

NAN = float("nan")
RPM_FILTER_RC = 0.05          # s: rpm_rate is filtered like the fork's clutch-gate flare detector (pinned by a test)
V_CRUISE_UNSET_KPH = 250.     # carState.vCruise is km/h; 255 = unset

# cloudlog events the console follows (emitted by the fork's feature modules; see CLAUDE.md)
WATCHED_EVENTS = ("clutch_shift_gate", "engine_brake_preferred", "no_service_brake_backstop", "no_service_brake_pulse",
                  "manual_low_speed_handback", "engine_brake_coast")
EBP_CAPACITY_WARNING = "engine_brake_preferred: car not slowing"
HANDBACK_ALERT = "manualLowSpeedHandback"
OVERSPEED_ALERT = "noServiceBrakeOverspeed"   # Task 11 pulse-mode escalation ("Overspeed: Take Over")
# selfdriveState alert types that mean something went wrong with longitudinal/CAN during a capture
FAULT_ALERTS = ("accFaulted", "controlsMismatch", "canError", "commIssue", "cruiseMismatch", "brakeUnavailable",
                "steerTempUnavailable", "controlsdLagging", "pandaSafetyMismatch")


@dataclass(slots=True)
class Sample:
  t: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  v_cruise: float = NAN          # m/s, NaN when unset
  gas: bool = False
  brake: bool = False
  clutch: bool = False           # clutch open (profile signal map; role clutch_coupled inverted)
  rpm: float = 0.
  rpm_rate: float = 0.           # rpm/s, filtered like the clutch gate's flare detector
  gear_raw: int = -1             # the car's own manual-gear signal (profile raw tap), -1 when unavailable
  gear: int = 0                  # gear inferred from rpm / wheel speed (profile powertrain), 0 = none
  ratio: float = 0.              # measured engine/wheel speed ratio (profile powertrain)
  pitch: float = NAN             # rad, + = uphill (carControl.orientationNED[1])
  reverse: bool = False
  standstill: bool = False
  enabled: bool = False
  long_active: bool = False
  accel_cmd: float = 0.          # carControl.actuators.accel
  accel_out: float = 0.          # carOutput.actuatorsOutput.accel (after the car controller)
  accel_sent: float = NAN        # the longitudinal command as sent on the bus (profile raw tap), NaN when unavailable
  brake_assist: int = -1         # the brake-request bit as sent (profile raw tap), -1 when unavailable
  a_target: float = 0.
  allow_brake: bool = True
  plan_valid: bool = False
  fcw: bool = False
  has_lead: bool = False
  lead_d: float = NAN
  lead_vrel: float = NAN
  acc_faulted: bool = False
  cruise_on: bool = False
  stock_aeb: bool = False
  stock_fcw: bool = False
  brake_hold: bool = False
  gas_raw: float = NAN           # raw gas pedal position (profile raw tap)
  raw_payload: int = -1          # one raw CAN payload as an int (profile raw tap; the bit hunt)
  a_coast: float = NAN           # the profile's coast model for this gear/rpm/speed/pitch (may be an uncalibrated seed)
  excess: float = NAN            # a_ego - a_coast: well below 0 => something besides coasting is slowing the car
  gate: str = ""                 # clutch-gate state: controlsd's published one (Task 12), else the last cloudlog one
  gate_paused: bool = False      # controlsd published: the gate is capping the throttle for a shift (Task 12 A2)
  urgency: str = ""              # controlsd published: the Rule-5 braking-urgency reason ("" = none)
  collision: bool = False        # plannerd published: non-adaptive is braking for a collision threat (Task 12 A1)
  handback_active: bool = False  # controlsd published: the low-speed handback ramp is running
  latched: dict = field(default_factory=dict)   # Param -> the value the feature actually latched (published, Task 12 B3)
  ebp: bool = False              # EngineBrakePreferred suppressing (last cloudlog state)
  alert: str = ""                # selfdriveState.alertType
  tx_blocked: int = 0            # pandaStates[0].safetyTxBlocked (cumulative)
  events: tuple = ()             # cloudlog events received this frame: ((name, payload dict), ...)


SAMPLE_FIELDS = tuple(f.name for f in fields(Sample))


def sample_row(s: Sample, names: tuple[str, ...]) -> list:
  row = []
  for n in names:
    v = getattr(s, n)
    if isinstance(v, float):
      v = None if not math.isfinite(v) else round(v, 4)
    elif isinstance(v, bool):
      v = int(v)
    row.append(v)
  return row


class LogEventTap:
  """Parses logMessage strings for the feature modules' cloudlog events (cheap substring test before json)."""

  def __init__(self):
    self.gate = ""
    self.ebp = False

  @staticmethod
  def parse(record: str) -> tuple[str, dict] | None:
    if EBP_CAPACITY_WARNING in record:
      return "ebp_capacity", {}
    if not any(name in record for name in WATCHED_EVENTS):
      return None
    try:
      msg = json.loads(record).get("msg")
    except (ValueError, AttributeError):
      return None
    if isinstance(msg, dict) and msg.get("event") in WATCHED_EVENTS:
      return msg["event"], msg
    return None

  def update(self, records) -> tuple:
    out = []
    for record in records:
      ev = self.parse(record)
      if ev is None:
        continue
      name, payload = ev
      if name == "clutch_shift_gate":
        self.gate = str(payload.get("state", ""))
      elif name == "engine_brake_preferred":
        self.ebp = bool(payload.get("active", False))
      out.append(ev)
    return tuple(out)


@dataclass
class SampleBuilder:
  """Builds a Sample from the latest messages through the vehicle profile. Holds the little state some derived signals
  need."""
  profile: object                                      # profile.VehicleProfile
  rpm_filter: FirstOrderFilter = field(default_factory=lambda: FirstOrderFilter(0., RPM_FILTER_RC, DT_CTRL))
  rpm_prev: float = 0.
  rpm_seeded: bool = False

  def build(self, t: float, CS, CC, CO, plan, plan_valid: bool, radar, ss, tx_blocked: int, tap,
            log: LogEventTap, events: tuple, cc_sp=None, cc_sp_ok: bool = False, plan_sp=None,
            plan_sp_ok: bool = False) -> Sample:
    sig, pt = self.profile.signals, self.profile.powertrain
    s = Sample(t=t)
    s.v_ego, s.a_ego = float(CS.vEgo), float(CS.aEgo)
    s.v_cruise = float(CS.vCruise) * CV.KPH_TO_MS if 0. < CS.vCruise < V_CRUISE_UNSET_KPH else NAN
    s.gas, s.brake = bool(CS.gasPressed), bool(CS.brakePressed)
    s.clutch = bool(sig.clutch(CS))
    s.rpm = float(sig.engine_rpm(CS))
    s.reverse = CS.gearShifter == structs.CarState.GearShifter.reverse
    s.standstill = bool(CS.standstill) or s.v_ego < 0.1
    s.acc_faulted, s.cruise_on = bool(CS.accFaulted), bool(CS.cruiseState.enabled)
    s.stock_aeb, s.stock_fcw, s.brake_hold = bool(CS.stockAeb), bool(CS.stockFcw), bool(CS.brakeHoldActive)

    if not self.rpm_seeded:
      self.rpm_filter.x = self.rpm_prev = s.rpm
      self.rpm_seeded = True
    rpm_f = self.rpm_filter.update(s.rpm)
    s.rpm_rate = (rpm_f - self.rpm_prev) / DT_CTRL
    self.rpm_prev = rpm_f

    s.gear = pt.infer_gear(s.rpm, s.v_ego, s.clutch)
    s.ratio = pt.measured_ratio(s.rpm, s.v_ego)
    orientation = CC.orientationNED
    s.pitch = float(orientation[1]) if len(orientation) == 3 else NAN
    s.enabled, s.long_active = bool(CC.enabled), bool(CC.longActive)
    s.accel_cmd = float(CC.actuators.accel)
    s.accel_out = float(CO.actuatorsOutput.accel)

    s.plan_valid = plan_valid
    s.a_target, s.allow_brake, s.fcw = float(plan.aTarget), bool(plan.allowBrake), bool(plan.fcw)
    lead = radar.leadOne
    s.has_lead = bool(lead.present)
    if s.has_lead:
      s.lead_d, s.lead_vrel = float(lead.dRel), float(lead.vRel)
    s.alert = str(ss.alertType).split("/")[0]
    s.tx_blocked = int(tx_blocked)

    if tap is not None and tap.available:
      tap.fill(s)

    grade = s.pitch if math.isfinite(s.pitch) else 0.
    s.a_coast = pt.coast_decel(s.gear, s.rpm, s.v_ego, grade) if math.isfinite(s.pitch) else NAN
    s.excess = s.a_ego - s.a_coast
    s.gate, s.ebp, s.events = log.gate, log.ebp, events
    published_state(s, cc_sp, cc_sp_ok, plan_sp, plan_sp_ok)
    return s


def published_state(s: Sample, cc_sp, cc_sp_ok: bool, plan_sp, plan_sp_ok: bool) -> None:
  """Task 12 B3/D2: what the features ACTUALLY latched and are doing, as controlsd / plannerd publish it. Only from
  fresh, valid messages; otherwise the sample keeps the Param / cloudlog view (and conditions fall back to Params)."""
  if cc_sp is not None and cc_sp_ok:
    ml = cc_sp.manualLongitudinal
    s.gate = str(ml.gateState) if ml.gateEnabled else ""
    s.gate_paused, s.urgency, s.handback_active = bool(ml.gatePaused), str(ml.urgency), bool(ml.handbackActive)
    s.ebp = bool(ml.ebpSuppressing)
    s.latched.update({"ClutchShiftGate": bool(ml.gateEnabled), "ClutchReengageSmart": bool(ml.reengageSmart),
                      "EngineBrakePreferred": bool(ml.ebpEnabled)})
  if plan_sp is not None and plan_sp_ok:
    f, nsb = plan_sp.features, plan_sp.noServiceBrake
    s.collision = bool(f.collisionBraking)
    s.latched.update({"NonAdaptiveLongitudinal": bool(f.nonAdaptive), "GearAwareAccel": bool(f.gearAware),
                      "NoServiceBrakeHold": bool(nsb.enabled), "NoServiceBrakeOverspeedBackstop": bool(nsb.backstop),
                      "NoServiceBrakeOverspeedMode": str(nsb.mode), "NoServiceBrakeDecelFloor": float(nsb.decelFloor),
                      "NoServiceBrakeOverspeedMargin": float(nsb.margin)})
