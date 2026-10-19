"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console signal layer: one flat Sample per carState frame (100 Hz), built from the published messages, an
optional raw CAN tap (Honda Bosch radarless only) and the feature modules' cloudlog events. Read-only.
"""
import json
import math
from dataclasses import dataclass, field, fields

from opendbc.car import Bus, structs
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import RPM_FILTER_RC
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_model import estimate_coast_decel, infer_gear, measured_ratio

NAN = float("nan")
V_CRUISE_UNSET_KPH = 250.     # carState.vCruise is km/h; 255 = unset

# cloudlog events the console follows (emitted by the feature modules; see CLAUDE.md)
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
  clutch: bool = False           # carState clutchPressed (Honda: GEAR_MT == 0)
  rpm: float = 0.
  rpm_rate: float = 0.           # rpm/s, filtered like the clutch gate's flare detector
  gear_mt: int = -1              # raw GEARBOX_ALT_2.GEAR_MT, -1 when the tap isn't available
  gear: int = 0                  # gear inferred from rpm / wheel speed (engine_brake_model.infer_gear), 0 = none
  ratio: float = 0.              # measured engine/wheel ratio x wheel radius (engine_brake_model.measured_ratio)
  pitch: float = NAN             # rad, + = uphill (carControl.orientationNED[1])
  reverse: bool = False
  standstill: bool = False
  enabled: bool = False
  long_active: bool = False
  accel_cmd: float = 0.          # carControl.actuators.accel
  accel_out: float = 0.          # carOutput.actuatorsOutput.accel (after the car controller)
  accel_sent: float = NAN        # ACC_CONTROL.ACCEL_COMMAND as sent (sendcan tap), NaN when unavailable
  brake_assist: int = -1         # ACC_CONTROL.COMPUTER_BRAKE_ASSIST as sent (PR #2050 bit), -1 when unavailable
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
  pedal_gas: float = NAN         # raw POWERTRAIN_DATA.PEDAL_GAS (tap)
  raw_17c: int = -1              # raw POWERTRAIN_DATA payload as an int (tap; S6 bit hunt)
  a_coast: float = NAN           # engine_brake_model coast estimate for this gear/rpm/speed/pitch (UNCALIBRATED seed)
  excess: float = NAN            # a_ego - a_coast: well below 0 => something besides coasting is slowing the car
  gate: str = ""                 # last clutch_shift_gate state seen in cloudlog
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


class RawCanTap:
  """
  Honda Bosch radarless only: raw GEAR_MT (carState only carries GEAR_MT == 0), PEDAL_GAS, the POWERTRAIN_DATA
  payload (S6), and ACC_CONTROL as actually sent on sendcan. Never sends anything. Fails soft: any problem -> the raw
  fields stay unavailable (-1 / NaN) and the tests that need them show 'GEAR_MT tap live' as not met.
  """

  def __init__(self, CP: structs.CarParams):
    self.available = False
    self.gear_mt = -1
    self.pedal_gas = NAN
    self.raw_17c = -1
    self.accel_sent = NAN
    self.brake_assist = -1
    try:
      from opendbc.can import CANParser
      from opendbc.car.honda.hondacan import CanBus
      from opendbc.car.honda.values import DBC, HondaFlags
      if CP.brand != "honda" or not (CP.flags & HondaFlags.BOSCH_RADARLESS):
        return
      dbc = DBC[CP.carFingerprint][Bus.pt]
      self.bus = CanBus(CP).pt
      self.cp = CANParser(dbc, [("GEARBOX_ALT_2", float("nan")), ("POWERTRAIN_DATA", float("nan"))], self.bus)
      self.cp_send = CANParser(dbc, [("ACC_CONTROL", float("nan"))], self.bus)
      self.available = True
    except Exception:
      cloudlog.exception("test_console: raw CAN tap unavailable")

  def update(self, can_list, sendcan_list) -> None:
    if not self.available:
      return
    if can_list:
      updated = self.cp.update(can_list)
      if updated:
        vl = self.cp.vl
        if self.cp.ts_nanos["GEARBOX_ALT_2"]["GEAR_MT"]:
          self.gear_mt = int(vl["GEARBOX_ALT_2"]["GEAR_MT"])
        if self.cp.ts_nanos["POWERTRAIN_DATA"]["PEDAL_GAS"]:
          self.pedal_gas = float(vl["POWERTRAIN_DATA"]["PEDAL_GAS"])
      for _, frames in can_list:
        for address, dat, src in frames:
          if address == 0x17C and src == self.bus:
            self.raw_17c = int.from_bytes(dat, "big")
    if sendcan_list:
      self.cp_send.update(sendcan_list, sendcan=True)
      if self.cp_send.ts_nanos["ACC_CONTROL"]["ACCEL_COMMAND"]:
        self.accel_sent = float(self.cp_send.vl["ACC_CONTROL"]["ACCEL_COMMAND"])
        self.brake_assist = int(self.cp_send.vl["ACC_CONTROL"]["COMPUTER_BRAKE_ASSIST"])


@dataclass
class SampleBuilder:
  """Builds a Sample from the latest messages. Holds the little state some derived signals need."""
  rpm_filter: FirstOrderFilter = field(default_factory=lambda: FirstOrderFilter(0., RPM_FILTER_RC, DT_CTRL))
  rpm_prev: float = 0.
  rpm_seeded: bool = False

  def build(self, t: float, CS, CC, CO, plan, plan_valid: bool, radar, ss, tx_blocked: int, tap: RawCanTap | None,
            log: LogEventTap, events: tuple) -> Sample:
    s = Sample(t=t)
    s.v_ego, s.a_ego = float(CS.vEgo), float(CS.aEgo)
    s.v_cruise = float(CS.vCruise) * CV.KPH_TO_MS if 0. < CS.vCruise < V_CRUISE_UNSET_KPH else NAN
    s.gas, s.brake = bool(CS.gasPressed), bool(CS.brakePressed)
    s.clutch = bool(CS.deprecated.clutchPressed)
    s.rpm = float(CS.deprecated.engineRpm)
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

    s.gear = infer_gear(s.rpm, s.v_ego, s.clutch)
    s.ratio = measured_ratio(s.rpm, s.v_ego)
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
      s.gear_mt, s.pedal_gas, s.raw_17c = tap.gear_mt, tap.pedal_gas, tap.raw_17c
      s.accel_sent, s.brake_assist = tap.accel_sent, tap.brake_assist

    grade = s.pitch if math.isfinite(s.pitch) else 0.
    s.a_coast = estimate_coast_decel(s.gear, s.rpm, s.v_ego, grade) if math.isfinite(s.pitch) else NAN
    s.excess = s.a_ego - s.a_coast
    s.gate, s.ebp, s.events = log.gate, log.ebp, events
    return s
