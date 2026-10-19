"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Civic 6MT raw-CAN tap (Honda Bosch radarless only): the profile's source for the raw roles — GEAR_MT (carState only
carries GEAR_MT == 0, as clutchPressed) -> gear_raw, PEDAL_GAS -> gas_raw, the POWERTRAIN_DATA (0x17C) payload ->
raw_payload (S6), and ACC_CONTROL as actually sent on sendcan -> accel_sent / brake_assist (COMPUTER_BRAKE_ASSIST).
Never sends anything. Fails soft: any problem -> the raw roles stay unavailable (-1 / NaN) and the tests that need them
show 'GEAR_MT tap live' as not met.
"""
from opendbc.car import Bus, structs
from openpilot.common.swaglog import cloudlog

NAN = float("nan")


class RawCanTap:
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

  def fill(self, s) -> None:
    """The raw roles of a Sample (signals.Sample)."""
    s.gear_raw, s.gas_raw, s.raw_payload = self.gear_mt, self.pedal_gas, self.raw_17c
    s.accel_sent, s.brake_assist = self.accel_sent, self.brake_assist
