from openpilot.cereal import log
from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP

EventName = log.OnroadEvent.EventName
TransmissionType = car.CarParams.TransmissionType


def run(transmission, gate_on: bool, clutch: bool, brake: bool = False) -> Events:
  Params().put_bool("ClutchShiftGate", gate_on)
  CP = car.CarParams.new_message()
  CP.transmissionType = transmission
  ev = CarSpecificEventsSP(CP, structs.CarParamsSP())
  CS = car.CarState.new_message()
  CS.gasPressed = True
  CS.brakePressed = brake
  CS.deprecated.clutchPressed = clutch
  events = Events()
  events.add(EventName.gasPressedOverride)
  if brake:
    events.add(EventName.pedalPressed)
  ev.update(CS, events)
  return events


class TestClutchGateEvents(OpenpilotTestCase):
  def test_blip_with_clutch_open_is_not_an_override(self):
    assert not run(TransmissionType.manual, gate_on=True, clutch=True).has(EventName.gasPressedOverride)

  def test_heel_toe_keeps_brake_event(self):
    # the gas blip is dropped, but the brake must still disengage exactly as stock
    events = run(TransmissionType.manual, gate_on=True, clutch=True, brake=True)
    assert not events.has(EventName.gasPressedOverride)
    assert events.has(EventName.pedalPressed)

  def test_gas_with_clutch_engaged_is_still_an_override(self):
    assert run(TransmissionType.manual, gate_on=True, clutch=False).has(EventName.gasPressedOverride)

  def test_gate_off_unchanged(self):
    assert run(TransmissionType.manual, gate_on=False, clutch=True).has(EventName.gasPressedOverride)

  def test_automatics_unchanged(self):
    for transmission in (TransmissionType.automatic, TransmissionType.cvt):
      assert run(transmission, gate_on=True, clutch=True).has(EventName.gasPressedOverride)
