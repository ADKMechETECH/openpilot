from openpilot.cereal import custom, log
from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP

EventName = log.OnroadEvent.EventName
TransmissionType = car.CarParams.TransmissionType
GateState = custom.CarControlSP.ManualLongitudinal.GateState


def run(transmission, clutch: bool, brake: bool = False, paused: bool | None = True, fresh: bool = True,
        gate_state=GateState.shifting) -> Events:
  """selfdrived acts on what controlsd published (carControlSP.manualLongitudinal); paused=None: never received."""
  CP = car.CarParams.new_message()
  CP.transmissionType = transmission
  ev = CarSpecificEventsSP(CP, structs.CarParamsSP())
  CS = car.CarState.new_message()
  CS.gasPressed = True
  CS.brakePressed = brake
  CS.deprecated.clutchPressed = clutch
  ml = None
  if paused is not None:
    ml = custom.CarControlSP.new_message().manualLongitudinal
    ml.gateEnabled, ml.gatePaused, ml.gateState = True, paused, gate_state
  events = Events()
  events.add(EventName.gasPressedOverride)
  if brake:
    events.add(EventName.pedalPressed)
  ev.update(CS, events, True, ml, fresh)
  return events


class TestClutchGateEvents(OpenpilotTestCase):
  def test_blip_with_clutch_open_is_not_an_override(self):
    assert not run(TransmissionType.manual, clutch=True).has(EventName.gasPressedOverride)

  def test_heel_toe_keeps_brake_event(self):
    # the gas blip is dropped, but the brake must still disengage exactly as stock
    events = run(TransmissionType.manual, clutch=True, brake=True)
    assert not events.has(EventName.gasPressedOverride)
    assert events.has(EventName.pedalPressed)

  def test_gas_with_clutch_engaged_is_still_an_override(self):
    assert run(TransmissionType.manual, clutch=False).has(EventName.gasPressedOverride)

  def test_gate_not_paused_unchanged(self):
    # gate off / unavailable (automatics) / idle: controlsd publishes gatePaused False -> stock override
    for transmission in (TransmissionType.manual, TransmissionType.automatic, TransmissionType.cvt):
      assert run(transmission, clutch=True, paused=False, gate_state=GateState.idle).has(EventName.gasPressedOverride)

  def test_coast_timeout_blip_is_an_override(self):
    # Task 12 A2.6: after the shift timeout the gate is no longer paused (openpilot can brake again), so a blip there
    # is a real accel override - the panda blocks a nonzero command with gas pressed and stock must handle it
    assert run(TransmissionType.manual, clutch=True, paused=False,
               gate_state=GateState.coastTimeout).has(EventName.gasPressedOverride)

  def test_stale_or_missing_state_fails_to_stock(self):
    assert run(TransmissionType.manual, clutch=True, fresh=False).has(EventName.gasPressedOverride)
    assert run(TransmissionType.manual, clutch=True, paused=None).has(EventName.gasPressedOverride)
