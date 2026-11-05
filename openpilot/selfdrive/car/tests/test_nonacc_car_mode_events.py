"""
fork-only (civic-cruise, session 12): the car's own ACC <-> Cruise mode vs. openpilot's wrongCruiseMode refusal.

On a manual with openpilot longitudinal the car's conventional-cruise mode (cruiseState.nonAdaptive) is the fork's
non-ACC mode, so it must not raise wrongCruiseMode (NO_ENTRY + USER_DISABLE) - that silently refused every engage with
non-ACC armed on 2026-11-05. Every other configuration keeps the stock refusal.
"""
from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.selfdrived.events import ET, Events

EventName = log.OnroadEvent.EventName


def _events(manual, op_long, non_adaptive, button_enable=True):
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
  CP.transmissionType = "manual" if manual else "automatic"
  CP.openpilotLongitudinalControl = op_long
  CP.pcmCruise = not op_long
  cs = car.CarState.new_message()
  cs.cruiseState.nonAdaptive = non_adaptive
  cs.cruiseState.available = True
  cs.buttonEnable = button_enable
  cs.gearShifter = 'drive'
  cs.vEgo = 15.0
  events = Events()
  events.add_from_msg(CarEvents(CP).update(cs.as_reader(), cs.as_reader(), car.CarControl.new_message().as_reader()).to_msg())
  return events


def test_manual_op_long_engages_in_the_cars_cruise_mode():
  ev = _events(True, True, True)
  assert EventName.wrongCruiseMode not in ev.names
  assert ev.contains(ET.ENABLE) and not ev.contains(ET.NO_ENTRY) and not ev.contains(ET.USER_DISABLE)


def test_manual_op_long_acc_mode_unchanged():
  ev = _events(True, True, False)
  assert ev.contains(ET.ENABLE) and not ev.contains(ET.NO_ENTRY)


def test_stock_refusal_kept_everywhere_else():
  for manual, op_long in ((True, False), (False, True), (False, False)):
    ev = _events(manual, op_long, True)
    assert EventName.wrongCruiseMode in ev.names, (manual, op_long)
    assert ev.contains(ET.NO_ENTRY) and ev.contains(ET.USER_DISABLE)
