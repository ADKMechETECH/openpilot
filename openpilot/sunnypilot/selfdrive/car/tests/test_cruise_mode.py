from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.parameterized import parameterized_class
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName


@parameterized_class(('openpilot_longitudinal',), [(True,)])
class TestCruiseHelper(OpenpilotTestCase):
  def setup_method(self):
    self.CP = car.CarParams(openpilotLongitudinalControl=self.openpilot_longitudinal)
    self.cruise_helper = CruiseHelper(self.CP)
    self.cruise_helper.experimental_mode_switched = False
    self.events = Events()

  def reset(self):
    for _ in range(2):
      CS = car.CarState(cruiseState={"available": False})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=False)]
      self.cruise_helper._experimental_mode = False
      self.cruise_helper.experimental_mode_switched = False
      self.cruise_helper.update(CS, self.events, False)


  def test_gap_adjust_cruise_long_press_toggle_mode(self) -> None:
    for pressed in (True, False):
      for experimental_mode in (True, False):
        self.reset()
        self.cruise_helper._experimental_mode = experimental_mode
        toggled_mode = not experimental_mode if pressed else experimental_mode

        for i in range(DISTANCE_LONG_PRESS):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, experimental_mode)

        # mode should be toggled
        assert self.cruise_helper._experimental_mode == toggled_mode
        assert self.cruise_helper.experimental_mode_switched is pressed

        # keep holding button after switching mode
        for _ in range(DISTANCE_LONG_PRESS):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)]
          self.cruise_helper.update(CS, self.events, toggled_mode)

        # mode should not be toggled
        assert self.cruise_helper._experimental_mode == toggled_mode
        assert self.cruise_helper.experimental_mode_switched is pressed

  def test_gap_adjust_cruise_short_press_toggle_mode(self) -> None:
    for pressed in (True, False):
      for experimental_mode in (True, False):
        self.reset()
        self.cruise_helper._experimental_mode = experimental_mode

        for i in range(DISTANCE_LONG_PRESS - 1):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, experimental_mode)

        # mode should not be toggled
        assert self.cruise_helper._experimental_mode == experimental_mode
        assert self.cruise_helper.experimental_mode_switched is False


# civic-cruise session 10: on a manual with openpilot longitudinal the distance-button hold toggles NonAdaptiveLongitudinal


class TestGapHoldNonAdaptive(OpenpilotTestCase):
  def _helper(self, transmission=car.CarParams.TransmissionType.manual, gap_hold=None, non_adaptive=False):
    params = Params()
    params.remove("GapHoldNonAdaptive")
    if gap_hold is not None:
      params.put_bool("GapHoldNonAdaptive", gap_hold, block=True)
    params.put_bool("NonAdaptiveLongitudinal", non_adaptive, block=True)
    params.put_bool("ExperimentalMode", False, block=True)
    CP = car.CarParams(openpilotLongitudinalControl=True, transmissionType=transmission)
    return CruiseHelper(CP), params

  @staticmethod
  def _hold(helper, frames, enabled=False, available=True):
    events = EventsSP()
    for i in range(frames):
      CS = car.CarState(cruiseState={"available": available})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=True)] if i == 0 else []
      helper.update(CS, events, False, enabled)
    return events

  @staticmethod
  def _release(helper):
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=False)]
    helper.update(CS, EventsSP(), False, False)
    helper.experimental_mode_switched = False   # selfdrived clears it on the release (after its personality check)

  def test_hold_toggles_non_adaptive_not_experimental(self):
    helper, params = self._helper()
    self.assertTrue(helper.gap_hold_non_adaptive)   # default ON for manuals
    events = self._hold(helper, DISTANCE_LONG_PRESS)
    self.assertTrue(params.get_bool("NonAdaptiveLongitudinal"))
    self.assertTrue(events.has(EventNameSP.nonAdaptiveOn))
    self.assertFalse(params.get_bool("ExperimentalMode"))
    self.assertTrue(helper.experimental_mode_switched, "the hold is used: no personality step on release")
    self._release(helper)
    events = self._hold(helper, 3 * DISTANCE_LONG_PRESS)   # a long hold toggles once
    self.assertFalse(params.get_bool("NonAdaptiveLongitudinal"))
    self.assertTrue(events.has(EventNameSP.nonAdaptiveOff))

  def test_engaged_hold_changes_nothing(self):
    helper, params = self._helper()
    events = self._hold(helper, DISTANCE_LONG_PRESS, enabled=True)
    self.assertFalse(params.get_bool("NonAdaptiveLongitudinal"))
    self.assertTrue(events.has(EventNameSP.nonAdaptiveDisengageFirst))
    self.assertTrue(helper.experimental_mode_switched, "still no personality step on release")

  def test_main_cruise_off_hold_still_counts(self):
    # session 10: a hold with main cruise off used to fall through to a personality step on release
    helper, params = self._helper()
    self._hold(helper, DISTANCE_LONG_PRESS, available=False)
    self.assertTrue(params.get_bool("NonAdaptiveLongitudinal"))
    self.assertTrue(helper.experimental_mode_switched)

  def test_short_press_is_a_personality_step(self):
    helper, params = self._helper()
    events = self._hold(helper, DISTANCE_LONG_PRESS - 1)
    self.assertFalse(params.get_bool("NonAdaptiveLongitudinal"))
    self.assertFalse(helper.experimental_mode_switched)
    self.assertFalse(events.has(EventNameSP.nonAdaptiveOn))

  def test_opt_out_and_automatics_are_stock(self):
    for helper, _ in (self._helper(gap_hold=False), self._helper(transmission=car.CarParams.TransmissionType.automatic)):
      self.assertFalse(helper.gap_hold_non_adaptive)
      events = self._hold(helper, DISTANCE_LONG_PRESS)
      self.assertTrue(helper._experimental_mode, "stock: the hold toggles Experimental mode")
      self.assertTrue(events.has(EventNameSP.experimentalModeSwitched))
