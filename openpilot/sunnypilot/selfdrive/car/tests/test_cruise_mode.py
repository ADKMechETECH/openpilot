from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
TransmissionType = car.CarParams.TransmissionType


def _cs(pressed=None):
  CS = car.CarState()
  if pressed is not None:
    CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)]
  return CS


class TestCruiseHelper(OpenpilotTestCase):
  """Re-homed Phase 6 (civic-cruise): a distance-button HOLD toggles NonAdaptiveLongitudinal on a manual, once per
  hold. Composition design, manual-only, no Experimental branch (MVL removed it)."""

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("NonAdaptiveLongitudinal", False)
    self.CP = car.CarParams(openpilotLongitudinalControl=True, transmissionType=TransmissionType.manual)
    self.helper = CruiseHelper(self.CP)

  def _na(self) -> bool:
    return self.params.get_bool("NonAdaptiveLongitudinal")

  def _press_edge(self):
    self.helper.update(_cs(True))   # 0 -> 1

  def _hold_frame(self):
    self.helper.update(_cs())       # count increments

  def _release_edge(self):
    self.helper.update(_cs(False))  # count -> 0

  def _full_hold(self):
    self._press_edge()
    for _ in range(DISTANCE_LONG_PRESS - 1):
      self._hold_frame()

  def test_long_press_toggles_once_per_hold(self):
    assert not self._na()
    self._full_hold()
    assert self._na()               # toggled ON at the long-press threshold
    assert self.helper.toggled
    # keep holding past the threshold -> no second toggle
    for _ in range(DISTANCE_LONG_PRESS):
      self._hold_frame()
    assert self._na()

  def test_release_then_hold_toggles_back(self):
    self._full_hold()
    assert self._na()
    self._release_edge()
    assert self._na()               # the release itself does not toggle
    self._full_hold()
    assert not self._na()           # a fresh hold flips it back OFF

  def test_short_press_does_not_toggle(self):
    self._press_edge()
    for _ in range(DISTANCE_LONG_PRESS - 10):   # stay below the threshold
      self._hold_frame()
    self._release_edge()
    assert not self._na()
    assert not self.helper.toggled

  def test_automatic_untouched(self):
    cp_auto = car.CarParams(openpilotLongitudinalControl=True, transmissionType=TransmissionType.automatic)
    helper = CruiseHelper(cp_auto)
    for i in range(DISTANCE_LONG_PRESS + 5):
      helper.update(_cs(True) if i == 0 else _cs())
    assert not self._na()           # a manual-only feature never touches an automatic

  def test_no_oplong_untouched(self):
    cp = car.CarParams(openpilotLongitudinalControl=False, transmissionType=TransmissionType.manual)
    helper = CruiseHelper(cp)
    for i in range(DISTANCE_LONG_PRESS + 5):
      helper.update(_cs(True) if i == 0 else _cs())
    assert not self._na()           # gated on openpilot owning longitudinal
