"""
fork-only (civic-cruise) Phase 6: the distance-button long-press shim. On a manual it flips NonAdaptiveLongitudinal
(live); on an automatic it keeps toggling ExperimentalMode. The latch (experimental_mode_switched) gates one toggle per
hold (reset on release in selfdrived).
"""
from openpilot.common.params import Params
from opendbc.car.structs import car
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS

ButtonType = car.CarState.ButtonEvent.Type
TT = car.CarParams.TransmissionType


class _Events:
  def add(self, _):  # SP events sink
    pass


def _helper(fingerprint):
  CP = CarInterface.get_non_essential_params(fingerprint)
  return CruiseHelper(CP), CP


def _hold_toggle(h):
  h.button_frame_counts[ButtonType.gapAdjustCruise] = DISTANCE_LONG_PRESS
  h.update_mode_toggle(_Events(), experimental_mode=False)


def test_manual_longpress_flips_nonadaptive_live():
  h, CP = _helper(CAR.HONDA_CIVIC)
  assert CP.transmissionType == TT.manual
  Params().put_bool("NonAdaptiveLongitudinal", False, block=True)
  Params().put_bool("ExperimentalMode", False, block=True)
  _hold_toggle(h)
  assert Params().get_bool("NonAdaptiveLongitudinal") is True      # flipped on
  assert Params().get_bool("ExperimentalMode") is False            # Experimental NOT touched on a manual
  assert h.experimental_mode_switched                              # one toggle per hold
  # simulate the release reset (selfdrived), hold again -> flips back off
  h.experimental_mode_switched = False
  _hold_toggle(h)
  assert Params().get_bool("NonAdaptiveLongitudinal") is False


def test_automatic_longpress_toggles_experimental():
  h, CP = _helper(CAR.HONDA_ACCORD)
  if CP.transmissionType == TT.manual:
    return
  Params().put_bool("NonAdaptiveLongitudinal", False, block=True)
  Params().put_bool("ExperimentalMode", False, block=True)
  _hold_toggle(h)
  assert Params().get_bool("ExperimentalMode") is True             # automatic: Experimental toggles (stock)
  assert Params().get_bool("NonAdaptiveLongitudinal") is False     # non-ACC not touched
