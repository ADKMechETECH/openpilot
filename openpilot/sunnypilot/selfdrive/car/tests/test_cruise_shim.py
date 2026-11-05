"""
fork-only (civic-cruise): the distance-button long-press shim. On a manual openpilot does nothing on the hold (the car
switches its own ACC <-> Cruise mode, which is the fork's non-ACC mode); on an automatic it keeps toggling
ExperimentalMode. The latch (experimental_mode_switched) gates one toggle per hold (reset on release in selfdrived).
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


def test_manual_longpress_changes_nothing_in_openpilot():
  # the hold is the car's own ACC <-> Cruise switch (= the fork's non-ACC mode, read from carState); openpilot must not
  # keep a second toggle on the same gesture, and must not toggle Experimental on a manual
  h, CP = _helper(CAR.HONDA_CIVIC)
  assert CP.transmissionType == TT.manual
  Params().put_bool("ExperimentalMode", False, block=True)
  puts = []
  real = h.params
  class Spy:
    def __getattr__(self, name):
      if name.startswith("put") or name == "remove":
        puts.append(name)
      return getattr(real, name)
  h.params = Spy()
  _hold_toggle(h)
  assert puts == []                                                # no param write at all (none on the driving path)
  assert Params().get_bool("ExperimentalMode") is False
  assert h.experimental_mode_switched                              # the one-per-hold latch still behaves


def test_automatic_longpress_toggles_experimental():
  h, CP = _helper(CAR.HONDA_ACCORD)
  if CP.transmissionType == TT.manual:
    return
  Params().put_bool("ExperimentalMode", False, block=True)
  _hold_toggle(h)
  assert Params().get_bool("ExperimentalMode") is True             # automatic: Experimental toggles (stock)
