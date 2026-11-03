"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName

DISTANCE_LONG_PRESS = 50


class CruiseHelper:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()
    # fork-only (civic-cruise): on a manual the distance-button hold toggles non-ACC mode, not Experimental
    self.manual = CP.transmissionType == structs.CarParams.TransmissionType.manual

    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0}
    self._gap_prev_count = 0
    self._experimental_mode = False
    self.experimental_mode_switched = False   # also the long-press latch (reset on release in selfdrived)

  def update(self, CS, events, experimental_mode) -> None:
    if self.CP.openpilotLongitudinalControl:
      if CS.cruiseState.available:
        self.update_button_frame_counts(CS)

        # toggle a mode once on a distance-button hold (non-ACC on a manual, else Experimental)
        self.update_mode_toggle(events, experimental_mode)

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_mode_toggle(self, events, experimental_mode) -> None:
    # Clear the one-toggle-per-hold latch on a genuine new press (rising edge: previous count 0 -> now held). The latch
    # is otherwise only cleared on a gap-button RELEASE event; if that release is never processed (observed on-car
    # 2026-11-03: a toggle immediately followed by an onroad->offroad transition dropped the release), the latch stays
    # stuck True and eats the next long-press. Resetting at press-start makes every hold self-contained so a lost
    # release can't eat it. Uses the 0->nonzero edge (not count == 1) so a held button that re-sends a pressed event
    # every frame - as the unit test models a hold - does not keep re-clearing the latch mid-hold.
    count = self.button_frame_counts[ButtonType.gapAdjustCruise]
    if self._gap_prev_count == 0 and count > 0:
      self.experimental_mode_switched = False
    self._gap_prev_count = count
    if count >= DISTANCE_LONG_PRESS and not self.experimental_mode_switched:
      if self.manual:
        # fork-only (civic-cruise): long-press flips non-ACC ("set-speed only") mode, live (the mode is read live, so
        # the flip takes effect engaged or not). The persistent "NON-ACC" HUD overlay confirms it - the old transient
        # ET.WARNING alert never showed. Experimental is NOT toggled on a manual (its e2e brakes to stops / launches,
        # unusable on a manual, and was the earlier "~20 mph" surprise).
        self.params.put_bool("NonAdaptiveLongitudinal", not self.params.get_bool("NonAdaptiveLongitudinal"), block=True)
      else:
        self._experimental_mode = not experimental_mode
        self.params.put_bool("ExperimentalMode", self._experimental_mode)
        events.add(EventNameSP.experimentalModeSwitched)
      self.experimental_mode_switched = True
