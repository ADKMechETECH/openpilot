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

    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0}
    self._experimental_mode = False
    self.experimental_mode_switched = False

    # civic-cruise session 10 (owner, 2026-10-21): on a manual with openpilot longitudinal, the distance-button hold
    # toggles NonAdaptiveLongitudinal instead of Experimental mode (e2e longitudinal brakes to stops and launches, which a
    # manual can't use; the planner ignores e2e in non-adaptive anyway). GapHoldNonAdaptive (default ON) restores stock.
    self.gap_hold_non_adaptive = (bool(CP.openpilotLongitudinalControl) and
                                  CP.transmissionType == structs.CarParams.TransmissionType.manual and
                                  bool(self.params.get("GapHoldNonAdaptive", return_default=True)))

  def update(self, CS, events, experimental_mode, enabled: bool = False) -> None:
    if self.CP.openpilotLongitudinalControl:
      if self.gap_hold_non_adaptive:
        # counted with main cruise off too: NonAdaptive doesn't need it, and a hold must never step the personality
        self.update_button_frame_counts(CS)
        self.update_non_adaptive(events, enabled)
      elif CS.cruiseState.available:
        self.update_button_frame_counts(CS)

        # toggle experimental mode once on distance button hold
        self.update_experimental_mode(events, experimental_mode)

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_non_adaptive(self, events, enabled: bool) -> None:
    """Once per hold (>= DISTANCE_LONG_PRESS frames, 0.5 s): flip NonAdaptiveLongitudinal - only while disengaged, where
    the planner latches it (within ~1 s); engaged, it only says so. experimental_mode_switched marks the hold as used,
    so selfdrived doesn't also step the personality on the release."""
    if self.button_frame_counts[ButtonType.gapAdjustCruise] >= DISTANCE_LONG_PRESS and not self.experimental_mode_switched:
      self.experimental_mode_switched = True
      if enabled:
        events.add(EventNameSP.nonAdaptiveDisengageFirst)
        return
      non_adaptive = not self.params.get_bool("NonAdaptiveLongitudinal")
      self.params.put_bool("NonAdaptiveLongitudinal", non_adaptive, block=True)   # once per hold, disengaged only
      events.add(EventNameSP.nonAdaptiveOn if non_adaptive else EventNameSP.nonAdaptiveOff)

  def update_experimental_mode(self, events, experimental_mode) -> None:
    if self.button_frame_counts[ButtonType.gapAdjustCruise] >= DISTANCE_LONG_PRESS and not self.experimental_mode_switched:
      self._experimental_mode = not experimental_mode
      self.params.put_bool("ExperimentalMode", self._experimental_mode)
      events.add(EventNameSP.experimentalModeSwitched)
      self.experimental_mode_switched = True
