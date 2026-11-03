"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type

DISTANCE_LONG_PRESS = 50


class CruiseHelper:
  """fork-only (civic-cruise): on a manual with openpilot longitudinal, a distance-button HOLD toggles non-ACC
  ("set-speed only") mode.

  Composition, not inheritance (the earlier civic-nonacc wired this into SelfdriveD by subclassing; MVL's new base
  deleted that cruise_helpers.py and the experimental_mode_switched mechanism it rode on). selfdrived owns an instance,
  calls update(CS) each frame, and skips its own short-press personality step while .toggled is set.

  Manual-only: MVL's base dropped the long-press -> Experimental behavior (DEC sunset), and Experimental's e2e brakes to
  stops / launches, so it is unusable on a manual anyway (the earlier "~20 mph" surprise). No alert/event is emitted -
  the persistent NON-ACC HUD overlay is the confirmation (the old transient ET.WARNING alert never showed).
  """

  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()
    self.manual = CP.transmissionType == structs.CarParams.TransmissionType.manual
    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0}
    self._gap_prev_count = 0
    self.toggled = False  # one-toggle-per-hold latch; also tells selfdrived to skip the personality step on this release

  def update(self, CS) -> None:
    # Gated only on openpilot owning longitudinal, NOT on cruiseState.available (owner, 2026-11-03): the hold must flip
    # non-ACC whenever op-long control exists, so the mode can be armed with main cruise still off. Automatics and
    # stock-ACC cars are untouched (MVL owns their gap-button behavior).
    if not (self.CP.openpilotLongitudinalControl and self.manual):
      return

    self.update_button_frame_counts(CS)

    # Clear the latch on a genuine new press (rising edge: previous count 0 -> now held), so a dropped gap-button RELEASE
    # event can't leave the latch stuck True and eat the next long-press (observed on-car 2026-11-03: a toggle followed
    # by an onroad->offroad transition dropped the release). Uses the 0->nonzero edge (not count == 1) so a held button
    # that re-sends a pressed event every frame does not keep re-clearing the latch mid-hold.
    count = self.button_frame_counts[ButtonType.gapAdjustCruise]
    if self._gap_prev_count == 0 and count > 0:
      self.toggled = False
    self._gap_prev_count = count

    if count >= DISTANCE_LONG_PRESS and not self.toggled:
      # Live flip (the planner reads NonAdaptiveLongitudinal live, so it takes effect engaged or not). block=True keeps
      # the flip immediate; this fires only on a deliberate hold, not continuously, so it is not a driving-path burst.
      self.params.put_bool("NonAdaptiveLongitudinal", not self.params.get_bool("NonAdaptiveLongitudinal"), block=True)
      self.toggled = True

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)
