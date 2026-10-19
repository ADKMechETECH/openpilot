"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Steering-wheel start/stop button for the test console (vehicle-agnostic gestures). Reads carState.buttonEvents only —
it never suppresses or replaces the button's native function (the vehicle profile's ButtonMap.note says what that is).

Params (dev-test build): TestConsoleStartButton = a CarState.ButtonEvent.Type name ("" = the profile's default; a name
the profile doesn't allow -> its default), TestConsoleStartPress = "short" (default) | "long" | "triple".
  short   fire on release of a press shorter than LONG_PRESS.
  triple  three short presses within TRIPLE_WINDOW (a button whose press steps a setting comes back where it started
          after three steps — e.g. a three-way following-distance button).
  long    fire once a hold reaches LONG_PRESS (on some cars a long hold has its own native function — check the note).
"""
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.test_console.profile import ButtonMap

ButtonType = structs.CarState.ButtonEvent.Type

PRESS_MODES = ("short", "triple", "long")
LONG_PRESS = 0.5       # s, matches sunnypilot's DISTANCE_LONG_PRESS (50 frames at 100 Hz)
TRIPLE_WINDOW = 1.5    # s, first press to third release


def _raw(enum_value) -> int:
  return int(getattr(enum_value, "raw", enum_value))  # capnp enums carry the int in .raw


def button_type(name: str | None, buttons: ButtonMap) -> int:
  try:
    return _raw(getattr(ButtonType, buttons.resolve(name)))
  except (AttributeError, TypeError):
    return _raw(getattr(ButtonType, buttons.default))


class StartButton:
  def __init__(self, name: str | None = None, mode: str | None = None, *, buttons: ButtonMap):
    self.buttons = buttons
    self.configure(name, mode)
    self.seen = 0          # press events seen this drive (on-car decode check)

  def configure(self, name: str | None, mode: str | None) -> None:
    # Task 12 D-15: store (and show) the RESOLVED button - an unknown / disallowed name is the profile default - and a
    # press mode this button allows (D-10)
    self.requested = (name or "", mode or "")
    self.name = self.buttons.resolve(name)
    self.type = button_type(self.name, self.buttons)
    self.mode = self.buttons.resolve_mode(self.name, mode)
    self.pressed = False
    self.t_down = 0.
    self.long_fired = False
    self.taps: list[float] = []

  def differs(self, name: str | None, mode: str | None) -> bool:
    """The Params ask for something other than what this button is configured as (compared after resolving)."""
    return (name or "", mode or "") != self.requested

  def update(self, t: float, button_events) -> bool:
    """Returns True on the frame the configured gesture completes."""
    fired = False
    for be in button_events:
      if _raw(be.type) != self.type:
        continue
      if be.pressed and not self.pressed:
        self.pressed, self.t_down, self.long_fired = True, t, False
        self.seen += 1
      elif not be.pressed and self.pressed:
        self.pressed = False
        short = t - self.t_down < LONG_PRESS
        if self.mode == "short" and short:
          fired = True
        elif self.mode == "triple" and short:
          self.taps = [x for x in self.taps if t - x <= TRIPLE_WINDOW] + [self.t_down]
          if len(self.taps) >= 3:
            fired, self.taps = True, []
    if self.mode == "long" and self.pressed and not self.long_fired and t - self.t_down >= LONG_PRESS:
      self.long_fired = fired = True
    return fired
