"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Steering-wheel start/stop button for the test console. Reads carState.buttonEvents only — it never suppresses or
replaces the button's native function (on a sunnypilot Honda with openpilot long: a gap/distance press steps the
following-distance personality on release; a >= 0.5 s hold toggles Experimental mode).

Params (dev-test build): TestConsoleStartButton = a CarState.ButtonEvent.Type name (default "gapAdjustCruise"),
TestConsoleStartPress = "short" (default) | "long" | "triple".
  short   fire on release of a press shorter than LONG_PRESS. Side effect: one personality step per press (only
          matters with lead following, i.e. adaptive mode — non-adaptive ignores the lead MPC).
  triple  three short presses within TRIPLE_WINDOW. Three personality steps = back where it started, so following
          distance is net-unchanged (use this for adaptive-mode tests such as E2).
  long    fire once a hold reaches LONG_PRESS. On this car with openpilot long that hold ALSO toggles Experimental
          mode (sunnypilot's distance-button long press) — not recommended.
TODO(on-car): confirm in cabana that SCM_BUTTONS.CRUISE_SETTING == 3 (DISTANCE) decodes on this car, i.e. the
 console shows "button seen" after a press.
"""
from opendbc.car import structs

ButtonType = structs.CarState.ButtonEvent.Type

DEFAULT_BUTTON = "gapAdjustCruise"
PRESS_MODES = ("short", "triple", "long")
LONG_PRESS = 0.5       # s, matches sunnypilot's DISTANCE_LONG_PRESS (50 frames at 100 Hz)
TRIPLE_WINDOW = 1.5    # s, first press to third release


def _raw(enum_value) -> int:
  return int(getattr(enum_value, "raw", enum_value))  # capnp enums carry the int in .raw


def button_type(name: str | None) -> int:
  try:
    return _raw(getattr(ButtonType, name or DEFAULT_BUTTON))
  except (AttributeError, TypeError):
    return _raw(getattr(ButtonType, DEFAULT_BUTTON))


class StartButton:
  def __init__(self, name: str | None = None, mode: str | None = None):
    self.configure(name, mode)
    self.seen = 0          # press events seen this drive (on-car decode check)

  def configure(self, name: str | None, mode: str | None) -> None:
    self.name = name if name else DEFAULT_BUTTON
    self.type = button_type(self.name)
    self.mode = mode if mode in PRESS_MODES else "short"
    self.pressed = False
    self.t_down = 0.
    self.long_fired = False
    self.taps: list[float] = []

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
