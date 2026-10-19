"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log, custom
from opendbc.car import structs

from opendbc.car.chrysler.values import RAM_DT
from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.controls.lib.manual_low_speed_handback import DEFAULT_HANDBACK_SPEED, ENGAGE_HYSTERESIS, \
                                                                                handback_available
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarSpecificEventsSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params | None = None):
    self.CP = CP
    self.CP_SP = CP_SP

    self.low_speed_alert = False

    self.params = params or Params()
    self.handback_available = handback_available(CP)

  def update(self, CS: structs.CarState, events: Events, enabled: bool = False, ml=None, ml_fresh: bool = False):
    """ml: carControlSP.manualLongitudinal as controlsd published it; ml_fresh: received within the last few frames.
    Task 12: selfdrived acts on controlsd's published state only (one source of truth) and fails to stock without it."""
    events_sp = EventsSP()
    fresh = ml is not None and ml_fresh

    # Clutch/shift gate: during a shift the throttle only revs a disconnected engine (rev-match / heel-toe blip), so it
    # is not a driver accel override - but ONLY while controlsd says the gate is actually paused (A2.6). Then controlsd
    # holds the command at exactly 0 while the gas is pressed (the panda allows nothing else). Brake events are untouched.
    if fresh and ml.gatePaused and CS.deprecated.clutchPressed and events.has(EventName.gasPressedOverride):
      events.remove(EventName.gasPressedOverride)

    # Manual low-speed handback (controlsd owns it): the prompt from the ramp start, the stock speedTooLow disengage when
    # controlsd says so, and no engaging below the latched handback speed (no brake-to-stop, no hold, no launch).
    if self.handback_available:
      if fresh and ml.handbackDisengage:
        events.add(EventName.speedTooLow)
      elif fresh and ml.handbackActive:
        events_sp.add(EventNameSP.manualLowSpeedHandback)
      speed = ml.handbackSpeed if fresh and ml.handbackSpeed > 0. else DEFAULT_HANDBACK_SPEED
      if CS.vEgo < speed + ENGAGE_HYSTERESIS:
        events.add(EventName.belowEngageSpeed)
        events_sp.add(EventNameSP.manualLowSpeedNoEntry)

    if self.CP.brand == 'chrysler':
      if self.CP.carFingerprint in RAM_DT:
        # remove belowSteerSpeed event from CarSpecificEvents as RAM_DT uses a different logic
        if events.has(EventName.belowSteerSpeed):
          events.remove(EventName.belowSteerSpeed)

        # TODO-SP: use if/elif to have the gear shifter condition takes precedence over the speed condition
        # TODO-SP: add 1 m/s hysteresis
        if CS.vEgo >= self.CP.minEnableSpeed:
          self.low_speed_alert = False
        if self.CP.minEnableSpeed >= 14.5 and CS.gearShifter != GearShifter.drive:
          self.low_speed_alert = True
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.brand == 'toyota':
      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed and self.CP_SP.enableGasInterceptor:
          if events.has(EventName.resumeRequired):
            events.remove(EventName.resumeRequired)

    return events_sp
