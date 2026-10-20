"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Manual-transmission low-speed handback: below ManualLowSpeedHandbackSpeed, openpilot longitudinal hands control back
to the driver, in both adaptive and non-adaptive modes. openpilot can't work the clutch, and on a manual the driver
takes the stop themselves (downshift -> neutral -> own braking), so openpilot never brakes to a stop, never holds the
car at a standstill and never launches it.

ASSUMPTION: holding at a stop and anti-rollback are covered by the driver and the car's own Auto Brake Hold
(driver-selectable) and automatic hill-hold. Nothing here provides a standstill hold.

Sequence, once engaged and below the handback speed (latched until disengaged):
  1. controlsd ramps its THROTTLE side from where the command was to 0 over ManualLowSpeedHandbackRamp seconds, and its
     braking side too while nothing is urgent (a shrinking envelope around the start value: no step, no PID windup);
     the driver's prompt, manualLowSpeedHandback ("Low Speed: Take Over", below FCW in priority), shows from here,
  2. then pauses longitudinal (longActive=False: no gas, no brake, no standstill/stop request),
  3. selfdrived disengages DISENGAGE_MARGIN later with the stock speedTooLow event ("openpilot Canceled").
Engaging below the handback speed (+ hysteresis) is refused (stock belowEngageSpeed, with manualLowSpeedNoEntry as the
text), so openpilot can't be engaged at a standstill and pull away.

Emergency braking is never handed back (Task 12 A3, Rule 5), but only a real emergency counts (Task 13 A1): the
handback uses braking_urgency.handback_urgency(), not the broad reason set. FCW, stock AEB, a hard (< -1.5 m/s^2) plan
request and invalid/stale inputs pass through; the plan's "stopping" request and the broad lead reasons do not — an
ordinary approach to slow or stopped traffic hands back at the handback speed exactly as with no lead. A lead counts
only when it is imminent: a driver who brakes at a moderate 2 m/s^2 only after the ramp, the margin and a 1.5 s
reaction could not stop 2 m short of it. While that holds, the braking side stays at the incoming limit and the ramp
clock stops (the car may brake to a stop); the handback finishes, re-seeded from the current command, once it clears.
An imminent lead latches (as openpilot brakes, the lead soon stops looking imminent — handing back then would leave the
car rolling a few metres behind a stopped car). Task 14: the latch SETS on the imminence test itself, whatever reason is
returned (A2: a hard-braking / FCW approach used to hide it); it holds until
the stop (standstill resolution).
Standstill resolution (Task 13 A1): openpilot never holds a manual at a stop. If the car has stood still for
STANDSTILL_RESOLVE seconds while urgent, the handback goes ahead anyway — the braking side ramps to 0 over the ramp
time and openpilot disengages; the driver / Auto Brake Hold / hill-hold hold the car, as after any handback. If the car
moves again (> STANDSTILL_SPEED + hysteresis) before the disengage, urgency counts again.
Driver-monitoring forceDecel never lets a handback start and aborts one in progress: stock brings the car to a stop (a
stall is acceptable in that emergency).

ONE source of truth: controlsd runs this state machine and publishes it (carControlSP.manualLongitudinal: active,
rampDone, disengage, the latched speed/ramp); selfdrived acts on the published state only (car_specific.py) — no second
timer, no second param read.

Manual + openpilot longitudinal only. Automatics/CVTs untouched; stock ACC (pcmCruise) is a no-op. Not a toggle:
without it openpilot brakes a manual to a stop and auto-launches it (stall/lurch), so this is the safe default.
"""
from opendbc.car.structs import car

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.braking_urgency import HANDBACK_DRIVER_REACTION

TransmissionType = car.CarParams.TransmissionType

# TODO(on-car): tune the handback speed and ramp for a smooth, non-jerky handoff in 1st gear. 5 mph in 1st is near
#  idle, so the right point may be a bit higher; hand off while still rolling in gear at a reasonable RPM.
DEFAULT_HANDBACK_SPEED = 2.24  # m/s (~5 mph)
MIN_HANDBACK_SPEED = 1.5       # m/s, floor so the param can't be tuned into braking to a stop / launching
MAX_HANDBACK_SPEED = 8.0       # m/s
DEFAULT_RAMP_TIME = 0.5        # s, accel command ramps to 0 over this before the disengage
MAX_RAMP_TIME = 2.0            # s
DISENGAGE_MARGIN = 0.1         # s, selfdrived disengages this long after the ramp ends (cross-process timing slack)
ENGAGE_HYSTERESIS = 0.5        # m/s, engaging is refused below handback speed + this
STANDSTILL_SPEED = 0.1         # m/s: below this the car is stopped (standstill resolution)
STANDSTILL_MOVING = 0.5        # m/s: moving again above this -> urgency counts again
STANDSTILL_RESOLVE = 0.5       # s stopped while urgent -> hand back anyway (never hold a manual at a stop)
UNCONDITIONAL = ("fcw", "stock_aeb")  # never resolved at a stop (FCW can't fire at a standstill anyway: planner gate)


def _read_float(params: Params, key: str, default: float, lo: float, hi: float) -> float:
  try:
    v = params.get(key, return_default=True)
    v = default if v is None else float(v)
  except (TypeError, ValueError):
    v = default
  return min(max(v, lo), hi)


def read_handback_params(params: Params) -> tuple[float, float]:
  return (_read_float(params, "ManualLowSpeedHandbackSpeed", DEFAULT_HANDBACK_SPEED, MIN_HANDBACK_SPEED, MAX_HANDBACK_SPEED),
          _read_float(params, "ManualLowSpeedHandbackRamp", DEFAULT_RAMP_TIME, 0., MAX_RAMP_TIME))


def handback_available(CP: car.CarParams) -> bool:
  return CP.transmissionType == TransmissionType.manual and bool(CP.openpilotLongitudinalControl) and not CP.pcmCruise


class LowSpeedHandback:
  def __init__(self, CP: car.CarParams, params: Params | None = None, process: str = "controlsd"):
    self.params = params or Params()
    self.available = handback_available(CP)
    self.process = process
    self._requested = read_handback_params(self.params)
    self.speed, self.ramp_time = self._requested
    self.active = False  # handing back (latched until disengaged)
    self.timer = 0.
    self.accel_start = 0.
    self.urgent = False
    self.standstill_timer = 0.
    self.resolved = False  # standstill resolution: stopped while urgent -> hand back anyway
    self.lead_latched = False  # an imminent lead was seen: braking stays until the stop (then standstill resolution)

  @property
  def reaction_time(self) -> float:
    """Ramp + disengage margin + the driver's reaction: how long until the driver brakes (handback_urgency)."""
    return self.ramp_time + DISENGAGE_MARGIN + HANDBACK_DRIVER_REACTION

  def read_params(self) -> None:
    # call at 1 Hz; values are applied only while disengaged, so a change never alters a handback in progress
    if self.available:
      self._requested = read_handback_params(self.params)

  def update(self, enabled: bool, v_ego: float, last_accel: float = 0., urgency: str | None = None,
             force_decel: bool = False, lead_imminent: bool = False) -> None:
    """Call every controlsd frame. last_accel: the accel command sent on the previous frame; urgency: the handback's
    carve-out, braking_urgency.handback_urgency() (Task 13 A1) — not the broad braking_urgency() set.
    lead_imminent: braking_urgency.handback_lead_imminent() - the latch's set condition, independent of which reason
    `urgency` is (Task 14 A2)."""
    if not self.available:
      return
    if self.lead_latched and urgency is None:
      urgency = "lead_imminent"  # an imminent lead holds the braking until the stop: never hand back mid-stop
    if self.active and urgency is not None and urgency not in UNCONDITIONAL and v_ego < STANDSTILL_SPEED:
      self.standstill_timer += DT_CTRL
      if self.standstill_timer >= STANDSTILL_RESOLVE - 1e-6 and not self.resolved:
        self.resolved = True
        cloudlog.event("manual_low_speed_handback_standstill", process=self.process, urgency=urgency)
    else:
      self.standstill_timer = 0.
      if v_ego > STANDSTILL_MOVING:
        self.resolved = False
    urgent = urgency is not None and not self.resolved
    if not enabled:
      self.active = False
      self.timer = 0.
      self.resolved = False
      self.lead_latched = False
      self.speed, self.ramp_time = self._requested
    elif force_decel:
      if self.active:
        cloudlog.event("manual_low_speed_handback_abort", process=self.process, reason="force_decel", v_ego=v_ego)
      self.active = False   # stock brings the car to a stop; no handback while driver monitoring forces a decel
      self.timer = 0.
      self.resolved = False
      self.lead_latched = False
    elif not self.active:
      if v_ego < self.speed:
        self.active = True
        self.timer = 0.
        self.accel_start = last_accel
        cloudlog.event("manual_low_speed_handback", process=self.process, v_ego=v_ego, accel=last_accel,
                       speed=self.speed, ramp=self.ramp_time, urgency=urgency or "")
    elif urgent:
      pass                  # the ramp clock stops: braking stays available
    else:
      if self.urgent:       # urgency just cleared: finish from where the command is now (no step)
        self.timer = 0.
        self.accel_start = last_accel
      self.timer += DT_CTRL
    self.urgent = urgent and self.active
    if self.active and (lead_imminent or urgency == "lead_imminent") and not self.lead_latched:
      self.lead_latched = True

  @property
  def ramp_done(self) -> bool:
    """controlsd: the command has reached 0, stop longitudinal (no gas, no brake, no stop/standstill request)."""
    return self.active and not self.urgent and self.timer >= self.ramp_time

  @property
  def disengage_due(self) -> bool:
    """Published for selfdrived: hand back now."""
    return self.active and not self.urgent and self.timer >= self.ramp_time + DISENGAGE_MARGIN - 1e-6

  def limit_accel(self, accel_limits: tuple[float, float]) -> tuple[float, float]:
    """controlsd: shrink the accel range linearly from the command at handback start to 0. While urgent the braking
    side stays at the incoming limit (Rule 5); only the throttle side ramps."""
    if not self.active:
      return accel_limits
    f = max(1. - self.timer / self.ramp_time, 0.) if self.ramp_time > 0. else 0.
    hi = min(accel_limits[1], max(self.accel_start, 0.) * f)
    lo = accel_limits[0] if self.urgent else max(accel_limits[0], min(self.accel_start, 0.) * f)
    return lo, max(hi, lo)

  def below_engage_speed(self, v_ego: float) -> bool:
    return self.available and v_ego < self.speed + ENGAGE_HYSTERESIS
