#!/usr/bin/env python3
"""EPS bench - low-speed-band steering state machine (pure, no I/O).

Run 1 (2026-10-25) proved the stock EPS gates LKAS actuation on vehicle speed
(min engage ~=10 km/h): a well-formed 0x0E4 is ACCEPTED parked (STEER_STATUS ->
normal) but no motor torque is applied. So the torque-cap (clip-vs-track) test
must run with the car MOVING in the engage band, on a closed course. This module
is the INVERSE of the parked interlock: it may command ONLY inside a low-speed
band, triggered by the clutch-released CAN signal and a speed-gated start.

Method: CLOSED_COURSE_TEST_DIRECTIVE_2026-10-25.md / SP_DEV_HANDOFF_CLOSED_COURSE
(both in Dropbox CommaAI/EPS_Testing/). Firmware-RE verified facts only; ASK for
anything else (directive s0).

This file is PURE (no cereal / no panda / no wall-clock of its own): update() takes
the current time, speed, clutch state and ui-abort flag and returns the command to
send. steer_injector.py wires cereal to it; test_lowspeed.py drives it with
synthetic inputs (the directive s8 bench test - no motion).

State machine:
  IDLE  --clutch released (rising edge)-->  ARMED
  ARMED --speed >= V_MIN + V_START_MARGIN (fresh)-->  RUNNING   (clutch pressed -> IDLE)
         (ARMED pre-rolls REQUEST=1 at torque 0 so the stream/counter is continuous
          before the ramp - no cold-start step at the engage crossing; RE flag 1)
  RUNNING --single bounded ramp toward DIRECTION*TARGET; auto-stop on the tightest of
            T_MAX / speed>V_MAX / speed<V_MIN-V_DROP / clutch-in / stale speed /
            session timeout / ui-abort / exception-->  STOPPING
  STOPPING --ramp cmd -> 0, then explicit STEER_TORQUE=0,REQUEST=0-->  DONE
Every exit biases to zero command and is rate-limited (never a step).
"""
from __future__ import annotations

from dataclasses import dataclass

# States
IDLE = "IDLE"
ARMED = "ARMED"
RUNNING = "RUNNING"
STOPPING = "STOPPING"
DONE = "DONE"

# Advisory high-command threshold (0x0E4 units). Above ~half the ~1393 cap the car
# turns more; the RE team suggested a yaw/excursion abort before crossing it. Owner
# decision (2026-10-25): NOT a blocker at creep speed (~3 m/s, bounded <=1.5 s run,
# driver on the brake + clutch + kill) - the human + T_MAX + band are the backstop.
# So this is an ADVISORY ONLY: the injector prints a one-line note above it and runs
# normally. If the yaw abort is ever built it can gate here instead.
ESCALATION_ADVISORY = 700.0


@dataclass
class LowSpeedConfig:
  v_min: float = 12.0            # km/h, EPS engage band floor (firmware ~10; margin above)
  v_max: float = 20.0            # km/h, hard band ceiling -> abort above
  v_start_margin: float = 1.0    # km/h, START only at V_MIN + this (provably engaged)
  v_drop: float = 2.0            # km/h, abort if speed < V_MIN - this (EPS would disengage)
  t_max: float = 1.5             # s, max duration of a commanded run (from RUNNING start)
  target: float = 300.0          # raw 0x0E4 STEER_TORQUE peak (well below cap first; raise deliberately)
  max_cmd: float = 600.0         # hard magnitude ceiling, independent of target
  max_rate: float = 400.0        # raw units / s, anti-slam
  ramp_s: float = 1.0            # intended 0 -> target ramp time
  rate_hz: float = 100.0
  direction: int = 1             # +1 / -1 single-direction ramp
  speed_stale_s: float = 0.5     # s, speed older than this = unknown
  session_timeout_s: float = 60.0  # s, global watchdog from controller start
  arm_requires_clutch: bool = True  # if False, ARM is automatic (bench/debug only)

  @classmethod
  def from_dict(cls, cfg: dict) -> "LowSpeedConfig":
    return cls(
      v_min=float(cfg.get("V_MIN", 12.0)),
      v_max=float(cfg.get("V_MAX", 20.0)),
      v_start_margin=float(cfg.get("V_START_MARGIN", 1.0)),
      v_drop=float(cfg.get("V_DROP", 2.0)),
      t_max=float(cfg.get("T_MAX", 1.5)),
      target=abs(float(cfg.get("TARGET", 300.0))),
      max_cmd=abs(float(cfg.get("MAX_CMD", 600.0))),
      max_rate=float(cfg.get("MAX_RATE", 400.0)),
      ramp_s=float(cfg.get("RAMP_S", 1.0)),
      rate_hz=float(cfg.get("RATE_HZ", 100.0)),
      direction=1 if int(cfg.get("DIRECTION", 1)) >= 0 else -1,
      speed_stale_s=float(cfg.get("SPEED_STALE_S", 0.5)),
      session_timeout_s=float(cfg.get("SESSION_TIMEOUT_S", 60.0)),
      arm_requires_clutch=bool(cfg.get("ARM_REQUIRES_CLUTCH", True)),
    )


def escalation_note(cfg: "LowSpeedConfig") -> str | None:
  """Advisory only (not a gate). Returns a one-line note to print if TARGET/MAX_CMD
  is in the higher-command region (> ESCALATION_ADVISORY), else None. The run is not
  blocked - the safety driver + T_MAX + band are the backstop at creep speed."""
  worst = max(abs(cfg.target), abs(cfg.max_cmd))
  if worst > ESCALATION_ADVISORY:
    return (f"high command: TARGET/MAX_CMD {worst:.0f} > {ESCALATION_ADVISORY:.0f} "
            f"(car turns more at speed; driver ready to brake/clutch, {cfg.t_max:.1f}s T_MAX)")
  return None


@dataclass
class Step:
  cmd: float          # STEER_TORQUE to send this frame (rate-limited)
  request: bool       # STEER_TORQUE_REQUEST
  state: str          # current state after this update
  setpoint: float     # the setpoint cmd is slewing toward
  reason: str         # abort/stop reason, "" otherwise
  armed_edge: bool    # True on the frame ARM fired (for logging-with-marker)
  started_edge: bool  # True on the frame RUNNING began


class LowSpeedController:
  """Pure state machine. Feed update() once per frame; it never blocks or does I/O."""

  def __init__(self, cfg: LowSpeedConfig, t0: float):
    self.cfg = cfg
    self.t0 = t0
    self.state = IDLE
    self.cmd = 0.0
    self.reason = ""
    self.prev_clutch_up = None   # type: bool | None
    self.t_arm = None            # type: float | None
    self.t_run = None            # type: float | None
    # strictest of configured rate and target/ramp (matches the parked intent)
    ramp_rate = cfg.target / cfg.ramp_s if cfg.ramp_s > 0 else cfg.max_rate
    self.max_step = min(cfg.max_rate, ramp_rate) / cfg.rate_hz
    self.stop_max_step = cfg.max_rate / cfg.rate_hz  # ramp-out uses the full anti-slam rate

  def _slew(self, setpoint: float, step: float) -> None:
    delta = setpoint - self.cmd
    if delta > step:
      delta = step
    elif delta < -step:
      delta = -step
    self.cmd += delta
    if self.cmd > self.cfg.max_cmd:
      self.cmd = self.cfg.max_cmd
    elif self.cmd < -self.cfg.max_cmd:
      self.cmd = -self.cfg.max_cmd

  def _go_stopping(self, reason: str) -> None:
    if self.state != STOPPING and self.state != DONE:
      self.state = STOPPING
      if not self.reason:
        self.reason = reason

  def update(self, t: float, speed: float | None, clutch_up: bool | None,
             ui_abort: bool) -> Step:
    """One frame.

    t          monotonic seconds (same clock as t0)
    speed      km/h, or None if unknown/stale (caller decides staleness)
    clutch_up  True = clutch pedal fully released (armable), False = pressed,
               None = clutch signal unknown/stale
    ui_abort   True = STOP requested / kill / exception upstream
    """
    cfg = self.cfg
    armed_edge = False
    started_edge = False

    # --- global watchdog + UI abort apply in every live state ---
    if self.state in (IDLE, ARMED, RUNNING):
      if ui_abort:
        self._go_stopping("ui_abort")
      elif t - self.t0 > cfg.session_timeout_s:
        self._go_stopping("session_timeout")

    if self.state == IDLE:
      # ARM on clutch-released rising edge (None/False -> True). If ARM doesn't
      # require the clutch (bench/debug), arm as soon as we're live.
      if not cfg.arm_requires_clutch:
        self.state = ARMED
        self.t_arm = t
        armed_edge = True
      elif clutch_up is True and self.prev_clutch_up is not True:
        self.state = ARMED
        self.t_arm = t
        armed_edge = True

    elif self.state == ARMED:
      if cfg.arm_requires_clutch and clutch_up is False:
        # clutch pressed again before start -> dis-arm, wait for the next release
        self.state = IDLE
        self.t_arm = None
      elif speed is None:
        pass  # no fresh speed yet; stay armed, command nothing
      elif speed > cfg.v_max:
        pass  # too fast to start a run; wait for it to settle into the band
      elif speed >= cfg.v_min + cfg.v_start_margin:
        self.state = RUNNING
        self.t_run = t
        started_edge = True

    elif self.state == RUNNING:
      # auto-stop on the tightest condition
      if speed is None:
        self._go_stopping("speed_unknown_or_stale")
      elif cfg.arm_requires_clutch and clutch_up is False:
        self._go_stopping("clutch_in")
      elif speed > cfg.v_max:
        self._go_stopping("speed_above_band")
      elif speed < cfg.v_min - cfg.v_drop:
        self._go_stopping("speed_below_band")
      elif self.t_run is not None and t - self.t_run >= cfg.t_max:
        self._go_stopping("t_max")

    # --- command generation ---
    if self.state == RUNNING:
      setpoint = cfg.direction * cfg.target
      self._slew(setpoint, self.max_step)
    elif self.state == STOPPING:
      setpoint = 0.0
      self._slew(setpoint, self.stop_max_step)
      if abs(self.cmd) < 1.0:
        self.cmd = 0.0
        self.state = DONE
    else:  # IDLE, ARMED, DONE -> command nothing
      setpoint = 0.0
      # in case we armed with a residual (shouldn't happen), bleed to 0
      if self.cmd != 0.0:
        self._slew(0.0, self.stop_max_step)

    # REQUEST assertion:
    #  ARMED  -> pre-roll: REQUEST=1 at torque 0 so the counter/stream and the
    #           request bit are continuous BEFORE the ramp, removing any cold-start
    #           step at the engage crossing (RE-team flag 1, 2026-10-25).
    #  RUNNING/STOPPING -> REQUEST=1 (stays 1 through the ramp-out until cmd hits 0).
    #  IDLE   -> not streaming an asserted request (REQUEST=0), except to bleed a residual.
    #  DONE   -> explicit release (REQUEST=0).
    if self.state in (ARMED, RUNNING, STOPPING):
      request = True
    elif self.state == DONE:
      request = False
    else:  # IDLE
      request = abs(self.cmd) >= 1.0  # only if bleeding a residual

    self.prev_clutch_up = clutch_up
    return Step(cmd=self.cmd, request=request, state=self.state,
                setpoint=setpoint, reason=self.reason,
                armed_edge=armed_edge, started_edge=started_edge)
