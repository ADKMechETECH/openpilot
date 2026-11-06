"""
fork-only (civic-cruise) NNLC low-speed hunt self-tuner (owner 2026-11-06: "build it"; default OFF).

The low-speed steering hunt (EPS developer's handoff; analysis/session15/lat_hunt.py): below ~25 mph the lateral loop
limit-cycles at ~2 Hz. The oscillator is the torque controller's feedback, through NNLC's low-speed curvature term
(nnlc.py: setpoint / measurement += low_speed_factor * curvature, low_speed_factor = interp(v, LOW_SPEED_X,
LOW_SPEED_Y)^2 = 104 at 4.5 mph), on a yaw-rate/speed curvature that is noisy at low speed and an EPS that delivers
1.85x stock authority. Nothing adapts that term: NNLC's model is trained offline, torqued learns the overall gain,
MVL's model learns the wire. This tuner adapts exactly that term, the way a relay auto-tuner backs a gain off:

  detect   engaged, lateral active, hands off, near-straight (|desired curvature| < 0.01, |angle| < 20 deg),
           2-25 mph: the wheel angle's 2 Hz band RMS (RBJ biquad, Q 1.4, 1 s RMS) above ANGLE_BAND and the torque
           request's above REQ_BAND. On the four drives of 2026-11-06 the hunting runs sit at 0.37-1.3 deg (p10-p90)
           and the calm runs under 0.23; manoeuvring is kept out by the near-straight gate.
  act      hunting for ON_TIME -> scale *= STEP_DOWN (then COOLDOWN before the next step), down to SCALE_MIN.
  recover  calm low-speed cruising for CREEP_TIME -> scale *= CREEP_UP, up to 1.0 - so an over-reduction heals and the
           tune follows the car (EPS tune changes, tyres, learning).
  persist  NnlcLowSpeedScale, written only parked and disengaged, at most once per stop, only if it moved
           (the session 11 rule: no Params writes on the driving path).

Toggle NnlcLowSpeedTuner (default OFF): off, the scale is 1.0 and NNLC is stock. Low-speed authority was never a goal
of this project; mid / high speed is untouched (the term is 0 above 30 m/s and tiny above 20).
"""
import math

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

PARAM_TOGGLE = "NnlcLowSpeedTuner"
PARAM_SCALE = "NnlcLowSpeedScale"

V_MIN, V_MAX = 1.0, 11.2        # m/s: the regime the hunt lives in (up to ~25 mph)
STRAIGHT_CURV = 0.01            # 1/m
STRAIGHT_ANGLE = 20.0           # deg
ANGLE_BAND = 0.30               # deg, 1 s RMS of the wheel angle's 2 Hz band
REQ_BAND = 0.10                 # torque request (-1..1), same band
ON_TIME = 1.0                   # s hunting before a step
COOLDOWN = 3.0                  # s after a step before the next
STEP_DOWN = 0.85
SCALE_MIN, SCALE_MAX = 0.25, 1.0
CREEP_TIME = 60.0               # s of calm low-speed cruising per creep
CREEP_UP = 1.02
PERSIST_MIN_CHANGE = 0.01


class Biquad:
  """RBJ band-pass, constant 0 dB peak gain."""

  def __init__(self, f0: float, q: float, fs: float):
    w = 2 * math.pi * f0 / fs
    al = math.sin(w) / (2 * q)
    a0 = 1 + al
    self.b0, self.b1, self.b2 = al / a0, 0.0, -al / a0
    self.a1, self.a2 = -2 * math.cos(w) / a0, (1 - al) / a0
    self.x1 = self.x2 = self.y1 = self.y2 = 0.0

  def update(self, x: float) -> float:
    y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
    self.x2, self.x1, self.y2, self.y1 = self.x1, x, self.y1, y
    return y


class BandRms:
  def __init__(self, f0: float, q: float, fs: float, tau: float):
    self.bp = Biquad(f0, q, fs)
    self.a = (1.0 / fs) / (tau + 1.0 / fs)
    self.ms = 0.0

  def update(self, x: float) -> float:
    y = self.bp.update(x)
    self.ms += self.a * (y * y - self.ms)
    return math.sqrt(self.ms)

  def reset(self) -> None:
    self.bp.x1 = self.bp.x2 = self.bp.y1 = self.bp.y2 = 0.0
    self.ms = 0.0


class LowSpeedHuntTuner:
  def __init__(self, dt: float, params: Params | None = None):
    self.dt = dt
    self._params = params or Params()
    self._frame = 0
    self._read_period = max(1, int(round(1.0 / dt)))
    self.enabled = False
    self.scale = SCALE_MAX
    self._persisted = SCALE_MAX
    self._read_params(first=True)
    self.angle_band = BandRms(2.0, 1.4, 1.0 / dt, 1.0)
    self.req_band = BandRms(2.0, 1.4, 1.0 / dt, 1.0)
    self._hunt_t = 0.0
    self._cool_t = 0.0
    self._calm_t = 0.0
    self._stopped = False
    self.hunting = False            # for logs / tests
    self.steps_down = 0

  def _read_params(self, first: bool = False) -> None:
    self.enabled = bool(self._params.get_bool(PARAM_TOGGLE))
    if first:
      s = self._params.get(PARAM_SCALE, return_default=True)
      try:
        self.scale = min(max(float(s), SCALE_MIN), SCALE_MAX) if s is not None else SCALE_MAX
      except (TypeError, ValueError):
        self.scale = SCALE_MAX
      self._persisted = self.scale

  @property
  def factor_scale(self) -> float:
    """What nnlc.py multiplies its low-speed factor by (1.0 whenever the tuner is off)."""
    return self.scale if self.enabled else SCALE_MAX

  def update(self, v_ego: float, hands_off: bool, desired_curvature: float, angle_deg: float, torque_request: float) -> None:
    """Every frame the lateral controller is active."""
    if self._frame % self._read_period == 0:
      self._read_params()
    self._frame += 1
    a = self.angle_band.update(float(angle_deg))
    r = self.req_band.update(float(torque_request))
    regime = hands_off and V_MIN <= v_ego <= V_MAX and abs(desired_curvature) < STRAIGHT_CURV and abs(angle_deg) < STRAIGHT_ANGLE
    self.hunting = bool(regime and a > ANGLE_BAND and r > REQ_BAND)
    if not self.enabled:
      self._hunt_t = self._calm_t = 0.0
      return
    self._cool_t = max(self._cool_t - self.dt, 0.0)
    if self.hunting:
      self._calm_t = 0.0
      self._hunt_t += self.dt
      if self._hunt_t >= ON_TIME and self._cool_t <= 0.0 and self.scale > SCALE_MIN:
        self.scale = max(self.scale * STEP_DOWN, SCALE_MIN)
        self._cool_t, self._hunt_t = COOLDOWN, 0.0
        self.steps_down += 1
        cloudlog.event("nnlc_low_speed_tuner_step", scale=self.scale, v_ego=v_ego, angle_band=a, req_band=r)
    else:
      self._hunt_t = 0.0
      if regime:
        self._calm_t += self.dt
        if self._calm_t >= CREEP_TIME and self.scale < SCALE_MAX:
          self.scale = min(self.scale * CREEP_UP, SCALE_MAX)
          self._calm_t = 0.0
          cloudlog.event("nnlc_low_speed_tuner_creep", scale=self.scale)

  def inactive(self) -> None:
    """Every frame the lateral controller is not active: the filters start fresh next time."""
    self.angle_band.reset()
    self.req_band.reset()
    self._hunt_t = self._calm_t = 0.0
    self.hunting = False

  def persist(self, standstill: bool, enabled: bool) -> None:
    """Parked and disengaged: write the scale once per stop if it moved. Never on the driving path."""
    stopped = bool(standstill and not enabled)
    if stopped and not self._stopped and abs(self.scale - self._persisted) >= PERSIST_MIN_CHANGE:
      self._params.put(PARAM_SCALE, float(self.scale), block=True)   # parked: a blocking write is allowed here
      self._persisted = self.scale
      cloudlog.event("nnlc_low_speed_tuner_persist", scale=self.scale)
    self._stopped = stopped
