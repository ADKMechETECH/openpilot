"""
fork-only (civic-cruise) non-ACC: engine-brake learner.

Refines the per-gear closed-throttle decel table (engine_brake.STATIC_DECEL) from the car's own brake-free coasts, and -
per the owner's "apply live, bounded" decision - the refinement is APPLIED (the controller's coast estimate uses the
learned value), not just logged.

Learns ONLY from measurements that cannot contain the service brakes (owner decision): disengaged coasts with no gas,
the brake switch off and USER_BRAKE pedal pressure at rest, a trusted gear, closed throttle (0x130 axle torque < 0),
settled a moment after the gas release / a gear change and >= SETTLE_IN_COAST into an unbroken coast (the first second
of a coast reads light). The grade-corrected flat-road decel is D = -aEgo - g*sin(pitch).

Bounds (so a bad cell can never remove braking authority or lurch): per gear x speed bin, with >= MIN_SAMPLES, the
learned value is the conservative lower bound of the measured median (an order statistic Z binomial SEs below 0.5,
n_eff = coast seconds), clamped to [static, static + MAX_LEARN]. Never below static (learned only adds decel), never
more than MAX_LEARN above it. Persisted to Params EngineBrakeLearnedTable (JSON) so it survives reboots.
"""
import json
import math

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.engine_brake import G, STATIC_DECEL, V_MIDS, table_decel

PARAM = "EngineBrakeLearnedTable"
V_EDGES = (3., 7., 11., 15., 19., 23., 27., 31., 40.)   # bins around engine_brake.V_MIDS
SAMPLE_PERIOD = 0.1          # learner runs at 10 Hz
SETTLE_AFTER_GAS = 0.5       # s after the gas pedal is released
SETTLE_AFTER_GEAR = 1.0      # s after the trusted gear last changed
SETTLE_IN_COAST = 1.0        # s into an unbroken coast before its samples count
USER_BRAKE_MAX = 0.2         # VSA_STATUS.USER_BRAKE (deprecated.brake): above this = foot on the brake
MIN_SAMPLES = 50             # per cell before it is trusted (>= 5 s of coast)
MAX_LEARN = 0.25             # m/s^2: the learned value may exceed static by at most this
CONFIDENCE_Z = 2.0           # learned target = a lower order-statistic bound on the median
CORRELATION_TIME = 1.0       # s of coast per independent sample
HIST_LO, HIST_STEP, HIST_BINS = -0.5, 0.01, 250   # D histogram per cell, -0.5 .. +2.0 m/s^2
SAVE_PERIOD = 10.0           # s between Param writes while there is new data


def v_bin(v_ego: float) -> int | None:
  if not math.isfinite(v_ego) or v_ego < V_EDGES[0] or v_ego >= V_EDGES[-1]:
    return None
  for i in range(len(V_MIDS)):
    if V_EDGES[i] <= v_ego < V_EDGES[i + 1]:
      return i
  return None


class Cell:
  """One gear x speed bin: a histogram of measured D, with a conservative lower bound on its median."""
  def __init__(self, hist=None):
    self.hist: dict[int, int] = hist or {}

  @property
  def n(self) -> int:
    return sum(self.hist.values())

  def add(self, d: float) -> None:
    i = min(max(int((d - HIST_LO) / HIST_STEP), 0), HIST_BINS - 1)
    self.hist[i] = self.hist.get(i, 0) + 1

  def quantile(self, p: float) -> float:
    n = self.n
    if n == 0:
      return 0.
    target, acc = max(p, 0.) * n, 0
    for i in sorted(self.hist):
      acc += self.hist[i]
      if acc >= target:
        return HIST_LO + (i + 0.5) * HIST_STEP
    return HIST_LO + (HIST_BINS - 0.5) * HIST_STEP

  def lower_bound(self) -> float:
    n_eff = max(self.n * SAMPLE_PERIOD / CORRELATION_TIME, 1.)
    return self.quantile(0.5 - CONFIDENCE_Z * 0.5 / math.sqrt(n_eff))


class EngineBrakeLearner:
  def __init__(self, enabled: bool, dt: float, params: Params | None = None):
    self.enabled = enabled
    self.dt = dt
    self._params = params or Params()
    self.cells: dict[tuple[int, int], Cell] = {}
    self._since_save = 0.
    self._dirty = False
    # coast settling state
    self._coast_t = 0.
    self._since_gas = 1e9
    self._since_gear = 1e9
    self._gear = None
    self.learned: dict[int, list[float]] = {}
    self._load()

  # ---- persistence -------------------------------------------------------
  def _load(self) -> None:
    try:
      raw = self._params.get(PARAM)
      if raw:
        data = json.loads(raw)
        for key, cell in data.get("cells", {}).items():
          g, b = (int(x) for x in key.split(","))
          self.cells[(g, b)] = Cell({int(i): int(c) for i, c in cell.items()})
    except (ValueError, TypeError, AttributeError):
      self.cells = {}
    self._recompute()

  def _save(self) -> None:
    data = {
      "cells": {f"{g},{b}": c.hist for (g, b), c in self.cells.items()},
      "applied": {str(g): row for g, row in self.learned.items()},   # what the controller reads
    }
    self._params.put(PARAM, json.dumps(data))   # non-blocking by default
    self._dirty = False
    self._since_save = 0.

  # ---- learning ----------------------------------------------------------
  def observe(self, *, engaged: bool, gas: bool, brake_switch: bool, user_brake: float,
              gear, torque_axle: float, v_ego: float, a_ego: float, pitch) -> None:
    """Call at 10 Hz. Accumulates a brake-free coast sample into its cell when every gate passes."""
    if not self.enabled:
      return
    # track settling
    self._since_gas = 0. if gas else self._since_gas + self.dt
    if gear != self._gear:
      self._gear = gear
      self._since_gear = 0.
    else:
      self._since_gear += self.dt

    coasting = (not engaged and not gas and not brake_switch and user_brake < USER_BRAKE_MAX
                and gear is not None and math.isfinite(torque_axle) and torque_axle < 0.
                and pitch is not None and math.isfinite(pitch) and v_ego >= V_EDGES[0])
    if not coasting:
      self._coast_t = 0.
      return
    self._coast_t += self.dt
    if self._coast_t < SETTLE_IN_COAST or self._since_gas < SETTLE_AFTER_GAS or self._since_gear < SETTLE_AFTER_GEAR:
      return
    b = v_bin(v_ego)
    if b is None:
      return
    d = -a_ego - G * math.sin(pitch)   # grade-corrected flat-road closed-throttle decel
    if not math.isfinite(d):
      return
    self.cells.setdefault((int(gear), b), Cell()).add(d)
    self._dirty = True
    self._since_save += self.dt
    self._recompute()
    if self._since_save >= SAVE_PERIOD:
      self._save()

  def _recompute(self) -> None:
    """Rebuild the applied learned table: per gear a list over V_MIDS, each clamped to [static, static + MAX_LEARN].
    The learned value is a conservative lower bound on the measured median (Z binomial SEs below 0.5, n_eff = coast
    seconds / CORRELATION_TIME), so a cell needs a lot of data before it credits more engine braking than static."""
    learned: dict[int, list[float]] = {}
    for (g, b), cell in self.cells.items():
      if cell.n < MIN_SAMPLES:
        continue
      static = STATIC_DECEL.get(g)
      if static is None:
        continue
      n_eff = max(cell.n * self.dt / CORRELATION_TIME, 1.)
      p = 0.5 - CONFIDENCE_Z * 0.5 / math.sqrt(n_eff)
      row = learned.setdefault(g, list(static))
      row[b] = float(min(max(cell.quantile(p), static[b]), static[b] + MAX_LEARN))
    self.learned = learned

  def flush(self) -> None:
    if self._dirty:
      self._save()


def load_applied(params: Params) -> dict[int, list[float]]:
  """Read the applied learned table the learner persisted (for the controller process, which doesn't run the learner)."""
  try:
    raw = params.get(PARAM)
    if raw:
      applied = json.loads(raw).get("applied", {})
      return {int(g): [float(x) for x in row] for g, row in applied.items()}
  except (ValueError, TypeError, AttributeError):
    pass
  return {}


def effective_decel(gear: int, v_ego: float, learned: dict[int, list[float]] | None) -> float:
  """Flat-road closed-throttle decel, learned value if a cell is trusted else the static seed."""
  if learned and gear in learned:
    import numpy as np
    return max(float(np.interp(v_ego, V_MIDS, learned[gear])), 0.)
  return table_decel(gear, v_ego)
