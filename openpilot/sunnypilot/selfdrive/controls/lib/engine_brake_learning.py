"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Engine-brake learning (Task 17): data collection, a brake check and persistence for refining the per-gear table
(engine_brake_table.STATIC_DECEL). Telemetry only - it never changes driving: the table updates it proposes are NOT
applied (engine_brake_preferred.APPLY_LEARNED is off until the collected data has been reviewed).

What it learns from - only measurements that cannot contain the service brakes (owner decision 2026-10-20). Comparing
the car's slowdown with the request can't tell engine braking from braking on this car (Stage 2: after a firm request
the brakes release slowly, so "slows more than asked" frames were still braked 80 % of the time):
  1. disengaged coasts: openpilot not engaged, no gas, no brake pedal (switch off AND VSA_STATUS.USER_BRAKE pedal
     pressure at rest - S8: rest -0.02, a light press ~0.5), a trusted gear, closed throttle (0x130 axle torque < 0),
     settled after the lift and the last gear change. The flat-road decel D = -(aEgo + g*sin(pitch)) is engine
     braking + road load, nothing else.
  2. the closed-throttle axle torque itself (CarStateSP.engineTorqueAxle, the engine's own figure): the engine part
     E = -K_TQ * torque at any grade, kept per cell next to D (D - E = road load; a cell whose D and E + road load
     disagree points at a pitch bias on the roads it was measured on).
Engaged frames (toggle on, longitudinal active, negative request) are logged for review with the torque check:
  extra = D - (E + road load(v))      > BRAKE_SUSPECT: decel the powertrain and the road don't explain = brakes likely
(Stage 2: 0.00 median / 2 % of frames past 0.3 on pure coasts; 0.6-1.1 at -1.2 requests). They are never learned from.

Table update rule (propose_table, NOT applied): per gear and speed bin, after >= MIN_SAMPLES coast samples, step the
learned value toward the measured lower confidence bound by at most MAX_STEP per drive cycle; never above that bound,
never below the static table (learned values only add decel authority). Run once per plannerd start ("drive cycle");
the proposal is stored and logged for review, and the next cycle builds on it (Stage 3: it used to restart from the
static table every cycle, so a proposal could never move more than MAX_STEP).

Estimator (Stage 3, v2 store): the MEDIAN of D per cell, from a persisted histogram (HIST_STEP bins), not the mean - the
mean ran 0.05-0.06 m/s^2 above the analysis's settled median (a long upper tail from descents / pitch error). Its lower
confidence bound is an order statistic: the p-quantile with p = 0.5 - Z * 0.5 / sqrt(n_eff), n_eff = the samples'
seconds / CORRELATION_TIME (10 Hz samples of one coast aren't independent). And a coast counts only SETTLE_IN_COAST into
it: the first second after the lift reads ~0.05 light (lift transient / fuel cut arriving). Every eligible frame still
goes to samples.jsonl, with its time into the coast.

Persistence: Params EngineBrakeLearnedTable (JSON, PERSISTENT, survives reboots), written (non-blocking) at most every
SAVE_PERIOD while there is new data. Samples: /data/engine_brake_learn/samples.jsonl (rotating, writer thread).
"""
import json
import logging
import math
import os
import queue
import threading
import time
from logging.handlers import RotatingFileHandler

from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_preferred import trusted_gear
from openpilot.sunnypilot.selfdrive.controls.lib.engine_brake_table import G, GEARS, STATIC_DECEL, V_MIDS

PARAM_KEY = "EngineBrakeLearnedTable"
VERSION = 2                 # 2: histogram cells (median); a v1 store (mean cells) is dropped at start, logged
V_EDGES = (3., 7., 11., 15., 19., 23., 27., 31., 40.)   # bins around engine_brake_table.V_MIDS
assert len(V_EDGES) == len(V_MIDS) + 1

K_TQ = 2.0767e-3            # m/s^2 per axle N*m = 1/(m*r): Stage 2 fit (m*r ~ 481 kg*m)
ROAD_LOAD = (0.188, 1.52e-4)   # m/s^2: c0 + c2*v^2, Stage 1 + Stage 3 fit (neutral coasts + settled in-gear D - E)
USER_BRAKE_MAX = 0.2        # VSA_STATUS.USER_BRAKE (deprecated.brake): pedal pressure above this = foot on the brake
SETTLE_AFTER_GAS = 0.5      # s after the gas pedal is released
SETTLE_AFTER_GEAR = 1.0     # s after the trusted gear last changed
SETTLE_IN_COAST = 1.0       # s into an unbroken coast before its samples count (Stage 3: the first second reads light)
MIN_COAST_SPEED = 3.        # m/s
BRAKE_SUSPECT = 0.3         # m/s^2 of unexplained decel
SAMPLE_PERIOD = 0.1         # s (10 Hz)
SAVE_PERIOD = 60.           # s
MIN_SAMPLES = 50            # coast samples (5 s at 10 Hz) in a cell before it may move
MAX_STEP = 0.02             # m/s^2 per drive cycle
CONFIDENCE_Z = 2.           # learned target = a lower order-statistic bound on the median (conservative)
CORRELATION_TIME = 1.0      # s of coast per independent sample (n_eff for the median's bound)
HIST_LO, HIST_STEP, HIST_BINS = -0.5, 0.01, 250   # D histogram per cell: -0.5 .. +2.0 m/s^2 (ends clamp)
LOG_MAX_BYTES = 1024 * 1024
LOG_BACKUPS = 10
QUEUE_MAX = 256
LOG_DIR_ENV = "ENGINE_BRAKE_LEARN_LOG_DIR"


def learn_log_dir() -> str:
  # next to swaglog's directory (/data/log -> /data/engine_brake_learn on device), like the coast logger
  return os.environ.get(LOG_DIR_ENV) or os.path.join(os.path.dirname(Paths.swaglog_root().rstrip("/")), "engine_brake_learn")


def v_bin(v_ego: float) -> int | None:
  if not math.isfinite(v_ego) or v_ego < V_EDGES[0] or v_ego >= V_EDGES[-1]:
    return None
  return next(i for i in range(len(V_MIDS)) if v_ego < V_EDGES[i + 1])


def road_load(v_ego: float) -> float:
  return ROAD_LOAD[0] + ROAD_LOAD[1] * v_ego * v_ego


class Cell:
  """One gear x speed bin: a histogram of D (its median and quantiles) and the mean torque-based engine part E."""
  __slots__ = ("n", "hist", "n_e", "mean_e")

  def __init__(self, n=0, hist=None, n_e=0, mean_e=0.):
    self.n, self.hist, self.n_e, self.mean_e = int(n), dict(hist or {}), int(n_e), float(mean_e)

  def add(self, d: float, e: float | None) -> None:
    i = min(max(int(math.floor((d - HIST_LO) / HIST_STEP)), 0), HIST_BINS - 1)
    self.hist[i] = self.hist.get(i, 0) + 1
    self.n += 1
    if e is not None:
      self.n_e += 1
      self.mean_e += (e - self.mean_e) / self.n_e

  def quantile(self, p: float) -> float:
    """D at quantile p (0..1): the centre of the bin the p*n-th sample falls in; 0 for an empty cell."""
    if self.n == 0:
      return 0.
    target, acc = max(p, 0.) * self.n, 0
    for i in sorted(self.hist):
      acc += self.hist[i]
      if acc >= target:
        return HIST_LO + (i + 0.5) * HIST_STEP
    return HIST_LO + (max(self.hist) + 0.5) * HIST_STEP

  @property
  def median(self) -> float:
    return self.quantile(0.5)

  def lower_bound(self) -> float:
    """A conservative median: the quantile Z binomial standard errors below 0.5, with n_eff = coast seconds /
    CORRELATION_TIME. 5 s of data -> the 5th percentile; 100 s -> the 40th."""
    if self.n == 0:
      return 0.
    n_eff = max(self.n * SAMPLE_PERIOD / CORRELATION_TIME, 1.)
    return self.quantile(0.5 - CONFIDENCE_Z * 0.5 / math.sqrt(n_eff))

  def to_dict(self) -> dict:
    return {"n": self.n, "h": {str(i): c for i, c in sorted(self.hist.items())}, "ne": self.n_e, "e": round(self.mean_e, 5)}


def _valid_number(x) -> bool:
  return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _parse_cell(cell) -> Cell | None:
  if not isinstance(cell, dict) or not isinstance(cell.get("h"), dict):
    return None
  n, n_e, e = cell.get("n"), cell.get("ne"), cell.get("e")
  if not all(_valid_number(x) for x in (n, n_e, e)) or n < 0 or n_e < 0:
    return None
  hist = {}
  for k, c in cell["h"].items():
    if not (isinstance(k, str) and k.isdigit() and int(k) < HIST_BINS) or not _valid_number(c) or c <= 0:
      return None
    hist[int(k)] = int(c)
  if sum(hist.values()) != int(n):
    return None
  return Cell(n, hist, n_e, e)


def parse_store(raw) -> dict:
  """The persisted JSON (Params returns a dict for a JSON key; a str / bytes is accepted too) -> {"stats": {gear: {bin:
  Cell}}, "proposed": {gear: [..]}, "cycles": int}. Anything malformed - or another VERSION - is dropped, never raised:
  a corrupt store only loses learning, it can't stop plannerd."""
  out: dict = {"stats": {g: {} for g in GEARS}, "proposed": {}, "cycles": 0}
  try:
    data = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
  except (TypeError, ValueError):
    return out
  if not isinstance(data, dict) or data.get("version") != VERSION:
    return out
  stats = data.get("stats")
  if isinstance(stats, dict):
    for gk, bins in stats.items():
      g = int(gk) if isinstance(gk, str) and gk.isdigit() else None
      if g not in GEARS or not isinstance(bins, dict):
        continue
      for bk, cell in bins.items():
        b = int(bk) if isinstance(bk, str) and bk.isdigit() else None
        c = _parse_cell(cell) if b is not None and 0 <= b < len(V_MIDS) else None
        if c is not None:
          out["stats"][g][b] = c
  proposed = data.get("proposed")
  if isinstance(proposed, dict):
    for gk, row in proposed.items():
      g = int(gk) if isinstance(gk, str) and gk.isdigit() else None
      if g in GEARS and isinstance(row, list) and len(row) == len(V_MIDS) and all(_valid_number(x) for x in row):
        out["proposed"][g] = [float(x) for x in row]
  if _valid_number(data.get("cycles")):
    out["cycles"] = max(int(data["cycles"]), 0)
  return out


def dump_store(store: dict) -> dict:
  return {"version": VERSION, "cycles": store["cycles"],
          "stats": {str(g): {str(b): c.to_dict() for b, c in bins.items()} for g, bins in store["stats"].items() if bins},
          "proposed": {str(g): [round(x, 3) for x in row] for g, row in store["proposed"].items()}}


def propose_table(stats: dict, static: dict = STATIC_DECEL, learned_prev: dict | None = None,
                  min_samples: int = MIN_SAMPLES, max_step: float = MAX_STEP) -> tuple[dict, list[str]]:
  """One drive cycle's table update (pure). Returns ({gear: row}, review notes). Per cell:
    prev   = the previous learned value, else the static one
    target = the measured lower confidence bound of D (brake-free coasts only)
    new    = prev moved toward target by at most max_step, never above target, never below static.
  A cell with fewer than min_samples keeps prev. A measurement below the static value is NOT applied (static is the
  floor by design) but is noted: it means the static table over-states engine braking there."""
  learned_prev = learned_prev or {}
  table, notes = {}, []
  for g in GEARS:
    row = []
    for b in range(len(V_MIDS)):
      s = static[g][b]
      prev_row = learned_prev.get(g)
      prev = prev_row[b] if prev_row is not None else s
      cell = stats.get(g, {}).get(b)
      new = prev
      if cell is not None and cell.n >= min_samples:
        target = cell.lower_bound()
        if target < s - 1e-6:
          notes.append(f"g{g} {V_MIDS[b]:.0f} m/s: measured {target:.3f} < static {s:.3f} (n {cell.n})")
        new = min(max(target, prev - max_step), prev + max_step)
        new = min(new, max(target, s))
      # TODO(review): decide whether a cell may move at all when its D and E + road load disagree (pitch bias)
      row.append(max(new, s))
    table[g] = row
  return table, notes


class _JsonlWriter:
  """Rotating JSON-lines file on a background thread; the caller only enqueues (a full queue drops and counts)."""

  def __init__(self, log_dir: str | None):
    self.log_dir = log_dir
    self._logger: logging.Logger | None = None
    self._failed = False
    self._queue: queue.Queue[str] = queue.Queue(maxsize=QUEUE_MAX)
    self._thread: threading.Thread | None = None
    self.written = self.dropped = 0

  def _file_logger(self) -> logging.Logger | None:
    if self._logger is None and not self._failed:
      try:
        log_dir = self.log_dir or learn_log_dir()
        os.makedirs(log_dir, exist_ok=True)
        handler = RotatingFileHandler(os.path.join(log_dir, "samples.jsonl"), maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUPS)
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger = logging.getLogger(f"engine_brake_learn.{id(self)}")
        logger.propagate = False
        logger.setLevel(logging.INFO)
        logger.addHandler(handler)
        self._logger = logger
      except OSError:
        cloudlog.exception("engine_brake_learn: can't open the sample log")
        self._failed = True
    return self._logger

  def put(self, obj: dict) -> None:
    if self._thread is None:
      self._thread = threading.Thread(target=self._loop, name="engine_brake_learn_writer", daemon=True)
      self._thread.start()
    try:
      self._queue.put_nowait(json.dumps(obj, separators=(",", ":")))
    except queue.Full:
      self.dropped += 1

  def _loop(self) -> None:
    while True:
      line = self._queue.get()
      try:
        logger = self._file_logger()
        if logger is not None:
          logger.info(line)
          self.written += 1
      except (OSError, ValueError):
        cloudlog.exception("engine_brake_learn: write failed")
      finally:
        self._queue.task_done()

  def drain(self, timeout: float = 2.0) -> bool:
    end = time.monotonic() + timeout
    while self._queue.unfinished_tasks and time.monotonic() < end:
      time.sleep(0.005)
    return not self._queue.unfinished_tasks


class EngineBrakeLearner:
  """plannerd, every frame (20 Hz): collects at SAMPLE_PERIOD. Manual + live clutch signal only; inert otherwise."""

  def __init__(self, available: bool, dt: float, params: Params | None = None, log_dir: str | None = None):
    self.available = available
    self.dt = dt
    self.params = params or Params()
    self.writer = _JsonlWriter(log_dir)
    raw = self.params.get(PARAM_KEY) if available else None
    self.store = parse_store(raw)
    if isinstance(raw, dict) and raw.get("version") not in (None, VERSION):
      cloudlog.event("engine_brake_learn_store_reset", found=raw.get("version"), version=VERSION)
    self._since_sample = SAMPLE_PERIOD
    self._since_save = 0.
    self._dirty = False
    self._since_gas = self._since_gear = 0.
    self._last_gear = 0
    self._coast_run = 0.   # s into the current unbroken coast (SETTLE_IN_COAST)
    self.coast_samples = self.engaged_samples = self.brake_suspect = 0
    if available:
      self._new_cycle()

  def _new_cycle(self) -> None:
    """Once per plannerd start: the proposal from everything collected so far, stored and logged for review only."""
    self.store["cycles"] += 1
    proposed, notes = propose_table(self.store["stats"], learned_prev=self.store["proposed"] or None)   # builds up
    self.store["proposed"] = proposed
    moved = sum(abs(proposed[g][b] - STATIC_DECEL[g][b]) > 1e-6 for g in GEARS for b in range(len(V_MIDS)))
    cells = sum(c.n >= MIN_SAMPLES for bins in self.store["stats"].values() for c in bins.values())
    cloudlog.event("engine_brake_learn_cycle", cycle=self.store["cycles"], cells_ready=cells, cells_moved=moved,
                   notes=notes[:12], applied=False)
    # TODO(review): once the logs are reviewed, apply the proposal (store it as the learned table the clamp reads, with
    #  engine_brake_preferred.APPLY_LEARNED on) - until then nothing here reaches the clamp.
    self._dirty = True

  def update(self, CS, CS_SP, enabled: bool, long_active: bool, sent_accel: float, ebp_on: bool,
             pitch: float | None) -> None:
    if not self.available:
      return
    clutch_valid = bool(CS_SP.clutchSignalValid)
    gear = trusted_gear(CS_SP.gearRaw, CS_SP.clutchPedal, CS.deprecated.clutchPressed, CS.deprecated.engineRpm,
                        CS.vEgo) if clutch_valid else 0
    self._since_gas = 0. if CS.gasPressed else self._since_gas + self.dt
    self._since_gear = 0. if gear != self._last_gear else self._since_gear + self.dt
    self._last_gear = gear
    self._since_save += self.dt
    if self._dirty and self._since_save >= SAVE_PERIOD:
      self.save()

    self._since_sample += self.dt
    if self._since_sample < SAMPLE_PERIOD - 1e-6:
      return
    self._since_sample = 0.
    if not clutch_valid or pitch is None or not math.isfinite(pitch):
      self._coast_run = 0.
      return

    v, torque = float(CS.vEgo), float(CS_SP.engineTorqueAxle)
    d = -(float(CS.aEgo) + G * math.sin(pitch))
    e = -K_TQ * torque if math.isfinite(torque) else None
    b = v_bin(v)
    pedal = float(CS.deprecated.brake)

    if not enabled:
      coasting = gear and b is not None and v >= MIN_COAST_SPEED and not CS.gasPressed and not CS.brakePressed and \
                 pedal < USER_BRAKE_MAX and e is not None and torque < 0. and \
                 self._since_gas >= SETTLE_AFTER_GAS and self._since_gear >= SETTLE_AFTER_GEAR
      if coasting:
        learned = self._coast_run >= SETTLE_IN_COAST - 1e-6
        if learned:
          self.store["stats"][gear].setdefault(b, Cell()).add(d, e)
          self._dirty = True
          self.coast_samples += 1
        self.writer.put({"src": "coast", "t": round(time.monotonic(), 2), "gear": gear, "v": round(v, 3),
                         "a_ego": round(float(CS.aEgo), 4), "pitch": round(pitch, 5), "tq": round(torque, 1),
                         "rpm": round(float(CS.deprecated.engineRpm), 1), "d": round(d, 4), "e": round(e, 4),
                         "run": round(self._coast_run, 1), "learned": learned})
        self._coast_run += SAMPLE_PERIOD
      else:
        self._coast_run = 0.
      return
    self._coast_run = 0.

    # engaged, for review only: every negative-request frame while the toggle is latched on
    if ebp_on and long_active and sent_accel < 0.:
      extra = d - (e + road_load(v)) if e is not None else math.nan
      suspect = math.isfinite(extra) and extra > BRAKE_SUSPECT
      self.engaged_samples += 1
      self.brake_suspect += int(suspect)
      self.writer.put({"src": "engaged", "t": round(time.monotonic(), 2), "gear": gear, "v": round(v, 3),
                       "sent": round(float(sent_accel), 3), "a_ego": round(float(CS.aEgo), 4), "pitch": round(pitch, 5),
                       "tq": round(torque, 1) if math.isfinite(torque) else None, "pedal": round(pedal, 2),
                       "extra": round(extra, 3) if math.isfinite(extra) else None, "brake_suspect": suspect})

  def save(self) -> None:
    self.params.put(PARAM_KEY, dump_store(self.store))   # block=False: Params persists it in the background
    self._dirty = False
    self._since_save = 0.
