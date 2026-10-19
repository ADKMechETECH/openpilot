"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console capture state machine + storage (vehicle-agnostic). Pure logic over Samples (the daemon feeds it; tests
drive it directly). The plan comes from the Context's vehicle profile; capture files and markers name each Sample field
by the profile's column name (a car's logs keep their spelling).

  idle --arm--> armed --button--> capturing --goal met--> complete --button--> capturing (fresh run)
                                      |  ^
                               button v  | button          (stop = pause: progress is kept and the next press
                                   stopped                  resumes into a new capture window)
  disarm / test change / shutdown while capturing -> abort marker.

Default-inert: nothing armed and background off = no markers, no capture files (only the live status file).
"""
import json
import logging
import math
import os
import time
from logging.handlers import RotatingFileHandler

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.test_console.analyzers import CAPTURED, FAIL, PASS, Analyzer, make_analyzer
from openpilot.sunnypilot.selfdrive.test_console.conditions import Context, check_conditions
from openpilot.sunnypilot.selfdrive.test_console.registry import Kind, TestSpec
from openpilot.sunnypilot.selfdrive.test_console.signals import Sample, sample_row
from openpilot.sunnypilot.selfdrive.test_console.status_io import MARKERS_FILE, RESULTS_FILE, STATE_FILE, read_json, \
  sanitize_results, write_json_atomic

MARKER_MAX_BYTES = 1024 * 1024
MARKER_BACKUPS = 5
CAPTURE_FILE_MAX = 20 * 1024 * 1024   # bytes per capture file; rows past it are dropped (marker says so)
CAPTURE_DIR_MAX = 300 * 1024 * 1024   # oldest capture files are pruned past this
RESULTS_SAVE_PERIOD = 5.0             # s, progress survives a crash/offroad mid-capture
PRUNE_PERIOD = 60.                    # s: the capture-directory cap is enforced on this period too, not only when a file opens (D-21)
KEY_SIGNALS = ("v_ego", "a_ego", "gear_raw", "gear", "rpm", "enabled", "long_active", "accel_cmd", "v_cruise", "pitch")


def _num(x, nd: int):
  """A finite number rounded for the status file, else None (NaN / inf from a dead signal must not crash status())."""
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  return round(x, nd) if math.isfinite(x) else None


class Store:
  def __init__(self, root: str):
    self.root = root
    self.capture_dir = os.path.join(root, "captures")
    self._markers: logging.Logger | None = None
    self.failed = False
    try:
      os.makedirs(self.capture_dir, exist_ok=True)
    except OSError:
      cloudlog.exception("test_console: can't create its directory")
      self.failed = True
    self.results: dict = sanitize_results(read_json(os.path.join(root, RESULTS_FILE)))

  def _marker_logger(self) -> logging.Logger | None:
    if self._markers is None and not self.failed:
      try:
        handler = RotatingFileHandler(os.path.join(self.root, MARKERS_FILE), maxBytes=MARKER_MAX_BYTES, backupCount=MARKER_BACKUPS)
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger = logging.getLogger(f"test_console.markers.{id(self)}")
        logger.propagate = False
        logger.setLevel(logging.INFO)
        logger.addHandler(handler)
        self._markers = logger
      except OSError:
        cloudlog.exception("test_console: can't open markers file")
        self.failed = True
    return self._markers

  def marker(self, record: dict) -> None:
    cloudlog.event("test_marker", **record)
    logger = self._marker_logger()
    if logger is not None:
      try:
        logger.info(json.dumps(record, separators=(",", ":")))
      except (OSError, ValueError):
        cloudlog.exception("test_console: marker write failed")

  def prune(self) -> None:
    try:
      files = [os.path.join(self.capture_dir, f) for f in os.listdir(self.capture_dir)]
      files = sorted((os.path.getmtime(f), os.path.getsize(f), f) for f in files if os.path.isfile(f))
      total = sum(sz for _, sz, _ in files)
      for _, sz, f in files:
        if total <= CAPTURE_DIR_MAX:
          break
        os.remove(f)
        total -= sz
    except OSError:
      cloudlog.exception("test_console: prune failed")

  def save_result(self, test_id: str, entry: dict) -> None:
    self.results[test_id] = entry
    if self.failed:
      return
    try:
      write_json_atomic(os.path.join(self.root, RESULTS_FILE), self.results)
    except OSError:
      cloudlog.exception("test_console: results write failed")

  def save_state(self, state: dict) -> None:
    if self.failed:
      return
    try:
      write_json_atomic(os.path.join(self.root, STATE_FILE), state)
    except OSError:
      pass


class CaptureFile:
  """Qualifying samples of one capture window: a header line, then one JSON row per sample at the test's rate."""

  def __init__(self, store: Store, spec: TestSpec, run: int, window: int, background: bool, ctx: Context):
    self.spec = spec
    self.period = 1. / spec.sample_hz
    self.t_last = -math.inf
    self.rows = 0
    self.dropped = 0
    self.bytes = 0
    self.f = None
    if store.failed:
      return
    store.prune()
    stamp = time.strftime("%Y%m%d-%H%M%S")
    self.path = os.path.join(store.capture_dir, f"{stamp}_{spec.id}_r{run}w{window}{'_bg' if background else ''}.jsonl")
    try:
      self.f = open(self.path, "w")
      self._write({"test": spec.id, "phase": spec.phase, "run": run, "window": window, "background": background,
                   "fields": [ctx.column(f) for f in spec.fields], "sample_hz": spec.sample_hz, "wall": time.time(), "mono": time.monotonic()})  # noqa: TID251  (human-readable wall clock in the files)
    except OSError:
      cloudlog.exception("test_console: can't open capture file")
      self.f = None

  def _write(self, obj) -> None:
    line = json.dumps(obj, separators=(",", ":")) + "\n"
    if self.bytes + len(line) > CAPTURE_FILE_MAX:
      self.dropped += 1
      return
    self.f.write(line)
    self.bytes += len(line)

  def add(self, s: Sample) -> None:
    if self.f is None or s.t - self.t_last < self.period - 1e-6:
      return
    self.t_last = s.t
    try:
      self._write(sample_row(s, self.spec.fields))
      self.rows += 1
    except (OSError, ValueError):
      self.f = None

  def close(self) -> None:
    if self.f is not None:
      try:
        self.f.close()
      except OSError:
        pass
      self.f = None


class Run:
  """One armed test (or one background telemetry test): analyzer + capture windows."""

  def __init__(self, store: Store, spec: TestSpec, ctx: Context, run: int, background: bool = False):
    self.store, self.spec, self.ctx = store, spec, ctx
    self.run, self.background = run, background
    self.analyzer: Analyzer = make_analyzer(spec, ctx)
    self.window = 0
    self.file: CaptureFile | None = None
    self.t_start = 0.
    self.complete = False
    self.dropped = 0   # frames lost to errors the daemon's guard caught while this run was active (Task 13 B3)

  def open_window(self, t: float) -> None:
    self.window += 1
    self.t_start = t
    self.file = CaptureFile(self.store, self.spec, self.run, self.window, self.background, self.ctx)

  def close_window(self) -> list[str]:
    notes = self.analyzer.finish() if hasattr(self.analyzer, "finish") else []
    if self.file is not None:
      self.file.close()
      self.file = None
    return notes

  def feed(self, s: Sample, ok: bool) -> list[str]:
    notes = self.analyzer.feed(s, ok)
    if ok and self.file is not None:
      self.file.add(s)
    return notes

  def result(self) -> tuple[str | None, str]:
    """The analyzer's verdict, except (Task 13 B3) a run that lost frames to caught console errors never PASSes: its
    data is partial, so a PASS is only provisional (CAPTURED); any verdict says how many frames were lost."""
    status, detail = self.analyzer.result()
    if self.dropped:
      if status == PASS:
        status = CAPTURED
      detail += f" [provisional: {self.dropped} frame(s) lost to console errors]"
    return status, detail

  def summary(self) -> dict:
    text, frac = self.analyzer.progress()
    status, detail = self.result()
    return {**self.analyzer.data, "status": status, "detail": detail, "progress": text, "frac": round(frac, 3),
            "complete": self.complete, "wall": time.time(), "phase": self.spec.phase,  # noqa: TID251  (human-readable wall clock in the files)
            "background": self.background, "dropped": self.dropped}


class Console:
  def __init__(self, store: Store, ctx: Context):
    self.store, self.ctx = store, ctx
    ctx.results = store.results   # the same dict: rules see calibration results as soon as they are saved (D3)
    self.registry = ctx.profile.registry
    self.selected: str = ""
    self.armed = False
    self.state = "idle"
    self.run: Run | None = None
    self.runs = 0
    self.conditions: list[tuple[str, bool]] = []
    self.qualifying = False
    self.background = False
    self.bg_runs: dict[str, Run] = {}
    self._t_saved = 0.
    self._t_pruned = 0.
    self._last_t = 0.

  # -- markers ---------------------------------------------------------------------------------------------------
  def _marker(self, action: str, run: Run | None, s: Sample | None = None, note: str = "") -> None:
    spec = run.spec if run is not None else self.registry.tests_by_id.get(self.selected)
    rec = {"test": spec.id if spec else self.selected, "phase": spec.phase if spec else None, "action": action,
           "mono": round(s.t if s is not None else self._last_t, 3), "wall": round(time.time(), 3)}  # noqa: TID251  (human-readable wall clock in the files)
    if run is not None:
      rec.update(run=run.run, window=run.window, background=run.background)
      text, _ = run.analyzer.progress()
      rec["progress"] = text
      if action in ("complete", "stop", "abort"):
        rec["status"], rec["detail"] = run.result()
    if note:
      rec["note"] = note
    if s is not None:
      rec["signals"] = dict(zip((self.ctx.column(k) for k in KEY_SIGNALS), sample_row(s, KEY_SIGNALS), strict=True))
    self.store.marker(rec)

  def _save(self, run: Run) -> None:
    self.store.save_result(run.spec.id, run.summary())

  # -- control (from Params, ~1 Hz) ---------------------------------------------------------------------------------
  def set_control(self, selected: str, armed: bool, background: bool) -> None:
    tests = self.registry.tests_by_id
    selected = selected if selected in tests else ""
    armed = armed and bool(selected)
    if (selected != self.selected or not armed) and self.armed:
      self._disarm()
    self.selected = selected
    if armed and not self.armed:
      bg = self.bg_runs.pop(selected, None)
      if bg is not None:
        bg.close_window()  # the armed run owns this test's result now
      self.armed = True
      self.state = "armed"
      self.runs += 1
      self.run = Run(self.store, tests[selected], self.ctx, self.runs)
      self._marker("arm", self.run)
    if background != self.background:
      self.background = background
      if not background:
        self._stop_background()

  def _disarm(self) -> None:
    if self.run is not None:
      if self.state == "capturing":
        self._close(self.run)
        self._marker("abort", self.run, note="disarmed or test changed")
        self._save(self.run)
      self._marker("disarm", self.run)
    self.armed, self.state, self.run = False, "idle", None

  def _close(self, run: Run) -> None:
    for note in run.close_window():
      self._marker("sample", run, note=note)

  def shutdown(self) -> None:
    if self.run is not None and self.state == "capturing":
      self._close(self.run)
      self._marker("abort", self.run, note="daemon exit (offroad)")
      self._save(self.run)
    self._stop_background()

  # -- background telemetry ------------------------------------------------------------------------------------------
  def _stop_background(self) -> None:
    for run in self.bg_runs.values():
      self._close(run)
      self._save(run)
    self.bg_runs.clear()

  def _update_background(self, s: Sample) -> None:
    for spec in self.registry.tests:
      if spec.kind != Kind.TELEMETRY or not spec.background or (self.armed and spec.id == self.selected):
        continue
      run = self.bg_runs.get(spec.id)
      if run is None:
        if self.store.results.get(spec.id, {}).get("complete"):
          continue  # already have it; arm it explicitly to re-run
        run = self.bg_runs[spec.id] = Run(self.store, spec, self.ctx, 0, background=True)
        run.open_window(s.t)
      if run.complete:
        continue
      ok = all(v for _, v in check_conditions(spec.conditions, spec.toggles, s, self.ctx))
      for note in run.feed(s, ok):
        self._marker("sample", run, s, note)
      if run.analyzer.complete:
        run.complete = True
        self._close(run)
        self._marker("complete", run, s)
        self._save(run)

  # -- per frame ----------------------------------------------------------------------------------------------------
  def update(self, s: Sample, button: bool) -> None:
    self._last_t = s.t
    if s.t - self._t_pruned > PRUNE_PERIOD and (self.state == "capturing" or self.bg_runs):
      self._t_pruned = s.t
      self.store.prune()
    spec = self.registry.tests_by_id.get(self.selected)
    if spec is not None:
      self.conditions = check_conditions(spec.conditions, spec.toggles, s, self.ctx)
      self.qualifying = all(v for _, v in self.conditions)
    else:
      self.conditions, self.qualifying = [], False

    run = self.run
    if self.armed and run is not None:
      if button:
        if self.state in ("armed", "stopped"):
          self.state = "capturing"
          run.open_window(s.t)
          self._marker("start", run, s)
        elif self.state == "capturing":
          self.state = "stopped"
          self._close(run)
          self._marker("stop", run, s)
          self._save(run)
        elif self.state == "complete":
          self.runs += 1
          run = self.run = Run(self.store, spec, self.ctx, self.runs)
          self.state = "capturing"
          run.open_window(s.t)
          self._marker("start", run, s, note="re-run")
      if self.state == "capturing":
        for note in run.feed(s, self.qualifying):
          self._marker("sample", run, s, note)
        if run.analyzer.complete:
          run.complete = True
          self.state = "complete"
          self._close(run)
          self._marker("complete", run, s)
          self._save(run)
        elif s.t - self._t_saved > RESULTS_SAVE_PERIOD:
          self._t_saved = s.t
          self._save(run)

    if self.background:
      self._update_background(s)

  def note_dropped(self) -> None:
    """The daemon's guard caught an error that lost this frame (Task 13 B3): every active run's data is now partial."""
    if self.run is not None and self.state == "capturing":
      self.run.dropped += 1
    for r in self.bg_runs.values():
      if not r.complete:
        r.dropped += 1

  # -- status for the UI -----------------------------------------------------------------------------------------------
  def status(self, s: Sample | None, button_info: dict) -> dict:
    # stamp: CLOCK_MONOTONIC is system-wide, so the UI process can judge staleness against its own time.monotonic()
    out = {"version": 1, "mono": round(self._last_t, 3), "stamp": round(time.monotonic(), 3), "selected": self.selected,
           "armed": self.armed, "state": self.state, "conditions": self.conditions, "qualifying": self.qualifying,
           "background": self.background, "button": button_info, "progress": "", "frac": 0., "status": None, "detail": "",
           "window": 0, "elapsed": 0.}
    if self.run is not None:
      summary = self.run.summary()
      out.update(progress=summary["progress"], frac=summary["frac"], status=summary["status"], detail=summary["detail"],
                 window=self.run.window, elapsed=round(self._last_t - self.run.t_start, 1) if self.state == "capturing" else 0.,
                 dropped=self.run.dropped)
    if s is not None:
      out["live"] = {"v": _num(s.v_ego, 2), "rpm": _num(s.rpm, 0), self.ctx.column("gear_raw"): s.gear_raw, "gear": s.gear,
                     "pitch": _num(s.pitch, 4), "gate": s.gate, "long_active": s.long_active,
                     "accel_cmd": _num(s.accel_cmd, 2)}
    out["bg"] = {k: r.analyzer.progress()[0] for k, r in self.bg_runs.items() if not r.complete}
    return out


__all__ = ["Console", "Store", "PASS", "FAIL", "CAPTURED"]
