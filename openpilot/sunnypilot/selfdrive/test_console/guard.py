"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test console failure containment (Task 12 D1): console code must never take down the process it runs in — not
test_consoled (selfdrived also ignores it), and above all not the ui process. An exception is logged ONCE per site
(cloudlog.exception, so the traceback is in the rlog) and swallowed; the caller keeps running (or goes inert).
"""
import functools

from openpilot.common.swaglog import cloudlog

_logged: set[str] = set()


def log_once(site: str, msg: str = "") -> None:
  if site not in _logged:
    _logged.add(site)
    cloudlog.exception(f"test console: {site} failed{' - ' + msg if msg else ''} (logged once; kept running)")


def guarded(fn=None, *, default=None):
  """Decorator: run fn, log the first exception per function, return `default` instead of raising."""
  def wrap(f):
    site = f.__qualname__

    @functools.wraps(f)
    def wrapper(*a, **kw):
      try:
        return f(*a, **kw)
      except Exception:
        log_once(site)
        return default
    return wrapper
  return wrap(fn) if fn is not None else wrap


class Guard:
  """The daemon's per-frame guard: counts failures; after MAX_CONSECUTIVE in a row the console goes inert."""
  MAX_CONSECUTIVE = 100

  def __init__(self):
    self.errors = 0
    self.consecutive = 0
    self.inert = False

  def run(self, site: str, fn, *a, force: bool = False, **kw):
    """force: run even when inert (the status writer: it must keep saying the console is inert, Task 13 B3)."""
    if self.inert and not force:
      return None
    try:
      out = fn(*a, **kw)
      self.consecutive = 0
      return out
    except Exception:
      self.errors += 1
      self.consecutive += 1
      log_once(site)
      if self.consecutive >= self.MAX_CONSECUTIVE:
        self.inert = True
        cloudlog.error(f"test console: {self.consecutive} consecutive failures in {site} - going inert for this drive")
      return None
