"""fork-only diagnostic (civic-cruise, session 12): hardwared's StallTrace must dump the blocked stack when the loop
overruns, stay silent when it doesn't, and never raise (hardwared must not die because of a diagnostic)."""
import faulthandler
import time

import pytest

from openpilot.system.hardware.hardwared import StallTrace


@pytest.fixture(autouse=True)
def _disarm():
  yield
  faulthandler.cancel_dump_traceback_later()   # don't leave a watchdog armed for the rest of the test run


def _blocked_here(seconds):
  time.sleep(seconds)


def test_overrun_dumps_stack_and_duration(tmp_path):
  log = tmp_path / "stall.log"
  st = StallTrace(path=str(log), limit=0.2, enabled=True)
  st.beat()
  _blocked_here(0.5)
  st.beat()
  st.file.flush()
  text = log.read_text()
  assert "_blocked_here" in text            # faulthandler's dump names where the loop was stuck
  assert "hardwared loop took 0." in text   # and the loop reports how long it was gone


def test_quiet_when_loop_is_on_time(tmp_path):
  log = tmp_path / "stall.log"
  st = StallTrace(path=str(log), limit=0.5, enabled=True)
  for _ in range(5):
    st.beat()
    time.sleep(0.02)
  st.beat()
  assert log.read_text() == ""


def test_noop_when_disabled_or_unwritable(tmp_path):
  StallTrace(path=str(tmp_path / "x.log"), enabled=False).beat()              # off-device: nothing opened
  assert not (tmp_path / "x.log").exists()
  StallTrace(path=str(tmp_path / "no_such_dir" / "x.log"), enabled=True).beat()   # can't open: still no raise
