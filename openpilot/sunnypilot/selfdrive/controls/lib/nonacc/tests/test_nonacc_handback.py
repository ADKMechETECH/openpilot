"""
fork-only (civic-cruise) Phase 5: manual low-speed handback. Ramps the accel range to 0 below the (gear-aware) handback
speed, then signals disengage; refuses to engage below the speed; aborts on force_decel; never brakes to a stop / holds.
"""
from openpilot.common.params import Params
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import handback as H

DT = 0.01


def _hb():
  Params().remove("ManualLowSpeedHandbackSpeed")   # use the registered defaults (2.24 / 0.5)
  Params().remove("ManualLowSpeedHandbackRamp")
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)   # manual
  return H.LowSpeedHandback(CP, DT)


class TestHandback:
  def test_gear_aware_threshold(self):
    hb = _hb()
    hb.update(True, 10.0, 0.0, 2, False)   # 2nd gear: threshold raised to ~8.7 mph
    assert abs(hb.threshold - H.GEAR_HANDBACK_SPEED[2]) < 1e-6
    hb2 = _hb()
    hb2.update(True, 10.0, 0.0, 4, False)  # 4th: the setting (5 mph)
    assert abs(hb2.threshold - H.DEFAULT_HANDBACK_SPEED) < 1e-6

  def test_latches_and_ramps_then_disengages(self):
    hb = _hb()
    hb.update(True, 1.5, -0.4, 0, False)   # below 2.24 m/s -> latch
    assert hb.active
    # before the ramp completes: not disengaging, throttle+brake ramping toward 0
    lo, hi = hb.limit_accel(-3.5, 1.0)
    assert lo <= 0.0 <= hi + 1e-9
    for _ in range(int((H.DEFAULT_RAMP_TIME) / DT) + 1):
      hb.update(True, 1.5, -0.4, 0, False)
    assert hb.ramp_done
    lo, hi = hb.limit_accel(-3.5, 1.0)
    assert abs(lo) < 1e-6 and abs(hi) < 1e-6    # command forced to 0
    for _ in range(int(H.DISENGAGE_MARGIN / DT) + 1):
      hb.update(True, 1.5, -0.4, 0, False)
    assert hb.disengage_due

  def test_not_enabled_resets(self):
    hb = _hb()
    hb.update(True, 1.5, -0.4, 0, False)
    assert hb.active
    hb.update(False, 1.5, -0.4, 0, False)
    assert not hb.active
    assert hb.limit_accel(-3.5, 1.0) == (-3.5, 1.0)

  def test_force_decel_aborts(self):
    hb = _hb()
    hb.update(True, 1.5, -0.4, 0, False)
    assert hb.active
    hb.update(True, 1.5, -0.4, 0, True)    # force_decel -> abort (stock brings to stop)
    assert not hb.active

  def test_below_engage_speed_refuses(self):
    hb = _hb()
    assert hb.below_engage_speed(1.0)                      # below threshold -> refuse engage
    assert not hb.below_engage_speed(2.24 + 0.5 + 0.1)     # above threshold + hysteresis -> ok

  def test_no_latch_above_threshold(self):
    hb = _hb()
    hb.update(True, 5.0, 0.5, 0, False)    # well above threshold
    assert not hb.active
    assert hb.limit_accel(-3.5, 1.0) == (-3.5, 1.0)

  def test_params_read_at_1hz_not_every_frame(self):
    # session 12 (2026-11-05): the two params were re-read every 100 Hz control frame (200 Params reads/s in
    # controlsd, on the saturated control core). They must be read on the first frame, then once per second.
    class CountingParams:
      def __init__(self):
        self.gets = 0
      def get(self, key, return_default=False):
        self.gets += 1
        return None
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    p = CountingParams()
    hb = H.LowSpeedHandback(CP, DT, params=p)
    hb.update(False, 20.0, 0.0, 4, False)
    assert p.gets == 2                      # first frame reads both
    for _ in range(int(1.0 / DT) - 1):
      hb.update(False, 20.0, 0.0, 4, False)
    assert p.gets == 2                      # nothing more inside the first second
    hb.update(False, 20.0, 0.0, 4, False)
    assert p.gets == 4                      # re-read at 1 Hz
    hb.update(False, 20.0, 0.0, 2, False)
    assert abs(hb.threshold - H.GEAR_HANDBACK_SPEED[2]) < 1e-6   # the gear still tracks every frame
