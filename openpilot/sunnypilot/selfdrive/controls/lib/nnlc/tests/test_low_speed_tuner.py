"""
fork-only (civic-cruise): the NNLC low-speed hunt self-tuner (session 15, owner "build it"). Default OFF; backs the
low-speed curvature term off while the wheel hunts at ~2 Hz under 25 mph, creeps back when calm, persists only parked.
"""
import math

import numpy as np

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc import low_speed_tuner as T

DT = 0.01


def _tuner(enabled=True, scale=None):
  p = Params()
  p.put_bool(T.PARAM_TOGGLE, enabled, block=True)
  p.put(T.PARAM_SCALE, 1.0 if scale is None else float(scale), block=True)
  return T.LowSpeedHuntTuner(DT)


def _run(t, secs, v=4.0, hands_off=True, dcurv=0.0, amp_deg=1.5, amp_req=0.3, f=2.0, offset_deg=0.0, phase=0.0):
  n = int(secs / DT)
  for k in range(n):
    th = 2 * math.pi * f * k * DT + phase
    t.update(v, hands_off, dcurv, offset_deg + amp_deg * math.sin(th), amp_req * math.sin(th))


class TestDetector:
  def test_hunting_wheel_is_seen_and_calm_is_not(self):
    t = _tuner()
    _run(t, 3.0)                                   # 1.5 deg, 0.3 torque at 2 Hz, straight, 9 mph
    assert t.hunting
    assert t.angle_band.update(0.0) > T.ANGLE_BAND
    t = _tuner()
    _run(t, 3.0, amp_deg=0.15, amp_req=0.03)       # the calm runs' level
    assert not t.hunting

  def test_frequency_selective(self):
    t = _tuner()
    _run(t, 4.0, f=0.4)                            # a slow wander, same amplitude: not the hunt
    assert not t.hunting
    t = _tuner()
    _run(t, 4.0, f=6.0)
    assert not t.hunting

  def test_regime_gates(self):
    for kw in ({"v": 15.0}, {"v": 0.5}, {"hands_off": False}, {"dcurv": 0.02}, {"offset_deg": 30.0}):
      t = _tuner()
      _run(t, 3.0, **kw)
      assert not t.hunting, kw


class TestAdaptation:
  def test_backs_off_in_steps_with_a_cooldown(self):
    t = _tuner()
    _run(t, T.ON_TIME + 1.0)                       # the 1 s RMS needs ~0.7 s to see it, then ON_TIME
    assert t.steps_down == 1 and abs(t.scale - T.STEP_DOWN) < 1e-9
    _run(t, T.COOLDOWN - 0.5)
    assert t.steps_down == 1                       # still cooling down
    _run(t, 0.6 + T.ON_TIME)
    assert t.steps_down == 2
    assert abs(t.factor_scale - T.STEP_DOWN ** 2) < 1e-9

  def test_floor(self):
    t = _tuner()
    _run(t, 120.0)
    assert t.scale == T.SCALE_MIN and t.factor_scale == T.SCALE_MIN

  def test_creeps_back_when_calm(self):
    t = _tuner(scale=0.6)
    assert abs(t.scale - 0.6) < 1e-9
    _run(t, T.CREEP_TIME + 1.0, amp_deg=0.1, amp_req=0.02)
    assert abs(t.scale - 0.6 * T.CREEP_UP) < 1e-9
    t = _tuner(scale=0.999)
    _run(t, T.CREEP_TIME + 1.0, amp_deg=0.1, amp_req=0.02)
    assert t.scale == T.SCALE_MAX                   # never above stock

  def test_no_creep_outside_the_regime(self):
    t = _tuner(scale=0.6)
    _run(t, T.CREEP_TIME + 1.0, amp_deg=0.1, amp_req=0.02, v=15.0)
    assert abs(t.scale - 0.6) < 1e-9

  def test_off_means_stock(self):
    t = _tuner(enabled=False, scale=0.5)
    _run(t, 5.0)
    assert t.hunting                               # it still watches
    assert t.factor_scale == 1.0 and abs(t.scale - 0.5) < 1e-9 and t.steps_down == 0


class TestPersistence:
  def test_written_once_per_stop_only_when_parked_and_moved(self):
    t = _tuner()
    _run(t, 3.0)
    assert t.scale < 1.0
    p = Params()
    t.persist(standstill=False, enabled=True)
    assert float(p.get(T.PARAM_SCALE, return_default=True)) == 1.0      # driving: never
    t.persist(standstill=True, enabled=True)
    assert float(p.get(T.PARAM_SCALE, return_default=True)) == 1.0      # stopped but engaged: no
    t.persist(standstill=True, enabled=False)
    assert abs(float(p.get(T.PARAM_SCALE, return_default=True)) - t.scale) < 1e-6
    _run(t, 3.0)
    before = float(p.get(T.PARAM_SCALE, return_default=True))
    t.persist(standstill=True, enabled=False)                            # same stop: no second write
    assert float(p.get(T.PARAM_SCALE, return_default=True)) == before
    t.persist(standstill=False, enabled=False)
    t.persist(standstill=True, enabled=False)                            # next stop: written
    assert abs(float(p.get(T.PARAM_SCALE, return_default=True)) - t.scale) < 1e-6

  def test_reads_the_persisted_scale_and_clamps(self):
    assert abs(_tuner(scale=0.7).scale - 0.7) < 1e-9
    assert _tuner(scale=0.05).scale == T.SCALE_MIN
    assert _tuner(scale=3.0).scale == T.SCALE_MAX
    Params().put(T.PARAM_SCALE, 1.0, block=True)
    Params().put_bool(T.PARAM_TOGGLE, False, block=True)


class TestWiring:
  def test_nnlc_scales_its_low_speed_term(self):
    import inspect
    from openpilot.sunnypilot.selfdrive.controls.lib.nnlc import nnlc
    src = inspect.getsource(nnlc.NeuralNetworkLateralControl.update_neural_network_feedforward)
    assert "self.low_speed_tuner.factor_scale" in src and "LOW_SPEED_Y" in src

  def test_defaults_off(self):
    assert Params().get_bool(T.PARAM_TOGGLE) is False
    assert float(Params().get(T.PARAM_SCALE, return_default=True)) == 1.0

  def test_band_filter_is_a_2hz_bandpass(self):
    b = T.Biquad(2.0, 1.4, 100.0)
    out = [abs(b.update(math.sin(2 * math.pi * 2.0 * k / 100))) for k in range(400)]
    g2 = max(out[200:])
    b = T.Biquad(2.0, 1.4, 100.0)
    out = [abs(b.update(math.sin(2 * math.pi * 0.4 * k / 100))) for k in range(1000)]
    g04 = max(out[500:])
    assert 0.9 < g2 < 1.1 and g04 < 0.35
    assert np.isfinite(g2)
