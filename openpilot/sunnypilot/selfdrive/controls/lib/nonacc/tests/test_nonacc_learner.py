"""
fork-only (civic-cruise) Phase 3: the engine-brake learner.

Learns per-gear closed-throttle decel from brake-free coasts, bounded (never below static, never more than MAX_LEARN
above it, conservative lower bound), applied live via effective_decel; gates out anything that could contain the brakes.
"""
import json

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import learner as L
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc.engine_brake import STATIC_DECEL, table_decel

DT = 0.05
GEAR, V = 4, 17.0   # static D(4, 17) = 0.45


def _fresh():
  Params().remove(L.PARAM)
  return L.EngineBrakeLearner(True, DT, Params())


def _feed(lr, n, **kw):
  frame = {"engaged": False, "gas": False, "brake_switch": False, "user_brake": 0.0, "gear": GEAR,
               "torque_axle": -100.0, "v_ego": V, "a_ego": -0.6, "pitch": 0.0}   # brake-free coast, flat, D = 0.6
  frame.update(kw)
  for _ in range(n):
    lr.observe(**frame)


class TestLearner:
  def test_learns_above_static_bounded(self):
    lr = _fresh()
    _feed(lr, 220)
    b = L.v_bin(V)
    row = lr.learned.get(GEAR)
    assert row is not None, "a well-sampled cell should produce a learned row"
    static = STATIC_DECEL[GEAR][b]
    assert row[b] > static, f"learned {row[b]:.3f} should exceed static {static:.3f} (measured 0.6)"
    assert row[b] <= static + L.MAX_LEARN + 1e-9, "bounded above static by MAX_LEARN"
    assert all(row[i] >= STATIC_DECEL[GEAR][i] - 1e-9 for i in range(len(row))), "never below static"

  def test_gates_out_non_coast_frames(self):
    for kw in ({"engaged": True}, {"gas": True}, {"brake_switch": True}, {"user_brake": 0.5},
               {"gear": None}, {"torque_axle": 50.0}, {"pitch": None}):
      lr = _fresh()
      _feed(lr, 220, **kw)
      assert lr.learned == {}, f"should not learn from {kw}"

  def test_min_samples(self):
    lr = _fresh()
    _feed(lr, 25)   # too few after the settle window
    assert lr.learned == {}

  def test_effective_decel_uses_learned_else_static(self):
    assert abs(L.effective_decel(GEAR, V, {}) - table_decel(GEAR, V)) < 1e-9
    b = L.v_bin(V)
    learned = {GEAR: list(STATIC_DECEL[GEAR])}
    learned[GEAR][b] = 0.6
    assert abs(L.effective_decel(GEAR, V, learned) - 0.6) < 1e-6

  def test_param_round_trip(self):
    Params().put(L.PARAM, json.dumps({"applied": {"4": list(STATIC_DECEL[4])}}), block=True)
    got = L.load_applied(Params())
    assert 4 in got and len(got[4]) == len(STATIC_DECEL[4])
