"""
fork-only (civic-cruise): ACC-mode engine-brake priority - when the lead plan's braking may pass the coast floor
(owner 2026-11-06: inside the approach-warning margin; "stopped car should always cause brakes"; plus hard braking).
"""
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import acc_coast as AC
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import approach as A
from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import controller as C

DT = 0.05


def _lead(d, vl, present=True, prob=0.99):
  return [(present, prob, d, vl), (False, 0.0, 0.0, 0.0)]


def _run(g, secs, v, d, vl, need=-0.5, **kw):
  out = False
  for _ in range(int(round(secs / DT))):
    out = g.update(v, _lead(d, vl, **kw), need)
  return out


class TestLeadBrakeGate:
  def test_far_lead_is_engine_braking_only(self):
    g = AC.LeadBrakeGate(DT)
    v, vl = 30.0, 25.0
    assert not _run(g, 5.0, v, A.acc_closest_gap(v, vl), vl, need=-0.8)      # at the normal follow distance: coast
    assert not _run(g, 5.0, v, 0.6 * A.acc_closest_gap(v, vl), vl, need=-1.2)  # still outside the margin
    assert g.reason == ""

  def test_inside_the_margin_allows_braking_after_a_debounce(self):
    g = AC.LeadBrakeGate(DT)
    v, vl = 30.0, 25.0
    d = 0.5 * A.acc_closest_gap(v, vl)
    assert not _run(g, AC.MARGIN_ON_TIME - DT, v, d, vl)
    assert _run(g, 2 * DT, v, d, vl) and g.reason == "margin"

  def test_margin_is_the_approach_warning_distance(self):
    v, vl = 27.0, 24.0
    edge = A.FRACTION * A.acc_closest_gap(v, vl)
    assert _run(AC.LeadBrakeGate(DT), 1.0, v, edge - 0.5, vl)
    assert not _run(AC.LeadBrakeGate(DT), 1.0, v, edge + 0.5, vl)

  def test_stopped_car_always_brakes(self):
    # owner: "stopped car should always cause brakes" - at any distance, with no debounce
    g = AC.LeadBrakeGate(DT)
    assert g.update(30.0, _lead(250.0, 0.0), -0.1) and g.reason == "stopped"
    assert AC.LeadBrakeGate(DT).update(20.0, _lead(150.0, AC.STOPPED_SPEED - 0.1), -0.1)
    assert not AC.LeadBrakeGate(DT).update(20.0, _lead(150.0, AC.STOPPED_SPEED + 0.5), -0.1)

  def test_hard_braking_need_passes_at_once_and_latches(self):
    g = AC.LeadBrakeGate(DT)
    v, vl = 30.0, 18.0
    far = A.acc_closest_gap(v, vl)
    assert not g.update(v, _lead(far, vl), -1.9)
    assert g.update(v, _lead(far, vl), AC.HARD_BRAKE) and g.reason == "hard"
    assert g.update(v, _lead(far, vl), -1.5)                         # stays on while the plan still needs real braking
    assert _run(g, AC.HOLD_TIME - 2 * DT, v, far, vl, need=-0.5)     # ... and for HOLD_TIME after it no longer does
    assert not _run(g, 4 * DT, v, far, vl, need=-0.5)

  def test_doubtful_or_absent_lead_gives_no_permission(self):
    v = 30.0
    assert not _run(AC.LeadBrakeGate(DT), 2.0, v, 10.0, 25.0, prob=0.3)
    assert not _run(AC.LeadBrakeGate(DT), 2.0, v, 10.0, 25.0, present=False)

  def test_second_lead_counts(self):
    g = AC.LeadBrakeGate(DT)
    for _ in range(10):
      ok = g.update(30.0, [(True, 0.99, 120.0, 29.0), (True, 0.99, 15.0, 27.0)], -0.3)
    assert ok and g.reason == "margin"

  def test_hold_bridges_a_flickering_lead_distance(self):
    g = AC.LeadBrakeGate(DT)
    v, vl = 30.0, 25.0
    inside_d, outside_d = 0.4 * A.acc_closest_gap(v, vl), 0.9 * A.acc_closest_gap(v, vl)
    _run(g, 1.0, v, inside_d, vl)
    seen = [g.update(v, _lead(outside_d if k % 6 < 3 else inside_d, vl), -0.5) for k in range(60)]
    assert all(seen)

  def test_thresholds_agree_with_the_controller(self):
    assert AC.HARD_BRAKE == C.ACC_HARD_BRAKE


class TestControllerRelease:
  """How the allowed braking reaches the car through the controller's floor."""

  def _ctrl(self):
    from opendbc.car.honda.values import CAR
    from opendbc.car.honda.interface import CarInterface
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP.openpilotLongitudinalControl = True
    return C.NonAccController(CP, 0.01)

  def test_engine_braking_only_while_the_bound_is_high(self):
    # a lead limits the plan (plan_is_cruise False): the low-speed hold may not drive the speed up
    c, last = self._ctrl(), 0.0
    for _ in range(400):
      last = max(-1.5, c.update(True, 25.0, 25.0, 5, 0.0, False, -3.5, False, last_accel=last, lead_accel_limit=2.0, plan_is_cruise=False))
    assert -0.35 < last < -0.05                       # the plan wants -1.5 for the lead: held at the coast floor

  def test_allowed_braking_ramps_in(self):
    c, last = self._ctrl(), 0.0
    for _ in range(400):
      last = max(-1.5, c.update(True, 25.0, 25.0, 5, 0.0, False, -3.5, False, last_accel=last, lead_accel_limit=2.0))
    out = []
    for _ in range(100):
      last = max(-1.5, c.update(True, 25.0, 25.0, 5, 0.0, False, -3.5, False, last_accel=last, lead_accel_limit=-1.5))
      out.append(last)
    assert min(b - a for a, b in zip(out, out[1:], strict=False)) >= -C.ACC_RELEASE_JERK * 0.01 - 1e-9
    assert out[0] > -0.4 and abs(out[-1] + 1.5) < 1e-6

  def test_hard_braking_is_not_ramped(self):
    c, last = self._ctrl(), 0.0
    for _ in range(400):
      last = max(-3.0, c.update(True, 25.0, 25.0, 5, 0.0, False, -3.5, False, last_accel=last, lead_accel_limit=2.0))
    assert c.update(True, 25.0, 25.0, 5, 0.0, False, -3.5, False, last_accel=last, lead_accel_limit=-3.0) == -3.0
