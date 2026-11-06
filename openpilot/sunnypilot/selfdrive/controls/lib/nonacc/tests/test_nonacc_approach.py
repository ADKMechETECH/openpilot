"""
fork-only (civic-cruise): the non-ACC approach warning (session 14, owner 2026-11-06). Warning only - these tests pin the
trigger (inside half of ACC's closest gap AND closing), the debounce / hold / re-arm, and that it is wired as a
warning alert behind a default-ON toggle.
"""
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.nonacc import approach as A

DT = 0.05
MPH = 0.44704


def _run(w, secs, active=True, v=24.0, present=True, d=20.0, v_lead=22.0, prob=0.99):
  out = []
  for _ in range(int(round(secs / DT))):
    out.append(w.update(active, v, present, d, v_lead, v_lead - v, prob))
  return out


class TestTrigger:
  def test_gap_formula_matches_acc_closest_setting(self):
    # 54 mph behind a car doing 50: ACC (closest setting) would hold ~52 m; half of that is the trigger
    v, vl = 54 * MPH, 50 * MPH
    assert A.acc_closest_gap(v, vl) == pytest.approx((v * v - vl * vl) / 5.0 + 1.25 * v + 6.0)
    assert 50.0 < A.acc_closest_gap(v, vl) < 54.0

  def test_constants_are_long_mpc_values(self):
    lm = pytest.importorskip("openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc")
    from openpilot.cereal import log
    assert A.COMFORT_BRAKE == lm.COMFORT_BRAKE and A.STOP_DISTANCE == lm.STOP_DISTANCE
    assert A.T_FOLLOW_MIN == min(lm.get_T_FOLLOW(p) for p in (log.LongitudinalPersonality.relaxed,
                                                             log.LongitudinalPersonality.standard,
                                                             log.LongitudinalPersonality.aggressive))

  def test_session14_cases(self):
    # route 00000056, 642.9 s: 23 m at 54 mph closing 3.8 mph -> warn. 949 s: 23 m at 49 mph closing 3.6 -> warn.
    assert A.too_close(54 * MPH, 23.0, 50.2 * MPH, -3.8 * MPH)
    assert A.too_close(49 * MPH, 23.0, 45.4 * MPH, -3.6 * MPH)
    # the same gap at the same speed with the lead pulling away, or holding station: no warning
    assert not A.too_close(54 * MPH, 23.0, 56 * MPH, 2 * MPH)
    assert not A.too_close(54 * MPH, 23.0, 54 * MPH, 0.0)
    # closing fast from far back (ACC would hold 87 m at 77 mph closing 6.4): outside half of it at 60 m
    assert not A.too_close(77 * MPH, 60.0, 70.6 * MPH, -6.4 * MPH)
    assert A.too_close(77 * MPH, 40.0, 70.6 * MPH, -6.4 * MPH)

  def test_faster_closing_warns_earlier(self):
    v = 30.0
    def first_gap(closing):
      return next(d for d in range(100, 0, -1) if A.too_close(v, float(d), v - closing, -closing))
    assert first_gap(6.0) > first_gap(2.0) > first_gap(0.6)


class TestStateMachine:
  def test_debounce_then_warn_and_hold(self):
    w = A.ApproachWarning(DT)
    out = _run(w, 1.0)
    assert not any(out[:int(A.ON_TIME / DT) - 1]) and out[-1]
    # the condition goes away: the text stays MIN_SHOW from its start and OFF_TIME past the condition, then ends
    out = _run(w, 4.0, d=80.0)
    assert out[0] and not out[-1]
    assert sum(out) * DT >= A.OFF_TIME - DT

  def test_no_chime_chatter(self):
    # condition flickering at the boundary: one warning episode, then nothing new inside REARM_TIME
    w = A.ApproachWarning(DT)
    starts, prev = 0, False
    for k in range(int(8.0 / DT)):
      cur = w.update(True, 24.0, True, 20.0 if (k // 8) % 2 == 0 else 60.0, 22.0, -2.0, 0.99)
      starts += cur and not prev
      prev = cur
    assert starts == 1

  def test_rearm_after_it_ends(self):
    w = A.ApproachWarning(DT)
    _run(w, 1.0)
    _run(w, A.MIN_SHOW + A.OFF_TIME + A.REARM_TIME + 0.5, d=80.0)
    assert not w.warning
    assert _run(w, 1.0)[-1]

  @pytest.mark.parametrize("kw", [{"active": False}, {"present": False}, {"prob": 0.3}, {"v": 3.0, "v_lead": 1.0, "d": 3.0},
                                  {"v_lead": 24.0}, {"d": 60.0}])
  def test_never_warns_when(self, kw):
    # mode off / toggle off / disengaged / gas (all = not active), no lead, a doubtful lead, crawling, not closing, far
    assert not any(_run(A.ApproachWarning(DT), 3.0, **kw))

  def test_inactive_clears_at_once(self):
    w = A.ApproachWarning(DT)
    _run(w, 1.0)
    assert w.warning
    assert not w.update(False, 24.0, True, 20.0, 22.0, -2.0, 0.99)


class TestWiring:
  def test_alert_is_a_warning_that_says_non_acc_will_not_brake(self):
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, EventNameSP
    from openpilot.selfdrive.selfdrived.events import ET
    alerts = EVENTS_SP[EventNameSP.nonAccApproach]
    assert list(alerts) == [ET.WARNING]          # shown while engaged; never a disable / no-entry
    a = alerts[ET.WARNING]
    assert "Non-ACC" in a.alert_text_2 and "Brake" in a.alert_text_2

  def test_toggle_defaults_on(self):
    from openpilot.common.params import Params
    assert Params().get("NonAccApproachWarning", return_default=True) is True


class TestSession15:
  """Owner 2026-11-06 after the first drive with it: "increase it by 10 %, it's a little too tight. Suppress it if the
  gap is holding or increasing." """

  def test_distance_is_ten_percent_longer(self):
    assert A.FRACTION == pytest.approx(0.55)
    v, vl = 61 * MPH, 58 * MPH
    edge = 0.5 * A.acc_closest_gap(v, vl)
    assert A.too_close(v, edge * 1.09, vl, vl - v) and not A.too_close(v, edge * 1.11, vl, vl - v)

  def test_holding_a_close_gap_with_a_noisy_lead_speed_stays_quiet(self):
    # 1513-1536 s on route 0000005a: a steady 0.6 s gap at 66 mph; the vision vRel jumped +-3 m/s frame to frame and
    # the raw value kept the text up for 14 s. The gap is not closing: no warning.
    w = A.ApproachWarning(DT)
    v = 66 * MPH
    out = [w.update(True, v, True, 18.0, v, (-3.0 if k % 2 else 3.0), 0.99) for k in range(int(20.0 / DT))]
    assert not any(out)

  def test_warning_ends_as_soon_as_the_gap_opens(self):
    w = A.ApproachWarning(DT)
    _run(w, 1.5)                                   # closing 2 m/s inside the distance
    assert w.warning
    out = [w.update(True, 24.0, True, 20.0, 25.5, 1.5, 0.99) for _ in range(int(6.0 / DT))]   # still close, now opening
    shown = sum(out) * DT
    assert not out[-1] and shown < A.MIN_SHOW + A.CLOSING_TAU + 1.0   # gone well before a lead that stays close would clear it

  def test_real_closing_through_noise_still_warns(self):
    w = A.ApproachWarning(DT)
    out = [w.update(True, 24.0, True, 20.0, 22.0, -2.0 + (2.5 if k % 2 else -2.5), 0.99) for k in range(int(3.0 / DT))]
    assert out[-1]

  def test_a_new_lead_starts_its_own_estimate(self):
    w = A.ApproachWarning(DT)
    for _ in range(40):
      w.update(True, 24.0, True, 20.0, 27.0, 3.0, 0.99)          # a lead pulling away
    w.update(True, 24.0, False, 0.0, 0.0, 0.0, 0.0)              # lost
    out = _run(w, 0.6)                                           # a new lead, closing: not held back by the old estimate
    assert out[-1]

  def test_handback_notification_is_wired(self):
    # owner 2026-11-06: the handback gave no chime and no text (sunnypilot's MADS strips speedTooLow); the earlier
    # build's notification is back
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, EventNameSP
    from openpilot.selfdrive.selfdrived.events import ET, AudibleAlert, Priority
    alerts = EVENTS_SP[EventNameSP.manualLowSpeedHandback]
    assert list(alerts) == [ET.WARNING]
    a = alerts[ET.WARNING]
    assert a.alert_text_1 == "Low Speed: Take Over" and a.audible_alert == AudibleAlert.prompt
    assert a.priority < Priority.HIGHEST           # never replaces or masks FCW
