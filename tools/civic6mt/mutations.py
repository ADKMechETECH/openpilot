"""Mutation manifest (run with mutate.py). Each entry breaks one safety property; `tests` must catch it.
Paths are relative to the openpilot checkout. Task 12 entries name their finding IDs; every Rule-5 call site has one.
"""
SP = "openpilot/sunnypilot/selfdrive/controls"
T = f"{SP}/lib/tests"
R5 = [f"{T}/test_rule5.py"]
HB = [f"{T}/test_manual_low_speed_handback.py", f"{T}/test_manual_handback_integration.py"]
NA = [f"{T}/test_non_adaptive_longitudinal.py"]
NSB = [f"{T}/test_task12_nsb_latch.py", f"{T}/test_no_service_brake.py", f"{T}/test_no_service_brake_pulse.py"]
GATE = [f"{T}/test_clutch_shift_gate.py", f"{T}/test_clutch_reengage.py"]
EBP = [f"{T}/test_engine_brake_preferred.py"]
GAA = [f"{T}/test_gear_aware_accel.py"]
B3 = [f"{T}/test_task12_nsb_latch.py"]
EV = ["openpilot/sunnypilot/selfdrive/car/tests/test_clutch_gate_events.py"]
ODBC = ["opendbc_repo/opendbc/car/honda/tests/test_manual_carstate.py"]


def m(mid, finding, file, old, new, tests):
  return {"id": mid, "finding": finding, "file": file, "old": old, "new": new, "tests": tests}


MUTATIONS = [
  # --- R5: the predicate itself -----------------------------------------------------------------------------------
  m("r5_predicate_off", "R5", f"{SP}/lib/braking_urgency.py",
    "  CS, plan = sm['carState'], sm['longitudinalPlan']\n  return braking_urgency(",
    "  CS, plan = sm['carState'], sm['longitudinalPlan']\n  return None and braking_urgency(", R5),
  m("r5_valid_not_all_checks", "R5 A-F-L4", f"{SP}/lib/braking_urgency.py",
    "plan_ok=sm.all_checks(['longitudinalPlan'])", "plan_ok=sm.valid['longitudinalPlan']", R5),
  m("r5_no_fcw", "R5", f"{SP}/lib/braking_urgency.py", "  if fcw:\n", "  if False:\n", R5),
  m("r5_no_force_decel", "R5", f"{SP}/lib/braking_urgency.py", "  if force_decel:\n", "  if False:\n", R5),
  m("r5_no_stock_aeb", "R5 A-F-L1", f"{SP}/lib/braking_urgency.py", "  if stock_aeb:\n", "  if False:\n", R5),
  m("r5_no_lead", "R5", f"{SP}/lib/braking_urgency.py",
    "    reason = lead_reason(lead, v_ego)\n", "    reason = None\n", R5),
  m("r5_plan_live_ignored", "A3 (handback freeze)", f"{SP}/lib/braking_urgency.py",
    "  if plan_live and should_stop:", "  if should_stop:", HB + R5),
  # --- R5 call sites ------------------------------------------------------------------------------------------------
  m("r5_nsb_controlsd", "R5 C-04", f"{SP}/controlsd_ext.py",
    "    if self.urgency is not None:\n      self.no_service_brake_pulse.floor = None\n      return accel_limits\n", "", R5),
  m("r5_nsb_maintain_plannerd", "R5 B2 C-04", f"{SP}/lib/no_service_brake.py",
    "self.backstop_active or urgency is not None or target_low", "self.backstop_active or target_low", NSB),
  m("r5_nsb_pulse_plannerd", "R5 B2", f"{SP}/lib/no_service_brake.py",
    'urgency or ("lower_target" if target_low else None), rising)', '("lower_target" if target_low else None), rising)',
    [f"{T}/test_no_service_brake_pulse.py", f"{T}/test_non_adaptive_longitudinal.py"]),
  m("r5_ebp", "R5", f"{SP}/lib/engine_brake_preferred.py",
    '    if urgency is not None:\n      return "urgency"\n', "", R5 + EBP),
  m("r5_gate_takeover", "R5 A2", f"{SP}/lib/clutch_reengage.py",
    "    if urgent is not None:\n      self.end(urgent)\n      return accel_limits\n", "", R5),
  m("r5_handback_freeze", "R5 A3.2", f"{SP}/lib/manual_low_speed_handback.py",
    "    elif urgent:\n      pass ", "    elif False:\n      pass ", HB),
  m("r5_handback_lower", "R5 A3.2", f"{SP}/lib/manual_low_speed_handback.py",
    "    lo = accel_limits[0] if self.urgent else max(", "    lo = max(", R5),
  m("r5_gaa", "R5 C-11", f"{SP}/lib/gear_aware_accel.py",
    "clutch_valid and urgency is None and pitch", "clutch_valid and pitch", GAA + R5),
  m("r5_gate_raises_lower", "A2", f"{SP}/lib/clutch_shift_gate.py",
    "    return min(lo, hi), hi", "    return max(lo, min(hi, 0.)), hi", R5),
  # --- A1 non-adaptive collision braking ----------------------------------------------------------------------------
  m("a1_never_readmit", "A1 A-F-C1 B-C1", "openpilot/selfdrive/controls/lib/longitudinal_planner.py",
    "    if self.non_adaptive and not collision:", "    if self.non_adaptive:", NA),
  m("a1_no_threat", "A1", f"{SP}/lib/longitudinal_planner.py",
    "      reason = reason or collision_threat(lead, v_ego)", "      reason = reason", NA),
  m("a1_nsb_not_released", "A1", f"{SP}/lib/longitudinal_planner.py",
    '    urgency = self.urgency or ("collision" if collision else None)', "    urgency = self.urgency", NA),
  m("a1_closing_follow_brakes", "A1 (owner decision)", f"{SP}/lib/braking_urgency.py",
    "    return reason if closing > COLLISION_CLOSE_MIN_CLOSING or lead.aLeadK < LEAD_DECEL else None",
    "    return reason", NA),
  m("a1_lead_visible", "A1", f"{SP}/controlsd_ext.py",
    "    return has_lead and (not features.nonAdaptive or features.collisionBraking)", "    return has_lead", NA),
  m("a1_no_cruise_reseed", "A1 (no step)", "openpilot/selfdrive/controls/lib/longitudinal_planner.py",
    "      self.a_cruise = min(self.a_cruise, float(self.output_a_target))", "      pass", NA),
  # --- A2 clutch gate -----------------------------------------------------------------------------------------------
  m("a2_blip_dropped_unpaused", "A2.6", "openpilot/sunnypilot/selfdrive/car/car_specific.py",
    "    if fresh and ml.gatePaused and CS.deprecated.clutchPressed", "    if fresh and CS.deprecated.clutchPressed", EV),
  m("a2_stale_state_trusted", "A3.5 A2.6", "openpilot/sunnypilot/selfdrive/car/car_specific.py",
    "    fresh = ml is not None and ml_fresh", "    fresh = ml is not None", EV),
  m("a2_coast_accel_unclamped", "A2.1", f"{SP}/lib/clutch_shift_gate.py",
    "COAST_ACCEL_RANGE = (-0.5, 0.0)", "COAST_ACCEL_RANGE = (-0.5, 1.0)", GATE),
  m("a2_max_coast_unbounded", "A2.3", f"{SP}/lib/clutch_reengage.py",
    "MAX_COAST_RANGE = (2., 5.)", "MAX_COAST_RANGE = (2., 20.)", GATE),
  m("a2_signal_death_ignored", "B5 A-F-M6", f"{SP}/lib/clutch_shift_gate.py",
    "    elif self.enabled and not clutch_valid:", "    elif False:", GATE),
  # --- A3 handback --------------------------------------------------------------------------------------------------
  m("a3_force_decel_ignored", "A3.1", f"{SP}/lib/manual_low_speed_handback.py",
    "    elif force_decel:", "    elif False:", HB),
  m("a3_prompt_at_disengage", "A3.3", "openpilot/sunnypilot/selfdrive/car/car_specific.py",
    "      elif fresh and ml.handbackActive:", "      elif False:", HB),
  # --- B1/B2 NSB ----------------------------------------------------------------------------------------------------
  m("b1_lower_target_capped", "B1 C-01", f"{SP}/lib/no_service_brake.py",
    "    target_low = v_target is not None and v_target < v_set - 0.1", "    target_low = False", NSB),
  m("b2_margin_unclamped", "B2 C-04", f"{SP}/lib/no_service_brake.py",
    'DEFAULT_OVERSPEED_MARGIN, 0.5, 15.), ceiling)', 'DEFAULT_OVERSPEED_MARGIN, 0.5, 15.), 99.)', NSB),
  m("b2_alert_no_repeat", "B2 C-06", f"{SP}/lib/no_service_brake.py",
    "(rising or not self._alerted)", "(not self._alerted)", NSB),
  m("b2_no_set_drop_grace", "B2 A-F-L5", f"{SP}/lib/no_service_brake.py",
    "      self.set_drop_timer = SET_DROP_GRACE", "      self.set_drop_timer = 0.", NSB),
  # --- B3 latching --------------------------------------------------------------------------------------------------
  m("b3_nsb_latch_engaged", "B3", f"{SP}/lib/no_service_brake.py",
    "    if not engaged:  # never switch modes or re-tune mid-engagement\n      self.p = self._requested\n",
    "    self.p = self._requested\n    if not engaged:\n", B3 + NSB),
  m("b3_non_adaptive_latch", "B3", f"{SP}/lib/longitudinal_planner.py",
    "    if not self._engaged(sm):\n      self.non_adaptive = self._non_adaptive_requested",
    "    if True:\n      self.non_adaptive = self._non_adaptive_requested", B3 + NA),
  m("b3_gaa_latch", "B3", f"{SP}/lib/gear_aware_accel.py",
    "    if not engaged:  # never switches mid-engagement", "    if True:", B3 + GAA),
  m("b3_ebp_latch", "B3", f"{SP}/lib/engine_brake_preferred.py",
    "    if not self.enabled:\n      return False\n    if not long_active:",
    "    self._latch()\n    if not self.enabled:\n      return False\n    if not long_active:", B3 + EBP),
  m("b3_ebp_lockout_cleared", "B3", f"{SP}/lib/engine_brake_preferred.py",
    '      self._release("inactive")\n      return False', '      self.reset()\n      return False', B3),
  m("b3_handback_latch", "B3 A3.5", f"{SP}/lib/manual_low_speed_handback.py",
    "    if not enabled:\n      self.active = False", "    self.speed, self.ramp_time = self._requested\n    if not enabled:\n      self.active = False",
    B3 + HB),
  # --- B4 smart re-engagement ---------------------------------------------------------------------------------------
  m("b4_lock_any_gear", "B4.1 C-02", f"{SP}/lib/clutch_reengage.py",
    "if gear == 0 or gear != gear_raw or", "if gear == 0 or", GATE),
  m("b4_lock_tol_loose", "B4.2 C-02", f"{SP}/lib/clutch_reengage.py",
    "abs(ratio / overall_ratio(gear) - 1.) > tol:", "abs(ratio / overall_ratio(gear) - 1.) > 0.08:", GATE),
  m("b4_no_steady_time", "B4.3 C-02", f"{SP}/lib/clutch_reengage.py",
    "RATIO_LOCK_TIME = 0.12 ", "RATIO_LOCK_TIME = 0.0 ", GATE),
  # --- B5 plumbing --------------------------------------------------------------------------------------------------
  m("b5_logger_blocking", "B5 A-F-L6", f"{SP}/lib/engine_brake_model.py",
    "      self._queue.put_nowait(line)", "      self._queue.put(line)\n      self._queue.join()",
    [f"{T}/test_engine_brake_model.py"]),
  m("b5_gaa_pitch_none", "B5 C-11", f"{SP}/lib/gear_aware_accel.py",
    "and pitch is not None and math.isfinite(pitch)", "and (pitch is None or math.isfinite(pitch))", GAA),
  m("b5_card_converts_unknown", "B5 (found in Task 12)", "openpilot/selfdrive/car/helpers.py",
    " and k in known}))", "}))", [f"{T}/test_manual_plumbing.py"]),
  m("b5_odbc_gearbox_freq", "B5 A-F-M6", "opendbc_repo/opendbc/car/honda/carstate.py",
    '("GEARBOX_ALT_2", float(\'nan\'))', '("GEARBOX_ALT_2", 50)', ODBC),
  m("b5_odbc_liveness", "B5 A-F-M6", "opendbc_repo/opendbc/car/honda/carstate.py",
    "ret_sp.clutchSignalValid = ts > 0 and (cp.last_nonempty_nanos - ts) < GEARBOX_ALT_2_TIMEOUT_NS",
    "ret_sp.clutchSignalValid = True", ODBC),
  m("b5_odbc_unknown_gear_coupled", "A2.7 B-L1", "opendbc_repo/opendbc/car/honda/carstate.py",
    "ret.deprecated.clutchPressed = gear_mt not in GEAR_MT_COUPLED", "ret.deprecated.clutchPressed = gear_mt == 0", ODBC),
  m("b5_odbc_integra_rpm", "B5 B-M7", "opendbc_repo/opendbc/car/honda/carstate.py",
    "        if self.CP.carFingerprint == CAR.ACURA_INTEGRA:\n", "        if False:\n", ODBC),
]
