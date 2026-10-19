#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

test_consoled — on-car test console daemon (DEV-TEST BUILD ONLY). Telemetry only: subscribes, labels, logs and
writes a status file. It publishes no cereal message, sends no CAN, and writes no Param except reading its own
TestConsole* controls. Subscribing can't slow a publisher (msgq is a ring buffer; a slow reader just drops).

Vehicle-agnostic: the console runs the plan of the vehicle profile selected for this car (profile.select_profile). No
profile for the platform -> inactive: it subscribes to nothing and writes nothing (the UI shows "no test-console profile
for this platform"); a CarParams change is re-checked every second either way.
"""
import time

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.sunnypilot.selfdrive.controls.lib.no_service_brake import read_overspeed_mode, read_pulse_params
from openpilot.sunnypilot.selfdrive.test_console.button import StartButton
from openpilot.sunnypilot.selfdrive.test_console.capture import Console, Store
from openpilot.sunnypilot.selfdrive.test_console.conditions import Context
from openpilot.sunnypilot.selfdrive.test_console.guard import Guard
from openpilot.sunnypilot.selfdrive.test_console.profile import inactive_reason, select_profile
from openpilot.sunnypilot.selfdrive.test_console.signals import LogEventTap, SampleBuilder
from openpilot.sunnypilot.selfdrive.test_console.status_io import P_ARMED, P_BACKGROUND, P_BUTTON, P_PRESS, P_SELECTED, console_dir

PARAMS_PERIOD = 1.0      # s
STATUS_PERIOD = 0.2      # s
SERVICES = ["carState", "carControl", "carOutput", "longitudinalPlan", "radarState", "selfdriveState", "pandaStates",
            "carControlSP", "longitudinalPlanSP"]   # Task 12: the features' published (latched) state


def read_context(params: Params, ctx: Context) -> None:
  # return_default: Params.get_bool() ignores registered defaults (NoServiceBrakeOverspeedBackstop defaults ON)
  for key in ctx.watched_toggles:
    ctx.toggles[key] = bool(params.get(key, return_default=True))
  ctx.nsb_floor = float(params.get("NoServiceBrakeDecelFloor", return_default=True))
  ctx.nsb_margin = float(params.get("NoServiceBrakeOverspeedMargin", return_default=True))
  ctx.nsb_mode = read_overspeed_mode(params)
  pulse = read_pulse_params(params, min(max(ctx.nsb_margin, 0.5), 15.))  # the margin clamp NoServiceBrake applies
  ctx.nsb_trigger, ctx.nsb_release, ctx.nsb_ceiling = pulse.trigger, pulse.release, pulse.ceiling
  ctx.nsb_pulse_decel, ctx.nsb_pulse_jerk = pulse.decel, pulse.jerk
  ctx.handback_ramp = float(params.get("ManualLowSpeedHandbackRamp", return_default=True))
  ctx.ebp_coast_target = float(params.get("EngineBrakeCoastTarget", return_default=True))


def run_console(params: Params, CP, CP_bytes: bytes, profile) -> None:
  """The active console for one profile, until the process exits or CarParams change (then returns)."""
  ctx = Context(profile=profile, manual=CP.transmissionType == car.CarParams.TransmissionType.manual,
                op_long=CP.openpilotLongitudinalControl)
  store = Store(console_dir())
  console = Console(store, ctx)
  tap = profile.signals.raw_tap(CP)
  tap = tap if tap is not None and tap.available else None
  builder, log_tap = SampleBuilder(profile), LogEventTap()
  button = StartButton(params.get(P_BUTTON, return_default=True), params.get(P_PRESS, return_default=True),
                       buttons=profile.buttons)

  sm = messaging.SubMaster(SERVICES, poll="carState")
  log_sock = messaging.sub_sock("logMessage", conflate=False)
  can_sock = messaging.sub_sock("can", conflate=False) if tap is not None else None
  sendcan_sock = messaging.sub_sock("sendcan", conflate=False) if tap is not None else None

  t_params = t_status = 0.
  guard = Guard()   # Task 12 D1: a console bug logs once and the daemon keeps running (or goes inert) - never dies
  frame = {"s": None, "events": ()}

  def read_params() -> bool:
    if params.get("CarParams") != CP_bytes:
      cloudlog.event("test_console_carparams_changed")
      return False
    read_context(params, ctx)
    name, mode = params.get(P_BUTTON, return_default=True), params.get(P_PRESS, return_default=True)
    if (name or "") != button.name or (mode or "") != button.mode:
      button.configure(name, mode)
    console.set_control(params.get(P_SELECTED) or "", params.get_bool(P_ARMED), params.get_bool(P_BACKGROUND))
    return True

  def drain() -> None:
    frame["events"] = log_tap.update([m.logMessage for m in messaging.drain_sock(log_sock)])
    if tap is not None:
      can_raw, sendcan_raw = messaging.drain_sock_raw(can_sock), messaging.drain_sock_raw(sendcan_sock)
      if console.selected or console.background:  # only pay for parsing when something can use it
        tap.update(can_capnp_to_list(can_raw), can_capnp_to_list(sendcan_raw, "sendcan"))

  def step() -> None:
    t = sm.logMonoTime["carState"] / 1e9
    tx_blocked = sum(ps.safetyTxBlocked for ps in sm["pandaStates"])
    s = builder.build(t, sm["carState"], sm["carControl"], sm["carOutput"], sm["longitudinalPlan"],
                      sm.all_checks(["longitudinalPlan"]), sm["radarState"], sm["selfdriveState"], tx_blocked, tap, log_tap,
                      frame["events"], sm["carControlSP"], sm.all_checks(["carControlSP"]), sm["longitudinalPlanSP"],
                      sm.all_checks(["longitudinalPlanSP"]))
    frame["s"] = s
    console.update(s, button.update(t, sm["carState"].buttonEvents))

  def save(exited: bool = False) -> None:
    info = {"name": button.name, "mode": button.mode, "seen": button.seen}
    state = console.status(frame["s"], info)
    if guard.errors:
      state["error"] = f"{guard.errors} console errors (see the rlog){' - inert' if guard.inert else ''}"
    store.save_state({**state, "exited": True} if exited else state)

  try:
    while True:
      sm.update()
      now = time.monotonic()
      if now - t_params > PARAMS_PERIOD:
        t_params = now
        if params.get("CarParams") != CP_bytes:
          cloudlog.event("test_console_carparams_changed")
          return
        guard.run("params", read_params)
      guard.run("drain", drain)
      if not sm.updated["carState"]:
        continue
      guard.run("frame", step)
      if now - t_status > STATUS_PERIOD:
        t_status = now
        guard.run("status", save)
  finally:
    guard.inert = False   # the exit status is written even after going inert
    guard.run("shutdown", console.shutdown)
    if frame["s"] is not None:
      guard.run("exit_status", save, True)


def main() -> None:
  # default (non-realtime) priority on purpose: never competes with controlsd/plannerd for CPU
  params = Params()
  cloudlog.info("test_consoled is waiting for CarParams")
  while True:
    CP_bytes = params.get("CarParams", block=True)
    CP = messaging.log_from_bytes(CP_bytes, car.CarParams)
    profile = select_profile(CP)
    if profile is not None:
      cloudlog.info(f"test_consoled: profile {profile.name} for {CP.carFingerprint}")
      run_console(params, CP, CP_bytes, profile)
      continue
    # inactive: nothing subscribed, nothing written; only watch for a CarParams change
    cloudlog.warning(f"test_consoled: {inactive_reason(CP)} - console inactive")
    while params.get("CarParams") == CP_bytes:
      time.sleep(PARAMS_PERIOD)


if __name__ == "__main__":
  main()
