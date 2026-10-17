#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

test_consoled — on-car test console daemon (DEV-TEST BUILD ONLY). Telemetry only: subscribes, labels, logs and
writes a status file. It publishes no cereal message, sends no CAN, and writes no Param except reading its own
TestConsole* controls. Subscribing can't slow a publisher (msgq is a ring buffer; a slow reader just drops).
"""
import time

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.sunnypilot.selfdrive.test_console.button import StartButton
from openpilot.sunnypilot.selfdrive.test_console.capture import Console, Store
from openpilot.sunnypilot.selfdrive.test_console.conditions import WATCHED_TOGGLES, Context
from openpilot.sunnypilot.selfdrive.test_console.signals import LogEventTap, RawCanTap, SampleBuilder
from openpilot.sunnypilot.selfdrive.test_console.status_io import P_ARMED, P_BACKGROUND, P_BUTTON, P_PRESS, P_SELECTED, console_dir

PARAMS_PERIOD = 1.0      # s
STATUS_PERIOD = 0.2      # s
SERVICES = ["carState", "carControl", "carOutput", "longitudinalPlan", "radarState", "selfdriveState", "pandaStates"]


def read_context(params: Params, ctx: Context) -> None:
  # return_default: Params.get_bool() ignores registered defaults (NoServiceBrakeOverspeedBackstop defaults ON)
  for key in WATCHED_TOGGLES:
    ctx.toggles[key] = bool(params.get(key, return_default=True))
  ctx.nsb_floor = float(params.get("NoServiceBrakeDecelFloor", return_default=True))
  ctx.nsb_margin = float(params.get("NoServiceBrakeOverspeedMargin", return_default=True))
  ctx.handback_ramp = float(params.get("ManualLowSpeedHandbackRamp", return_default=True))
  ctx.ebp_coast_target = float(params.get("EngineBrakeCoastTarget", return_default=True))


def main() -> None:
  # default (non-realtime) priority on purpose: never competes with controlsd/plannerd for CPU
  params = Params()
  cloudlog.info("test_consoled is waiting for CarParams")
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  ctx = Context(manual=CP.transmissionType == car.CarParams.TransmissionType.manual, op_long=CP.openpilotLongitudinalControl)

  store = Store(console_dir())
  console = Console(store, ctx)
  tap = RawCanTap(CP)
  builder, log_tap = SampleBuilder(), LogEventTap()
  button = StartButton(params.get(P_BUTTON, return_default=True), params.get(P_PRESS, return_default=True))

  sm = messaging.SubMaster(SERVICES, poll="carState")
  log_sock = messaging.sub_sock("logMessage", conflate=False)
  can_sock = messaging.sub_sock("can", conflate=False) if tap.available else None
  sendcan_sock = messaging.sub_sock("sendcan", conflate=False) if tap.available else None

  t_params = t_status = 0.
  s = None
  try:
    while True:
      sm.update()
      now = time.monotonic()
      if now - t_params > PARAMS_PERIOD:
        t_params = now
        read_context(params, ctx)
        name, mode = params.get(P_BUTTON, return_default=True), params.get(P_PRESS, return_default=True)
        if (name or "") != button.name or (mode or "") != button.mode:
          button.configure(name, mode)
        console.set_control(params.get(P_SELECTED) or "", params.get_bool(P_ARMED), params.get_bool(P_BACKGROUND))

      events = log_tap.update([m.logMessage for m in messaging.drain_sock(log_sock)])
      if tap.available:
        can_raw, sendcan_raw = messaging.drain_sock_raw(can_sock), messaging.drain_sock_raw(sendcan_sock)
        if console.selected or console.background:  # only pay for parsing when something can use it
          tap.update(can_capnp_to_list(can_raw), can_capnp_to_list(sendcan_raw, "sendcan"))

      if not sm.updated["carState"]:
        continue
      t = sm.logMonoTime["carState"] / 1e9
      tx_blocked = sum(ps.safetyTxBlocked for ps in sm["pandaStates"])
      s = builder.build(t, sm["carState"], sm["carControl"], sm["carOutput"], sm["longitudinalPlan"], sm.valid["longitudinalPlan"],
                        sm["radarState"], sm["selfdriveState"], tx_blocked, tap, log_tap, events)
      console.update(s, button.update(t, sm["carState"].buttonEvents))

      if now - t_status > STATUS_PERIOD:
        t_status = now
        store.save_state(console.status(s, {"name": button.name, "mode": button.mode, "seen": button.seen}))
  finally:
    console.shutdown()
    if s is not None:
      store.save_state({**console.status(s, {"name": button.name, "mode": button.mode, "seen": button.seen}), "exited": True})


if __name__ == "__main__":
  main()
