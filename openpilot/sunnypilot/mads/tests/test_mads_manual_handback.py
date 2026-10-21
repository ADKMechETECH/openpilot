"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Manual low-speed handback vs MADS (owner, 2026-10-21): the handback hands back LONGITUDINAL only. Its belowEngageSpeed /
speedTooLow (a gear-aware speed of up to 13 mph since Stage 3) must never end or refuse lateral.
"""
from openpilot.cereal import log, custom
from opendbc.car import structs
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.tests.test_mads_steering_mode import make_car_state, make_mads, run_frames

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
TransmissionType = structs.CarParams.TransmissionType


def honda_mads(mocker, manual=True, op_long=True, unified=True):
  mads, sd = make_mads(mocker, MadsSteeringModeOnBrake.REMAIN_ACTIVE)
  CP = structs.CarParams()
  CP.brand = "honda"
  CP.transmissionType = TransmissionType.manual if manual else TransmissionType.automatic
  CP.openpilotLongitudinalControl = op_long
  CP.pcmCruise = not op_long
  sd.CP = CP
  mads.__init__(sd)
  mads.enabled_toggle, mads.unified_engagement_mode = True, unified
  return mads, sd


def frame(mads, sd, *events, v=5.4):
  for e in events:
    sd.events.add(e)
  run_frames(mads, sd, make_car_state(v_ego=v))


class TestManualHandbackKeepsLateral(OpenpilotTestCase):
  def test_set_below_handback_speed_still_engages_lateral(self, mocker):
    # 12 mph in 3rd: selfdrived refuses longitudinal (belowEngageSpeed, gear-aware 13 mph); lateral engages anyway
    mads, sd = honda_mads(mocker)
    frame(mads, sd, EventName.pcmEnable, EventName.belowEngageSpeed)
    self.assertEqual(mads.state_machine.state, State.enabled)
    self.assertTrue(mads.enabled)

  def test_handback_disengage_keeps_lateral(self, mocker):
    mads, sd = honda_mads(mocker)
    frame(mads, sd, EventName.pcmEnable)
    self.assertTrue(mads.enabled)
    sd.enabled = False                       # selfdrived has acted on speedTooLow before MADS runs
    frame(mads, sd, EventName.speedTooLow, EventName.belowEngageSpeed, v=2.0)
    self.assertTrue(mads.enabled, "only longitudinal is handed back")

  def test_stock_honda_unchanged(self, mocker):
    # not a manual-handback car (stock ACC: minEnableSpeed is real there): stock MADS-on-Honda behavior stays
    for kw in ({"manual": True, "op_long": False}, {"manual": False, "op_long": True}):
      with self.subTest(**kw):
        mads, sd = honda_mads(mocker, **kw)
        frame(mads, sd, EventName.pcmEnable, EventName.belowEngageSpeed)
        self.assertEqual(mads.state_machine.state, State.disabled)
