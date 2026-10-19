"""Task 12 plumbing: the fork's openpilot-internal carControlSP state never reaches the car interface."""
from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.helpers import convert_carControlSP


class TestManualPlumbing(OpenpilotTestCase):
  def test_card_converts_carcontrolsp_with_manual_longitudinal(self):
    # card.py converts carControlSP every frame: an unknown field would raise (and take card down) on the car
    msg = custom.CarControlSP.new_message()
    msg.manualLongitudinal.gatePaused = True
    msg.manualLongitudinal.handbackSpeed = 2.24
    msg.mads.enabled = True
    out = convert_carControlSP(msg.as_reader())
    self.assertTrue(out.mads.enabled)
    self.assertFalse(hasattr(out, "manualLongitudinal"))
