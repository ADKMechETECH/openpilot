"""
Regression test for selfdrive/car/helpers.convert_carControlSP.

The fork's custom.capnp CarControlSP carries fields that opendbc's structs.CarControlSP
dataclass does not mirror (e.g. nonAcc, handback - consumed by controls/UI, never by the car
interface). card calls convert_carControlSP every control frame to hand carControlSP to CI.apply;
if an unknown capnp field is passed as a kwarg to the dataclass, card crashes on the car:

    TypeError: CarControlSP.__init__() got an unexpected keyword argument 'nonAcc'

That shipped once (sp-steer base swap dropped the known-fields filter) and showed as an immediate
LKAS/camera fault at boot (card crash-loop -> no CAN to the camera). This pins the filter so any
fork-only capnp field is dropped, every real field passes, and no unknown field can crash card.
"""
from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.selfdrive.car.helpers import convert_carControlSP

# every top-level field the fork's capnp CarControlSP exposes that opendbc's dataclass does NOT have
_KNOWN = set(structs.CarControlSP.__dataclass_fields__)
_FORK_ONLY = sorted(set(custom.CarControlSP.schema.fields) - _KNOWN)


def test_fork_only_fields_exist():
  # if this is ever empty the test is vacuous; the point is there ARE capnp fields opendbc lacks
  assert _FORK_ONLY, "expected fork-only CarControlSP capnp fields (e.g. nonAcc, handback)"


def test_convert_does_not_crash_and_drops_unknown():
  msg = custom.CarControlSP.new_message()
  # a real nested field that must survive, plus fork-only fields that must be dropped
  msg.mads.available = True
  if "nonAcc" in _FORK_ONLY:
    msg.nonAcc.active = True
  if "handback" in _FORK_ONLY:
    msg.handback.active = True

  out = convert_carControlSP(msg.as_reader())

  assert isinstance(out, structs.CarControlSP)
  assert out.mads.available is True                       # real field passed through
  for f in _FORK_ONLY:
    assert not hasattr(out, f), f"fork-only field {f!r} must not reach the opendbc dataclass"


def test_every_fork_only_field_set_individually():
  # setting any single fork-only field must never crash the conversion
  for f in _FORK_ONLY:
    msg = custom.CarControlSP.new_message()
    sub = getattr(msg, f)
    # touch a bool/float sub-field if present so the field is actually populated
    for name in ("active", "available", "enabled"):
      if hasattr(sub, name):
        setattr(sub, name, True)
        break
    convert_carControlSP(msg.as_reader())  # must not raise
