import capnp
from typing import Any

from openpilot.cereal import custom
from opendbc.car import structs

_FIELDS = '__dataclass_fields__'  # copy of dataclasses._FIELDS


def is_dataclass(obj):
  """Similar to dataclasses.is_dataclass without instance type check checking"""
  return hasattr(obj, _FIELDS)


def _asdictref_inner(obj) -> dict[str, Any] | Any:
  if is_dataclass(obj):
    ret = {}
    for field in getattr(obj, _FIELDS):  # similar to dataclasses.fields()
      ret[field] = _asdictref_inner(getattr(obj, field))
    return ret
  elif isinstance(obj, (tuple, list)):
    return type(obj)(_asdictref_inner(v) for v in obj)
  else:
    return obj


def asdictref(obj) -> dict[str, Any]:
  """
  Similar to dataclasses.asdict without recursive type checking and copy.deepcopy
  Note that the resulting dict will contain references to the original struct as a result
  """
  if not is_dataclass(obj):
    raise TypeError("asdictref() should be called on dataclass instances")

  return _asdictref_inner(obj)


def convert_to_capnp(struct: structs.CarParamsSP | structs.CarStateSP) -> capnp.lib.capnp._DynamicStructBuilder:
  struct_dict = asdictref(struct)

  if isinstance(struct, structs.CarParamsSP):
    struct_capnp = custom.CarParamsSP.new_message(**struct_dict)
  elif isinstance(struct, structs.CarStateSP):
    struct_capnp = custom.CarStateSP.new_message(**struct_dict)
  else:
    raise ValueError(f"Unsupported struct type: {type(struct)}")

  return struct_capnp


_PLAN_CACHE: dict[tuple, tuple] = {}
_CAPNP_LISTS = (capnp.lib.capnp._DynamicListReader, capnp.lib.capnp._DynamicListBuilder)


def _convert_plan(cls, schema) -> tuple:
  """(field name, nested dataclass type or None) for every dataclass field that the capnp struct also has.
  Cached per (dataclass, capnp schema): built once, then conversion is plain attribute reads."""
  key = (cls, schema.node.id)
  plan = _PLAN_CACHE.get(key)
  if plan is None:
    default = cls()
    plan = _PLAN_CACHE[key] = tuple(
      (name, type(getattr(default, name)) if is_dataclass(getattr(default, name)) else None)
      for name in getattr(cls, _FIELDS) if name in schema.fields and not name.endswith('DEPRECATED'))
  return plan


def _capnp_to_dataclass(struct, cls):
  kwargs = {}
  for name, nested in _convert_plan(cls, struct.schema):
    v = getattr(struct, name)
    if nested is not None:
      v = _capnp_to_dataclass(v, nested)
    elif isinstance(v, capnp.lib.capnp._DynamicEnum):
      v = str(v)
    elif isinstance(v, _CAPNP_LISTS):
      v = [x.to_dict() if hasattr(x, 'to_dict') else x for x in v]
    kwargs[name] = v
  return cls(**kwargs)


def convert_carControlSP(struct: capnp.lib.capnp._DynamicStructReader) -> structs.CarControlSP:
  # Only what the car interface knows: openpilot-internal carControlSP fields (e.g. the fork's nonAcc / handback state
  # for selfdrived/UI) never reach opendbc, and passing an unknown kwarg crashes card on the car. The conversion is
  # driven by the dataclass's own fields, so an unknown capnp field can never be passed.
  # It reads those fields straight off the capnp message instead of struct.to_dict() + rebuild: card runs this every
  # control frame on the saturated control core (session 12, 2026-11-05).
  return _capnp_to_dataclass(struct, structs.CarControlSP)
