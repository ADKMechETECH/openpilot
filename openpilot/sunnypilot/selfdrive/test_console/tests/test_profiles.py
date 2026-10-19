"""
Task 10: the vehicle-agnostic core + per-vehicle profile split. Core purity (no platform token outside a profile),
profile selection by fingerprint (unknown -> inactive, safe), a stub second car running the unchanged core, the per-profile
UI allowlist, and the daemon staying inert (subscribes to and writes nothing) on a car with no profile.
"""
import dataclasses
import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile

from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.test_console import analyzers, signals
from openpilot.sunnypilot.selfdrive.test_console.button import StartButton
from openpilot.sunnypilot.selfdrive.test_console.conditions import Context
from openpilot.sunnypilot.selfdrive.test_console.profile import PROFILE_ENV, ROLES, inactive_reason, select_profile
from openpilot.sunnypilot.selfdrive.test_console.profiles import PROFILES
from openpilot.sunnypilot.selfdrive.test_console.profiles.civic_6mt import PROFILE as CIVIC
from openpilot.sunnypilot.selfdrive.test_console.registry import OP_LONG_PARAM
from openpilot.sunnypilot.selfdrive.test_console.sessions import ui_writable
from openpilot.sunnypilot.selfdrive.test_console.signals import SAMPLE_FIELDS
from openpilot.sunnypilot.selfdrive.test_console.status_io import CONSOLE_PARAMS
from openpilot.sunnypilot.selfdrive.test_console.tests.stub_profile import FINGERPRINT as STUB_FP, PROFILE as STUB, POWERTRAIN, \
  stub_drive

PKG = pathlib.Path(__file__).resolve().parents[1]
UI = PKG.parents[2] / "selfdrive" / "ui" / "sunnypilot" / "mici"
CORE_FILES = sorted(PKG.glob("*.py")) + [UI / "layouts" / "dev_test.py", UI / "layouts" / "dev_test_sessions.py",
                                         UI / "layouts" / "dev_test_params.py", UI / "onroad" / "test_console_chip.py"]
# platform knowledge that must live in a profile: brand / model, the Civic's raw field names and message IDs, its start
# button, its feature-model module (seed ratios) and its brake-authority PR (any case) ...
PLATFORM_TOKENS = ("honda", "civic", "bosch", "radarless", "toyota", "hyundai", "gear_mt", "raw_17c", "pedal_gas",
                   "gearbox_alt_2", "powertrain_data", "acc_control", "engine_data", "scm_buttons", "computer_brake_assist",
                   "0x1dd", "0x17c", "0x1c8", "gapadjustcruise", "engine_brake_model", "#2050")
# ... and DBC signal names (case-sensitive: "accel_command_sent" is a core role name)
DBC_SIGNALS = ("GEAR_MT", "ACCEL_COMMAND", "CRUISE_SETTING", "ENGINE_RPM", "PEDAL_GAS", "CRUISE_BUTTONS")
MESSAGE_ID = re.compile(r"\b0x[0-9a-f]{2,3}\b", re.IGNORECASE)
ButtonType = structs.CarState.ButtonEvent.Type


def make_cp(fingerprint: str, transmission: str = "manual"):
  cp = car.CarParams.new_message()
  cp.carFingerprint, cp.transmissionType = fingerprint, transmission
  return cp


class TestCorePurity(OpenpilotTestCase):
  def test_core_names_no_platform(self):
    self.assertGreaterEqual(len(CORE_FILES), 13)
    for path in CORE_FILES:
      self.assertTrue(path.exists(), path)
      text = path.read_text()
      for token in PLATFORM_TOKENS:
        self.assertNotIn(token, text.lower(), f"{path.name} names '{token}' - that belongs in a vehicle profile")
      for token in DBC_SIGNALS:
        self.assertNotIn(token, text, f"{path.name} names the DBC signal {token} - that belongs in a vehicle profile")
      self.assertIsNone(MESSAGE_ID.search(text), f"{path.name} names a CAN message id")

  def test_core_imports_no_profile(self):
    # the registration list (profiles/__init__.py) is the one place that names the cars; the core reaches it only
    # through select_profile()'s default
    for path in CORE_FILES:
      text = path.read_text()
      self.assertNotIn("profiles.civic_6mt", text, path.name)
      self.assertNotIn("controls.lib", text.replace("controls.lib.no_service_brake import", ""), path.name)

  def test_mirrored_gate_constants(self):
    # the core mirrors two clutch-gate constants instead of importing the feature module (which pulls in the Civic's
    # engine-brake model); pinned here so they can't drift
    from openpilot.sunnypilot.selfdrive.controls.lib.clutch_shift_gate import FLARE_RPM_RATE, RPM_FILTER_RC
    self.assertEqual(signals.RPM_FILTER_RC, RPM_FILTER_RC)
    self.assertEqual(analyzers.FLARE_RPM_RATE, FLARE_RPM_RATE)

  def test_core_runs_without_any_civic_module(self):
    # a fresh interpreter: the core + the stub car run a drive without importing the Civic profile or its model
    code = "\n".join((
      "import sys",
      "from openpilot.sunnypilot.selfdrive.test_console.tests.stub_profile import stub_drive",
      "stub_drive()",
      "bad = [m for m in sys.modules if 'civic_6mt' in m or 'engine_brake_model' in m or 'opendbc.car.honda' in m]",
      "print('BAD', bad)",
    ))
    env = {**os.environ, "TEST_CONSOLE_DIR": tempfile.mkdtemp()}
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env, check=True).stdout
    self.assertIn("BAD []", out)


class TestProfileSelection(OpenpilotTestCase):
  def test_by_fingerprint(self):
    self.assertIs(select_profile(make_cp("HONDA_CIVIC_2022")), CIVIC)
    self.assertIsNone(select_profile(make_cp("HONDA_CIVIC_2022", "cvt")), "the 6MT plan doesn't serve a CVT Civic")
    self.assertIsNone(select_profile(make_cp("TOYOTA_RAV4")))
    self.assertIsNone(select_profile(None))
    self.assertIn("TOYOTA_RAV4", inactive_reason(make_cp("TOYOTA_RAV4")))
    self.assertIn("not identified", inactive_reason(None))

  def test_adding_a_car_is_a_profile_plus_registration(self):
    self.assertIsNone(select_profile(make_cp(STUB_FP)), "the stub is test-only, never registered")
    profiles = PROFILES + (STUB,)
    self.assertIs(select_profile(make_cp(STUB_FP), profiles), STUB)
    self.assertIs(select_profile(make_cp("HONDA_CIVIC_2022"), profiles), CIVIC)
    self.assertIsNone(select_profile(make_cp("TOYOTA_RAV4"), profiles))

  def test_forced_profile(self):
    os.environ[PROFILE_ENV] = "civic_6mt"
    try:
      self.assertIs(select_profile(make_cp("TOYOTA_RAV4")), CIVIC)
      os.environ[PROFILE_ENV] = "no_such_car"
      self.assertIsNone(select_profile(make_cp("HONDA_CIVIC_2022")))
    finally:
      del os.environ[PROFILE_ENV]

  def test_civic_profile_is_complete(self):
    self.assertEqual(set(ROLES) - {"cruise_buttons", "start_button"} <= set(CIVIC.signals.sources) | {"gear"}, True)
    for field in CIVIC.signals.columns:
      self.assertIn(field, SAMPLE_FIELDS)
    self.assertEqual(CIVIC.signals.columns, {"gear_raw": "gear_mt", "raw_payload": "raw_17c", "gas_raw": "pedal_gas"})
    self.assertIn(CIVIC.buttons.default, CIVIC.buttons.allowed)
    self.assertEqual(CIVIC.powertrain.num_gears, 6)
    self.assertNotIn(OP_LONG_PARAM, CIVIC.allowlist)
    self.assertTrue(set(CIVIC.watched_toggles) <= set(CIVIC.allowlist))


class TestStubProfile(OpenpilotTestCase):
  def test_core_runs_a_second_car(self):
    root, console = stub_drive()
    results = json.load(open(os.path.join(root, "results.json")))
    self.assertEqual(results["B1"]["status"], "PASS", results["B1"])
    self.assertTrue(results["B1"]["complete"])
    r1 = results["R1"]   # background telemetry, binned by the stub's own 5-speed inference
    self.assertEqual(r1["status"], "CAPTURED")
    for g in (1, 2, 3, 4):
      self.assertIn(f"g{g} ratio {POWERTRAIN.overall_ratio(g):.2f}", r1["detail"])
    with open(os.path.join(root, "markers.jsonl")) as f:
      markers = [json.loads(line) for line in f]
    starts = [m for m in markers if m["action"] == "start" and m["test"] == "B1"]
    self.assertEqual(len(starts), 1, "the stub's button started it; the Civic's gap button is not this car's")
    self.assertIn("gear_raw", starts[0]["signals"], "no column map: the core's role names")
    caps = [n for n in os.listdir(os.path.join(root, "captures")) if "_B1_" in n]
    with open(os.path.join(root, "captures", caps[0])) as f:
      header = json.loads(f.readline())
    self.assertEqual(header["fields"], ["t", "v_ego", "long_active", "accel_cmd"])

  def test_stub_conditions_use_its_names(self):
    ctx = Context(profile=STUB)
    from openpilot.sunnypilot.selfdrive.test_console.conditions import condition_label
    self.assertEqual(condition_label("raw_gear_live", ctx), "STUB_GEAR tap live")
    self.assertEqual(condition_label("raw_gear_in_gear", ctx), "STUB_GEAR 1-5")
    self.assertEqual(condition_label("raw_gear_in_gear", Context(profile=CIVIC)), "GEAR_MT 1-6")

  def test_button_map(self):
    b = StartButton("gapAdjustCruise", "short", buttons=STUB.buttons)   # not this car's: falls back to its default
    self.assertEqual(b.type, int(ButtonType.accelCruise))
    self.assertEqual(STUB.buttons.label(""), "RES+")
    self.assertEqual(CIVIC.buttons.label(None), "gap")
    self.assertEqual(StartButton("lkas", buttons=CIVIC.buttons).type, int(ButtonType.lkas))


class FakeParams:
  def __init__(self):
    self.writes: list = []

  def put(self, k, v):
    self.writes.append((k, v))

  def put_bool(self, k, v):
    self.writes.append((k, bool(v)))


class TestAllowlistPerProfile(OpenpilotTestCase):
  def test_allowlist_follows_the_profile(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.dev_test_params import ParamNotAllowed, write_param
    self.assertEqual(ui_writable(STUB), {"NonAdaptiveLongitudinal"} | set(CONSOLE_PARAMS))
    self.assertEqual(ui_writable(None), set(CONSOLE_PARAMS), "no profile: only the console's own keys")
    self.assertNotIn(OP_LONG_PARAM, ui_writable(dataclasses.replace(STUB, allowlist=(OP_LONG_PARAM,))), "never op long")
    p = FakeParams()
    write_param(p, "NonAdaptiveLongitudinal", True, profile=STUB)
    write_param(p, "TestConsoleArmed", True, profile=None)
    for key, profile in (("ClutchShiftGate", STUB), ("NonAdaptiveLongitudinal", None), (OP_LONG_PARAM, CIVIC)):
      with self.assertRaises(ParamNotAllowed, msg=key):
        write_param(p, key, True, profile=profile)
    self.assertEqual(p.writes, [("NonAdaptiveLongitudinal", True), ("TestConsoleArmed", True)])


class _Stop(Exception):
  pass


class TestDaemonInactive(OpenpilotTestCase):
  def test_no_profile_subscribes_and_writes_nothing(self):
    from openpilot.sunnypilot.selfdrive.test_console import test_consoled as d
    cp_bytes = make_cp("TOYOTA_RAV4").to_bytes()

    class P:
      def __init__(self, *a, **kw):
        pass

      def get(self, key, block=False, return_default=False):
        return cp_bytes if key == "CarParams" else None

    def forbidden(*a, **kw):
      raise AssertionError("inactive console must not subscribe or open its store")

    def stop(*a, **kw):
      raise _Stop

    saved = (d.Params, d.messaging.SubMaster, d.messaging.sub_sock, d.Store, d.time.sleep)
    d.Params, d.messaging.SubMaster, d.messaging.sub_sock, d.Store, d.time.sleep = P, forbidden, forbidden, forbidden, stop
    root = tempfile.mkdtemp()
    os.environ["TEST_CONSOLE_DIR"] = root
    try:
      with self.assertRaises(_Stop):
        d.main()
    finally:
      d.Params, d.messaging.SubMaster, d.messaging.sub_sock, d.Store, d.time.sleep = saved
      del os.environ["TEST_CONSOLE_DIR"]
    self.assertEqual(os.listdir(root), [], "nothing written")

  def test_carparams_change_ends_the_active_console(self):
    from openpilot.sunnypilot.selfdrive.test_console import test_consoled as d
    civic = make_cp("HONDA_CIVIC_2022")
    calls = {"n": 0}

    class P:
      def get(self, key, block=False, return_default=False):
        if key == "CarParams":
          calls["n"] += 1
          return make_cp("TOYOTA_RAV4").to_bytes()   # a different car since the console started
        return None

      def get_bool(self, key, block=False):
        return False

    class SM:
      def __init__(self, *a, **kw):
        self.updated = {"carState": False}

      def update(self, *a):
        pass

    saved = (d.messaging.SubMaster, d.messaging.sub_sock)
    d.messaging.SubMaster, d.messaging.sub_sock = SM, lambda *a, **kw: None
    os.environ["TEST_CONSOLE_DIR"] = tempfile.mkdtemp()
    try:
      d.run_console(P(), civic, civic.to_bytes(), CIVIC)   # returns instead of running on with the old profile
    finally:
      d.messaging.SubMaster, d.messaging.sub_sock = saved
      del os.environ["TEST_CONSOLE_DIR"]
    self.assertEqual(calls["n"], 1)
