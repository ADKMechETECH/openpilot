"""
A stub vehicle profile (TEST-ONLY, not shipped, not registered in profiles.PROFILES): an imaginary 5-speed manual with no
raw-CAN tap, a different start button and a two-test plan. It exists to exercise the core / profile seam — the core
runs it unchanged, it needs nothing but this file plus a fingerprint, and nothing in it is Civic.
"""
import math
import tempfile

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.sunnypilot.selfdrive.test_console.analyzers import CAPTURED, SegmentAnalyzer
from openpilot.sunnypilot.selfdrive.test_console.button import StartButton
from openpilot.sunnypilot.selfdrive.test_console.capture import Console, Store
from openpilot.sunnypilot.selfdrive.test_console.conditions import Context
from openpilot.sunnypilot.selfdrive.test_console.profile import ButtonMap, Powertrain, SignalMap, VehicleProfile
from openpilot.sunnypilot.selfdrive.test_console.registry import OP_LONG_PARAM, Kind, Phase, Registry, Session, TestSpec
from openpilot.sunnypilot.selfdrive.test_console.signals import LogEventTap, SampleBuilder

FINGERPRINT = "STUB_CAR_5MT"
GEARS = (1, 2, 3, 4, 5)

PHASES = {0: Phase(0, "Stub checks", "bench", "nothing")}
TESTS = (
  TestSpec("B1", 0, Kind.FEATURE, "Stub baseline", ("Drive engaged on op long for 10 s.",),
           ("op_long", "long_active"), "segments", {"seconds": 10.0, "min_segment": 0.0},
           "baseline", "no ACC fault, no blocked TX, no fault alerts", ("t", "v_ego", "long_active", "accel_cmd"),
           toggles={"NonAdaptiveLongitudinal": True}, session="1"),
  TestSpec("R1", 0, Kind.TELEMETRY, "Stub gear ratios", ("Pull through gears 1-5, clutch out, 2 s each.",),
           ("manual", "rolling", "in_gear", "clutch_out"), "segments",
           {"bin": "gear", "keys": list(GEARS), "seconds": 2.0, "min_segment": 0.5},
           "stub_ratios", "", ("t", "v_ego", "rpm", "gear", "ratio", "gear_raw"), session="1"),
)
SESSIONS = (Session("1", "Stub session", "Both stub tests", ((OP_LONG_PARAM, True), ("NonAdaptiveLongitudinal", True)),
                    ("B1", "R1")),)


def r_stub_ratios(a: SegmentAnalyzer):
  parts = [f"g{g} ratio {a.pooled('ratio', g):.2f}" for g in a.keys if a.by_key(g)]
  return (CAPTURED, "; ".join(parts)) if parts else (None, "")


def _clutch(CS) -> bool:
  return bool(CS.deprecated.clutchPressed)


def _rpm(CS) -> float:
  return float(CS.deprecated.engineRpm)


POWERTRAIN = Powertrain(gear_ratios=(3.5, 2.0, 1.3, 1.0, 0.8), final_drive=4.0, tire_radius=0.30, match_tol=0.08)

PROFILE = VehicleProfile(
  name="stub_5mt",
  title="Stub 5MT",
  applies=lambda CP: CP.carFingerprint == FINGERPRINT,
  fingerprints=(FINGERPRINT,),
  signals=SignalMap(clutch=_clutch, engine_rpm=_rpm, labels={"gear_raw": "STUB_GEAR"}),
  powertrain=POWERTRAIN,
  buttons=ButtonMap(default="accelCruise", allowed=("accelCruise",), short={"accelCruise": "RES+"}),
  registry=Registry(PHASES, TESTS, SESSIONS, first_run_note="Stub: nothing to characterize."),
  allowlist=("NonAdaptiveLongitudinal",),
  watched_toggles=("NonAdaptiveLongitudinal",),
  results={"stub_ratios": r_stub_ratios},
)


ButtonType = structs.CarState.ButtonEvent.Type


def stub_drive(test_id: str = "B1") -> tuple[str, Console]:
  """The stub car through the unchanged core: SampleBuilder -> StartButton -> Console, pulling through gears 1-5 engaged."""
  root = tempfile.mkdtemp()
  ctx = Context(profile=PROFILE, manual=True, op_long=True, toggles={"NonAdaptiveLongitudinal": True})
  console = Console(Store(root), ctx)
  console.set_control(test_id, True, True)
  builder, log, button = SampleBuilder(PROFILE), LogEventTap(), StartButton(buttons=PROFILE.buttons)
  cc, co = messaging.new_message("carControl").carControl, messaging.new_message("carOutput").carOutput
  cc.enabled = cc.longActive = True
  plan, radar, ss = (messaging.new_message(w) for w in ("longitudinalPlan", "radarState", "selfdriveState"))
  t, v, s = 100., 4., None
  for i in range(1300):
    t += 0.01
    gear = min(1 + i // 250, 5)
    v += 0.004
    cs = messaging.new_message("carState").carState
    cs.vEgo, cs.deprecated.engineRpm = v, v / POWERTRAIN.tire_radius * POWERTRAIN.overall_ratio(gear) * 60. / (2. * math.pi)
    events = []
    if i in (5, 6):   # the stub's start button (a Civic gap press would do nothing here)
      events = [structs.CarState.ButtonEvent(pressed=i == 5, type=ButtonType.accelCruise)]
    if i in (20, 21):
      events = [structs.CarState.ButtonEvent(pressed=i == 20, type=ButtonType.gapAdjustCruise)]
    s = builder.build(t, cs, cc, co, plan.longitudinalPlan, True, radar.radarState, ss.selfdriveState, 0, None, log, ())
    console.update(s, button.update(t, events))
  console.shutdown()
  return root, console
