# EPS parked-bench harness (`eps_bench/`)

A minimal, **PARKED-ONLY** bench tool to inject a rate-limited steering command
(`0x0E4` STEERING_CONTROL) into a 11th-gen Honda Civic EPS and log all CAN, so a
separate firmware-RE team can characterize the stock EPS steering limits. Built
per `SP_HARNESS_DIRECTIVE.md`.

> **This is NOT a driveable system.** No autonomy, no longitudinal, no lane model,
> no road use. Engine-on, parked, wheels free, hands clear of the wheel spokes,
> kill/relay reachable. If the EPS ignores `0x0E4` at standstill: **STOP and report**
> — do not escalate magnitude or limits to force it (directive §8/§11).

## Base / isolation
- Forked from **upstream `commaai/openpilot` master** (canonical, unmodified panda
  safety — the right foundation to hand a firmware team), branch **`eps-bench-parked`**.
- Lives only here (`~/EPS_testing/openpilot`). **Never merge** into the civic-cruise
  cruise branches (`sp-honda-dev-202608` / `civic-6mt-dev-test`) or any base. Reflash
  the car back to its normal build after bench work.

## Firmware-verified facts used (from the team; see directive §0 — ASK, don't assume)
- **Inject `0x0E4` on bus 0** (F-CAN, where the EPS lives). The stock camera sends
  `0x0E4` on bus 2; it must be isolated by the open relay (below). Bus 1 unconnected.
- **`0x18F` STEER_STATUS** (EPS TX) is the LKAS-active / handback / `low_speed_lockout`
  tell-tale to watch in the log. Enum in the DBC: `normal / driver_steering /
  no_torque_alert_1 / low_speed_lockout / no_torque_alert_2 / fault_1 / tmp_fault /
  permanent_fault`.
- **Speed interlock source**: `0x158` bus 0, bytes 4-5 big-endian × 0.01 kph.
- `0x0E4` full-scale `STEER_TORQUE` is ±4096 (16-bit signed). Firmware reference only
  (do **not** hardcode — sweep and observe): external cap ~34% ≈ 1393, reduced
  sub-mode ~17% ≈ 696, physical motor ceiling ~50% ≈ 2048.
- **Standstill-honor is the test's OUTPUT.** Keep the engine running and stock bus-0
  traffic alive (speed/sensor validity flows naturally); add only `0x0E4`
  (`STEER_TORQUE_REQUEST=1`, ~100 Hz); watch `0x18F` + the EPS-TX IDs for a response.
  Null result parked ⇒ off-image standstill gate ⇒ report; next step is a separate
  low-speed plan.

### Still to ASK the firmware team
- Should the `0x0E4` `STEER_DOWN_TO_ZERO` bit (TJA) be set for the EPS to act, or is
  the plain non-TJA frame correct? (`SET_STEER_DOWN_TO_ZERO`, default **false**.)
- Confirm, from their own passive log with the panda in ALLOUTPUT, that the relay
  reads **open** (stock camera `0x0E4` stops appearing on bus 0) and bus-0
  speed/`0x18F` traffic is still live engine-on parked.

## Deliverables
| File | Role |
|---|---|
| `steer_injector.py` | D1: rate-limited `0x0E4` injector → `sendcan` (sole steer source) |
| `can_logger.py` | D2: subscribe `can`, log **all buses** → timestamped CSV |
| `eps_menu.py` | D3 fallback: SSH text menu (two toggles + STOP) — **validated** |
| `eps_bench_ui.py` | D3 preferred: on-screen raylib two-button + STOP (on-car-verify) |
| `config.json` | all per-test knobs (never compiled in) |
| `common.py` | config / status-IPC / firmware-verified speed parse |
| `launch_eps_bench.sh` | sets `EPS_BENCH=1` and launches the stripped stack |

## Architecture (runs inside the SP stack via cereal; no exclusive panda access)
- **Inject**: publish `0x0E4` frames (CANPacker: bit layout + rolling COUNTER +
  CHECKSUM) to **`sendcan`**; pandad TXes them to the panda. Our injector is the only
  `sendcan` publisher because the autonomy/control procs (incl. `card`/`controlsd`,
  which also publish `sendcan`) are stripped — see below.
- **Log**: subscribe **`can`** (pandad publishes every frame from all buses). No
  contention; both processes are plain cereal clients.

## One-time build changes (minimal, isolated, reversible)
Both are gated on `EPS_BENCH` (set only by `launch_eps_bench.sh`); with it unset the
build is byte-for-byte stock.

1. **`openpilot/selfdrive/pandad/panda_safety.cc`** — in `EPS_BENCH` mode, hold
   `SAFETY_ALLOUTPUT (param 0)` instead of the car's safety model. This single choice:
   - sets `controls_allowed = true` → `0x0E4` TX allowed at standstill;
   - opens the **physical relay** (ALLOUTPUT hits the `default:` branch of
     `set_safety_mode` → `set_intercept_relay(true, false)`), isolating the camera's
     bus-2 `0x0E4` from the EPS;
   - `param 0` → `disable_forwarding = true` → the camera's `0x0E4` is **not** bridged
     onto bus 0 either (no command fight);
   - needs no `CarParams`/`ControlsReady`, so `controlsd`/`card` need not run.
   The injector's own limits (speed interlock + rate limit + `MAX_CMD`) are the real
   safety envelope. **Debug panda build only** (`ALLOW_DEBUG`, the default when
   `RELEASE` is unset) — `SAFETY_ALLOUTPUT` is compiled out of release firmware.
2. **`openpilot/system/manager/process_config.py`** — in `EPS_BENCH` mode, disable
   every autonomy / control / model / `sendcan`-publishing process (controlsd,
   plannerd, selfdrived, card, radard, modeld, dmonitoring*, locationd, paramsd, lagd,
   torqued, calibrationd, maneuvers, joystick, bridge, webrtcd). Keep `pandad`
   (sendcan→panda + `can` + heartbeat) and `hardwared` (ignition→onroad, which pandad
   needs to apply ALLOUTPUT) plus base logging/system procs.

   > pandad sends panda heartbeats at 10 Hz unconditionally, so ALLOUTPUT won't decay
   > to SILENT; it just needs the device "onroad" (engine running = ignition on).
   > On exit / loss of ignition, pandad closes the relay (→ NO_OUTPUT) — safe default.

No panda firmware / safety-rule edits, no EPS firmware changes.

## Run (on the device, engine running, PARKED, wheels free, hands clear)
```bash
# 1) launch the stripped ALLOUTPUT stack
eps_bench/launch_eps_bench.sh
# 2) in a second SSH session, the operator UI (text fallback, no screen needed)
python3 eps_bench/eps_menu.py       #  [B] CAN LOG first, then [A] STEER RAMP, [S] to abort
```
Edit `eps_bench/config.json` between runs to raise `TARGET` / `MAX_CMD` deliberately.
**Start small.** Short dwell, pause between cycles (dry-steer = max motor current +
tire scrub; a current/thermal DTC near the cap is useful data, not a failure).

## Safety (all enforced in `steer_injector.py`, §8)
- HARD speed interlock: commands only when `0x158` speed ≤ `SPEED_ABORT_KPH` **and**
  fresh; exceeding it, or stale/unknown speed, **latches** an abort and ramps to 0.
- Rate limit (anti-slam): per-frame delta ≤ `MAX_RATE`/`RATE_HZ`; never a step.
- Hard magnitude ceiling `MAX_CMD`, independent of `TARGET`, never exceeded.
- Global `SESSION_TIMEOUT_S` → self-stop.
- Instant abort (UI / SIGTERM / any exception) → ramp to 0, `STEER_TORQUE_REQUEST=0`,
  then stop. Never an instant cut.

## Hand back to the firmware-RE team (directive §10)
- The raw CAN log (`canlog_*.csv`, all buses) + the injector command log
  (`cmdlog_*.csv`: ts, phase, setpoint, commanded `STEER_TORQUE`, request, speed).
- Report: which bus `0x0E4` is on; whether motor torque appeared at standstill at all
  (the speed-gate answer); any DTC; raw traces of `0x0E4` and the EPS-TX status IDs
  (`0x18F`, `0x324/326/328/32F/33D/374/3A1/516/78E`) around the ramp.
- They do the analysis (motor torque CLIPS below the rising command = cap binding, vs
  TRACKS it; time-to-handback; validity/DTC). We produce clean, labeled logs.

## `TODO(on-car)`
- Confirm relay-open + live bus-0 traffic in ALLOUTPUT from a passive log before injecting.
- Confirm `STEER_DOWN_TO_ZERO` / TJA requirement with the firmware team.
- Verify the on-screen raylib layout + that the stock `ui` process is stopped first.
- Confirm `0x158` byte layout decodes to true speed on this car.
