# EPS parked-bench harness — build report for evaluation

**Prepared for:** the requestor / firmware-RE team (evaluate against `SP_HARNESS_DIRECTIVE.md`).
**Repo:** `commaai/openpilot` master `ff47f484e`, branch `eps-bench-parked`, commit `99f311f75`.
**Status:** code-complete; **not yet built or run on a device** (verification status in §7).
**Scope reminder:** PARKED bench tool. Injects a rate-limited `0x0E4` steering command and
logs all CAN. No autonomy, no longitudinal, no road use.

---

## 1. Does it meet the goals? (directive → conformance)

| Directive requirement | Met | Where / how |
|---|---|---|
| §1 Inject rate-limited steer parked + log all CAN + 2-button UI | ✅ | `steer_injector.py`, `can_logger.py`, `eps_menu.py` / `eps_bench_ui.py` |
| §3 Run inside SP via cereal; **no exclusive panda access** | ✅ | inject via `sendcan`, log via `can`; both are plain cereal clients |
| §3 Per-test params in editable JSON, not compiled | ✅ | `config.json` (all knobs); scripts read it at start |
| §4 `0x0E4` built with checksum + rolling counter via opendbc packer @~100 Hz | ✅ | `CANPacker.make_can_msg("STEERING_CONTROL", 0, …)` |
| §4 Injector is the sole steer source (no command fight) | ✅ | autonomy/`sendcan` procs stripped in bench mode (§5 below) |
| §4 Ramp/dwell/±/N-cycle profile; `TARGET`/`MAX_RATE`/`MAX_CMD`; ramp-to-0 on stop | ✅ | `build_segments()` + rate limiter + ceiling + graceful stop |
| §5 Logger: all buses, `ts,bus,addr,len,data` CSV, timestamped file, concurrent | ✅ | `can_logger.py` |
| §6 Two toggles + status + prominent STOP; SSH fallback allowed | ✅ | `eps_bench_ui.py` (on-screen) + `eps_menu.py` (SSH, validated) |
| §7 Disable controlsd lateral; bypass ACC gate for lateral; permit steer TX parked; strip to min procs; no longitudinal | ✅ | stronger: controlsd/card not run at all; ALLOUTPUT; `process_config.py` strip |
| §8 Speed interlock (hard, abort+ramp-to-0 >~0.5 kph) | ✅ | latching abort on `0x158` speed, stale, or unknown |
| §8 Rate limit + hard ceiling; session timeout; instant abort → ramp to 0 | ✅ | all in `steer_injector.py` |
| §10 Hand-back: raw CAN log + command log + what to report | ✅ | CSVs + §8 of `README.md` |
| §11 No driving/longitudinal/lane/road; no EPS fw change; no panda-safety widening beyond need; reversible; stop-and-report if ignored parked | ✅ | see §2–§6 below; stop-and-report is operator procedure, injector never escalates on its own |
| §0 Don't assume/search EPS behavior; ASK | ✅ | used only firmware-verified facts; open items listed in §6 |

**Bottom line:** every in-scope deliverable and every §8 safety requirement is implemented, with
two isolated, reversible, env-gated build changes and zero panda-safety-rule or EPS-firmware edits.

---

## 2. Base / fork decision (and why)

**Chosen: upstream `commaai/openpilot` master**, not the sunnypilot/MVL fork the car normally runs.

Reasoning — the tool's value is a *clean, canonical* foundation the firmware team can reason against:
1. **Unmodified panda safety.** comma's Honda steer gate is a plain `if (!controls_allowed)`; the
   MVL/sunnypilot fork wraps it in `controls_allowed || controls_allowed_lateral` + a MADS state
   machine. For a tool handing CAN traces to a firmware team, fork-specific safety deltas next to the
   `0x0E4` path are noise. Give them stock semantics.
2. **Minimalism.** The directive says strip to the minimum and "not a general fork." sunnypilot is a
   *superset* carrying exactly the machinery we remove; comma master is already leaner.
3. **No cost on the reused piece.** The Honda CAN packer / `0x0E4` DBC / CANPacker are in opendbc and
   identical in both, so choosing comma costs nothing where it matters.
4. The device's current AGNOS (19.7) was deliberately *not* weighed — the bench rig gets its own flash
   and never touches the cruise build.

**Isolation:** branch `eps-bench-parked`, kept in `~/EPS_testing/openpilot` (off the Dropbox mount).
**Never merged** into the civic-cruise cruise branches (`sp-honda-dev-202608`/`civic-6mt-dev-test`) or
any base. The car is reflashed back to its normal build after bench work.

---

## 3. What changed in the SP source (the only two edits)

Both are gated on the `EPS_BENCH` environment variable, set **only** by `launch_eps_bench.sh`. With it
unset the build is byte-for-byte stock (important: this is why it's safe to carry on a branch).

### 3a. `openpilot/selfdrive/pandad/panda_safety.cc` — hold `SAFETY_ALLOUTPUT (param 0)`
**How:** at the top of `PandaSafety::configureSafetyMode()`, if `getenv("EPS_BENCH")` and the device is
onroad, call `panda_->set_safety_model(ALL_OUTPUT, 0U)` once and return — bypassing the normal
`CarParams`-driven safety setter and the OBD/ELM327 multiplexing step.

**Why this one line does everything the test needs (traced in the panda firmware):**
- `alloutput_init()` sets `controls_allowed = true` → the panda permits `0x0E4` TX **at standstill**
  (stock Honda safety would drop it: `honda.h` blocks `0x0E4` unless `controls_allowed`).
- ALLOUTPUT is not SILENT/NOOUTPUT/ELM327, so `set_safety_mode()` takes the `default:` branch →
  `set_intercept_relay(true, false)` → the **physical relay opens**. This isolates the stock camera's
  bus-2 `0x0E4` from the EPS — directly answering the relay caveat: no command fight.
- `param 0` → `disable_forwarding = true` → the panda also does **not** bridge the camera's `0x0E4`
  from bus 2 onto bus 0. (Passthrough, `param 1`, would bridge them — not what we want.)
- Opening the relay isolates only the camera; the car's own F-CAN (bus 0) keeps carrying speed
  `0x158`, sensor validity and `0x18F` natively, engine-on — satisfying the directive's "keep stock
  traffic alive" so the EPS's off-image cross-checks are fed.
- ALLOUTPUT needs no `CarParams`/`ControlsReady`, so `controlsd`/`card` need not run (stronger than
  §7.1's "disable lateral output").

**Why ALLOUTPUT over a Honda-safety patch (your §7.3 choice):** smaller and fully reversible — it
edits no safety *rule*, and `SAFETY_ALLOUTPUT` already exists for exactly this bench purpose. The
injector's own limits (§5) are the real safety envelope, as the directive intends. ALLOUTPUT is
compiled only on the **debug** panda build (`ALLOW_DEBUG`, the default when `RELEASE` is unset); it is
absent from release firmware, which is itself a safeguard against leaving it on.

**Heartbeat note (traced):** pandad sends panda heartbeats at 10 Hz *unconditionally* (not gated on
`enabled`), so ALLOUTPUT will not decay to SILENT; it only needs the device onroad (engine running =
ignition). On exit or loss of ignition, pandad closes the relay (→ NO_OUTPUT) — a safe default.

### 3b. `openpilot/system/manager/process_config.py` — strip to the minimum running set
**How:** after the process list is built, if `getenv("EPS_BENCH")`, set `enabled = False` on every
autonomy / control / model / `sendcan`-publishing process: `controlsd, plannerd, selfdrived, card,
radard, modeld, dmonitoring*, navd, mapsd, locationd, paramsd, lagd, torqued, calibrationd,
maneuvers*, joystick(d), bridge, webrtcd`.

**Why:** two reasons. (1) `card` **and** `controlsd` both publish `sendcan`; msgq allows a single
publisher per topic, so if either runs our injector can't publish (or one dies). They must be off for
the injector to be the sole steer source (§4). (2) It realizes §7.4's "strip to the minimum" — nothing
is left that commands steering or longitudinal. Kept: `pandad` (sendcan→panda + `can` + heartbeat),
`hardwared` (ignition→onroad, which pandad needs to apply ALLOUTPUT), and base logging/system procs.

No edits to panda firmware, `opendbc/safety`, the Honda safety hook, or any EPS firmware.

---

## 4. What was added (new, self-contained; `eps_bench/`)

- **`steer_injector.py`** — builds `STEERING_CONTROL` (0x0E4) via `CANPacker` (auto COUNTER + CHECKSUM),
  `STEER_TORQUE` + `STEER_TORQUE_REQUEST=1`, publishes to `sendcan` on **bus 0** at `RATE_HZ`. Profile:
  `0 → +TARGET → dwell → 0 → −TARGET → dwell → 0`, ×`N_CYCLES`. The sweep setpoint is computed
  analytically and then passed through the rate limiter, so even a mis-set `RAMP_S=0` cannot produce a
  step. Writes `cmdlog_*.csv` (ts, phase, setpoint, commanded torque, request, speed).
- **`can_logger.py`** — subscribes `can`, writes every frame from all buses to `canlog_*.csv`
  (`monotonic_ts,bus,addr_hex,len,data_hex`) with a wall-clock + watch-list header (`0x18F` and the
  EPS-TX IDs). Independent of the injector.
- **`eps_menu.py`** (SSH fallback, validated) / **`eps_bench_ui.py`** (on-screen raylib) — start/stop the
  two workers, show status, and a STOP/ABORT that requests a **graceful ramp-to-0** (never an instant cut).
- **`config.json`** — all knobs, documented inline, conservative first-run defaults.
- **`common.py`** — config load, JSON status/abort IPC, and the DBC-free `0x158` speed parse.
- **`launch_eps_bench.sh`** — sets `EPS_BENCH=1` and launches the stripped stack (the child pandad
  inherits the env).

---

## 5. Safety envelope (how each §8 item is enforced, all in `steer_injector.py`)

- **Hard speed interlock** — reads `0x158` (bus 0, bytes 4-5 BE × 0.01 kph, firmware-verified layout,
  deliberately kept DBC-free). Commands only when speed ≤ `SPEED_ABORT_KPH` **and** fresh within
  `SPEED_STALE_S`. Exceeding the threshold, *or* stale, *or* never-seen speed → **latches** an abort
  (no resume) and ramps to 0. Unknown speed is treated as unsafe by construction.
- **Rate limit (anti-slam)** — per-frame delta clamped to `MAX_RATE / RATE_HZ`; never a step.
- **Hard magnitude ceiling** — `|cmd| ≤ MAX_CMD` always, independent of `TARGET`.
- **Session timeout** — `SESSION_TIMEOUT_S` watchdog → self-stop (ramp to 0).
- **Instant abort** — UI abort file, SIGINT/SIGTERM, or any exception → latch abort, ramp to 0, then
  send explicit `STEER_TORQUE=0, STEER_TORQUE_REQUEST=0` release frames, then stop. All failure modes
  bias to zero command, never to more torque.
- **Operator stop-and-report** — the injector never raises magnitude on its own; `TARGET`/`MAX_CMD` are
  only ever changed by a human editing `config.json` between runs. If the EPS ignores `0x0E4` parked,
  the procedure (README) is to stop and report — the tool does not escalate to force it.

---

## 6. Firmware-verified facts used, and the open ASK items

**Used (from you; not assumed, not web-searched):**
- Inject `0x0E4` on **bus 0** (EPS on F-CAN); camera's `0x0E4` is on bus 2 and must be relay-isolated.
- `0x18F` STEER_STATUS is the LKAS-active / handback / `low_speed_lockout` tell-tale (enum present in
  the DBC) — flagged in the logger header for the analysis.
- Speed interlock from `0x158` bus 0, bytes 4-5 BE × 0.01 kph.
- `0x0E4` `STEER_TORQUE` full-scale ±4096; your cap landscape (~34%≈1393, ~17%≈696, ~50%≈2048) used
  only to size conservative defaults and document the sweep range — **not hardcoded**.
- Standstill-honor is the test's output; keep engine running + stock bus-0 traffic alive.

**Still to confirm before injecting (please verify / answer):**
1. From your own passive log with the panda in **ALLOUTPUT**: relay reads open (stock camera `0x0E4`
   stops appearing on bus 0) and bus-0 speed/`0x18F` traffic is live engine-on parked.
2. Does the EPS require the `0x0E4` `STEER_DOWN_TO_ZERO` (TJA) bit set to act on the command? Left
   **off** by default (`SET_STEER_DOWN_TO_ZERO=false`) as the stock non-TJA frame; this is EPS behavior,
   so I did not assume it.
3. Confirm the `0x158` byte layout decodes to true speed on this specific car (we also log raw `0x158`).

---

## 7. Verification status (honest)

**Done (on the PC):**
- All five Python modules byte-compile (`py_compile`).
- API/behavior **traced in the actual source**, not assumed: the panda relay/forwarding/`controls_allowed`
  behavior of ALLOUTPUT (`opendbc/safety` + `panda/board/main.c` `set_safety_mode` default branch); the
  `sendcan`→pandad→panda TX path and the `can` publish; pandad's unconditional heartbeat; the `0x0E4`
  and `0x18F` definitions and `STEER_TORQUE ±4096` in the Honda radarless DBC; the `CANPacker` /
  `messaging` / `can_list_to_can_capnp` signatures; that `card` and `controlsd` both publish `sendcan`.

**NOT done (requires the device / a build — all marked `TODO(on-car)`):**
- The C++ change (`panda_safety.cc`) is **not compiled**; needs a device build of pandad.
- Nothing has been run on a car or against a live panda; no on-device import of opendbc/cereal.
- The raylib on-screen layout is unverified and needs the stock `ui` process stopped first (the SSH
  `eps_menu.py` needs no screen and is the recommended first UI).
- Items in §6 and the README `TODO(on-car)` list.

---

## 8. Hand-back package (what the firmware team receives, §10)
- `canlog_*.csv` (all buses) + `cmdlog_*.csv` (commanded `STEER_TORQUE` vs time).
- Report: which bus `0x0E4` is on; whether motor torque appeared at standstill at all; any DTC; raw
  traces of `0x0E4` and the EPS-TX status IDs (`0x18F`, `0x324/326/328/32F/33D/374/3A1/516/78E`) around
  the ramp. The firmware team does the analysis (torque CLIPS vs TRACKS the rising command;
  time-to-handback; validity/DTC); the harness only produces clean, labeled logs.

## 9. How to review
- Code: `eps_bench/` + the two patches `openpilot/selfdrive/pandad/panda_safety.cc` and
  `openpilot/system/manager/process_config.py` (commit `99f311f75`, branch `eps-bench-parked`).
- Start with `eps_bench/README.md` (operator guide) and this report.
- `git show 99f311f75 -- openpilot/selfdrive/pandad/panda_safety.cc openpilot/system/manager/process_config.py`
  shows the entire footprint of changes to the SP stack.
