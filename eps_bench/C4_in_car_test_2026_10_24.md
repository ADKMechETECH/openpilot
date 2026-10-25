# C4 in-car EPS bench test — session handoff (2026-10-24)

Pick-up note for the parked EPS steering-characterization bench. **The C4 is built, loaded,
and ready; no testing has been run yet.** Next time, start at "NEXT STEPS."

---

## STATUS: ready to run (nothing injected yet)
- Harness is **code-complete, pushed, loaded on the car, and built**. AGNOS updated.
- The device is sitting offroad running *normal* comma-master openpilot (that's expected).
- Not done: the actual parked passive capture (Stage 1) and ramp (Stage 2).

## Where everything lives
- **WSL repo (full build):** `~/EPS_testing/openpilot`, branch `eps-bench-parked`, tip **`c7f6f0859`**.
  Base = commaai/openpilot master `ff47f484e`. Remote `adk` = `ADKMechETECH/openpilot` (push via
  `git push --no-verify adk eps-bench-parked`; `--no-verify` skips the huggingface LFS hook).
- **Dropbox** `…/CommaAI/EPS_Testing/`: `SP_HARNESS_DIRECTIVE.md`, `SP_HARNESS_DIRECTIVE_ADDENDUM_2026-10-24.md`,
  `BUILD_REPORT.md`, `HARNESS_README.md`, `INSTALL_C4.md`, `REPO_POINTER.md`, `openpilot_build/`
  (eps_bench + patches + APPLY.md), and this file.
- **Harness code:** `eps_bench/` at repo root (`/data/openpilot/eps_bench/` on the car).
- **Two SP-source edits (only ones), both gated on `EPS_BENCH` env (stock when unset):**
  `openpilot/selfdrive/pandad/panda_safety.cc` (hold SAFETY_ALLOUTPUT param 0) and
  `openpilot/system/manager/process_config.py` (strip autonomy/sendcan procs).

## Device state (comma four @ 192.168.0.150, "comma-16096f0a")
- `/data/openpilot` is on branch **`eps-bench-parked`** @ `c7f6f0859` (switched IN PLACE from the
  cruise branch — there is **no** `openpilot.cruise.bak` anymore; the backup was restored then the
  same repo was checked out to the bench branch).
- **Cruise branch to restore later = `civic-6mt-dev-test`.**
- **AGNOS was updated 19.7 → 20.1** (comma master required it). This persists. When the cruise fork
  is restored it will rebuild on 20.1 (was validated on 19.7 — watch for any friction then).
- Harmless leftover: `?? openpilot/sunnypilot/` (untracked dir from the cruise branch; not in comma
  master; ignore, or `rm -rf openpilot/sunnypilot`).
- Build verified: `pandad` running (C++ `./pandad` + python launcher). Binary is at the NESTED path
  `/data/openpilot/openpilot/selfdrive/pandad/pandad` (repo has an inner `openpilot/` package dir).
- Device SSH key is **NOT** authorized for GitHub push (publickey denied); the WSL box's key is NOT
  authorized to SSH into the C4. Device git fetch works over its HTTPS `origin`.
- Device Python for manual scripts: `/usr/local/venv/bin/python` with `PYTHONPATH=/data/openpilot`
  (system `python3` lacks zmq/capnp).

## Firmware-verified facts (from directive + addendum — do NOT assume beyond these; ASK)
- Inject `0x0E4` STEERING_CONTROL on **bus 0** (EPS/F-CAN). Camera's `0x0E4` is on bus 2, isolated by
  the open relay. `STEER_TORQUE` full-scale ±4096.
- **ALLOUTPUT gives the relay for free:** relay OPEN + no bus0↔2 forwarding + controls_allowed + no
  CarParams dependency. (Confirmed in panda `set_safety_mode` default branch.)
- EPS actuation gates are **internal**; isolating the camera does NOT suppress actuation; no camera
  context frame needed. Keep engine running so bus-0 speed/sensor health flows.
- **`SET_STEER_DOWN_TO_ZERO=false` for run 1** (basic LKAS assist = firmware "mode 0 / 34%").
- Delivered-torque tell-tale = **`0x18F` STEER_STATUS** + the EPS-TX set
  (`0x324/326/328/32F/33D/374/3A1/516/78E`). Firmware cap = **±0x2b4e = 11086 = 34% on the INTERNAL ±32767 Q15 scale** (= ~1393 in 0x0E4
  ±4096 units, the value TARGET ramps toward); 17% sub-mode is a
  separate `0x55AA`-handshake path the ramp never reaches (mode-word bit 0x800). Run 1 = mode-0 34% cap only.
- **CCP internal reads are NOT in the fork** (firmware team reads internals on their own CAN interface).
- **No fingerprint, no calibration needed** for the bench (card/modeld/calibrationd are stripped;
  pandad sets ALLOUTPUT from the env var; injector loads the DBC by name; logger reads raw CAN).

---

## NEXT STEPS — run the bench (at the car, engine running, PARKED, wheels free, HARDWARE KILL REACHABLE)

Command everything over SSH from the laptop (no C4 screen needed). Use `tmux` so an SSH drop can't
abort the test (the injector also handles SIGHUP with a graceful ramp-to-0).

1. **Stop the normal openpilot that auto-started:**
   ```
   ssh comma@192.168.0.150
   tmux ls                       # find the openpilot session
   tmux kill-session -t comma    # (or: tmux kill-server)
   pgrep -af manager             # confirm empty
   ```
2. **Start the engine** (parked, wheels straight, hands clear of spokes).
3. **Launch the bench stack:**
   ```
   cd /data/openpilot
   tmux new -s eps
   ./eps_bench/launch_eps_bench.sh                                               # window 1 (EPS_BENCH=1 → ALLOUTPUT + stripped stack)
   #  Ctrl-b c →
   PYTHONPATH=/data/openpilot /usr/local/venv/bin/python eps_bench/eps_menu.py   # window 2 (B=CAN LOG, A=STEER RAMP, S=STOP)
   ```
4. **Stage 1 (logger only — NO ramp):**
   - Start **CAN LOG** (B).
   - **Validate `0x158` speed-decode** (the gating safety check): roll slowly, confirm `0x158`
     bytes 4-5 (×0.01 kph) track the speedo/GPS in the log.
   - **Confirm the relay is open** under ALLOUTPUT: the stock camera's `0x0E4` should NO LONGER appear
     on bus 0; bus-0 speed/`0x18F` still live.
   - Optional: arm STEER RAMP while rolling to confirm the interlock **aborts to 0 immediately** (hands ready).
5. **Stage 2 (parked ramp):** park, confirm `0x158` ≈ 0, start **STEER RAMP** at the conservative
   defaults (`config.json`: `TARGET 600 / MAX_CMD 1000`). Watch `0x18F`/EPS-TX torque saturate against
   the rising `0x0E4`. Raise `TARGET`/`MAX_CMD` deliberately between runs toward the 34% cap (**~1393 in 0x0E4
   units** - ±0x2b4e/11086 is the INTERNAL Q15 value on ±32767, NOT a 0x0E4 target; parked never
   reaches it anyway), staying under the ~50% motor ceiling (~2048). **If no EPS response: STOP and report — do NOT escalate** (§11).
6. **Hand back:** `/data/eps_bench/canlog_*.csv` (all buses) + `cmdlog_*.csv` (commanded torque). Copy
   them off the device (route-pull style: the firmware team analyzes clip-vs-track from `0x18F`).

## Restore the cruise install when done
```
cd /data/openpilot
git checkout civic-6mt-dev-test
git submodule update --init --recursive
sudo reboot
```
(It will rebuild on AGNOS 20.1 — verify the cruise stack still comes up clean.)

## Open items to confirm on the FIRST run (before trusting results)
- `0x158` decode actually tracks true speed (gates the interlock).
- Relay reads open under ALLOUTPUT (camera `0x0E4` gone from bus 0).
- Bare `0x0E4` (REQUEST=1, no STEER_DOWN_TO_ZERO) produces motor torque at standstill — or null →
  stop & report (off-image standstill gate; separate low-speed plan).

## Optional, not yet done
- `run_bench.sh` one-shot (stop manager + bring up stack + menu in one line) — offered, not built.
- Push happens via the laptop (`git push --no-verify adk eps-bench-parked`); Claude's env is blocked
  from pushing and can't SSH the car.

---

## RESULTS — first run (2026-10-25, parked, engine running)

**Outcome: NULL RESULT at standstill (the directive's "off-image standstill gate").**
The EPS accepts our `0x0E4` stream and clears its own fault while we send it, but applies
**no motor torque** at a standstill. Per directive §8/§11 we stopped here; next step is the
firmware team's separate low-speed plan. Did NOT escalate TARGET/MAX_CMD.

### What we ran
- Stack: `eps-bench-parked` @ `c7f6f0859` + one on-car fix (below). `EPS_BENCH=1`,
  panda `allOutput` param 0 confirmed live (`ignition True`), autonomy procs stripped.
- Stage 1 passed: camera isolated (no `0x0E4` on bus 0; it stays on bus 2), bus-0
  `0x158`/`0x18F` live; `0x158` bytes 4-5 speed-decode tracks real motion (0.00 parked,
  1.5 kph rolling, back to 0.00 — Honda wheel-speed floor ~1.5 kph, so PARK BRAKE is the
  real standstill guarantee, not the 0.5 kph interlock).
- Stage 2: one ramp at defaults TARGET 600 / MAX_CMD 1000, 2 cycles +/-600, ~25 s, completed
  `DONE` reason none.

### Logs handed off (in Dropbox `…/CommaAI/EPS_Testing/`)
- `canlog_20261025_011420.csv` (34 MB, all buses, full session incl. engine-off/on and the ramp)
- `cmdlog_20261025_012004.csv` (the ramp that ran: ts, phase, setpoint, commanded torque, request, speed)

### Findings (from `analyze.py` over the logs)
- **Frames reached the EPS.** Panda echo on bus 0x80(=128) = 2589 frames, torque min/max -600/+600,
  `STEER_TORQUE_REQUEST=1` on 2583 of them. Panda `safetyTxBlocked = 0` (nothing blocked).
  Camera's `0x0E4` on bus 2 stayed torque 0 (relay open, no command fight).
- **No actuation.** Steer angle 2.4–2.5° flat through the whole ramp; `0x18F` torque sensor
  116–123 flat; `0x18F` STEER_CONTROL_ACTIVE bit never set.
- **`0x18F` STEER_STATUS timeline (0=normal,3=low_speed_lockout,5=fault_1):** ~8.5 s after each
  engine start (camera isolated, no steering stream) it goes 3→5 (fault). At ramp start (t≈0.07 s)
  it returns to **0 (normal)** and holds normal for the entire ramp. ~8.5 s after our stream stops
  (t≈26.6→35 s) it goes 3→5 again. So the EPS tracks presence of a `0x0E4` stream, and the dash
  **"Steering assist reduced, see dealer"** warning is the fault from the *missing* stream (engine-on
  with camera isolated and injector not running), NOT from the torque we sent. It cleared on a normal
  ignition off/on cycle — temporary fault, no scan tool needed.

### Interpretation for the firmware team
- Mode-0 34% LKAS assist does **not** apply torque at 0 kph with a bare `0x0E4`
  (`STEER_TORQUE_REQUEST=1`, `STEER_DOWN_TO_ZERO=false`). The EPS "honors the stream" (clears
  low_speed_lockout→normal while present) but gates actuation at standstill — consistent with a
  speed/vehicle-moving actuation gate inside the EPS, not a panda/relay/command-delivery problem
  (all of those were verified good).
- Open levers NOT tried this run (ask the firmware team before any): `SET_STEER_DOWN_TO_ZERO=true`
  (TJA frame), rolling >~1.5 kph (needs a separate low-speed driving plan + interlock rethink, out of
  scope for the parked bench), the `0x55AA` 17% sub-mode handshake.

### Code fix made on-car (committed `06554c51f`)
`steer_injector.py`: wait up to 3 s for the first `0x158` frame before starting the ramp clock.
A fresh `can` subscriber drains empty on its first loop → `last_speed=None` → the HARD interlock
latched `speed_unknown_or_stale` at t=0 and the first ramp aborted instantly (0 torque). After the
fix the second ramp ran full with speed read as 0.00 throughout. **The device copy was hot-patched
in place** (`/data/openpilot/eps_bench/steer_injector.py`); also committed to the WSL repo. If the
device is ever re-checked-out/reset, re-apply (or `git pull` once the commit is pushed with
`git push --no-verify adk eps-bench-parked`). **`config.json` is .gitignore'd** so it was absent on
the device — recreated by hand at `/data/openpilot/eps_bench/config.json` (defaults above); it will
need recreating again on any fresh checkout.

### Device left AS-IS for further bench testing (owner, 2026-10-25)
NOT restored to `civic-6mt-dev-test`. Device unplugged after the run. To resume: replug, engine on
parked, re-attach tmux (or relaunch `./eps_bench/launch_eps_bench.sh` + `eps_menu.py`), confirm
`config.json` still present. To return to cruise later: `git checkout civic-6mt-dev-test &&
git submodule update --init --recursive && sudo reboot` (rebuilds on AGNOS 20.1 — unvalidated there).

---

## NEXT BUILD DONE — low-speed-band mode (2026-10-25, commit `9151118b1`)
Per the firmware team's `CLOSED_COURSE_TEST_DIRECTIVE_2026-10-25.md` + `SP_DEV_HANDOFF_CLOSED_COURSE`,
added `MODE=lowspeed` to the injector (parked mode unchanged/default). Inverse interlock: commands
`0x0E4` only inside [V_MIN,V_MAX], ARM on the clutch-released `0x130` bit 50 (read raw), speed-gated
START, single bounded ramp toward the cap, auto-stop + ramp-to-0. Pure state machine `lowspeed.py`;
bench test `test_lowspeed.py` (13 cases, simulated speed+clutch, no motion) — ALL PASS on the PC.
Full detail: `eps_bench/LOWSPEED_MODE.md` (mirrored to Dropbox `EPS_Testing/`). Tracked sample
configs now exist (`config.{lowspeed,parked}.sample.json`) so the device never lacks a `config.json`.
**Device bench step next:** copy `config.lowspeed.sample.json`→`config.json`, confirm parked it ARMs
on a clutch cycle but never STARTs (speed 0). Closed course only after that. Commit not yet pushed.

---

## DEVICE LOADED & READY (2026-10-25) — low-speed mode
Device (`eps-bench-parked`) fast-forwarded `c7f6f0859` → **`c80b44cb3`** (Run-1 hot-patch discarded —
it's in history now; `openpilot/sunnypilot/` untracked leftover left alone). **All 15 bench tests pass
on-device** under `/usr/local/venv/bin/python`. `config.json` set to the **low-speed** sample
(MODE lowspeed, band 12–20 km/h, TARGET 300 / MAX_CMD 600). Pure-Python update, no rebuild needed.
**Next test session:** launch the stack → parked trigger check (ARM on a clutch cycle, never START),
then closed-course low-TARGET runs. Restore parked (Run-1) mode any time with
`cp eps_bench/config.parked.sample.json eps_bench/config.json`.
