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
  (`0x324/326/328/32F/33D/374/3A1/516/78E`). Firmware cap = **±0x2b4e = 11086 = 34%**; 17% sub-mode is a
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
   the rising `0x0E4`. Raise `TARGET`/`MAX_CMD` deliberately between runs toward the 34% cap (±0x2b4e),
   staying under the ~50% motor ceiling. **If no EPS response: STOP and report — do NOT escalate** (§11).
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
