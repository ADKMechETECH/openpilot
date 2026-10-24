# Installing the EPS bench build on the comma four (c4)

Branch `eps-bench-parked` on `ADKMechETECH/openpilot` (c4 pulls it with its GitHub SSH key).
PARKED BENCH ONLY — see `README.md` / `BUILD_REPORT.md` / `SP_HARNESS_DIRECTIVE.md`.

## Critical requirements
1. **Source/dev install, NOT a comma release channel.** `SAFETY_ALLOUTPUT` is compiled in
   only on the **debug panda build** (`ALLOW_DEBUG`, set whenever `RELEASE` is unset — i.e. any
   dev-branch source build). On a release/prebuilt channel ALLOUTPUT is absent and the harness
   cannot open the relay or TX steer.
2. **The panda firmware must be reflashed from this branch** (it changes `panda_safety.cc`). A
   normal on-device source build + boot does this. No `params_keys.h` / `.capnp` changes were
   made (the switch is the `EPS_BENCH` env var, not a Param), so there is no schema migration.
3. **Do not leave this branch on a car meant to drive.** With `EPS_BENCH` set it replaces the
   normal safety model; it is a parked bench stack, not a driving build.

## Install (over the c4's SSH key)
```bash
# back up the current cruise install first (binaries embed the /data/openpilot path)
cd /data && mv openpilot openpilot.cruise.bak
git clone -b eps-bench-parked git@github.com:ADKMechETECH/openpilot.git openpilot
cd openpilot && git submodule update --init --recursive
sudo reboot          # device builds pandad + flashes the debug panda firmware (ALLOUTPUT present)
```
LFS binaries resolve read-only from comma's public huggingface endpoint; no creds needed.

## Run (command entirely from your laptop over SSH; no C4 screen)
Use `tmux`/`screen` so an SSH drop doesn't kill the test (the injector also handles SIGHUP with
a graceful ramp-to-0, but tmux avoids the drop entirely). Keep a hardware kill/relay reachable.
```bash
tmux new -s eps
#  window 1:
eps_bench/launch_eps_bench.sh            # EPS_BENCH=1 -> stripped stack, pandad holds ALLOUTPUT
#  Ctrl-b c  -> window 2:
python3 eps_bench/eps_menu.py            # [B] CAN LOG, [A] STEER RAMP, [S] STOP/ABORT
```

## Test order (addendum 2026-10-24)
1. **Validate `0x158` speed-decode first** (open item #1): with CAN LOG running, roll slowly and
   confirm `0x158` bytes 4-5 track the speedo/GPS. Optionally arm STEER RAMP while rolling to
   confirm the interlock **aborts to 0 immediately** (hands ready). Stage 1 and the ramp cannot
   overlap — the speed interlock forbids injecting while moving, by design.
2. **Park**, confirm `0x158` ~= 0, then run STEER RAMP at the conservative defaults
   (`TARGET 600 / MAX_CMD 1000` = "does it respond parked"). Raise `TARGET`/`MAX_CMD` in
   `config.json` deliberately between runs toward the 34% cap, under the ~50% motor ceiling,
   tuning by watching the EPS-TX status torque (`0x18F` + the EPS-TX set) saturate against the
   rising `0x0E4`. If no response: **do not escalate** — stop and report (directive §11).

## Restore the cruise install when done
```bash
cd /data && rm -rf openpilot && mv openpilot.cruise.bak openpilot
sudo reboot
```

## Hand back to the firmware-RE team
`/data/eps_bench/canlog_*.csv` (all buses) + `cmdlog_*.csv` (commanded torque vs time). See
`BUILD_REPORT.md` §8.
