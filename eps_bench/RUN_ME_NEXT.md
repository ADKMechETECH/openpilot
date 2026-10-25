# RUN ME NEXT — EPS bench (updated 2026-10-25)

**NEXT STEP = IN-CAR TESTING of the low-speed-band mode.** The code is built, bench-tested (13/13),
committed and pushed. Nothing else to write before the car.

## The next two things, in order
1. **Device bench (engine on, PARKED, zero risk) — validate the trigger wiring.**
   On the car (`eps-bench-parked` @ latest; hotspot `10.69.162.42` / home WiFi `192.168.0.150`):
   - `cp eps_bench/config.lowspeed.sample.json eps_bench/config.json`
   - `tmux new -s eps` → `./eps_bench/launch_eps_bench.sh` → (new window) `PYTHONPATH=/data/openpilot
     /usr/local/venv/bin/python eps_bench/eps_menu.py`
   - Press **B** (CAN LOG), then **A** (enable injector). Parked, it must **ARM on a clutch cycle but
     NEVER START** (speed stays 0) and command nothing. That proves the clutch+speed triggers.
2. **Closed course (safety driver, 2nd-gear idle creep).** Only after step 1. Injector auto-STARTs in
   the band, one bounded ramp, ramps to 0. Start `TARGET` low (300), raise deliberately toward the
   ±0x2b4e cap. Hand back `canlog_*.csv` + `cmdlog_lowspeed_*.csv` per run.

## Before the closed-course day
Get the RE team's answers to **Q1–Q5** in `EPS_LOWSPEED_PRETEST_REVIEW.md` (clutch bits, band +
escalation sign-off, CCP-in-parallel?, excursion budget?, TJA-frame?).

## Pointers
- Full pre-test review: `EPS_LOWSPEED_PRETEST_REVIEW.md` (this folder)
- How the mode works: `LOWSPEED_MODE.md`
- Run 1 result (why we moved to a moving test): `EPS_RUN1_REPORT.md`
- Harness run procedure / device state: `C4_in_car_test_2026_10_24.md`
- Repo: `~/EPS_testing/openpilot`, branch `eps-bench-parked` (pushed to `adk`).
