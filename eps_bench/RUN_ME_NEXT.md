# RUN ME NEXT — EPS bench (updated 2026-10-25)

**NEXT STEP = IN-CAR TESTING of the low-speed-band mode.** RE team APPROVED to proceed (2026-10-25);
their corrections are applied (cap scale ~1393 not 11086, pre-roll REQUEST=1 from ARM). Code built,
bench-tested (14/14), pushed. Nothing else to write before the car — EXCEPT a yaw/heading excursion
abort is required *before* raising TARGET past ~700 (half cap); fine to run the first low-TARGET
closed-course runs without it.

## The next two things, in order
1. **Device bench (engine on, PARKED, zero risk) — validate the trigger wiring.**
   On the car (`eps-bench-parked` @ latest; hotspot `10.69.162.42` / home WiFi `192.168.0.150`):
   - `cp eps_bench/config.lowspeed.sample.json eps_bench/config.json`
   - `tmux new -s eps` → `./eps_bench/launch_eps_bench.sh` → (new window) `PYTHONPATH=/data/openpilot
     /usr/local/venv/bin/python eps_bench/eps_menu.py`
   - Press **B** (CAN LOG), then **A** (enable injector). Parked, it must **ARM on a clutch cycle but
     NEVER START** (speed stays 0) and command nothing. That proves the clutch+speed triggers.
2. **Closed course (safety driver, 2nd-gear idle creep).** Only after step 1. Injector auto-STARTs in
   the band, one bounded ramp, ramps to 0. Start `TARGET` low (300), raise deliberately toward **~1393** (34% in 0x0E4 units; set
   `MAX_CMD` ~1500-1600 for cap runs). A yaw/heading abort is required before TARGET > ~700. Hand back `canlog_*.csv` + `cmdlog_lowspeed_*.csv` per run.

## Open items before the closed-course day
Get the RE team's answers to **Q1–Q5** in `EPS_LOWSPEED_PRETEST_REVIEW.md` (clutch bits, band +
escalation sign-off, CCP-in-parallel?, excursion budget?, TJA-frame?).

## Pointers
- Full pre-test review: `EPS_LOWSPEED_PRETEST_REVIEW.md` (this folder)
- How the mode works: `LOWSPEED_MODE.md`
- Run 1 result (why we moved to a moving test): `EPS_RUN1_REPORT.md`
- Harness run procedure / device state: `C4_in_car_test_2026_10_24.md`
- Repo: `~/EPS_testing/openpilot`, branch `eps-bench-parked` (pushed to `adk`).
