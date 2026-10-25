# Low-speed-band mode (`MODE=lowspeed`) — closed-course EPS actuation test

Added 2026-10-25 after Run 1 proved the stock EPS will **not actuate parked** (LKAS engage is gated
on vehicle speed, firmware min ≈ 10 km/h; a well-formed `0x0E4` is accepted — STEER_STATUS→normal —
but no motor torque at 0 kph). The torque-cap (clip-vs-track) test must therefore run with the car
**moving in the engage band**, on a closed course. Full method + safety model:
`CLOSED_COURSE_TEST_DIRECTIVE_2026-10-25.md` and `SP_DEV_HANDOFF_CLOSED_COURSE_2026-10-25.md`
(Dropbox `…/CommaAI/EPS_Testing/`).

This mode is the **inverse** of the parked interlock: it commands `0x0E4` **only inside a low-speed
band**, armed by the clutch-released signal and a speed-gated start. The parked mode is unchanged and
still the default (`MODE=parked`).

## Files
- `lowspeed.py` — **pure** state machine (`LowSpeedController`), no cereal/panda/wall-clock. This is
  what the bench test drives and what carries all the decisions.
- `steer_injector.py` — `run_lowspeed()` wires cereal (`0x158` speed + `0x130` clutch in, `sendcan`
  out) to the controller; `main()` branches on `MODE`. Parked path byte-identical to Run 1.
- `common.py` — `parse_clutch()` raw-bit read of `0x130` (DBC-free, same approach as the speed parse).
- `config.lowspeed.sample.json` / `config.parked.sample.json` — tracked templates (copy to
  `config.json`; the real file is `.gitignore`'d).
- `test_lowspeed.py` — the directive §8 bench test (simulated speed + clutch, **no motion**).

## State machine
```
IDLE  --CLUTCH_PEDAL_UP rising edge-->  ARMED
ARMED --speed ≥ V_MIN+V_START_MARGIN (fresh)-->  RUNNING      (clutch pressed again -> IDLE)
RUNNING --single bounded ramp toward DIRECTION*TARGET; auto-stop on the tightest of:
          T_MAX / speed>V_MAX / speed<V_MIN−V_DROP / clutch-in / stale speed /
          session timeout / UI-abort / exception-->  STOPPING
STOPPING --ramp cmd → 0, then explicit STEER_TORQUE=0, REQUEST=0-->  DONE
```
Every exit biases to zero and is rate-limited (never a step). START never fires on the clutch alone —
it waits for the speed gate so the command can't step at the engage crossing.

## Clutch signal (fork-discovered)
`GAS_PEDAL_2` (0x130), bus 0: `CLUTCH_PEDAL_UP` = DBC bit 50 (byte 6 bit 2), `CLUTCH_PEDAL_FLOOR` =
bit 51 (byte 6 bit 3). Discovered on the 2024 Civic 6MT (Bosch radarless) and defined by name in the
**separate cruise fork's** `_honda_common.dbc`. Here we read the raw bits DBC-free so the bench stays
independent of that fork. Both bits set = inconsistent → clutch treated as unknown.

## Config knobs (see `config.lowspeed.sample.json` for the annotated defaults)
`MODE=lowspeed`, `V_MIN` 12, `V_MAX` 20, `V_START_MARGIN` 1, `V_DROP` 2, `T_MAX` 1.5 s, `TARGET` 300
(raise toward the ±0x2b4e/34%≈11086 cap deliberately), `MAX_CMD` 600 (hard ceiling), `MAX_RATE` 400/s,
`RAMP_S` 1.0, `DIRECTION` +1/−1, `CLUTCH_STALE_S` 0.3, `SPEED_STALE_S` 0.5, `SESSION_TIMEOUT_S` 60,
`ARM_REQUIRES_CLUTCH` true (set false for a bench/debug auto-arm only).

## Run order (directive §8)
1. **Bench (no car):** `python3 eps_bench/test_lowspeed.py` → all pass. Covers ARM→START→bounded→
   ramp-to-0 and every abort path, plus the clutch bit decode.
2. **Device bench, engine running, PARKED, wheels straight:** copy `config.lowspeed.sample.json` to
   `config.json`, launch `launch_eps_bench.sh` + `eps_menu.py`, press **B** (CAN LOG), then **A** to
   enable the injector. Parked it will **ARM** on a clutch cycle but **never START** (speed stays 0) —
   confirm it sits in ARMED/IDLE and commands nothing. This validates the trigger wiring with zero risk.
3. **Closed course only:** safety driver in the seat, hands off during a valid run, ready to abort via
   brake / clutch-in / steering grab / kill. 2nd-gear near-idle creep into the band; the injector
   auto-STARTs, runs one bounded ramp (≤ T_MAX), ramps to 0. Start TARGET well below the cap; raise it
   deliberately across runs. Hand back `canlog_*.csv` + `cmdlog_lowspeed_*.csv` per run.

## Not done / deferred
- **Excursion/heading budget auto-stop** (directive §6 "if derivable from yaw/speed") is **not**
  implemented — the band + `T_MAX` + bounded single ramp are the software envelope. Add a yaw-based
  budget only if wanted (would need yaw in the drain loop).
- CCP internal reads stay OUT of the fork (separate, unproven tool).
- Cereal wiring of `run_lowspeed()` is validated on the device bench (step 2), not on the PC.
