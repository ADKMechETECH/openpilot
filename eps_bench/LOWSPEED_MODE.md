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

**Pre-roll (RE flag 1, 2026-10-25):** while ARMED, the injector streams `0x0E4` with **REQUEST=1 at
torque 0**, so the counter/stream and the request bit are continuous *before* the ramp — no cold-start
step at the engage crossing. IDLE streams REQUEST=0; RUNNING/STOPPING REQUEST=1; DONE = explicit release.

## Clutch signal (fork-discovered)
`GAS_PEDAL_2` (0x130), bus 0: `CLUTCH_PEDAL_UP` = DBC bit 50 (byte 6 bit 2), `CLUTCH_PEDAL_FLOOR` =
bit 51 (byte 6 bit 3). Discovered on the 2024 Civic 6MT (Bosch radarless) and defined by name in the
**separate cruise fork's** `_honda_common.dbc`. Here we read the raw bits DBC-free so the bench stays
independent of that fork. Both bits set = inconsistent → clutch treated as unknown.

## Config knobs (see `config.lowspeed.sample.json` for the annotated defaults)
`MODE=lowspeed`, `V_MIN` 12, `V_MAX` 20, `V_START_MARGIN` 1, `V_DROP` 2, `T_MAX` 1.5 s, `TARGET` 300
(the cap in 0x0E4 units is ~34% ≈ **1393** — 0x2b4e/11086 is the INTERNAL Q15 value on ±32767,
not a 0x0E4 value; raise toward ~1393 and set `MAX_CMD` above it for cap runs), `MAX_CMD` 600
(hard ceiling), `MAX_RATE` 400/s,
`RAMP_S` 1.0, `DIRECTION` +1/−1, `CLUTCH_STALE_S` 0.3, `SPEED_STALE_S` 0.5, `SESSION_TIMEOUT_S` 60,
`ARM_REQUIRES_CLUTCH` true (set false for a bench/debug auto-arm only).

## RE-team review (2026-10-25) — applied / noted
- **Cap scale corrected:** ramp `TARGET` toward **~1393** (34% in 0x0E4 units), `MAX_CMD` ~1500–1600
  for cap runs. Defaults 300/600 stay for first runs.
- **Pre-roll added** (above): REQUEST=1 at torque 0 from ARM.
- **V_MIN:** [12,20] signed off; if actuation looks partial at 12 (related windows ~13/15 km/h), raise
  V_MIN to ~16.
- **Clip-vs-track is read from `0x18F` `STEER_TORQUE_MOTOR`** (delivered motor torque) vs commanded
  `0x0E4` — **not** `STEER_TORQUE_SENSOR` (handwheel). We log raw full frames, so either field decodes.
- **Yaw/heading excursion abort:** **not pursued** (owner decision 2026-10-25). At creep speed (~3 m/s,
  bounded ≤ `T_MAX` run, driver on the brake + clutch + kill) it buys little the human + `T_MAX` + band
  don't already give. The RE team had suggested one before crossing ~700; the owner judged it unnecessary
  at these speeds and is notifying them. Code keeps only an **advisory** (`escalation_note`,
  `ESCALATION_ADVISORY=700`): a one-line startup note above 700, no block. If ever wanted, the yaw source
  is a Honda VSA yaw-rate CAN frame (read from the all-bus log, validate scale — not an EPS/RE fact).
- **756-count duration hand-back** (bail-cause B) is a separate, longer sustained-engagement run on a
  bigger course, after the cap is characterized — `T_MAX` 1.5 s won't capture it.
- CCP stays out of the fork (CAN-only for first runs); `SET_STEER_DOWN_TO_ZERO` stays false.

## Run order (directive §8)
1. **Bench (no car):** `python3 eps_bench/test_lowspeed.py` → all pass. Covers ARM→START→bounded→
   ramp-to-0 and every abort path, plus the clutch bit decode.
2. **Device bench, engine running, PARKED, wheels straight:** copy `config.lowspeed.sample.json` to
   `config.json`, launch `launch_eps_bench.sh` + `eps_menu.py`, press **B** (CAN LOG), then **A** to
   enable the injector. Parked it will **ARM** on a clutch cycle (then stream REQUEST=1 at **torque 0** — pre-roll) but
   **never START** (speed stays 0). Confirm it sits ARMED/IDLE, torque stays 0, no ramp. Validates the
   trigger wiring + pre-roll with zero actuation risk.
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
