# EPS parked-bench — Run 1 report (for the firmware-RE team)

**Vehicle:** 2024 Honda Civic Sport Hatchback 6MT (K20C2), comma four, Bosch radarless harness.
**Date:** 2026-10-25, parked, engine running, park brake on, wheels straight.
**Stack:** `eps-bench-parked` @ `c7f6f0859` + on-car fix `06554c51f` (see note 5).
**Run:** single ramp at the conservative defaults — TARGET 600, MAX_CMD 1000, MAX_RATE 400/s,
2 cycles (+600 / −600), `STEER_TORQUE_REQUEST=1`, `STEER_DOWN_TO_ZERO=false` (mode-0 / 34% LKAS).

---

## Headline
**The EPS accepts our `0x0E4` stream and reports STEER_STATUS = normal while we send the full
±600 ramp, but applies NO motor torque at standstill.** Null result at 0 kph — consistent with an
actuation gate inside the EPS that is off the CAN image (speed/vehicle-moving gate), not a
command-delivery, relay, or panda-safety problem (all verified good below). Stopped here per
directive §8/§11; did not escalate magnitude or limits.

## §10 answers
1. **Which bus is `0x0E4` on?** Injected on **bus 0** (F-CAN, EPS). In the log our TX appears with
   src **128 (= 0x80, the bus-0 transmit-echo flag): 2,590 frames**, torque min/max −600/+600,
   `STEER_TORQUE_REQUEST=1`. The stock **camera's `0x0E4` stays on bus 2 at torque 0** (56,001
   frames) — the ALLOUTPUT relay is open, so no command fight on bus 0.
2. **Did motor torque appear at standstill?** **No.** Through the entire ±600 ramp the handwheel
   torque sensor in `0x18F` stays flat (~118–121 raw, i.e. unchanged from rest) and
   **STEER_CONTROL_ACTIVE never sets (0)**. No wheel movement observed by the operator.
3. **Any DTC?** The dash showed **"Steering assist reduced — see dealer"**; it is the fault raised
   by the EPS when the `0x0E4` stream is ABSENT (engine-on with the camera isolated and the
   injector not running). It **cleared on a normal ignition off/on cycle** — temporary fault, no
   scan tool used. Whether a code was stored is for you to pull if useful.
4. **Delivery integrity:** panda `safetyTxBlocked = 0`, `safetyRxInvalid = 0`, safety model
   `allOutput`, ignition True. `0x158` bus-0 speed decode (bytes 4-5 ×0.01 kph) tracked real
   motion (0 parked → 1.5 kph rolling → 0), confirming the speed interlock source is valid.

## STEER_STATUS behaviour (0x18F; 0=normal, 3=low_speed_lockout, 5=fault_1)
- Engine-on, camera isolated, **no injection** → EPS settles to **5 (fault_1)** after ~8.5 s.
- A low-magnitude hold phase earlier in the log sat at **5 (fault_1)**.
- **During the real ±600 ramp → 0 (normal) the whole time**, i.e. the EPS does NOT treat a valid
  high-magnitude request as a fault or lockout — it just doesn't actuate parked.
- After our stream stops → back to 3→5.

So a well-formed `0x0E4` stream (correct COUNTER/CHECKSUM, REQUEST=1) is *honored* — it pulls the
EPS out of lockout/fault into normal — but actuation at 0 kph is gated separately.

## Aligned trace, ±600 ramp window (t relative to ramp start, ~0.5 s steps)
Columns: injected `0x0E4` torque / request | `0x18F` torque_sensor / STEER_STATUS / CONTROL_ACTIVE
```
t_rel[s]  e4_torque  req   18F_sensor  18F_status  active
  86.50         20    1         119           5       0
  87.00         20    1         119           5       0
  87.50         40    1         120           0       0
  88.00        240    1         120           0       0
  88.50        436    1         120           0       0
  89.00        600    1         121           0       0
  89.50        600    1         122           0       0
  90.00        600    1         119           0       0
  90.50        600    1         120           0       0
  91.00        558    1         121           0       0
  91.50        358    1         121           0       0
  92.00        162    1         119           0       0
  92.50          0    1         120           0       0
  93.00          0    1         118           0       0
  93.50          0    1         121           0       0
  94.00        -42    1         120           0       0
  94.50       -242    1         118           0       0
  95.00       -442    1         119           0       0
  95.50       -600    1         118           0       0
  96.00       -600    1         119           0       0
  96.50       -600    1         119           0       0
  97.00       -600    1         119           0       0
  97.50       -558    1         120           0       0
  98.00       -358    1         118           0       0
  98.50       -158    1         120           0       0
  99.00          0    1         119           0       0
  99.50          0    1         119           0       0
 100.00          0    1         119           0       0
 100.50         41    1         118           0       0
 101.00        237    1         118           0       0
 101.50        437    1         118           0       0
 102.00        600    1         119           0       0
 102.50        600    1         118           0       0
 103.00        600    1         117           0       0
 103.50        600    1         119           0       0
 104.00        559    1         118           0       0
 104.50        363    1         119           0       0
 105.00        159    1         119           0       0
 105.50          0    1         119           0       0
 106.00          0    1         119           0       0
 106.50          0    1         119           0       0
 107.00        -41    1         118           0       0
 107.50       -241    1         119           0       0
 108.00       -437    1         119           0       0
 108.50       -600    1         118           0       0
 109.00       -600    1         119           0       0
 109.50       -600    1         118           0       0
 110.00       -600    1         117           0       0
 110.50       -563    1         118           0       0
 111.00       -363    1         118           0       0
 111.50       -163    1         119           0       0
 112.00          0    1         118           0       0
 112.50          0    1         118           0       0
 113.00          0    1         117           0       0
```
Note: `18F_sensor` ~118–121 is the resting handwheel-torque reading; it does not deflect with the
±600 command → no reaction torque at the column → no motor output.

## EPS-TX status IDs present on bus 0 during injection (counts)
`0x324`:1135  `0x326`:1135  `0x328`:1135  `0x374`:1135  `0x3A1`:568  `0x516`:227
(`0x33D`, `0x32F`, `0x78E` not seen in this window.)

## Raw logs (this folder)
- `canlog_20261025_011420.csv` — all buses, full session (engine off/on + ramp), 34 MB.
  Format: `monotonic_ts,bus,addr_hex,len,data_hex`. Our TX = bus/src **128**; camera = bus **2**.
- `cmdlog_20261025_012004.csv` — the ramp that ran: `ts,phase,setpoint,cmd,request,speed_kph,note`.

## Open levers NOT tried (ask before any — directive §0)
- `SET_STEER_DOWN_TO_ZERO=true` (TJA frame) — may be required for the EPS to actuate.
- Rolling >~1.5 kph (Honda wheel-speed floor) — tests the suspected motion gate, but needs a
  separate low-speed driving plan and an interlock rethink; out of scope for the parked bench.
- The `0x55AA`-handshake 17% sub-mode (mode-word bit 0x800).

## Notes
5. On-car fix `06554c51f`: the injector waited for the first `0x158` before starting its ramp
   clock (a fresh `can` subscriber drains empty on frame 0 and had false-aborted
   `speed_unknown_or_stale` at t=0). Also `config.json` is `.gitignore`'d and was absent on the
   device — recreated by hand with the defaults above. Both re-apply on any fresh device checkout.
