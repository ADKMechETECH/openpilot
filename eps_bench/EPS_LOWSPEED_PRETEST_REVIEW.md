# EPS low-speed-band harness — pre-test review (for the firmware-RE team)

> **STATUS: APPROVED to proceed (RE team, 2026-10-25).** Corrections applied in build after `eabd798ef`.
> Resolution summary (details inline below have been updated to match):
> - **Cap scale fixed:** ramp `TARGET` toward **~1393** (34% in 0x0E4 ±4096 units), `MAX_CMD` ~1500–1600
>   for cap runs — **not** 11086 (that is the internal Q15 cap on ±32767). Defaults 300/600 stay for
>   first runs. ~1393 is approximate (unity-gain assumption) — ramp past it and watch the `0x18F` motor-
>   torque flat-top.
> - **Pre-roll added (flag 1):** ARMED now streams `0x0E4` REQUEST=1 at torque 0 so the stream/counter
>   is continuous before the ramp — no cold-start step at engage. Tested (`test_lowspeed.py`, now 14/14).
> - **Clip-vs-track field (flag 3):** analysis reads `0x18F` **STEER_TORQUE_MOTOR** (delivered), not
>   STEER_TORQUE_SENSOR (handwheel). We log raw full frames so either decodes.
> - **V_MIN:** [12,20] signed off; raise V_MIN to ~16 if actuation looks partial at 12 (windows ~13/15).
> - **Yaw/excursion abort (Q4):** deferred per RE direction; first-run logs pin the yaw source (Honda
>   VSA yaw-rate frame) + real excursion, then it is built as its own pass. **HARD GATE now enforced in
>   code** (`escalation_blocked`, `ESCALATION_GUARD=700`): the injector REFUSES any run with TARGET or
>   MAX_CMD > 700 until the abort exists and `YAW_ABORT_READY` is set. First runs (≤ 700) unaffected.
> - **Duration hand-back (flag 2):** separate longer sustained run later; `T_MAX` 1.5 s won't capture it.
> - Q1 clutch bits accepted; Q3 CCP stays out of the fork (CAN-only first); Q5 `SET_STEER_DOWN_TO_ZERO`
>   stays false.


**Prepared:** 2026-10-25 · **Vehicle:** 2024 Honda Civic Sport Hatchback 6MT (K20C2), comma four,
Bosch radarless harness · **Build:** `ADKMechETECH/openpilot` branch `eps-bench-parked` @ `edd181148`
(low-speed mode at `9151118b1`) · **Purpose of this doc:** everything to review **before** the first
closed-course run — what the harness does, the safety envelope, the parameters, what we log, what we
verified on the bench, and the decisions we need from you.

Companion docs (same folder): `CLOSED_COURSE_TEST_DIRECTIVE_2026-10-25.md` (your method),
`SP_DEV_HANDOFF_CLOSED_COURSE_2026-10-25.md` (the build brief), `EPS_RUN1_REPORT.md` (the parked run),
`LOWSPEED_MODE.md` (fuller engineering notes). **Standing rule: for any EPS/CAN specific not stated
here, we ASK you — we do not assume or search externally.**

---

## 1. Why this test exists (Run 1 recap)
Run 1 (parked, 2026-10-25) sent a rate-limited ±600 `0x0E4` ramp. Result: the EPS **accepts** a
well-formed frame parked — `0x18F` STEER_STATUS went to **normal** for the whole ramp — but applied
**zero motor torque** (handwheel torque sensor flat, STEER_CONTROL_ACTIVE never set). Panda
`txBlocked=0`, relay open, camera isolated on bus 2 — delivery was clean. You identified the cause as
an internal speed/motion gate (`FUN_801d9600 ≥ 0x600`, engage ≈ 10 km/h). **Conclusion:** the torque-cap
(clip-vs-track) and duration/validity behavior can only be observed with the car **moving in the engage
band**. This harness adds that capability.

## 2. What the harness is (and is not)
- A **parked/low-speed bench tool** that injects a single, rate-limited `0x0E4` STEERING_CONTROL ramp
  and logs **all** CAN, so you can read clip-vs-track from the log. It is **not** a driving system: no
  longitudinal, no lane model, no autonomy. The injector is the **only** `sendcan` steer source (all
  autonomy/control processes are stripped on this build).
- Panda runs **SAFETY_ALLOUTPUT** (debug build): relay open (camera's bus-2 `0x0E4` isolated from the
  EPS on bus 0), no bus-0↔2 forwarding, TX allowed at standstill. No panda safety-rule or EPS firmware
  edits. Reversible (stock when the `EPS_BENCH` env var is unset).
- Two modes, selected by `config.json` `MODE`: **`parked`** (Run 1, unchanged, still default) and the
  new **`lowspeed`** described here.

## 3. Low-speed mode — behavior
Commands `0x0E4` **only inside a configurable speed band**, triggered by the clutch-released signal and
a speed gate. State machine:

```
IDLE  --CLUTCH_PEDAL_UP rising edge-->  ARMED
ARMED --0x158 speed ≥ V_MIN + V_START_MARGIN (fresh)-->  RUNNING     (clutch pressed again -> back to IDLE)
RUNNING --single bounded ramp toward DIRECTION×TARGET; AUTO-STOP on the tightest of:
          T_MAX reached · speed > V_MAX · speed < V_MIN−V_DROP · clutch pressed (CLUTCH_PEDAL_UP→0) ·
          speed stale/unknown · session timeout · operator STOP/kill · any exception
        -->  STOPPING
STOPPING --ramp cmd → 0 at the rate limit, then explicit STEER_TORQUE=0, STEER_TORQUE_REQUEST=0-->  DONE
```
- **Single bounded ramp, not a sweep:** 0 → DIRECTION×TARGET (rate-limited), hold until a stop
  condition, ramp back to 0. Reaching the cap in the *command* is enough to see the clip; mechanical
  excursion is kept minimal.
- **START is speed-gated, never on the clutch alone** — so the command can't step at the engage
  crossing. The clutch only ARMs.
- **Every exit biases to zero and is rate-limited.** No instant cut.

## 4. Trigger signals (both already on bus 0; read raw, DBC-free)
| Purpose | Message | Field | Raw location | Notes |
|---|---|---|---|---|
| Speed gate / interlock | `0x158` | (speed) | bytes 4–5 big-endian ×0.01 km/h | Validated in Run 1 (0 parked → 1.5 kph rolling → 0). Honda wheel-speed floor ≈ 1.5 kph. |
| ARM | `0x130` GAS_PEDAL_2 | `CLUTCH_PEDAL_UP` | DBC bit 50 = byte 6, bit 2 | 1 = clutch fully released. Fork-discovered; read raw here. |
| Optional abort | `0x130` GAS_PEDAL_2 | `CLUTCH_PEDAL_FLOOR` | DBC bit 51 = byte 6, bit 3 | 1 = on the floor. Both bits set → clutch treated as **unknown**. |

**Please confirm (Q1):** the `0x130` bit 50/51 mapping on this specific car — we read it raw rather than
by name because this bench is forked from `commaai/openpilot` master, which does not carry the
fork-added signal. We have it cross-referenced to your cruise-fork DBC (`CLUTCH_PEDAL_UP : 50|1@0+`).

## 5. Safety envelope
**Software (enforced in the harness):**
- Low-speed-BAND only (the inverse of the parked mode's 0.5 kph abort). Outside [V_MIN, V_MAX] → no
  START, or abort a run in progress.
- `T_MAX` bounds run duration from START; the single bounded ramp + rate limit bound command slew;
  `MAX_CMD` is a hard magnitude ceiling independent of `TARGET`; a global `SESSION_TIMEOUT_S` watchdog.
- Clutch-in, stale speed, operator STOP, or any exception → STOPPING → ramp-to-0 → explicit release.

**Operational (test-team side, per your directive — stated here for review):** closed course, no
traffic, geofenced clear pad with run-off sized to run-time × speed × full-cap turn. Safety driver in
the seat, hands off the wheel during a valid run, ready to abort via **brake** (cuts throttle),
**clutch-in** (removes motive power, manual trans), **steering grab**, or **ignition/relay kill**.
Operating point = **2nd-gear near-idle creep, no throttle** → speed set by idle×gearing, inherently
bounded, cannot run away. Hard kill independent of software reachable at all times.

**Not implemented (by design):** an excursion/heading-budget auto-stop (your §6 "if derivable from
yaw/speed"). The band + `T_MAX` + bounded single ramp are the software envelope. We can add a
yaw-integrated budget if you want one — **decision Q4**.

## 6. Parameters (conservative defaults; escalate deliberately)
`config.lowspeed.sample.json` ships these; the operator copies it to `config.json`.

| Key | Default | Meaning |
|---|---|---|
| `V_MIN` / `V_MAX` | 12 / 20 km/h | engage-band floor/ceiling (firmware engage ≈ 10; margin above) |
| `V_START_MARGIN` | 1 km/h | START only at V_MIN + this |
| `V_DROP` | 2 km/h | abort if speed falls below V_MIN − this |
| `T_MAX` | 1.5 s | max run duration from START |
| `TARGET` | **300** (~7% of ±4096) | ramp peak — **well below the cap on first runs** |
| `MAX_CMD` | 600 | hard ceiling, independent of TARGET |
| `MAX_RATE` / `RAMP_S` | 400 /s / 1.0 s | anti-slam; effective step = stricter of the two |
| `DIRECTION` | +1 | single-direction ramp (do + and − as separate runs) |
| `SESSION_TIMEOUT_S` | 60 s | global watchdog |

**Escalation plan (for your sign-off, Q2):** start at `TARGET` 300, confirm it engages and steers
gently at speed, then raise `TARGET`/`MAX_CMD` across short runs **toward the ~1393 (34% in 0x0E4 units) cap** (0x2b4e/11086 is the internal Q15 value on
±32767, not a 0x0E4 value; set `MAX_CMD` ~1500–1600 to cross it) — never near the ~2048/50% motor ceiling without your explicit OK. Reaching the cap in the
*command* is what reveals the clip.

## 7. What we log / deliverables per run
- `canlog_*.csv` — **all buses**, `monotonic_ts,bus,addr_hex,len,data_hex`. Our TX appears as src
  **128** (= 0x80, bus-0 transmit echo); camera on bus 2.
- `cmdlog_lowspeed_*.csv` — `monotonic_ts,state,setpoint,cmd,request,speed_kph,clutch_up,reason,marker`
  with `ARM`/`START` markers for alignment.
- The analysis is yours: commanded `0x0E4` vs delivered `0x18F` **STEER_TORQUE_MOTOR** (flat-top while the
  command keeps rising = cap binding at ~1393, vs tracking) — NOT STEER_TORQUE_SENSOR (handwheel); time-to-handback (the ~756-count duration
  cause); EPS-TX status set (`0x324/326/328/32F/33D/374/3A1/516/78E`); any DTC / validity flip.
- **Known gap (Q3):** we have **no confirmed CAN "service-brake/torque-applied" or engaged tell-tale**
  beyond `0x18F` STEER_STATUS / STEER_CONTROL_ACTIVE and the torque sensor. If you want the internal
  probe reads (`0x70002B3C` post-cap command, `0x700034D2` engaged flag, etc. from your Stage-2 table),
  that CCP reader stays **out of the fork** and runs on your own interface in parallel — confirm whether
  you'll run it alongside, so we can mark the logs.

## 8. Bench validation already done (no car)
`eps_bench/test_lowspeed.py` — **13 cases, all passing** on the PC, driving the pure state machine with
synthetic `0x158` + clutch sequences (no motion):
- happy path ARM→START→bounded ramp→T_MAX→ramp-to-0→DONE (exactly one ARM edge, one START edge, peak =
  TARGET, never exceeds MAX_CMD, no per-frame step > rate limit, final cmd 0 / request 0);
- never STARTs on the clutch alone / below band; dis-arms if clutch pressed before START;
- aborts: speed > V_MAX, speed < V_MIN−V_DROP, clutch-in during run, stale speed, operator STOP, session
  timeout — each ramps to 0 and ends DONE;
- ramp-out from full TARGET is a smooth bleed (no cut); negative DIRECTION; clutch-decode bit mapping.

The cereal wiring of the device loop is validated on the **device bench** (step 2 below), not the PC.

## 9. Run order (what will happen on the day)
1. **PC bench (done):** `python3 eps_bench/test_lowspeed.py` → all pass.
2. **Device bench — engine on, PARKED, wheels straight, zero risk:** copy `config.lowspeed.sample.json`
   → `config.json`; launch the stack; start the CAN log; enable the injector. Parked, it will **ARM** on
   a clutch cycle but **never START** (speed stays 0) — we confirm it sits ARMED/IDLE and commands
   nothing. This proves the clutch+speed trigger wiring with no actuation risk.
3. **Closed course:** 2nd-gear idle creep into the band; the injector auto-STARTs, runs one bounded ramp
   (≤ T_MAX), ramps to 0. `TARGET` starts low, raised deliberately across runs per Q2. Hand back both
   logs per run.

## 10. Decisions we need from you before the closed-course day
- **Q1.** Confirm `0x130` `CLUTCH_PEDAL_UP`/`FLOOR` bit 50/51 on this car (we read raw).
- **Q2.** Sign off the band [12, 20] km/h, `T_MAX` 1.5 s, and the `TARGET`/`MAX_CMD` escalation toward
  ~1393 (the 0x0E4-unit cap; MAX_CMD ~1500-1600 to cross it). Any different starting point?
- **Q3.** Will you run your CCP internal-read rig in parallel (so we mark the logs), or is the CAN-only
  `0x18F`-vs-`0x0E4` clip read sufficient for the first runs?
- **Q4.** Do you want a yaw/heading excursion-budget auto-stop added, or are band + `T_MAX` + bounded
  single ramp sufficient?
- **Q5.** `SET_STEER_DOWN_TO_ZERO` stays **false** (stock non-TJA) unless you say the EPS needs the TJA
  frame to actuate — confirm.

## 11. Non-goals / do-not (restated)
No driving beyond the bounded low-speed test; no longitudinal; not a general fork. No EPS firmware
changes. No widening of panda safety beyond this test; keep it reversible. Parked mode stays intact.
CCP reads stay out of the fork. For any EPS/CAN uncertainty: **ASK.**
