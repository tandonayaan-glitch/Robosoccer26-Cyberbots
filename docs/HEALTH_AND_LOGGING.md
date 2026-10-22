# Hardware health monitor and serial diagnostics (Arduino Mega 2560)

## 1. What can be checked, and how sure we are

| Component / capability | Automatic check | Needs a controlled test | Not measurable with this hardware |
|---|---|---|---|
| Motor PWM pins | each enable pin is a hardware PWM pin (software check only) | – | – |
| Wheel direction | – | `:mtest`/`:dtest` + operator observation (`:mok`/`:mrev`) | actual rotation, speed, stall, driver faults (no encoders / current sensing) |
| Ball receivers (A0, A1, …) | a receiver whose lowest reading stays at or above `ringrailhi` for `ringrailms` (2 s): saturated, excluded; no usable receiver; no completed scan for 600 ms; estimated blind arc above `ringmaxgap` | `:ring cal noise`, `:ring bearing`, `:ring cal near` with the real ball | whether a swing is the ball, another IR source or noise; the real mounting angles and fields of view; distance |
| IR modules (2) | none (a module that is always on cannot be told from a ball in place) | `:mod` with a hand/ball in range | whether the trimmer distance is right |
| MPU6050 | I²C ACK, WHO_AM_I, plausibility of every frame, frozen-frame detection, staleness, timeouts | – | absolute heading accuracy |
| Configuration | EEPROM magic, version, size, CRC and per-field range check at boot | – | – |
| Power | none | – | battery voltage, motor current, brown-outs (the AVR does not report a reset reason here) |

Statuses: `PASS` (the stated check passed) · `FAIL` · `WARNING` · `NOT TESTED` · `NOT MEASURABLE`. Booting never implies PASS for physical hardware. **Absence of the ball is a navigation condition, never a sensor failure.**

## 2. Operating conditions

* **NORMAL** – every function passed its applicable checks.
* **DEGRADED** – a partial capability is lost; the robot continues at reduced speed (`degscale`, default 0.6): IMU missing/timeout/invalid (no heading hold, no shot), receivers saturated and excluded (`RING_CHANNEL`), or an estimated blind arc above `ringmaxgap` (`RING_GAP`).
* **SAFE_STOP** – autonomous movement is unsafe or impossible: no usable receiver or a stalled scan, a motor enable pin that is not a PWM pin, or a control-loop stall over 1 s. An **uncalibrated** receiver set is not a fault: `a` is simply refused until `:ring cal noise` has been done. Motors are stopped at once (no ramp), autonomy is disabled, diagnostics remain. Manual/bench motion is still possible when the cause is a *sensing* fault.

## 3. Fault codes

| Code | Severity | Latched | Action |
|---|---|---|---|
| `RING_NONE` (no usable receiver), `RING_STALL` (no scan for 600 ms) | CRITICAL | no | SAFE_STOP |
| `RING_CHANNEL` (saturated receivers, listed as A0, A1, …) | DEGRADED | no | reduced speed; those receivers excluded from the bearing |
| `RING_GAP` (estimated uncovered arc above `ringmaxgap`) | DEGRADED | no | reduced speed; a ball in the gap is found by rotating |
| `IMU_MISSING`, `IMU_TIMEOUT`, `IMU_INVALID` | DEGRADED | no | heading hold off, no shot |
| `MOTOR_INIT`, `LOOP_STALL` | CRITICAL | **yes** | SAFE_STOP until `:recover` |
| `CFG_REPLACED` | WARN | no | invalid saved fields were replaced by defaults |

Each fault prints its severity, component, evidence, certainty, action, mode and recovery rule. Repeats are suppressed (one reminder per 10 s). A fault clears only after the condition has been absent for `recoverms` (3 s).

## 4. Ball state

`i` / `:ring` prints every receiver (swing, smoothed swing, low/high reading, noise floor, threshold, strength, seen) and the ball state; `:mod` prints the two modules; `d` prints the navigation state; `I` toggles a live view. See `BALL_SENSING.md`.

## 5. Records in the EEPROM (`:log`)

About 298 twelve-byte records in EEPROM bytes 512–4095 (the configuration uses 0–511), written by the firmware itself, oldest overwritten first, surviving power-off. Types: `BOOT`, `FAULT` / `RECOVER` (code name), `NAV` (`FROM>TO`), `BALL_LOST` (last bearing), `SHOT` (number), `SAFE_STOP`, `CFG_SAVED`, `NOTE` (your `:log note <n>`). Routine records are limited to 6 per second (dropped ones are counted); faults, SAFE_STOP and BOOT are never dropped. `:log` prints them oldest first as `seq,t_ms,EVENT,detail` between `#RECORDS …` and `#END-RECORDS`; `tools/get_records.py` saves that to `records.txt`. Each EEPROM cell is rewritten once per ~300 events (the EEPROM is rated for about 100,000 writes per cell [2nd]).

## 6. Output on the Mega (no flash logging)

The ESP32 version wrote CSV files to flash with a RAM flight recorder. The Mega has 8 KB of RAM and no filesystem, so:

* All text goes through a 512-byte non-blocking queue; whole lines are dropped (and counted in the `h` report) rather than ever blocking the control loop. Long reports (`h`, `?`, `i`, `d`, `m`, `t`) are **paced**: they are printed in several passes as the queue drains.
* `:diag on` streams CSV rows over Serial every `snapms` (250 ms) plus event rows; save the Serial Monitor output to a file to keep it. `:diag off` stops it. Header:
  `t_ms,type,cond,nav,ringmask,ball,bearing_deg,strength,flags,imu,yaw_deg,herr_deg,gz_dps,vx,vy,w,pwm_fl,pwm_fr,pwm_bl,pwm_br,faults,loop_us,note`
  `type`: S snapshot · E event · F fault · R recovery · N navigation transition. `cond`: 0 NORMAL, 1 DEGRADED, 2 SAFE_STOP. `nav`: 0 IDLE 1 MANUAL 2 SEEK 3 APPROACH 4 SEARCH 5 SEARCH_PAUSE 6 WAIT 7 TEST 8 SAFE_STOP 9 SHOOT.
  `ringmask`: hex, bit n = receiver n sees the ball. `bearing_deg`: blank when not seen right now. `strength`: 1.0 = `ringnear`. `flags`: bit0 held, bit1 ready, bit2 front module active, bit3 second module active. `pwm_*`: **commanded** PWM, not measured speed. `faults`: hex bit mask in the order of the table above (`health.h`).
* Removed (no flash filesystem): `x` and `:xsel` (log export), `:fsformat`, `:logdel`, the flight recorder. They answer with a message instead of being silently ignored.
