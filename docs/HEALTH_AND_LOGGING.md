# Hardware Health Monitor and logging

## 1. What can be checked, and how sure we are

| Component / capability | Automatic check | Needs a controlled test | Not measurable with this hardware |
|---|---|---|---|
| Motor GPIO + PWM setup | LEDC attach result (PASS/FAIL) – **software only** | – | – |
| Wheel direction | – | `:mtest`/`:dtest` + operator observation (`:mok`/`:mrev`) | Actual rotation, speed, stall, driver faults (no encoders / current sensing) |
| Ball ring (12–16 channels on the multiplexer) | A channel whose lowest reading stays above 4000 for `ringrailms` (2 s): saturated, excluded; no usable channel; no completed scan for 600 ms; estimated blind arc above `ringmaxgap` | `:ring cal noise`, `:ring bearing`, `:ring cal near` with the real ball | Whether a swing is the ball, another IR source or crosstalk; the real mounting angles and fields of view; distance of the ball |
| IR modules (2) | none (a module that is always on cannot be told from a ball in place) | `:mod` with a hand/ball in range | Whether the trimmer distance is right |
| MPU6050 | I²C ACK, WHO_AM_I, plausibility of every frame, frozen-frame detection, staleness, timeouts | – | Absolute heading accuracy |
| Filesystem | Mount + real write/read-back test at boot; write failures at runtime | – | – |
| Power | Reset reason (brownout / panic / watchdog) | – | Battery voltage, motor current |

Report statuses: `PASS` (the stated check passed) · `FAIL` (a test positively established a fault) ·
`WARNING` (suspicious, cause unproven) · `NOT TESTED` · `NOT MEASURABLE`. Booting never implies PASS
for physical hardware. **Absence of the ball is a navigation condition, never a sensor failure.**

## 2. Operating conditions

* **NORMAL** – every function passed its applicable checks.
* **DEGRADED** – an optional/partial capability is lost; the dependent function is disabled and the
  rest continues (reduced speed `degscale`, default 0.6): IMU missing/timeout/invalid (→ no heading
  hold, no shot), filesystem unavailable or failing (→ Serial + RAM recorder only),
  ring channel(s) saturated and excluded (`RING_CHANNEL`) or a ring blind arc larger than `ringmaxgap` (`RING_GAP`).
* **SAFE_STOP** – autonomous movement is unsafe or impossible: no usable ring channel or a stalled ring scan,
  drivetrain init failure, or a control-loop stall > 1 s. An **uncalibrated** ring is not a fault: `a` is
  simply refused until `:ring cal noise` has been done. Motors are stopped at once (no ramp), autonomy is disabled, diagnostics remain.
  Manual/bench motion is still possible when the cause is a *sensing* fault (so the drivetrain can be
  tested before the receivers are wired); it is refused for drivetrain faults.

Recovery: non-latched faults clear only after the condition has been **absent for `recoverms`**
(default 3 s) – the IMU additionally needs 20 consecutive valid frames. Latched faults
(`MOTOR_INIT`, `LOOP_STALL`) need `:recover`. **After any fault, reset or recovery autonomy stays off
until the operator sends `a`.**

## 3. Fault codes

| Code | Severity | Latched | Action |
|---|---|---|---|
| `RING_NONE` (no usable ring channel), `RING_STALL` (no channel scan for 600 ms) | CRITICAL | no | SAFE_STOP |
| `RING_CHANNEL` (saturated channels, listed) | DEGRADED | no | reduced speed; those channels excluded from the bearing |
| `RING_GAP` (estimated uncovered arc above `ringmaxgap`) | DEGRADED | no | reduced speed; a ball in the gap is found by rotating |
| `IMU_MISSING`, `IMU_TIMEOUT`, `IMU_INVALID` | DEGRADED | no | heading hold off, no shot |
| `FS_UNAVAILABLE`, `FS_WRITE` | DEGRADED | no | persistent logging off |
| `MOTOR_INIT`, `LOOP_STALL` | CRITICAL | **yes** | SAFE_STOP until `:recover` |
| `CFG_REPLACED`, `BROWNOUT_RESET`, `ABNORMAL_RESET`, `LOG_DROPS` | WARN | no | informational; autonomy off after reset |

Every fault line carries: timestamp, severity, component, code, evidence, **certainty** (what is
confirmed vs suspected), action, current mode and recovery status. Identical repeats are suppressed
(one reminder per 10 s).

## 4. Ball state from the ring and the modules

`i` / `:ring` prints every ring channel (swing, smoothed swing, low/high reading, noise floor, threshold, strength, seen) and the ball state; `:mod` prints the two modules; `d` prints the navigation state.
Strength is `smoothed swing / ringnear`; a channel **sees** the ball at `max(ringseen, noise × ringnoisex)`. The ball is *valid* while a channel sees it and for `losthold` ms afterwards; its **bearing** (0 = ahead, + = right) comes from the seen channels and their configured mounting angles.
*Held* = front module detecting; *ready* = held and (with `modneedring`) the ring puts the ball within `centerdeg` of straight ahead.
None of this is a calibrated probability or a distance: see `docs/RING_AND_MODULES.md`.

## 5. Logging

### Where
`LittleFS` (partition label `spiffs`), files `/L<session5>_<part2>.csv`, a session counter in
`/session.cnt`, settings in `/config.bin`. **The filesystem is never formatted automatically.**
`:fsformat CONFIRM` (explicit) creates one; `:logdel CONFIRM` removes old logs (keeps the open one).

### CSV schema (version 2) – one header block per file
```
#schema=2;fw=1.0.0;session=<n>;part=<p>
t_ms,sess,seq,type,cond,nav,irmask,ball,offset,s0,s1,imu,yaw_deg,herr_deg,gz_dps,vx,vy,w,pwm_fl,pwm_fr,pwm_bl,pwm_br,faults,loop_us,note
```

| Column | Unit / values |
|---|---|
| `t_ms` | ms since boot (`millis()`, wraps at 2³²) |
| `sess`, `seq` | session id, row sequence number |
| `type` | `S` snapshot · `E` event · `F` fault · `R` recovery/recovery attempt · `N` navigation transition · `D` per-sensor detail · `B` black-box dump |
| `cond` | 0 NORMAL, 1 DEGRADED, 2 SAFE_STOP |
| `nav` | 0 IDLE 1 MANUAL 2 SEEK 3 APPROACH 4 SEARCH 5 SEARCH_PAUSE 6 WAIT 7 TEST 8 SAFE_STOP 9 SHOOT |
| `ringmask` | hex: bit n = ring channel Cn sees the ball |
| `ball` | 1 = the ring sees the ball (or did within `losthold`) |
| `bearing_deg` | −180…180 (0 ahead, + right); blank when the ball is not seen right now |
| `strength` | strongest channel's strength (1.0 = `ringnear`) |
| `flags` | bit0 front module detecting (held) · bit1 ready (shooting position) · bit2 module 0 raw-debounced active · bit3 module 1 active |
| `imu` | 0 absent, 1 present but not OK, 2 OK |
| `yaw_deg`, `herr_deg`, `gz_dps` | relative yaw (cw+), heading error, gyro Z; blank when unavailable |
| `vx,vy,w` | requested body motion, −1..1 |
| `pwm_*` | **commanded** signed PWM counts per wheel (not measured speed) |
| `faults` | hex bit mask of active fault codes (bit = enum order in `health.h`) |
| `loop_us` | longest loop gap since the last snapshot |
| `note` | free text (no commas). Snapshot rows: `drop=<records dropped>;wf=<write failures>`. Detail rows: `c<channel> a<swing> <strength>[S|X]` (S = sees the ball, X = saturated) |

Rates: snapshots every `snapms` (250 ms) while moving, every 5 s while idle (flash wear);
`:diag on` adds one `D` row per enabled ring channel every `detailms` (100 ms).

### Rotation and budget
Max `filemax` bytes per file (65536), at most `maxfiles` files (4) – when a new part is needed the
**oldest** `L*.csv` is deleted first; writes stop when free space would fall below `fsreserve`
(65536 B). Batched writes every `flushms` (2 s), at most ~1 KB at a time, taken from a 6 KB RAM
queue by a writer task on the other core. A full queue drops whole lines and counts them.

### Failure handling
Failed/short write → exponential back-off, data kept for retry; after 5 consecutive failures
persistence is switched off (`FS_WRITE` DEGRADED), the queue is drained, Serial + flight recorder
continue, the robot keeps driving. No automatic format. `g` shows free space, failures, drops.

### Black-box flight recorder
A 160-record RAM ring (one every `frms`=100 ms ≈ 16 s, plus every event) holds sensor summaries,
decisions and PWM. A DEGRADED/CRITICAL fault or SAFE_STOP freezes a trigger, keeps recording
`frpost` ms (2 s) of post-event data, then copies the whole window to a frozen buffer and writes it to
the log as `B` rows. Overlapping triggers are merged. Works with no filesystem.

### Export over USB Serial
```
:xsel ring      (or :xsel 0 = newest file, 1 = next …; list with 'g')
x
#BEGIN-EXPORT name=<file|ring> schema=2
<lines verbatim>
#END-EXPORT lines=<n> crc32=<8 hex>   (CRC-32/IEEE over every exported line incl. '\n')
```
Every other message on the port starts with `[`, so a parser can drop those. The export is paced by
free space in the output queue; it never blocks the control loop. Python: read lines between the
markers, ignore lines starting with `[`, `zlib.crc32` them to check `crc32=`.
