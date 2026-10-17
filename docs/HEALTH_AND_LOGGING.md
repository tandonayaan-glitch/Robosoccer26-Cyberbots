# Hardware Health Monitor and logging

## 1. What can be checked, and how sure we are

| Component / capability | Automatic check | Needs a controlled test | Not measurable with this hardware |
|---|---|---|---|
| Motor GPIO + PWM setup | LEDC attach result (PASS/FAIL) – **software only** | – | – |
| Wheel direction | – | `:mtest`/`:dtest` + operator observation (`:mok`/`:mrev`) | Actual rotation, speed, stall, driver faults (no encoders / current sensing) |
| IR scan running | Scan cycles completing, window overruns, ISR flood guard | – | – |
| IR receiver working | Stuck-active output (≥ 2 s suspect, ≥ 15 s fail), interference (pulse flood) | **`:verify <ch>`** with the real ball held on the receiver | Whether an open/idle line is a dead receiver or "no ball" |
| IR coverage | Computed gap from *configured* bearings + assumed field of view | **`:learn <deg>`** sweep with the ball | Real optical coverage without measurements |
| Ball/receiver compatibility (38 vs 40 kHz) | – | `:verify`, `:learn` at several distances | – |
| MPU6050 | I²C ACK, WHO_AM_I, plausibility of every frame, frozen-frame detection, staleness, timeouts | – | Absolute heading accuracy |
| Filesystem | Mount + real write/read-back test at boot; write failures at runtime | – | – |
| Power | Reset reason (brownout / panic / watchdog) | – | Battery voltage, motor current |

Report statuses: `PASS` (the stated check passed) · `FAIL` (a test positively established a fault) ·
`WARNING` (suspicious, cause unproven) · `NOT TESTED` · `NOT MEASURABLE`. Booting never implies PASS
for physical hardware. **Absence of the ball is a navigation condition, never a sensor failure.**

## 2. Operating conditions

* **NORMAL** – every function passed its applicable checks.
* **DEGRADED** – an optional/partial capability is lost; the dependent function is disabled and the
  rest continues (reduced speed `degscale`, default 0.6): IMU missing/timeout/invalid (→ turn-toward-ball
  fallback, no heading hold), filesystem unavailable or failing (→ Serial + RAM recorder only),
  suspect/failed IR channels (→ excluded), coverage gap wider than `maxgap`.
* **SAFE_STOP** – autonomous movement is unsafe or impossible: no enabled IR channel, scan stalled,
  fewer trusted channels than `mintrusted` (default 4), drivetrain init failure, or a control-loop
  stall > 1 s. Motors are stopped at once (no ramp), autonomy is disabled, diagnostics remain.
  Manual/bench motion is still possible when the cause is a *sensing* fault (so the drivetrain can be
  tested before the receivers are wired); it is refused for drivetrain faults.

Recovery: non-latched faults clear only after the condition has been **absent for `recoverms`**
(default 3 s) – the IMU additionally needs 20 consecutive valid frames. Latched faults
(`MOTOR_INIT`, `LOOP_STALL`) need `:recover`. **After any fault, reset or recovery autonomy stays off
until the operator sends `a`.**

## 3. Fault codes

| Code | Severity | Latched | Action |
|---|---|---|---|
| `IR_NO_CHANNELS`, `IR_SCAN_STALL`, `IR_TRUSTED_LOW` | CRITICAL | no | SAFE_STOP |
| `IR_GAP`, `IR_CHANNEL` | DEGRADED | no | reduced speed; bad channels excluded |
| `IMU_MISSING`, `IMU_TIMEOUT`, `IMU_INVALID` | DEGRADED | no | heading hold off, fallback |
| `FS_UNAVAILABLE`, `FS_WRITE` | DEGRADED | no | persistent logging off |
| `MOTOR_INIT`, `LOOP_STALL` | CRITICAL | **yes** | SAFE_STOP until `:recover` |
| `CFG_REPLACED`, `BROWNOUT_RESET`, `ABNORMAL_RESET`, `LOG_DROPS` | WARN | no | informational; autonomy off after reset |

Every fault line carries: timestamp, severity, component, code, evidence, **certainty** (what is
confirmed vs suspected), action, current mode and recovery status. Identical repeats are suppressed
(one reminder per 10 s). Channel states: `UNVERIFIED` (default), `VERIFIED` (controlled ball test),
`SUSPECT_*`/`FAIL_*`. A channel is excluded from navigation only while it shows a positive fault
symptom (stuck, interference) or after **two** failed controlled tests; a single failed test is only
`SUSPECT_TEST` (the ball may not have been on the receiver).

## 4. Direction estimate and confidence (heuristic)

Per-channel inspection: `i` (all enabled channels), `:ch <n>` (one channel: status, last window state/edges/period, hits, detections, off-band rejects, verified flags). Pulse-period gate: `:irgate` (see `IR_DETECTION_AUDIT.md`). Saved configurations are version 4.

Each trusted channel contributes a vector at its calibrated bearing, weighted by its hits in the last
8 windows (0..1). The vectors are summed (circular maths, 359° and 1° are neighbours), smoothed as a
vector, and converted to a bearing. 

```
confidence = R_filtered × min(1, W) × (0.85 if any contributing channel is UNVERIFIED else 1)
R_filtered = length of the smoothed mean unit vector (agreement between channels)
W          = sum of the contributing channels' hit fractions
```
This is **not** an empirically calibrated probability. A ball is "acquired" after confidence ≥
`acqconf` (0.45) on two consecutive scans and "lost" after `losthold` (0.5 s) without a good sample.
Contradictory channels (agreement < 0.5) are flagged ambiguous and never acted on.

## 5. Logging

### Where
`LittleFS` (partition label `spiffs`), files `/L<session5>_<part2>.csv`, a session counter in
`/session.cnt`, settings in `/config.bin`. **The filesystem is never formatted automatically.**
`:fsformat CONFIRM` (explicit) creates one; `:logdel CONFIRM` removes old logs (keeps the open one).

### CSV schema (version 1) – one header block per file
```
#schema=1;fw=1.0.0;session=<n>;part=<p>
t_ms,sess,seq,type,cond,nav,irmask,actmask,ball,bear_deg,conf,imu,yaw_deg,herr_deg,gz_dps,vx,vy,w,pwm_fl,pwm_fr,pwm_bl,pwm_br,faults,scan_us,loop_us,note
```

| Column | Unit / values |
|---|---|
| `t_ms` | ms since boot (`millis()`, wraps at 2³²) |
| `sess`, `seq` | session id, row sequence number |
| `type` | `S` snapshot · `E` event · `F` fault · `R` recovery/recovery attempt · `N` navigation transition · `D` per-channel detail · `B` black-box dump |
| `cond` | 0 NORMAL, 1 DEGRADED, 2 SAFE_STOP |
| `nav` | 0 IDLE 1 MANUAL 2 SEEK 3 APPROACH 4 SEARCH 5 SEARCH_PAUSE 6 WAIT 7 TEST 8 SAFE_STOP |
| `irmask` | hex bit per channel: detection in its last window |
| `actmask` | hex bit per channel: smoothed "active" and trusted |
| `ball` | 1 acquired |
| `bear_deg` | ball bearing, degrees, 0 = forward, + clockwise, blank if no detection |
| `conf` | heuristic 0..1 |
| `imu` | 0 absent, 1 present but not OK, 2 OK |
| `yaw_deg`, `herr_deg`, `gz_dps` | relative yaw (cw+), heading error, gyro Z; blank when unavailable |
| `vx,vy,w` | requested body motion, −1..1 |
| `pwm_*` | **commanded** signed PWM counts per wheel (not measured speed) |
| `faults` | hex bit mask of active fault codes (bit = enum order in `health.h`) |
| `scan_us` | last full IR scan duration |
| `loop_us` | longest loop gap since the last snapshot |
| `note` | free text (no commas). Snapshot rows: `drop=<records dropped>;wf=<write failures>`. Detail rows: `c<ch> r<raw> e<edges> <state>` |

Rates: snapshots every `snapms` (250 ms) while moving, every 5 s while idle (flash wear);
`:diag on` adds one `D` row per enabled channel every `detailms` (100 ms).

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
#BEGIN-EXPORT name=<file|ring> schema=1
<lines verbatim>
#END-EXPORT lines=<n> crc32=<8 hex>   (CRC-32/IEEE over every exported line incl. '\n')
```
Every other message on the port starts with `[`, so a parser can drop those. The export is paced by
free space in the output queue; it never blocks the control loop. Python: read lines between the
markers, ignore lines starting with `[`, `zlib.crc32` them to check `crc32=`.
