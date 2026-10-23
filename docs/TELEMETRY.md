# Telemetry (Mega → ESP32 over the HC-05)

Newline-delimited JSON, protocol version `"v":1`. One object per line, ≤ 299 bytes, ASCII. Every frame has `v`, `k` (kind), `n` (sequence, wraps at 65535) and `ms` (Mega uptime in ms).
A frame is built only when the HC-05 transmit queue is empty and the whole frame fits; a frame is never sent truncated, never queued behind another, and a stuck or absent link never blocks the control loop.
**Values the hardware cannot provide are `null` or absent. There is no distance, no motor current, no wheel speed and no absolute heading anywhere in the protocol.** Motor figures are the PWM
the firmware *commanded* (`motor_command_pwm`), not measured rotation.

Rate: `telehz` (1–20, default 5) requested; the real rate is limited to ~60 % of the HC-05 UART capacity (see `docs/BLUETOOTH.md`: ~2 frames/s at 9600 baud, ~9 at 38400). Every `teledetail`-th (default 4th) frame is a
detail frame, rotating through `r`, `i`, `d`; all others are `c`. `:tele` shows the counters, `:tele off|on`, `:tele usb on` echoes frames to the USB console, `:tele now` sends core + receivers once.
Requested on demand by the words `status` (core), `sensors` (receivers + core), `gyro` and `motors` (imu / diag).

## `"k":"c"` core
| key | meaning |
|---|---|
| `mode` | `IDLE`, `MANUAL`, `ATTACK`, `DEFEND`, `CALIBRATION`, `FAULT` (latched e-stop, SAFE_STOP, drivetrain fault, defend fault-stop) |
| `sub` | navigation state (`SEEK`, `APPROACH`, `SEARCH`, `REVERSE`, …) or, in DEFEND, the defend state (`POSITION`, `SEARCH`, `TRACK`, `PREDICT`, `INTERCEPT`, `BLOCK`, `LOST`, `RECOVER`, `FAULT_STOP`) |
| `safety`, `fault`, `cond` | `OK`, `ESTOP_LATCHED`, `SAFE_STOP`, `DRIVETRAIN_FAULT`, `DEFEND_FAULT_STOP`, `MOTION_WATCHDOG`, `DEGRADED`; first active fault code; `NORMAL/DEGRADED/SAFE_STOP` |
| `bt` | 1 = bytes arrived from the remote within `btlink` (robot's view of the link) |
| `estop`, `cmd` | latch state; last single-key command received |
| `ball`, `bearing_deg`, `conf`, `ambiguous`, `centered` | ball seen (also during the loss hold); bearing 0 = straight ahead, + clockwise/right — **null when no ball or when the receivers disagree**; confidence 0–100 (a diagnostic figure, not a probability); conflicting receivers; within `centerdeg` |
| `obst` | `[module D34, module D35]` detecting (0 also when a module is switched off) |
| `under`, `under_ok`, `shot_pos` | underside sensor (D40) active; 1 = trusted (0 = ignored as stuck/never answering); ball in the shooting position |
| `motor_command_pwm` | `[FL, FR, BL, BR]` signed PWM last written (logical direction) |

## `"k":"r"` receivers (arrays are A0..A5; `-1` = channel not enabled)
`raw_level` (middle of the last sampling window, 0–1023), `swing` (peak-to-peak in the window), `filtered` (smoothed swing), `noise_floor` (stored by `calibrate`), `threshold` (swing needed to count as seeing the ball),
`enabled` (mask), `usable` (count), `seen` (mask), `best` (strongest channel), `strength` (best swing ÷ `ringnear`, 1.0 ≈ "very close", not a distance), `cal` (noise floor calibrated), `map` (`operator` = mounting angles entered by you, `default_guess` = not yet).

## `"k":"i"` IMU (all angles are RELATIVE ESTIMATES)
`imu` (delivering data), `imu_cal` (0 none, 1 running, 2 done, 3 rejected), `ax_g ay_g az_g`, `gx_dps gy_dps gz_dps`, `roll_est_deg`, `pitch_est_deg` (gyro integration corrected by gravity; axis signs unverified), `yaw_rel_est_deg`
(gyro only; drifts; zeroed at calibration), `yaw_unc_est_deg` (bias standard error × elapsed time – excludes bias instability), `gyro_bias_dps`, `drift` (0 idle, 1 running, 2 done, 3 failed), `drift_deg_min`, `drift_bias_dps` (after a finished `gyro drift`).

## `"k":"d"` diagnostics
`wheel_dir` (commanded −1/0/+1 FL FR BL BR), `moving_ms` (continuous commanded motion), `turning_ms` (rotation without translation), `wd_trips` (motion-watchdog stops), `stale_cmds` (stale motor commands cleared), `loop_avg_us`, `loop_max_us`, `hw_wdt` (hardware watchdog compiled in: 0 by default),
`faults` (mask), `imu_err`, `calibrated`, `tx_drops`, `cmds`, `cmd_unknown`, `def_state`.

## Examples (produced by the simulation, not measured on the robot)
```
{"v":1,"k":"c","n":1,"ms":8595,"mode":"IDLE","sub":"IDLE","safety":"OK","fault":"","cond":"NORMAL","bt":0,"estop":0,"cmd":"-","ball":0,"bearing_deg":null,"conf":0,"ambiguous":0,"centered":0,"obst":[0,0],"under":0,"under_ok":1,"shot_pos":0,"motor_command_pwm":[0,0,0,0]}
{"v":1,"k":"r","n":4,"ms":10022,"raw_level":[347,381,300,300,300,300],"swing":[94,162,0,0,0,0],"filtered":[94,162,0,0,0,0],"noise_floor":[0,0,0,0,0,0],"threshold":[20,20,20,20,20,20],"enabled":63,"usable":8,"seen":3,"best":1,"strength":0.43,"cal":1,"map":"operator"}
{"v":1,"k":"i","n":5,"ms":10485,"imu":1,"imu_cal":2,"ax_g":-0.000,"ay_g":0.000,"az_g":1.000,"gx_dps":-0.02,"gy_dps":0.02,"gz_dps":0.26,"roll_est_deg":0.0,"pitch_est_deg":0.0,"yaw_rel_est_deg":-0.6,"yaw_unc_est_deg":0.00,"gyro_bias_dps":0.022,"drift":0,"drift_deg_min":null,"drift_bias_dps":null}
{"v":1,"k":"d","n":6,"ms":10998,"wheel_dir":[0,0,0,0],"moving_ms":0,"turning_ms":0,"wd_trips":0,"stale_cmds":0,"loop_avg_us":147,"loop_max_us":162,"hw_wdt":0,"faults":0,"imu_err":0,"calibrated":1,"tx_drops":0,"cmds":0,"cmd_unknown":0,"def_state":"TRACK"}
```
(`usable":8` comes from the 8-receiver test layout; on the robot it is at most 6.)

## ESP32 side
`remote_esp32/telemetry_parse.h` (portable C++, host-tested): `teleLineFeed` assembles bounded lines (an over-long line is dropped whole and the next newline resyncs), `teleParseLine` rejects anything that is not a v1
frame with `v,k,n,ms` and counts it as bad, counts gaps in `n` as lost frames, keeps one snapshot per frame kind with the time it arrived. `teleLinkState` = `OK` (< 1.5 s), `STALE` (< 5 s), `LINK LOST`
(≥ 5 s or the ESP32's Bluetooth is down), `NO DATA YET`. `teleRender` prints the dashboard; unavailable values print `n/a` and sections older than 1.5 s are marked `[STALE/none]`.
There is no display on the remote: the dashboard goes to the ESP32's USB console once a second.
