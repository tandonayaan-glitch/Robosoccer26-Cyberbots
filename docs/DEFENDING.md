# Defending mode

Enter: `defend` (USB Serial Monitor, Newline / Both NL & CR; or the same word over Bluetooth, or the DEFEND button on the remote sketch). Leave: `stop`, `attack`, `manual`, `idle`, or the emergency stop.
Refused (with the reason printed and `ERR:REFUSED` over Bluetooth) when: the emergency-stop latch is set, SAFE_STOP / sensing / drivetrain fault, a motor test is running, no usable receiver,
receiver noise floor not calibrated (`calibrate`), or the receiver mounting angles are still the default guess (`:ring bearing <0-5> <deg>`; sets `bearingsset`). It does **not** need the MPU6050, but without a usable gyro the robot
cannot hold or restore a heading and says so.

## What it can and cannot do
* **No position sensing exists** (no encoders, no distance sensor, no field marker). "Defensive position" is a **heading** (the gyro yaw at the moment `defend` was entered, relative) plus an **open-loop estimate** of the sideways
  offset from where it started (`g_defLat`, integrated from the commanded strafe time; limited by `deflatms`). The estimate only limits how far the robot wanders; it is not a measurement and wheel slip is not seen.
* The receivers give a bearing and a strength, not a distance. **"Approaching" = the strength rose over ≥ 4 time-stamped observations spanning ≥ 0.3 s** (least-squares slope ≥ `defslope` per second). That does not prove
  the ball is heading for the goal; with no trend data the robot uses a conservative fallback (shadow a *close* ball only: strength ≥ 0.4).
* No goalkeeper mechanism and no kicker exist. "Block" means standing in front of the ball with the wheels. Nothing guarantees it stops the ball.
* The receivers' coverage and the mounting angles are unverified; the ring covers 360° only if the real angles say so.

## States (each transition is printed as `[DEFEND] A -> B (reason)`, recorded, and sent as `sub` in telemetry)
| state | behaviour | leaves when |
|---|---|---|
| POSITION | rotate gently back to the entry heading (needs the gyro), drift back toward the start offset | settled → SEARCH; `defposms` timeout → SEARCH; ball confirmed → TRACK |
| SEARCH | stand still, watch (no spinning) | ball confirmed (`defconfon`, `defconfirm`) in the front arc → TRACK; behind → RECOVER |
| TRACK | hold, follow the ball in the readings | lost → LOST; behind → RECOVER; at the front sensor / strong and in line → BLOCK; approaching → PREDICT; close and off-line → INTERCEPT |
| PREDICT | commit a strafe direction from bearing + measured lead (`deflead`, ≤ 25°) | after `defconfirm`: INTERCEPT or BLOCK; receding → TRACK; lost → LOST |
| INTERCEPT | strafe sideways toward the ball (speed `defalign`..`defspeed`, scaled by confidence), heading held, no rotation | aligned within 0.75 × `defaligndeg` → BLOCK; lost → LOST; limits → TRACK |
| BLOCK | stand still, final alignment at 0.6 × `defalign`; checks the ball is still there | ball gone → LOST; moved away / receding → TRACK |
| LOST | stand still `deflost` ms, watching | ball reacquired → TRACK; else bounded RECOVER (only if the gyro works and the last bearing was off-line) or POSITION |
| RECOVER | bounded turn (`defrecover` ms, ≤ 3 tries) toward the last bearing | ball confirmed → TRACK; time up → POSITION (heading restored) |
| FAULT_STOP | motors stopped | operator action (`stop`, then fix and `defend`) |

Hysteresis and timing: confidence must reach `defconfon` (40) to start tracking and fall below `defconfoff` (25) to lose it; a state is held at least `defdwell` (250 ms) before a normal transition; the INTERCEPT exit
threshold (0.75 × `defaligndeg`) is tighter than its entry; the strafe direction is not flipped on a small overshoot.

Limits: at most `defstrafems` (1.5 s) of continuous strafe then a pause; the estimated sideways offset never passes ±`deflatms` (2.5 s at `defspeed`); an active obstacle module 1 (D35) stops strafing and holds for 0.5 s;
the global continuous-motion limit `maxmovesec` (45 s) applies; the emergency stop works from every state in one loop pass. Receiver ambiguity (strong receivers disagreeing) counts as "no ball".

## Settings (`:set <name> <value>`, `:get`, `:save`)
`defspeed 0.4`, `defalign 0.2`, `defturn 0.3` (fractions of `maxauto`), `defaligndeg 12`, `deffrontarc 100` (bearings beyond this are "behind"), `defblock 0.7` (strength at the block position), `defslope 0.15`, `defconfon 40`, `defconfoff 25`,
`defdwell 250`, `defconfirm 150`, `deflost 1500`, `defrecover 1200`, `deflead 250`, `defposms 3000`, `defstrafems 1500`, `deflatms 2500`. All defaults are untested guesses; they need tuning on the field.

## Simulation coverage / what is not verified
Verified in simulation only (`t_defend_*`): entering/leaving, refusals, POSITION→SEARCH→TRACK→INTERCEPT→BLOCK→LOST sequences, PREDICT only on a rising strength, no flicker between the confidence thresholds, strafe and offset limits,
fault stop, emergency stop from every mode. **Not verified:** that the real receivers give a usable bearing and strength trend, that the robot gets in front of a real ball in time, strafe speed and slip, anything about blocking.
