# Start-up sequence and "robot does not move" debugging

**Status:** all results below are from the host simulation (`test/run_tests.sh`) and the AVR compile (`tools/avr_build.sh`). Nothing here has been run on the real robot.

## Why the robot used to refuse to move after flashing
1. A settings-layout change wiped the saved config, so the receiver noise floor was "not calibrated"; autonomy and the automatic start refused with that reason while manual keys still worked. Now: saved settings from the previous layout are migrated (`CL_MIGRATED`), and if nothing is calibrated the start-up measures the quiet receivers itself.
2. Six receivers were enabled by default, but only A0/A1 are fitted; floating inputs looked like faults. Default is now A0, A1.

## Start-up stages (non-blocking, about 7 s, messages on USB Serial)
`MOTOR INIT` (motors commanded stopped) -> `IMU CHECK` (MPU6050 + gyro bias, gives up after 6 s) -> `IR SETTLING` (2 s, receivers ignored in software only) -> `IR CHECK` (1 s: signal present = ball/sunlight/other robot) -> `SAFETY CHECK` -> `READY` or `READY (degraded: ...)` or `FAULT: ...`.
Movement, autonomy, defend and motor tests are refused with the reason until READY. `!` and diagnostics work at all times. The automatic attack only arms on plain READY with a working IMU and quiet receivers, at `maxauto` speed. `:startup` repeats the status. Settings: `startsettle`, `startcal`, `irwait`, `autocal`, `hdgmanual`.

## Bottom IR sensor
One 3-pin module (3.3 V, GND, OUT) on **D34**; it is module 0 and is the alignment / shooting-position input (`held`; `ready` also needs the receivers straight ahead when `modneedring` is 1). Active level: `:mod level` (check with `:raw`). The old separate "underside" vote on D40 is gone (`underenable` 0 by default, it would read the same pin).

## Heading and motor diagnostics
`:yawcheck` (turn the robot by hand, reports the yaw sign), `:hdgtest` (floor test, `:confirm` required), `:mtest <wheel>`, `:dtest` (lifted wheels, `:confirm`). They report the COMMANDED direction and PWM; whether the wheels really turned is for you to confirm.
