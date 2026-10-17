# Calibration guide, first power-on, and physical test plan

See also `COMMISSIONING_CHECKLIST.md` (ordered, with tick boxes) and `IR_DETECTION_AUDIT.md` §8 (distance/angle matrix at 10/20/40/60 cm).

Everything here is a **procedure for you to run**. No step has been performed on the robot;
every result below is **NOT TESTED**.

## A. First power-on (do in this order)

1. **Hardware first** (power off): remove the ENA/ENB jumpers on both L298N boards; add 4.7–10 kΩ
   pull-downs on all 12 L298N control inputs (IN1–IN4, ENA, ENB, both boards); common ground
   everywhere; 5 V logic from a buck converter (not from the L298N regulator when the pack is > 12 V);
   10–100 kΩ pull-up from mux SIG (GPIO34) to 3.3 V; receivers powered per `HARDWARE_REVIEW.md` §2
   (**confirm the receiver pinout and supply with a meter before applying power**); fuse the battery.
2. **Wheels off the ground**, robot on a stand, motor battery connector reachable. First flash with the
   motor battery **disconnected** (USB only).
3. Upload, open the Serial Monitor at 115200. Check the `[BOOT]` report: reset reason, filesystem,
   PWM init, IR channels, MPU6050 found at 0x68/0x69. Expect warnings: bearings UNCALIBRATED, wheels
   NOT TESTED, receivers NOT TESTED. Confirm **no wheel moved** and autonomy is DISABLED.
4. Press the reset button several times and power-cycle: with the motor battery *connected* and wheels
   lifted, confirm no wheel twitches at reset (validates the pull-downs; GPIO5/14 boot behaviour).
5. Connect the motor battery. Send `h` and `?`. Nothing may move.
6. **Motor direction** (§C), then **IR** (§D), then **IMU** (§E), then `:save`.
7. Only then try autonomy on the stand (`a`), then on the floor at low speed (`:set maxauto 100`).

Stopping fast: `s` stops and disables autonomy, `!` is an emergency stop that works even in the middle
of typing a `:` command; the manual keys time out by themselves; disconnect the motor battery for a hard stop.

## B. Software verification already done
Host tests only (see `VERIFICATION_REPORT.md`). Not a hardware result.

## C. Motor direction calibration
1. `t` (test mode), then for each wheel: `:mtest FL` → read the warning → `:confirm`. The wheel turns
   forward for 1 s, pauses, reverses for 1 s at `testpwm` (100).
2. If the first motion was **robot-forward**: `:mok FL`. If it was backward: `:mrev FL`, repeat the test, then `:mok FL`.
   (Left and right motors are normally mounted mirrored, so expect some reversals.)
3. `:dtest` → `:confirm` runs forward, backward, strafe left, strafe right, rotate cw, rotate ccw.
   If strafes are mirrored but forward/rotate are right → `:roller O`.
4. Find the smallest PWM that starts a wheel under load: `:set minpwm <n>`; set `:set maxauto`/`maxmanual`
   from the motor/driver ratings. `:save`.

## D. IR calibration (use the real ball, motors stopped, lights as in competition)
Bearings: 0° = robot forward, increasing **clockwise** (right = 90°, rear = 180°, left = 270°).
1. **Per receiver (identity, polarity)**: hold the ball against the face of receiver *k*:
   `:verify k`. PASS sets the verified flag. If it fails, check the ball is on, the part's pinout/supply,
   and polarity (`:ch k pol high` for an active-high module); a second failed test marks FAIL and excludes it.
2. **Bearings**: with the ball at a known bearing on a marked circle (e.g. every 15° at 30 cm), run
   `:learn <deg>` (2.5 s). It prints which channels saw the ball. Set each channel's centre with
   `:ch <n> bear <deg>` (marks it calibrated). Confirm the **clockwise ordering**.
3. **Rings**: repeat for the front ring, rear ring, and both sides; include the reported 4 cm gaps
   (place the ball exactly beside the chassis between rings), near and far distances, at the ball's
   different orientations, under competition lighting, with motors stopped and then with the wheels
   spinning (use `I` to watch for `!` noisy channels and `edges` counts).
4. **Mounting-angle experiment**: record the coverage table for the proposed 20–25° downward tilt, then
   change the angle (`:ch n tilt <deg>` records the metadata), re-run the sweep, compare. Data format:
   `:cov` prints CSV rows `COV,<deg>,<seen_mask_hex>,<pct C0..C15>`; paste them into a spreadsheet.
5. Decide per evidence: persistent blind spot → change sensor angles, add side receivers (enable up
   to four on C12–C15 with `:ch 12 en 1`, calibrate and verify them), relocate the rings, or change
   the acquisition circuit (`HARDWARE_REVIEW.md` §4). Do not claim full coverage until the table shows it.
6. Tune `irwindow`, `irminedges`, `irnoisy`, and optionally the pulse-period gate (`:irgate auto` after a `:verify`/`:learn` with the ball) using the `edges` and `per_us` columns of `i` (the measured
   pulse pattern of *your* ball). `:save`.

## E. IMU calibration
Robot level and still: `:gcal` (≈2 s). Rotate the robot by hand clockwise 90°: `m` must show the
heading changing by about +90 (if it goes negative: `:set yawsign 1`). `:zero` sets the reference.
Drift is expected (the gyro is integrated); the reference is re-set every time the ball is acquired.
Dropouts: unplug SDA → `IMU_TIMEOUT`, heading hold off, ball seeking continues; reconnect → verified
recovery after ~3 s.

## F. Physical test plan (all NOT TESTED)

| # | Test | How | Pass criterion | Result |
|---|---|---|---|---|
| 1 | First power-on | §A 1–5 | no motion, report as expected | NOT TESTED |
| 2 | Startup / reset safety | §A 4, also pull USB, brown-out by lowering supply | no wheel motion; autonomy off after every reset | NOT TESTED |
| 3 | Individual motor direction | §C 1–2 | each wheel forward = robot-forward | NOT TESTED |
| 4 | Four mecanum motions | `:dtest` | motions match labels | NOT TESTED |
| 5 | Individual IR channels | `:verify k` per receiver with the real ball | 12/12 PASS | NOT TESTED |
| 6 | Receiver angle + coverage | §D 2–5 | coverage table without holes | NOT TESTED |
| 7 | Front, side, rear sectors | §D 3 | every bearing seen by ≥ 1 channel | NOT TESTED |
| 8 | The 4 cm side gaps | ball beside the chassis | detected, or gap documented | NOT TESTED |
| 9 | Near-ground ball | ball on the floor at 5/15/30/60 cm | detection range recorded | NOT TESTED |
| 10 | Scan-cycle timing | `i` shows cycle µs; compare with an oscilloscope on S0–S3 | ≈ 36 ms, no overruns | NOT TESTED |
| 11 | Detection while motors run | `I` while driving (wheels lifted) | no `!` flags / false balls | NOT TESTED |
| 12 | MPU6050 calibration, heading stability | §E, 60 s still, 90° turns | drift rate recorded | NOT TESTED |
| 13 | Ball loss and reacquisition | remove/replace the ball while autonomous | SEARCH, reacquire time in stats | NOT TESTED |
| 14 | Missing optional parts | unplug MPU6050; remove the flash partition scheme | DEGRADED, robot still seeks | NOT TESTED |
| 15 | Deliberate storage failure | choose a partition scheme without spiffs | logging OFF, robot unaffected, no format | NOT TESTED |
| 16 | Runtime fault alerts / recovery | unplug a receiver supply, SDA; stuck/noisy channel | fault line, then verified recovery, autonomy still off | NOT TESTED |
| 17 | Repeated autonomous runs | 10 runs, check `h` stats and logs | consistent, no unexplained safety stops | NOT TESTED |
| 18 | Log export and analysis | `g`, `:xsel 0`, `x`, verify CRC in Python | CRC matches | NOT TESTED |
