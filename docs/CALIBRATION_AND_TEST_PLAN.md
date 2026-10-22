# Calibration guide, first power-on, and physical test plan

See also `COMMISSIONING_CHECKLIST.md` (ordered, with tick boxes) and `RING_AND_MODULES.md` (the multiplexer ring and the two IR modules).

Everything here is a **procedure for you to run**. No step has been performed on the robot; every result below is **NOT TESTED**.

## A. First power-on (do in this order)

1. **Hardware first** (power off): remove the ENA/ENB jumpers on both L298N boards; add 4.7–10 kΩ pull-downs on all 12 L298N control inputs
   (IN1–IN4, ENA, ENB, both boards); common ground everywhere; a regulated 5 V for the ESP32 (not the L298N regulator when the pack is above 12 V);
   the ring: each phototransistor (collector 5 V) through the 6.8 kΩ series + 10 kΩ divider into its multiplexer channel, the multiplexer on 3V3/GND/EN=GND, SIG→GPIO34, S0→GPIO4, S1→GPIO13, S2→GPIO2, S3→GPIO15, the two IR modules on 3.3 V (`RING_AND_MODULES.md` §1, §4; **the phototransistors cannot connect straight to a channel; confirm every channel voltage stays within 0–3.3 V with a meter before applying power**; check that the board still boots and uploads with the S2/S3 wires connected, §3; GPIO0 and GPIO12 stay free); fuse the battery.
2. **Wheels off the ground**, robot on a stand, motor battery connector reachable. First flash with the motor battery **disconnected** (USB only).
3. Upload, open the Serial Monitor at 115200. Check the `[BOOT]` report: reset reason, filesystem, PWM init, ball ring (`NOT CALIBRATED`) and modules, MPU6050 found at 0x68/0x69.
   Expect warnings: wheels NOT TESTED, sensors NOT TESTED. Confirm **no wheel moved** and autonomy is DISABLED.
4. Press reset several times and power-cycle: with the motor battery *connected* and wheels lifted, confirm no wheel twitches at reset (validates the pull-downs; GPIO5/14 boot behaviour).
5. Connect the motor battery. Send `h` and `?`. Nothing may move.
6. **Motor direction** (§B), **ring and modules** (§C), **IMU** (§D), then `:save`.
7. Only then try autonomy on the stand (`a`), then on the floor at low speed (`:set maxauto 100`).

Stopping fast: `s` stops and disables autonomy, `!` is an emergency stop that works even in the middle of typing a `:` command; manual keys time out by themselves;
disconnect the motor battery for a hard stop.

## B. Motor direction calibration

1. `t` (test mode), then for each wheel: `:mtest FL` → read the warning → `:confirm`. The wheel turns forward for 1 s, pauses, reverses for 1 s at `testpwm` (100).
2. If the first motion was **robot-forward**: `:mok FL`. If it was backward: `:mrev FL`, repeat the test, then `:mok FL`. (Left and right motors are normally mounted mirrored.)
3. `:dtest` → `:confirm` runs forward, backward, strafe left, strafe right, rotate cw, rotate ccw. If strafes are mirrored but forward/rotate are right → `:roller O`.
4. Find the smallest PWM that starts a wheel under load: `:set minpwm <n>`; set `:set maxauto`/`maxmanual` from the motor/driver ratings. `:save`.

## C. Ring and module calibration and testing

Follow `RING_AND_MODULES.md` §9: circuit and strapping-pin check, channel-by-channel test with the ball at 5/10/20/40 cm, mounting angles (`:ring bearing <ch> <deg>`), `:ring cal noise` (ball OFF), `:ring cal near`,
module test (`:mod`, trimmer, polarity), interference test (modules powered, motors running, competition lighting), blind-spot test, then `:save`. Autonomy (`a`) is refused until the noise floor is calibrated and module 0 is on.

## D. IMU calibration

Robot level and still: `:gcal` (about 2 s). Rotate the robot by hand clockwise 90°: `m` must show the heading changing by about +90 (if it goes negative: `:set yawsign 1`).
`:zero` sets the reference. Drift is expected (the gyro is integrated); the approach heading reference is re-set every time the ball is acquired.
Dropouts: unplug SDA → `IMU_TIMEOUT`, heading hold off, ball seeking continues; reconnect → verified recovery after about 3 s.
`:gcal` and `:zero` invalidate the registered goal heading (`:goal set` after pointing the robot at the goal).

## E. Goal heading and the shot (only when you want it)

1. Set `goalwidth` and `goaldist` for your field. 2. Power the robot on **pointing at the goal and still**; 5 s later `[NAV] goal heading registered` appears (`:goal` to check). 3. Calibrate the ring and modules (§C),
`:set shootenable 1`. 4. Wheels lifted first, then restrained, then on the floor. See `README.md` (autonomous behaviour) and `RING_AND_MODULES.md` §7–8 for what makes a ball `ready` and what cancels a shot.

## F. Physical test plan (all NOT TESTED)

| # | Test | How | Pass criterion | Result |
|---|---|---|---|---|
| 1 | First power-on | §A 1–5 | no motion, report as expected | NOT TESTED |
| 2 | Startup / reset safety | §A 4, also pull USB, brown-out by lowering supply | no wheel motion; autonomy off after every reset | NOT TESTED |
| 3 | Individual motor direction | §B 1–2 | each wheel forward = robot-forward | NOT TESTED |
| 4 | Four mecanum motions | `:dtest` | motions match labels | NOT TESTED |
| 5 | Ring wiring and raw swings | `RING_AND_MODULES.md` §9 steps 2–4 | quiet and steady with the ball off; only the channel(s) facing the ball react | NOT TESTED |
| 6 | Strapping pins | five power-off/on cycles and one upload with S2 (GPIO2) / S3 (GPIO15) connected | boots and uploads every time | NOT TESTED |
| 7 | Mounting angles and bearing | ball around the robot at a fixed distance | printed bearing within a few degrees of the real angle; blind arcs recorded | NOT TESTED |
| 8 | Module 0 distance and polarity | ball, hand and a wall in front of it | `DETECTING` exactly in the shooting position; polarity set | NOT TESTED |
| 9 | Detection range of the ball | ball at 5/10/20/40 cm on several channels | range recorded; if too short, the search cannot find a distant ball | NOT TESTED |
| 10 | Interference | modules powered, competition lighting, sunlight, other robots, motors running (wheels lifted) | no false "seen" | NOT TESTED |
| 11 | MPU6050 calibration, heading stability | §D, 60 s still, 90° turns | drift rate recorded | NOT TESTED |
| 12 | Search, loss, blind spot and reacquisition | remove/replace the ball; step it into a blind arc | SEARCH starts towards where it went, ball re-found, stats | NOT TESTED |
| 13 | Approach, alignment and stop at the shooting position | autonomy on the stand, then the floor | creeps in, centres, stops | NOT TESTED |
| 14 | Goal heading and aim | §E with the wheels lifted | rotates toward the registered heading, stops inside the window | NOT TESTED |
| 15 | Shot | §E restrained, then on the floor | reverse then forward burst; one shot per ball | NOT TESTED |
| 16 | Missing optional parts | unplug the MPU6050 | DEGRADED, robot still seeks, no shot | NOT TESTED |
| 17 | Deliberate storage failure | partition scheme without spiffs | logging OFF, robot unaffected, no format | NOT TESTED |
| 18 | Ring fault | tie one channel / SIG to 3.3 V | `RING_CHANNEL` degraded / `RING_NONE` safe stop; recovery after the verification period | NOT TESTED |
| 19 | Repeated autonomous runs | 10 runs, check `h` stats and logs | consistent, no unexplained safety stops | NOT TESTED |
| 20 | Log export and analysis | `g`, `:xsel 0`, `x`, verify the CRC in Python | CRC matches | NOT TESTED |
