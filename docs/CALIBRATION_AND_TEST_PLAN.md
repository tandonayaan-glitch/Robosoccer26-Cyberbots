# Calibration guide, first power-on, and physical test plan

See also `COMMISSIONING_CHECKLIST.md` (ordered, with tick boxes) and `BALL_SENSING.md` (the receivers and the two IR modules) and `WIRING_MEGA.md` (pins, power, electrical checks).

Everything here is a **procedure for you to run**. No step has been performed on the robot; every result below is **NOT TESTED**.

## A. First power-on (do in this order)

1. **Hardware first** (power off): remove the ENA/ENB jumpers on both L298N boards; add 10 kΩ pull-downs on all 12 L298N control inputs (IN1–IN4, ENA, ENB, both boards); common ground everywhere (`WIRING_MEGA.md` §8); decide the Mega's power source and **never put the battery's ~11 V on the Mega's 5 V pin**; each phototransistor with its own 10 kΩ load resistor on its own analog input (`WIRING_MEGA.md` §4; **verify the phototransistor pinout and measure each junction before relying on it**); the IR modules' supply and output levels (`WIRING_MEGA.md` §6); the MPU6050 supply and pull-ups (`WIRING_MEGA.md` §7); fuse the battery.
2. **Wheels off the ground**, robot on a stand, motor battery connector reachable. First flash (board *Arduino Mega or Mega 2560*, processor *ATmega2560*) with the motor battery **disconnected** (USB only).
3. Upload, open the Serial Monitor at 115200. Check the `[BOOT]` report: config, motor PWM pins, ball receivers (`NOT CALIBRATED`) and modules, MPU6050 found at 0x68/0x69.
   Expect warnings: wheels NOT TESTED, sensors NOT TESTED. Confirm **no wheel moved** and autonomy is DISABLED.
4. Press reset several times and power-cycle: with the motor battery *connected* and wheels lifted, confirm no wheel twitches at reset or while the bootloader runs (validates the pull-downs).
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

## C. Receiver and module calibration and testing

Follow `BALL_SENSING.md` §5: junction voltages and pinout check, receiver-by-receiver test with the ball at 5/10/20/40 cm, mounting angles (`:ring bearing <n> <deg>`), `:ring cal noise` (ball OFF), `:ring cal near`, module test (`:mod`, trimmer, polarity per module), interference test (transmitter LEDs and modules powered, motors running, competition lighting), blind-spot test, then `:save` (EEPROM). Autonomy (`a`) is refused until the noise floor is calibrated and the front module is on.

## D. IMU calibration

Robot level and still: `:gcal` (about 2 s). Rotate the robot by hand clockwise 90°: `m` must show the heading changing by about +90 (if it goes negative: `:set yawsign 1`).
`:zero` sets the reference. Drift is expected (the gyro is integrated); the approach heading reference is re-set every time the ball is acquired.
Dropouts: unplug SDA → `IMU_TIMEOUT`, heading hold off, ball seeking continues; reconnect → verified recovery after about 3 s.
`:gcal` and `:zero` invalidate the registered goal heading (`:goal set` after pointing the robot at the goal).

## E. Goal heading and the shot (only when you want it)

1. Set `goalwidth` and `goaldist` for your field. 2. Power the robot on **pointing at the goal and still**; 5 s later `[NAV] goal heading registered` appears (`:goal` to check). 3. Calibrate the receivers and modules (§C),
`:set shootenable 1`. 4. Wheels lifted first, then restrained, then on the floor. See `README.md` (autonomous behaviour) and `BALL_SENSING.md` §3–4 for what makes a ball `ready` and what cancels a shot.

## F. Physical test plan (all NOT TESTED)

| # | Test | How | Pass criterion | Result |
|---|---|---|---|---|
| 1 | First power-on | §A 1–5 | no motion, report as expected | NOT TESTED |
| 2 | Startup / reset safety | §A 4, also pull USB, brown-out by lowering supply | no wheel motion; autonomy off after every reset | NOT TESTED |
| 3 | Individual motor direction | §B 1–2 | each wheel forward = robot-forward | NOT TESTED |
| 4 | Four mecanum motions | `:dtest` | motions match labels | NOT TESTED |
| 5 | Receiver wiring and raw swings | `BALL_SENSING.md` §5 steps 1–4 | junctions 0–~4.7 V, quiet with the ball off, only the receiver facing the ball reacts | NOT TESTED |
| 6 | Boot and pull-downs | power cycles with the motor battery connected, wheels lifted | no wheel moves; autonomy off after every reset | NOT TESTED |
| 7 | Mounting angles and bearing | ball around the front at a fixed distance | printed bearing within a few degrees; blind arcs recorded | NOT TESTED |
| 8 | Module distance and polarity | ball, hand and a wall in front of each module | `DETECTING` exactly in the shooting position; polarity set per module | NOT TESTED |
| 9 | Detection range of the ball | ball at 5/10/20/40 cm on each receiver | range recorded; if too short, the search cannot find a distant ball | NOT TESTED |
| 10 | Interference | transmitter LEDs and modules on, competition lighting, sunlight, other robots, motors running (wheels lifted) | no false "seen" | NOT TESTED |
| 11 | MPU6050 calibration, heading stability | §D, 60 s still, 90° turns | drift rate recorded | NOT TESTED |
| 12 | Search, loss, blind spot and reacquisition | remove/replace the ball; step it into a blind arc | SEARCH starts towards where it went, ball re-found, stats | NOT TESTED |
| 13 | Approach, alignment and stop at the shooting position | autonomy on the stand, then the floor | creeps in, centres, stops | NOT TESTED |
| 14 | Goal heading and aim | §E with the wheels lifted | rotates toward the registered heading, stops inside the window | NOT TESTED |
| 15 | Shot | §E restrained, then on the floor | reverse then forward burst; one shot per ball | NOT TESTED |
| 16 | Missing optional parts | unplug the MPU6050 | DEGRADED, robot still seeks, no shot | NOT TESTED |
| 17 | Corrupt configuration | not testable safely; the host tests cover the EEPROM CRC | defaults used and reported | HOST only |
| 18 | Receiver fault | tie one receiver junction to 5 V | `RING_CHANNEL` degraded / `RING_NONE` safe stop (all receivers); recovery after the verification period | NOT TESTED |
| 19 | Repeated autonomous runs | 10 runs, check `h` stats and logs | consistent, no unexplained safety stops | NOT TESTED |
| 20 | Log export and analysis | `g`, `:xsel 0`, `x`, verify the CRC in Python | CRC matches | NOT TESTED |

## Staged hardware test plan for the 2026 additions (all steps are NOT yet done on the robot)

Wheels off the ground (robot on a stand) until step 9.

1. Power up with motors disconnected; `:status` -> check boot report, no faults.
2. HC-05: wire with the divider; pair from the Windows laptop (`Connect.md`); send `P` -> `PONG:IDLE`.
3. `M`, then `W/S/A/D/Z/C` with wheels lifted: check each wheel direction against the table in WIRING_MEGA.md.
4. Release a held key: wheels stop within about 400 ms. Switch the remote off: stop within `btlink`.
5. `X`: stops at once; `W`, `M`, `U` are refused (`ERR:ESTOP`); `R` clears; robot stays idle.
6. Per-wheel trim: `:set wscalefl/fr/bl/br` (50-150 %); compare wheel speeds by eye/tachometer; open-loop only.
7. Receivers A0/A1: `:ring cal noise` in the dark, then check swing with the ball at several bearings; note the blind regions.
8. Bottom sensor D34: with `:raw` watch its reading with and without the ball; set `underlow` to the real active level; check the stuck/never-answers warnings.
9. Floor, low speed: `U` with the ball ahead; check approach, hold, shot burst.
10. Failed push: place the ball so the push fails; confirm at most `remax` reverse tries, then a pause (`recooldown`), and a manual `X` during a reverse.
11. Search: hide the ball; confirm search sweeps, WAIT, fresh search (`researchms`), give-up (`giveupms`).


## Staged physical test plan for the intelligence / telemetry / defending update (NONE of these steps has been done on the robot)

Wheels off the ground (robot on a stand) until step 12. Motor battery switch within reach. Stop at the first surprise.

1. Boot with USB only (motor battery off): `status`, `sensors`, `gyro`, `motors` all answer; no fault you cannot explain; `help` lists the words.
2. Wiring: HC-05 VCC measured with a multimeter against your breakout's rating; the divider junction at HC-05 RXD reads about 3.3 V with the Mega transmitting; no pin shared with a motor.
3. Pairing: set the HC-05 name/PIN (AT mode, syntax per your module), power-cycle, pair Windows, run `laptop_remote.py --port COMx`, type `connect`; `P` → `PONG:IDLE`. Walk away until it drops: note the range, check `bt` goes 0 and `status` shows the link lost.
4. Telemetry: frames arrive in the laptop client (`ir`, `imu` views); `n` increments without gaps; check the real rate against the table in `docs/BLUETOOTH.md`; raise the HC-05 baud to 38400 and repeat.
5. Receivers: with no ball, `calibrate`; then `sensors` with the ball at 8 positions around the robot. Write the bearing of each receiver down and enter it with `:ring bearing <n> <deg>`, `:save`. Confirm telemetry `map` = `operator` and that the bearing follows the ball. Look for blind arcs, and for ambiguity when the ball is beside two receivers.
6. Receiver polarity: with a flashlight on one receiver the junction voltage and the swing must rise; if not, check the pinout (wiring doc).
7. Bottom sensor D34: watch `:raw` with and without the ball; set `underlow`; confirm `under_ok` stays 1.
8. IMU: robot still: `gyro cal`; `gyro drift 60` and note deg/min; tilt the robot by hand: roll/pitch follow gravity, yaw does not (it must not); move it during a drift test: the test must fail.
9. Manual: `manual`, then W/S/A/D/Z/C over Bluetooth with wheels lifted: directions; release the button (stop within ~0.4 s); switch the remote off (link lost, stop).
10. Emergency stop: from manual, attack and defend, send `X` and `!`: all four wheels stop at once and nothing restarts until `R`.
11. Motion watchdog: hold a command past `maxmovesec` (lower it to 5 s for the test): the robot stops and says so.
12. Floor, low `maxauto`, ball held by hand: `attack`: turn toward the ball, no charging at a ball on the side, recovery when the ball is hidden.
13. `defend` with a ball rolled slowly across the front, then toward the robot: watch the printed state transitions and telemetry `sub`; note how far it strafes (compare with `deflatms`), whether it overshoots, and tune `defspeed`, `defaligndeg`, `defslope`, `defstrafems`.
14. Never claim blocking works until a real ball has been blocked repeatedly.
