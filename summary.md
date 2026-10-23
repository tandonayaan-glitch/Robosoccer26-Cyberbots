# RoboSoccer 2026: work and hardware status

Updated 24 October 2026. This distinguishes PC simulation from observations on the actual Mega and HC-05. The motor battery was disconnected for all hardware work below; wheel motion has not been tested.

## Software and Git

- Worked on branch `claude/sweet-knuth-hpyd7i`. Commit `37bb857` was pushed to that branch. It corrected the laptop login instructions and Windows verification paths; it did not change motor or sensor pins.
- Compiled the Mega firmware with the Arduino AVR toolchain with zero warnings. The Arduino IDE also uploaded firmware successfully to the Mega on COM3.
- Host simulation passed 86 C++ cases (2,814 checks) and 34 Python tests. These are software simulations, not proof of physical behavior. Sanitizer and optional static-analysis checks were unavailable in the portable Windows toolchain.

## Real hardware work completed

- Paired Windows with the HC-05 using PIN `1234`. Windows assigned outgoing Bluetooth serial port COM6; COM5 is the incoming port. Verified COM6 is associated with the HC-05 device.
- Provisioned a real authentication key through the Mega USB port COM3. The key is stored in the local private key file and Mega EEPROM. Only its public fingerprint, `e8939e9c`, was displayed; the secret was not printed or placed in this repository.
- Diagnosed an initial one-way UART failure. A temporary bench sketch sent beacons from Mega D18 through the HC-05 to Windows at 9600 baud, while D19/RX1 stayed LOW and received no transitions from the HC-05. The user reseated the HC-05 TXD to Mega D19 lead; the link then authenticated successfully. Every temporary diagnostic upload was followed by a verified restore of the robot firmware.
- Tried 115200 baud on the Mega and verified the setting after reboot. The module's UART was shown to be operating at 9600 baud by readable beacons, so the Mega was restored to 9600 and verified after reboot. The HC-05 module's own configuration was not changed.
- The laptop client on COM6 printed `AUTHENTICATED`. An unauthenticated serial session sent `M`, `W`, and `defend`; each received `ERR:AUTH`. Emergency stop `X` received `OK:X`. The authorized client then authenticated again. The e-stop latch was cleared with `R`, and the robot remained idle.
- Live IR and motor telemetry reached the laptop. The motor view showed commanded PWM `0` for all four wheels. The IMU laptop view did not receive a usable IMU frame; a USB diagnostic `:tele now i` produced no frame and incremented the firmware's drop counter. Its frame exceeded the 300-byte telemetry buffer on this hardware. I increased the telemetry buffer and Bluetooth transmit queue in the current working branch; this change is not yet rebuilt, uploaded, or retested on the Mega.

## Component audit over USB

| Component | What was observed | Still needed |
|---|---|---|
| Mega and firmware | Firmware `2.0.0-mega` booted; operating condition NORMAL; no active faults. | Retest after any firmware change. |
| Bluetooth and authentication | HC-05 radio and both serial directions worked after reseating D19; real authentication and refusal tests passed. | Physical link-loss and recovery test. |
| MPU6050 | Detected at I2C address `0x68`; live accelerometer and gyro readings and gyro bias calibration appeared over USB. WHO_AM_I was `0x70`, flagged as a possible clone. | Check axis signs and drift while stationary; fix Bluetooth IMU frame. |
| Ball receiver ring | Six inputs A0–A5 enabled and usable; A2 showed a small changing raw reading, the others were quiet. | Calibrate noise floor with the ball off; move the real ball around each receiver; set actual mounting bearings. |
| Obstacle modules | D34 enabled and clear (HIGH); D35 disabled and reads LOW. | Check D34 with an object; fit/configure and test D35 if intended. |
| Underside sensor | D40 read LOW and reported BALL but was marked `IGNORED (suspect: stuck)`. | Check its wiring, active level, and response with/without a ball. |
| Motors and drivers | Four motor commands were STOPPED at PWM 0; pin map and PWM-capable enable pins passed software checks. | With wheels lifted and motor battery connected, test each wheel's actual direction and stopping. No encoders/current sensors exist. |
| Power | Mega ran on USB. | Motor battery voltage/current and driver electrical health cannot be measured by this firmware. |

The firmware's startup request for autonomous mode was refused because the receiver noise floor is not calibrated. The robot remained IDLE. The current pin map is in `docs/WIRING_MEGA.md`.

## Remaining safe-test steps

1. Build, upload, and retest the enlarged IMU Bluetooth telemetry frame; assess frame loss on the 9600 baud link.
2. Test HC-05 power-off/link-loss and reconnection. This requires someone at the robot to cycle its power or move it out of range.
3. Calibrate and physically stimulate the ball receivers, D34 module, and D40 underside sensor.
4. Only after the sensor and link checks, connect motor power with the wheels still lifted and perform the supervised wheel tests in `Connect.md` and `docs/CALIBRATION_AND_TEST_PLAN.md`.

The raw USB component audit for this session was captured outside the repository in the task's working files. No secret key value is included here.
