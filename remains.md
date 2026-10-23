# Remaining real-hardware testing

Updated 24 October 2026. This tracks work **not yet verified on the physical robot**. See [summary.md](summary.md) for tests already completed. The HC-05 did carry traffic in both directions: the laptop authenticated on outgoing COM6, received replies and telemetry, and the Mega accepted authenticated commands. Its LED pattern alone is not a reliable end-to-end test; rapid blinking often means no active serial connection, while connected indications vary by breakout.

## 1. Firmware and Bluetooth (wheels lifted, motor battery disconnected)

- [ ] Build the committed telemetry-buffer change, run the host tests, and upload the new firmware to the Mega. The change has been committed but **has not been built, uploaded, or retested**.
- [ ] Reconnect through outgoing COM6 at **9600 baud** and confirm `AUTHENTICATED` after the upload. The HC-05 UART was verified at 9600; changing the Mega setting alone to 115200 broke communication. Confirm the real key remains provisioned.
- [ ] Verify `imu` now receives complete `i` JSON frames and that `:tele now i` does not increment `dropped`. Check `ir`, `motors`, and `raw` too; record frame loss and actual rate at 9600 baud.
- [ ] With the client actively connected, compare the HC-05 LED pattern with its rapid-blink idle pattern. Close/reopen the outgoing COM port and confirm the pattern and authentication change appropriately. A paired device may still blink rapidly when its serial port is closed.
- [ ] Test actual link loss and recovery: disconnect the client or remove HC-05 power/range, confirm the laptop reports loss and an active **manual** command stops within the configured timeout; reconnect and confirm automatic reauthentication. Keep motor power disconnected for the initial session test.
- [ ] If higher telemetry speed is wanted, identify this HC-05 breakout's AT-command procedure, change its own UART setting, then set matching `btbaud100` on the Mega and retest. Do not assume the module is already at 115200.

## 2. Power and wiring checks (power off for continuity work)

- [ ] Verify the HC-05 VCC rating for this breakout, common ground, and the Mega D18-to-HC-05 RXD voltage divider; measure about 3.3 V at the divider junction when D18 is HIGH. Verify HC-05 TXD reaches Mega D19 reliably after the earlier loose-wire symptom.
- [ ] Check fused motor supply, Mega supply, both L298N boards' ENA/ENB jumper removal, and their input pull-downs. Never connect the battery's roughly 11 V rail to the Mega 5 V pin.
- [ ] With wheels lifted and motor battery connected, power-cycle/reset several times; verify no startup wheel twitch or unexpected autonomous movement. Check for voltage sag or Bluetooth resets when motors later run.

## 3. Sensors (motor battery disconnected first)

- [ ] Calibrate the A0–A5 ball-receiver noise floor with the ball off. Check each phototransistor's pinout and junction voltage, then present the ball to each receiver at 5, 10, 20, and 40 cm. Record range, raw swing, blind spots, and actual mounting bearing; enter bearing values and save them. Check response in competition lighting and with all transmitters/modules powered.
- [ ] Test obstacle module D34 with an object at relevant distances and confirm polarity. D35 is currently disabled; fit/configure/test it only if it is part of the robot.
- [ ] Investigate underside sensor D40, which read LOW and reported BALL but was marked `IGNORED (suspect: stuck)`. Observe it with and without the ball, confirm wiring and active level, then verify it becomes usable.
- [ ] Calibrate the MPU6050 while still; record stationary drift, check roll/pitch by tilting, and check yaw sign with a hand-turned 90-degree rotation. The observed WHO_AM_I value `0x70` may indicate a compatible clone; watch for I2C faults and recovery.

## 4. Motors and safety (wheels lifted; connect motor battery only for these tests)

- [ ] Test each wheel forward/reverse at low PWM using the supervised `:mtest`/`:confirm` procedure; correct and save direction. Confirm each wheel stops at command release and after emergency stop.
- [ ] Test forward/backward/strafe/rotation with `:dtest` at low speed and correct mecanum roller orientation if needed. Find minimum reliable PWM, compare wheel outputs, and set safe speed limits.
- [ ] Exercise manual W/S/A/D/Z/C through the authenticated client; verify motion stops when the key is released, on Bluetooth loss, and on the motion watchdog. Verify `X`/`!` latches a stop and `R` clears it only after authentication.
- [ ] Check startup/reset behavior again with motor power present, including power interruption; no wheel should twitch or start unexpectedly.

## 5. Autonomous behavior (after sensors and motor safety pass)

- [ ] First run attack and defend on the stand at reduced PWM. Confirm ball search, reacquisition, approach, alignment, obstacle handling, and safe stops.
- [ ] Register a goal heading, verify aim, then test a restrained shot and one-shot-per-ball behavior. Test failed-push recovery and its retry limit.
- [ ] Test low-speed floor driving and repeated ball runs; record receiver blind spots, defending overshoot, battery behavior, faults, and logs. Do not claim reliable blocking until multiple real balls have been blocked.
- [ ] Test degraded operation for an unplugged MPU6050 and a failed receiver only with a controlled procedure. Export a log and verify its CRC if log analysis is needed.

Detailed commands and pass criteria: [Connect.md](Connect.md), [calibration and test plan](docs/CALIBRATION_AND_TEST_PLAN.md), and [wiring map](docs/WIRING_MEGA.md). Check boxes only after observing each result on the actual robot.
