# Verification report (Arduino Mega 2560 firmware)

Evidence classes: **AVR-BUILD** (compiled with the real Arduino AVR core 1.8.6 and avr-gcc 7.3.0 for the ATmega2560) · **HOST** (host simulation with a mock of the Arduino core, simulated sensor values, IMU and EEPROM) ·
**STATIC** (cppcheck, clang-tidy, Valgrind on the host build) · **PHYS** (needs the robot) · **UNVERIFIED**. Nothing here is a measurement of the physical robot, and **the firmware has never run on a Mega**: it was not uploaded to any board.

## What was checked

| Check | Result |
|---|---|
| **AVR-BUILD**: `tools/avr_build.sh` (core 1.8.6 from the Debian package `arduino-core-avr`, same sources as the Boards Manager release; avr-gcc 7.3.0; the core's flags; `-Wall -Wextra` on the sketch) | builds, **0 warnings**: 77,154 B of 253,952 B flash (30 %), 3,519 B of 8,192 B static RAM (42 %), 4,673 B left for the stack |
| Stack use | `-fstack-usage`: largest frames 347 B (`logRow`), 325 B (`cliExecLine`), 235 B (`cfgValidate`), 223 B (message formatter); the deepest realistic chain (loop → CLI → `:set` → config validation → formatter) is estimated at about 1.5 KB, well inside the 4.6 KB left. An estimate, not a run-time measurement |
| **HOST** (`test/run_tests.sh`): 53 test cases | **53/53 pass**, 2,238 checks, `-Wall -Wextra -Wshadow`, ASan + UBSan |
| Pin map | `static_assert`s in `pins.h`: 8 direction + 4 PWM + 2 module + 2 I2C + 2 serial digital pins all distinct; the 4 enable pins are in the Mega PWM set (D2–D13, D44–D46); the 8 receiver inputs A0–A7 distinct and not digital pins; I2C is D20/D21. `t_pinmap` compares the motor pins with an independent copy of the specification; `t_ring_boot_sequence` checks that the motor pins are set first and that no unassigned pin (D0/D1, D30–D33, …) is ever touched |
| No multiplexer / no ESP32 | `test/run_tests.sh` greps the firmware sources for CD74/4067/multiplex/MUX names, the ESP32 APIs, LittleFS, FreeRTOS and `GPIO<n>` and fails if any appears; `t_old_hardware_removed` checks the output |
| STATIC | cppcheck: 1 style note (an unused variable in `cli.h`, pre-existing); clang-tidy: no findings; Valgrind memcheck clean on 12 test cases |

## What the host tests cover

Motor pin map and mecanum mixing (which wheel turns which way for forward, backward, strafe left/right, rotate, stop; with the per-wheel reversal and the X/O roller layout), ramps, reversal dwell, boot safety (all motor pins LOW first, nothing moves by itself); configuration validation, CRC and EEPROM persistence including a simulated power cycle; the receiver scan (per-receiver isolation, A2–A7 not scanned until enabled, window versus the 833 µs burst period, DC rejection, noise and thresholds with hysteresis, noise-floor calibration, saturation faults, no usable receiver, stalled scan, coverage gap); the bearing computation; the modules (debounce, per-module polarity, module-only versus receiver-confirmed shooting position, second module inert); the navigation (turn towards the ball, approach and slow-down, strafe when close, stop at the shooting position, search direction from the ball's motion / nearest uncovered arc / last side, re-acquisition, bounded search, safe stop on a fault); the goal heading and the shot; the IMU; the CLI; the paced long reports (complete, once, in order, without stalling the loop); the message formatter against the C library.
**The receiver input is a simulation**: a made-up cosine response per receiver with a made-up burst waveform and made-up thresholds. It checks the logic, not the physical sensors.

Simulated shot timing (assumed rotation model of 180°/s at full rotation command, **not** a measured value): from "ball in the shooting position" to the start of the forward drive, 436 ms (already pointing at the goal) and 696 ms (40° off), against 716 and 1446 ms for the sequential settings.

## What was NOT verified

* **That any of it works on the board.** The firmware was compiled for the ATmega2560, never run on it: the `Wire` bus timeout, the 400 kHz I2C, the EEPROM, `analogWrite` on D5/D6/D9/D10, the ADC conversion time and the serial speed are all unexercised.
* **The receivers**: the phototransistor pinout, the junction voltages (the 4.7 V maximum is computed), the response to the real ball's pulses, the mounting angles (placeholders), lighting and motor-noise sensitivity (PHYS).
* **The modules**: their supply and OUT levels, the polarity (assumed active-LOW by default, a setting), the distance setting.
* **The MPU6050 breakout** supply and pull-up arrangement (`WIRING_MEGA.md` §7).
* The motors' direction and rotation, speeds, stall, the reset state of the L298N inputs, the power supply (PHYS).
* The gyro drift over a match and the real rotation speed used by the aim; whether 400 ms of full power is the right shot (PHYS).
* The goal width of your rule set.
* The hardware watchdog (disabled by default because of the stock Mega bootloader).

## Behaviour that changed from the ESP32 version

Removed because the Mega has 8 KB of RAM, 256 KB of flash and no filesystem: CSV files in flash, the flight recorder, log export (`x`, `:xsel`), `:fsformat`, `:logdel` (these answer with a message), brown-out/reset-reason reporting, the FreeRTOS log writer and the hardware watchdog. `:diag on` streams the same kind of CSV rows over Serial instead. Configuration moved from LittleFS to the EEPROM (CRC protected; `:save` / `:load`). The ESP32 firmware is intact at git commit `1963a3f` (local tag `esp32-final`).

## Defects found during this work (and fixed)

* The Mega's 512-byte output queue dropped most of a long report (`h`): reports are now paced (printed in passes as the queue drains).
* The "last known bearing" after a loss drifted while the signal faded receiver by receiver; it now uses a sample from 100–150 ms earlier.
* With one receiver excluded, the bearing averaging window excluded its neighbour on the other side; the default blend was widened.
