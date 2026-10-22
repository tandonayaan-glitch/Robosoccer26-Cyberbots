# Verification report (two-sensor firmware)

Evidence classes: **REAL-BUILD** (compiled with the real Arduino-ESP32 cores and Xtensa GCC) · **HOST** (host simulation with a mock core, simulated sensor values, IMU and filesystem) ·
**STATIC** (cppcheck, clang-tidy, Valgrind on the host build) · **PHYS** (needs the robot) · **UNVERIFIED**.
Nothing here is a measurement of the physical robot.

## What was checked

| Check | Result |
|---|---|
| **REAL-BUILD**, Arduino-ESP32 **2.0.17**, FQBN `esp32:esp32:esp32`, `--warnings all` | builds, **0 warnings**, 410,181 B flash (31 %), 64,428 B RAM (19 %) |
| **REAL-BUILD**, Arduino-ESP32 **3.0.7** | builds, **0 warnings**, 433,477 B flash (33 %), 63,160 B RAM (19 %) |
| **HOST** (`test/run_tests.sh`): 65 test cases × 2 core API shapes (mock core v3 and v2) | **130/130 runs pass**, 4,656 checks, `-Wall -Wextra -Wshadow`, ASan + UBSan |
| Pin map | `static_assert`s: 12 motor + 2 I2C output pins distinct; the two receiver pins distinct, not outputs, valid input-only GPIOs on the real SoC masks; the two (proposed) emitter pins distinct from every other pin, output-capable, not flash pins, GPIO0 or GPIO12; the emitter pins are never touched while `iremit` = 0 (`t_ir_emit_pins_untouched_by_default`) |
| Removed hardware | `test/run_tests.sh` greps the firmware sources for the old multiplexer, ring and three-pin receiver names and fails if any reappears; `t_old_hardware_removed` checks that the old commands are gone and the outputs mention none of it |
| STATIC | cppcheck: 1 style note (a never-used variable in `cli.h`, pre-existing); clang-tidy: 1 known analyser artefact in `util.h` (pre-existing); Valgrind memcheck clean on 12 test cases |

## What the host tests cover

Motor pin map and mecanum mixing, ramps, reversal dwell, boot safety; configuration validation, CRC and persistence (calibrations survive a reboot); the sensor rules
(strength in both polarities, digital mode, hysteresis, debounce, offset from the two strengths, shooting position, loss grace, wiring-fault detection, uncalibrated ≠ fault);
the navigation (approach and slow-down, cautious single-sensor alignment, stop at the shooting position, search toward the last side, re-acquisition, bounded search, safe stop on a
sensor fault); the goal heading (registration, invalidation by gyro resets) and the shot (preconditions, aim, reverse, full-power drive, launch clock, one shot per ball, cancellation);
the IMU (integration, bias, dropout, invalid/frozen frames); the CLI (robust input, manual keys, motor tests); logging (rotation, queue overflow, flight recorder, export, reboot persistence).

Simulated shot timing (assumed rotation model of 180°/s at full rotation command, which is **not** a measured value): from "ball in the shooting position" to the start of the forward drive,
445 ms (robot already pointing at the goal) and 705 ms (40° off), against 720 and 1450 ms for the sequential settings (`shootparallel 0`, slower aim, longer hold, reverse first).

## What was NOT verified

* **The real receivers and emitters**: their models, the receiver and LED-driver circuits (candidates only), polarity, range, field of view, response to the real ball, lighting and motor-noise sensitivity (**PHYS**). The test inputs are made-up numbers; the emitter model in the tests (reading = ambient + reflection while lit) is an assumption.
* Whether the emitter pins GPIO4 / GPIO13 match your wiring (they are a proposal).
* Whether the two fields of view overlap at the shooting position, and which side sensor 1 is on (set by you).
* The motors' direction and rotation, speeds, stall, the reset-state of the L298N inputs, the power supply (**PHYS**).
* The gyro drift over a match and the real rotation speed used by the aim; whether 400 ms of full power is the right shot (**PHYS**).
* The goal width of your rule set (the current rulebook could not be downloaded here).
* The firmware has not been run on the ESP32 hardware at all.

## Defects found during this work (and fixed)

* The shot's launch detection compared the wheels with the previous target, so the "drive" clock started during the reverse phase (the full-power drive was shortened by the reversal dwell). It now starts when the wheels actually turn forward (a test measures full speed within about 10 ms of the launch).
* A registered goal heading survived gyro resets (`:zero`, recalibration, IMU outage) and would have aimed at a meaningless direction; it is now tied to the gyro's reference and invalidated.
* The acceleration limit would have spent most of the shot's drive ramping up; it is lifted for the shot.
