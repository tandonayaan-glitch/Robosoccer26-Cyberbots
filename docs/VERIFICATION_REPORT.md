# Verification report (multiplexer ring + two IR modules firmware)

Evidence classes: **REAL-BUILD** (compiled with the real Arduino-ESP32 cores and Xtensa GCC) · **HOST** (host simulation with a mock core, simulated sensor values, IMU and filesystem) ·
**STATIC** (cppcheck, clang-tidy, Valgrind on the host build) · **PHYS** (needs the robot) · **UNVERIFIED**.
Nothing here is a measurement of the physical robot.

## What was checked

| Check | Result |
|---|---|
| **REAL-BUILD**, Arduino-ESP32 **2.0.17**, FQBN `esp32:esp32:esp32`, `--warnings all` | builds, **0 warnings**, 417,461 B flash (31 %), 65,708 B RAM (20 %) |
| **REAL-BUILD**, Arduino-ESP32 **3.0.7** | builds, **0 warnings**, 440,561 B flash (33 %), 64,440 B RAM (19 %) |
| **HOST** (`test/run_tests.sh`): 66 test cases × 2 core API shapes (mock core v3 and v2) | **132/132 runs pass**, 4,776 checks, `-Wall -Wextra -Wshadow`, ASan + UBSan |
| Pin map | `static_assert`s: 12 motor + 2 I2C + 4 multiplexer-select output pins all distinct (18); SIG (GPIO34) and the two module pins (GPIO35, GPIO36) distinct, none of them an output pin, all valid **input-only** GPIOs on the real SoC masks; no pin in the flash range GPIO6–11; the select pins are output-capable. The motor and MPU6050 pins are unchanged (`t_pinmap` compares them with an independent copy) |
| Removed hardware | `test/run_tests.sh` greps the firmware sources for the TSOP receivers, the old scan engine and the emitter pulsing and fails if any reappears; `t_old_hardware_removed` checks that the old commands are gone |
| STATIC | cppcheck: 1 style note (a never-used variable in `cli.h`, pre-existing); clang-tidy: no findings; Valgrind memcheck clean on 13 test cases |

## What the host tests cover

Motor pin map and mecanum mixing, ramps, reversal dwell, boot safety; configuration validation, CRC and persistence (ring settings survive a reboot); the multiplexer scan (select lines, channel isolation,
C12–C15 not scanned until enabled, window versus the 833 µs burst period, DC rejection, noise and thresholds with hysteresis, noise-floor calibration, saturation faults, no usable channel, stalled scan, coverage gap);
the bearing computation and its wrap-around; the modules (debounce, polarity, module-only versus ring-confirmed shooting position, module 1 inert); the navigation (turn towards the ball, approach and slow-down, strafe when close, stop at the shooting position,
search direction from the ball's motion / nearest uncovered arc / last side, re-acquisition, bounded search, safe stop on a ring fault); the goal heading and the shot (preconditions, aim, reverse, full-power drive, launch clock, one shot per ball, cancellation);
the IMU; the CLI; logging (rotation, queue overflow, flight recorder, export, reboot persistence).
**The ring input is a simulation**: a made-up cosine response per channel with a made-up burst waveform and made-up thresholds. It checks the logic, not the physical sensors.

Simulated shot timing (assumed rotation model of 180°/s at full rotation command, which is **not** a measured value): from "ball in the shooting position" to the start of the forward drive,
436 ms (robot already pointing at the goal) and 695 ms (40° off), against 715 and 1445 ms for the sequential settings (`shootparallel 0`, slower aim, longer hold, reverse first).

## What was NOT verified

* **The real phototransistors and circuit**: whether the ball's pulses reach the multiplexer pin as a usable swing, the load resistor, the response time, range, the real mounting angles and field of view of each receiver (the bearings are placeholders), lighting and motor-noise sensitivity (**PHYS**).
* **The strapping pins**: whether the multiplexer breakout lets the ESP32 boot and upload with S2 on GPIO12 and S3 on GPIO0 (**PHYS**); the real ADC conversion time (the scan window is time-based, so it adapts, but the time per scan is unmeasured).
* **The modules**: the pin of module 1 (GPIO36 is my proposal), the polarity and distance setting, whether their LEDs disturb the ring.
* The ball's actual burst pattern (192 µs / 833 µs comes from the ball firmware source I read, not from a measurement of your ball).
* The motors' direction and rotation, speeds, stall, the reset-state of the L298N inputs, the power supply (**PHYS**).
* The gyro drift over a match and the real rotation speed used by the aim; whether 400 ms of full power is the right shot (**PHYS**).
* The goal width of your rule set (the current rulebook could not be downloaded here).
* The firmware has not been run on the ESP32 hardware at all.

## Defects found during this work (and fixed)

* The shot's launch detection compared the wheels with the previous target, so the "drive" clock started during the reverse phase (the full-power drive was shortened by the reversal dwell). It now starts when the wheels actually turn forward (a test measures full speed within about 10 ms of the launch).
* A registered goal heading survived gyro resets (`:zero`, recalibration, IMU outage) and would have aimed at a meaningless direction; it is now tied to the gyro's reference and invalidated.
* The acceleration limit would have spent most of the shot's drive ramping up; it is lifted for the shot.
