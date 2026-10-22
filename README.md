# Robosoccer26 Cyberbots – RoboCup Junior 2026 autonomous soccer robot firmware

Firmware for an ESP32-S (38-pin) robot: four mecanum wheels on two L298N drivers, **two IR transmitter–receiver pairs at the front** (2 IR LEDs + 2 two-pin IR receivers, reflected-light ball detection; no multiplexer, no sensor ring),
an MPU6050 for relative heading, LittleFS CSV logging with a RAM flight recorder, and a Hardware Health Monitor that degrades or stops instead of guessing.

> **Status – read this first.**
>
> | Verified by | Result |
> |---|---|
> | **Real ESP32 compilation** (Arduino-ESP32 **2.0.17** and **3.0.7**, real Xtensa GCC, `--warnings all`) | builds, **0 warnings**, 31–33 % flash, 19 % RAM |
> | **Host simulation** (simulated sensor values, IMU, filesystem) | 65 test cases × 2 API shapes = 130 runs, 4,656 checks, all pass, ASan/UBSan clean |
> | Requires the real sensors and the real ball | **not done**: the sensors' type, polarity, range, field of view and response to the ball are unknown to the firmware and untested |
> | Requires electrical inspection and supervised robot tests | **not done**: pull-downs, reset-state voltages, motor directions, ramps, heading, autonomy, the shot |
>
> **The robot is not competition-ready.** Follow `docs/COMMISSIONING_CHECKLIST.md`. Details: `docs/VERIFICATION_REPORT.md`.

## Hardware (what the firmware is written for)

| Qty | Part |
|---:|---|
| 1 | ESP-32S v1.1 development board (38 pins) |
| 2 | L298N motor driver |
| 4 | yellow BO geared DC motor |
| 4 | mecanum wheel |
| 1 | 3S 11.1 V 4500 mAh battery |
| 2 | IR transmitter LED (**exact part not known**) |
| 2 | 5 mm IR phototransistor (2-pin receiver; **part number not known**; it needs a load resistor, it cannot go straight to a GPIO: collector to 3.3 V, emitter to the pin and a resistor (start 10 kΩ) to GND; see `docs/IR_SENSORS.md` §2.1) |
| 1 | MPU6050 |
| 1 | breadboard, jumper wires, power wiring |

**Exactly 2 IR LEDs and 2 IR receivers. No multiplexer. No 12-sensor ring. No kicker or other shooting actuator** (none is assumed; the "shot" is a drive burst).

### Wiring (pins fixed in `pins.h`)

| ESP32 GPIO | Connects to |
|---|---|
| **25 / 26 / 27 / 14** | **Right** L298N IN1 / IN2 / IN3 / IN4 |
| **33 / 32** | Right L298N ENA / ENB (PWM) |
| | Right OUT1/OUT2 = **front-right** motor, OUT3/OUT4 = **back-right** motor |
| **18 / 19 / 21 / 22** | **Left** L298N IN1 / IN2 / IN3 / IN4 |
| **23 / 5** | Left L298N ENA / ENB (PWM) |
| | Left OUT1/OUT2 = **front-left** motor, OUT3/OUT4 = **back-left** motor |
| **35** | IR receiver 0, **front centre** (input-only, ADC1), through a receiver circuit, not directly |
| **36** | IR receiver 1, **front, slightly off-centre** (input-only, ADC1) |
| *4 / 13* | *IR emitter drivers 0 / 1: **proposed, not from your notes**; used only after `:set iremit 1`, through a transistor driver* |
| **16 / 17** | MPU6050 SDA / SCL |
| 1 / 3 | USB serial (115200 baud) |

Battery + / − go to both L298N motor-supply and ground terminals (fuse and a switch within reach); **all grounds common**. How the ESP32 itself is powered is not specified in the project notes
(do not feed it from the L298N regulator with a charged 3S pack, ~12.6 V). Full tables, electrical notes and the boot/reset analysis: `docs/HARDWARE_REVIEW.md`.

**Free GPIOs:** 34 and 39 (input-only, ADC1) · 2 and 15 (strapping pins: usable as outputs, avoid pull-ups on GPIO2) · 0 and 12 (boot strapping: deliberately unused). GPIO6–11 are the flash and cannot be used.

## How the two IR pairs are used (`docs/IR_SENSORS.md`)

Each pair is an IR LED plus a receiver; the receiver measures the infrared light **reflected from the ball**. This detects the ball, not the goal.

* **Pair 0 (front centre)** reports how strong the reflection is. When it reaches the *near* level the ball is "in the shooting position". The robot does not shoot because a ball is merely visible far away.
* **Pair 1 (offset)** lets the robot judge centred or off-centre: the stronger pair shows the side; a ball seen by only one pair gives a cautious sideways move toward its side (needs `:set iroffset 1|-1` = pair 1 right/left of pair 0).
* **Both see the ball and pair 0 is near** → shooting position (assumes the two beams overlap there; `:set irneedboth 0` if they do not).
* **Ambient light:** with `:set iremit 1` the firmware pulses each emitter and reads its receiver dark and lit; the signal is `2048 + (lit − dark)`, so ambient light and the ball's own emission cancel. This needs the emitter drivers wired (proposed GPIO4 / GPIO13); with the default `iremit 0` the LEDs are assumed hard-wired on and ambient light is **not** cancelled.
* **A reflective pair gives strength only**: no position, no distance, a limited range and field of view. The robot therefore *searches by rotating*; whether that finds a ball depends on the range. See the options in `docs/IR_SENSORS.md`.
* **Nothing is assumed about the receivers:** analog or digital is a setting, which direction means "ball" is learned by `:ir cal`, and **the receiver circuit and the LED driver are candidates until you give the part numbers** (`docs/IR_SENSORS.md` §8).

## Autonomous behaviour

`a` starts autonomy (refused until sensor 0 is calibrated, or in SAFE_STOP). SEEK → APPROACH (drive forward, slowing as the signal rises; centre with sensor 1; heading held by the gyro) → stop at the shooting position;
ball lost (`losthold` 300 ms) → SEARCH (rotate toward the last side, pause, reverse direction, wait; bounded). With `:set shootenable 1`, a registered goal heading and the ball held centred: aim at the goal heading, reverse a little, drive forward at full power.
Motor commands are ramped (acceleration, deceleration, a dwell at zero when a wheel reverses); sensor faults stop or degrade the robot; the MPU6050 failing disables heading hold and the shot.

## Repository layout

| Path | Contents |
|---|---|
| `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` | Arduino sketch (`setup`/`loop`, boot sequence, watchdog) |
| `Robosoccer26_Cyberbots/*.h` | Modules, one translation unit: `pins` `util` `config` `storage` `health` `motors` `ir2` `imu` `logger` `nav` `report` `ircli` `cli` |
| `test/` | Host test harness: `mock_arduino.h`, `test_main.cpp`, `run_tests.sh`, `static_analysis.sh` |
| `docs/IR_SENSORS.md` | The two IR pairs: can the receivers connect directly (no), candidate receiver and LED-driver circuits, ambient subtraction, limits, calibration and testing, the shot, questions I need answered |
| `docs/HARDWARE_REVIEW.md` | Component list, wiring tables, GPIO summary, electrical notes, boot/strapping analysis, limitations |
| `docs/HEALTH_AND_LOGGING.md` | Health monitor, fault codes, CSV schema, rotation, flight recorder, export |
| `docs/CALIBRATION_AND_TEST_PLAN.md` | Calibration guide, first power-on, physical test plan |
| `docs/COMMISSIONING_CHECKLIST.md` | Ordered, unticked checklist from unpowered to floor tests |
| `docs/VERIFICATION_REPORT.md` | Evidence, what was and was not verified |
| `docs/CHANGES_TWO_SENSOR.md` | What was removed, kept and added, and the files changed |
| `tools/real_build.sh` | Real-toolchain build (see the header of the script for toolchain setup) |

## Compile and upload (Arduino IDE)

1. Install **Arduino IDE 2.x**. In *Boards Manager* install **"esp32 by Espressif Systems"**. Versions **2.0.17** and **3.0.7** have been compiled with the real toolchains (zero warnings); newer releases are untested. The code selects the LEDC and watchdog APIs with `ESP_ARDUINO_VERSION_MAJOR`.
2. Open `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` (the folder name must match the sketch name). **No external libraries are required** (`Wire`, `LittleFS` and FreeRTOS ship with the core; the MPU6050 is driven through raw registers).
3. *Tools → Board → ESP32 Arduino → **ESP32 Dev Module***; *Tools → Partition Scheme → **Default 4MB with spiffs** (1.2 MB APP / 1.5 MB SPIFFS)*: LittleFS needs the `spiffs` label (without it the robot still runs; logging is reported OFF and nothing is ever auto-formatted); *Upload Speed* 921600 or lower; select the *Port*.
4. **First upload with the motor battery disconnected** (USB only). Click *Upload*. If it stays on "Connecting…", hold the board's BOOT button until the upload starts.
5. *Tools → Serial Monitor*, **115200 baud**, line ending "Both NL & CR" or "No line ending". Send `?` for the commands. The boot report must show autonomy DISABLED and no wheel moving.
6. Calibrate the sensors (`docs/IR_SENSORS.md`), then `:save`.

## Safety first

* First tests with the **wheels off the ground**, robot secured, motor battery switch/connector within reach.
* The robot **never moves at boot** and **never resumes autonomy after a reset or fault**; autonomy needs an explicit `a`. Manual motion auto-stops after `manualhold` ms (default 600 ms) unless repeated.
* Fit **external pull-downs (4.7–10 kΩ) on all twelve L298N control lines** (`docs/HARDWARE_REVIEW.md` §5). Software cannot control pins while the ESP32 is in reset or boot ROM.
* Software failsafes are not a substitute for fuses, current limits, a regulated logic supply, or a physical power switch.
* The shot drives at full power (`shootpwm` 255) and is off by default: test it with the wheels lifted first.

## Serial command guide

Single keys act immediately. Words start with `:` and end with Enter.

| Key | Behaviour |
|---|---|
| `f` `b` / `l` `r` / `q` `e` | manual forward-backward / strafe left-right / rotate counter-clockwise-clockwise |
| `s` | stop and disable autonomy |
| `!` | emergency stop (also works in the middle of a `:` line) |
| `i` | snapshot of both IR pairs and the ball state |
| `I` | toggle rate-limited live IR view (`streamms`, default 500 ms) |
| `m` | MPU6050 readings and heading estimate |
| `t` | test mode: stops everything, prints the bench checklist (motors stay off) |
| `a` | enable autonomous ball seeking (refused if SAFE_STOP or pair 0 is not calibrated) |
| `h` | full hardware-health report |
| `d` | navigation state, ball state, heading, request, four commanded PWM values |
| `g` | logger status, buffers, storage, file list |
| `x` | export the selected log (`:xsel ring` or `:xsel <n>`) |
| `?` | command list |

Word commands: `:set <name> <v>`, `:get [name]`, `:save`, `:load`, `:defaults`, `:rev <FL|FR|BL|BR> <0|1>`, `:roller <X|O>`,
`:ir [cal clear|near <0|1> | mode <0|1> analog|digital | on|off <0|1> | emit on|off | side right|left|none | reset]`, `:goal [set|clear]`, `:zero`, `:gcal`, `:diag <on|off>`,
`:mtest <wheel>`, `:dtest`, `:confirm`, `:mok <wheel>`, `:mrev <wheel>`, `:recover`, `:xsel <ring|n>`, `:fsformat CONFIRM`, `:logdel CONFIRM`.

Input handling: CR/LF/tab/space ignored; unknown or non-printable bytes are ignored (rate-limited warning); `:` lines are limited to 63 characters, rejected if too long or containing control characters, and abandoned after 4 s without Enter.
Diagnostic commands never start motors; motor tests require `:mtest`/`:dtest` **and** `:confirm`.

## Running the host tests

```
./test/run_tests.sh        # host simulation: mock core v3 and v2, -Wall -Wextra -Wshadow, ASan+UBSan, every case in its own process
./test/static_analysis.sh  # cppcheck, clang-tidy, Valgrind
tools/real_build.sh both   # REAL Arduino-ESP32 2.0.17 + 3.0.7 compile (see the header of the script for toolchain setup)
```
