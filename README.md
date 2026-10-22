# Robosoccer26 Cyberbots – RoboCup Junior 2026 autonomous soccer robot firmware

Firmware for an ESP32-S (38-pin) robot: four mecanum wheels on two L298N drivers, **a 12-channel IR phototransistor ring on a CD74HC4067 multiplexer** (sees the ball's own infrared emission; optional extra receivers on C12–C15 for left/right) and **two IR obstacle modules** (front: ball in the shooting position),
an MPU6050 for relative heading, LittleFS CSV logging with a RAM flight recorder, and a Hardware Health Monitor that degrades or stops instead of guessing.

> **Status – read this first.**
>
> | Verified by | Result |
> |---|---|
> | **Real ESP32 compilation** (Arduino-ESP32 **2.0.17** and **3.0.7**, real Xtensa GCC, `--warnings all`) | builds, **0 warnings**, 31–33 % flash, 19–20 % RAM |
> | **Host simulation** (simulated sensor values, IMU, filesystem) | 66 test cases × 2 API shapes = 132 runs, 4,776 checks, all pass, ASan/UBSan clean |
> | Requires the real sensors and the real ball | **not done**: the receivers' response to the real ball, the real mounting angles, the module distance setting and the strapping-pin behaviour of the multiplexer wiring are unknown to the firmware and untested |
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
| 1 | CD74HC4067 16-channel analog multiplexer (EN tied to GND) |
| 12 | 5 mm IR phototransistor on multiplexer C0–C11 (the ring; **part number not known**; each needs a load resistor, it cannot go straight to a channel: collector to 3.3 V, emitter to the channel and a resistor (start 10 kΩ) to GND; `docs/RING_AND_MODULES.md` §4) |
| 0–4 | optional extra phototransistors on C12–C15 (planned: looking left / right; you add them, `:ring on <ch>`) |
| 2 | IR obstacle module, 3-pin (VCC / GND / OUT, trimmer), 3.3 V: module 0 front (shooting position), module 1 position not decided |
| 1 | MPU6050 |
| 1 | breadboard, jumper wires, power wiring |

**No emitter LEDs are controlled by the firmware** (the ball is the emitter). **No kicker or other shooting actuator** (none is assumed; the "shot" is a drive burst).

### Wiring (pins fixed in `pins.h`)

| ESP32 GPIO | Connects to |
|---|---|
| **25 / 26 / 27 / 14** | **Right** L298N IN1 / IN2 / IN3 / IN4 |
| **33 / 32** | Right L298N ENA / ENB (PWM) |
| | Right OUT1/OUT2 = **front-right** motor, OUT3/OUT4 = **back-right** motor |
| **18 / 19 / 21 / 22** | **Left** L298N IN1 / IN2 / IN3 / IN4 |
| **23 / 5** | Left L298N ENA / ENB (PWM) |
| | Left OUT1/OUT2 = **front-left** motor, OUT3/OUT4 = **back-left** motor |
| **34** | CD74HC4067 **SIG** (analog input, input-only, ADC1) |
| **4 / 13 / 12\* / 0\*** | CD74HC4067 **S0 / S1 / S2 / S3** (\* strapping pins: `docs/RING_AND_MODULES.md` §3) |
| **35** | IR obstacle module 0 OUT (front, shooting position) |
| *36* | *IR obstacle module 1 OUT: **my proposal** (you wrote "34 and 35", GPIO34 is SIG)* |
| **16 / 17** | MPU6050 SDA / SCL |
| 1 / 3 | USB serial (115200 baud) |

Battery + / − go to both L298N motor-supply and ground terminals (fuse and a switch within reach); **all grounds common**. How the ESP32 itself is powered is not specified in the project notes
(do not feed it from the L298N regulator with a charged 3S pack, ~12.6 V). Full tables, electrical notes and the boot/reset analysis: `docs/HARDWARE_REVIEW.md`.

**Free GPIOs:** 39 (input-only, ADC1) · 2 and 15 (strapping pins: usable as outputs, avoid pull-ups on GPIO2). GPIO0 and GPIO12 are the multiplexer's S3 and S2 (strapping pins). GPIO6–11 are the flash and cannot be used.

## How the ring and the modules are used (`docs/RING_AND_MODULES.md`)

* **The ring** (C0–C11, front and back, with a slight blind spot) gives each receiver's **swing** (peak-to-peak over 900 µs, which spans the ball's pulse gaps; steady light has no swing). The ball's **bearing** (0 = ahead, + = right) is computed from the channels that see it and their configured mounting angles. **The default mounting angles are placeholders (even 30° spacing): set the real ones with `:ring bearing <ch> <deg>`.**
* **Module 0 (front)** detecting = the ball is in the shooting position. By default the ring must also see the ball straight ahead (a wall or a robot also trips a module). Module 1 is read and logged only until you decide its job.
* **Blind spots:** a ball in an uncovered arc is invisible. When the ball is lost the robot rotates towards where it went (the way it was moving, else the nearest uncovered arc, else its last side) and re-acquires; the search is bounded.
* **C12–C15** (left/right receivers you add): `:ring on 12`, `:ring bearing 12 90` etc.
* **A channel gives strength only**, not distance; nothing about the real ball is verified (`NOT TESTED`).

## Autonomous behaviour

`a` starts autonomy (refused until `:ring cal noise` has been done, module 0 is on and a ring channel works, or in SAFE_STOP). SEEK → APPROACH (turn towards the ball when it is more than `turndeg` off the front, drive forward slowing as the signal rises, strafe to centre when close, heading held by the gyro) → stop at the shooting position (module 0);
ball lost (`losthold` 300 ms) → SEARCH (rotate toward where it went, pause, reverse direction, wait; bounded). With `:set shootenable 1`, a registered goal heading and the ball held centred: aim at the goal heading, reverse a little, drive forward at full power.
Motor commands are ramped (acceleration, deceleration, a dwell at zero when a wheel reverses); sensor faults stop or degrade the robot; the MPU6050 failing disables heading hold and the shot.

## Repository layout

| Path | Contents |
|---|---|
| `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` | Arduino sketch (`setup`/`loop`, boot sequence, watchdog) |
| `Robosoccer26_Cyberbots/*.h` | Modules, one translation unit: `pins` `util` `config` `storage` `health` `motors` `ring` `imu` `logger` `nav` `report` `ringcli` `cli` |
| `test/` | Host test harness: `mock_arduino.h`, `test_main.cpp`, `run_tests.sh`, `static_analysis.sh` |
| `docs/RING_AND_MODULES.md` | The multiplexer ring and the two modules: wiring, strapping pins, receiver circuit, how a channel is measured, bearing, blind spots and search, shooting position, calibration and testing, questions I need answered |
| `docs/HARDWARE_REVIEW.md` | Component list, wiring tables, GPIO summary, electrical notes, boot/strapping analysis, limitations |
| `docs/HEALTH_AND_LOGGING.md` | Health monitor, fault codes, CSV schema, rotation, flight recorder, export |
| `docs/CALIBRATION_AND_TEST_PLAN.md` | Calibration guide, first power-on, physical test plan |
| `docs/COMMISSIONING_CHECKLIST.md` | Ordered, unticked checklist from unpowered to floor tests |
| `docs/VERIFICATION_REPORT.md` | Evidence, what was and was not verified |
| `tools/real_build.sh` | Real-toolchain build (see the header of the script for toolchain setup) |

## Compile and upload (Arduino IDE)

1. Install **Arduino IDE 2.x**. In *Boards Manager* install **"esp32 by Espressif Systems"**. Versions **2.0.17** and **3.0.7** have been compiled with the real toolchains (zero warnings); newer releases are untested. The code selects the LEDC and watchdog APIs with `ESP_ARDUINO_VERSION_MAJOR`.
2. Open `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` (the folder name must match the sketch name). **No external libraries are required** (`Wire`, `LittleFS` and FreeRTOS ship with the core; the MPU6050 is driven through raw registers).
3. *Tools → Board → ESP32 Arduino → **ESP32 Dev Module***; *Tools → Partition Scheme → **Default 4MB with spiffs** (1.2 MB APP / 1.5 MB SPIFFS)*: LittleFS needs the `spiffs` label (without it the robot still runs; logging is reported OFF and nothing is ever auto-formatted); *Upload Speed* 921600 or lower; select the *Port*.
4. **First upload with the motor battery disconnected** (USB only). Click *Upload*. If it stays on "Connecting…", hold the board's BOOT button until the upload starts.
5. *Tools → Serial Monitor*, **115200 baud**, line ending "Both NL & CR" or "No line ending". Send `?` for the commands. The boot report must show autonomy DISABLED and no wheel moving.
6. Set the mounting angles and calibrate (`docs/RING_AND_MODULES.md` §9), then `:save`.

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
| `i` | snapshot of the ring channels, the two modules and the ball state |
| `I` | toggle rate-limited live ring view (`streamms`, default 500 ms) |
| `m` | MPU6050 readings and heading estimate |
| `t` | test mode: stops everything, prints the bench checklist (motors stay off) |
| `a` | enable autonomous ball seeking (refused if SAFE_STOP, the ring noise floor is not calibrated, or module 0 is off) |
| `h` | full hardware-health report |
| `d` | navigation state, ball state, heading, request, four commanded PWM values |
| `g` | logger status, buffers, storage, file list |
| `x` | export the selected log (`:xsel ring` or `:xsel <n>`) |
| `?` | command list |

Word commands: `:set <name> <v>`, `:get [name]`, `:save`, `:load`, `:defaults`, `:rev <FL|FR|BL|BR> <0|1>`, `:roller <X|O>`,
`:ring [on|off <ch> | bearing <ch> <deg> | cal noise | cal near | reset]`, `:mod [on|off <0|1> | level low|high]`, `:goal [set|clear]`, `:zero`, `:gcal`, `:diag <on|off>`,
`:mtest <wheel>`, `:dtest`, `:confirm`, `:mok <wheel>`, `:mrev <wheel>`, `:recover`, `:xsel <ring|n>`, `:fsformat CONFIRM`, `:logdel CONFIRM`.

Input handling: CR/LF/tab/space ignored; unknown or non-printable bytes are ignored (rate-limited warning); `:` lines are limited to 63 characters, rejected if too long or containing control characters, and abandoned after 4 s without Enter.
Diagnostic commands never start motors; motor tests require `:mtest`/`:dtest` **and** `:confirm`.

## Running the host tests

```
./test/run_tests.sh        # host simulation: mock core v3 and v2, -Wall -Wextra -Wshadow, ASan+UBSan, every case in its own process
./test/static_analysis.sh  # cppcheck, clang-tidy, Valgrind
tools/real_build.sh both   # REAL Arduino-ESP32 2.0.17 + 3.0.7 compile (see the header of the script for toolchain setup)
```
