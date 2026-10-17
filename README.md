# Robosoccer26 Cyberbots – RoboCup Junior soccer robot firmware

Firmware for an ESP32-S (38-pin) robot: four mecanum wheels on two L298N drivers, twelve TSOP17xx
IR receivers read through a CD74HC4067 multiplexer, an MPU6050 for relative heading, LittleFS CSV
logging with a RAM flight recorder, and a Hardware Health Monitor that degrades or stops instead of
guessing.

> **Status – read this first.** Verified so far, by evidence class (details in `docs/VERIFICATION_REPORT.md`):
>
> | Verified by | Result |
> |---|---|
> | **Real ESP32 compilation** (Arduino-ESP32 **2.0.17** and **3.0.7**, real Xtensa GCC, `--warnings all`) | builds, **0 warnings**, 31–33 % flash, 20–21 % RAM |
> | Real binary in a third-party emulator | boots to the health report with autonomy disabled; no motor line rises (boot-level only) |
> | **Host simulation** (simulated ball/IMU/filesystem) | 69 test cases × 2 API shapes = 138 runs, 4,832 checks, all pass, ASan/UBSan clean |
> | Static analysis (cppcheck, clang-tidy, Valgrind) | no defects found |
> | Requires electrical inspection | **not done** (receiver pinout/supply, pull-downs, reset-state voltages, IMU board) |
> | Requires testing with the real ball | **not done** (38 kHz receiver vs 40 kHz ball, range, coverage, side gap, interference) |
> | Requires supervised physical robot tests | **not done** (motor directions, ramps, heading, autonomy) |
> | Not yet verified | datasheet/rules facts (source documents were unreachable) – `docs/IR_DETECTION_AUDIT.md` §1 |
>
> **The robot is not competition-ready.** Follow `docs/COMMISSIONING_CHECKLIST.md`.

## Repository layout

| Path | Contents |
|---|---|
| `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` | Arduino sketch (`setup`/`loop`, boot sequence, watchdog) |
| `Robosoccer26_Cyberbots/*.h` | Modules, one translation unit: `pins` `util` `config` `storage` `health` `motors` `irscan` `imu` `logger` `nav` `report` `cli` |
| `test/` | Host test harness: `mock_arduino.h`, `test_main.cpp`, `run_tests.sh` |
| `docs/HARDWARE_REVIEW.md` | Compatibility review, pin map, boot/strapping analysis, IR timing analysis, limitations |
| `docs/HEALTH_AND_LOGGING.md` | Health monitor, fault codes, CSV schema, rotation, flight recorder, export |
| `docs/CALIBRATION_AND_TEST_PLAN.md` | Calibration guide, first-power-on procedure, physical test plan |
| `docs/IR_DETECTION_AUDIT.md` | Highest-priority risk: ball/receiver compatibility, timing, interference, geometry, physical test procedure |
| `docs/COMMISSIONING_CHECKLIST.md` | Ordered, unticked checklist from unpowered to floor tests |
| `docs/VERIFICATION_REPORT.md` | Evidence classes, exact commands/results, defects found, what remains untested |
| `tools/real_build.sh`, `tools/coverage_calc.py` | Real-toolchain build; receiver geometry calculator |

## Environment and installation (Arduino IDE)

* Arduino IDE 2.x (any recent version).
* Board package: **"esp32 by Espressif Systems"**. The code supports both generations of the core
  (LEDC and watchdog APIs are selected at compile time with `ESP_ARDUINO_VERSION_MAJOR`):
  **2.0.17** and **3.0.7** have both been compiled with the real toolchains (zero warnings). Newer releases are untested.
* Board: **ESP32 Dev Module** (ESP-32S, 38 pins). Upload speed 921600 or lower.
* **Partition scheme must contain a data partition labelled `spiffs`**, e.g. *Default 4MB with spiffs
  (1.2MB APP/1.5MB SPIFFS)*. LittleFS uses that label. Without it the robot still runs; logging is
  reported OFF and nothing is ever auto-formatted.
* **No external libraries are required** (`Wire`, `LittleFS`, FreeRTOS and ESP-IDF headers ship with
  the core). The MPU6050 is driven through raw registers.
* Open `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` (the folder name must match the sketch).
* Serial Monitor: **115200 baud**; line ending "Both NL & CR" or "No line ending" both work.

## Safety first

* First tests with the **wheels off the ground**, robot secured, motor battery switch/connector within reach.
* The robot **never moves at boot** and **never resumes autonomy after a reset or fault**; autonomy
  needs an explicit `a`. Manual motion auto-stops after `manualhold` ms (default 600 ms) unless repeated.
* Fit **external pull-downs (4.7–10 kΩ) on all twelve L298N control lines**; see
  `docs/HARDWARE_REVIEW.md` §3. Software cannot control pins while the ESP32 is in reset or boot ROM.
* Software failsafes are not a substitute for fuses, current limits, a regulated logic supply, or a
  physical power switch.

## Serial command guide

Single keys act immediately. Words start with `:` and end with Enter. Output below is an **example
of the format, not a hardware measurement.**

| Key | Behaviour |
|---|---|
| `f` `b` | manual forward / backward |
| `l` `r` | manual strafe left / right |
| `q` `e` | rotate counter-clockwise / clockwise |
| `s` | stop and disable autonomy |
| `!` | emergency stop (also works in the middle of a `:` line) |
| `i` | one snapshot of all enabled IR channels |
| `I` | toggle rate-limited live IR view (`streamms`, default 500 ms) |
| `m` | MPU6050 readings and heading estimate |
| `t` | test mode: stops everything, prints the bench checklist (motors stay off) |
| `a` | enable autonomous ball seeking (refused if SAFE_STOP, or fewer than `mintrusted` calibrated channels) |
| `h` | full hardware-health report |
| `d` | navigation state, ball estimate, confidence, heading, request, four commanded PWM values |
| `g` | logger status, buffers, storage, file list |
| `x` | export the selected log (`:xsel ring` or `:xsel <n>`) |
| `?` | command list |

Word commands: `:set <name> <v>`, `:get [name]`, `:save`, `:load`, `:defaults`, `:rev <FL|FR|BL|BR> <0|1>`,
`:roller <X|O>`, `:ch <n> [<en|pol|trust|bear|tilt> <v>]` (no field = inspect that channel), `:chreset <n>`, `:learn <deg>`, `:verify <n>`,
`:cov`, `:irgate <off|auto|min max>`, `:zero`, `:gcal`, `:diag <on|off>`, `:mtest <wheel>`, `:dtest`, `:confirm`, `:mok <wheel>`,
`:mrev <wheel>`, `:recover`, `:xsel <ring|n>`, `:fsformat CONFIRM`, `:logdel CONFIRM`.

Input handling: CR/LF/tab/space ignored; unknown or non-printable bytes are ignored (rate-limited
warning); `:` lines are limited to 63 characters, rejected if too long or containing control
characters, and abandoned after 4 s without Enter. Diagnostic commands never start motors; motor
tests require `:mtest`/`:dtest` **and** `:confirm`.

### Example output (format illustration only)

```
[INFO] 41.250 IR snapshot: 12 enabled channel(s), scan cycle 36120 us (avg 36118, max 36240), cycles=1130, window=3000 us
[INFO] 41.250  ch label bearing   raw det act edges per_us  hits8 status
[INFO] 41.250  C3  AL      90     0   1   1      3    830  5/8   UNVERIFIED
[INFO] 41.250 ball: DETECTED, direction 90 deg (heuristic from 1 active channel(s)), confidence 0.53 (heuristic), channel agreement 1.00
[INFO] 41.250 no distance or signal strength is available: receivers are digital (pulse seen / not seen)
[WARN] 52.100 IMU_TIMEOUT sev=DEGRADED comp=MPU6050 evidence="IMU not delivering data (errors=3 timeouts=1)" certainty="bus errors observed; cause unproven" action="heading hold disabled, retrying init" mode=DEGRADED recovery=automatic when verified
[SAFETY] 60.004 SAFE_STOP entered: autonomous movement disabled, motors stopped. Diagnostics stay available ('h' for cause).
```

## Running the host tests

```
./test/run_tests.sh        # host simulation: mock core v3 and v2, -Wall -Wextra -Wshadow, ASan+UBSan, every case in its own process
./test/static_analysis.sh  # cppcheck, clang-tidy, Valgrind
tools/real_build.sh both   # REAL Arduino-ESP32 2.0.17 + 3.0.7 compile (see the header of the script for toolchain setup)
```
