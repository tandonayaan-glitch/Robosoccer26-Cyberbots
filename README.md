# Robosoccer26 Cyberbots – RoboCup Junior 2026 autonomous soccer robot firmware (Arduino Mega 2560)

Firmware for an **Arduino Mega 2560**: four mecanum wheels on two L298N drivers, six 5 mm IR phototransistor receivers (A0–A5, each on its own analog input) that see the ball's own infrared emission, two digital IR obstacle modules, an underside ball-position sensor (D40), an MPU6050 for a relative heading, an HC-05 Bluetooth module (Serial1) talking to an **ESP32 remote**, EEPROM-stored settings and a Hardware Health Monitor that degrades or stops instead of guessing.
Modes: **ATTACK** (autonomous ball seeking and shooting-position alignment), **DEFEND** (`defend`: heading hold + sideways intercept/block), **MANUAL**, **CALIBRATION**, **IDLE**, **FAULT**. **No multiplexer. No kicker, no encoders, no current sensors.** The earlier ESP32-S version is preserved in git as commit `1963a3f` (local tag `esp32-final`).

> **Status – read this first.**
>
> | Verified by | Result |
> |---|---|
> | **Real ATmega2560 compilation** (Arduino AVR core 1.8.6, avr-gcc 7.3.0, `tools/avr_build.sh`) | builds, **0 warnings**: 125,542 B flash (49 %), 5,031 B static RAM (61 %), 3,161 B left for the stack |
> | **Host simulation** (mock Arduino core, simulated sensors, IMU, EEPROM, Bluetooth) | 80 test cases, 2,734 checks, all pass, ASan/UBSan clean; static analysis (cppcheck, clang-tidy, valgrind) clean |
> | **ESP32 sketch** | **not built for an ESP32** (no toolchain). Its protocol parser is unit-tested on the host; the sketch is only type-checked against host stubs (`tools/esp32_syntax_check.sh`) |
> | Upload / run on a Mega, HC-05 pairing, range, receivers, modules, underside sensor, motors, power, MPU6050 on the real robot | **not done** – every behaviour below is unverified on hardware |
>
> **The robot is not competition-ready.** Follow `docs/COMMISSIONING_CHECKLIST.md` and the staged plan in `docs/CALIBRATION_AND_TEST_PLAN.md`. Details: `docs/VERIFICATION_REPORT.md`.

## Commands (Serial Monitor 115200 baud, line ending **Newline** or **Both NL & CR**; the same words work over Bluetooth)

`defend` · `attack` · `manual` · `stop` (or `idle`) · `status` · `sensors` · `gyro` (`gyro drift [s]`, `gyro cal`, `gyro stop`) · `calibrate` · `motors` · `tele` · `help`; emergency stop `!` (USB) / `X` (Bluetooth).
Classic single keys (`f b l r q e s a h i m d g t ?`) work **when sent alone**; `:set`, `:get`, `:save`, `:ring`, `:mod`, `:log`, … are unchanged. A multi-letter burst that is not a known word executes nothing. Full list: `help`; protocol: `docs/BLUETOOTH.md`; telemetry: `docs/TELEMETRY.md`; defending: `docs/DEFENDING.md`.

**Startup:** after a clean boot the robot starts ATTACK by itself after 1.5 s (`autostart`, team decision; `:set autostart 0` + `:save` for always-idle). DEFEND is never started automatically. Bluetooth is not required.

## Build and test

```
tools/avr_build.sh            # real ATmega2560 compile (avr-gcc + Arduino AVR core from Debian packages); never uploads
test/run_tests.sh             # host simulation, one process per test case, ASan/UBSan
test/static_analysis.sh       # cppcheck, clang-tidy, valgrind
tools/esp32_syntax_check.sh   # type-check of the ESP32 sketch against host STUBS (not an ESP32 build)
```
Arduino IDE: open `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino`, board "Arduino Mega or Mega 2560". ESP32: open `remote_esp32/remote_esp32.ino`, an **original ESP32** board (Classic Bluetooth), partition "Huge APP".

## Unverified hardware assumptions (all of them)
HC-05 breakout supply voltage and AT syntax · the divider on Mega TX1 · phototransistor pinout and junction voltages · where each receiver looks (angles are a guess until `:ring bearing`) · receiver response to the real ball · D40 sensor type and active level · module polarity · MPU6050 axis signs, pull-ups and supply · motor directions and the power supply · strafe speed/slip · everything about defending or shooting a real ball · Bluetooth range · the exact ESP32 board.

## Pin map (fixed in `pins.h`)

| Mega pin | Connects to |
|---|---|
| **D22 / D23 / D5** | **Right** L298N IN1 / IN2 / ENA (PWM) → OUT1/OUT2 = **front-right** motor |
| **D24 / D25 / D6** | Right L298N IN3 / IN4 / ENB (PWM) → OUT3/OUT4 = **back-right** motor |
| **D26 / D27 / D9** | **Left** L298N IN1 / IN2 / ENA (PWM) → OUT1/OUT2 = **front-left** motor |
| **D28 / D29 / D10** | Left L298N IN3 / IN4 / ENB (PWM) → OUT3/OUT4 = **back-left** motor |
| **A0 … A5** | phototransistor receivers 0–5 (each its own junction and 10 kΩ load resistor); bearings via `:ring bearing` |
| A6, A7 | spare (off until fitted: `:ring on <n>`) |
| **D18 / D19** | HC-05 RXD (through a 1 kΩ/2 kΩ divider) / HC-05 TXD (Serial1) |
| **D40** | underside ball-position sensor (active level is the `underlow` setting) |
| **D34** | front IR obstacle module OUT (ball in the shooting position) |
| **D35** | second IR obstacle module OUT (obstacle input for defend mode, logged) |
| **D20 / D21** | MPU6050 SDA / SCL (hardware I2C) |
| D0 / D1 | USB serial (115200 baud) |

Receiver circuit: phototransistor **collector → 5 V**, **emitter → its junction**, **10 kΩ junction → GND**, junction → its analog input; the receivers are never joined. Any IR transmitter LEDs that are fitted are hard-wired and **not controlled by the firmware**. **Verify the phototransistor pinout before applying 5 V.**
Battery + goes through the power switch to both L298N motor-supply inputs; **all grounds are common**; **never put the battery's ~11 V on the Mega's 5 V pin**; do not power the electronics from the L298N's onboard 5 V. Full wiring, voltage checks, power options and pull-down advice: **`docs/WIRING_MEGA.md`**.

## Hardware (what the firmware is written for)

1 Arduino Mega 2560 · 2 L298N · 4 BO motors + 4 mecanum wheels · battery pack + power switch · 6 IR phototransistors (5 mm, 2-pin) · 2 IR obstacle modules · 1 underside IR sensor · 1 HC-05 · (ESP32 remote) · 1 MPU6050 · breadboard, wires, resistors.

## How the ball is sensed (`docs/BALL_SENSING.md`)

* Each receiver gives a **swing** (peak-to-peak over 900 µs, which spans the gaps between the ball's pulses; steady light has no swing and is ignored). The ball's **bearing** (0 = ahead, + = right) comes from the receivers that see it and their configured mounting angles. **The mounting angles default to 0° for both receivers: set the real ones with `:ring bearing <n> <deg>`.**
* The **front module** detecting = the ball is in the shooting position; by default the receivers must also see the ball straight ahead. The second module is read and logged only.
* **Polarity of each module is a setting** (`:mod level <0|1> low|high`), not an assumption.
* **Blind spots:** when the ball is lost the robot rotates towards where it went (its motion, the nearest uncovered arc, its last side); the search is bounded.
* The receivers give **strength only** (no distance); nothing about the real ball is verified.

## Autonomous behaviour

`a` starts autonomy (refused until `:ring cal noise` has been done, the front module is on and a receiver works, or in SAFE_STOP). SEEK → APPROACH (turn towards the ball, drive forward slowing as the signal rises, strafe to centre when close, heading held by the gyro) → stop at the shooting position;
ball lost (`losthold` 300 ms) → SEARCH (rotate towards where it went, pause, reverse direction, wait; bounded). With `:set shootenable 1`, a registered goal heading and the ball ready: aim at the goal heading, reverse a little, drive forward at full power (no kicker).
Motor commands are ramped; faults stop or degrade the robot; the MPU6050 failing disables heading hold and the shot. Motors stay off at boot and after every reset.

## What happens when something is missing

The firmware starts and keeps running; it tells you what is missing instead of stopping (`:status` and `:raw` show it at any time).

| Missing | What happens |
|---|---|
| **MPU6050** (or its wiring) | boots; `IMU_MISSING` → DEGRADED (reduced speed). Manual driving and autonomy work; no heading hold, no goal heading, **no shot**. It retries every few seconds. |
| **A receiver / its wiring** | an open analog input floats and reads random values, so **a missing receiver cannot be detected reliably** (the firmware has no way to tell a floating input from a quiet one). `:raw` shows it (random numbers instead of ~0 in the dark); `:ring cal noise` then stores that input's noise and raises its threshold. Switch an unused receiver off: `:ring off <n>`. |
| **All receivers** (none enabled) | `RING_NONE` → SAFE_STOP; autonomy refused, manual driving still allowed. |
| **A module / its wiring** | an active-LOW module (the default) has the internal pull-up, so a missing wire reads "nothing detected": no shot, the robot keeps approaching slowly (it never sees "ball in place"). Active-HIGH modules have no pull-down on the AVR and would float. |
| **Serial / USB** | nothing waits for it; the robot runs normally, output is dropped when nobody reads it. |
| **Saved settings** (blank or corrupt EEPROM) | defaults are used and reported; autonomy still needs `:ring cal noise` first. |
| **Motors / L298N** | cannot be detected (no feedback): the PWM shown is the commanded value. |

## Records: how to see and download them

The robot keeps its own event log in the Mega's EEPROM (about 300 records, survives power-off): boot, faults and recoveries, navigation transitions, ball losses, shots, SAFE_STOP, configuration saves. You do not have to do anything to write them; they are written by themselves.

* **See them:** open the Serial Monitor (115200 baud) and type `:log` + Enter. The newest are at the bottom; `t_ms` counts from the `BOOT` line above it. `:log live on` prints each new record the moment it happens; `:log note 5` adds a marker of your own; `:log clear` erases them.
* **Download them to `records.txt` on the PC:** close the Serial Monitor, then run
  `pip install pyserial` (once) and `python3 tools/get_records.py` (it finds the Mega; or `python3 tools/get_records.py COM5`). It writes `records.txt` next to where you run it. Opening the port restarts the Mega (as when uploading): nothing is lost, one more `BOOT` record is added, and the robot does not move.
  `python3 tools/get_records.py --follow` instead saves **everything** the robot prints (with PC time stamps) until Ctrl-C: use it during a run; `:diag on` additionally prints one CSV status row every 250 ms.
* The Mega cannot create files itself; `records.txt` is made by the PC tool from what the robot prints. Not tested on a real Mega.

## Repository layout

| Path | Contents |
|---|---|
| `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` | Arduino sketch (`setup`/`loop`, boot sequence) |
| `Robosoccer26_Cyberbots/*.h` | Modules, one translation unit: `compat` `pins` `util` `config` `storage` `health` `motors` `ring` `imu` `logger` `nav` `report` `ringcli` `cli` |
| `test/` | Host test harness: `mock_arduino.h`, `test_main.cpp`, `run_tests.sh`, `static_analysis.sh` |
| `tools/avr_build.sh` | Real ATmega2560 compile (avr-gcc + the Arduino AVR core) and flash/RAM report |
| `tools/get_records.py` | Saves the robot's EEPROM event records (or everything it prints) to `records.txt` on the PC |
| `docs/WIRING_MEGA.md` | Components, pin map, receiver / LED / module / MPU6050 circuits, voltage checks, power and ground, boot safety |
| `docs/BALL_SENSING.md` | Receivers and modules in the firmware, bearing, blind spots, calibration, open questions |
| `docs/HEALTH_AND_LOGGING.md` | Health monitor, fault codes, Serial CSV |
| `docs/CALIBRATION_AND_TEST_PLAN.md`, `docs/COMMISSIONING_CHECKLIST.md` | Calibration, first power-on, ordered unticked checklist |
| `docs/VERIFICATION_REPORT.md` | Evidence, what was and was not verified |

## Compile and upload (Arduino IDE)

1. Install **Arduino IDE 2.x**. In *Boards Manager* install **"Arduino AVR Boards"** (version **1.8.3 or newer**; 1.8.6 was used for the build check).
2. Open `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` (the folder name must match the sketch name). **No external libraries are required** (`Wire` and `EEPROM` ship with the core; the MPU6050 is driven through raw registers).
3. *Tools → Board → Arduino AVR Boards → **Arduino Mega or Mega 2560***; *Tools → Processor → **ATmega2560 (Mega 2560)***; select the *Port*.
4. **First upload with the motor battery disconnected** (USB only). Click *Upload*.
5. *Tools → Serial Monitor*, **115200 baud**. Send `?` for the commands. The boot report must show autonomy DISABLED and no wheel moving.
6. Set the mounting angles and calibrate (`docs/BALL_SENSING.md` §5), then `:save`.

Command-line build check without the IDE: `tools/avr_build.sh` (needs `gcc-avr avr-libc binutils-avr arduino-core-avr` on Debian/Ubuntu, or point `AVR_CORE` at an installed core). It compiles and links; it **does not upload**.

## Safety first

* First tests with the **wheels off the ground**, robot secured, motor battery switch/connector within reach.
* The robot **never moves at boot** and **never resumes autonomy after a reset or fault**; autonomy needs an explicit `a`. Manual motion auto-stops after `manualhold` ms (600 ms) unless repeated.
* Fit **10 kΩ pull-downs on all twelve L298N control lines**: a Mega pin floats from reset until the sketch starts and during the bootloader.
* The hardware watchdog is **off** by default (the stock Mega bootloader can loop after a watchdog reset; `docs/WIRING_MEGA.md` §8).
* Software failsafes are not a substitute for fuses, current limits, a regulated logic supply, or a physical power switch. The shot drives at full power (`shootpwm` 255) and is off by default.

## Serial command guide

Single keys act immediately. Words start with `:` and end with Enter.

| Key | Behaviour |
|---|---|
| `f` `b` / `l` `r` / `q` `e` | manual forward-backward / strafe left-right / rotate counter-clockwise-clockwise |
| `s` | stop and disable autonomy |
| `!` | emergency stop (also works in the middle of a `:` line) |
| `i` | snapshot of the receivers, the two modules and the ball state |
| `I` | toggle rate-limited live receiver view (`streamms`, default 500 ms) |
| `m` | MPU6050 readings and heading estimate |
| `t` | test mode: stops everything, prints the bench checklist (motors stay off) |
| `a` | enable autonomous ball seeking (refused if SAFE_STOP, noise floor not calibrated, or the front module off) |
| `h` | full hardware-health report |
| `d` | navigation state, ball state, heading, request, four commanded PWM values |
| `g` | diagnostics status (CSV on/off, record count, dropped lines, free RAM) |
| `?` | command list |
| `x` | not available on the Mega (no flash log); prints a message |

Word commands (`:help` lists them on the robot):

| Group | Commands |
|---|---|
| Records / diagnosis | `:log` (show records) · `:log clear` · `:log live on\|off` · `:log note <n>` · `:status` (one screen) · `:raw` (raw A0–A7, module pins, IMU) · `:pins` · `:ver` · `:diag on\|off` (CSV rows over Serial) |
| Control | `:auto on\|off` · `:stop` · `:estop` · `:manual f\|b\|l\|r\|q\|e` · `:goal [set\|clear]` · `:zero` · `:gcal` · `:recover` |
| Motor tests | `:mtest <wheel>` · `:dtest` · `:confirm` · `:mok <wheel>` · `:mrev <wheel>` |
| Settings (EEPROM) | `:set <name> <v>` · `:get [name]` · `:save` · `:load` · `:defaults` · `:rev <FL\|FR\|BL\|BR> <0\|1>` · `:roller <X\|O>` |
| Receivers / modules | `:ring [on\|off <0-7> \| bearing <0-7> <deg> \| cal noise \| cal near \| reset]` · `:mod [on\|off <0\|1> \| level <0\|1> low\|high]` |

Not available on the Mega (they answer with a message): `:xsel`, `:fsformat`, `:logdel`, `x`.

Input handling: CR/LF/tab/space ignored; unknown or non-printable bytes are ignored (rate-limited warning); `:` lines are limited to 63 characters, rejected if too long or containing control characters, and abandoned after 4 s without Enter.
Diagnostic commands never start motors; motor tests require `:mtest`/`:dtest` **and** `:confirm`. Long reports are printed in passes as the small output queue drains.

## Running the checks

```
./test/run_tests.sh        # host simulation, mock Arduino core, -Wall -Wextra -Wshadow, ASan+UBSan, every case in its own process
./test/static_analysis.sh  # cppcheck, clang-tidy, Valgrind
tools/avr_build.sh         # REAL ATmega2560 compile + link, flash / RAM report (no upload)
```

## Bluetooth remote, underside sensor, re-attack (update)

HC-05 on Serial1 (D18/D19) with an ESP32 remote (`remote_esp32/`, not compiled); protocol and wiring in `docs/BLUETOOTH.md`. Underside ball-position sensor on D40. After a failed shot the robot judges, reverses a bounded distance and re-attacks at most `remax` times, then pauses. Host tests: `test/run_tests.sh` (65 cases); AVR build: `tools/avr_build.sh`. Nothing in this update has been tested on the robot.

**Startup is automatic:** after a clean boot the robot begins autonomous mode by itself after 1.5 s (`autostart`, `startdelay`); no serial command and no Bluetooth link needed. See `docs/BLUETOOTH.md` ("Startup") for cancelling and switching modes safely.
