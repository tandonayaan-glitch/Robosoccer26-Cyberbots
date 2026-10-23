# Bluetooth link: HC-05 on the Mega, ESP32 remote (commands + telemetry)

Status: **simulated and compiled only. Nothing here has been tested over the air.** The Mega firmware is compiled for the ATmega2560 (`tools/avr_build.sh`) and
tested against simulated sensors (`test/run_tests.sh`). The ESP32 sketch (`remote_esp32/remote_esp32.ino`) has **not been built for an ESP32**; only its parser
(`telemetry_parse.h`) is unit-tested on the host and the sketch is type-checked against host stubs (`tools/esp32_syntax_check.sh`).

## Architecture

```
 sensors, motors ── Arduino Mega 2560 ── Serial1 (D18/D19) ── HC-05 ~~~ Bluetooth Classic SPP (radio) ~~~ ESP32 (master) ── USB console / buttons
```

The ESP32 and the HC-05 talk **over the air**; there is no wire between them, so there is no ESP32 pin to level-shift. The only wired level problem is the Mega
(5 V) to the HC-05 (3.3 V logic), below. If you ever wire an ESP32 directly to the Mega for debugging, its pins are 3.3 V only and the Mega's 5 V TX needs the same
kind of divider. **A buck converter is a power supply, never a serial-data level shifter.**

## UART choice (checked against the motor wiring)

| UART | Pins | Used by | Free? |
|---|---|---|---|
| Serial (USB) | D0/D1 | USB console | in use |
| **Serial1** | **TX1 D18, RX1 D19** | **HC-05 (chosen)** | yes; no motor, sensor or I2C pin |
| Serial2 | TX2 D16, RX2 D17 | – | free, would also work |
| Serial3 | TX3 D14, RX3 D15 | – | free |
| I2C | SDA D20, SCL D21 | MPU6050 | in use |

Motors use D5, D6, D9, D10 (PWM) and D22–D29; receivers A0–A5; modules D34/D35; underside sensor D40. Serial1 was already implemented and tested, so it stays;
moving to Serial2 is a pin change in `pins.h` plus `Serial1`→`Serial2` in `bt.h`/`telemetry.h` and is not needed. SoftwareSerial on D10/D11 is **not** used (D10 is the back-left motor PWM).

## HC-05 wiring

| HC-05 pin | Connect to | Note |
|---|---|---|
| VCC | see below | **depends on your breakout board – read its marking** |
| GND | Mega GND | common ground |
| TXD | Mega RX1 (D19) | HC-05 TX is 3.3 V; the Mega reads it as high, no shifter needed |
| RXD | Mega TX1 (D18) through a divider | D18 → 1 kΩ → junction → HC-05 RXD; junction → 2 kΩ → GND (5 V × 2/3 ≈ 3.3 V) |
| EN/KEY | unconnected in normal use | held high at power-up = AT mode (38400 baud) |
| STATE | optional | high when paired; not used by the firmware |

**VCC (unverified for your board):** the bare HC-05 module is specified at 3.3 V logic with a supply of 3.6–6 V on the common 6-pin breakout (which carries a 3.3 V
regulator) and 3.3 V on boards without one. Check whether your breakout has a regulator chip and its printed supply range. Power it from a clean 5 V rail that is not
the motor rail; the HC-05 draws a few tens of mA with spikes while pairing. If the robot resets or the link drops when motors start, give the HC-05 its own filtered
supply. Do not assume 5 V is safe on a module without a regulator.

## Framing and commands

Line oriented. The ESP32 sends `<text>\n`. In the USB Serial Monitor set the line ending to **Newline** or **Both NL & CR** (115200 baud for USB; 9600 default for the HC-05 side).

* **One character = single-key command** (case-insensitive): `M` manual, `U` autonomous attack, `Q` stop, `W S A D Z C` drive (manual only), `X` emergency stop, `R` clear the latch, `P` ping, `?` status line.
* **Several characters = word command**, executed by the **same parser as the USB console** (`cliExecLine`): `defend`, `attack`, `manual`, `stop`, `status`, `sensors`, `gyro`, `calibrate`, `motors`, `tele`, `help`, `idle`.
* A line without a terminator is completed after 25 ms of silence on Bluetooth (8 ms on USB), so a lone `W` or `stop` still works.
* Lines longer than 24 characters (Bluetooth) or 39 (USB) are rejected whole: `ERR:TOO_LONG` / "input line too long". Unknown words are rejected whole (`ERR:UNKNOWN`) and **never executed as keys**.
* An `X` anywhere in the bytes received from Bluetooth, or `!` on USB, is an emergency stop at once; the rest of the waiting bytes are discarded. No word contains an `x`.
* Replies: `OK:<COMMAND>`, `ERR:NOT_MANUAL`, `ERR:ESTOP`, `ERR:REFUSED`, `ERR:FAULT`, `ERR:UNKNOWN`, `ERR:TOO_LONG`, `PONG:<mode>`, `ST:<mode>,ESTOP=,BALL=,COND=`.
* Nothing received can name a pin, a register or a setting: only the fixed word table is reachable, and every word goes through the same safety checks (emergency-stop latch, faults, calibration).

The USB console keeps its older forms too: single keys (`f b l r q e s a h i m d g t`, sent alone) and `:` commands (`:set`, `:get`, `:ring`, `:mod`, `:save`, …).
Typing a multi-letter burst that is not a known word (for example `ff`) now executes **nothing**, because it is no longer possible to tell it from a mistyped word.

## State machine and command-loss protection

IDLE (boot) → MANUAL (`M`/`manual`), ATTACK (`U`/`attack`) or DEFEND (`defend`); `X` from anywhere → FAULT (latched, `safety: ESTOP_LATCHED`); latch → IDLE only by `R` / `:recover`
(refused while a drivetrain fault is active). Each mode change first stops the previous mode and clears its motor command.

* Every movement command is a **lease** (`btlease`, 400 ms): the robot stops by itself when it lapses. The remote repeats a held button every 100 ms.
* No byte for `btlink` ms (3000) → link declared lost, a manual move stopped, logged "Bluetooth link LOST". The remote sends `P` every second.
* Attack and defend run on the robot; they **do not depend on the link**. Losing Bluetooth does not stop them (and does not start anything).
* Disable the radio side with `:set btenable 0`.

## Automatic start

After a clean boot the robot starts ATTACK mode by itself after `startdelay` (1.5 s) when `autostart` is 1 (default; chosen by the team). It never starts DEFEND by itself. To make the robot
always power up idle: `:set autostart 0`, `:save`. The start is cancelled by `X`, `!`, `stop`, `manual`, `defend` or a fault during the delay. Bluetooth is not required.

## Telemetry throughput (important at 9600 baud)

The HC-05 data rate is set by its own UART setting (factory 9600 baud; `AT+UART=` in AT mode) and must equal `btbaud100` on the Mega (`96` = 9600). The radio link itself is independent of this.
Frames are 200–300 bytes. Telemetry is limited to ~60 % of the UART capacity and a frame is only started when the transmit queue is empty:

| HC-05 baud | `btbaud100` | frames per second (typical) |
|---|---|---|
| 9600 | 96 | ~2 |
| 38400 | 384 | ~9 |
| 115200 | 1152 | up to `telehz` (max 20) |

So the requested 10–20 updates per second **cannot be delivered at 9600 baud**; raise the HC-05 baud first (`AT+UART=38400,0,0` or `115200,0,0` in AT mode on your module; the AT syntax is unverified for
your revision), then `:set btbaud100 384`, `:save`, reset. Allowed values: 96, 192, 384, 576, 1152. See `docs/TELEMETRY.md`.

## ESP32 compatibility

* Bluetooth **Classic with SPP** exists only on the **original ESP32** (ESP32-WROOM/WROVER/DevKitC). The ESP32-S2 has no Bluetooth; the **S3, C3, C6, H2 are BLE-only** and **cannot connect to an HC-05**.
  The sketch stops with `#error` when the core has no Classic SPP. BLE and Wi-Fi were deliberately not substituted.
* Core/library: Espressif `esp32` Arduino core with the built-in `BluetoothSerial` library; partition scheme "Huge APP". Not built here.
* Your exact ESP32 board variant has not been given: **unresolved question**.

## Pairing and connection

1. Put the HC-05 in AT mode (hold EN/KEY high while powering it; 38400 baud, usually `\r\n` line ends), set a unique name and PIN, role slave, e.g. `AT+NAME=VVS-CYBERBOTS`, `AT+PSWD="1234"`/`AT+PSWD=1234` (syntax varies between revisions: **unverified for yours**), `AT+ROLE=0`. Do not copy a MAC from anywhere: none is hard-coded.
2. Power the robot normally. The HC-05 LED blinks fast while unpaired, slowly (about every 2 s) when paired.
3. Flash the ESP32 sketch; open its USB console at 115200 baud. It prints its own Bluetooth MAC and then tries `bt.connect("VVS-CYBERBOTS")` every 3 s.
4. When it prints `connected`, type `status` (Newline) in the ESP32 console: `OK:STATUS` and a telemetry frame arrive; the dashboard prints once a second.
5. Typing `defend`, `attack`, `manual`, `stop`, … in the ESP32 console sends them to the robot. `X` or `!` sends the emergency stop immediately.

## Security (honest limits)

Bluetooth Classic with a legacy PIN is weak. Change the default PIN, use a unique name, and power the HC-05 only when needed. Anyone who pairs can send commands, including `defend` and `attack`.
The lease, the link timeout, the latched `X` and the fact that nothing but the fixed command words is reachable are the safety layer – not secrecy.
