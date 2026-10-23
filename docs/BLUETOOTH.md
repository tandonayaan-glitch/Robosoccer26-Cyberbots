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

> **Everything below except `X` (emergency stop) and `P` (ping) is refused with `ERR:AUTH` until the ESP32 has authenticated - see "Authentication" further down.**

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
4. Type `esp32connect <passcode>` in the ESP32 console (after `docs/ESP32_QUICKSTART.md` provisioning). It prints `bluetooth connected` and `robot session AUTHENTICATED`; the dashboard then prints once a second (`ir`, `imu`, `motors`, `ball`, `safety`, `back` choose what is shown).
5. Typing `defend`, `attack`, `manual`, `stop`, … sends them to the robot. `X` or `!` sends the emergency stop immediately, even before `esp32connect`.

## Authentication (application level) - what is new

**Pairing is not authentication.** The HC-05's name and PIN only decide who may open a Bluetooth link. Since this update the Mega refuses *every* control command on that link until the ESP32 has proven it knows a
shared 32-byte key. Quick start with the Arduino IDE steps: **`docs/ESP32_QUICKSTART.md`**.

### Protocol (Mega: `btauth.h`; ESP32: `remote_esp32/auth_client.h`; both use `sha256_hmac.h`, tested against the NIST / RFC 4231 vectors)
```
ESP32 -> Mega   AUTH                                  request; ends any current session
Mega  -> ESP32  AUTH CH <sn: 32 hex>                  challenge: 16 bytes, unique, single use, valid 10 s
ESP32 -> Mega   AUTH RESP <cn: 32 hex> <tag: 32 hex>  cn = the ESP32's own 16-byte random challenge; tag = HMAC-SHA256(key, "E2M"|sn|cn)[0..16]
Mega  -> ESP32  AUTH OK <tag2: 32 hex>                tag2 = HMAC-SHA256(key, "M2E"|cn|sn)[0..16]: the Mega proves it knows the key too
ESP32 -> Mega   AUTH OUT                              end the session
```
Replies on failure: `ERR:AUTH_FAIL`, `ERR:AUTH_EXPIRED`, `ERR:AUTH_NOCHAL` (a proof with no outstanding challenge: replay / out of order), `ERR:AUTH_LOCKOUT`, `ERR:AUTH_BUSY`, `ERR:AUTH_SYNTAX`, `ERR:NO_KEY`, and `AUTH OPEN` when enforcement is switched off.

* **Nothing secret crosses the air.** Only random challenges and truncated (128-bit) HMAC tags do. The key is never printed, logged, recorded, echoed or sent in telemetry; the host tests assert that neither the key nor the passcode appears on either serial line.
* **Replay protection.** A challenge is used once (consumed even by a wrong answer), expires after 10 s, and the proofs are bound to both nonces and to a direction label (no reflection). The Mega has no hardware RNG: its challenge is `HMAC(key, "NON" | boot counter | challenge counter | micros | millis)`. It is unpredictable without the key and **never repeats** because the boot counter is stored in the EEPROM and increases at every power-up (tested across a reboot). The ESP32 uses its hardware RNG (`esp_fill_random`) for its challenge.
* **Throttling.** Three failures in a row lock the handshake for 4 s, then 8, 16, 32, 60 s; at most one challenge per 300 ms. Wrong *format* counts as a failure.
* **Session end.** 2.5 s without a valid command (the ESP32 pings every 500 ms), the link timeout (`btlink`, 3 s), a new `AUTH`, `AUTH OUT`, or any reset of the Mega. The ESP32 forgets its side on a Bluetooth disconnect and re-handshakes by itself.
* **What an unauthenticated peer can still do:** `X` (emergency stop - always safe) and `P` (ping, answered `PONG:AUTH`). Everything else gets `ERR:AUTH` (rate limited), including `R` (so nobody can clear the emergency-stop latch), mode changes, calibration, settings, and the telemetry stream.
* **Emergency-stop latch.** Authentication never touches it: authenticating, failing or losing the session leaves `estop` as it was, and only an authenticated `R` clears it.
* **No key on the Mega** (fresh board, or `:authkey clear`): fail closed. The remote can only send `X` and `P` until a key is stored. Autonomous start-up (`autostart`) does not depend on Bluetooth and is unchanged.
* **Attack / defend do not need the link.** Losing the session or the link stops a manual move (as before) but does not stop or start autonomous play.

### Cryptography - limits stated plainly
* The Arduino AVR core ships no cryptography library, so `sha256_hmac.h` is a small standard SHA-256 / HMAC-SHA-256 implementation, verified against published test vectors and an independent Python implementation. It has not been reviewed by a cryptographer, and a software implementation on an 8-bit AVR can leak timing; the tag comparison is constant time. On the ESP32 the same file is used (so both sides are tested against each other) although `mbedtls` is available in its core.
* **Only the handshake is authenticated.** After it, commands carry no per-message MAC and are not encrypted by this layer. Someone who can *inject* bytes into the already-established Bluetooth link (needs the Bluetooth link key, i.e. the HC-05 pairing) or who hijacks it within the 2.5 s window after the real remote disappears could still send commands. Bluetooth's own link encryption is the only protection for the data after the handshake. If you want per-command MACs, that is a protocol change (a counter plus a tag on every line) and would break plain key presses.
* **Anyone who knows the key can authenticate**, from any device. The key sits unencrypted in the Mega's EEPROM and in the ESP32's flash (`secrets.h`); someone with physical access to a board can read it. The Mega's USB console is trusted: whoever can plug into the Mega can set `:authkey` or `:auth off`.
* The emergency stop and ping are open to everyone in Bluetooth range by design.

### Credentials
| item | where it lives | used for |
|---|---|---|
| **SHARED_KEY_HEX** (64 hex = 32 random bytes) | ESP32 `remote_esp32/secrets.h` (git-ignored) and the Mega's EEPROM (`:authkey`) | ESP32 <-> Mega handshake. Never typed by the operator. |
| **CONSOLE_PASSCODE** (>= 8 characters) | ESP32 `secrets.h` only | `esp32connect <passcode>` in the ESP32's USB Serial Monitor |

`python3 tools/provision.py --mega-port <port>` makes both, writes `secrets.h`, and stores the key in the Mega. Changing the key or passcode = rerun with `--force` / `--new-passcode`, then **reflash the ESP32**; the Mega only needs `:authkey` (no reflash). **The Mega must be flashed once with this firmware** (it contains the authentication).
`secrets.h.example` contains placeholders only; the ESP32 sketch and the client refuse the placeholders ("NOT PROVISIONED"). Recovery and moving to another laptop: see `docs/ESP32_QUICKSTART.md`.

### ESP32 USB console lock
Locked at every power-up. `esp32connect <passcode>` (or `esp32connect` and then the passcode on the next line) unlocks it, connects Bluetooth and authenticates to the robot; it stays until `esp32disconnect` (or `LOCK`), or the ESP32 restarts, or the optional idle lock (`CONSOLE_IDLE_LOCK_MS`) fires. Locked, nothing is forwarded, the buttons do nothing, and only `X` / `!` (emergency stop), `help` and `esp32status` work. Wrong passcodes are throttled (1 s, 1 s, 5 s, 15 s, 30 s, 60 s) and a wrong attempt also re-locks. An `esp32connect ...` line (and the passcode line after a prompt) is consumed by the lock and is never interpreted as a movement command.
**The passcode does not identify a laptop.** It proves that whoever typed it knows it. Any laptop plugged into the ESP32's USB port can use the remote after `esp32connect <passcode>`; anyone who learns the passcode can do the same. Typing it in a Serial Monitor shows it on screen and may keep it in the terminal's history.

### HC-05 pairing PIN
Optional extra hurdle, **not** a replacement for the above. Only change it if your breakout accepts AT commands (hold EN/KEY high while powering it on; many 6-pin HC-05 boards enter AT mode at 38400 baud with `\r\n` endings, some revisions differ - check yours). Typical commands: `AT+NAME=<name>`, `AT+PSWD=<pin>` (some revisions want `AT+PSWD="<pin>"`), `AT+ROLE=0`, `AT+UART=<baud>,0,0`. These are **unverified for your module**; nothing in this repository changes the HC-05 for you and the firmware works with any PIN. Never connect the Mega's 5 V TX1 (D18) straight to the HC-05 RXD: use the 1 kΩ / 2 kΩ divider.
