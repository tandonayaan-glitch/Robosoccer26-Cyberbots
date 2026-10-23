# Bluetooth link: Windows laptop -> HC-05 -> Arduino Mega (commands + telemetry)

Status: **simulated and compiled only. Nothing here has been tested over the air or on the robot.** The Mega firmware is compiled for the ATmega2560 (`tools/avr_build.sh`) and
tested against simulated sensors (`test/run_tests.sh`); the Python laptop client (`laptop/`) is tested against the same firmware logic in simulation. Beginner steps: **`Connect.md`**.

## Architecture

```
 sensors, motors ── Arduino Mega 2560 ── Serial1 (D18/D19) ── HC-05 ~~~ Bluetooth Classic SPP ~~~ Windows laptop (laptop_remote.py on the Bluetooth COM port)
```

The laptop connects straight to the HC-05's Bluetooth serial service; the Mega keeps talking to the HC-05 through `Serial1`. There is no other controller in the chain.
The only wired level problem is the Mega (5 V) to the HC-05 (3.3 V logic), below. **A buck converter is a power supply, never a serial-data level shifter.**

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

> **Everything below except `X` (emergency stop) and `P` (ping) is refused with `ERR:AUTH` until the laptop has authenticated - see "Authentication" further down.**

Line oriented. The laptop client sends `<text>\n`. In the Mega's USB Serial Monitor set the line ending to **Newline** or **Both NL & CR** (115200 baud for USB; 9600 default for the HC-05 side).

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
Frames are about 200–380 bytes. Telemetry is limited to ~60 % of the UART capacity and a frame is only started when the transmit queue is empty:

| HC-05 baud | `btbaud100` | frames per second (typical) |
|---|---|---|
| 9600 | 96 | ~2 |
| 38400 | 384 | ~9 |
| 115200 | 1152 | up to `telehz` (max 20) |

So the requested 10–20 updates per second **cannot be delivered at 9600 baud**; raise the HC-05 baud first (`AT+UART=38400,0,0` or `115200,0,0` in AT mode on your module; the AT syntax is unverified for
your revision), then `:set btbaud100 384`, `:save`, reset. Allowed values: 96, 192, 384, 576, 1152. See `docs/TELEMETRY.md`.

## Pairing and connection (Windows)

1. Leave the HC-05 as it is if it already talks to the Mega. Do not change its name, PIN or baud rate unless you must (AT commands differ between HC-05 revisions; nothing in this repository changes them for you). Your pairing PIN is documented as `2114`; typical factory values are name `HC-05`, PIN `1234`, 9600 baud, slave role - **unverified for your module**, and the firmware never sets the PIN.
2. Power the robot. The HC-05 LED blinks fast while unpaired and slowly (about every 2 s) when connected.
3. Windows: Settings > Bluetooth & devices > Add device > Bluetooth > `HC-05`, enter the PIN. Pairing creates two virtual serial ports.
4. Find the right one: Settings > Bluetooth & devices > Devices > More Bluetooth options > **COM Ports**: use the **Outgoing** entry for the HC-05 (the Incoming one is for devices that connect to the laptop). `python laptop\laptop_remote.py --list-ports` lists them too.
5. `python laptop\laptop_remote.py --port COMx`, then type `connect` (the login is started by you), `help` for the commands. `x` is the emergency stop.

**Pairing is not authentication:** the PIN only decides who may open a Bluetooth link; the Mega still refuses every control command until the laptop has proved it knows the key.

## Authentication (application level) - what is new

**Pairing is not authentication.** The HC-05's name and PIN only decide who may open a Bluetooth link. Since this update the Mega refuses *every* control command on that link until the laptop has proven it knows a
shared 32-byte key. Beginner steps: **`Connect.md`**.

### Protocol (Mega: `btauth.h` + `sha256_hmac.h`; laptop: `laptop/robot_protocol.py` using Python's standard `hmac`/`hashlib`; the two are cross-checked in `laptop/tests/`)
```
laptop -> Mega  AUTH                                  request; ends any current session
Mega -> laptop  AUTH CH <sn: 32 hex>                  challenge: 16 bytes, unique, single use, valid 10 s
laptop -> Mega  AUTH RESP <cn: 32 hex> <tag: 32 hex>  cn = the laptop's own 16 random bytes (os.urandom); tag = HMAC-SHA256(key, "E2M"|sn|cn)[0..16]
Mega -> laptop  AUTH OK <tag2: 32 hex>                tag2 = HMAC-SHA256(key, "M2E"|cn|sn)[0..16]: the Mega proves it knows the key too
laptop -> Mega  AUTH OUT                              end the session
```
Replies on failure: `ERR:AUTH_FAIL`, `ERR:AUTH_EXPIRED`, `ERR:AUTH_NOCHAL` (a proof with no outstanding challenge: replay / out of order), `ERR:AUTH_LOCKOUT`, `ERR:AUTH_BUSY`, `ERR:AUTH_SYNTAX`, `ERR:NO_KEY`, and `AUTH OPEN` when enforcement is switched off.

* **Nothing secret crosses the air.** Only random challenges and truncated (128-bit) HMAC tags do. The key is never printed, logged, recorded, echoed or sent in telemetry; the host tests assert that neither the key nor the passcode appears on either serial line.
* **Replay protection.** A challenge is used once (consumed even by a wrong answer), expires after 10 s, and the proofs are bound to both nonces and to a direction label (no reflection). The Mega has no hardware RNG: its challenge is `HMAC(key, "NON" | boot counter | challenge counter | micros | millis)`. It is unpredictable without the key and **never repeats** because the boot counter is stored in the EEPROM and increases at every power-up (tested across a reboot). The laptop uses the operating system's secure generator (`os.urandom`) for its challenge.
* **Throttling.** Three failures in a row lock the handshake for 4 s, then 8, 16, 32, 60 s; at most one challenge per 300 ms. Wrong *format* counts as a failure.
* **Session end.** 2.5 s without a valid command (the laptop client pings every 500 ms), the link timeout (`btlink`, 3 s), a new `AUTH`, `AUTH OUT`, or any reset of the Mega. The laptop client notices a lost session (`ERR:AUTH`, `PONG:AUTH`, or 4 s of silence) and, if you are logged in, authenticates again by itself; `disconnect` ends it on purpose.
* **What an unauthenticated peer can still do:** `X` (emergency stop - always safe) and `P` (ping, answered `PONG:AUTH`). Everything else gets `ERR:AUTH` (rate limited), including `R` (so nobody can clear the emergency-stop latch), mode changes, calibration, settings, and the telemetry stream.
* **Emergency-stop latch.** Authentication never touches it: authenticating, failing or losing the session leaves `estop` as it was, and only an authenticated `R` clears it.
* **No key on the Mega** (fresh board, or `:authkey clear`): fail closed. The remote can only send `X` and `P` until a key is stored. Autonomous start-up (`autostart`) does not depend on Bluetooth and is unchanged.
* **Attack / defend do not need the link.** Losing the session or the link stops a manual move (as before) but does not stop or start autonomous play.

### Cryptography - limits stated plainly
* The Arduino AVR core ships no cryptography library, so `sha256_hmac.h` is a small standard SHA-256 / HMAC-SHA-256 implementation, verified against published test vectors and an independent Python implementation. It has not been reviewed by a cryptographer, and a software implementation on an 8-bit AVR can leak timing; the tag comparison is constant time. The laptop side uses Python's standard library, an independent implementation, so the two are tested against each other (200 random cases) as well as against published vectors.
* **Only the handshake is authenticated.** After it, commands carry no per-message MAC and are not encrypted by this layer. Someone who can *inject* bytes into the already-established Bluetooth link (needs the Bluetooth link key, i.e. the HC-05 pairing) or who hijacks it within the 2.5 s window after the real remote disappears could still send commands. Bluetooth's own link encryption is the only protection for the data after the handshake. If you want per-command MACs, that is a protocol change (a counter plus a tag on every line) and would break plain key presses.
* **Anyone who knows the key can authenticate**, from any device. The key sits unencrypted in the Mega's EEPROM and in the laptop's key file; someone with physical access to a board can read it. The Mega's USB console is trusted: whoever can plug into the Mega can set `:authkey` or `:auth off`.
* The emergency stop and ping are open to everyone in Bluetooth range by design.

### Credentials
One secret: the **shared key** (32 random bytes = 64 hex digits). It lives in the Mega's EEPROM (set over USB with `:authkey <64 hex>`) and in a private file on each laptop (Windows: `%LOCALAPPDATA%\Robosoccer26\robot.key`). It is never typed on the robot, never sent over Bluetooth, never printed or logged.

| action | command |
|---|---|
| create + store on the Mega (first time) | `python tools\provision.py --mega-port COM3` (COM3 = the Mega's USB port) |
| rotate (old key stops working at once) | `python tools\provision.py --mega-port COM3 --rotate`, then give the new key to every other laptop |
| replace a Mega with the saved key | `python tools\provision.py --mega-port COM3 --reuse` |
| erase on the Mega | `python tools\provision.py --mega-port COM3 --clear` (or `:authkey clear` on its USB console) |
| second laptop | copy `robot.key`, or `python laptop\laptop_remote.py --import-key` (hidden paste) |
| compare keys | the laptop prints a fingerprint at start-up; the Mega prints one for `:authkey` |

No flashing is needed to change the key. The Mega must be flashed once with this firmware. Opening the Mega's USB port resets it and it may start its automatic attack, so lift the wheels or disconnect the motor battery while provisioning.
**The key proves knowledge of the key, not which laptop is connected.** Anyone who copies `robot.key` can control the robot. EEPROM and the key file are not encrypted; the Mega's USB port is trusted.

### Laptop client (`laptop/laptop_remote.py`)
Opens the Bluetooth COM port, waits for **you** to type `connect`, then authenticates; only supported commands can be sent (`set`, `:authkey`, raw AUTH lines are refused locally); `x` sends the emergency stop at any time, even before login; `quit` stops the robot (without latching the emergency stop) and ends the session; closing the program unexpectedly or Ctrl+C requests an emergency stop. Needs only `pyserial`.

### HC-05 pairing PIN
Optional extra hurdle, **not** a replacement for the above. Only change it if your breakout accepts AT commands (hold EN/KEY high while powering it on; many 6-pin HC-05 boards enter AT mode at 38400 baud with `\r\n` endings, some revisions differ - check yours). Typical commands: `AT+NAME=<name>`, `AT+PSWD=<pin>` (some revisions want `AT+PSWD="<pin>"`), `AT+ROLE=0`, `AT+UART=<baud>,0,0`. These are **unverified for your module**; nothing in this repository changes the HC-05 for you and the firmware works with any PIN. Never connect the Mega's 5 V TX1 (D18) straight to the HC-05 RXD: use the 1 kΩ / 2 kΩ divider.
