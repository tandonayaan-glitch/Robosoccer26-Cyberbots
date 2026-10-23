# ESP32 remote - quick start (Arduino IDE), daily use, other laptops, recovery

Status: **host-simulated only.** The firmware logic and the protocol are tested on a PC; the ESP32 sketch has never been built for an ESP32, and nothing has been tested over Bluetooth with the real HC-05.

## How it fits together
```
 laptop --USB--> ESP32 (your sketch, remote_esp32.ino) ~~~ Bluetooth ~~~> HC-05 --wires--> Arduino Mega (robot)
```
The ESP32 stays plugged into the laptop with its program already inside it. You talk to it in the Arduino IDE's **Serial Monitor** (the ESP32's port, 115200 baud, line ending **Newline**). The ESP32 finds the HC-05 by name over the air by itself; it is not wired to the Mega. Daily use is **one command**: `esp32connect <passcode>`.

## One-time setup (about 20 minutes)
1. **Arduino IDE 2.x.** File > Preferences > "Additional boards manager URLs": `https://espressif.github.io/arduino-esp32/package_esp32_index.json`. Tools > Board > Boards Manager > install **esp32 by Espressif Systems**. (Use an *original ESP32* board: ESP32-S3/C3/C6 have no Bluetooth Classic and cannot talk to the HC-05.)
2. **Flash the Mega with this firmware first** (it contains the authentication): plug the Mega in, open `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino`, Tools > Board > "Arduino Mega or Mega 2560", pick the Port, Upload. Wheels lifted or motor battery disconnected.
3. **Make the credentials - one command** (Python 3 and `python3 -m pip install pyserial`; use the Mega's port from Tools > Port, e.g. `COM3`, `/dev/ttyACM0`, `/dev/cu.usbmodem1101`):
   `python3 tools/provision.py --mega-port COM3`
   It writes `remote_esp32/secrets.h` (never committed), stores the key in the Mega, checks the fingerprint, and prints your **passcode** once. Write it down somewhere safe. (Without `--mega-port` it only writes the file; `--print-mega-command` shows the `:authkey ...` line to type into the Mega's Serial Monitor yourself.)
4. **Name the HC-05** `VVS-CYBERBOTS` (or change `ROBOT_BT_NAME` in the sketch to match), pair it once if your OS asks, see `docs/BLUETOOTH.md`. The Mega's side uses Serial1 (D18/D19) with the divider on D18.
5. **Flash the ESP32:** open `remote_esp32/remote_esp32.ino` (the folder must contain `secrets.h` and the other `.h` files). Tools > Board > ESP32 Arduino > **ESP32 Dev Module**, Tools > Partition Scheme > **Huge APP**, Port = the ESP32's port, Upload (hold the BOOT button while "Connecting..." if it does not start).

## Every day
1. Power the robot. Plug the ESP32 into the laptop. Arduino IDE > Tools > Port = the ESP32 > Serial Monitor, **115200 baud, "Newline"**.
2. It starts **locked** (`console LOCKED`). Type: `esp32connect <passcode>` (or just `esp32connect`, then the passcode on the next line).
3. It prints `console UNLOCKED`, `bluetooth connected`, `robot session AUTHENTICATED`. Now type commands: `help` lists all of them, `help ir` explains one.
4. When finished: `esp32disconnect`. The robot forgets the session at once, the console locks.

### Seeing the sensors (all need `esp32connect` first)
| type | shows |
|---|---|
| `ir` | the 6 receivers A0-A5 (level, swing, filtered, noise floor, threshold), obstacle modules D34/D35, underside sensor D40 |
| `imu` (or `gyro`) | accelerometer, gyro, roll/pitch/yaw estimates, gyro bias, drift test |
| `motors` | motor command PWM / direction, moving time, watchdog |
| `ball` | bearing, confidence, ambiguity |
| `safety` (or `status`) | mode, faults, emergency-stop latch, timing, telemetry counters |
| `raw` | the last raw telemetry line of each kind |
| `back` (or `all`) | the normal full view |
Commands that act on the robot: `defend`, `attack`, `manual`, `stop`, `calibrate`, `gyro drift 30`, `tele ...` and the keys `M U Q W S A D Z C R P ?`. **`X` or `!` is the emergency stop and works even when locked.**

## Using a different laptop
Nothing to reflash and nothing to change on the robot. Unplug the ESP32 from laptop A (it loses power and boots **locked**), plug it into laptop B, open the Serial Monitor, type `esp32connect <passcode>`. If the ESP32 is powered from a battery rather than USB it stays unlocked when the cable moves - type `esp32disconnect` before swapping, or set `CONSOLE_IDLE_LOCK_MS` in `secrets.h` (e.g. `300000UL`). Remember: the passcode does not identify a laptop; anyone who knows it can unlock the remote from any computer.

## Changing or recovering credentials
* **New passcode only:** `python3 tools/provision.py --new-passcode`, then reflash the ESP32.
* **New key and passcode:** `python3 tools/provision.py --force --mega-port <port>`, then reflash the ESP32.
* **Replaced the Mega / EEPROM lost:** `python3 tools/provision.py --reuse --mega-port <port>` (sends the key already in `secrets.h`; no ESP32 reflash).
* **Lost `secrets.h` / forgot the passcode:** generate new ones with `--force` and reflash. (Without the file there is no way to recover the old key; that is intended.)
* **Remove the key from the robot:** `python3 tools/provision.py --clear --mega-port <port>` or `:authkey clear` in the Mega's Serial Monitor. The remote can then only send `X`.
* **Fingerprints must match:** the ESP32 prints `key fingerprint xxxxxxxx` at start-up; the Mega prints it for `:authkey` (no argument). Different = different keys.
* **`ERR:AUTH_LOCKOUT` / wrong passcode throttling:** wait (up to 60 s); the counters reset after a correct answer or a reboot.
* **Robot says `AUTH OPEN`:** someone typed `:auth off` on the Mega's USB console (bench use). The ESP32 will not treat the robot as authenticated; type `:auth on` or reset the Mega.
* **Bench use without the key:** on the Mega's USB console `:auth off` (lasts until the next reset; any paired device can then control the robot).

## Physical test procedure (none of this has been done yet)
Wheels **lifted** (robot on a stand) **and the motor battery disconnected or its switch off** for steps 1-6; reconnect the motor battery only for step 7, wheels still lifted.
1. Flash, provision, fingerprints equal on both boards. 2. Power cycle the ESP32: `console LOCKED`; type `W`, `defend`, `ir`: nothing is sent ("LOCKED: nothing was sent"). 3. `esp32connect wrong-wrong-wrong`: refused, delay; the right one works. 4. After unlocking: `ir`, `imu`, `motors` show data; `back` returns.
5. Unplug the HC-05's power or walk out of range: the ESP32 prints `bluetooth link lost`, the robot session ends; return: re-authenticates by itself. 6. From a phone or laptop with a Bluetooth terminal (paired with the HC-05 *without* the key): `M`, `W`, `defend` must answer `ERR:AUTH`; `P` answers `PONG:AUTH`; `X` stops the robot.
7. Motor battery on, wheels lifted: `manual`, `W`: wheels turn, release: stop within ~0.4 s; `esp32disconnect` mid-move: stops, further keys do nothing; `X` from a locked console stops everything and the latch needs `R` after `esp32connect`. 8. Only then trial on the floor.
