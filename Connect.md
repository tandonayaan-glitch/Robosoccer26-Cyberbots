# Connect the laptop to the robot

**Status:** the code passes the PC simulation tests only. Nothing has been tested yet on the real Mega, ESP32 or HC-05, and the ESP32 sketch has never been compiled for an ESP32. Do the hardware test at the end before you drive.

```
laptop --USB--> ESP32 ~~~ Bluetooth ~~~> HC-05 --wires--> Arduino Mega (robot)
```

## One-time setup

Wheels lifted and motor battery disconnected for all of this.

1. **Flash the Mega.** Arduino IDE → open `Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino` → board "Arduino Mega or Mega 2560" → Upload.
2. **Make the secrets.** In the repository folder run (needs Python 3 and `python3 -m pip install pyserial`):
   ```
   python3 tools/provision.py --mega-port <port>
   ```
   `<port>` is the Mega's port in Arduino IDE → Tools → Port (`COM3`, `/dev/ttyACM0`, `/dev/cu.usbmodem…`). It writes `remote_esp32/secrets.h` (never committed), stores the key in the Mega, and prints your **passcode once**. Write it down and keep it private.
3. **Flash the ESP32.** Install "esp32 by Espressif Systems" in the Boards Manager → open `remote_esp32/remote_esp32.ino` (`secrets.h` must be in the same folder) → board "ESP32 Dev Module" → Partition Scheme "Huge APP" → Upload. It must be an original ESP32, not an S3/C3/C6.

## Every day

1. Power the robot. Plug the ESP32 into the laptop.
2. Arduino IDE → Tools → Port = the ESP32 → Serial Monitor → **115200 baud, line ending "Newline"**.
3. Type:
   ```
   esp32connect YOUR_PASSCODE
   ```
4. Use the robot. When finished: `esp32disconnect`.

| Type | What it does |
|---|---|
| `help` / `help ir` | list every command / explain one (works even when locked) |
| `esp32status` | state of this remote: locked, Bluetooth, robot session |
| `ir` | infrared receivers A0–A5, obstacle modules, underside sensor |
| `imu` | accelerometer, gyro, roll/pitch/yaw estimates |
| `motors` | motor command PWM and directions |
| `ball` / `safety` / `raw` | ball bearing / mode and faults / raw telemetry lines |
| `back` | return to the normal full view |
| `defend`, `attack`, `manual`, `stop`, `calibrate` | robot modes (after `manual`: `W S A D Z C` drive) |
| `X` or `!` | **emergency stop** (works even when locked) |
| `esp32disconnect` | end the session and lock |

## Another laptop

Nothing to reflash. Unplug the ESP32 from the first laptop, plug it into the second, open the Serial Monitor (115200, Newline) and type `esp32connect YOUR_PASSCODE`.

* The ESP32 starts **locked** after every power-up. If it runs from a battery instead of USB, type `esp32disconnect` before swapping.
* The passcode does not identify a laptop: anyone who knows it can connect from any computer.

## If something goes wrong

| Problem | Fix |
|---|---|
| Forgot the passcode or lost `secrets.h` | `python3 tools/provision.py --force --mega-port <port>`, then reflash the ESP32 |
| New Mega, or its key was erased | `python3 tools/provision.py --reuse --mega-port <port>` (no ESP32 reflash) |
| Want a new passcode only | `python3 tools/provision.py --new-passcode`, then reflash the ESP32 |
| "wrong passcode" delays | wait up to 60 s; a correct passcode resets it |
| Robot answers `ERR:AUTH` | not authenticated: type `esp32connect …` again |
| Key fingerprints differ | the ESP32 prints one at start-up; the Mega prints its own for `:authkey`. Re-run provisioning |
| Robot says `AUTH OPEN` | someone typed `:auth off` on the Mega's USB console; type `:auth on` or reset the Mega |

## Hardware test before driving

Full list: `docs/ESP32_QUICKSTART.md`. The essentials, wheels lifted and motor battery disconnected until the last step:

1. Right after power-up the ESP32 says **locked** and sends nothing when you type `defend`.
2. A wrong passcode is refused; the right one connects.
3. A phone or laptop Bluetooth terminal paired with the HC-05 but without the key gets `ERR:AUTH` for `M`, `W` and `defend`. `X` still stops the robot.
4. Switch the HC-05 off or walk away: the ESP32 reports the link lost and reconnects and re-authenticates by itself.
5. Only then connect the motor battery (wheels still lifted): `manual`, `W` turns the wheels, releasing stops them within about 0.4 s, `X` stops everything.

## Cautions

* The custom SHA-256/HMAC code has had no independent cryptographic review. Treat this as protection against casual or accidental access, not as a guarantee.
* Commands after the handshake are not individually authenticated or encrypted by this layer.
* The key is stored unencrypted in the Mega's EEPROM and the ESP32's flash, and the Mega's USB port is trusted.
* Never connect the Mega's 5 V TX pin (D18) directly to the HC-05 RXD; use the 1 kΩ / 2 kΩ divider from `docs/BLUETOOTH.md`.
* Never commit `remote_esp32/secrets.h`.
