# Connect the laptop directly to the robot (no ESP32)

```
Windows laptop ~~~ Bluetooth ~~~> HC-05 --wires--> Arduino Mega (robot)
```

**Status:** written and tested in PC simulation only. It has not been tried on the real Mega, HC-05 or a Windows laptop yet. Do the safe test at the end before driving.

The HC-05 stays wired to the Mega exactly as before (Mega TX1 D18 → 1 kΩ/2 kΩ divider → HC-05 RXD, HC-05 TXD → Mega RX1 D19, common ground). Nothing on the robot changes.

## One-time setup (Windows, PowerShell)

Wheels lifted and motor battery disconnected for all of this.

1. **Flash the Mega** with this version: Arduino IDE → open `Robosoccer26_Cyberbots\Robosoccer26_Cyberbots.ino` → board "Arduino Mega or Mega 2560" → Upload.
2. **Install Python 3** from python.org (tick "Add python.exe to PATH"), then in PowerShell, in the repository folder:
   ```
   python -m pip install -r laptop\requirements.txt
   ```
3. **Pair Windows with the HC-05:** Settings → Bluetooth & devices → Add device → Bluetooth → pick `HC-05` (or the name you gave it). Enter the PIN `2114` (the HC-05 must actually be set to it; the factory value is usually `1234`). Pairing only opens a Bluetooth link; it does **not** let anyone drive the robot.
4. **Find the Bluetooth COM port:** Settings → Bluetooth & devices → Devices → More Bluetooth options → **COM Ports** tab → the entry whose direction is **Outgoing** for the HC-05 (for example `COM6`). Or run:
   ```
   python laptop\laptop_remote.py --list-ports
   ```
5. **Make the key and store it in the Mega** (the Mega's *USB* port, shown in Arduino IDE → Tools → Port, e.g. `COM3`):
   ```
   python tools\provision.py --mega-port COM3
   ```
   It creates a random key, saves it privately on this laptop, sends it to the Mega over USB, and checks the fingerprint. The key never goes over Bluetooth and is not printed.

## Every day

```
python laptop\laptop_remote.py --port COM6
```
(use **your** Bluetooth port). It authenticates by itself and then shows a `robot>` prompt. Type `help` for everything.

| Type | What it does |
|---|---|
| `defend` / `attack` / `manual` | robot modes |
| `w s a d z c` | in manual mode: forward, back, turn left/right, strafe left/right (0.4 s each) |
| `drive` | hold-to-drive with the keyboard (SPACE stops, `q` leaves, `x` emergency stop) |
| `stop` | stop the motors and leave the mode |
| `ir` · `imu` · `motors` · `ball` · `safety` · `raw` | show only that part of the robot's sensor data |
| `back` | return to the normal full view |
| `link` | connection and session state |
| **`x`** | **EMERGENCY STOP** (works even when not logged in; then type `r` after login to clear it) |
| `quit` | stop the robot, end the session, close the port |

If the program is closed unexpectedly (Ctrl+C, window closed) it asks the robot for an emergency stop.

## Another laptop

1. Install Python and the packages (step 2), pair the HC-05 (steps 3–4).
2. Get the key onto the new laptop: copy the file `%LOCALAPPDATA%\Robosoccer26\robot.key` from the first laptop to the same place on the new one (USB stick), **or** run `python tools\provision.py --reuse --show-key` on the first laptop and, on the new one, `python laptop\laptop_remote.py --import-key` and paste the 64 digits (hidden input).
3. Run `python laptop\laptop_remote.py --port COM6` (its own port number).

Anyone who has the key file can control the robot. The key proves that someone *knows the key*; it does not identify a particular laptop.

## Change or remove the key

| Goal | Command |
|---|---|
| New key (old one stops working at once; other laptops need the new key) | `python tools\provision.py --mega-port COM3 --rotate` |
| Re-send the saved key to a replaced Mega | `python tools\provision.py --mega-port COM3 --reuse` |
| Erase the key on the robot (nobody can control it by Bluetooth) | `python tools\provision.py --mega-port COM3 --clear` |

## If something goes wrong

| Problem | Try |
|---|---|
| `cannot open COM6` | wrong port, HC-05 not powered/paired, or another program has it open: `--list-ports` |
| Stays "waiting for challenge" | HC-05 not connected to the Mega's Serial1 / wrong COM port (use the *Outgoing* one) |
| "authentication refused" | the laptop's key differs from the Mega's: compare the fingerprints (laptop prints one at start; the Mega prints one for `:authkey`), then provision again |
| "throttling attempts" | wait up to a minute; a correct key resets it |
| "no key stored" on the laptop | `python laptop\laptop_remote.py --import-key` (or copy `robot.key`) |
| Robot ignores commands | type `link`; log in again; if the emergency stop is latched type `r` |
| Link drops | the program logs in again by itself; the robot stops a manual move within 0.4 s |

## Safe first test (nothing here has been done on real hardware yet)

Wheels lifted and motor battery disconnected until step 5.

1. Pair and find the COM port; run the program: it must reach **AUTHENTICATED**.
2. Stop the program, then connect from a phone/laptop Bluetooth terminal **without** the key: `M`, `W`, `defend` must answer `ERR:AUTH`, and `X` must still stop the robot.
3. Type `ir`, `imu`, `motors`: data appears. `back` returns.
4. Switch the HC-05 off or walk out of range: the program reports the link lost; switch it on again: it logs in again by itself.
5. Connect the motor battery (wheels still lifted): `manual`, `w`: wheels turn for 0.4 s; `drive` and release the key: they stop; `x`: everything stops; `r` after login clears the latch.
6. Only then try on the floor.

## Limits

* The SHA-256/HMAC code on the Mega is custom and has had no independent cryptographic review. This is protection against casual or accidental access, not a guarantee.
* After login, commands are not individually authenticated or encrypted by this layer; only Bluetooth's own link encryption covers them.
* The key is stored unencrypted in the Mega's EEPROM and in the laptop's key file; the Mega's USB port is trusted.
* Never connect the Mega's 5 V TX pin (D18) straight to the HC-05 RXD; keep the divider. See `docs/BLUETOOTH.md`.
