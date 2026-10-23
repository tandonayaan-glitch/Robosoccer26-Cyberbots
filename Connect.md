# Connect

This is a substantial improvement to your RoboSoccer bot's security. The software tests have passed, but the authentication has not yet been verified on your actual Mega, ESP32, and HC-05. Here's what you should do next.

## What you need to do

### 1. Flash the Arduino Mega first

Open the updated Mega firmware from branch `claude/sweet-knuth-hpyd7i` in Arduino IDE, select Arduino Mega or Mega 2560, and upload it. Keep the robot's wheels lifted and disconnect motor power during initial setup.

### 2. Provision the secret

On your laptop, run the provisioning command from the repository root:

```
python3 tools/provision.py --mega-port <port>
```

Replace `<port>` with the Mega's actual serial port. This step writes the credential to the Mega and creates `remote_esp32/secrets.h`. Keep the generated passcode private.

### 3. Build and flash the ESP32

Install the Espressif ESP32 board package, select ESP32 Dev Module, and use the partition scheme specified in `docs/ESP32_QUICKSTART.md`. Compile and upload the updated `remote_esp32/remote_esp32.ino`.

The ESP32 must have the generated `secrets.h` available when compiling.

### 4. Connect from your laptop

Plug the ESP32 into USB, open its Serial Monitor at 115200 baud with Newline line endings, and enter:

```
esp32connect YOUR_PASSCODE
```

Replace `YOUR_PASSCODE` with the generated passcode. Use `esp32status` to inspect the connection state, and `esp32disconnect` when finished.

### 5. Test before driving

Follow all eight hardware-test steps in `docs/ESP32_QUICKSTART.md`. Verify that an unauthenticated connection cannot control the robot, the emergency stop works, and reconnection requires authentication. Only test motor movement with the wheels lifted first.

## Can you use another laptop?

Yes. Once the firmware and credentials are configured, you should be able to move the ESP32's USB connection to another laptop and enter the same passcode. You do not normally need to reflash either board just to change laptops.

* The second laptop needs a serial terminal or Arduino IDE Serial Monitor.
* The ESP32 connects wirelessly to the HC-05; the laptop does not need to pair directly with the HC-05.
* The ESP32 starts locked after reboot, so enter the passcode again.
* Anyone who obtains the passcode can authenticate, so don't share it publicly.

## Important cautions

* Don't push to `Robocup-2026-Ayaan` yet. The changes are on `claude/sweet-knuth-hpyd7i`, and the real hardware still needs testing.
* Keep the motor battery disconnected while provisioning and checking the authentication protocol.
* Do not assume the ESP32 sketch compiles successfully on real hardware just because the simulation tests passed.
* The custom SHA-256/HMAC implementation has not had independent cryptographic review. Treat this as protection against casual access, not a guarantee of security.

The most useful next step is to open `docs/ESP32_QUICKSTART.md` and follow it in order.
