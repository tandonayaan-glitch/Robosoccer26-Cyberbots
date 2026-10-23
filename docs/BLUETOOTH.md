# Bluetooth manual control (HC-05 on the Mega, ESP32 remote)

Status: **simulated and compiled only. Not tested on the robot.** The ESP32 sketch (`remote_esp32/remote_esp32.ino`) has **not been compiled** (no ESP32 toolchain was available).

## Wiring (HC-05 on Serial1)

| HC-05 | Mega 2560 | Note |
|---|---|---|
| VCC | 5 V (or the module's own regulator input) | confirm your breakout's supply range; measure under motor load |
| GND | GND | common ground with the Mega |
| TXD | RX1 = D19 | HC-05 TX is 3.3 V: the Mega reads it as high, no shifter needed |
| RXD | TX1 = D18 **through a divider** | Mega TX1 (5 V) -> 1 kΩ -> junction -> HC-05 RXD; junction -> 2 kΩ -> GND (about 3.3 V) |

D18/D19 are free in the current pin map (motors use D22-D29 and PWM D5/D6/D9/D10, I2C is D20/D21). SoftwareSerial on D10/D11 is **not** used: D10 is the back-left motor PWM. Baud 9600 (`BT_BAUD` in `pins.h`); change both ends if your module is configured differently.

## Protocol (single characters, case-insensitive)

| Byte | Action |
|---|---|
| `M` | manual mode (ends autonomy, robot stands still) |
| `U` | autonomous mode (same checks as the `a` key; refused with an unhealthy sensor or fault) |
| `W` `S` `A` `D` | forward, backward, turn left, turn right (manual mode only) |
| `Z` `C` | strafe left / right (manual mode only) |
| `Q` | normal stop: motion and autonomy end, manual mode stays |
| `X` | **emergency stop**: stops at once, latches; no movement or autonomy until reset |
| `R` | clear the latch (refused while a drivetrain fault is active); the robot stays idle until `M` or `U` |
| `P` / `?` | ping -> `PONG:<mode>` / status line `ST:<mode>,ESTOP=,BALL=,COND=` |

Replies: `OK:<cmd>` or `ERR:<reason>` (`NOT_MANUAL`, `ESTOP`, `FAULT`, `UNKNOWN`). `A` is always "turn left"; autonomy is `U`, so the two never collide.

## State machine

IDLE (boot) -> MANUAL (`M`) or AUTO (`U`); `X` from anywhere -> ESTOP (latched); ESTOP -> IDLE only by `R` (or `:recover` on USB). The USB console (`!`, `:estop`) uses the same latch.

## Command-loss protection

- Each movement command is a **lease** (`btlease`, 400 ms): the robot stops by itself when it expires. The remote repeats a held button every 100 ms.
- No byte for `btlink` ms (3000) -> link declared lost, motion stopped (logged "Bluetooth link LOST"). The remote sends `P` every second to keep the link alive.
- `X` anywhere in the bytes waiting is handled first and the rest are dropped.
- Unknown bytes never move anything.
- Disable with `:set btenable 0`.

## Pairing and security (honest limits)

- The ESP32 is the master and connects **by name** (`VVS-CYBERBOTS` in the sketch); no MAC address is hard-coded. The remote prints its own BT MAC at startup.
- HC-05 AT commands (name, PIN, role, pairing) differ between module revisions; **verify against your module's datasheet**. Typical: enter AT mode (hold EN/KEY at power-up), `AT+NAME=`, `AT+PSWD=`, `AT+ROLE=0`. These are unverified for your module.
- Bluetooth Classic with a legacy PIN is weak security. Change the default PIN, power the robot's HC-05 only when needed, and do not rely on it to keep out a determined attacker. The lease, link timeout and latched `X` are the safety layer.
- Other robots' remotes in the same venue can connect if they know the name/PIN: use a unique name and PIN.

## Startup: automatic, no command needed

On power-up or reset the Mega initialises with all four motors stopped (motor pins LOW first, then config, HC-05, receivers, MPU6050 and gyro-bias calibration, health check). If the health check is not SAFE_STOP, it waits `startdelay` (1500 ms) and then enables autonomous mode by itself, using the same safety checks as the `a` key. Nothing needs the Serial Monitor, and **a Bluetooth connection is not required**.

Motors stay off, with the reason printed (USB) and logged, if: a fault or SAFE_STOP exists, the drivetrain or sensing is faulted, no usable receiver, ring noise floor not calibrated (`:ring cal noise`, `:save`), the front module is off, or an emergency stop is latched.

The automatic start is one-shot and cancelled by any operator action during the delay: `X`, `M`, `!`/`:estop`, `s`, or a fault. After it has been cancelled or after you stop (`Q`, `s`), the robot does **not** restart by itself until you send `U` / `a` or reset it.

Changing modes safely (all from any state; `X` always wins):
- To manual: send `M` (autonomy ends, robot stands still), then `W/S/A/D/Z/C`.
- To autonomous: send `U` (or `a`); refused if a check fails. After `X` you must send `R` first, then `M` or `U` (the robot stays idle after `R`).
- Emergency stop: `X` (or `!` on USB), latches until `R` / `:recover`.
- Turn automatic start off (always idle at power-up): `:set autostart 0` then `:save`. Change the delay: `:set startdelay <ms>`, `:save`.
