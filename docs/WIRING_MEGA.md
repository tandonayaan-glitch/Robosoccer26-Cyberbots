# Arduino Mega 2560: components, wiring, power and electrical checks

Evidence classes: **[V]** checked by this repository's build or host tests · **[2nd]** general knowledge, not checked against your parts · **[?]** unknown, you must check · **[PHYS]** needs a measurement on the robot.
Nothing in this document has been verified on the physical robot. A firmware that compiles proves nothing about the wiring.

## 1. Components

| Qty | Part | Notes |
|---:|---|---|
| 1 | Arduino Mega 2560 (ATmega2560, 16 MHz, 5 V logic) | replaces the earlier controller board; that firmware is kept in git as commit `1963a3f` (local tag `esp32-final`) |
| 2 | L298N dual H-bridge | remove the ENA / ENB jumpers so the Mega's PWM controls the enable pins |
| 4 | BO geared DC motor + 4 mecanum wheels | roller layout X or O: `:roller X|O`, checked with `:dtest` |
| 1 | battery pack and power switch | 3S 11.1 V nominal (12.6 V full) in the earlier notes **[?]** |
| 6 | 5 mm IR phototransistor (2-pin receivers), each with its own 10 kΩ | on A0…A5; the pinout **[?]** must be checked (§4); where each looks: `:ring bearing` |
| 2 | 940 nm IR LED (transmitters), 5 V | hard-wired with a current-limiting resistor each; **not controlled by the firmware** (§5) |
| 2 | IR obstacle module (VCC / GND / OUT, LM393, trimmer) | front module D34, second module D35 |
| 1 | MPU6050 breakout | hardware I2C D20 / D21 |
| 1 | breadboard, jumper wires, resistors (2 × 10 kΩ load, 2 × LED resistor, pull-downs) | |

**No multiplexer. No CD74HC4067. No kicker or other shooting actuator** (the "shot" is a drive burst). The firmware contains no multiplexer code, and `test/run_tests.sh` fails if its names reappear in the sources.

## 2. Final pin map (compile-time constants in `pins.h`; no setting can change a pin)

### Motors (exactly as specified) [V: `t_pinmap` compares `pins.h` with an independent copy of this table; static_asserts check uniqueness and PWM capability]

| Wheel | Driver / outputs | IN1 / IN3 | IN2 / IN4 | ENA / ENB (PWM) |
|---|---|---:|---:|---:|
| Front-right | **Right** L298N, OUT1 / OUT2 | **D22** | **D23** | **D5** (ENA) |
| Back-right | **Right** L298N, OUT3 / OUT4 | **D24** (IN3) | **D25** (IN4) | **D6** (ENB) |
| Front-left | **Left** L298N, OUT1 / OUT2 | **D26** | **D27** | **D9** (ENA) |
| Back-left | **Left** L298N, OUT3 / OUT4 | **D28** (IN3) | **D29** (IN4) | **D10** (ENB) |

D5, D6, D9 and D10 are hardware PWM pins of the Mega (D2–D13, D44–D46) [V: static_assert]; PWM is `analogWrite` at about 490 Hz, 8 bit. The firmware sets all 12 pins to LOW outputs as the first statement of `setup()` and keeps the motors off until the boot report has been printed.

### Sensors

| Function | Mega pin | Notes |
|---|---|---|
| Phototransistor receiver 0 (first) | **A0** | its own analog input, its own 10 kΩ load resistor |
| Phototransistor receiver 1 (second) | **A1** | its own analog input, its own 10 kΩ load resistor |
| Optional further receivers | A2 … A7 | reserved, off until fitted and switched on with `:ring on <n>` |
| Front IR module OUT (ball in the shooting position) | **D34** | digital input |
| Second IR module OUT | **D35** | digital input, read and logged only |
| MPU6050 SDA / SCL | **D20 / D21** | Mega hardware I2C (`Wire`) |
| USB serial | D0 / D1 | 115200 baud; nothing else may use them |

Free: D2–D4, D7, D8, D11–D19, D30–D33 (unused), D36–D53, A8–A15. Every digital pin used has exactly one function [V: static_assert counts 18 distinct digital pins; the 8 receiver inputs A0–A7 are distinct from them].

## 3. Reference for the checks below: ATmega2560 inputs

The Mega's I/O is 5 V logic: a digital input reads HIGH from about 3.0 V (0.6 × VCC) [2nd]; absolute maximum on any pin is VCC + 0.5 V [2nd]. The ADC uses the 5 V supply as reference (AREF open): readings 0–1023, about 4.9 mV per count. The ADC wants a source impedance of 10 kΩ or less [2nd].

## 4. Phototransistor receivers: circuit and voltage verification

```
 +5 V ──── collector ┐
                     │  5 mm phototransistor (pinout to be verified)
            emitter ─┴───┬──────── analog input A0…A5 (receiver n on An)
                         │
                        10 kΩ       (one resistor PER receiver; the two circuits are never joined)
                         │
 GND ────────────────────┘
```
* **Voltage check (computed, not measured).** The emitter node is at most `5 V − V_CE(sat)`, about 4.6–4.8 V with the collector on 5 V, and it is `I_C × 10 kΩ` below that. The ATmega2560 is a 5 V part, so the pin never exceeds its supply (limit VCC + 0.5 V). With no multiplexer there is no 3.3 V device in this path, so **the 5 V collector is compatible with the Mega's analog inputs**; no divider or series resistor is needed. Use the Mega's regulated 5 V (the same 5 V as VCC of the Mega): if the sensor supply were higher than the Mega's VCC the pin could exceed VCC + 0.5 V. **[PHYS]** measure each junction in the dark and with a flashlight on the lens: expect about 0 V in the dark and up to about 4.7 V in strong light.
* **Pinout is not assumed.** On a 2-lead 5 mm phototransistor the collector / emitter are not marked by the package; the longer lead is **not** a reliable indicator, and clear/dark epoxy and flat-side conventions vary [2nd]. Identify the pins from the datasheet or part marking **[?]**. If the junction does not react to light, the leads are probably swapped. **Before applying 5 V to a part of unknown pinout, check its emitter-collector breakdown (BV_ECO): for many phototransistors it is only about 5–6 V in the reverse direction [2nd]**, so reversing it on a 5 V rail is at the limit; try the first connection at 3.3 V (from the Mega's 3.3 V pin) and look for a response to light.
* **Source impedance.** The junction impedance is at most 10 kΩ (the load resistor), at the recommended limit of the ADC [2nd]. The firmware discards one conversion after the ADC input changes.
* **Speed.** The ball's pulses are short (about 192 µs per burst in the ball source code I read, **your ball may differ**). A phototransistor with a 10 kΩ load needs tens of microseconds to respond [2nd]; if the swing in `:ring` is small with the ball close, try a lower load resistor (4.7 kΩ, 2.2 kΩ) **[PHYS]**.
* **Open input.** An analog input with nothing connected floats and reads random values. Wire every enabled receiver; leave unused ones switched off (the default).
* **Shielding.** A short black tube in front of each receiver limits its view and defines its bearing; the receiver should not see its own transmitter LED directly.

## 5. IR transmitter LEDs (940 nm, 5 V)

Each LED needs its own current-limiting resistor: `R = (5 V − V_f) / I_f`. With an **assumed** V_f of 1.3 V (read your LED's datasheet **[?]**): 20 mA → 185 Ω → **180 Ω** (about 0.07 W); 30 mA → 123 Ω → 120 Ω; 50 mA → 74 Ω → 68 Ω. Start at or below the LED's continuous rating (20 mA is a safe start for a generic 5 mm LED [2nd]). Wire each as 5 V → resistor → LED anode, cathode → GND, always on. Two LEDs at 20 mA are 40 mA from the 5 V rail.

**The firmware does not use the transmitters** and assigns them no pin. Reason: the ball detection implemented here measures the *swing* (pulsing) of the signal at each receiver, which is what the ball's own emitter produces; the light of an always-on LED is steady, has no swing, and is ignored. It therefore does not help (and does not hurt, unless it saturates a receiver) the detection. If you want reflective sensing with these LEDs the firmware would have to pulse them: that needs two output pins and a driver transistor per LED. **Proposal only, not implemented:** D36 and D37 (free, no special function), each through 1 kΩ to the base of a small NPN with the LED and its resistor on the collector side. Tell me if you want it.

## 6. IR obstacle modules (front D34, second D35)

* Three pins each: VCC, GND, OUT; the LM393 output is pulled up to the module's own supply [2nd], so OUT is at most the module's supply voltage.
* **Supply and level check before connecting [PHYS]:** these boards are usually specified for 3.3–5 V [2nd] (check yours). Powered from the Mega's **5 V**, OUT swings between about 0 V and 5 V, which the Mega reads correctly. Powered from 3.3 V, the HIGH level is 3.3 V, above the Mega's threshold of about 3.0 V but with little margin: prefer 5 V. Measure OUT against GND in both states (nothing in range / object in range) before connecting it.
* **Polarity is a setting per module, not an assumption.** The common behaviour is OUT LOW when something is in range [2nd], so the default is "active LOW" for both. Check with `:mod` (it prints the raw pin level and the decoded state) with your hand in front, and set `:mod level 0 low|high`, `:mod level 1 low|high`.
* An **active-LOW** module (the default) gets the ATmega's internal pull-up (about 20–50 kΩ [2nd]): a module whose wire is missing then reads HIGH = "nothing detected" instead of random noise; a real module output overrides the weak pull-up. An **active-HIGH** module gets a plain input (the AVR has no pull-down): a missing one would float, so fit a 10 kΩ pull-down to GND on its OUT if you use that polarity.
* The trimmer sets the distance (about 2–30 cm [2nd]); set it so the module trips exactly where the ball should be when shooting.
* A module reacts to any reflective object, and its LED (usually unmodulated) can add steady light at nearby receivers; steady light has no swing and is ignored, but a strong one can saturate a receiver.

## 7. MPU6050

* SDA → **D20**, SCL → **D21** (Mega hardware I2C; the firmware uses the `Wire` library at 400 kHz with a 20 ms bus timeout [V on the real AVR core: it compiles; the bus itself is **[PHYS]**]). Nothing else uses D20/D21. INT is not used.
* GND → common ground. VCC: a GY-521-type breakout has an on-board 3.3 V regulator and accepts 5 V [2nd]; a bare MPU6050 chip breakout without a regulator is a **3.3 V part** and must not be fed 5 V **[?]**. Check which one you have.
* **Pull-ups.** The Mega enables weak internal pull-ups to 5 V on SDA/SCL when `Wire` starts. The firmware switches them off again (`RS_I2C_INTERNAL_PULLUPS 0`) because the breakout normally has its own 4.7 kΩ pull-ups. If the MPU6050 is then not found, your breakout has no pull-ups: define `RS_I2C_INTERNAL_PULLUPS 1` (or add 4.7 kΩ resistors to the breakout's supply). **[PHYS]** with the robot idle, measure SDA and SCL against GND: they should sit at the breakout's logic level (about 3.3 V for a regulated GY-521); if they read 5 V on a 3.3 V part, use a level shifter (not assumed here).
* Address 0x68 (AD0 low) or 0x69 is auto-detected.

## 8. Power and common ground

```
 battery + ──[ fuse ]──[ power switch ]──┬── L298N right: motor supply (12 V terminal)
                                         ├── L298N left:  motor supply (12 V terminal)
                                         └── (5 V converter input, see below)
 battery − ──────────────────────────────┬── L298N right GND ── L298N left GND
                                         └── Mega GND ── MPU6050 GND ── receiver / LED / module GND   (ALL grounds common)
```
* **Battery positive goes through the power switch** before the L298N motor-supply inputs; battery negative, both L298N grounds, the Mega, the MPU6050 and every sensor circuit share one common ground. Without a common ground the logic signals have no reference.
* **USB for the first tests:** power the Mega from USB with the motor battery disconnected. The L298N logic inputs are then driven by the Mega but nothing can move.
* **Powering the Mega from the robot.** *Never apply the battery's roughly 11 V (12.6 V full) to the Mega's 5 V pin.* Options: (a) the Mega's barrel jack or VIN pin takes 7–12 V (the board's regulator then makes the 5 V; the earlier notes give 12.6 V for a full 3S pack, slightly above the recommended 12 V, and the regulator gets hot at motor-stage currents [2nd]), or (b) a separate regulated **5 V buck converter** from the battery into the Mega's 5 V pin (it bypasses the board's regulator and USB protection: confirm the converter's output with a meter, disconnect USB while running from it, and do not connect both at the same time). Confirm the voltage and polarity of whatever you choose with a meter **before** connecting it to the Mega **[PHYS]**.
* **Do not power the electronics from the L298N's onboard 5 V output.** It is a small regulator meant for the L298N's own logic, it is only present when its 12 V jumper is fitted, and it is not rated for the Mega plus sensors [2nd].
* **Pull-downs.** A Mega pin is a high-impedance input from reset until `motorsPinsSafe()` runs, and the bootloader runs first (about a second). Fit **10 kΩ pull-down resistors** on the 8 direction inputs and the 4 enable inputs of the L298Ns (12 resistors) so the motors cannot twitch while the pins float **[PHYS]**. Software cannot control the pins in that window.
* **Receiver / LED / module supply** from the Mega's 5 V pin (USB or converter), with 100 nF close to the sensors; keep sensor wires away from motor wires (motor noise couples into the analog inputs).
* **Hung CPU.** The hardware watchdog is **off** by default (`RS_USE_WATCHDOG 0`): the stock Mega 2560 bootloader does not clear the watchdog after a watchdog reset and can loop forever; enable it only with a bootloader that handles it. A software loop-stall check still stops the motors if the loop is blocked for over 1 s, but a truly hung CPU leaves the last PWM values in force: the power switch and fuse are the protection.

## 9. Hardware not measurable / not tested

No wheel feedback (PWM shown is the commanded value), no battery voltage or current measurement, motor and driver capability against the BO motors' stall current is unchecked, the receivers' response to your ball, the module distance setting, the strength of motor noise on the analog inputs, and everything in §4–§8 marked **[PHYS]**.

## Added in the Bluetooth / underside-sensor update

| Function | Mega pin | Note |
|---|---|---|
| HC-05 TX -> Mega | D19 (RX1) | direct |
| Mega -> HC-05 RX | D18 (TX1) | 1 kΩ / 2 kΩ divider, see docs/BLUETOOTH.md |
| Bottom IR alignment sensor (single, = module 0) | D34 | 3.3 V, GND, OUT; active level `:mod level` |
| Second module | D35 | not fitted, disabled |

D36 is not used. Per-wheel trim: `wscalefl/fr/bl/br`.


## Receivers and HC-05 (update)

| Function | Pin | Note |
|---|---|---|
| Ball receivers 0–5 | A0, A1, A2, A3, A4, A5 | each junction to its own input; A6, A7 spare |
| Obstacle modules | D34 (front, shooting position), D35 | active level is a setting (`:mod level`) |
| Bottom IR sensor (the only one) | D34 (3.3 V, GND, OUT) | same input as module 0; active level: `:mod level` (default LOW when detecting) |
| HC-05 TXD → Mega | D19 (RX1) | direct |
| Mega → HC-05 RXD | D18 (TX1) | 1 kΩ / 2 kΩ divider; HC-05 VCC per your breakout (docs/BLUETOOTH.md) |
| MPU6050 | SDA D20, SCL D21 | |
| Motors | D22–D29 direction, D5 D6 D9 D10 PWM | unchanged |

Serial2 (D16/D17) and Serial3 (D14/D15) are also free; Serial1 was kept. The Windows laptop connects over the air to the HC-05; nothing else is wired.
