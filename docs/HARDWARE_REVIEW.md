# Hardware: components, wiring, pins and electrical notes

Evidence classes: **[V]** established by this repository's build or tests · **[2nd]** general knowledge, not checked against your parts ·
**[?]** unknown, you must check · **[PHYS]** needs a measurement on the robot.

## 1. Component list (what the robot has now)

| Qty | Part | Notes |
|---:|---|---|
| 1 | ESP-32S v1.1 development board, 38 pins | exact module (WROOM vs WROVER/PSRAM, flash size) **[?]**: a PSRAM module uses GPIO16/17, which carry the IMU here |
| 2 | L298N motor driver | remove the ENA/ENB jumpers so the ESP32's PWM controls enable |
| 4 | yellow BO geared DC motor | stall current **[?]**; measure |
| 4 | mecanum wheel | roller layout X or O: `:roller X|O`, tested with `:dtest` |
| 1 | 3S 11.1 V 4500 mAh battery | 12.6 V when full: see the L298N regulator note below |
| 1 | CD74HC4067 16-channel analog multiplexer (module or bare chip **[?]**) | EN tied to GND; S2/S3 sit on the strapping pins GPIO12/GPIO0 (§5); `docs/RING_AND_MODULES.md` |
| 12 (+ up to 4) | 5 mm IR phototransistor (2-pin receiver) on multiplexer C0–C11 (C12–C15 optional) | the **ball ring**; **part number [?]**; each needs a load resistor (collector 3.3 V, emitter to the channel, R_L to GND, start 10 kΩ; `docs/RING_AND_MODULES.md` §4); they see the ball's own emission |
| 2 | IR obstacle module, 3-pin (VCC / GND / OUT, LM393 comparator, trimmer), 3.3 V supply | module 0 at the front (ball in the shooting position); module 1 position undecided; OUT = digital |
| 1 | MPU6050 (gyro + accelerometer) breakout | |
| 1 | breadboard, jumper wires, power wiring | |

**Ball detection hardware:** a ring of 12 phototransistors on a CD74HC4067 (one analog line, GPIO34) that see the **ball's own infrared emission**, optional extra receivers on C12–C15 (left / right), and two digital IR obstacle modules (GPIO35, GPIO36).
The earlier reflective emitter/receiver pair design and its emitter-LED pulsing are gone (`test/run_tests.sh` fails if their names reappear); the 3-pin digital TSOP receivers and the old scan engine are gone too.

**No kicker or shooting actuator is installed.** No kicker pin exists. The firmware "shot" is a drive burst (`docs/RING_AND_MODULES.md`).

## 2. Wiring table (pins fixed in `pins.h`; no setting can change a pin) [V]

### Motors (two L298N; ESP32 logic is 3.3 V)

| Motor | Driver and outputs | IN1 | IN2 | Enable (PWM) |
|---|---|---:|---:|---:|
| Front-right | **Right** L298N, OUT1/OUT2 (IN1/IN2, ENA) | GPIO25 | GPIO26 | GPIO33 (ENA) |
| Back-right | **Right** L298N, OUT3/OUT4 (IN3/IN4, ENB) | GPIO27 (IN3) | GPIO14 (IN4) | GPIO32 (ENB) |
| Front-left | **Left** L298N, OUT1/OUT2 (IN1/IN2, ENA) | GPIO18 | GPIO19 | GPIO23 (ENA) |
| Back-left | **Left** L298N, OUT3/OUT4 (IN3/IN4, ENB) | GPIO21 (IN3) | GPIO22 (IN4) | GPIO5 (ENB) |

Test `t_pinmap` compares `pins.h` with an independent copy of this table and checks the pins that are driven for each wheel and both reversal settings.

### Sensors and IMU

| Device | Pin on the device | ESP32 | Notes |
|---|---|---|---|
| CD74HC4067 SIG | common signal | **GPIO34** | input-only, ADC1_CH6, no internal pull; ring channels arrive here one at a time |
| CD74HC4067 S0 / S1 / S2 / S3 | channel select | **GPIO4 / GPIO13 / GPIO12\* / GPIO0\*** | outputs after boot; \* strapping pins (§5); the chip's select inputs are high-impedance |
| CD74HC4067 VCC / GND / EN | supply / return / enable | 3V3 / common GND / **GND** (always enabled) | |
| Ring phototransistor n (C0–C11, optional C12–C15) | collector → 3.3 V; emitter → channel Cn with R_L to GND | via the multiplexer | **not a direct connection**; every channel voltage must stay within 0–3.3 V |
| IR obstacle module 0 (front, shooting position) | VCC 3.3 V · GND · OUT | **GPIO35** | input-only, ADC1_CH7, no internal pull; digital; `modactivelow` 1 by default [2nd] |
| IR obstacle module 1 (undecided position) | VCC 3.3 V · GND · OUT | **GPIO36 – PROPOSAL** (you wrote "34 and 35": GPIO34 is SIG) | read and logged only |
| MPU6050 | SDA | **GPIO16** | explicit `Wire.begin(16, 17)`; address 0x68 or 0x69 auto-detected |
| MPU6050 | SCL | **GPIO17** | |
| MPU6050 | VCC / GND | 3.3 V (check your breakout's regulator) / GND | |
| USB serial | | GPIO1 (TX), GPIO3 (RX) | 115200 baud |

### Power

| Connection | Notes |
|---|---|
| Battery + / − → both L298N motor-supply (12 V) and GND terminals | fuse and a power switch within reach |
| All grounds common | battery −, both L298N, ESP32, sensors, MPU6050 |
| ESP32 supply **[?]** | not specified in the project notes. Do not power the ESP32 from the 3.3 V pin for loads; use a regulated 5 V into VIN. A charged 3S pack (12.6 V) is above what many L298N boards' on-board 5 V regulator is rated for [2nd]: tell me how yours is powered |

## 3. GPIO assignment summary

| Used | GPIOs |
|---|---|
| Motors (12) | 18, 19, 21, 22, 23, 5 (left) · 25, 26, 27, 14, 33, 32 (right) |
| Multiplexer select S0–S3 | 4, 13, 12, 0 |
| Multiplexer SIG | 34 |
| IR obstacle modules (2) | 35, 36 (36 proposed) |
| I2C / MPU6050 (2) | 16, 17 |
| Serial | 1, 3 |
| Unusable | 6–11 (flash) |

**Still free:** **GPIO39** (input-only, ADC1; no internal pull) · **GPIO2** and **GPIO15** (strapping pins: GPIO2 must not be pulled high at boot, GPIO15 silences the boot log when low; outputs are fine, avoid pull-ups on GPIO2).
GPIO0 and GPIO12 are now the multiplexer's S3 and S2: see §5. GPIO32–39 are ADC1.

## 4. Electrical notes

| Item | Assessment |
|---|---|
| ESP32 logic level | 3.3 V, **not 5 V tolerant**. Every sensor signal into GPIO34/35/36/16/17 must stay ≤ 3.3 V. |
| L298N inputs from 3.3 V | normally accepted [2nd]; remove the ENA/ENB jumpers |
| L298N motor supply | 11.1 V nominal. The bipolar L298N drops about 2 V, so a motor sees about 9 V at full duty [2nd]; about 2 A per channel continuous [2nd] |
| BO motor current **[?]** | measure the stall current; add current limiting/fuses if it exceeds the driver rating. Firmware limits: manual PWM ≤ 150/255, autonomous ≤ 170/255 (placeholders), the shot's drive phase uses `shootpwm` (255 by default: lower it if the driver or battery cannot take it) |
| Ring channels | each phototransistor needs a load resistor; its supply must be 3.3 V so no channel can exceed the multiplexer's VCC. The multiplexer's on-resistance (about 100 Ω [2nd]) and the ADC sampling capacitor load the channel: R_L above about 10 kΩ is slow and picks up the previous channel. The firmware waits `ringsettle` and discards the first conversion after switching |
| GPIO34 (SIG) | input-only, no pull. A disconnected multiplexer leaves SIG floating [2nd] (reads noise, not a stuck value): `:ring cal noise` then shows large swings on every channel and the autonomy refusal message points there |
| IR obstacle modules | OUT is an LM393 output with the board's pull-up: its high level is the module's supply, so power the modules from **3.3 V** (not 5 V) or the pin sees 5 V. Their own IR LED is continuous [2nd] and may disturb nearby receivers |
| Motor noise | the L298N drivers and brushed motors inject noise into supplies: decouple the sensor supply (100 nF close to the sensor), keep the sensor wires away from motor wires |
| MPU6050 breakout **[?]** | GY-521-type boards carry a 3.3 V regulator and 4.7 kΩ pull-ups to the regulated rail; confirm yours does not pull SDA/SCL up to 5 V |
| I2C on GPIO16/17 | free on a plain ESP-32S (WROOM) module; PSRAM modules use them |

## 5. Boot, reset and strapping (no pin changed)

* **Firmware [V]**: `motorsPinsSafe()` is the first statement of `setup()` and drives all 12 motor lines LOW; PWM attaches at duty 0; nothing moves until `f/b/l/r/q/e/:confirm/a`; every reset starts with autonomy off.
* **Window before `setup()` [PHYS]**: from reset until `motorsPinsSafe()` the ESP32 does not drive these pins. Commonly cited reset states: GPIO5 internal pull-up (HIGH), GPIO2/GPIO4 pull-down, others floating inputs; GPIO5/GPIO14 are reported to show activity during boot [2nd]. The consequence depends on your L298N board's input conditioning.
* **Which line matters**: with the L298N, EN low disables the outputs whatever IN1/IN2 are [2nd]. The protective element is a **pull-down on each EN line (ENA/ENB: GPIO23, 33, 5, 32)**; pull-downs on the IN lines are secondary protection. Use ≤ 10 kΩ (4.7 kΩ preferred): GPIO5's internal pull-up forms a divider with the external resistor.
* **GPIO5** (back-left EN) is a strapping pin with an internal pull-up and is reported to emit a PWM signal during boot [2nd]: the back-left motor can twitch at every reset if the L298N inputs are not pulled low. **GPIO14** (back-right IN4) is reported to output PWM at boot [2nd]; harmless if its EN (GPIO32) is held low.
* **Hung CPU / watchdog**: if the loop hangs, LEDC keeps its last duty until the 3 s watchdog fires; the motor power switch/fuse is the only protection for that time.
* **GPIO12 (S2) and GPIO0 (S3)** are strapping pins: GPIO12 must be LOW at reset on a 3.3 V flash module, GPIO0 must be HIGH (LOW = serial bootloader) [2nd]. The firmware drives the select lines only after boot. A bare CD74HC4067's select inputs are high-impedance and do not disturb either level; a breakout board with a pull-up on S2 or a pull-down on S3 would. If the board does not boot or upload with the wires connected, unplug S2 / S3. Automatic upload (DTR/RTS toggling EN and GPIO0) needs S3 to stay high-impedance. [PHYS: reset five times and upload once with the multiplexer connected.]
* **Verify before trusting [PHYS]**: with the motor supply disconnected, probe every L298N control input (12 lines) through repeated resets, brown-outs and a watchdog reset; require ≤ 0.8 V throughout.

## 6. Known limitations

1. **The ring gives strength per channel, not distance**; its bearing is only as good as the configured mounting angles (placeholders until you set them), and it has blind arcs between receivers (the robot rotates to find a ball in one): `docs/RING_AND_MODULES.md`.
2. **Response to the real ball is NOT TESTED**: whether the circuit passes the ball's burst pattern, the module distance setting, interference between the modules' LEDs and the receivers.
3. **No wheel feedback**: commanded PWM only. Rotation, stall, driver faults and wheel slip are NOT MEASURABLE; `:mok` records an operator observation only.
4. **No battery or current measurement.** Brownout resets are the only power evidence.
5. **Gyro heading drifts**; it is relative to the last reference. The goal heading is registered once at start-up.
6. **Pins float during reset/boot**: external pull-downs are the hardware fix.
7. **Flash writes stall the code cache.** Logging runs in a low-priority task on the other core; a flash erase can still pause the control loop for tens of milliseconds.
8. **Motor-driver capability** (L298N voltage drop, 2 A rating) against the motors' stall current is unchecked.
