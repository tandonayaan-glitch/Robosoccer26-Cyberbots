# Hardware compatibility review

Legend: **[V]** verified from a primary source or by the host tests in this repository ·
**[2nd]** second-hand (search snippet, general knowledge) · **[?]** could not be verified – you must check.

> Primary-source fetches (Vishay TSOP17xx datasheet, Elekit RCJ-05 page, RoboCupJunior 2024 soccer
> rules PDF) **all failed in the build sandbox (DNS blocked)**. Anything about those documents below is
> marked [2nd] or [?]. Please read the datasheet and the current rules yourself before relying on them.

## 1. Pin map (preserved exactly; fixed in `pins.h`, no setting can change it) [V]

### Motors (two L298N, 5 V logic inputs driven at 3.3 V)

| Wheel | Driver / outputs | IN1 | IN2 | EN (PWM) |
|---|---|---:|---:|---:|
| Front-right | Right L298N, OUT1/2 (IN1/IN2, ENA) | 25 | 26 | 33 |
| Back-right | Right L298N, OUT3/4 (IN3/IN4, ENB) | 27 | 14 | 32 |
| Front-left | Left L298N, OUT1/2 (IN1/IN2, ENA) | 18 | 19 | 23 |
| Back-left | Left L298N, OUT3/4 (IN3/IN4, ENB) | 21 | 22 | 5 |

The test `t_pinmap` compares `pins.h` against an independent copy of this table and checks, for every
wheel and both reversal settings, that the right IN pins and the right EN pin are driven.

### Multiplexer CD74HC4067

| Mux pin | Connection |
|---|---|
| VCC | ESP32 3.3 V |
| GND | common ground |
| EN | GND (always enabled) |
| SIG | GPIO34 (input only) |
| S0 / S1 / S2 / S3 | GPIO4 / GPIO13 / GPIO15 / GPIO2 |
| C0–C11 | the twelve planned TSOP17xx outputs |
| C12–C15 | reserved, **disabled by default, never scanned, never reported as failed** [V] |

### MPU6050: SDA = GPIO16, SCL = GPIO17 (explicit `Wire.begin(16,17)`), address auto-detected 0x68/0x69 (AD0 low/high).

## 2. Electrical compatibility

| Item | Assessment |
|---|---|
| ESP32 logic level | 3.3 V, not 5 V tolerant. Every signal into GPIO34/16/17 must stay ≤ 3.3 V. |
| L298N inputs from 3.3 V | The L298N datasheet's logic-high threshold is lower than 3.3 V [2nd], so 3.3 V drive is normally fine. Remove the ENA/ENB jumpers so the ESP32 PWM controls enable. |
| **TSOP17xx supply [?]** | The TSOP17xx family is specified for roughly 4.5–5.5 V [2nd, from sibling Vishay datasheets]. Its output has an internal pull-up to its own supply. At 5 V the output idles at 5 V → **would over-volt GPIO34 and the 3.3 V-powered HC4067** (absolute max ≈ VCC + 0.5 V). Two safe options: (a) power the receivers from 3.3 V (outside the nominal range, commonly works with reduced sensitivity – verify with the real ball); (b) power from 5 V and put a divider/series-resistor + clamp on each output. Do not mix. |
| **Receiver pinout [?]** | Bare TSOP parts and "VS1838B-style" breakout modules have **different pin orders** (some datasheets list supply on pin 2 and output on pin 3, others the reverse). Check the marking and the datasheet for the exact part before powering. Counterfeit/relabelled parts are reported for "TSOP1738" [2nd]. |
| **Output polarity [?]** | The TSOP family is active-LOW (output pulls low while a carrier burst is present) [2nd]. Firmware default `activeLow=1`; per channel `:ch <n> pol low|high` changes it. A wrong polarity looks like "stuck" and is flagged. |
| Supply decoupling | Datasheet-style 4.7 µF + series resistor per receiver supply is recommended [2nd]; motors injecting noise into the receiver supply is the most likely cause of false pulses. |
| GPIO34 | Input only, **no internal pull-up/down**. A disconnected line floats and produces random edges (the firmware will see "NOISY" on every channel and refuse autonomy). Fit a 10–100 kΩ pull-up from SIG to 3.3 V so an open line idles HIGH. (With the pull-up, a disconnected receiver is indistinguishable from "no ball" – that is NOT MEASURABLE without a stimulus.) |
| Common ground | Battery −, both L298N GND, ESP32 GND, mux GND, receivers, MPU6050 must share one ground. |
| ESP32 supply | Never from the 3.3 V pin for motors. Use a regulated 5 V (buck) into VIN/5V, not the L298N's on-board regulator. Common L298N modules specify the on-board 5 V regulator only for Vs ≲ 12 V [2nd]; a charged 3S pack is ~12.6 V → remove the regulator jumper and feed logic separately. |
| L298N motor supply | 11.1 V nominal. The bipolar L298N drops roughly 2 V, so a motor sees ~9 V at full duty [2nd]. Continuous current rating is ~2 A per channel [2nd]. |
| BO motor current [?] | Stall current of your "blue BO" motors was not available; measure it. If stall at ~9 V exceeds the driver rating, add current limiting/fuses. Default firmware limits: manual PWM ≤150/255, autonomous ≤170/255 – conservative placeholders, not derived from the motor data. |
| MPU6050 breakout [?] | GY-521-type boards carry a 3.3 V LDO and usually 4.7 kΩ I²C pull-ups to the **regulated** rail; confirm yours does not pull SDA/SCL up to 5 V. AD0 normally has a pull-down (→ 0x68); the firmware tries 0x68 then 0x69 and prints the address found. |
| I²C on GPIO16/17 | Free on a plain ESP-32S (WROOM-class) module. On WROVER/PSRAM modules GPIO16/17 are used by PSRAM – check that your ESP-32S V1.1 is not a PSRAM variant. |

## 3. Boot, reset and strapping analysis (evidence-based, no pin changed)

| GPIO | Role here | Concern | Verdict |
|---|---|---|---|
| **5** | Back-left **EN (ENB)** | Strapping pin (SDIO slave timing), internal pull-up at reset and commonly reported to emit a PWM signal during boot [2nd]. So the back-left driver enable can be HIGH during reset/boot ROM. | **Real risk.** If IN3/IN4 (GPIO21/22, floating inputs during boot) are not pulled low on the driver board, that motor can twitch at every reset. Mitigation: 4.7 kΩ pull-down on every L298N input including this EN. The strapping value itself is harmless for a normal flash-boot. No pin change needed. |
| **14** | Back-right **IN4** | Commonly reported to output PWM at boot [2nd]. | Harmless if the matching EN (GPIO32, floating input at boot) is held low by a pull-down. Same mitigation. |
| **2** | Mux **S3** | Strapping pin; must be LOW/floating to enter download mode. Internal pull-down by default. | Mux select input is high-Z CMOS, so no conflict. Do not add a pull-up to this line. |
| **15** | Mux **S2** | Strapping pin; if LOW at boot, ROM boot messages are silenced. Internal pull-up by default. | Harmless (affects only boot-log output). |
| 12, 0 | unused | – | Not used (good: GPIO12 selects flash voltage). |
| 34 | Mux SIG | Input only, no pull. | See §2. |

Firmware responses: `motorsPinsSafe()` is the first statement in `setup()` and drives all 12 motor pins
LOW; `esp_reset_reason()` is printed and brownout / panic / watchdog resets raise a warning
(autonomy is off after *every* reset anyway). A watchdog (3 s) resets a hung loop; after a reset pins
return to inputs, which is why the **external pull-downs matter** – nothing in software can hold a pin
while the chip is in reset.

## 4. Multiplexer and receiver timing analysis

* The HC4067 connects **one** channel at a time. It cannot capture all twelve outputs simultaneously.
* Switching/settling is expected to be tiny compared with the 10 µs the firmware waits after each
  select change (datasheet-class figure, **not measured here** [?]). The real constraint is *observation
  time*: a TSOP output is a demodulated envelope. The commonly used competition ball pulses at ~1.2 kHz
  (period ≈ 833 µs) [2nd], so a channel must be watched for several periods before "no pulses" means
  "no ball".
* Implementation: for each enabled channel the firmware selects it, waits `irsettle` (10 µs), then
  counts active-level pulses with an edge interrupt on GPIO34 for `irwindow` (default **3000 µs ≈ 3.6
  periods at 1.2 kHz**). The wait is timed by `micros()` while `loop()` keeps running the motors; a late
  window (> 1.5× + 0.5 ms) is discarded, never interpreted. Per window it records channel, raw level,
  pulse count, active time, measured mean pulse period, state, timestamp.
* **Full scan = 12 × (3000 + ~10) µs ≈ 36.1 ms (≈ 28 Hz)**. The host test measured 36.12 ms.
  Each receiver is therefore *looked at* for 3 ms out of every 36 ms (≈ 8 % duty).
  * Fine for a ball that pulses continuously at ~1.2 kHz and moves slowly.
  * Brief occlusions shorter than ~33 ms, or a ball whose duty is very low, can be missed by the
    channel that happens to be unobserved; hysteresis (2 hits within 8 windows ≈ 290 ms) makes
    detection robust but also slower.
  * Raising `irwindow` improves reliability and lengthens the scan proportionally.
* **Hardware alternative if this proves too slow/unreliable** (decide only after measuring with the real
  ball): capture all outputs in parallel – e.g. two 74HC165 parallel-in shift registers read over SPI at
  ≥ 1 MHz (a full 16-line snapshot every few tens of µs, pulse counting in an ISR/timer), or a small
  dedicated microcontroller per ring that counts pulses on all its receivers and reports a bearing
  mask over I²C/UART. A software filter cannot recover pulses the hardware never sampled.

### Receiver / ball compatibility

* TSOP1738 is a **38 kHz** receiver [2nd]. The target ball is described as **40 kHz carrier**. The
  Vishay datasheet family shows sensitivity falling off away from f₀ [2nd]; a +5 % offset normally
  costs range rather than killing detection, **but this is unproven for your parts**. Do not assume
  either outcome: run `:verify <ch>` and `:learn <deg>` with the real ball at several distances.
* If range is inadequate the documented fix is a receiver tuned to 40 kHz (a 40 kHz variant of the
  same receiver family exists – confirm part number in the Vishay catalogue) or a different
  acquisition front end. Nothing in the firmware silently substitutes parts.
* Competition ball: a search result describes the Elekit RCJ-05 ball as a 40 kHz carrier with a
  1.2 kHz stepped (trapezoidal) modulation and a separate 1200 Hz pulse mode [2nd], and indicates
  that the 2024 RoboCupJunior rules use an IR ball in **Soccer Lightweight** while Soccer Open uses a
  passive orange ball [2nd]. **Neither could be confirmed from the primary documents.** Check the
  rules of your actual event. The firmware's detection logic does not depend on a particular
  frequency: it counts pulses (`irminedges`, default 2) and reports the measured period so you can
  see the ball's real pattern in `i`.

## 5. Known limitations (cannot be solved in software alone)

1. **Sensor coverage is unproven.** Bearings are placeholders until measured; the reported 4 cm side
   gaps and the 20–25° downward tilt experiment can only be evaluated physically (`:learn`).
2. **No distance or signal strength.** Receivers are digital. Direction is a heuristic from which
   receivers fire; the "confidence" is a documented heuristic, not a calibrated probability.
3. **Receiver/ball mismatch** (38 kHz vs 40 kHz) is untested.
4. **Sequential mux sampling** – see §4 – 28 Hz scan, 8 % observation duty per receiver.
5. **No wheel feedback**: commanded PWM only. Motor rotation, stall, driver fault, wheel slip are
   NOT MEASURABLE. Only an operator observation (`:mok`) is recorded.
6. **No battery/current measurement.** Brownout resets are the only power evidence.
7. **Gyro heading drifts**; it is relative to the last zero and is only used to hold orientation.
8. **Pins float during reset/boot** – external pull-downs are a hardware fix.
9. **Flash writes stall the code cache.** Logging runs in a low-priority task on the other core, and
   LEDC keeps its last PWM during a stall, but a flash erase can still pause the control loop for tens
   of milliseconds; mitigated by batching (`flushms`), a size cap, and slow idle snapshots.
10. **Motor-driver capability** (L298N voltage drop, 2 A rating) vs motor stall current is unchecked.
