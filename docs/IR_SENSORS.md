# The two front IR pairs (emitter LED + 2-pin receiver): reflected-light ball detection

**Status of the circuit: NOT FINAL.** What is known: the receivers are **5 mm phototransistors** (reported by you). What is still unknown: the receivers' part number, the emitter LEDs' part number, how the LEDs are to be powered and controlled (§8).
The circuits below are candidates for those parts; check them against the datasheets before building the final wiring.

## 1. What is built

Exactly **2 IR transmitter LEDs and 2 two-pin IR receivers**, as **two transmitter–receiver pairs at the front** of the robot. No multiplexer, no 12-receiver ring.
They detect the ball by **reflected infrared light**: the emitter lights the space in front of the robot and the receiver measures how much of that light comes back from the ball.
This is a ball detector. It is **not** an opponent-goal detector and not a detector of the ball's own emission.

| Pair | Mounting | Receiver pin | Emitter pin | Role |
|---|---|---|---|---|
| **0** | front **centre** | **GPIO35** (input-only, ADC1) | GPIO4 (*proposed*, only if the firmware pulses the LED) | reflection strength → *seen*; *near* = the ball is close enough to be in the shooting position |
| **1** | front, slightly **off-centre** (right or left of pair 0) | **GPIO36** (input-only, ADC1) | GPIO13 (*proposed*) | with pair 0: is the ball centred or to one side |

**The emitter pins are a proposal, not taken from your wiring notes.** Your notes assign no GPIO to the LEDs. GPIO4 and GPIO13 are free, ordinary I/O pins chosen by this firmware; confirm them or
tell me different free pins. The firmware **never touches them** unless you set `:set iremit 1` (`:ir emit on`); with the default `iremit 0` it assumes the emitters are hard-wired on (or driven by something else) and uses no GPIO for them.

## 2. Can the two-pin receivers connect directly to the ESP32? (verification)

**No, not as bare parts. This was checked against what a 2-pin IR receiver is and what the ESP32 inputs are; the exact part is still needed to finish it.**

* A two-pin IR receiver is a photodiode or phototransistor; **yours are 5 mm phototransistors**. A phototransistor is a current source controlled by light, not a logic output. Wired alone between a GPIO and 3.3 V or GND it produces **no defined voltage**: the pin floats.
* It therefore needs, at the least, **one resistor** so the light-dependent current becomes a voltage. **For a phototransistor that one resistor (circuit A) is the starting design**; no amplifier or comparator is needed for a reflective pair at a few centimetres, and none is assumed. What the other types would need:

| Receiver type | What it gives | Minimum circuit | Amplification / comparator? |
|---|---|---|---|
| **Phototransistor** (typically a dark/black or clear 3/5 mm part with a flat or domed lens) | tens of µA to a few mA when lit | **resistor** to make a voltage, ADC or comparator on the pin (circuit A, §2.1) | usually not needed for a reflective sensor at a few cm; a longer range or weak reflection may need a comparator or a higher resistor |
| **Photodiode** (a clear-lens 5 mm part, often with a flat sensitive area) | **nanoamps to a few µA** | a **transimpedance amplifier** (op-amp) or a very large resistor with a buffer (circuit B) | **yes**: with the 100 kΩ–1 MΩ resistor it would need, the source impedance is far above what the ESP32 ADC input tolerates (about 10 kΩ or less [2nd]) |
| **Receiver module with its own demodulator/comparator** (TSOP-type) | a logic level | none | already conditioned, but these are **3-pin**: not a 2-pin part |

Also required, whatever the receiver:

* the pin must stay **between 0 and 3.3 V** (the ESP32 is not 5 V tolerant);
* GPIO35 and GPIO36 are **input-only with no internal pull resistors**: any pull-up or pull-down must be external;
* an unfiltered receiver sees **ambient light and the ball's own emission**: sunlight, lamps, other robots' IR. That is why the firmware can pulse the emitter and subtract (§4), and why a daylight filter / shroud may still be needed;
* the ESP32 ADC is not linear near its ends and has a limited input range (about 0.1–3.1 V at the default attenuation [2nd]); the firmware therefore calibrates with real readings instead of assuming a transfer curve.

### 2.1 Circuit A – 5 mm phototransistor, the circuit to build first (values are starting points, not verified)

```
 3.3 V ──┬── collector ┐
         │             │ phototransistor (emitter toward the pin)
         │      emitter┴───┬──────── GPIO35 (or GPIO36)
         │                 │
         │               R_L  (start with 10 kΩ; 1 kΩ–4.7 kΩ if it saturates, higher if the signal is tiny)
         │                 │
 GND ────┴─────────────────┴──── 100 nF from the pin to GND (optional, reduces noise)
```
The voltage rises with the light (the firmware also accepts the opposite polarity: e.g. the common-emitter variant with the load resistor to 3.3 V, where the voltage falls when lit; `:ir cal` finds the direction).
Choose R_L so that, with the ball in the shooting position, the pin sits well inside 0.3–3.0 V and changes by **at least 500 counts** from the "nothing in front" reading.

Practical notes for a 5 mm phototransistor (general knowledge, **[2nd]**, not checked against your part):

* **Which lead is which.** The collector and emitter are not marked by the two-lead package. Usually the shorter lead / the flat side of the case is the emitter, but **check the datasheet or the marking**; if the signal does not react to light, swap the two leads (a swap is not expected to damage it at 3.3 V).
* **Speed versus signal.** The bigger R_L is, the larger the signal but the slower the response (the transistor's capacitance and R_L form a delay). With emitter pulsing the firmware waits `irsettle` (200 µs) after switching the LED. Test it: with the ball held still, raise `:set irsettle` through 200, 500, 1000, 2000 µs and watch `i` (the `lit`/`dark` readings): if the difference keeps growing, the receiver is too slow for that R_L: lower R_L (4.7 kΩ, 2.2 kΩ, 1 kΩ) or accept a longer settle time.
* **Saturation.** In strong sunlight or very close to the ball a phototransistor with a large R_L saturates (the pin goes to its supply) and the reflection is lost. A smaller R_L, a shroud or a daylight filter helps.
* **Direct light.** The phototransistor must not see its own LED directly (it would be saturated all the time): mount the LED and the receiver side by side with an opaque wall between them, both aimed forward, or the receiver in a short black tube.
* **Cross-talk between the pairs** is handled by the firmware (one emitter on at a time), not by the hardware: aim the two pairs so that pair 0's LED does not shine into receiver 1 more than the ball's reflection does.
* **Variation.** Phototransistors of the same type differ a lot in sensitivity; the two pairs will not read the same: that is why each is calibrated separately (`:ir cal`).

### 2.2 Circuit B – photodiode (only if the parts turn out to be photodiodes; yours are phototransistors)

A reverse-biased photodiode into an op-amp transimpedance stage (feedback resistor in the 100 kΩ–1 MΩ range with a small capacitor), the op-amp powered from 3.3 V, output to the pin.
It needs a rail-to-rail 3.3 V op-amp and care with leakage and noise: do not build it before the photodiode's datasheet (dark current, sensitivity at the LED's wavelength) is known.

### 2.3 If the receivers turn out to be unsuitable

If the part is a 3-pin receiver module, or has an open-collector / digital output, the interface is simpler (a pull-up to 3.3 V, read as digital: `:ir mode <n> digital`), but this is **not** the case that was described.

## 3. The IR transmitter LEDs: 940 nm, 5 V supply (circuit candidate)

Known: **940 nm LEDs powered from 5 V** (reported by you). Not known: the LED part number, so its forward voltage and its maximum continuous and pulsed current (these decide the resistor).

A GPIO **cannot drive an IR LED directly**: an ESP32 pin should source only about 12 mA continuously [2nd], while typical 5 mm IR LEDs are run at 20–50 mA. The 5 V rail is not a problem for the pin: the ESP32 only drives a **transistor base**, and the LED current comes from the 5 V rail through a **low-side NPN switch**. The ESP32 pin never sees 5 V.

```
 +5 V ──┬── R_LED ── LED anode … LED cathode ── NPN collector
        │                                        NPN emitter ── GND
        └─ 10–100 µF + 100 nF to GND, close to the LEDs
 ESP32 emitter pin ── 1 kΩ ── NPN base         (a small-signal NPN such as a 2N2222 / BC547 class part [2nd]; hFE of 50 or more is plenty)
 (a logic-level N-MOSFET works too: pin to the gate, 10 kΩ gate-to-GND)
```
* **Base current:** (3.3 V − 0.7 V) / 1 kΩ ≈ 2.6 mA from the pin, well inside what a pin can give, and enough to switch 20–50 mA with margin.
* **Resistor:** `R_LED = (5 V − V_f − V_ce(sat)) / I_f`. **Assumed** values (read your LED's datasheet): V_f = 1.3 V and V_ce(sat) = 0.2 V.

| LED current I_f | R_LED (calculated) | nearest standard | resistor power |
|---:|---:|---:|---:|
| 20 mA | 175 Ω | 180 Ω | about 0.07 W |
| 30 mA | 117 Ω | 120 Ω | about 0.11 W |
| 50 mA | 70 Ω | 68 Ω | about 0.17 W |

  Choose I_f **at or below the LED's continuous rating** (20 mA is a safe starting point for a generic 5 mm 940 nm LED [2nd]). A higher peak current is allowed only up to the datasheet's pulsed rating at the duty cycle used: the firmware turns an emitter on for about `irsettle` (200 µs) every 5 ms, a duty of about 4 %, but start at the continuous rating.
* **Hard-wired option:** if you do not want the firmware to control the LEDs, wire each LED with the same R_LED straight from 5 V to GND (no transistor, no GPIO). Then the LED current is continuous and **must not exceed the continuous rating**, and ambient light is not cancelled (§4).
* **Supply:** two LEDs at 20–50 mA are 40–100 mA from the 5 V rail: make sure that rail can supply it besides the ESP32 (how the ESP32 is powered is not in your notes). The switching spikes need local decoupling (above). Common ground with the ESP32 is required.
* **HIGH on the emitter pin = LED on** (low-side NPN). The firmware leaves the pin LOW (LED off) except during a pulse.
* **Phototransistor supply:** keep the receiver's collector on **3.3 V** (not 5 V), so its pin voltage can never exceed 3.3 V even in strong light.

## 4. How the firmware uses the pairs

Every 5 ms each enabled receiver is read. With `iremit 0` (default) the reading is used as it is. With `iremit 1` each pair is measured **twice**: with its emitter off ("dark": ambient light and any constant emission such as the ball's own)
and on ("lit"), one pair at a time so the pairs do not disturb each other. The signal is `2048 + (lit − dark)`, so only the reflection of **our own** emitter remains.
`2048` means "no reflection"; the direction of change (up or down) is learned in the calibration, so a receiver that goes low when lit works as well.

From the signal: strength = `(signal − clear) / (near − clear)`, **seen** at `irseen` % (15 %) and **near** at `irnearpct` % (70 %), with hysteresis and `irdebounce` samples.

| What the pairs report | What the robot does |
|---|---|
| neither sees the ball | no ball: after `losthold` ms (300) the ball is "lost" and the robot searches |
| the ball is seen, far | drives forward at `approach` speed, slowing as the reflection strengthens (down to `slow`) |
| **only one** pair sees it | cautious: forward speed limited to `slow`, plus a small sideways move (at most `alignspeed`) toward the ball's side **if the mounting side of pair 1 is set** (`:ir side right|left`). Pair 1 alone → the ball is on pair 1's side; pair 0 alone → on the other side |
| **both** see it | sideways correction from the strength difference until centred (`|offset| ≤ center`, 0.35) |
| pair 0 reports **near** (and, with `irneedboth` 1, pair 1 sees it too) | **shooting position**: forward motion stops; a small sideways correction continues only while off-centre |
| shooting position and centred for `shoothold` ms, goal heading registered, `shootenable` 1 | the shot (§6) |

Assumptions to **check on the robot**: the two beams overlap at the shooting position (otherwise `:set irneedboth 0`); which side pair 1 is on (`iroffset` 0 = not set → no sideways move is ever made); the offset sign (−1 left … +1 right as seen from behind).

## 5. Limits that no software can remove

1. **Strength, not position or distance.** A receiver cannot say where in its field of view the ball is, or how far. Reflection strength also depends on the ball's size, colour and surface, the emitter's current, the angle, and the receiver's aim.
2. **Short range.** A reflective pair typically sees a few centimetres to a few tens of centimetres, depending on the parts. Outside its field of view and range it sees nothing, so the robot has **no 360° view**: it finds a ball by **rotating** (search), which only works when the ball is within range of the front pairs. If your parts turn out to be short-range, a longer-range ball sensor (or a camera) is the practical fix for *finding* the ball; the pairs stay for the shooting position. Nothing like that is added without your decision.
3. **Other IR sources.** Pulsing the emitter removes ambient light and the ball's own emission from the signal; it does not remove other pulsed IR sources aimed at the receivers, nor sunlight so strong that the receiver saturates (then both the dark and lit readings sit at the end stop and the reflection is lost). A shroud, a daylight filter or a lower R_L helps.
4. **Nothing here is tested on the robot.** Whether these parts detect your ball reliably, at what range and in what lighting, is **NOT TESTED**.

## 6. Shooting (no kicker is installed)

There is **no kicker or other shooting actuator** in the project and none is assumed. The "shot" is the one built in the firmware (off by default, `shootenable 0`): aim, reverse a little, then drive forward at full power.

1. Place the robot pointing straight at the goal and keep it still. `goalregms` (5 s) after power-on, once the MPU6050 is calibrated, the heading is registered as "straight at the goal" (`:goal` shows it, `:goal set` registers again, `:goal clear` forgets it).
2. With the ball in the shooting position for `shoothold` ms: rotate in place until within half the aim window of the goal heading; at the same time (`shootparallel` 1) reverse at `shootbackspeed` for `shootbackms`; then drive forward along the goal heading for `shootdrivems` at `shootpwm` (255 = full power; the acceleration limit is lifted for the shot).
3. Aim window from the goal size: half-angle = atan(((goalwidth − balldia) / 2) / goaldist), clamped to 2–20°; the shot fires inside half of that. Defaults: goal 450 mm, ball 43 mm, distance 1000 mm → ±11.5°, fires within about ±5.7°. Set your field's real values (`:set goalwidth`, `:set goaldist`).
4. Cancelled (cooldown started) if the aim exceeds `aimtimeout`, the gyro reference is lost, or the ball leaves the shooting position for 400 ms. One shot per ball. `:zero`, a gyro recalibration or an IMU outage invalidate the registered goal heading (`:goal set`). The registered heading drifts with the gyro.

## 7. Calibration and testing procedure (motors off, wheels lifted)

1. **Before any power:** answer the questions in §8; build the receiver circuit (§2) and, if you want the firmware to pulse the LEDs, the driver (§3); check with a meter that each receiver pin stays within 0–3.3 V in light and dark and that no emitter pin sources more than a few mA.
2. Power up. `i` shows both pairs, `NOT CALIBRATED`; `a` is refused until pair 0 is calibrated. Autonomy is off.
3. **Raw readings** (emitters as wired, `iremit 0`): `i` repeatedly for 30 s with nothing in front: stable to a few counts. Put the ball in front: the reading must change clearly. `:ir reset` clears min/max. Stuck at the end stop on the clear side for 2 s raises `IR_FRONT` (pair 0, CRITICAL) / `IR_ALIGN` (pair 1, DEGRADED): check the wiring.
4. If the receiver output is digital (only two values): `:ir mode <n> digital`. To let the firmware pulse the emitters: `:ir emit on` (clears the calibrations); check with the LEDs visible through a phone camera that they blink.
5. **Calibrate** each pair: nothing in front → `:ir cal clear 0`; ball in the shooting position → `:ir cal near 0`; same for pair 1. The firmware says whether a higher or lower signal means a closer ball. `:set iroffset 1|-1` (or `:ir side right|left`). `:save`.
6. **States.** `I` shows a live view. Move the ball far → near on the centre line (`seen`, then `near`), left/centre/right at the shooting distance (the printed offset side must match), cover each receiver in turn.
7. **Table.** Distance (cm) against signal for each pair with the ball centred and 2 cm to each side; adjust `irseen`, `irnearpct`, `center`, `irdebounce`.
8. **Interference.** Repeat step 3 in the competition lighting, with sunlight if there is any, other robots' emitters on, and the motors running (wheels lifted). Note how the signal moves with `iremit 0` and with `iremit 1`.
9. Only then: autonomy on the stand (`a`), then the floor at low speed.

## 8. What I need from you before the circuit can be finalised

1. **Receivers:** they are 5 mm phototransistors (answered). Still needed: the part number or marking (or a photo of the lens, any markings) and the datasheet link: it gives the dark current, the light current, the response time and the lead order, which decide R_L and `irsettle`. Is the epoxy clear or a black/dark daylight filter?
2. **Emitters:** they are 940 nm and run from 5 V (answered). Still needed: the exact part number or datasheet (forward voltage, maximum continuous and pulsed current, beam angle) and which 5 V rail powers them (the ESP32's 5 V/VIN pin, a buck converter, or another).
3. **Emitter control:** may the firmware control the LEDs through a transistor (§3)? If yes, which free GPIOs do you want (the proposal is GPIO4 and GPIO13), and which transistor do you have (NPN part number, or a logic-level MOSFET)? If no, they stay hard-wired on with R_LED and no GPIO.
4. **Mounting:** the distance between the two pairs, their aiming (straight ahead or angled), the side the offset pair is on, the height above the floor, and whether a shroud or tube separates the receivers from the emitters (the receiver must not see the LED directly).
5. **Ball:** the ball you use (the RoboCupJunior IR ball emits its own IR, so a reflective pair sees the ball's emission as well as the reflection; pulsing the emitter cancels the emission) and its colour.
