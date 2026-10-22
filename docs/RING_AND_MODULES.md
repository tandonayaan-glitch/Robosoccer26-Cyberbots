# Ball sensing: the 12-channel phototransistor ring and the two IR obstacle modules

**Status: wired as described by you, not tested on the robot.** The receivers' part number, the mounting angles, the response to the real ball and the module pins are open (§9).
Evidence classes: **[V]** established by this repository's build or tests · **[2nd]** general knowledge, not checked against your parts · **[?]** unknown · **[PHYS]** needs a measurement on the robot.

## 1. What is built

| Part | Wiring | Job |
|---|---|---|
| **CD74HC4067** 16-channel analog multiplexer | VCC → ESP32 3V3 · GND → common GND · **EN → GND** (always enabled) · **SIG → GPIO34** (analog input) · **S0 → GPIO4 · S1 → GPIO13 · S2 → GPIO12\* · S3 → GPIO0\*** (\* strapping pins, §3) | one analog line for the whole ring |
| **12 phototransistors** on **C0–C11** | each through its own load resistor (circuit in §4), front and back of the robot | see the **ball's own infrared emission** (a bearing and a strength) |
| **C12–C15** (optional) | not fitted yet; planned: extra receivers looking **left and right** | `:ring on <ch>` once they are fitted |
| **IR obstacle module 0** (3-pin VCC / GND / OUT, blue board with trimmer) | VCC → 3.3 V · OUT → **GPIO35** | at the **front**: "the ball is in the right position to shoot" |
| **IR obstacle module 1** | VCC → 3.3 V · OUT → **GPIO36** (*proposal*) | position **not decided yet** ("maybe the back"): read and logged only |

**About the module pins.** You wrote "34 and 35". GPIO34 is the multiplexer's SIG line, so two things cannot share it. The firmware uses **GPIO35 for module 0 and GPIO36 for module 1**
(both input-only ADC1 pins with no pull resistors, which suits a push-pull module output). If your wiring differs, tell me the pins; they are compile-time constants in `pins.h`.

There is no emitter LED on the robot that the firmware controls: the ball is the emitter. The previous reflective emitter/receiver pair design is gone.

## 2. How a channel is measured (`ring.h`)

For one channel: set S0–S3, wait `ringsettle` (30 µs), discard one conversion (the sample-and-hold still holds the previous channel), then sample the line for `ringwinus` (900 µs) and keep the
**peak-to-peak swing** (highest − lowest reading). One channel per loop iteration (about 1 ms each); all 12 take about 12 loop iterations. In the host simulation one scan takes about 1 ms (virtual time; the real ADC speed is **[PHYS]**: `:ring` prints the last and longest scan time).

* **Why peak-to-peak.** The RoboCupJunior IR ball is a pulsed emitter [2nd]: in the ball firmware source I read earlier in this project (`golf_ball.ino`, commit `28b66b5` of a repository outside this one; **not part of this repo**, and your physical ball may differ) a 40 kHz carrier is gated into bursts of 192 µs about every 833 µs. A receiver that is
  sampled for less than the gap between bursts can miss them; `t_ring_burst_window` shows a 100 µs window missing bursts and a 900 µs window never missing one. [V] The window must cover at least one burst period (833 µs minus the burst = 641 µs, plus margin).
  If your ball uses a different pattern, raise `ringwinus`.
* A **steady** light (sunlight, a lamp, the modules' unmodulated LEDs [2nd]) has no swing and is ignored. A saturated channel (lowest reading above 4000 for `ringrailms`) is reported and excluded (`RING_CHANNEL`).
* **What the phototransistor circuit does to the 40 kHz carrier** decides how much swing reaches the pin. A phototransistor with a large load resistor cannot follow 40 kHz; it follows the burst *envelope* (192 µs) if its response time is well under that. Whether your circuit shows a usable swing is **[PHYS]**: read `:ring` with the ball held at 10 cm.
* Per channel: smoothed swing (`avg = ½ avg + ½ new`), strength = `avg / ringnear`, **seen** at a swing of at least `max(ringseen, noise × ringnoisex)` (80 counts and 3.0× the stored noise floor by default), off again below 0.6× that, `ringdebounce` (2) scans to change.
* **Noise floor.** `:ring cal noise` (ball switched off or taken away, robot still) measures 2 s of swing on every enabled channel and stores the largest per channel. Autonomy is refused until it has been done.
* `ringnear` (1500 counts) is a placeholder for "very close". `:ring cal near` with the ball in the shooting position sets it from the strongest channel's swing.

## 3. The strapping pins (S2 = GPIO12, S3 = GPIO0)

* **GPIO0** must be HIGH at reset (LOW selects the serial bootloader). **GPIO12 (MTDI)** must be LOW at reset on a 3.3 V flash module (HIGH selects 1.8 V flash and the board will not boot). [2nd]
* The CD74HC4067 select inputs are high-impedance CMOS and draw no current, so a bare chip does not disturb either level. A **breakout board** that has a pull-up on S2 or a pull-down on S3 would. If the board no longer boots or the upload stops working, unplug the S2 and S3 wires and retry. [PHYS]
* The firmware drives S0–S3 only after boot (`ringBegin()`, called from `setup()` after the motor pins are safe), so nothing is driven on them while the chip samples its strapping pins.
* Automatic upload (USB DTR/RTS toggling EN and GPIO0) works with a high-impedance select input on GPIO0. If you add a resistor or LED to S3, uploads can fail. [2nd]

## 4. The receiver circuit (one per channel; values are starting points, not verified)

A 2-pin phototransistor **cannot connect to the multiplexer or a GPIO directly**: it is a light-controlled current source and gives no defined voltage on its own. Per channel (circuit A):

```
 3.3 V ──── collector ┐
                      │  5 mm phototransistor
             emitter ─┴───┬──────── multiplexer channel Cn
                          │
                         R_L  (start 10 kΩ; lower for speed, higher if the signal is tiny)
                          │
 GND ─────────────────────┘
```
* Keep the collector on **3.3 V**, so no channel can exceed 3.3 V (the ESP32 and the multiplexer run on 3.3 V). The multiplexer passes signals between GND and its VCC.
* The multiplexer's on-resistance (about 100 Ω [2nd]) and the ADC's sampling capacitor load the node: R_L above about 10 kΩ is slow and sensitive to the channel switching. The firmware's settle time and the discarded first conversion help; a 100 nF capacitor on SIG to GND is optional but slows the response, so try without.
* **Speed.** The response time of a phototransistor with R_L is roughly R_L × (junction capacitance); at 10 kΩ it is tens of microseconds [2nd]. The ball's burst is 192 µs, so R_L of 10 kΩ or less is the place to start. [PHYS]
* **Lead order.** The collector and emitter are not marked on a 2-lead package; if the signal does not react to light, swap the leads (no damage expected at 3.3 V).
* **Direct light.** Phototransistors saturate in sunlight; a short black tube or shroud in front of each limits the field of view *and* defines the bearing. A shroud also makes the field of view a known quantity (§6).
* The four spare channels C12–C15 need the same circuit.

## 5. Bearing and strength

Each channel has a configured **bearing** (`ringBearing`, degrees; 0 = straight ahead, positive = clockwise = right, ±180 = behind).
The ball's bearing is the vector average of the strongest channel and all other seen channels within `ringblend` (65°) of it, weighted by their swing. `t_ring_detect_and_bearing`: with a cosine response and 30° spacing the estimate lands within 8° of the true angle at 0, ±30, 45, 90, 180 and −135°. [V, simulated response]

**The default bearings are placeholders**: C0–C11 evenly spaced every 30° starting at 0° going clockwise. The firmware does not know where you mounted them. Set the real mounting angles:

```
:ring bearing 0 0        (C0 looks straight ahead)
:ring bearing 1 25 ...   (and so on; whole degrees, -180..180; :save keeps them)
:ring on 12              (when C12 is fitted; :ring bearing 12 90 = looks right, 13 270(-90) = left, ...)
```
Placeholder defaults for the optional channels: C12 and C14 = +90° (right), C13 and C15 = −90° (left); they are **off** until you switch them on.
A channel's strength is only a proxy for distance, valid for this ball, mounting and lighting.

## 6. Blind spots and the search

The ring has "front and back, a slight blind spot" between receivers. The firmware knows about blind arcs from the configured bearings and an **assumed** half field of view per receiver (`ringfov`, 45°):

* `:ring` prints the largest uncovered arc (an *estimate*, not a measurement). Above `ringmaxgap` (90°) the health monitor raises `RING_GAP` (DEGRADED: reduced speed); a smaller gap is only reported.
* A ball in a blind arc is not seen. When the ball is lost (not seen for `losthold` 300 ms) the robot searches by **rotating**. The direction is chosen, in this order: (1) the way the ball was moving across the view, if faster than 20°/s (it left through that side); (2) the nearest blind arc on either side of where it was last seen (it can only have slipped into a gap); (3) the side of its last bearing. Last bearing and rate are taken only from a healthy signal (swing ≥ 2× the threshold), because near the threshold channels drop out one by one and the bearing drifts.
  Tests: `t_nav_blind_spot_search` (blind arc on the left → search left; on the right → right; fast leftward motion → left even though the last bearing was on the right). [V, simulated]
* The search is the existing bounded one: `searchspin`, `searchtime` per attempt, `searchtries` attempts with direction reversal, then WAIT; it never spins forever.
* A single receiver cannot tell which side of its axis the ball is on: with only C0 seeing it, the reported bearing is C0's bearing. The side comes from the neighbouring channels.

## 7. The modules and the shooting position

* Module OUT is **digital**. These boards (FC-51-type [2nd]) normally pull OUT **LOW** when something is in range (`modactivelow` 1). That is not assumed: `:mod` prints the pin level; `:mod level low|high` sets it. A trimmer on the board sets the distance (about 2–30 cm [2nd]); set it so the module trips when the ball is where you want to shoot from.
* **Module 0 detecting = `held`** (the ball is in the shooting position). With `modneedring` 1 (default) the ring must also see the ball **straight ahead** (|bearing| ≤ `centerdeg` 15°): `ready`. A module trips on *any* reflective object (a wall, another robot); requiring the ring's confirmation keeps it from shooting at them. `:set modneedring 0` makes the module alone decisive.
* While `held`, the robot does not drive forward; if `ready` it also stops strafing. `ready` for `shoothold` ms + registered goal heading + `shootenable 1` → the shot (aim, reverse, full-power drive; see `README.md`).
* **Module 1** is read, debounced and logged (`flags` column) and shown by `:mod`; **nothing in the navigation uses it** until you decide its role. Enable with `:mod on 1` (or `:set modenable 3`).
* **Interference.** The modules have their own IR LED (usually continuous, 940 nm [2nd]). A continuous light has no swing and is ignored by the ring's measurement; a *modulated* module LED (some boards) near a receiver would look like a ball. [PHYS]: with the ball off, `:ring cal noise` with the modules powered shows it. Mount the modules so their LEDs do not shine into ring receivers.

## 8. Behaviour of the robot

`a` is refused unless: a usable ring channel exists, `:ring cal noise` was done, module 0 is enabled, and the robot is not in SAFE_STOP. Then:

| Ball state | Action |
|---|---|
| not seen at all | after 1 s: search by rotating (direction as §6) |
| seen, more than `turndeg` (25°) off the front | turn on the spot towards it (rotation request `turnkp` × bearing, between 0.12 and `turnmax`); only a creep forward if it is within 60° of the front, none if it is further round |
| seen, roughly ahead | drive forward, slowing as the strength rises (`approach` → `slow`), steer gently towards the bearing; heading is re-latched after a turn |
| close (strength ≥ 0.5) and off-centre | keep the heading and strafe towards it (`alignspeed`) |
| module 0 detecting | stop going forward; strafe only while the ring says it is off-centre |
| lost | search (§6) |

Faults (`HEALTH_AND_LOGGING.md`): `RING_NONE` and `RING_STALL` stop the robot (CRITICAL); `RING_CHANNEL` (saturated channels excluded) and `RING_GAP` are DEGRADED.

## 9. Calibration and testing (motors off, wheels lifted)

1. **Before power:** build the receiver circuits (§4); check with a meter that every multiplexer output stays within 0–3.3 V in light and dark. Check that the board boots and uploads with the S2 and S3 wires connected (§3).
2. Power up. `i` (or `:ring`) lists C0–C11 with swing, low/high readings, threshold and bearing. `NOT CALIBRATED`; `a` is refused.
3. **One channel at a time.** With the ball off, every swing should be small and steady (record them). Switch the ball on at 10 cm in front of **each** receiver in turn: only that channel (and its neighbours at an angle) should react. A channel that reacts to another channel's ball points to crosstalk through the multiplexer (R_L too large) or a wiring error. Record the swing at 5/10/20/40 cm.
4. If a swing is 0 with the ball on: check lead order, R_L, the window (`ringwinus`), and that the ball is awake (in that source it sleeps after 60 s without vibration).
5. **Bearings.** Rotate the ball around the robot at a fixed distance; record which channels see it at each angle; `:ring bearing <ch> <deg>` for each; check the `coverage` line and `ringfov`.
6. `:ring cal noise` (ball off), `:ring cal near` (ball in the shooting position), `:save`.
7. **Modules.** `:mod`; move the ball in front of module 0: it must report `DETECTING` exactly in the shooting position (trimmer); check the polarity; check that a hand or a wall also trips it.
8. **Interference:** repeat with the modules powered, the motors running (wheels lifted), competition lighting and other robots/balls.
9. Blind spot: put the ball in a blind arc and step out; the robot should rotate towards the right side and re-acquire. Record the time.
10. Only then: autonomy on the stand (`a`), then the floor at low speed.

## 10. What I still need from you

1. **Mounting angles** of C0–C11 (and C12–C15 when fitted), or a photo from above: the bearing and blind-spot logic is only as good as these numbers.
2. **Module pins:** GPIO35 / GPIO36 are my proposal (GPIO34 is taken by SIG).
3. **Receiver part number** (lead order, response time) and your actual load resistor; whether each has a tube/shroud.
4. **The ball:** the RCJ IR golf ball model and its burst pattern (the 192 µs / 833 µs figures come from the ball firmware source I read, not from a measurement of your ball).
5. **Module 1's job** once you decide where it goes.
6. **The multiplexer board:** a bare CD74HC4067 or a breakout (pull resistors on S0–S3?), see §3.

## 11. Changes from the previous (two-sensor) design

Removed: the reflective emitter/receiver pairs, `irEmit` pulsing, the proposed emitter pins (GPIO4/13 are the multiplexer's S0/S1 now), `:ir`, sensor calibration of clear/near per receiver, `ir2.h`, `ircli.h`.
Added: `ring.h`, `ringcli.h` (`:ring`, `:mod`), 16 ring channel settings (`ringenable` mask and `ringBearing`/`ringNoise` tables via `:ring`), module settings, bearing-based approach, blind-spot search, fault codes `RING_NONE` / `RING_STALL` / `RING_CHANNEL` / `RING_GAP`,
CSV columns `ringmask,ball,bearing_deg,strength,flags` (schema 2), config version 15 (saved configurations are reset once on upgrade). The motor pins and the MPU6050 pins are unchanged; `pins.h` asserts it.
