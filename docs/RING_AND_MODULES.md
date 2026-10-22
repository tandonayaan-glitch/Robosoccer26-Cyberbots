# Ball sensing: the 12-channel phototransistor ring and the two IR obstacle modules

**Status: wired as described by you, not tested on the robot.** The receivers' part number, the mounting angles, the response to the real ball and the module pins are open (§9).
Evidence classes: **[V]** established by this repository's build or tests · **[2nd]** general knowledge, not checked against your parts · **[?]** unknown · **[PHYS]** needs a measurement on the robot.

## 1. What is built

| Part | Wiring | Job |
|---|---|---|
| **CD74HC4067** 16-channel analog multiplexer | VCC → ESP32 3V3 · GND → common GND · **EN → GND** (always enabled) · **SIG → GPIO34** (analog input) · **S0 → GPIO4 · S1 → GPIO13 · S2 → GPIO2\* · S3 → GPIO15\*** (\* strapping pins, the two harmless ones, §3) | one analog line for the whole ring |
| **12 phototransistors** on **C0–C11** | collector → **5 V**, emitter → own channel, **10 kΩ to GND** at the channel plus a **6.8 kΩ series resistor** (§4: required for voltage safety), front and back of the robot | see the **ball's own infrared emission** (a bearing and a strength) |
| **C12–C15** (optional) | not fitted yet; planned: extra receivers looking **left and right** | `:ring on <ch>` once they are fitted |
| **IR obstacle module 0** (3-pin VCC / GND / OUT, blue board with trimmer) | VCC → 3.3 V · OUT → **GPIO35** | at the **front**: "the ball is in the right position to shoot" |
| **IR obstacle module 1** | VCC → 3.3 V · OUT → **GPIO36** (approved) | position **not decided yet** ("maybe the back"): read and logged only |

**Module pins:** GPIO35 for module 0 and GPIO36 for module 1 (approved by you). Both are input-only pins with no pull resistors, so each module output must drive its line by itself (§7).

**S2/S3 moved from GPIO12/GPIO0 to GPIO2/GPIO15.** This is the one pin change I made, and it is not a motor or MPU6050 pin: the four output-capable GPIOs that were free (0, 2, 12, 15) are all strapping pins, and GPIO0 and GPIO12 are the dangerous ones (§3). **You need to move two wires:** S2 to GPIO2, S3 to GPIO15.

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

## 3. Boot-strapping pins: the risks and what was done

Free GPIOs after the motors, I²C, SIG and the modules: 0, 2, 12, 15 (output-capable, all strapping pins) and 39 (input-only). S2 and S3 need outputs, so a strapping pin is unavoidable. Ranked by risk [2nd, Espressif documentation as I know it; **check your module's datasheet**]:

| Pin | Reset-time role | Risk if the multiplexer disturbs it | Verdict |
|---|---|---|---|
| **GPIO15** (MTDO) | HIGH (internal pull-up) = boot log printed; LOW = log silenced | **none for booting**: only the boot messages change | **safest** → S3 |
| **GPIO2** | must be LOW or floating to enter download mode; unused in normal boot | none for a normal boot; matters only while uploading, and only if driven HIGH | **safe** → S2 |
| GPIO5 (already the back-left motor ENB, unchanged) | selects SDIO timing, no effect on normal boot | negligible | existing |
| GPIO12 (MTDI) | **HIGH at reset selects 1.8 V flash: a 3.3 V flash module will not boot** | board bricked until the level is fixed | **avoid** |
| GPIO0 | **LOW at reset = serial download mode**: the robot sits waiting instead of starting; also used by the USB auto-upload circuit | robot never starts after a power cycle | **avoid** |

* **Why a software delay is not the fix.** The strapping pins are sampled by hardware at reset. What matters is the *electrical level* of GPIO2 and GPIO15 during reset, which the multiplexer's select inputs do not change (they are high-impedance CMOS). The firmware simply never touches S0–S3 until the ESP32 has been running for `RING_BOOT_HOLDOFF_MS` (500 ms); that guarantees it does not fight the reset levels, it does not create them.
* **What can still go wrong [PHYS]:** a multiplexer *breakout board* with pull-up or pull-down resistors on S0–S3. A pull-down on S3 (GPIO15) only silences the boot log. A pull-up on S2 (GPIO2) could matter during a USB upload (GPIO2 should not be forced high). Check the board with a meter: with the ESP32 unpowered or in reset, S2 and S3 should float (no resistor to a rail).
* **Test before trusting:** disconnect and reconnect power five times and check the robot starts each time; upload over USB with the select wires connected. If an upload fails, unplug the S2 wire and retry.
* **USB upload** uses only EN and GPIO0. GPIO0 is no longer connected to anything in this project, so the auto-upload circuit is untouched.

**Initialisation sequence (`ringBegin()`, called once from `setup()`):** (1) the motor pins have already been made safe and the filesystem and PWM are up; (2) wait until 500 ms after reset; (3) for each select pin write LOW to the output register first, then switch it to output (no glitch to HIGH); (4) SIG and the modules become inputs; (5) the scan starts with the first `ringService()` call. Before step 3 the select pins are untouched inputs. `t_ring_boot_sequence` checks the order and that GPIO0 and GPIO12 are never touched. [V, host]

**EN cannot be disabled by software.** You wired EN to GND, and no output-capable GPIO is left to control it, so the multiplexer is enabled from the moment it is powered. This is harmless: the analog switches only pass the selected channel, and the select inputs are not driven (high-impedance) until step 3.

## 4. The receiver circuit: collector on 5 V, and why a series resistor is mandatory

**As specified: collector → 5 V, emitter → its own channel, 10 kΩ from that node to GND, outputs never joined.** **Checked, and not safe on its own:**

* The multiplexer runs from 3.3 V. The CD74HC4067 signal pins must stay between GND and its supply (the absolute maximum is about 0.5 V above VCC, with input clamp diodes beyond that) [2nd, datasheet: confirm]. SIG then goes into GPIO34, whose limit is about 3.6 V [2nd].
* With the collector on 5 V, the emitter node is `Ic × 10 kΩ` and can rise up to about `5 V − 0.2 V = 4.8 V` as soon as the collector current exceeds about 0.5 mA. A 5 mm phototransistor in strong light, or close to the ball, can deliver several mA [2nd]. **So the node would exceed 3.3 V and be clamped through the multiplexer's input diodes into its 3.3 V rail** and the ESP32 pin would be overdriven. Do **not** connect it that way.

**Fix (one extra resistor per channel, needed for safety):** keep the 10 kΩ from the channel node to GND exactly as specified and put a **6.8 kΩ in series between the emitter and the channel node**. The two resistors form a divider:

```
 5 V ──── collector ┐
                    │  phototransistor
           emitter ─┴──[ 6.8 kΩ ]──┬──────── multiplexer channel Cn   (one circuit per channel, never joined)
                                   │
                                 10 kΩ  (the specified load resistor)
                                   │
 GND ──────────────────────────────┘
```
* **Worst case:** emitter at most 5.25 V (5 V +5 %) − 0.2 V (saturation) = 5.05 V → channel node ≤ 5.05 × 10 / 16.8 = **3.0 V**, below 3.3 V with margin, so neither the multiplexer nor GPIO34 sees more than its supply. Computed, not measured: check every channel with a meter in strong light (flashlight at the lens) before connecting the multiplexer. [PHYS]
* The ADC then sees at most about 3.0 V (readings up to roughly 3700 of 4095; the ESP32 ADC is non-linear near the top [2nd]). Source impedance seen by the ADC: 6.8 kΩ ∥ 10 kΩ ≈ 4 kΩ, acceptable [2nd]. The signal is 0.6 of the emitter voltage.
* **Power sequencing:** if the 5 V rail is up while the multiplexer's 3.3 V is not, the channel node (≤ 3.0 V) is above the unpowered multiplexer and conducts through its input diodes: about (V − 0.6 V) / 6.8 kΩ ≈ 0.3–0.4 mA per lit channel back into the 3.3 V rail. Power the 5 V and 3.3 V rails together (one switch) and keep the receivers dark at power-up if you can.
* **A simpler option I recommend if you are free to change it:** collector on **3.3 V** and no series resistor. The emitter then cannot exceed `3.3 V − 0.2 V = 3.1 V` and nothing more is needed. The 5 V collector gives no benefit I can see for the firmware (the swing is what is measured). It is your decision; the code works with either (`ringrailhi` below).
* **Saturation detection:** `ringrailhi` (default 4000) flags a channel whose *lowest* reading in the window stays at or above it. With the divider a lit channel tops out near 3.0 V, so the default only catches a channel shorted to the 3.3 V rail; set `:set ringrailhi 3500` to flag sunlight saturation too (a saturated channel loses its swing). Tune with the real readings from `:ring` (low/high columns).
* The multiplexer's on-resistance (about 100 Ω [2nd]) is small against 4 kΩ. A 100 nF capacitor on SIG is optional and slows the response, so try without.
* **Speed.** The response time of a phototransistor with its load is roughly R × (junction capacitance), tens of µs at these values [2nd]; the ball's burst is 192 µs. [PHYS]
* **Lead order.** The collector and emitter are not marked on a 2-lead package; if the signal does not react to light, swap the leads (no damage expected).
* **Direct light.** A short black tube in front of each limits the field of view and defines the bearing.
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
* **Supply and levels (check before connecting).** These boards are normally specified for 3.3–5 V [2nd]. Power them from **3.3 V**: the LM393 output is pulled up to the module's own supply, so a 5 V module would put 5 V on GPIO35/36, which are not 5 V tolerant. Before connecting OUT, measure it with a meter against GND in both states (nothing in range / object in range): both readings must be ≤ 3.3 V. If a module must run from 5 V, put a divider (for example 10 kΩ + 20 kΩ) or a level shifter between OUT and the pin; that is a part I have *not* assumed.
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

1. **Before power:** build the receiver circuits (§4, **with the 6.8 kΩ series resistors**); with the multiplexer NOT yet connected, measure every channel node in the dark and with a flashlight on the lens: it must stay ≤ 3.3 V (expect ≤ 3.0 V). Check that the board boots and uploads with the S2 and S3 wires connected (§3).
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
7. **Module supply:** confirm they run from 3.3 V and measure OUT in both states (§7); confirm with a meter that the two S2/S3 wires are on GPIO2 and GPIO15.

## 11. Changes from the previous (two-sensor) design

S2/S3 moved from GPIO12/GPIO0 to GPIO2/GPIO15; start-up sequence with a 500 ms hold-off; `ringrailhi`; the 5 V-collector circuit now has a mandatory series resistor.
Removed: the reflective emitter/receiver pairs, `irEmit` pulsing, the proposed emitter pins (GPIO4/13 are the multiplexer's S0/S1 now), `:ir`, sensor calibration of clear/near per receiver, `ir2.h`, `ircli.h`.
Added: `ring.h`, `ringcli.h` (`:ring`, `:mod`), 16 ring channel settings (`ringenable` mask and `ringBearing`/`ringNoise` tables via `:ring`), module settings, bearing-based approach, blind-spot search, fault codes `RING_NONE` / `RING_STALL` / `RING_CHANNEL` / `RING_GAP`,
CSV columns `ringmask,ball,bearing_deg,strength,flags` (schema 2), config version 15 (saved configurations are reset once on upgrade). The motor pins and the MPU6050 pins are unchanged; `pins.h` asserts it.
