# Ball sensing: the six phototransistor receivers (A0–A5) and the two IR obstacle modules

Wiring, voltages and power are in `WIRING_MEGA.md`. This file explains what the firmware does with the signals. **Nothing here was measured on the robot.**
The code and the commands keep the historical name "ring" (`ring.h`, `:ring`) for the set of ball receivers; there is **no multiplexer**: every receiver has its own analog input.

## 1. What the firmware does

* **Receivers.** Receiver n is read on analog input An (A0 = receiver 0, A1 = receiver 1, up to A7). Six are fitted (A0–A5, `ringenable` default 0x3F); A6, A7 are spare and stay off until you fit them and run `:ring on <n>`.
* **Measurement (non-blocking).** For each enabled receiver in turn: one discarded conversion (the ADC input changed), then one `analogRead` per main-loop pass (about 112 µs each at the Mega's default ADC clock) for `ringwinus` (900 µs), keeping the lowest and the highest reading. The **peak-to-peak swing** (0–1023 counts) is the signal. The loop never waits for the ADC. [V: simulated `t_ring_burst_window`, `t_nonblocking`]
* **Why peak-to-peak.** The RoboCupJunior IR ball is a pulsed emitter [2nd]: in the ball source I read earlier (`golf_ball.ino`, commit `28b66b5` of a repository outside this one, **not part of this repo**; your physical ball may differ) a 40 kHz carrier is gated into bursts of 192 µs about every 833 µs. A window shorter than the gap between bursts misses them (the host test shows a 250 µs window missing bursts and a 900 µs window never missing one). Steady light (sunlight, the always-on transmitter LEDs, the modules' LEDs) has no swing and is ignored. If your ball uses another pattern, raise `ringwinus`.
* **What the receiver circuit does to the 40 kHz carrier** decides how much swing arrives; a phototransistor with a 10 kΩ load follows the burst envelope, not the carrier. Whether your circuit shows a usable swing is **[PHYS]**: read `:ring` with the ball held at 10 cm.
* **Per receiver:** smoothed swing, strength = swing / `ringnear`, **seen** at a swing of at least `max(ringseen, noise × ringnoisex)` (20 counts and 3.0 × the stored noise floor by default, 10-bit counts), off again below 0.6 × that, `ringdebounce` (2) scans to change. A receiver whose lowest reading stays at or above `ringrailhi` (1000) for `ringrailms` (2 s) is reported (`RING_CHANNEL`) and excluded (shorted to the 5 V rail, or blinded).
* **Noise floor.** `:ring cal noise` (ball off or taken away, robot still) measures 2 s of swing per enabled receiver and stores the largest. Autonomy is refused until it has been done.
* **`ringnear`** (375 counts) is a placeholder for "very close"; `:ring cal near` with the ball in the shooting position sets it from the strongest receiver.

## 2. Bearing and the two-receiver reality

Each receiver has a configured **bearing** (`:ring bearing <n> <deg>`; 0 = straight ahead, + = clockwise = right, ±180 = behind). The ball's bearing is the vector average of the strongest receiver and every other seen receiver within `ringblend` (100°) of it, weighted by swing.

**The firmware does not know where you mounted the receivers.** The defaults 0°, 60°, 120°, 180°, −120°, −60° for A0…A5 are an even-spacing GUESS (`bearingsset 0`; telemetry says `"map":"default_guess"`; `defend` refuses to run). A0 is *not* assumed to be the front receiver beyond that guess. Enter the real angles with `:ring bearing <n> <deg>` (0 = straight ahead, + = clockwise/right, ±180 = behind) for each of A0–A5, then `:save`; entering a bearing sets `bearingsset 1`. A single receiver cannot tell which side of its axis the ball is on: the side comes from comparing receivers. With only two receivers there are large blind arcs: the robot rotates to find the ball (§4).

## 3. The modules and the shooting position

* **Front module (D34) detecting = `held`** (the ball is in the shooting position). With `modneedring` 1 (default) the receivers must also see the ball straight ahead (|bearing| ≤ `centerdeg` 15°) for `ready`, because a wall or another robot also trips a module. `:set modneedring 0` makes the module alone decisive.
* **Second module (D35)** is read, debounced and logged only; the navigation does not use it until you decide its job. Enable with `:mod on 1`.
* **Polarity is a setting per module** (`:mod level <0|1> low|high`; default both active-LOW, **not assumed**: check with `:mod` while holding a hand in front).
* Both modules run their own debounce (`moddebounce` × 5 ms).

## 4. Behaviour of the robot

`a` is refused unless a usable receiver exists, `:ring cal noise` was done, the front module is enabled, and the robot is not in SAFE_STOP.

| Ball state | Action |
|---|---|
| not seen at all | after 1 s: search by rotating |
| seen, more than `turndeg` (25°) off the front | turn on the spot towards it; creep only if it is within 60° of the front |
| seen, roughly ahead | drive forward, slowing as the strength rises (`approach` → `slow`), steer gently towards the bearing; the gyro heading is re-latched after a turn |
| close (strength ≥ 0.5) and off-centre | keep the heading and strafe towards it (`alignspeed`) |
| front module detecting | stop going forward; strafe only while the receivers say it is off-centre |
| lost | search |

**Blind spots and search.** When the ball is lost (not seen for `losthold` 300 ms) the robot rotates, choosing the direction by: (1) the way the ball was moving across the view, if faster than 20°/s; (2) the nearest uncovered arc on either side of where it was last seen; (3) the side of its last bearing. The "last bearing" is the one from about 100–150 ms before the loss (the signal fades receiver by receiver and the bearing during the fade drifts). The search is bounded (`searchtime`, `searchtries`, then WAIT). The uncovered arc is an ESTIMATE from the configured bearings and an assumed view of ±`ringfov` (45°) per receiver; `RING_GAP` (DEGRADED) is raised only above `ringmaxgap` (default 360 = never, because two receivers always leave a large gap). [V: simulated `t_nav_blind_spot_search`, `t_nav_turn_toward_ball`, `t_nav_lost_search_reacquire`]

**The shot** (off by default, `:set shootenable 1`): once the ball has been `ready` for `shoothold` ms and the goal heading is registered, aim at the goal heading, reverse a little (in parallel), then drive forward at full power for `shootdrivems`. There is no kicker. See `README.md`.

## 5. Calibration and testing (motors off, wheels lifted)

1. Wire each receiver as in `WIRING_MEGA.md` §4 and measure the junctions (dark, flashlight) **before** relying on the readings; verify the phototransistor pinout.
2. Power up. `i` (or `:ring`) lists A0, A1 with swing, low/high readings, threshold and strength. `NOT CALIBRATED`; `a` is refused.
3. Ball off: the swings should be small and steady (record them). Ball on at 10 cm in front of each receiver: that receiver's swing must jump. Record swing at 5/10/20/40 cm.
4. A swing of 0 with the ball on: check the lead order, the load resistor, the window `ringwinus`, that the ball is awake (in the ball source I read it sleeps after 60 s without vibration), and the wiring.
5. Mounting angles: `:ring bearing <n> <deg>`; move the ball around the front and compare the printed bearing.
6. `:ring cal noise` (ball off), `:ring cal near` (ball in the shooting position), `:save`.
7. Modules: `:mod`, hand in front of the front module, set the level; set the trimmer; check that a wall also trips it.
8. Interference: repeat with the transmitter LEDs and modules powered, the motors running (wheels lifted), competition lighting and other robots.
9. Blind spot: step the ball out of view; the robot should rotate towards where it went. Record the time to re-acquire.
10. Only then: autonomy on the stand (`a`), then the floor at low speed.

## 6. What I still need from you

1. **Mounting angles** of A0 and A1 (left/right, how far off-axis), or a photo from above.
2. **The phototransistor part number** (lead order, BV_ECO) and your actual load resistors.
3. **The MPU6050 breakout** (GY-521 with regulator, or bare chip) and the module supply you use.
4. **The ball** (model; the 192 µs / 833 µs pattern is from the ball source I read, not measured).
5. **Whether you want the transmitters pulsed** (needs pins, see `WIRING_MEGA.md` §5) or hard-wired as now (they do nothing for the firmware).
6. **The second module's job** once you decide where it goes.


## Update: six receivers, polarity, confidence, ambiguity (simulation-verified only)

* **Polarity.** Detection uses the *peak-to-peak swing* inside the sampling window, never the absolute level, so it works whatever the resting level is (the circuit is collector → 5 V, emitter → junction → 10 kΩ → GND, so light *raises* the voltage; a circuit wired the other way round would still give a swing). Tests: `t_ring_polarity_and_calibration` (resting level 40 and 900). The real response to the real ball is NOT TESTED.
* **Noise floor and threshold.** `calibrate` (ball off, robot still, motors stopped; refused if a ball signal is present) stores each channel's largest swing; the detection threshold is `max(ringseen, noise × ringnoisex10/10)`. `:ring cal noise` does only the receiver part.
* **Confidence** (0–100, a diagnostic figure): strength (up to 60) + 20 when ≥ 2 receivers *agree* + 20 once seen for 100 ms. Disagreeing receivers → `ambiguous`: confidence capped at 30, and **no bearing is reported** in telemetry (`bearing_deg: null`); defend mode treats it as "no ball".
* **Ambiguity rule.** A receiver at least half as strong as the strongest one and looking more than `ringblend` (100°) away from it marks the pattern ambiguous (two sources, a reflection, or a wrong mounting-angle table).
* **Weak signals.** Below the threshold the channel is not "seen"; hysteresis (on at the threshold, off at 0.6 of it) and `ringdebounce` stop flicker. Lost-ball handling and the layered search are unchanged (above).
* Distance is never estimated: `strength` is the swing relative to `ringnear` (calibrated with the ball in the shooting position by `:ring cal near`).
