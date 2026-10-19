# Auxiliary IR inputs (GPIO35 distance sensor, GPIO36 IR receiver)

Status: the inputs are read and reported (`:aux`); with **obstacle avoidance** switched on (below) they also limit the approach speed. Avoidance and
the shot are **off by default**, so the ball detector and the default behaviour are unchanged. Host tests only; nothing here has been tried on
the robot, and the thresholds, speeds and times are untuned defaults.

## Wiring

| Pin | Meaning | Wiring |
|---|---|---|
| GPIO35 | analog IR distance sensor (Sharp GP2Y0A-style), read with `analogRead` (12 bit) | Supply as the sensor requires (usually 5 V). **The output must stay below 3.3 V** (check the datasheet; add a divider if not). 100 nF close to the sensor. |
| GPIO36 | digital IR receiver (TSOP style, active LOW) | Supply 3.3 V. 100 nF + about 100 Ω in the supply line, as for the ring receivers. Output straight to GPIO36 (input-only, no internal pull; a TSOP output already has its own pull-up). |

Both pins are input-only, so no output is added; the compile-time pin checks were extended (distinct from every other pin, not in the output mask, valid input-only GPIOs on the real SoC headers).

## `:aux`

* `:aux on` / `:aux off` / `:aux reset`
* `:aux` prints, while on: the distance sensor's raw ADC value (now, smoothed, min, max; **no unit conversion**: characterise your sensor and mounting first), and for the receiver the 20 ms windows, the share with ≥ 3 pulses, pulses per window and the mean spacing of the pulses in active windows (compare with `:irchar` of the ring receivers).

## Where to put the two on a circular robot

These are suggestions from the geometry, not tested. The ring already gives the ball 360° (12 receivers at 30° steps), so neither new input needs to see the ball.

* **Distance sensor: front, at 0°, pointing straight ahead.** Ahead is where the robot drives when it has the ball: it shows an opponent, the goal wall or a field wall in the way. Mount it at about the height of an opponent's body, not in the plane of the ball receivers, and keep its LED from shining across the receiver ring (a pulsed emitter next to a TSOP can raise noise: run `:irchar` on the nearest ring receiver with the sensor on and off, no ball).
* **IR receiver: rear, at 180°, pointing straight back.** The distance sensor already covers the front; a rear receiver covers the opposite half so a beacon (goal marker, teammate) behind the robot is noticed, and the robot can rotate to scan the rest. If the receiver is meant for a goal beacon you intend to drive towards, put it at the front instead and accept that the distance sensor then shares the front (mount them one above the other).
* Neither at 90° or 270°: a sideways sensor sees the walls while the robot moves along the field and mostly produces readings you cannot use without a heading reference.
* Keep both leads away from the motor and battery wires (L298N noise), and give the distance sensor a clear view (no screw heads or wires in front of it).

The side not covered is a blind spot for the distance sensor; for the ball nothing is lost.

## Obstacle avoidance (off by default)

Both inputs can be distance sensors: GPIO35 = sensor 0 (default direction 0°, front), GPIO36 = sensor 1 (default 180°, rear; when it is a distance
sensor the receiver diagnostic is off). Raw ADC values are compared with two thresholds per sensor, recorded on the robot (the sensor's own curve
decides whether larger means closer; the firmware infers it from which threshold is larger).

1. `:aux on`, put an obstacle at the distance where the robot should **slow down**, `:avoid cal 0 slow`; move it to where the robot should **not go closer**, `:avoid cal 0 stop`.
2. `:avoid on 0`, `:save` (a configuration version change: saved configurations are reset once; re-save your bearings).
3. `:avoid` shows thresholds, the smoothed raw value and the level (0 clear, 1 near, 2 stop zone).

Behaviour (approach only; manual keys and the search spin are not limited):

* The part of the commanded motion that points towards a sensor is limited; the sideways part is kept, so the robot slides along an obstacle.
* Near (level 1): at most `avoidslowpct` (40 %) toward it. Stop zone (level 2): none.
* **Ball in the direction of the obstacle:** if the ball is within `avoidsector` (45°) of that sensor's direction, the stop zone allows a cautious creep of `avoidcaution` (25 %) toward it, so the robot can still reach a ball that sits next to an obstacle. The creep is still limited by nothing but the caution speed: a wall right behind the ball would be touched.
* Stale sensor data counts as "near" (never "clear"). `:aux off` disables avoidance until `:aux on`.
* Mount the sensors **above the ball's height**: a ball on the floor in front of a low sensor reads as an obstacle.
* An unconnected analog pin floats; only enable a sensor that is wired. Raw value saturation, sunlight and black or angled targets are untested.

## Shot (off by default)

`:set shootenable 1`. While approaching, with autonomy on, the ball within `shootalign` (12°) of straight ahead for `shoothold` (300 ms) **and the goal in view**, the robot
reverses at `shootbackspeed` (0.5) for `shootbackms` (250 ms), then drives forward at full speed for `shootdrivems` (400 ms), holding its heading with
the gyro, then returns to seeking. At least `shootcooldown` (3 s) between shots.

* **There is no goal detection yet.** `:goal [ms]` marks the goal as in view for a while (a simulated sighting, for the bench and for tests); a real detector (compass heading, beacon, camera) must call `goalMarkSeen`.
* The ball being close enough to be struck is **not** known: the IR ring gives a direction, not a distance. A real "ball held" sensor is needed before this is more than a bench behaviour.
* Obstacle limits apply during the shot with no ball exemption: a rear obstacle stops the reverse phase, a front obstacle in the stop zone cuts the drive phase.
* Full speed is `maxauto` (scaled down when degraded). Test with the wheels lifted first.
