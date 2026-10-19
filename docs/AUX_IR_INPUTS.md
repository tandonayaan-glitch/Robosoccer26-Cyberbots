# Auxiliary IR inputs (GPIO35 distance sensor, GPIO36 IR receiver)

Status: **diagnostics only.** The inputs are read and reported (`:aux`); nothing in navigation uses them. They are off after every reset
(`:aux on` switches them on), so the ball detector, the saved configuration and the default behaviour are unchanged. Host tests only; nothing here
has been tried on the robot.

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
