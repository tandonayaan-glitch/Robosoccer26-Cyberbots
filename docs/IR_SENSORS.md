# The two front IR sensors: roles, limits, calibration and testing

Hardware: **two 2-pin IR sensors**, no multiplexer, no sensor ring. Each one is wired to its own ESP32 pin and read directly.

| Sensor | GPIO | Mounting | Role in the firmware |
|---|---|---|---|
| **0** | **35** | front **centre** | how strong the ball's signal is → *seen*, and *near* = "the ball is close enough to be in the shooting position" |
| **1** | **36** | front, **slightly off-centre** (to the right or left of sensor 0) | together with sensor 0: is the ball centred or to one side |

GPIO35 and GPIO36 are input-only pins with no internal pull resistors, both on ADC1. They can be read as analog (default) or digital.

## What a 2-pin sensor can and cannot tell the robot

**Unknown until you tell me the part.** "2-pin IR sensor" is not enough to know what it outputs: a bare IR photodiode or phototransistor
(2 leads) is a *current*, which needs a resistor to become a voltage on the pin; a 2-pin IR receiver *module* (some have only signal and
ground plus an internal supply) behaves differently again. Send the **exact part number or a photo of both sensors and of the circuit
around them**. Until then the firmware assumes nothing about them:

* whether the pin is read as **analog** (ADC, `:ir mode <n> analog`, the default) or **digital** (`:ir mode <n> digital`) is a setting;
* which direction means "ball" is **not assumed**: you calibrate each sensor with nothing in front (*clear*) and with the ball in the shooting
  position (*near*); the strength is `(reading − clear) / (near − clear)`, so a reading that rises *or* falls with the ball both work, and a
  digital sensor is simply one whose readings are only 0 and 4095.

What one sensor **cannot** do, whatever its type:

1. **It gives strength, not position or distance.** It cannot say where the ball is within its field of view, nor how far away it is.
   Strength is only a proxy for distance, and only for this ball, this mounting, this lighting, and the same surface/colour.
2. **It sees nothing outside its own field of view.** With two sensors at the front, the robot has no 360° view. A ball behind or beside the
   robot is invisible. The firmware's answer is a **search**: rotate in place (toward the side where the ball was last seen), pause,
   reverse direction, give up after a limited number of attempts (`searchtime`, `searchtries`, `searchpause`). That finds a ball only when its signal is detectable from the
   position the robot rotates around, i.e. it depends on the **range** of the sensors.
3. **If the sensors are short-range** (reflective sensors that see only a few centimetres) the robot cannot find a distant ball by
   rotating: it would see it only when it is almost touching the front. A practical fix, if you find that is the case: a longer-range ball
   sensor (for example a wider-angle receiver for the ball's own IR emission, or a camera) with the two close-range sensors kept for the
   shooting position. This is **not** added to the firmware: it needs your decision, and the part number.
4. **Ambient light and other IR sources** change the readings of an unmodulated sensor: sunlight, incandescent lamps, other robots' IR, the
   ball's own emission. A bare photodiode cannot tell the ball from sunlight. Test in the lighting of the competition hall.

## How the two sensors are used

Every 5 ms each enabled sensor is read, smoothed, converted to strength, and compared with two thresholds (with hysteresis and a debounce of `irdebounce` samples):

* **seen** at `irseen` % (default 15 %) of the clear→near span;
* **near** at `irnearpct` % (default 70 %).

From the two sensors:

| What the sensors report | What the robot does |
|---|---|
| neither sees the ball | no ball: after `losthold` ms (300) the ball is "lost" and the robot searches |
| the ball is seen, far | drives forward at `approach` speed, slowing smoothly as the strength rises to the near level (down to `slow`) |
| **only one** sensor sees it | cautious: forward speed limited to `slow`, plus a small sideways move (at most `alignspeed`) toward the ball's side **if the mounting side of sensor 1 is set** (`:ir side right|left`, or `:set iroffset 1|-1`). Sensor 1 alone → the ball is on sensor 1's side; sensor 0 alone → on the other side |
| **both** see it | sideways correction from the strength difference until centred (`|offset| ≤ center`, default 0.35) |
| sensor 0 reports **near** (and, with `irneedboth` = 1, sensor 1 sees it too) | **shooting position**: forward motion stops, a small sideways correction continues only while off-centre |
| shooting position **and** centred, held for `shoothold` ms, goal heading registered, `shootenable` = 1 | the shot (below) |

Assumptions to **check on the robot** (the firmware cannot know them):

* **Overlap.** "Both sensors see the ball ⇒ shooting position" assumes the two fields of view overlap at the shooting distance. If at the shooting
  position sensor 1 does *not* see the ball, `:set irneedboth 0` makes sensor 0 alone decide.
* **Side.** With the mounting side not set (`iroffset` 0, the default) no sideways move is ever made; the robot only slows down and
  stops forward motion at the shooting position.
* **Offset sign convention.** The offset is −1 (left) … +1 (right) as seen from behind the robot. If the robot strafes the wrong way, set the opposite side.

## Shooting (no kicker is installed)

The project has **no kicker or other shooting actuator**, and none is invented. The "shot" is the one built in the firmware: aim, reverse a
little, then drive forward at full power. It is **off by default** (`shootenable 0`).

1. Place the robot pointing straight at the goal and keep it still. `goalregms` (5 s) after power-on, once the MPU6050 is calibrated, the heading is registered as "straight at the goal" (`:goal` shows it, `:goal set` registers again, `:goal clear` forgets it).
2. When the ball is in the shooting position (above) for `shoothold` ms: rotate in place until within half the aim window of the goal heading; at the same time (`shootparallel` 1) reverse at `shootbackspeed` for `shootbackms`; then drive forward along the goal heading for `shootdrivems` at `shootpwm` (255 = full power; the acceleration limit is lifted for the shot).
3. Aim window from the goal size: half-angle = atan(((goalwidth − balldia) / 2) / goaldist), clamped to 2–20°; the shot fires inside half of that. Defaults: goal 450 mm, ball 43 mm, distance 1000 mm → window ±11.5°, fires within about ±5.7°. Set your field's real values with `:set goalwidth` and `:set goaldist` (the 2024 international rules were reported to give 600 mm and Australian rules 450 mm; the current rulebook could not be downloaded here: check yours).
4. Cancelled (cooldown started) if the aim takes longer than `aimtimeout`, the gyro reference is lost, or the ball leaves the shooting position for 400 ms. One shot per ball: the ball must leave sensor 0 before another shot can start. `:zero`, a gyro recalibration or an IMU outage invalidate the registered goal heading (`:goal` says INVALID; `:goal set`).

The registered heading drifts with the gyro; nothing corrects it during a match.

## Wiring a 2-pin IR sensor (example; check against your part)

An IR **phototransistor/photodiode** gives a current. To read it, make a divider: sensor between 3.3 V and the pin, resistor from the pin to GND
(or the other way round). Start with 10 kΩ and adjust until the reading with nothing in front is clearly away from 0 and 4095 and changes by at least
500 counts with the ball in the shooting position. Keep the pin voltage between 0 and 3.3 V. Add a 100 nF capacitor from the pin to GND close to the ESP32 if the reading is noisy.
Whichever way you wire it, `:ir cal` learns the direction. A sensor with its own LED (reflective type) needs the LED supply and resistor as its datasheet says.

## Calibration and sensor-testing procedure (motors off, wheels lifted)

1. Power up. `i` shows both sensors, `NOT CALIBRATED`; `a` is refused until sensor 0 is calibrated. Autonomy is off.
2. **Check the raw readings.** With nothing in front: `i` repeatedly for 30 s: raw should be stable (a few counts). Cover the sensor with your hand / hold the ball in front: raw should change clearly. Note min/max (`:ir reset` clears them). Once calibrated, a reading held at the end stop on the *clear* side for 2 s (stuck at 0 for a sensor whose reading rises with the ball, at 4095 for one that falls) raises `IR_FRONT` (sensor 0, CRITICAL: SAFE_STOP) or `IR_ALIGN` (sensor 1, DEGRADED): check the wiring. The end stop on the *near* side is legitimate (a ball touching the sensor can saturate it). Calibrate the *clear* reading in the lighting you will use: a photodiode that reads 0 in a dark hall must be calibrated there, otherwise the check will fire (a clear value within 38 counts of the end stop switches the check off).
3. If the sensor is a digital output (reads only 0 / 4095): `:ir mode 0 digital` (this clears its calibration).
4. **Calibrate sensor 0:** nothing in front → `:ir cal clear 0`; ball in the shooting position → `:ir cal near 0`. The firmware says whether a higher or lower reading means a closer ball. Same for sensor 1 (`:ir cal clear 1`, `:ir cal near 1` with the ball where sensor 1 should see it).
5. `:set iroffset 1` (sensor 1 right of sensor 0) or `-1` (left). `:save`.
6. **Test the states.** `I` toggles a live view. Move the ball from far to near on the centre line: `seen`, then `near` on sensor 0. Move it left and right at the shooting distance: the printed `off=` should be `left`/`centre`/`right` correctly; if it is mirrored, change `iroffset`. Cover each sensor in turn: single-sensor guidance.
7. **Record a table** (distance in cm vs raw for each sensor, with the ball centred and 2 cm left/right) for your own thresholds. Adjust `irseen`, `irnearpct`, `center`, `irdebounce`.
8. **Interference.** Repeat step 2 with the lighting of the competition hall, sunlight if there is any, other robots' emitters on, and the motors running with the wheels lifted. A bare IR sensor that moves with these has to be shielded (tube, hood, filter) or replaced by a modulated receiver.
9. Only then: autonomy on the stand (`a`), then the floor at low speed.

Nothing in this procedure has been performed on the robot: the response of your sensors to the real ball is **NOT TESTED**.
