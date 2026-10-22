# Commissioning checklist (ordered, nothing is checked yet)

Do the phases in order. Stop and fix at the first failure. Tick a box only with a recorded measurement or observation (note value/date/initials).
**No box has been ticked: no physical test has been performed.** Software behaviour referenced below was verified in simulation only (`docs/VERIFICATION_REPORT.md`).

## Phase 0 – Robot unpowered (no battery, no USB)
- [ ] Identify the exact ESP32 module (can marking: WROOM or WROVER/PSRAM, flash size). PSRAM modules use GPIO16/17: then the IMU pins must change (document it; do not silently change pins).
- [ ] **Identify the two IR receivers (5 mm phototransistors) and the two IR emitter LEDs**: exact part numbers or photos (receiver: lead order, dark/light current, response time; emitter: 940 nm, 5 V; forward voltage, maximum continuous and pulsed current) and answer the questions in `docs/IR_SENSORS.md` §8. **Do not finalise the circuit before this.**
- [ ] Build the receiver circuit (a phototransistor cannot go straight to a GPIO: collector to 3.3 V, emitter to the pin with a load resistor to GND, start 10 kΩ, `IR_SENSORS.md` §2.1) and the LED circuits: each 940 nm LED with R_LED from the 5 V rail (calculated from its datasheet), either hard-wired or through a low-side NPN (§3) if the firmware is to pulse it; confirm the emitter GPIOs (proposal: GPIO4 and GPIO13).
- [ ] Check the MPU6050 breakout: regulator present? pull-ups to which rail? AD0 state?
- [ ] Remove the ENA/ENB jumpers on both L298N boards; decide the logic-supply source (buck 5 V, not the L298N regulator if the pack is above 12 V).
- [ ] Fit: 4.7–10 kΩ **pull-downs on all 12 L298N control inputs** (IN1–IN4, ENA, ENB, both boards); decoupling on the sensor supply; a **fuse** and a **motor-power switch** reachable at all times.
- [ ] Wiring check against `docs/HARDWARE_REVIEW.md` §2 (every GPIO): continuity, no shorts between neighbours, common ground.

## Phase 1 – Electrical checks with a meter (battery disconnected from the motors)
- [ ] Logic supply: 5 V and 3.3 V rails correct under load.
- [ ] Each receiver's signal voltage at GPIO35 and GPIO36 stays within 0–3.3 V in every condition (nothing in front, ball in front, hand over it, bright light) – **never 5 V**; each emitter pin sources only a few mA (base/gate current), never the LED current.
- [ ] Motor supply polarity; both L298N boards receive Vs; motors **not** connected to the supply yet (or the supply switch off).

## Phase 2 – USB only, motor battery disconnected
- [ ] Build with the board package you intend (2.0.17 or 3.0.7), partition scheme *Default 4MB with spiffs*; upload.
- [ ] Serial 115200: boot report appears; reset reason, filesystem line, `PWM init OK`, IR pairs line, MPU found at 0x68/0x69 (note which).
- [ ] `h`: read every WARNING/NOT TESTED line; autonomy DISABLED.
- [ ] **Reset-state probe**: scope/meter on each of the 12 L298N control inputs while pressing EN reset, power-cycling and forcing a brown-out; record the maximum voltage (**≤ 0.8 V required**).
- [ ] `i`, `I`: raw readings of both sensors with nothing in front are stable for 2 minutes; no `IR_FRONT`/`IR_ALIGN` fault.
- [ ] `m`: IMU readings plausible (1 g on Z at rest), `:gcal` accepted; heading sign: rotate by hand clockwise → heading increases, otherwise `:set yawsign 1`.
- [ ] `g`: logging ON; deliberately choose a partition without `spiffs` once → logging OFF, robot reports it, **nothing is formatted**; restore.
- [ ] `h` after 10 minutes: record the stack high-water lines (loop and log-writer tasks); they must show headroom.

## Phase 3 – IR pairs with the real ball (motors off)
- [ ] `IR_SENSORS.md` §7: choose the load resistor R_L and `irsettle` (settle-time test, §2.1), read mode (`:ir mode`), emitter pulsing (`:ir emit on`, LEDs blinking on a phone camera), calibration of pair 0 and pair 1 (`:ir cal clear|near`), `:set iroffset`, `:save`; compare the signal in the hall's lighting with `iremit 0` and `iremit 1`.
- [ ] The ball moved far → near on the centre line: `seen`, then `near` on sensor 0 (record the distances).
- [ ] Ball left / centre / right at the shooting distance: the printed offset side matches (else swap `iroffset`).
- [ ] Both fields of view overlap at the shooting position (sensor 1 sees the ball there); otherwise `:set irneedboth 0`.
- [ ] Detection range recorded at 5/10/20/40 cm: **decision recorded** whether the search (rotation) can find a ball from where the robot will be, or a longer-range ball sensor is needed.
- [ ] No false "seen" for 60 s with the ball off and with the competition's other IR sources active, sunlight, other robots, the motors running (wheels lifted).

## Phase 4 – IMU checks (motors off)
- [ ] 60 s still: heading drift recorded; unplug SDA/SCL → `IMU_TIMEOUT` DEGRADED, reconnect → verified recovery.

## Phase 5 – Supervised motor tests, **wheels off the ground**, hand on the power switch
- [ ] Motor power on. `s` and `!` stop commands tested **first** (send `f`, then `s`).
- [ ] Per wheel: `:mtest FL` → `:confirm` (repeat for FR, BL, BR); `:mok`/`:mrev` per observation.
- [ ] `:dtest` → `:confirm`: forward, backward, strafe left/right, rotate cw/ccw match the labels; `:roller O` if strafe is mirrored.
- [ ] Set `minpwm`, `maxmanual`, `maxauto` from observation/ratings; measure battery current and driver temperature at those limits.
- [ ] `a` with sensor 0 calibrated: on the stand the wheels follow the ball in front of the sensors; check `d`.
- [ ] Fault drills (wheels still lifted): unplug the IMU; short a sensor to GND; `:set` out-of-range values; `!` mid-motion.

## Phase 6 – Goal heading and shot (only if wanted)
- [ ] `:set goalwidth`, `:set goaldist` for your field; power on pointing at the goal; `[NAV] goal heading registered` appears 5 s after power-on.
- [ ] Wheels lifted: `:set shootenable 1`, ball in the shooting position: aim rotation, reverse, forward burst, once. Then restrained; then on the floor at the lowest `shootpwm` that works.
- [ ] Gyro drift over a realistic match time measured; re-registering procedure (`:goal set`) rehearsed.

## Phase 7 – Floor tests, low speed
- [ ] `:set maxauto 100`; clear area; supervised runs; review the log (`g`, `:xsel 0`, `x`, verify the CRC). Only after repeated clean runs raise speeds step by step.

## Sign-off
Competition-ready only when Phases 0–7 are complete with recorded evidence. At present: **not competition-ready.**
