# Commissioning checklist (ordered, nothing is checked yet)

Do the phases in order. Stop and fix at the first failure. Tick a box only with a recorded measurement or observation (note value/date/initials).
**No box has been ticked: no physical test has been performed.** Software behaviour referenced below was verified in simulation only (`docs/VERIFICATION_REPORT.md`).

## Phase 0 – Robot unpowered (no battery, no USB)
- [ ] Identify the exact ESP32 module (can marking: WROOM or WROVER/PSRAM, flash size). PSRAM modules use GPIO16/17: then the IMU pins must change (document it; do not silently change pins).
- [ ] **Identify the ring phototransistors (5 mm) and the multiplexer board** (bare CD74HC4067 or a breakout: pull resistors on S0–S3?): part numbers or photos (phototransistor: lead order, dark/light current, response time) and answer the questions in `docs/RING_AND_MODULES.md` §10 (mounting angles, module pins). **Do not finalise the circuit before this.**
- [ ] Build one receiver circuit per channel (collector to 5 V, emitter → 6.8 kΩ → multiplexer channel, 10 kΩ channel → GND; the series resistor is mandatory, `RING_AND_MODULES.md` §4), the multiplexer wiring (3V3, GND, EN→GND, SIG→GPIO34, S0→GPIO4, S1→GPIO13, **S2→GPIO2, S3→GPIO15**) and the two IR modules (3.3 V, OUT→GPIO35 and GPIO36).
- [ ] Check the MPU6050 breakout: regulator present? pull-ups to which rail? AD0 state?
- [ ] Remove the ENA/ENB jumpers on both L298N boards; decide the logic-supply source (buck 5 V, not the L298N regulator if the pack is above 12 V).
- [ ] Fit: 4.7–10 kΩ **pull-downs on all 12 L298N control inputs** (IN1–IN4, ENA, ENB, both boards); decoupling on the sensor supply; a **fuse** and a **motor-power switch** reachable at all times.
- [ ] Wiring check against `docs/HARDWARE_REVIEW.md` §2 (every GPIO): continuity, no shorts between neighbours, common ground.

## Phase 1 – Electrical checks with a meter (battery disconnected from the motors)
- [ ] Logic supply: 5 V and 3.3 V rails correct under load.
- [ ] Every ring channel node (measured with the multiplexer NOT yet connected, then SIG at GPIO34) stays ≤ 3.3 V in every condition (ball off, ball in front, hand over it, flashlight on the lens; expect ≤ 3.0 V with the 6.8 kΩ + 10 kΩ divider) – **never 5 V**; each module OUT, measured before connecting, is ≤ 3.3 V in both states (modules powered from 3.3 V).
- [ ] **Strapping pins:** the multiplexer board's S0–S3 inputs float (no resistor to a rail; meter, ESP32 unpowered); with S2 (GPIO2) and S3 (GPIO15) connected the board boots after five power-off/on cycles and uploads over USB; GPIO0 and GPIO12 carry no wire.
- [ ] Motor supply polarity; both L298N boards receive Vs; motors **not** connected to the supply yet (or the supply switch off).

## Phase 2 – USB only, motor battery disconnected
- [ ] Build with the board package you intend (2.0.17 or 3.0.7), partition scheme *Default 4MB with spiffs*; upload.
- [ ] Serial 115200: boot report appears; reset reason, filesystem line, `PWM init OK`, ball ring / IR modules line, MPU found at 0x68/0x69 (note which).
- [ ] `h`: read every WARNING/NOT TESTED line; autonomy DISABLED.
- [ ] **Reset-state probe**: scope/meter on each of the 12 L298N control inputs while pressing EN reset, power-cycling and forcing a brown-out; record the maximum voltage (**≤ 0.8 V required**).
- [ ] `i`, `I`: with the ball off, every ring swing is small and steady for 2 minutes; no `RING_*` fault; `:ring cal noise` accepted.
- [ ] `m`: IMU readings plausible (1 g on Z at rest), `:gcal` accepted; heading sign: rotate by hand clockwise → heading increases, otherwise `:set yawsign 1`.
- [ ] `g`: logging ON; deliberately choose a partition without `spiffs` once → logging OFF, robot reports it, **nothing is formatted**; restore.
- [ ] `h` after 10 minutes: record the stack high-water lines (loop and log-writer tasks); they must show headroom.

## Phase 3 – Ring and modules with the real ball (motors off)
- [ ] `RING_AND_MODULES.md` §9: choose R_L and `ringwinus` (does the swing appear with the ball at 10 cm? record it per channel), `:ring cal noise` with the ball off, `:ring cal near`, `:save`.
- [ ] Ball at 5/10/20/40 cm in front of each receiver in turn: only that channel (and neighbours at an angle) reacts; no crosstalk onto other channels. Ranges recorded.
- [ ] Ball rotated around the robot: mounting angles entered (`:ring bearing`), printed bearing within a few degrees; blind arcs recorded (`coverage` line, `ringfov`, `ringmaxgap` adjusted).
- [ ] Module 0: `DETECTING` exactly in the shooting position (trimmer), polarity checked (`:mod level`), a wall/hand also trips it (understood: `modneedring`).
- [ ] C12–C15 fitted? `:ring on 12` etc. with their bearings, retest.
- [ ] **Decision recorded** whether the search (rotation) can find a ball from where the robot will be.
- [ ] No false "seen" for 60 s with the ball off, with the modules powered, with the competition's other IR sources active, sunlight, other robots, the motors running (wheels lifted).

## Phase 4 – IMU checks (motors off)
- [ ] 60 s still: heading drift recorded; unplug SDA/SCL → `IMU_TIMEOUT` DEGRADED, reconnect → verified recovery.

## Phase 5 – Supervised motor tests, **wheels off the ground**, hand on the power switch
- [ ] Motor power on. `s` and `!` stop commands tested **first** (send `f`, then `s`).
- [ ] Per wheel: `:mtest FL` → `:confirm` (repeat for FR, BL, BR); `:mok`/`:mrev` per observation.
- [ ] `:dtest` → `:confirm`: forward, backward, strafe left/right, rotate cw/ccw match the labels; `:roller O` if strafe is mirrored.
- [ ] Set `minpwm`, `maxmanual`, `maxauto` from observation/ratings; measure battery current and driver temperature at those limits.
- [ ] `a` with the ring calibrated: on the stand the wheels turn towards the ball (rotation), drive, and stop when module 0 detects it; a ball moved into a blind arc makes the robot rotate towards where it went; check `d`.
- [ ] Fault drills (wheels still lifted): unplug the IMU; tie one ring channel / SIG to 3.3 V; `:set` out-of-range values; `!` mid-motion.

## Phase 6 – Goal heading and shot (only if wanted)
- [ ] `:set goalwidth`, `:set goaldist` for your field; power on pointing at the goal; `[NAV] goal heading registered` appears 5 s after power-on.
- [ ] Wheels lifted: `:set shootenable 1`, ball in the shooting position: aim rotation, reverse, forward burst, once. Then restrained; then on the floor at the lowest `shootpwm` that works.
- [ ] Gyro drift over a realistic match time measured; re-registering procedure (`:goal set`) rehearsed.

## Phase 7 – Floor tests, low speed
- [ ] `:set maxauto 100`; clear area; supervised runs; review the log (`g`, `:xsel 0`, `x`, verify the CRC). Only after repeated clean runs raise speeds step by step.

## Sign-off
Competition-ready only when Phases 0–7 are complete with recorded evidence. At present: **not competition-ready.**
