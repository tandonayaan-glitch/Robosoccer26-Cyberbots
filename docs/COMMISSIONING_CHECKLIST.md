# Commissioning checklist (ordered, nothing is checked yet)

Do the phases in order. Stop and fix at the first failure. Tick a box only with a recorded measurement or observation (note value/date/initials).
**No box has been ticked: no physical test has been performed.** Software behaviour referenced below was verified in simulation and by compilation only (`docs/VERIFICATION_REPORT.md`).

## Phase 0 – Robot unpowered (no battery, no USB)
- [ ] The board is a genuine/clone **Arduino Mega 2560**; note which bootloader/USB chip it has (the watchdog stays disabled, `WIRING_MEGA.md` §8).
- [ ] **Identify the two phototransistors** (part number or photo; lead order; emitter-collector breakdown voltage) and the MPU6050 breakout (GY-521 with regulator or bare chip) and the IR modules' supply range. Answer `BALL_SENSING.md` §6.
- [ ] Build each receiver circuit: collector → 5 V, emitter → junction, **its own 10 kΩ** junction → GND, junction → its own analog input (A0, A1). The junctions are not joined.
- [ ] Transmitter LEDs: each with its own resistor (about 180 Ω for 20 mA from 5 V, from the datasheet's V_f/I_f), always on, no GPIO.
- [ ] Remove the ENA/ENB jumpers on both L298N boards; fit **10 kΩ pull-downs on all 12 L298N control inputs** (IN1–IN4, ENA, ENB, both boards); a **fuse** and the **power switch** in the battery positive, reachable at all times.
- [ ] Wiring check against `docs/WIRING_MEGA.md` §2 (every pin): continuity, no shorts between neighbours, **one common ground** (battery −, both L298N, Mega, MPU6050, sensors).
- [ ] Decide how the Mega is powered (USB for the first tests; VIN 7–12 V or a 5 V buck converter later). **Never battery + onto the Mega's 5 V pin.**

## Phase 1 – Electrical checks with a meter (motor battery disconnected from the L298Ns)
- [ ] Logic supply: 5 V rail correct under load (USB or converter).
- [ ] Every receiver junction (measured at the Mega's A0/A1 pin) is about 0 V in the dark and stays ≤ ~4.8 V with a flashlight on the lens; it reacts to light (if not, swap the phototransistor leads).
- [ ] Each IR module OUT, measured before connecting, is within 0–5 V in both states (and ≤ 5 V; ≤ the Mega's VCC + 0.5 V).
- [ ] MPU6050: supply voltage matches the breakout; with the robot idle, SDA/SCL read the breakout's logic level (about 3.3 V on a regulated GY-521).
- [ ] Motor supply polarity; both L298N boards receive Vs; motors **not** connected to the supply yet (or the supply switch off).

## Phase 2 – USB only, motor battery disconnected
- [ ] Build with `tools/avr_build.sh` (or the Arduino IDE: board *Arduino Mega or Mega 2560*, processor *ATmega2560*, AVR core 1.8.3 or newer) and upload.
- [ ] Serial 115200: boot report appears; config line, motor PWM pins OK, ball receivers line, MPU found at 0x68/0x69 (note which).
- [ ] `h`: read every WARNING/NOT TESTED line; autonomy DISABLED; free RAM shown (it should stay above ~2 KB).
- [ ] **Reset-state probe**: meter/scope on each of the 12 L298N control inputs while pressing reset, power-cycling and while the bootloader runs; record the maximum voltage (**≤ 0.8 V required**).
- [ ] `i`, `I`: with the ball off, each receiver's swing is small and steady for 2 minutes; no `RING_*` fault.
- [ ] `m`: IMU readings plausible (1 g on Z at rest), `:gcal` accepted; heading sign: rotate by hand clockwise → heading increases, otherwise `:set yawsign 1`.
- [ ] `:save`, power-cycle, `:get`: the settings come back from the EEPROM.

## Phase 3 – Receivers and modules with the real ball (motors off)
- [ ] `BALL_SENSING.md` §5: ball at 5/10/20/40 cm in front of each receiver: only that receiver reacts; record the swings; choose the load resistor and `ringwinus` if needed.
- [ ] Mounting angles entered (`:ring bearing`); printed bearing within a few degrees; blind arcs recorded.
- [ ] `:ring cal noise` (ball OFF), `:ring cal near`, `:save`.
- [ ] Front module: `DETECTING` exactly in the shooting position (trimmer); polarity set per module (`:mod level`); a wall/hand also trips it (understood: `modneedring`).
- [ ] **Decision recorded** whether the search (rotation) can find a ball from where the robot will be.
- [ ] No false "seen" for 60 s with the ball off, with the transmitter LEDs and modules powered, other IR sources, sunlight, other robots, the motors running (wheels lifted).

## Phase 4 – IMU checks (motors off)
- [ ] 60 s still: heading drift recorded; unplug SDA/SCL → `IMU_TIMEOUT` DEGRADED, reconnect → verified recovery.

## Phase 5 – Supervised motor tests, **wheels off the ground**, hand on the power switch
- [ ] Motor power on. `s` and `!` stop commands tested **first** (send `f`, then `s`).
- [ ] Per wheel: `:mtest FL` → `:confirm` (repeat for FR, BL, BR); `:mok`/`:mrev` per observation.
- [ ] `:dtest` → `:confirm`: forward, backward, strafe left/right, rotate cw/ccw match the labels; `:roller O` if strafe is mirrored.
- [ ] Set `minpwm`, `maxmanual`, `maxauto` from observation/ratings; measure battery current and driver temperature at those limits.
- [ ] `a` with the receivers calibrated: on the stand the wheels turn towards the ball, drive, and stop when the front module detects it; a ball moved out of view makes the robot rotate towards where it went; check `d`.
- [ ] Fault drills (wheels still lifted): unplug the IMU; tie one receiver junction to 5 V; `:set` out-of-range values; `!` mid-motion.

## Phase 6 – Goal heading and shot (only if wanted)
- [ ] `:set goalwidth`, `:set goaldist` for your field; power on pointing at the goal; `[NAV] goal heading registered` appears 5 s after power-on.
- [ ] Wheels lifted: `:set shootenable 1`, ball in the shooting position: aim rotation, reverse, forward burst, once. Then restrained; then on the floor at the lowest `shootpwm` that works.
- [ ] Gyro drift over a realistic match time measured; re-registering procedure (`:goal set`) rehearsed.

## Phase 7 – Floor tests, low speed
- [ ] `:set maxauto 100`; clear area; supervised runs with `:diag on` captured to a file. Only after repeated clean runs raise speeds step by step.

## Sign-off
Competition-ready only when Phases 0–7 are complete with recorded evidence. At present: **not competition-ready.**
