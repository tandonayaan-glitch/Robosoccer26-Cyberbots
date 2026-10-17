# Commissioning checklist (ordered, nothing is checked yet)

Do the phases in order. Stop and fix at the first failure. Tick a box only with a recorded measurement or
observation (note value/date/initials). **No box has been ticked: no physical test has been performed.**
Software behaviour referenced below was verified in simulation only (`docs/VERIFICATION_REPORT.md`).

## Phase 0 – Robot unpowered (no battery, no USB)
- [ ] Identify the exact ESP32 module (can marking: WROOM or WROVER/PSRAM, flash size). PSRAM modules use GPIO16/17 – then the IMU pins must change (document it; do not silently change pins).
- [ ] Read the **real** receiver part marking and datasheet (TSOP1738 vs counterfeit/module); note pinout, Vs range, output structure.
- [ ] Check the MPU6050 breakout: regulator present? pull-ups to which rail? AD0 state?
- [ ] Remove the ENA/ENB jumpers on both L298N boards; decide the logic-supply source (buck 5 V, not the L298N regulator if the pack is > 12 V).
- [ ] Mount/fit: 4.7–10 kΩ **pull-downs on all 12 L298N control inputs** (IN1–IN4, ENA, ENB, both boards); 10–100 kΩ pull-up SIG→3.3 V; decoupling on the receiver supply; a **fuse** and a **motor-power switch** reachable at all times.
- [ ] Wiring check against `docs/HARDWARE_REVIEW.md` §1 (every GPIO) – continuity, no shorts between neighbours, common ground.

## Phase 1 – Electrical checks with a meter (battery disconnected from the motors)
- [ ] Logic supply: 5 V and 3.3 V rails correct under load; ESP32 3.3 V not used for anything but logic.
- [ ] Receiver supply voltage and **idle output voltage ≤ 3.3 V** at the mux input and at GPIO34 (never 5 V).
- [ ] Mux VCC = 3.3 V, EN = GND, S0..S3 wired to GPIO4/13/15/2, SIG to GPIO34.
- [ ] Motor supply polarity, L298N Vs reaches both boards; motors **not** connected to the supply yet (or supply switch off).

## Phase 2 – USB only, motor battery disconnected
- [ ] Build with the board package version you intend (2.0.17 or 3.0.7), Partition Scheme *Default 4MB with spiffs*; upload.
- [ ] Serial 115200: boot report appears; reset reason, filesystem line, `PWM init OK`, MPU found at 0x68/0x69 (note which).
- [ ] `h`: read every WARNING/NOT TESTED line; autonomy DISABLED.
- [ ] **Reset-state probe**: scope/meter on each of the 12 L298N control inputs while pressing EN reset, power-cycling and forcing a brown-out; record the maximum voltage (**≤ 0.8 V required**).
- [ ] Watchdog reset (e.g. temporary test sketch or `esp_restart()` build) – repeat the probe.
- [ ] `i`, `I`: with no ball, channels idle (raw 1 for active-low), no `!` flags for ≥ 2 minutes; with the motors still disconnected.
- [ ] `m`: IMU readings plausible (1 g on Z at rest), `:gcal` accepted; heading sign: rotate by hand clockwise → heading increases, otherwise `:set yawsign 1`.
- [ ] `g`: logging ON; deliberately choose a partition without `spiffs` once → logging OFF, robot reports it, **nothing is formatted**; restore.
- [ ] `h` after 10 minutes: record the stack high-water lines (loop and log-writer tasks) – they must show headroom.

## Phase 3 – IR receivers with the real ball (motors off)
- [ ] `:verify <k>` for each of the 12 receivers (ball touching the face): record PASS/SUSPECT/FAIL.
- [ ] Record the ball's measured pulse period from the `:verify`/`:learn` output; compare with the published/ball documentation; decide `:irgate auto`.
- [ ] Distance/angle matrix at 10, 20, 40, 60 cm: `docs/IR_DETECTION_AUDIT.md` §8 (front, rear, both sides, the ~40 mm side gap, ball orientation, lighting, motors spinning with wheels lifted).
- [ ] 38 kHz receiver vs 40 kHz ball: range achieved vs required – **decision recorded** (keep, change receiver, or change acquisition).
- [ ] Enter measured bearings (`:ch <n> bear <deg>`), mounting tilt metadata, `:save`.
- [ ] No false ball for 60 s with the ball off and with the room's other IR sources active.

## Phase 4 – IMU checks (motors off)
- [ ] 60 s still: heading drift recorded; unplug SDA/SCL → `IMU_TIMEOUT` DEGRADED, reconnect → verified recovery.

## Phase 5 – Supervised motor tests, **wheels off the ground**, hand on the power switch
- [ ] Motor power on. `s` and `!` stop commands tested **first** (send `f`, then `s`).
- [ ] Per wheel: `:mtest FL` → `:confirm` (repeat for FR, BL, BR); `:mok`/`:mrev` per observation.
- [ ] `:dtest` → `:confirm`: forward, backward, strafe left/right, rotate cw/ccw match the labels; `:roller O` if strafe is mirrored.
- [ ] Set `minpwm`, `maxmanual`, `maxauto` from observation/ratings; measure battery current and driver temperature at those limits.
- [ ] `a` with bearings calibrated: robot on the stand follows the ball with the wheels off the ground; check `d` output.
- [ ] Fault drills (wheels still lifted): unplug a receiver supply, IMU; `:set` out-of-range; `!` mid-motion.

## Phase 6 – Floor tests, low speed
- [ ] `:set maxauto 100`; clear area; supervised runs; review the log (`g`, `:xsel 0`, `x`, verify the CRC).
- [ ] Only after repeated clean runs raise speeds step by step.

## Sign-off
Competition-ready only when Phases 0–6 are complete with recorded evidence. At present: **not competition-ready.**
