# IR ball detection audit (highest-priority open risk)

**Bottom line: real ball detection has NOT been demonstrated.** The firmware logic is exercised only
against a simulated ball. Whether twelve TSOP1738 receivers see the competition ball at useful range is
unknown until the physical tests in §8 are run.

Evidence classes used throughout: **[REAL-BUILD]** compiled with the real toolchain · **[EMU]** real
binary run in a third-party ESP32 emulator · **[HOST]** host simulation · **[2nd]** second-hand
(search snippet / general knowledge) · **[?]** unverified · **[PHYS]** needs the physical robot/ball.

## 1. Source documents: what could and could not be read

| Document | Status |
|---|---|
| Vishay TSOP17xx datasheet (pinout, Vs, output stage, f0, burst/gap limits, angle) | **Could not be fetched** (host unreachable from the build sandbox; the fetch agent and `curl` both failed on DNS/proxy). All values below marked [2nd]/[?]. |
| Elekit RCJ-05 / RCJ-05R ball pages | **Could not be fetched.** A search snippet only. |
| RoboCupJunior 2024 Soccer Rules PDF | **Could not be fetched.** A search snippet only. |
| ESP32 datasheet / TRM | **Could not be fetched.** The ESP-IDF headers shipped inside the Arduino cores *were* read (§3). |

No specification was invented. Please read the original documents before trusting any [2nd] value.

## 2. Receiver / ball compatibility

| Question | Status |
|---|---|
| TSOP1738 carrier | 38 kHz nominal [2nd] |
| Competition ball carrier | The brief says 40 kHz. A search snippet on the Elekit RCJ-05 ball says a 40 kHz carrier modulated at 1.2 kHz with a stepped/trapezoidal waveform, plus a separate 1200 Hz pulse mode [2nd]. A snippet of the 2024 rules says the IR ball is used in **Soccer Lightweight**, while Soccer Open uses a passive orange ball [2nd]. **Confirm which ball and league you play.** |
| 38 vs 40 kHz | +5 % offset. Datasheet-family curves show reduced sensitivity away from f0 [2nd]; the size of the range loss for *your* parts is **unknown [PHYS]**. Not assumed compatible, not assumed incompatible. |
| Output polarity | TSOP family is active-LOW [2nd]. Default `activeLow=1`; per channel `:ch <n> pol low|high`. A wrong setting shows up as a permanently "stuck" channel (test `t_ir_polarity`) [HOST]. |
| Pinout / supply | Differs between bare TSOP parts and breakout modules; Vs ≈ 4.5–5.5 V for the family [2nd]. **Must be checked with a meter [PHYS]**; see `HARDWARE_REVIEW.md` §2. A 5 V output must never reach GPIO34 or the 3.3 V multiplexer. |

If measurements show inadequate range, the documented fix is a receiver tuned to 40 kHz (a 40 kHz
variant of the same family exists; confirm the part number in the Vishay catalogue) or a different
acquisition front end. The firmware does not substitute parts.

## 3. How the firmware actually acquires and judges a signal

* Per enabled channel: select (S0..S3), wait `irsettle`=10 µs, then count active-level pulses for
  `irwindow`=3000 µs with an edge interrupt on GPIO34 (the loop keeps driving the motors meanwhile).
* The ISR is **self-contained and IRAM-safe**: it reads the GPIO input register directly and uses
  `esp_timer_get_time()`. *Found by compiling for real:* `digitalRead()` and `micros()` live in flash
  (`nm` shows 0x400d…), so the earlier version called flash code from an `IRAM_ATTR` ISR. Verified by
  disassembly that the only call left in `irIsr()` is `esp_timer_get_time` (IRAM) on both cores [REAL-BUILD].
  Default Arduino builds have `CONFIG_ARDUINO_ISR_IRAM` unset (read from the core's sdkconfig), so GPIO
  interrupts are masked while the logger writes flash: edges in that moment are lost and the loop is
  also stalled, which the late-window check (`window > 1.5×planned + 0.5 ms`) discards instead of misreading.
* Window classification (`irClassifyWindow`): 0 pulses and the line active the whole window → STUCK;
  more than `irnoisy`×window-ms pulses (default 3/ms) → NOISY; ≥ `irminedges` (2) pulses → DETECT;
  optionally also requires the mean pulse period inside `[irperlo, irperhi]` (§6), else OFFBAND.
* A channel is "active" after 2 detections in its last 8 windows and inactive after 0 hits in 8.
* Direction: vector sum of the active trusted channels' calibrated bearings, weighted by hit fraction;
  heuristic confidence (`HEALTH_AND_LOGGING.md` §4). No distance, no signal strength.

### Timing (simulation numbers unless stated) [HOST]

Measured by the seeded benchmark of `docs/LATENCY_OPTIMIZATION.md` (240 injection phases over two scan cycles per distribution; raw
output in `docs/bench/`). **No figure here is a robot measurement [PHYS].**

| Quantity | Legacy engine (`engine 0`) | Engine 2 (default) |
|---|---|---|
| Ring cycle (12 ch × (3000+10) µs) | 36.12 ms; this is also the revisit interval of every receiver while the ring is idle. While a candidate or a flickering track is watched, up to three receivers are visited every second window and the others wait up to two ring cycles. Loop stalls lengthen both (measured per receiver, `i`/`h`). | same |
| Observation duty per receiver | 3 ms of every 36 ms ≈ 8 % | same (ring); several times higher for the watched receivers |
| Ball appears → confirmed, 1 / 2 / 3 receivers (median / p95) | 224/240 · 152/168 · 118/134 ms | 160/177 · 90/105 · 87/102 ms (default `balanced` profile); 127/144 · 58/72 · 52/66 ms with `:irprofile fast` |
| Ball disappears → declared lost (steady signal, median / p95) | 545/561 ms | 134/149 ms |
| Ball disappears → declared lost (flickering signal) | 536/560 ms | 285/291 ms (longer grace on purpose) |
| Declared lost → translation ≤ 10 % of steady (commanded PWM, not wheel speed) | +50 ms | +50 ms (brake first, then search) |
| Consequence at 0.5 m/s (arithmetic on the medians) | ≈ 11 cm before acquiring (1 receiver), ≈ 27 cm before declaring the ball lost | ≈ 8 cm, ≈ 7 cm |

The previously quoted "125–189 ms / 537 ms" came from `t_latency`: **one trial** per case at one scan phase. The distributions in
`docs/LATENCY_OPTIMIZATION.md` replace them. What the receiver itself needs to react and recover (datasheet value, unverified) and the
real ball's modulation are **not** in these numbers.

## 4. Can it tell a ball from interference?

| Source | Firmware outcome | Evidence |
|---|---|---|
| Ambient DC light / sunlight | TSOP AGC removes it; line idles inactive → "no ball" [2nd] | [?] |
| Mains-lamp flicker or other slow envelopes (≈100 Hz) | < 2 pulses per 3 ms window → not a detection | `t_ir_slow_interferer` [HOST] |
| Motor-PWM / supply noise (kHz range, many pulses) | > 9 pulses/window → NOISY → channel excluded while it persists | `t_ir_noise`, `t_classify` [HOST] |
| Unrelated 38–40 kHz source with a *ball-like envelope rhythm* | **Accepted as a ball** unless the period gate is on | `t_ir_period_gate` documents this limitation [HOST] |
| Same-rhythm source (another team's ball) | **Indistinguishable** by design | – |
| Unrelated remote-control bursts (e.g. NEC-style frames) | Possible brief false detections; not simulated; intermittent frames limit it via the 2-of-8 hysteresis | [?] [PHYS] |

### Review of the earlier "4 kHz interferer taken for a ball" fix
* Before: noise threshold 6 pulses/ms (> 18 per window) accepted a 4.2 kHz interferer as a detection.
  After: **3 pulses/ms (> 9 per window)**: this *tightens* the threshold (it was not relaxed). The ball's nominal 1.2 kHz [2nd] is 0.4 of the limit.
* Cost: a ball whose envelope were faster than ~3 kHz would be called noisy. The limit is configurable (`:set irnoisy`).
* (Update: what the gate measures, its weakness against missed bursts, and the hardware experiment that decides it are in `docs/IR_HARDWARE_EXPERIMENT.md`.)
* The principled complement is the **pulse-period gate** (`:irgate`): measure the real ball with `:verify`/`:learn`
  (the firmware reports mean/min/max pulse period), then `:irgate auto` accepts only ±35 % around it. It
  is **off by default** because the ball's real pattern is unconfirmed; a threshold guessed without a
  measurement would be the "tune until the test passes" mistake.
* Regression tests: `t_classify` (13 pulses/ms → NOISY), `t_ir_noise` (4.2 kHz on a channel → excluded, no ball), `t_ir_period_gate`.

## 5. Healthy "no ball" versus a broken receiver

| Real condition | What the pin does | Firmware verdict | Can it be certain? |
|---|---|---|---|
| Healthy, no ball | idles inactive | no ball (navigation, **not** a fault) | – |
| Open/unplugged line, external pull-up fitted | idles inactive | looks like "no ball" | **No – NOT MEASURABLE** without a stimulus; only a failed `:verify` (twice) reveals it |
| Open line, no pull-up (floating GPIO34) | random edges | usually NOISY/STUCK suspect; cause unproven | Suspected only |
| Output shorted low / wrong polarity | held at active level | SUSPECT_STUCK after 2 s, FAIL_STUCK after 15 s | The symptom is certain; the cause (short vs polarity vs saturated part) is not |
| Output dead/stuck inactive | idles inactive | looks like "no ball" | No – as the open-line case |
| Saturated by strong IR | typically returns to inactive [2nd] | looks like "no ball" or detections if modulated | No |
| Supply noise / interference | pulse flood | SUSPECT_NOISY, excluded while it lasts | Symptom certain, cause unproven |
| Spare channels C12–C15 | never selected | DISABLED, never reported failed, can never create a detection | `t_ir_spare_channels` [HOST] |

## 6. What `:verify` and `:learn` do and do not prove
* `:verify <ch>` (3 s): counts that channel's detection windows. It records a pass if ≥ max(3, 25 %) of the windows detected. It proves that **something producing demodulated pulses was near that receiver**, not that it was the competition ball, not that 40 kHz works at range, not that the wiring is right for other channels. A single failure is only SUSPECT; two are FAIL.
* `:learn <deg>` (2.5 s): records which channels fired at an operator-declared bearing and the measured pulse period. It does not know the true bearing; the result is only as good as the operator's placement.
* Neither produces compatibility evidence until you have run them with the **actual ball** at the distances in §8. At the time of writing no such result exists.

## 7. Geometry (calculator output; inputs are assumptions) [HOST arithmetic]
`tools/coverage_calc.py` (receiver height 101.6 mm, ball 42 mm, half-angle assumed 30/45/60° because the datasheet angle is unverified):

```
receiver height 101.6 mm, ball centre at 21.0 mm -> receiver is 80.6 mm above the ball centre

1) VERTICAL: depression angle from receiver to the ball centre = atan(dz/d)
   d=  50 mm  depression  58.2 deg
   d= 100 mm  depression  38.9 deg
   d= 200 mm  depression  21.9 deg
   d= 400 mm  depression  11.4 deg
   d= 600 mm  depression   7.7 deg

2) Nearest horizontal distance d at which the ball CENTRE is inside the vertical cone
   (visible while |depression - tilt| <= half-angle; far end is limited by signal, not by geometry)
   tilt    half=  30  half=  45  half=  60     boresight meets floor   boresight meets ball centre
      0      140 mm       81 mm       47 mm                 (never)                    (never)
     10       96 mm       56 mm       29 mm                  576 mm                     457 mm
     20       68 mm       38 mm       14 mm                  279 mm                     221 mm
     22       63 mm       34 mm       11 mm                  251 mm                     199 mm
     25       56 mm       29 mm        7 mm                  218 mm                     173 mm
     30       47 mm       22 mm      0 mm                   176 mm                     140 mm
     45       22 mm      0 mm       0 mm                   102 mm                      81 mm

3) SIDE GAP model (azimuth, two outward-facing receivers separated by s along the chassis side)
   Cones overlap only beyond x = (s/2)/tan(half_h) from the chassis; a ball nearer than that, between
   them, can be invisible. s here = the reported gap; a real value needs a measurement.
   half-angle   30 deg: blind pocket reaches ~ 34.6 mm out from the chassis (s=40 mm)
   half-angle   45 deg: blind pocket reaches ~ 20.0 mm out from the chassis (s=40 mm)
   half-angle   60 deg: blind pocket reaches ~ 11.5 mm out from the chassis (s=40 mm)

4) AZIMUTH coverage of N evenly spaced receivers (the firmware's placeholder layout)
   12 receivers, half-angle   30: uncovered azimuth between neighbours =   0.0 deg; every bearing seen by >= 2 receivers
   12 receivers, half-angle   45: uncovered azimuth between neighbours =   0.0 deg; every bearing seen by >= 3 receivers
   12 receivers, half-angle   60: uncovered azimuth between neighbours =   0.0 deg; every bearing seen by >= 4 receivers

Nothing here proves coverage. Use ':learn <deg>' sweeps with the real ball (docs/CALIBRATION_AND_TEST_PLAN.md).
```

Reading it:
* At 101.6 mm height the receivers sit ≈ 80.6 mm above the ball centre. With zero tilt and a 45° vertical half-angle the ball is out of the vertical cone closer than ≈ 81 mm; a 22° downward tilt brings that to ≈ 34 mm, and boresight meets the ball centre at ≈ 200 mm. The tilt mainly helps **close** range; far range is limited by signal, not geometry.
* The azimuth table assumes 12 evenly spaced receivers. **The firmware makes no such assumption**: bearings are placeholders, flagged UNCALIBRATED, and `a` is refused until at least `mintrusted` trusted channels have measured bearings.
* **Side blind spot.** The reported ≈ 40 mm side gap is *not representable* in the azimuth-only coverage model, so `h` says "largest AZIMUTH gap 0 deg" **and** prints a warning that this is not continuous coverage. If the ball vanishes while last seen at 60–120° the log records `ball_lost_side_zone`. This is a diagnostic hint only; software does not fix a physical gap.
* Mapping physical position → channel is done by the operator (`:ch <n> bear <deg>`, `:learn`); nothing in the code assumes channel N points anywhere. Only C0–C11 are enabled by default; C12–C15 stay off until you install and calibrate receivers.

## 8. Physical test procedure with the real ball (not yet performed)
Preconditions: wheels off the ground or robot on a stand, motors stopped, competition lighting, ball powered with a fresh battery, `i`/`I` available.

1. Mark a circle plan on the floor: bearings every 30° (and every 15° near the front, sides and rear) at **10, 20, 40 and 60 cm** from the chassis edge, ball resting on the floor.
2. For each bearing/distance: `:learn <deg>` (hold still 2.5 s). Record the printed `seen mask`, per-channel hit %, and pulse period (copy the `COV,` rows from `:cov`).
3. Repeat at the **front ring, rear ring, both sides**, and in the **side gap**: ball exactly beside the chassis between the rings at 1, 2, 3 and 4 cm clearance, then 6 and 10 cm.
4. Vary the ball orientation (rotate it 90° about its axis between runs) and the lighting (competition lamps on/off, windows).
5. Repeat step 2 for two distances with the **wheels spinning (lifted)** to look for motor interference (`I`: `!` flags, `edges`).
6. Per receiver, `:verify <ch>` with the ball touching its face. Record PASS/SUSPECT/FAIL.
7. Compare tilt configurations (0°, 22°, 30°): redo steps 2–3 at 10 and 20 cm only.
8. When the ball period is known, `:irgate auto`, re-run a subset to confirm detection is unchanged.
9. Suggested starting acceptance (engineering judgement, adjust with data): every bearing at 10, 20, 40 cm seen by ≥ 1 trusted channel in ≥ 60 % of its windows; 60 cm documented, not required; no false detections with the ball off for 60 s with motors running.

Record results in a table (bearing, distance, mask, hit %, period) and attach it to the log export (`g`, `:xsel`, `x`). **Do not claim coverage until this table exists.**
