# Hardware experiment: the real ball, the TSOP1738 output and the pulse-pattern gate

Everything under "What the sources say" is read from files. Everything under "Procedure" is **not yet done**: no measurement from the
real ball or receivers exists in this repository, the result tables are blank on purpose, and the gate is **not** validated.

## 1. What the sources say

Ball: <https://github.com/robocup-junior/ir-golf-ball>, commit `28b66b5` (2026-01-08), `fimware/golf_ball/golf_ball.ino` (sic) and the README
waveform image `img/IR_ball_modulation.png`.

| Fact | Source | Consequence |
|---|---|---|
| One burst = 8 × (MOSFET on 12 µs, off 12 µs) = 8 carrier cycles, nominal 24 µs per cycle, 192 µs in total | `loop()`: `for (i<8) { digitalWriteFast(HIGH); delayMicroseconds(12); LOW; delayMicroseconds(12); }` | Carrier is 41.7 kHz if the code overhead is zero. README and image say 40 kHz; the ATtiny runs at 5 MHz (README step 4), so `delayMicroseconds` is approximate. **The real carrier is unmeasured.** |
| Burst repetition "1.2 kHz, 833 µs" | README and image. In the code it is not a timer: the rest of `loop()` (battery ADC read, vibration-pin read, `millis()`) plus `delayMicroseconds(550)` fill the period | 833 µs is a *design target of a software loop*, so the real period differs from ball to ball and is not crystal-locked. Every 1000th pass also runs the ADC averaging and LED logic (about every second), which can make one interval different. Unmeasured. |
| All bursts have the same amplitude | The code switches the MOSFET fully on/off; the documented waveform is a flat 100 mA | A "varying intensity" pattern is **not in the source or the documentation**. Any droop within or between bursts would come from the supply/coil (hardware) and is unmeasured. |
| The ball goes silent by itself | Power-up: 10 × 200 ms blink with the MOSFET off (2 s). **No vibration-pin change for 60 s: 10 blinks, then SLEEP (no IR)**. Wake: ≥ 10 vibration changes within 3 s, then 2 s of blinking. Battery average below ADC 614: 4 s of blinking, off until ≥ 716. | A ball resting on the bench for a minute disappears from the receivers. That is the ball, not the detector. Keep it moving during captures; record when it was last shaken. |

What this implies for a TSOP1738 (nominal 38 kHz), as hypotheses to test, not as findings:

* The receiver should output one active-low pulse per burst it detects, so the output rhythm should be about one pulse per 833 µs.
* The carrier is 40–41.7 kHz, off the receiver's centre frequency: reduced sensitivity, so a shorter range than a 38 kHz source.
* 8 cycles is short. My recollection of the TSOP17xx datasheet is a minimum burst length of about 10 cycles; **check the datasheet of the part you use**. If so, individual bursts can be missed or shortened, more often at long range, and the output then shows intervals of 2× or 3× the period.

## 2. Does `:irgate auto` measure something meaningful?

Yes, it measures; it does not assume 833 µs. The ISR records the start of every active-level pulse in the 3 ms window; the window's period is
(last start − first start) / (pulses − 1), i.e. the **mean start-to-start interval of the output pulses**. `:learn`/`:verify` average that over windows
with ≥ 3 pulses, and `:irgate auto` sets the gate to 0.65–1.35 × that mean. For this ball that is a real feature (the burst repetition interval).

Limits found by reading the code (the mean-interval claims are checked by `t_irchar_gate_functions`; their relevance to the real ball is untested):

1. **It learns whatever was in front of the receiver.** With an interferer present during `:learn` it calibrates to the interferer. `:irgate auto` now warns when the learned period is more than 25 % from 833 µs, and when the longest window period exceeds 1.5× the mean (missed bursts in the sample); it still sets the gate.
2. **A missed burst breaks it.** A window with starts at 0, 833 and 2499 µs has the mean interval 1249 µs, 50 % above 833, so the ±35 % gate classifies it `offband`. If the receiver misses bursts at long range (hypothesis above), the gate throws away genuine ball windows at the edge of the range, exactly where evidence is scarce.
3. A 2-pulse window is judged on one interval; a 1500 µs interferer rhythm is rejected, a source whose spacing happens to fall in 542–1125 µs is not.
4. It ignores the pulse width (about one burst long) and the gaps.
5. The ball period is software-timed, so two balls differ; the gate must be learned per ball, and a second identical ball is indistinguishable from the first.

## 3. What was added (diagnostics only, the detector and all defaults are unchanged)

`:irchar <ch> [seconds 1..20, default 3] [reference_period_us]` parks the scan on one enabled receiver, classifies back-to-back windows with the
**normal** classifier, and prints a report. It needs autonomy off, leaves the receiver's health counters and the ball track alone, and resumes the ring
sweep afterwards. While it runs the other receivers are not scanned (the scan-stall fault is held off). `:learn`/`:verify`/`:irgate` are unchanged,
and `:irgate auto` can use the period of an `:irchar` capture.

Cost: the ISR tests one flag per edge; while capturing it also records the pulse widths/gaps and the first 8 pulse start times of the window. The
cost on the ESP32 is not measured. Memory: about 1 KB of RAM.

| Report line | What it measures | What it decides |
|---|---|---|
| `windows … pre-gate detections … passed gate … rejected by gate … none/noisy/stuck` | how the real classifier sees this source; "pre-gate" = DETECT + OFFBAND | detection probability per window |
| `gate: a..b us is ON; pass rate of pre-gate detections` | share of the detections that today's gate lets through | whether `:irgate auto` costs genuine ball windows (ball) or rejects interference (interferer) |
| `pulse width us min/mean/max` + histogram | receiver output pulse width for the burst | compare with the 192 µs burst; width spread shows how reliably the receiver passes an 8-cycle burst |
| `gap us min/mean/max` + histogram | inactive time between output pulses inside a window | the ball's own gap (about 640 µs expected); the **receiver recovery time cannot be read from this ball**, whose gap is far longer than any recovery time; it needs a source with a controllable gap (not built) |
| `window period` histogram | the exact value today's gate tests | how wide the gate must be |
| `pulse intervals … most common ~X us` + histogram | the rhythm: where intervals between consecutive pulse starts cluster | X is the measured burst period of this ball; peaks at 2×/3× are missed bursts |
| `vs reference R us: near 1x/2x/3x/4x … pulses delivered %` | share of bursts that reached the output (missed burst = 2× interval); "NOMINAL 833" if no reference was given | burst delivery vs distance and angle |
| `gate SHADOW … mean-interval gate passes a/b/c, every-interval-near-a-multiple gate passes d/e/f` | pass counts at ±10/20/35 % for **today's gate** and for the **candidate** (every interval within tolerance of 1×–4× the reference) | the comparison that decides whether the candidate is worth implementing |
| `pulses per window`, `silent runs … longest` | dropout structure at 3 ms granularity | dropout tolerance; consecutive-miss budget |

The candidate gate is **evaluated only**; nothing in the detector uses it. The simulated checks (`t_irchar_*`) only show that the instruments compute what they claim on a synthetic square wave; the
simulated source uses a 190 µs output width that is an assumption.

## 4. Procedure (to be done on the robot)

Safety: wheels off the ground or motors disconnected, autonomy off (`:irchar` refuses otherwise), serial console at 115200. Use one receiver `<ch>` facing the source; `i` shows which receiver sees the ball. Use `:save` only deliberately.

1. **Baseline, no ball** (ball off or removed, room as it will be): `:irchar <ch> 10`. Record the detections and noisy/stuck counts. Repeat for each receiver you plan to rely on.
2. **Ball identity.** Shake the ball (vibration switch), then immediately: ball touching the receiver face, `:irchar <ch> 5`. Record all lines. Note the "most common ~X us"; call it **P**.
3. **Range and angle.** Repeat step 2 at 10, 30, 60, 100 cm on-axis and, at one distance, with the ball at ±30°, ±60° of the axis. Shake the ball before each run (it sleeps after 60 s still). Use `:irchar <ch> 5 <P>` so every run is judged against the same reference. Fix the distance with a ruler; do not move the ball during the run.
4. **Second ball**, if you have one: step 2 and 3 at two distances. Record P for each ball.
5. **Today's gate on the ball.** `:irgate auto` (from the step-2 capture), then repeat step 3 at the far distances. The line `gate: … pass rate` is the fraction of genuine ball windows today's gate keeps. `:irgate off` afterwards.
6. **Interference, one source at a time**, in front of the same receiver, `:irchar <ch> 10 <P>`: sunlight through a window, incandescent lamp, LED lamps (also a dimmable one), fluorescent/tube lights, a TV remote and a phone IR remote app, another robot, a second ball, the arena lighting. With a wheel test running (`:mtest <wheel>` then `:confirm`, wheels lifted) start the capture straight afterwards to see motor/PWM noise. Record whether the source produced any pre-gate detections at all.
7. **Dropouts.** With the ball at a fixed distance and a hand moved in and out of the beam, `:irchar <ch> 10 <P>` and read `silent runs` and `longest`.
8. **Sleep check.** Leave the ball untouched for 70 s and capture: the source should vanish. This confirms the ball behaves as its firmware says.
9. Optional, to measure receiver recovery: a reference emitter with adjustable burst and gap (function generator or a second microcontroller driving an IR LED). It is not part of this repository.

Results table (fill from the console; nothing is entered here):

| Run | ch | distance / angle | windows | pre-gate detections | pass rate of today's gate | width min/mean/max | gap min/mean/max | P (µs) | delivered % | shadow mean-gate ±10/20/35 | shadow multiple-gate ±10/20/35 | longest silent run |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| (blank) | | | | | | | | | | | | |

## 5. Deciding about the gate

Proposed criteria. They are my proposals, not validated thresholds.

* **Is a gate needed at all?** If no interference source in step 6 produces pre-gate detections, the gate gains nothing; leave it off.
* **Today's gate is safe to keep** only if its pass rate on the genuine ball stays ≥ 99 % at every distance and angle you intend to use, including the far runs, and the delivered % is 100 % there. If delivered % drops below 100 % at range, today's gate will cost range.
* **The multiple-gate candidate is worth implementing** only if (a) it passes ≥ 99 % of the ball's pre-gate detections at every distance/angle at some tolerance, (b) every interference source that produces pre-gate detections is passed ≤ 5 % at that tolerance, and (c) it passes strictly more genuine windows than today's gate where bursts are missed. A ±20 % multiple gate accepts about 40 % of arbitrary intervals, a ±10 % gate about 20 %; with 3–4 pulses per window the false pass rate multiplies per interval, so judge the 2-pulse windows separately.
* **No pulse-pattern gate helps** against a source whose rhythm matches the ball (another identical ball, an interferer at about 833 µs). That needs a different measure (direction, persistence), not a gate.
* Decide the tolerance from the measured spread of P (histogram) over balls and over the 1000-pass ADC interval, not from 35 %.

Only after those measurements: implement the candidate as an optional config switch (default off), unit-test it, re-run the real ball tests with it on, and only then recommend it.
