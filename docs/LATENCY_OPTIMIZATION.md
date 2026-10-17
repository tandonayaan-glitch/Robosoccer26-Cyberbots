# IR detector: latency and reliability (simulation only)

> **Every number in this document comes from the host simulation** (`test/bench.h`, virtual clock, 50 µs of virtual time per
> control-loop iteration, simulated ball and receivers, the real firmware code in the loop). **None is a measurement of the robot.**
> The receiver's reaction and recovery time, the real ball's modulation and carrier, optical effects, flash stalls, the real loop
> speed and the wheels' real response are not modelled. "Motor response" below means the PWM value the firmware *commands*, not wheel
> speed (the robot has no encoders). Nothing here shows competition readiness; see §9 for what still has to be measured.

Branch `Robocup-2026-Ayaan`. Starting point: commit `89c3503` ("fast IR estimator, engine 1"). Raw outputs: `docs/bench/`. Regenerate:
`tools/bench_all.sh` (about an hour), single suites: `tools/bench_summary.py`, `--bench suite=<name>` (README).

## 1. Starting point: was `89c3503` what it claimed?

* The checkout was clean, `HEAD == origin/Robocup-2026-Ayaan == 89c3503`. The reported figures **reproduce**: the original 96-trial harness
  (`docs/bench/v1_engine1_89c3503.txt`) gives acquisition 127.6 / 57.4 / 49.9 ms (1 / 2 / 3 receivers), loss 140 ms, translation ≤ 10 % at
  285 ms; the report said 128 / 57 / 50 / 141 / 285.
* The reported figures were **medians of the best case**. A harder, seeded benchmark (§2) showed that the "optimised" engine was materially
  less reliable than the legacy one in places the old tables did not look at (§3). So the previous result was *reproducible but not
  adequate*: it traded reliability for speed, which the task forbids.

## 2. Method

| Item | What |
|---|---|
| Time | Virtual microseconds; one `loop()` per 50 µs; interrupts fire at every simulated edge; the firmware reads `micros()`/`millis()` from the mock clock |
| Ball | 1.2 kHz, 42 % duty (833 µs period) per receiver, random pulse phase per trial. **The real ball's modulation is unverified**; §7.7 sweeps it |
| Injection | The ball is switched on/off at instants spread uniformly over two ring cycles (240 phases per latency distribution; 48 for transitions, tables of bursts and dropouts; 24 for fault cases) and on a 100 µs grid over one cycle for the "onset at a mux channel change" check |
| Randomness | `rand()` seeded with `seed*7919 + scenario`; seeds 12345 / 22345 / 32345; every suite can be run alone and reproduces exactly |
| Misses | Independent, per visit: a window on a ball receiver is blanked with probability p. Real misses are probably correlated; that is an assumption of the model, not a fact |
| Dropouts / stalls | Total silence of the ball for a given time; the control loop not running for a given time (interrupts still fire) |
| Faults | Receiver dead, stuck active, neighbour stuck active, far receiver noisy, far receiver stuck active, neighbour disabled |
| Soak | 600 s × 3 seeds with ground truth: ball segments (0.6-6 s, random bearing, 1-3 receivers, 0-50 % misses, jumps), dropouts 20-300 ms, bursts of 10-250 ms in five shapes (1 receiver, 2 / 3 neighbours, whole ring coherent / random phase) |
| Statistics | n, median, p95, p99 (only when n ≥ 100), max for every latency distribution; counts out of trials for rates; rates per minute |
| Burst acceptance | Deterministic profile (`suite=imm`): 96 phases per duration (20-250 ms, step 10), no ball; **E[accept]** = acceptance probability of a burst whose duration is uniform in 20-250 ms. Sampling error about ±0.02 |
| Stages | onset → first window closed on a ball receiver → first candidate evidence (DETECT) → confirmed → direction within 20° → navigation command → first PWM → PWM ≥ `minpwm`; removal → age of newest evidence → declared lost → translation request cleared → PWM < 90 % → PWM ≤ 10 % of steady |
| Scan metrics | per receiver: longest time between two window openings (revisit), longest late window; ring cycle |
| CPU | host wall-clock per call (x86), **only a relative comparison between engines; not ESP32 time** |

## 3. Root causes found in `89c3503` (engine 1)

Found by the seeded matrix of §2 (raw: `docs/bench/v2_engine1_89c3503_seed12345.txt` against `v2_engine0_legacy_seed12345.txt`), each confirmed with a debug trace, fixed in engine 2 (§5):

| # | Root cause | Evidence (engine 1 vs legacy engine) |
|---|---|---|
| 1 | Acquisition needed *consecutive* hits on one receiver; any miss reset the counter | 1 receiver with 50 % misses: 52 of 240 trials never acquired within 1.5 s (legacy 16) |
| 2 | The grace period was a fixed 120 ms, or (in the intermediate version) scaled by a quality estimate that kept decaying during the outage and so lengthened its own grace (loss 318 ms instead of 120 ms) | false losses per minute at 30 / 50 / 70 % misses, 1 receiver: 5.0 / 10.5 / 9.0 (legacy 0 / 3.0 / 13.0); 2 receivers: 0 / 3.0 / 10.5 (legacy 0 / 0 / 2.5); 3 receivers at 70 %: 5.0 (legacy 0) |
| 3 | Dropout tolerance was the grace minus one visit, whatever the signal | steady track survives 120 ms (legacy ≥ 300 ms); a flickering track is lost even in 20 ms gaps (12/48 at 50 % misses, 3/48 at 30 %) |
| 4 | A stalled control loop closed the window late, the track aged by the stall | weak 1-receiver ball, stalls of 50 / 100 / 200 ms every ~0.7 s: 15 / 19 / 19 false losses in 60 s (legacy 0 / 1 / 0) |
| 5 | No fast path after a loss: a returning ball needed the whole acquisition again | re-acquisition 126 ms (1 receiver) after every gap longer than the grace |
| 6 | Burst rejection had been traded for speed | accepted bursts: 1 receiver 120 ms: 13/48 (legacy 0), 2 neighbours 60 ms: 23/48 (legacy 0), 3 neighbours 40 ms: 9/48 (legacy 0) |
| 7 | After a loss the search rotation was commanded at once, so a strafe → spin reversed two wheels through the dead-time pause | translation ≤ 10 % of steady only 144 ms after the declaration (285 vs 141 ms) |
| 8 | Method: the old tables were single-trial or 96-trial medians of strong signals only | no miss, dropout, stall, burst-shape, fault or soak scenario was in them |

The **legacy engine** has its own weakness the old tables hid: ring-wide bursts of 60 ms are accepted as a ball (11/48 coherent, 9/48 random phase). Engine 2 rejects every ring burst (0/48 up to 500 ms).
Further defects found while developing engine 2 (an arc of "fresh" receivers left after a broadband burst, a re-acquisition test that compared the evidence with itself, wiping the evidence of a second
ball on loss, a focus rule that depended on the ring time in milliseconds, ...) are listed in `VERIFICATION_REPORT.md` §5.

## 4. What "36.12 ms scan cycle" means

* `cycleUs` is the time between two consecutive wraps of the **ring sweep** = 12 × (3000 µs window + 10 µs settle) = 36.12 ms, measured
  with `micros()` at the wrap. It is exact in the simulation because windows are closed on the first loop iteration after 3000 µs.
  On the robot every window closes late by up to one loop period, so the real cycle is longer; `i`/`h` print the measured value.
* While the ring is idle it is also the **revisit interval of every receiver**. That stopped being true with scheduling that visits
  some receivers more often, so the firmware now measures, per receiver, the longest time between two window openings
  (`revisitMaxUs[]`) and the longest late window (`winOverrunMaxUs`), and prints both (`i`, `h`). Under stalls the longest wait of a receiver
  equals the stall plus a cycle (stall 100 ms → 136 ms, 200 ms → 236 ms); in the focus mode a receiver outside the focus set waits at most two ring cycles
  (72 ms, `t_e2_focus_scheduling`). The metric was not wrong, it was incomplete; it is now complemented, not renamed.
* The 36 ms is also the main floor of every latency below: a ball that appears is first *sampled* after 0-36 ms (median 18, measured
  18.1 ms / p95 34.3 ms), because one multiplexer observes one receiver at a time. This is architecture, not software.

## 5. Engine 2 (default)

`engine 0` is the legacy estimator and stays selectable as the fallback; `:set engine 2` (default). Engine 1 was removed: engine 2 is equal or
better in every row of the matrix (its results stay in `docs/bench/v2_engine1_89c3503_seed12345.txt`). Code: `irscan.h`, `nav.h`, `config.h`.

1. **Evidence.** Per receiver a leaky counter *S*: +1 per detecting window, −0.1 per miss, forgotten after `epidle` (250 ms) without a hit.
   An intermittent signal accumulates; consecutive hits are not required.
2. **Acquisition (time-qualified).** The ball is accepted when (a) the summed evidence of neighbouring receivers within `gatedeg` ≥ 1
   (`acqstreak` = 4 hits per unit), (b) it has lasted `acqspan1` ms (one receiver) / `acqspan2` ms (two or more), (c) **every other
   receiver has been looked at since the onset** (a broadband onset hits receivers in scan order and looks local until the ring is revisited),
   (d) neither the fresh receivers nor the open episodes of the last ring cycle contradict each other (resultant length ≥ 0.5). Burst immunity is therefore
   a *time*, independent of how often a receiver is visited. Alternatives measured and rejected: consecutive-streak rules (engine 1), a cycle-count
   rule (legacy), a "busy environment" penalty that stretches the spans after recent false starts (burst acceptance −3 points, genuine acquisition +10-50 ms in the soak; not adopted).
3. **Fast re-acquisition.** For `reacq` (300 ms) after a loss, evidence within 45° of the lost bearing is accepted after two hits (`t_e2_reacquisition`).
4. **Focus scheduling.** A candidate that has just missed, a young track (miss budget > 6) or a track with ≥ 5 % measured misses gets every second window on
   its (up to three) receivers; the rest of the ring continues at half speed. A steady track gets none (ring revisit unchanged).
5. **Loss = visit statistics.** The grace is at least `losthold` (150 ms, steady signal) or `flickerhold` (290 ms) for a signal that was already missing ≥ 25 % of visits
   (linear between 5 % and 25 %), at least *M × revisit interval* where *M* is the shortest run of misses whose probability under the
   Beta-predictive hit rate of the strongest supporting receiver is below 10^-`losseps` (4), and at most `losthold_max` (300 ms). It is **fixed at the
   last good evidence**: an outage cannot lengthen its own grace. A young track is judged cautiously (large *M*): grace up to 300 ms during the first 1-2 s.
6. **Stall credit.** A window closed so late that it was discarded says nothing about the ball: that time (≤ 500 ms) is credited back to the track.
7. **Direction.** Weighted unit-vector sum of receivers hit within `dirwindows` ring cycles (linear freshness, weight 0.25 + 0.75 × hit rate), τ = 40 ms smoothing, evidence gated to ±100° of the track
   (70° excluded the edge receivers of a five-receiver cluster and biased the bearing: caught by `t_nav_sweep_and_multi`). Nothing moves the bearing once evidence stops
   (`t_e2_stale_not_followed`).
8. **Navigation/motors (only what the ball-loss path needs).** (a) The control tick runs immediately when the ball becomes valid/invalid instead of waiting for the next 10 ms tick and the
   state change is acted on in the same tick; (b) the approach speed fades towards the slow speed between 30 % and 60 % of the grace period (step at 50 % before); (c) after a loss the wheels are brought
   to rest *before* the search rotation starts (no strafe → spin reversal through the dead-time pause); (d) `minpwm` start jump (already in `89c3503`). The ramp rates (`accel`, `decel`),
   limits and safety logic are unchanged. `motors.h` has no change relative to `89c3503`.

## 6. Results: latency stages

Columns: **legacy** = legacy estimator at `89c3503` (old nav), **89c3503** = what was reported as optimised (engine 1), **engine 2** = this work, default `balanced` profile.
Each cell: median / p95 / p99 / max in ms, n = 240 phases; brackets = range over the three seeds where it differs by ≥ 1 ms.

| Stage (ms from the event) | legacy (89c3503 code): median / p95 / p99 / max | 89c3503 default (engine 1): median / p95 / p99 / max | engine 2 default (3 seeds): median / p95 / p99 / max |
|---|---|---|---|
| first window closed on a ball receiver, 1 receiver | 19 / 34 / 36 / 36 | 17 / 33 / 35 / 36 | 18 / 34 / 36 / 36 |
| first candidate evidence, 1 receiver | 19 / 35 / 36 / 37 | 18 / 34 / 36 / 37 | 19 / 35 / 36 / 37 |
| **ball confirmed**, 1 receiver | 224 / 240 / 241 / 242 | 126 / 143 / 145 / 146 | 160 / 177 / 178 / 178 |
| navigation command, 1 receiver | 239 / 254 / 257 / 259 | 142 / 160 / 162 / 162 | 160 / 177 / 178 / 178 |
| wheels reach start PWM, 1 receiver | 239 / 254 / 257 / 259 | 142 / 160 / 162 / 162 | 160 / 177 / 178 / 178 |
| first window closed on a ball receiver, 2 receivers | 15 / 31 / 33 / 33 | 14 / 31 / 33 / 33 | 16 / 32 / 33 / 33 |
| first candidate evidence, 2 receivers | 16 / 31 / 33 / 34 | 15 / 31 / 33 / 34 | 17 / 33 / 34 / 34 |
| **ball confirmed**, 2 receivers | 151 / 167 / 169 / 170 | 57 / 72 / 73 / 74 | 90 / 105 / 106 / 106 |
| navigation command, 2 receivers | 166 / 184 / 186 / 188 | 72 / 88 / 90 / 91 | 90 / 105 / 106 / 106 |
| wheels reach start PWM, 2 receivers | 166 / 184 / 186 / 188 | 72 / 88 / 90 / 91 | 90 / 105 / 106 / 106 |
| first window closed on a ball receiver, 3 receivers | 12 / 28 / 30 / 30 | 14 / 28 / 30 / 30 | 13 / 29 / 30 / 30 |
| first candidate evidence, 3 receivers | 13 / 29 / 31 / 31 | 16 / 30 / 31 / 31 | 15 / 30 / 31 / 31 |
| **ball confirmed**, 3 receivers | 119 / 134 / 136 / 136 | 52 / 66 / 67 / 68 | 87 / 102 / 103 / 104 |
| navigation command, 3 receivers | 134 / 151 / 153 / 156 | 66 / 82 / 85 / 86 | 87 / 102 / 103 / 104 |
| wheels reach start PWM, 3 receivers | 134 / 151 / 153 / 156 | 66 / 82 / 85 / 86 | 87 / 102 / 103 / 104 |
| **ball declared lost**, strong track, 1 receiver | 545 / 561 / 562 / 563 | 136 / 153 / 154 / 155 | 134 / 149 / 151 / 152 |
| translation request cleared, strong track, 1 receiver | 560 / 577 / 580 / 580 | 152 / 168 / 170 / 173 | 134 / 149 / 151 / 152 |
| translation PWM first < 90 %, strong track, 1 receiver | 60 / 77 / 80 / 80 | 34 / 51 / 55 / 56 | 44 / 62 / 65 / 66 |
| translation PWM <= 10 %, strong track, 1 receiver | 680 / 697 / 700 / 700 | 272 / 288 / 290 / 293 | 184 / 200 / 203 [201-203] / 210 [208-210] |
| **ball declared lost**, strong track, 2 receivers | 548 / 563 / 565 / 565 | 138 / 154 / 155 / 155 | 134 / 150 / 152 / 152 |
| translation request cleared, strong track, 2 receivers | 563 / 580 / 582 / 583 | 153 / 171 / 173 / 175 | 134 / 150 / 152 / 152 |
| translation PWM first < 90 %, strong track, 2 receivers | 205 / 219 / 221 / 223 | 36 / 52 / 56 / 57 | 60 / 71 / 72 / 75 |
| translation PWM <= 10 %, strong track, 2 receivers | 693 / 710 / 712 / 713 | 283 / 301 / 303 / 305 | 189 / 207 [203-207] / 211 / 211 |
| **ball declared lost**, strong track, 3 receivers | 549 / 562 / 564 / 566 | 142 / 154 / 154 / 155 | 137 / 151 / 152 / 152 |
| translation request cleared, strong track, 3 receivers | 564 / 578 / 582 / 583 | 157 / 170 / 173 / 174 | 137 / 151 / 152 / 152 |
| translation PWM first < 90 %, strong track, 3 receivers | 242 / 256 / 258 / 258 | 40 / 53 / 54 / 55 | 48 / 63 / 65 / 65 |
| translation PWM <= 10 %, strong track, 3 receivers | 684 / 708 / 712 / 713 | 277 / 296 / 302 / 304 | 191 / 209 / 211 / 212 |

| Weak-track loss (ms after removal) | legacy (89c3503 code): median / p95 / p99 / max | 89c3503 default (engine 1): median / p95 / p99 / max | engine 2 default (3 seeds): median / p95 / p99 / max |
|---|---|---|---|
| declared lost, 30 % misses, 1 rx | 536 / 560 / 561 / 562 | 159 / 269 / 320 / 326 | 285 [285-286] / 291 / 292 / 292 |
| declared lost, 30 % misses, 2 rx | 546 / 562 / 564 / 565 | 137 / 156 / 198 / 204 | 284 [284-286] / 291 / 292 / 292 |
| declared lost, 50 % misses, 1 rx | 527 / 557 / 560 / 561 | 200 / 321 / n/a / 332 | 285 [285-286] / 291 / 292 / 292 |
| declared lost, 50 % misses, 2 rx | 540 / 562 / 564 / 564 | 144 / 230 / 256 / 258 | 287 / 291 / 292 / 295 [292-295] |
| declared lost, 70 % misses, 1 rx | 472 / 550 / n/a / 562 | 232 / 314 / n/a / 328 | 280 [279-281] / 293 [293-297] / 300 / 302 |
| declared lost, 70 % misses, 2 rx | 520 / 560 / 563 / 564 | 190 / 326 / n/a / 333 | 286 [285-288] / 301 / 301 / 301 |

| Direction update (ms after the jump) | legacy (89c3503 code): median / p95 / p99 / max | 89c3503 default (engine 1): median / p95 / p99 / max | engine 2 default (3 seeds): median / p95 / p99 / max |
|---|---|---|---|
| adjacent sector 90 -> 120 | 221 / 237 / 238 / 238 | 90 / 104 / 106 / 106 | 74 [74-74] / 86 [86-87] / 87 [87-88] / 88 |
| adjacent sector 90 -> 60 | 224 / 240 / 242 / 242 | 93 / 108 / 110 / 110 | 72 / 87 / 88 / 89 |
| opposite sector 90 -> 270 | 207 / 222 / 223 / 224 | 113 / 128 / 128 / 128 | 160 / 177 / 178 / 178 |
| front seam 330 -> 0 | 200 / 216 / 218 / 218 | 90 / 104 / 105 / 106 | 74 / 86 / 87 / 88 |
| front seam 0 -> 330 | 199 / 216 / 217 / 217 | 93 / 108 / 110 / 110 | 72 / 87 / 89 / 89 |
| front seam 330 -> 30 | 268 / 282 / 283 / 284 | 113 / 125 / 126 / 126 | 98 / 111 / 112 / 113 |
| rear seam 150 -> 180 | 214 / 231 / 232 / 232 | 90 / 104 / 106 / 106 | 74 / 86 / 87 / 88 |
| rear seam 180 -> 210 | 212 / 228 / 229 / 230 | 90 / 104 / 105 / 106 | 74 / 86 [86-87] / 87 / 88 |
| rear seam 150 -> 210 | 254 / 270 / 272 / 272 | 113 / 125 / 126 / 127 | 98 / 112 / 113 / 113 |

### 6.1 The two profiles that matter (`:irprofile`)

| Profile | `acqspan1/2` | acquisition 1 / 2 / 3 rx, median / p95 (ms) | E[accept] 1 rx / 2 nb / 3 nb | Comment |
|---|---|---|---|---|
| legacy engine | – | 224/240 · 152/168 · 118/134 | 0.38 / 0.71 / 0.83 | reference |
| `fast` | 96 / 30 | 127/144 · 58/72 · 52/66 | 0.534 / 0.833 / 0.871 | meets the latency targets; accepts shorter bursts |
| **`balanced` (default)** | 140 / 70 | 160/177 · 90/105 · 87/102 | 0.393 / 0.701 / 0.716 | burst acceptance equal to the legacy engine |
| `safe` | 180 / 100 | 200/216 · 120/135 · 116/132 | 0.221 / 0.578 / 0.588 | fewer accepted bursts, slower |

The trade-off is physical: confirming a ball takes the burst-immunity time plus the 0-36 ms wait for the first sample, whatever the software does. A
single receiver cannot be confirmed faster than its immunity time; two neighbours cannot be confirmed faster than one ring revisit (36 ms) + the wait without accepting broadband onsets.
To meet *both* the latency targets and the legacy burst rejection is therefore impossible; the default keeps the rejection (the task: reliability is a hard requirement).

### 6.2 Where the remaining delay after a loss comes from (commanded PWM)

Strong track, removal → …: newest evidence is 18 ms old on average (it is a scan, not a ball switch) → declared lost 134 ms (grace 150 ms measured from the last evidence) → translation request cleared in the same
control tick → PWM below 90 % of steady at 44 ms (the speed fades before the declaration) → **PWM ≤ 10 % of steady at 184 ms**. The last 50 ms are the deceleration ramp (`decel` = 1200 PWM/s from a faded
speed). Raising `decel` in the simulation: 600 → 284 ms, 1200 → 184, 2400 → 158, 4000 → 148 (`docs/bench/v3_decel_sensitivity.txt`). Whether that is safe
for the real robot (wheel slip, current, tipping) was not tested, so the default is unchanged.

## 7. Results: reliability

All numbers: legacy engine → engine 2 (default). Seed 12345 unless stated; the other seeds are in `docs/bench/`.

### 7.1 Random misses (the signal flickers)

Independent per-visit misses, 120 s per cell. Legacy → engine 2.

| per-visit misses | 10 % | 30 % | 50 % | 70 % |
|---|---|---|---|---|
| 1 receiver: false losses/min, legacy → engine 2 | 0.00 → 0.00 | 0.00 → 0.00 | 3.00 → 0.00 | 13.00 → 0.00 |
| 2 receivers: false losses/min, legacy → engine 2 | 0.00 → 0.00 | 0.00 → 0.00 | 0.00 → 0.00 | 2.50 → 0.00 |
| 3 receivers: false losses/min, legacy → engine 2 | 0.00 → 0.00 | 0.00 → 0.00 | 0.00 → 0.00 | 0.00 → 0.00 |

The fraction of ball time reported valid is 1.000 in every engine-2 cell (legacy: 0.945 at 50 %, 0.425 at 70 % with one receiver).

### 7.2 Dropouts of a tracked ball (total silence for the given gap, then the ball returns)

| gap (ms) | 20 | 40 | 60 | 80 | 100 | 120 | 150 | 200 | 250 | 300 |
|---|---|---|---|---|---|---|---|---|---|---|
| strong, 1 receiver: lost of 48, legacy → engine 2 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 7 | 0 → 46 | 0 → 48 | 0 → 48 | 0 → 48 |
| strong, 2 receivers: lost of 48, legacy → engine 2 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 2 | 0 → 44 | 0 → 48 | 0 → 48 | 0 → 48 |
| weak 30 % misses, 1 rx: lost of 48, legacy → engine 2 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 1 | 0 → 2 | 0 → 48 |
| weak 50 % misses, 1 rx: lost of 48, legacy → engine 2 | 0 → 0 | 1 → 0 | 2 → 0 | 0 → 0 | 4 → 0 | 1 → 0 | 2 → 0 | 2 → 0 | 1 → 0 | 6 → 48 |
| weak 50 % misses, 2 rx: lost of 48, legacy → engine 2 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 48 |

Median re-acquisition time after the gap for a strong 1-receiver track, engine 2 (gaps ≥ 120 ms that ended the track): 64, 52, 54, 55, 54 ms (legacy: the track is never lost, so none).

A steady (strong) track survives gaps up to about 100 ms; a flickering one up to about 250 ms. Beyond that the ball is declared lost *by design* and found again within about 55 ms (§5.3). Surviving a longer silence in a steady track would raise the loss latency of a genuinely vanished ball above 175 ms: the two requirements cannot both hold for a signal that gives no hint of which case it is.

### 7.3 Ball genuinely gone after a weak track (removal → declared lost)

| signal before removal | legacy: median / p95 / max | engine 2: median / p95 / max |
|---|---|---|
| 30 % misses, 1 receiver | 536 / 560 / 562 (n=120) | 285 / 291 / 292 (n=120) |
| 30 % misses, 2 receivers | 546 / 564 / 566 (n=120) | 284 / 291 / 292 (n=120) |
| 50 % misses, 1 receiver | 516 / 557 / 562 (n=117) | 285 / 291 / 292 (n=120) |
| 50 % misses, 2 receivers | 544 / 563 / 565 (n=120) | 287 / 291 / 295 (n=120) |
| 70 % misses, 1 receiver | 462 / 550 / 560 (n=43) | 280 / 293 / 302 (n=120) |
| 70 % misses, 2 receivers | 514 / 561 / 566 (n=118) | 286 / 301 / 301 (n=120) |

The weak tracks are longer on purpose (§5.5). Engine 2 also *tracks* the 70 %-miss ball in all 120 trials; the legacy engine had a track at removal in 43 / 118 trials (1 / 2 receivers).

### 7.4 Ball-like bursts with no ball present (48 phases per cell)

| burst shape | legacy: 50 % accepted at (ms) | engine 2 | legacy: accepted of 48 at 60 / 100 / 140 / 180 ms | engine 2 |
|---|---|---|---|---|
| 1 receiver | 160 | 160 | 0 / 0 / 0 / 48 | 0 / 0 / 0 / 48 |
| 2 neighbours | 100 | 100 | 0 / 41 / 48 / 48 | 0 / 40 / 48 / 48 |
| 3 neighbours | 80 | 100 | 21 / 48 / 48 / 48 | 0 / 41 / 48 / 48 |
| ring coherent | never (≤ 500) | never (≤ 500) | 11 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| ring random | never (≤ 500) | never (≤ 500) | 9 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |

Expected acceptance of a burst whose duration is uniform in 20-250 ms (`suite=imm`, 96 phases per duration):

```
# deterministic burst-acceptance profile (no ball present, 96 phases per cell)
## engine=0 losthold=500 losthold_max=500 flickerhold=500
  1 receiver   E[accept | U(20,250)] = 0.383   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:0 90:0 100:0 110:0 120:0 130:0 140:0 150:21 160:37 170:67 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
  2 neighbours E[accept | U(20,250)] = 0.710   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:39 90:56 100:81 110:96 120:96 130:96 140:96 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
  3 neighbours E[accept | U(20,250)] = 0.833   (durations:accepted/96) 20:0 30:0 40:0 50:23 60:51 70:85 80:96 90:96 100:96 110:96 120:96 130:96 140:96 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
## engine=2 acqspan1=96 acqspan2=30
  1 receiver   E[accept | U(20,250)] = 0.534   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:0 90:0 100:0 110:1 120:35 130:57 140:77 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
  2 neighbours E[accept | U(20,250)] = 0.833   (durations:accepted/96) 20:0 30:0 40:1 50:26 60:48 70:84 80:96 90:96 100:96 110:96 120:96 130:96 140:96 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
  3 neighbours E[accept | U(20,250)] = 0.871   (durations:accepted/96) 20:0 30:0 40:24 50:49 60:75 70:96 80:96 90:96 100:96 110:96 120:96 130:96 140:96 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
## engine=2
  1 receiver   E[accept | U(20,250)] = 0.393   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:0 90:0 100:0 110:0 120:0 130:0 140:0 150:24 160:52 170:72 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
  2 neighbours E[accept | U(20,250)] = 0.701   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:23 90:54 100:79 110:96 120:96 130:96 140:96 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
  3 neighbours E[accept | U(20,250)] = 0.716   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:43 90:57 100:88 110:96 120:96 130:96 140:96 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
## engine=2 acqspan1=180 acqspan2=100
  1 receiver   E[accept | U(20,250)] = 0.221   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:0 90:0 100:0 110:0 120:0 130:0 140:0 150:0 160:0 170:0 180:0 190:26 200:47 210:78 220:96 230:96 240:96 250:96
  2 neighbours E[accept | U(20,250)] = 0.578   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:0 90:0 100:0 110:39 120:52 130:82 140:96 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
  3 neighbours E[accept | U(20,250)] = 0.588   (durations:accepted/96) 20:0 30:0 40:0 50:0 60:0 70:0 80:0 90:0 100:0 110:37 120:66 130:91 140:96 150:96 160:96 170:96 180:96 190:96 200:96 210:96 220:96 230:96 240:96 250:96
```
Engine 2 matches the legacy engine for one receiver and two neighbours and is better for three neighbours; it rejects all ring-wide bursts (the legacy engine accepts 60 ms ring bursts).

### 7.5 Interference while a ball is tracked, and a distractor after the ball is gone

| interference while tracking a 2-receiver ball | legacy (40 / 100 / 250 ms bursts): worst bearing error p95 °, false losses of 48 | engine 2 |
|---|---|---|
| opposite receiver | 2.2° / 0 · 3.5° / 0 · 10.2° / 0 | 0.1° / 0 · 0.1° / 0 · 0.1° / 0 |
| receiver next to ball | 4.8° / 0 · 6.8° / 0 · 12.8° / 0 | 10.8° / 0 · 12.6° / 0 · 14.4° / 0 |
| rest of ring | 13.8° / 0 · 13.9° / 0 · 23.6° / 0 | 15.2° / 0 · 18.5° / 0 · 15.9° / 0 |
| ball gone, steady source on the opposite receiver: old track abandoned after | 207 / 238 ms (median / p95) | 134 / 150 ms (median / p95) |
| ball gone, steady source on a neighbouring receiver: old track abandoned after | 263 / 279 ms (median / p95) | 95 / 109 ms (median / p95) |

With a burst on the receiver next to the ball engine 2's bearing is pulled further (10-14°, against 5-13° for legacy) while the burst lasts; no track is lost in either engine.

### 7.6 Receiver faults (ball on receivers 2-4; 'late' = diagnosed after 3 s)

| fault | legacy: acquire p95 / loss p95 (ms) | engine 2: acquire p95 / loss p95 | false acquisitions in 60 s (both) | false losses/min at 30 % misses (legacy / engine 2) |
|---|---|---|---|---|
| none | 133 / 564 | 103 / 150 | 0 / 0 | 0 / 0 |
| ball receiver dead (stuck inactive) | 164 / 563 | 100 / 150 | 0 / 0 | 0 / 0 |
| ball receiver stuck active | 163 / 564 | 97 / 150 | 0 / 0 | 0 / 0 |
| neighbour stuck active | 133 / 563 | 100 / 150 | 0 / 0 | 0 / 0 |
| far receiver noisy | 133 / 561 | 100 / 150 | 0 / 0 | 0 / 0 |
| far receiver stuck active | 136 / 563 | 103 / 150 | 0 / 0 | 0 / 0 |
| neighbour disabled | 121 / 558 | 99 / 150 | 0 / 0 | 0 / 0 |

### 7.7 Scheduler stalls (the control loop does not run; interrupts do)

| stall (every ~0.7 s, 60 s) | legacy: strong ball / weak 30 % ball — false losses, invalid windows, longest receiver wait ms | engine 2 |
|---|---|---|
| 10 ms | 0, 88, 46 / 0, 81, 46 | 0, 88, 46 / 0, 86, 82 |
| 20 ms | 0, 85, 56 / 0, 80, 56 | 0, 85, 56 / 0, 81, 92 |
| 50 ms | 0, 80, 86 / 0, 78, 86 | 0, 80, 86 / 0, 82, 122 |
| 100 ms | 0, 73, 135 / 1, 73, 136 | 0, 78, 136 / 0, 72, 172 |
| 200 ms | 0, 69, 236 / 0, 69, 236 | 0, 67, 236 / 0, 66, 272 |

The stalled windows are discarded (counted, never interpreted); the longest wait of a receiver is the stall plus one cycle (the measurement the old 'scan cycle' lacked).

### 7.8 Mixed soak with ground truth (3 files × 3 sub-seeds × 600 s)

| soak (600 s × 3 sub-seeds per file) | legacy | engine 2 |
|---|---|---|
| runs | 9 | 9 |
| ball segments / segments never acquired in ≥ 0.9 s | 1057 / 136 | 1076 / 30 |
| eligible bursts that became a phantom track | 41 % (340 of 839) | 44 % (497 of 1129) |
| phantom acquisitions per minute (count) | 3.8 | 5.3 |
| time spent tracking a phantom, s per minute | 6.4 | 2.7 |
| false losses per minute (ball present, no dropout in progress) | 1.13 | 0.59 |
| losses during injected dropouts (20-300 ms) | 38 | 316 |
| share of ball time reported valid | 92.7 % | 94.7 % |
| mean over runs of the median 'segment start → valid' (ms) | 219 | 159 |
| mean over runs of the median 'segment end → declared lost' (ms) | 540 | 225 |

Engine 2 produces **more** phantom acquisitions than the legacy engine in this busy mix (5.3 against 3.8 per minute, 44 % against 41 % of eligible bursts) but spends **less than half** the time on them (2.7 against 6.4 s per minute) because a phantom is dropped after 150-300 ms instead of 500 ms. It also loses the ball in **many more injected dropouts** (316 against 38): every silence longer than the grace of a steady track (§7.2) ends the track, and the ball is found again within about 55 ms. The per-shape tables of §7.4, measured on a quiet system, are the clean immunity comparison; the soak mixes everything and its eligibility depends on how long each engine keeps a track.

### 7.9 Ball moving around the robot (ball position updated every 10 ms)

bearing error against the true bearing; 'lag' is the signed mean error. '1-rx view' = only one receiver sees the ball at a time and there are blind arcs between receivers (a coverage-gap model: the long grace of the legacy engine coasts through them, engine 2 re-acquires after about 55 ms).

| ball speed, receivers see | legacy: mean / p95 error °, lag °, losses/rev, valid | engine 2 |
|---|---|---|
| 30 °/s, 0 % misses, 1-rx view | 8.8 / 20.5, -8.3, 0, 1.000 | 6.0 / 13.2, -3.0, 12, 0.747 |
| 30 °/s, 0 % misses, 2-rx view | 6.2 / 10.8, -6.2, 0, 1.000 | 3.9 / 9.6, -2.7, 0, 1.000 |
| 30 °/s, 30 % misses, 1-rx view | 9.0 / 21.0, -8.6, 0, 1.000 | 7.2 / 16.2, -5.0, 12, 0.817 |
| 30 °/s, 30 % misses, 2-rx view | 5.9 / 11.4, -5.9, 0, 1.000 | 4.2 / 10.8, -3.1, 0, 1.000 |
| 90 °/s, 0 % misses, 1-rx view | 18.6 / 25.4, -18.6, 0, 1.000 | 9.3 / 21.0, -7.6, 0, 1.000 |
| 90 °/s, 0 % misses, 2-rx view | 18.1 / 21.4, -18.1, 0, 1.000 | 7.5 / 11.1, -7.5, 0, 1.000 |
| 90 °/s, 30 % misses, 1-rx view | 20.1 / 29.5, -20.1, 0, 1.000 | 10.2 / 22.9, -9.4, 1, 0.986 |
| 90 °/s, 30 % misses, 2-rx view | 18.5 / 21.9, -18.5, 0, 1.000 | 8.2 / 15.1, -8.2, 0, 1.000 |
| 180 °/s, 0 % misses, 1-rx view | 36.4 / 43.1, -36.4, 0, 1.000 | 18.4 / 29.0, -18.4, 0, 1.000 |
| 180 °/s, 0 % misses, 2-rx view | 35.4 / 42.1, -35.4, 0, 1.000 | 15.9 / 19.2, -15.9, 0, 1.000 |
| 180 °/s, 30 % misses, 1-rx view | 37.0 / 47.7, -37.0, 0, 1.000 | 19.1 / 30.2, -19.1, 0, 1.000 |
| 180 °/s, 30 % misses, 2-rx view | 37.3 / 45.9, -37.3, 0, 1.000 | 16.6 / 24.9, -16.6, 0, 1.000 |
| 360 °/s, 0 % misses, 1-rx view | 70.8 / 120.2, -70.8, 0, 1.000 | 45.5 / 103.1, -45.5, 1, 0.572 |
| 360 °/s, 0 % misses, 2-rx view | 67.0 / 81.9, -67.0, 0, 1.000 | 31.7 / 39.7, -31.7, 0, 1.000 |
| 360 °/s, 30 % misses, 1-rx view | 96.6 / 172.8, -83.3, 1, 0.540 | 33.2 / 43.8, -33.2, 0, 1.000 |
| 360 °/s, 30 % misses, 2-rx view | 67.5 / 85.4, -67.5, 0, 1.000 | 34.0 / 55.0, -34.0, 0, 1.000 |
| 720 °/s, 0 % misses, 1-rx view | 89.9 / 169.9, +0.0, 0, 1.000 | 57.7 / 70.4, -57.7, 0, 1.000 |
| 720 °/s, 0 % misses, 2-rx view | 112.5 / 152.8, -112.5, 0, 1.000 | 59.7 / 74.5, -59.7, 0, 1.000 |
| 720 °/s, 30 % misses, 1-rx view | 89.9 / 169.9, +0.0, 0, 1.000 | 100.1 / 172.0, -44.7, 1, 0.701 |
| 720 °/s, 30 % misses, 2-rx view | 120.6 / 172.8, -59.4, 0, 1.000 | 89.5 / 166.2, -34.7, 1, 0.935 |

### 7.10 Brief appearance (ball visible for d ms, 48 phases)

| ball visible for … | 40 ms | 60 ms | 80 ms | 100 ms | 150 ms | 200 ms | 300 ms | 600 ms |
|---|---|---|---|---|---|---|---|---|
| 1 receiver: acquired of 48, loss declared after the signal ended (median/p95 ms): legacy → engine 2 | 0 (-) → 0 (-) | 0 (-) → 0 (-) | 0 (-) → 0 (-) | 0 (-) → 0 (-) | 8 (-) → 48 (149/189) | 48 (-) → 48 (146/150) | 48 (545/560) → 48 (134/150) | 48 (544/560) → 48 (157/171) |
| 2 receivers: acquired of 48, loss declared after the signal ended (median/p95 ms): legacy → engine 2 | 0 (-) → 0 (-) | 0 (-) → 0 (-) | 12 (-) → 14 (170/231) | 42 (-) → 44 (149/231) | 48 (557/563) → 48 (149/151) | 48 (547/563) → 48 (137/150) | 48 (546/562) → 48 (171/190) | 48 (547/563) → 48 (165/181) |

A ball that disappears right after being accepted is a *young* track: its grace is judged cautiously (up to 300 ms in the worst case, because the sample is small); with the follow-up of §11 the median is about 150-170 ms.

### 7.11 Modulation period of the ball (unverified in reality; 42 % duty)

| modulation period | legacy: 2-rx acquired of 48, median / p95 ms, steady loss median ms | engine 2 |
|---|---|---|
| 600 µs (1667 Hz) | 48/48, 150 / 167, 545 | 48/48, 89 / 104, 136 |
| 833 µs (1200 Hz) | 48/48, 151 / 167, 546 | 48/48, 90 / 104, 136 |
| 1000 µs (1000 Hz) | 48/48, 152 / 168, 546 | 48/48, 90 / 104, 136 |
| 1250 µs (800 Hz) | 48/48, 152 / 168, 545 | 48/48, 90 / 104, 136 |
| 1500 µs (667 Hz) | 48/48, 152 / 168, 543 | 48/48, 90 / 104, 137 |
| 2000 µs (500 Hz) | 48/48, 232 / 462, 542 | 48/48, 154 / 218, 288 |
| 2500 µs (400 Hz) | 0/48 (never) | 48/48, 290 / 660, 287 |
| 3000 µs (333 Hz) | 0/48 (never) | 0/48 (never) |

A longer period puts fewer pulses into a 3 ms window, so the signal flickers; engine 2 keeps tracking where the legacy engine cannot.

### 7.12 Static direction accuracy and approach speed

| static ball | legacy: mean |error| °, jitter °, worst ° | engine 2 |
|---|---|---|
| 0 % misses, 2-receiver view, 48 bearings | 3.75, 0.00, 7.5 | 3.76, 0.01, 7.5 |
| 30 % misses, 2-receiver view, 48 bearings | 4.25, 0.37, 8.9 | 3.91, 0.14, 7.5 |
| 50 % misses, 2-receiver view, 48 bearings | 4.52, 0.65, 15.0 | 4.09, 0.31, 11.4 |

Commanded translation while tracking (engine 2, mean over 6 s): 1 receiver 170 / 170 / 170 / 170 PWM at 0 / 30 / 50 / 70 % misses; 2 receivers 145 / 145 / 144 / 145 PWM. A flickering ball is approached as fast as a clean one (the speed fade starts only when the evidence is really old; the 2-receiver ball sits at 75 degrees instead of 90 degrees, hence a different value; it is the same at every miss rate).

### 7.13 Cost

Host wall clock (x86, not ESP32), per `loop()` call without / with a track and per estimator call: legacy 37 ns / 41 ns / 121 ns; engine 2 49 ns / 56 ns / 401 ns. The estimator runs once per 3 ms window (legacy: once per 36 ms cycle); a 50-fold slower core would still leave it at about 0.7 % of a window (an assumption, not a measurement). Static: +5,396 bytes of flash and +760 bytes of RAM against `89c3503` on core 2.0.17; stack: `irService` 288 B including the inlined estimator.

## 8. Tests

`./test/run_tests.sh`: 88 cases × 2 core API shapes = 176 runs, 12,092 checks, 0 failed, ASan + UBSan, `-Wall -Wextra -Wshadow`. New in this work: `t_e2_*`, `t_startup_schedule`,
`t_scan_revisit_metrics`, `t_irprofile`, `t_legacy_fallback`, `t_engine_switch_at_run_time`, `t_latency` (now separates young and steady tracks). Static analysis: `docs/bench/static_analysis.txt`
(cppcheck: style notes only; clang-tidy: one known analyzer artefact; Valgrind memcheck clean on 12 heavy cases). Real builds: Arduino-ESP32 2.0.17 and 3.0.7, 0 warnings
(`docs/bench/real_build_final.txt`); stack frames from `-fstack-usage`: `irService` 288 bytes including the inlined estimator (`docs/bench/stack_usage_2.0.17.txt`).

## 9. Targets and what is still unmeasured

| Target | Result with the default (`balanced`) | With `:irprofile fast` | Verdict |
|---|---|---|---|
| 2 or more receivers: acquisition p95 <= 75 ms | 105 ms (2 rx), 102 ms (3 rx) | 72 / 66 ms | **unmet by default**, met with `fast` |
| 1 receiver: acquisition p95 <= 150 ms | 177 ms | 144 ms | **unmet by default**, met with `fast` |
| ball-loss declaration p95 <= 175 ms | 149 / 150 / 151 ms for a steady 1 / 2 / 3-receiver track; **290.9 ms for a flickering one** (grace 290 ms on purpose) | same | met for steady signals; **unmet for flickering signals by design** (it is the price of surviving their gaps) |
| survive intermittent signals and dropouts up to 250 ms where the evidence says a real tracked ball | flickering tracks, trials lost of 48 at a 200 / 250 ms gap: 30 % misses 1 rx: 1 / 2; 50 % misses 1 rx: 0 / 0; 50 % misses 2 rx: 0 / 0; no false loss at 10-70 % misses. Steady tracks: 100 ms | same | met for flickering signals (≥ 95 % of trials at 250 ms); **not for a steady one** (it would contradict the loss target; no signal tells the two cases apart) |
| no unbounded tracking or loss hold | grace bounded by `losthold_max` 300 ms (young track), `t_e2_grace_period_bounds` | same | met |
| no regression in broadband-burst rejection | 0/48 ring bursts at every duration (legacy accepts 60 ms bursts: 11/48, 9/48) | same | met (improved) |
| no material increase in false acquisitions | E[accept] (1 rx / 2 nb / 3 nb): equal or better than legacy (0.38 / 0.71 / 0.83), §6.1 | higher (§6.1) | met by default; **not with `fast`** |
| no material increase in false losses | 0 per minute up to 70 % misses (legacy 3.0 / 13.0 at 50 / 70 %); soak 0.5-0.7 per minute against 0.7-1.3 | same | met |
| no unbounded scheduler stalls or starvation of a receiver | longest wait of a receiver = stall + one cycle; with focus at most two ring cycles; 0 false losses at stalls up to 200 ms | same | met |
| steady-track loss in a faulty ring (dead / stuck / noisy receivers) | p95 <= 150 ms | same | met |


### Physical limits (nothing here is invented; unverified items are marked)

* **38 kHz receiver vs the ball's carrier.** A TSOP1738-class receiver is a 38 kHz band-pass part. Whether the competition ball uses 38 or 40 kHz, and its modulation, is **unverified**; a mismatch lowers the hit
  probability and the range, which appears to the firmware as exactly the flickering signal that §7 covers, but nothing in this document says how often it happens.
* **Receiver reaction and recovery.** A demodulating receiver needs a burst of carrier cycles before its output reacts and a gap before it can react again; the datasheet values were not available. The 3 ms window may or
  may not be limiting; the true minimum is **unknown**.
* **One receiver at a time.** The CD74HC4067 selects one channel; the 12-receiver ring is sampled sequentially. Median wait 18 ms, worst case 36 ms, per receiver duty 8 %. Firmware changes can reorder and prioritise windows
  (done: focus) but cannot make sensing parallel.
* **Hardware options, not implemented** (they need electrical review, no pin is assigned here): (1) two multiplexers read by two free input-only pins would halve the ring cycle; (2) a wired-OR of all receivers into one
  "any ball" input would give instant presence detection while the multiplexer gives direction; (3) a hardware hold/stretch of each receiver's output would turn a short pulse train into a level that is easy to sample.
* **Window length** is the other lever (`irwindow`): 2400 µs makes the ring 28.9 ms and trims the wait, but whether the real ball gives enough pulses in a shorter window is unknown; sensitivity in `docs/bench/v3_sweeps_seed12345.txt`. Not changed.
* **Misses are not independent in reality** (orientation, AGC, other robots). The visit-statistics grace assumes they are; `losthold`/`flickerhold` are floors chosen to hold even if they are not, and must be set from real data (§7.3, `CALIBRATION_AND_TEST_PLAN.md` §D.7).

## 10. Files

`Robosoccer26_Cyberbots/irscan.h` (engine 2, scheduler, scan metrics), `config.h` (new parameters, version 9), `nav.h` (event-driven transitions, brake before search, graded fade), `Robosoccer26_Cyberbots.ino`
(early control tick), `report.h`, `cli.h` (`:irprofile`), `test/bench.h`, `test/test_main.cpp`, `tools/`, `docs/`. Unchanged: `pins.h`, `motors.h`, `imu.h`, `health.h`, `storage.h`, `logger.h` relative to `89c3503`.

## 11. Follow-up: why `fast` accepts more bursts, and what could be done about it

**Cause (exact).** A burst that looks like a ball can only be told apart from one by how long the evidence lasts, so the probability of accepting a burst of
duration *d* equals the probability that a real ball's acquisition would be finished within *d*: `P(accept | d) = P(latency <= d)`. For bursts uniform in 20-250 ms this gives
`E[accept] = (250 - mean latency) / 230`. Measured against the tables: 1 receiver fast 0.53 (predicted from 127 ms: 0.53), balanced 0.39 (160 ms: 0.39); 2 neighbours fast 0.83 (58 ms: 0.83), balanced 0.70 (90 ms: 0.70);
3 neighbours fast 0.87 (52 ms: 0.86), balanced 0.72 (87 ms: 0.71). So `fast` is not "less careful" in some separable step: it is simply faster by 33 / 32 / 35 ms, and every millisecond of latency saved is paid for in accepted bursts.
Where the saving comes from: with one receiver the 4-hit evidence unit gives 3 ring revisits (108 ms) and `fast`'s span of 96 ms is not even binding, while `balanced` adds 32 ms (span 140 ms);
with neighbours the ring-revisit check (36 ms) is the floor of `fast` (span 30 ms) and `balanced` adds 34 ms (span 70 ms).

**Candidates tried against the existing benchmark** (host simulation; none is hardware evidence):

| Candidate | Result | Decision |
|---|---|---|
| Use the remaining latency slack for longer spans (`acqspan2` 45 instead of 30; `acqspan1` is not binding below 108 ms, so there is no slack for one receiver) | measured earlier (`docs/bench/v3_sweeps_seed12345.txt`-style grid, span1 130): span2 45 gives 2-receiver p95 77 ms (misses the 75 ms target) for E[accept] 2 nb 0.81 against 0.83; 1-receiver unchanged | rejected: the target is missed for a gain inside the sampling error (±0.02) |
| "Busy environment" penalty: stretch the spans by up to 100-200 % while ball-like episodes keep ending without a ball | soak share of eligible bursts accepted 46.5 % (fast) -> 47.1 % (100) / 44.4 % (200); medians of genuine acquisition in the busy soak 118-156 ms -> 116-187 ms (100) / 141-179 ms (200); quiet-system tables unchanged by construction | rejected: no measurable rejection gain; the code was removed |
| Pulse-period gate (`:irgate auto`, existing) | bursts whose pulse period differs from the ball's by more than the gate (+-35 %): E[accept] 0.53 -> 0.00 (period 1500 us); inside the gate (1000 us) or equal to the ball's: unchanged (0.53 -> 0.53) | the right tool, but only against sources with a different rhythm, and only once the real ball's period is measured; independent of the profile. Unchanged, still off by default |
| Shorter first grace period of a new track (the first grace was judged at the ring revisit rate although new tracks are watched with focus windows) | does not change acceptance or any quiet metric; **median time a phantom survives after a short accepted burst: 299 -> 151 ms** (2 receivers, 80 ms burst; worst case still 299 ms); soak phantom time per minute: no significant change (noise of the soak is about ±0.4 s/min) | **adopted** (one line in `irscan.h`, regression test `t_e2_phantom_exposure`) |

**Result.** Burst acceptance of `fast` cannot be improved at the same latency with the information the receivers provide; what could be improved is the cost of an accepted phantom, and that is. `fast` stays
an explicit opt-in (`:irprofile fast`) and the default is unchanged (140 / 70 ms; acquisition and every quiet-system figure unchanged from §6-§7).

Default against `fast` with the final code, three seeds (raw: `docs/bench/v4_engine2_*_seed*.txt`; table: `tools/bench_compare.py`):

| metric | default (balanced) | fast |
|---|---|---|
| acq p95 1/2/3 rx (ms, per seed) | 177/105/102 ; 177/105/102 ; 177/105/102 | 144/72/66 ; 144/72/66 ; 144/72/66 |
| E[accept] 1rx/2nb/3nb (tables, per seed) | 0.39/0.71/0.72 ; 0.39/0.72/0.71 ; 0.39/0.70/0.72 | 0.53/0.83/0.87 ; 0.53/0.83/0.86 ; 0.53/0.84/0.87 |
| ring bursts accepted (all durations) | 0 | 0 |
| soak bursts accepted % (per seed) | 44 ; 42 ; 45 | 47 ; 52 ; 46 |
| soak phantom s/min (per seed) | 2.7 ; 2.7 ; 2.7 | 3.3 ; 3.7 ; 2.3 |
| soak phantom acquisitions /min (per seed) | 5.3 ; 5.1 ; 5.4 | 5.2 ; 6.3 ; 5.0 |
| soak false losses /min (per seed) | 0.63 ; 0.63 ; 0.50 | 0.47 ; 0.50 ; 0.47 |
| false losses/min at 10-70% misses (max) | 0.00 ; 0.50 ; 0.00 | 0.00 ; 0.50 ; 0.00 |
| steady dropout survived (ms) | 100 ; 100 ; 100 | 100 ; 100 ; 100 |
| flickering (50%) dropout survived (ms) | 250 ; 250 ; 250 | 200 ; 250 ; 200 |
| re-acquisition after a lost steady gap (ms) | 55 ; 54 ; 54 | 54 ; 54 ; 54 |
| brief 2rx 80 ms: acquired of 48 (loss median/p95) | 14 (170/231) ; 16 (170/231) ; 13 (171/232) | 48 (150/183) ; 48 (150/182) ; 48 (150/181) |
| brief 2rx 100 ms: acquired of 48 (loss median/p95) | 44 (149/231) ; 45 (149/231) ; 44 (150/231) | 48 (149/151) ; 48 (149/151) ; 48 (149/151) |

Reading: acquisition p95 is 33 / 33 / 36 ms lower with `fast` (1 / 2 / 3 receivers) and it meets the 150 / 75 ms targets; in exchange 14 / 12 / 15 points more of the bursts in the expected-acceptance tables are accepted
(the soak shows +3, +10 and +1 points; its noise is several points; phantom acquisitions per minute 5.1-5.4 against 5.0-6.3, phantom time 2.7 against 2.3-3.7 s per minute); ring bursts are rejected by both; steady dropout tolerance (100 ms),
re-acquisition (54 ms) and the false-loss rates are the same (flickering-track dropout tolerance 250 ms for the default, 200-250 ms for `fast`: a difference of one or two trials of 48); a short 2-receiver burst (80 ms) becomes a phantom in
48/48 trials with `fast` against 13-16 of 48 with the default (100 ms: 48 against 44-45) and is dropped after about 150-170 ms (median) in both. One false loss in 120 s was seen at 10 % misses with one receiver (seed 22345, both profiles, also before this
change): four misses in a row, which the miss budget of 5 visits at `losseps` 4 accepts with probability 10^-4 per visit, i.e. about 0.3 events per 120 s; `losseps 5` removes that class at about +15 ms median steady loss latency (`docs/bench/v3_sweeps_seed12345.txt`).
