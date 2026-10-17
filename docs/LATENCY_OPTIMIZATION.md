# IR latency optimisation (simulation only)

**Every number here comes from the host simulation (`./build/rs26 --bench`, virtual clock, 50 µs loop quantum, simulated ball and
receivers). None is a physical measurement.** Real latency depends on the real ball's modulation, the receiver's AGC behaviour and
range, flash stalls, and the motors; those are untested (`docs/IR_DETECTION_AUDIT.md`).

## How the old figures were measured
`t_latency` ran one trial per case: ball switched on at an arbitrary scan phase (125–189 ms = 1-receiver / 2-receiver cases at that one phase)
and switched off for loss (537 ms ≈ 500 ms hold + ~37 ms window). One trial is not a distribution; the benchmark now injects over two scan
cycles at random pulse phases and reports sample counts, median, p95 and max (`docs/bench/*.txt`, `test/bench.h`).

## Stage budget (median / p95, ms; scan cycle 36.1 ms = 12 × (3 ms window + 10 µs settle))
| Stage | Baseline (legacy engine) | Optimised (engine 1) |
|---|---|---|
| Ball confirmed, 1 receiver sees it | 224 / 240 | **128 / 143** |
| Ball confirmed, 2 receivers | 153 / 168 | **57 / 72** |
| Ball confirmed, 3 receivers | 120 / 134 | **50 / 66** |
| Wheels reach start PWM (1 receiver) | 238 / 254 | **142 / 159** |
| Adjacent sector 90→120° direction update | 221 / 237 | **90 / 104** |
| Opposite sector 90→270° | 207 / 223 | **115 / 126** |
| Rear seam 150→210° | 255 / 271 | **113 / 125** |
| Ball declared lost (strong signal) | 546 / 563 | **141 / 154** |
| Translation ≤10 % of steady after loss | 691 / 707 | **285 / 301** |

## Reliability cost (same benchmark, defaults)
| Test | Baseline | Optimised |
|---|---|---|
| Burst immunity (one-receiver ball-like burst never acquired up to) | 100 ms | 100 ms |
| Broadband bursts (all receivers, 40/100/250 ms) acquired | 0 | 0 |
| Random-burst soak, false acquisitions in 120 s | 4 | 6 |
| Longest dropout survived (24/24) | 250 ms | **100 ms** |
| False losses/min, 1-receiver ball, 10/30/50 % per-visit misses | 0 / 0 / 5 | 0 / 8 / 9 |
| Same, 2-receiver ball | 0 / 0 / 0 | 0 / 0 / 5 |

So the speed-up is **not free**: a weak, intermittent single-receiver signal now loses the ball more often (the grace is scaled between
`losthold` 120 ms and `losthold_max` 300 ms from a measured hit-rate), and the soak shows 6 vs 4 false acquisitions per 120 s of synthetic
bursts. `engine=0 losthold=500` restores the legacy behaviour (`:set engine 0`).

## What changed, and what was rejected
* Estimate updated after **every closed window** (was: once per scan cycle); per-receiver streak/recency evidence instead of an 8-window fraction.
* Acquisition: summed streak evidence ≥1 **and** one receiver hit `acqminstreak` (2) times in a row. Without that ring-revisit guard a
  broadband onset looks like a ball on adjacent receivers (scan order = bearing order): measured 24/24 false acquisitions → 0 with the guard.
* Direction from the last 3 windows per receiver, τ = 40 ms smoothing, gated to ±100° around the track (70° gave a self-reinforcing bias that
  excluded the edge receivers of a 5-receiver cluster; caught by `t_nav_sweep_and_multi`).
* Loss: grace scaled by hit-rate quality *measured only when evidence returns* (a first attempt let the quality decay during the outage and
  lengthened its own grace: loss 318 ms).
* Motors: a start from rest jumps to `minpwm` (the wheel cannot turn below it) instead of ramping through the dead zone (-96 ms start).
* Not done / rejected: shorter observation windows (3 ms is already ≈ 114 periods of 38 kHz; fewer pulses per window raised missed-pulse
  windows in the simulation); stop latency is bounded by `decel` ramp (285 ms) — shortening it is a mechanical-safety choice for the robot test.

## Physical limits (unverified, not invented)
* A TSOP-type receiver needs a burst of carrier cycles before its output goes active and then recovers; its exact timing is in the datasheet that
  was unreachable. The 3 ms window therefore may not be limiting, but the true minimum is **unknown**.
* The ball's carrier (reported 40 kHz in some leagues) versus a 38 kHz receiver is the dominant risk, not scheduling.
* One multiplexer means one receiver observed at a time: the scan cycle (36 ms) bounds single-receiver latency ≈ onset + 2–3 cycles; only spatial
  (multi-receiver) evidence is faster. A second ADC-free path (e.g. hardware OR of groups) would need electrical changes.

## Verification of this change
Host suite 70 cases × 2 cores = 140 runs, 5,108 checks, 0 failed, ASan/UBSan clean; real builds 2.0.17 (31 %, 0 warnings) and 3.0.7 (33 %, 0 warnings);
static analysis unchanged. Hardware: **none**.
