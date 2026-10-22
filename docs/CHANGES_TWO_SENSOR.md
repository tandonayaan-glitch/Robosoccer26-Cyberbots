# Change log: from the multiplexer / 12-receiver ring to two front IR sensors

## Obsolete hardware and features removed

* CD74HC4067 multiplexer: `PIN_MUX_SIG/S0–S3` (GPIO34, 4, 13, 15, 2), the channel select code, the settle delay, channels C0–C15, the spare-channel handling.
* The 12-receiver ring and everything that existed for it: the edge-counting ISR, the 3 ms windows and scan scheduler, the per-channel history, the leaky-evidence ball estimator ("engine 2") and the legacy estimator,
  bearings, coverage-gap and field-of-view maths, trusted channels, controlled-ball tests (`:verify`), coverage mapping (`:learn`, `:cov`), the pulse-period gate (`:irgate`), `:irprofile`, `:irchar`,
  the auxiliary-input experiment (`:aux`, `:avoid`, `:hold`), per-channel configuration records, the three-pin TSOP assumptions (pinout, supply, active-LOW) and the 38 kHz/40 kHz discussion.
* Their faults (`IR_NO_CHANNELS`, `IR_SCAN_STALL`, `IR_TRUSTED_LOW`, `IR_GAP`, `IR_CHANNEL`), statistics, log columns (`actmask`, `bear_deg`, `conf`, `scan_us`), report text, tests, the benchmark harness and its recorded data, and the documents that described them
  (`LATENCY_OPTIMIZATION`, `IR_DETECTION_AUDIT`, `IR_HARDWARE_EXPERIMENT`, `AUX_IR_INPUTS`, the old verification report). They remain in the git history (last commit before this change: `7423143`).
* Obstacle avoidance on spare inputs (both inputs are now the ball sensors).

## Kept unchanged

Motor GPIOs and `motors.h` (mecanum mixing, ramps, reversal dwell, LEDC for both core generations), the MPU6050 code and pins (GPIO16/17), storage, logging, flight recorder, export, health monitor, the CLI conventions (`f b l r q e s ! a i I m d h g x t ?`, `:set/:get/:save/:load/:defaults`, `:rev`, `:roller`, `:mtest/:dtest/:confirm/:mok/:mrev`, `:zero`, `:gcal`, `:recover`, `:goal`).

## Added

* `ir2.h`: two sensors on GPIO35 and GPIO36 (analog or digital per sensor, calibration, hysteresis, debounce, wiring-fault detection, ball state: seen / near / held / offset / ready).
* `ircli.h`: `:ir` (status, `cal clear|near`, `mode`, `on|off`, `side`, `reset`).
* Navigation: approach that slows with the signal, cautious single-sensor alignment, stop at the shooting position, search by rotation toward the last known side.
* Faults `IR_FRONT` (sensor 0, CRITICAL) and `IR_ALIGN` (sensor 1, DEGRADED).
* Configuration: sensor mode, calibration, thresholds, mounting side, speeds (configuration version 13: a saved configuration from the old firmware is replaced by the defaults once).
* The goal heading (registered by the gyro 5 s after power-on) and the shot (aim, reverse, full-power drive); no kicker pin exists or is assumed.

## Files

| File | Change |
|---|---|
| `Robosoccer26_Cyberbots/pins.h` | multiplexer pins removed; IR pins GPIO35/GPIO36; `static_assert`s updated (14 outputs, 2 input-only sensors) |
| `config.h` | ring/engine/mux parameters removed; sensor, navigation, goal and shot parameters; version 13 |
| `ir2.h`, `ircli.h` | new |
| `irscan.h`, `auxin.h`, `auxcli.h` | deleted |
| `health.h` | IR faults replaced; statistics trimmed |
| `nav.h` | two-sensor approach, search, shooting position, goal heading and shot |
| `logger.h` | log columns, flight-recorder fields, detail rows |
| `report.h`, `cli.h`, `Robosoccer26_Cyberbots.ino`, `storage.h` | reports, commands, boot sequence; `g_fsMux` renamed `g_fsMutex` |
| `test/` | `mock_arduino.h` (sensor model), `test_main.cpp` (62 cases), `run_tests.sh`, `static_analysis.sh`; `bench.h` deleted |
| `tools/` | `real_build.sh` kept; the benchmark scripts and the receiver-geometry calculator deleted |
| `docs/`, `README.md` | rewritten for the new hardware; obsolete documents and benchmark data deleted |
