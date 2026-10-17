# Verification report

Every claim in this repository carries one of these evidence classes. Nothing is "passed" on hardware.

| Class | Meaning | Status |
|---|---|---|
| **A. REAL-BUILD** | Compiled by the real Arduino-ESP32 core + Xtensa GCC | **Done for 2.0.17 and 3.0.7** (§1) |
| **B. EMU** | The real compiled binary executed in a third-party ESP32 emulator | Boot-level only (§2) |
| **C. HOST** | Firmware logic against a simulated ball/IMU/filesystem on the PC (mock Arduino API) | **Done** – 69 cases × 2 API shapes (§3) |
| **D. STATIC** | cppcheck, clang-tidy, Valgrind, sanitizers, compiler warnings | Done (§4) |
| **E. ELECTRICAL** | Needs inspection/measurement of the real circuit | **Not done** |
| **F. BALL** | Needs the real competition ball | **Not done** |
| **G. ROBOT** | Needs supervised testing of the physical robot | **Not done** |
| **H. UNVERIFIED** | Source documents unreachable or facts unknown | Listed in §6 |

## 1. A – Real ESP32 compilation

* Target: FQBN `esp32:esp32:esp32` (ESP32 Dev Module, ESP32-S 38-pin class, default 4 MB partition table with `spiffs`). `--warnings all` (= `-Wall -Wextra`).
* The sandbox blocks the Espressif/Arduino download hosts, so the toolchains were taken from the public Docker image `arduinoci/ci-arduino-esp32` (tags `v2.0.17`, `v3.0.7`, each contains `arduino-cli` and the matching esp32 core), by pulling its layers from `mirror.gcr.io` and extracting them. `tools/real_build.sh` documents the layout and the normal-machine commands (`arduino-cli core install esp32:esp32@<ver>`). The mock harness was **not** used for these builds.

| Core | Compiler | arduino-cli | Result | Flash | RAM (static) | Warnings |
|---|---|---|---|---|---|---|
| Arduino-ESP32 **2.0.17** (IDF 4.4.7) | xtensa-esp32-elf-gcc 8.4.0 (esp-2021r2-patch5) | 0.35.3 | **success** | 412,477 B (31 %) | 69,340 B (21 %) | **0** |
| Arduino-ESP32 **3.0.7** (IDF 5.1.4) | xtensa-esp32-elf-gcc 12.2 (esp-x32 2302) | 1.1.1 | **success** | 434,001 B (33 %) | 68,184 B (20 %) | **0** |

Command (per core): `arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all Robosoccer26_Cyberbots`.

What the real build proved: LEDC (2.x channel API and 3.x pin API), `Wire`, `LittleFS`/`File`, `HardwareSerial::setTxBufferSize/availableForWrite`, `esp_task_wdt_*` (both generations), `esp_reset_reason`, `soc/gpio_struct.h` all resolve and link; `static_assert`s on the pin map evaluate against the real SoC masks (negative tests with a motor pin on GPIO35 and with a duplicated pin both **failed the real build** as intended). It also found three defects the mock could not (§5).

## 2. B – The real binary in an emulator (limited value, third-party tool)

`esp32-emu` 1.2.0 (npm; installed with `--ignore-scripts`, run with a scrubbed environment, used only on my own images). Merged flash images (bootloader + partitions + boot_app0 + app) of both real builds were booted from a blank flash.

* Both images: real ROM → bootloader → app → `setup()` → full health report → "autonomous mode: DISABLED", no panic/abort/backtrace in the UART capture.
* On blank flash LittleFS fails to mount; the firmware reports `FS_UNAVAILABLE` (DEGRADED), **does not format**, continues. Without an MPU model it reports `IMU_MISSING` (DEGRADED).
* With GPIO34 at the emulator's default LOW level, all channels become stuck-active and the firmware enters SAFE_STOP at ~2.7 s (the stuck-channel and safe-stop paths work in the real binary).
* Motor lines (core 2.0.17 run): all 12 lines start as inputs at level 0, become outputs at level 0 in `motorsPinsSafe()`, the four EN pins pass to LEDC at duty 0; **0 rising edges in 12 recorded edges; LEDC duty 0**.
* **Limits**: the emulator does not model the chip's reset-time pull-ups (GPIO5 shows no pull); its UART has no input path I could use, so no commands were sent; its I²C slave staging was not used (no IMU); its reset-button test did not produce a second boot (inconclusive); the 3.0.7 run was only captured through UART (the automated GPIO summary run timed out). Its fidelity is unproven. **This is not hardware evidence.**

## 3. C – Host test suite

Command: `./test/run_tests.sh` (cleans `/tmp/mockfs*`, then for **mock core v3 and v2** builds `g++ -std=c++20 -O1 -g -Wall -Wextra -Wshadow -Wformat=2 -fsanitize=address,undefined` and runs every case in its own process).

**Final result: 70 test cases × 2 core API shapes = 140 runs, 140 passed, 0 failed, 5,108 checks, 0 compiler warnings, 0 sanitizer findings, 0 skipped.**

Reconciliation of the counts quoted during development (counted from `git show <commit>:test/test_main.cpp`): commit `47ebae6` = 53 cases · `b1987f6` = 55 · `d5d47b6` = 57 (114 runs, the figure in the previous report; a mid-session "55 per core" was a miscount of PASS lines) · `bd8862f` = 66 · current = **70**. The suite grew as audits added regression tests.

| Area (requirement) | Test cases |
|---|---|
| Pure maths, rollover | `t_angles`, `t_timer_rollover` |
| Motors: mixing, directions, pin map, ramps, safe start | `t_mecanum`, `t_pinmap`, `t_ramp`, `t_boot_safe`, `t_drive_test`, `t_motor_test`, `t_cli_manual` |
| Config defaults/validation/CRC, `:set`, boot with corrupt config, failed save | `t_config`, `t_cfg_set`, `t_gpio_tamper`, `t_cfg_boot_corrupt`, `t_cfg_save_fail` |
| IR scan, classification, timing, mapping | `t_classify`, `t_ir_detect`, `t_ir_wrap`, `t_ir_ambiguous`, `t_ir_polarity`, `t_ir_overrun`, `t_ir_coverage`, `t_ir_probe`, `t_ir_spare_channels`, `t_ch_inspect`, `t_latency` |
| Noise / interference / faults | `t_ir_noise`, `t_ir_slow_interferer`, `t_ir_period_gate`, `t_ir_stuck_and_recover`, `t_ir_system_loss`, `t_ir_trusted_low`, `t_ir_scan_stall`, `t_boot_no_channels` |
| IMU | `t_imu_integrate`, `t_imu_bias`, `t_imu_missing`, `t_imu_dropout`, `t_imu_invalid`, `t_imu_frozen_in_approach`, `t_heading_hold` |
| Navigation / state machine | `t_nav_approach`, `t_nav_all_directions`, `t_nav_sweep_and_multi`, `t_nav_unreliable_slow`, `t_nav_lost_search`, `t_nav_hysteresis`, `t_nav_isolated_samples`, `t_side_zone_note`, `t_safe_stop_moving`, `t_loop_stall` |
| Reset / reboot / explicit enable | `t_resets`, `t_reset_wdt`, `t_reboot_phase1/2` (two real process launches), `t_cal_startup_defaults` |
| Serial interface | `t_cli_robust` (CR/LF, NUL, high bytes, 4000-byte fuzz, overlong/unterminated lines), `t_diag_no_motion`, `t_live_ir_rate` |
| Storage / logging / export | `t_fs_missing`, `t_fs_write_failure`, `t_fs_full`, `t_fs_commands` (format + watchdog + no false stall), `t_log_rotation`, `t_log_queue_overflow`, `t_flight_recorder`, `t_export` (CRC and row count == rows on disk), `t_nonblocking` |
| Reports | `t_health_report_text`, `t_fault_report_format` |

Measured in simulation (not on hardware): scan cycle 36.1 ms; acquisition 57–143 ms (see docs/LATENCY_OPTIMIZATION.md; legacy engine 153–240 ms); loss 141 ms (legacy 546 ms); bearing jump while a ball circles the robot at 60°/s ≤ 6° per 50 ms with no loss event.

## 4. D – Static analysis (`./test/static_analysis.sh`)
* cppcheck: no defects; only style notes remain (const-qualifiable parameters).
* clang-tidy (analyzer + bugprone): one `clang-analyzer-core.uninitialized.Assign` note in `outPutLine`; **Valgrind memcheck on seven heavy cases is clean** (no uninitialised read), so it is treated as an analyzer artefact (symbolic `strlen` after `snprintf`).
* Real compilers with `--warnings all`: 0 warnings (§1). Host: `-std=c++20` also clean.

## 5. Defects found and fixed during this audit

| # | Defect | Found by | Fix |
|---|---|---|---|
| 1 | Edge ISR (`IRAM_ATTR`) called `digitalRead()` and `micros()`, which the linker puts in **flash**: a crash if the interrupt ran with the cache disabled (e.g. a build with ISR-in-IRAM, or any future IRAM flag). | `nm` on the real ELF | ISR reads `GPIO.in1.val` and `esp_timer_get_time()`; verified by disassembly (only call is IRAM `esp_timer_get_time`) on both cores |
| 2 | `-Wvolatile` deprecation warnings on volatile counters (GCC 12 / 3.0.7) | real 3.0.7 build | explicit load/store; builds are warning-free |
| 3 | Core version printed "unknown" on 2.0.17 (`ESP_ARDUINO_VERSION_STR` does not exist there) | real 2.0.17 image output | composed from MAJOR.MINOR.PATCH; test added |
| 4 | Ball-loss latency 718 ms vs 500 ms documented: the smoothing history refreshed the freshness timestamps | `t_latency` | freshness from the last two windows only → 537 ms |
| 5 | Coverage line printed "gap 0 deg" and could imply continuous coverage despite the reported 40 mm side blind zone | audit of reports | explicit warning in `h`; `ball_lost_side_zone` log note; docs |
| 6 | No way to separate a ball from an unrelated modulated source with a similar rhythm | analysis, `t_ir_period_gate` | optional measured pulse-period gate (`:irgate`), off by default; limitation documented |
| 7 | Large temporaries (up to 1.5 KB frames) on the 8 KB loop-task stack | `-fstack-usage` on the real build | moved to static storage; runtime stack high-water printed by `h` |
| 8 | Review omitted GPIO4 from the strapping list | real `gpio_struct.h` | documentation corrected |
| 9 | Scan never started if all channels were disabled at boot; `:fsformat` could trip the watchdog / false loop-stall | earlier review | fixed with regression tests (`t_boot_no_channels`, `t_fs_commands`) |
| 10 | Config version bumped to 4 (new gate fields): a saved v3 config is rejected with a clear message and defaults load | change | by design |

## 6. H – Unverified (no evidence either way)
TSOP1738 datasheet values; the ball's carrier/modulation/mode and which league uses it; current rules text; ESP32 reset-state pull strengths; L298N board input conditioning; motor stall current; receiver supply voltage/pinout; MPU6050 breakout wiring; the exact ESP-32S module (PSRAM?); real scan timing and flash-stall duration; real IMU noise, sign and drift.

## 7. What remains to be done (all NOT TESTED)
See `docs/COMMISSIONING_CHECKLIST.md` and `docs/CALIBRATION_AND_TEST_PLAN.md`. The robot is **not competition-ready**: no electrical, ball, or supervised-robot test has been performed.
