# Verification report

**Scope:** software only. No physical hardware was available, the real ESP32 toolchain could not be
installed (GitHub release assets, Espressif and Arduino download hosts are blocked in the build
sandbox), and the primary-source documents for the receiver and ball could not be fetched.

## 1. What was actually run

| Check | Result |
|---|---|
| Compile with the **real Arduino-ESP32 core + Xtensa GCC** | **NOT DONE** (toolchain unreachable). Flash/RAM size and real-core warnings are unknown. |
| API conformity against the real cores | Function signatures used (LEDC 2.x/3.x, `Wire`, `LittleFS`/`File`, `HardwareSerial::setTxBufferSize/availableForWrite`, `esp_task_wdt_*`, `esp_reset_reason_t`) were checked against headers fetched from `espressif/arduino-esp32` tags **2.0.17** and **3.0.7** and ESP-IDF 5.1.4/4.4. The `esp_task_wdt_init` 2.x form was written from knowledge of IDF 4.4 (its header could not be fetched). |
| Host build, mock core **v3 API** and **v2 API** | Builds clean with `g++ 13 -std=c++17 -Wall -Wextra -Wshadow -Wformat=2` (only the matching LEDC API exists in each mock, so using the wrong generation fails to compile). |
| Test suite (`test/run_tests.sh`) | **57 test cases × 2 core variants = 114 runs, 0 failures**, ≈2,250 checks per variant, each case in a fresh process, with AddressSanitizer + UBSan (no findings). |
| Valgrind memcheck on export, CLI fuzz, flight recorder, diagnostics, FS-failure | No uninitialised reads / invalid accesses. |
| `cppcheck` (warning, style, performance, portability) | No defects in the firmware; remaining notes are intentional byte-views for the config CRC (`memset` of float structs is all-zeros = 0.0f) and mock-only "always true" lock calls. |
| `clang-tidy` (analyzer + bugprone) | One `clang-analyzer-core.uninitialized.Assign` note in `outPutLine`; Valgrind shows no uninitialised read, treated as a false positive from modelling `strlen` after `snprintf`. |

### Coverage of the requested software checks (§16 of the brief)

| # | Requirement | Test(s) |
|---|---|---|
| 3 | Four motor direction mappings | `t_pinmap` (all wheels × both reversal states, pins checked against an independent copy of the wiring spec), `t_mecanum`, `t_cli_manual`, `t_drive_test` |
| 4 | PWM limits, normalisation, ramping | `t_mecanum`, `t_ramp` (per-step ≤ accel×dt, zero-crossing dwell, immediate stop, limit clamp, stall-clamped step) |
| 5 | Angle wraparound | `t_angles`, `t_ir_wrap` (330°/0° → −15°, not 165°), `t_heading_hold` (±180° seam) |
| 6 | Timer rollover | `t_timer_rollover` (virtual clock crosses 2³² ms mid-run while tracking) |
| 7 | Serial commands, safe input | `t_cli_manual`, `t_cli_robust` (CR/LF, NUL, high bytes, 4000-byte fuzz, overlong, control chars, unterminated `:`), `t_diag_no_motion`, `t_motor_test` |
| 8 | Buffer bounds, log rotation | `t_log_queue_overflow`, `t_log_rotation` (file-count and size caps, constant column count), `t_flight_recorder` (ring bounded) |
| 9 | Corrupt configuration | `t_config` (NaN, range, enum, cross-field, CRC, version, magic, size, partial repair, refuse to save invalid) |
| 10 | State-machine / fault recovery | `t_nav_*`, `t_safe_stop_moving`, `t_ir_scan_stall`, `t_ir_system_loss`, `t_loop_stall`, `t_imu_dropout` |
| 11 | Unavailable IMU / filesystem | `t_imu_missing`, `t_fs_missing`, `t_fs_write_failure`, `t_fs_full` |
| 12 | Optional failures do not disable unrelated functions | the same tests assert ball seeking continues (`g_navState == NAV_APPROACH`, wheels moving) |
| 13 | Logging cannot block control | `t_nonblocking` (worst `loop()` call < 300 µs of virtual time while diagnostics flood a baud-limited UART; writer lives outside `loop()`) |
| 14 | Simulated sensor loss, conflicts, IMU timeout, storage errors | `t_ir_stuck_and_recover`, `t_ir_noise`, `t_ir_ambiguous`, `t_ir_overrun`, `t_imu_invalid`, `t_fs_*` |
| – | Boot safety, no automatic motion, reboot never resumes autonomy | `t_boot_safe`, `t_resets`, `t_reset_wdt`, `t_reboot_phase1/2` (two real process launches sharing the filesystem: config restored, session counter incremented, earlier logs intact, autonomy off, ball in view does not move the robot) |
| – | Export integrity | `t_export` (independent CRC-32 of the captured Serial stream matches `#END-EXPORT`) |

## 2. Problems found by testing and fixed

1. **Default config failed its own validation.** Placeholder bearings for the spare channels C12–C15 were 360–450°, so a fresh configuration was "invalid" and `:set`/`:save` would have been refused. Found by `t_cfg_set`. Fixed (spare channels reuse 0–90°).
2. **Interference could pass as a ball.** With the original noise threshold (6 pulses/ms) a 4 kHz interferer was classified as a detection. Found by `t_ir_noise`. Threshold lowered to 3 pulses/ms (ball ≈ 1.2/ms); still configurable. A frequency gate was deliberately not added because the ball's modulation is unconfirmed.
3. **Session counter**: a fresh filesystem used a random session id instead of 1 (`t_reboot_phase1`). Fixed.
4. **IMU re-detect latency**: after a lost sensor the retry interval was 4 s; now 2 s once it had been seen (`t_imu_dropout`).
5. **Export format**: the ring-export header was counted as one line instead of two so `lines=` disagreed with the payload (`t_export`). Fixed.
6. **Export header buffer** too small for the longest column row (compiler `-Wformat-truncation`). Enlarged.
7. **Frozen-frame detector vs unrealistic mock**: the mock IMU produced identical frames, tripping the "frozen sensor" guard. The guard is correct for real hardware (real MPU6050 output always has LSB noise); the mock was made realistic instead of weakening the guard.
8. Test-only mistakes corrected (manual-hold timing, rollover timing, naive substring checks).
9. **Scan never started if every channel was disabled at boot** (found in review, then `t_boot_no_channels`): the scheduler is now armed unconditionally, so enabling a channel later starts scanning and the `IR_NO_CHANNELS` fault clears with verification.
10. **`:fsformat` could trip the 3 s watchdog or raise a false `LOOP_STALL`** (found in review, `t_fs_commands`): motors are stopped first, the task is removed from the watchdog for the erase, and the loop timer is reset afterwards.
11. Review additions made because the brief required them: full-drivetrain test (`:dtest`), fault reports carry certainty and mode, snapshot rows carry drop/write-failure counters, IMU re-init attempts are logged, writer-task stack enlarged.

## 3. NOT verified – needs the physical robot (all NOT TESTED)

* Real-core compile, binary size, RAM use, stack depth of the loop and writer tasks.
* Everything in `CALIBRATION_AND_TEST_PLAN.md` §F.
* TSOP1738 behaviour with the 40 kHz ball, mux settling, actual scan timing, ISR load, interference from the motors.
* Real MPU6050 noise, orientation sign (`yawsign`), drift.
* Motor direction, start PWM (`minpwm`), current draw, L298N heating, brownout margin.
* Boot-time pin behaviour (GPIO5/14/2/15) and the effect of the pull-downs.
* Real LittleFS latency and the length of flash-write stalls; real baud-limited Serial behaviour (modelled as 11.5 kB/s).
* The TSOP17xx datasheet values, ball modulation and the current rules text (primary sources unreachable).

## 4. Known limitations
See `HARDWARE_REVIEW.md` §5.
