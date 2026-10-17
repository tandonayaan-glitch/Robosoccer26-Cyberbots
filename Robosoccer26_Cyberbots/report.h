// report.h - human-readable diagnostics: health report, IR snapshot/stream, nav and IMU status.
#pragma once
#include "nav.h"

// ESP_ARDUINO_VERSION_STR does not exist in Arduino-ESP32 2.0.17 (found on the real build): compose it.
#define RS_STR2(x) #x
#define RS_STR(x) RS_STR2(x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && defined(ESP_ARDUINO_VERSION_MINOR) && defined(ESP_ARDUINO_VERSION_PATCH)
  #define CORE_VERSION_STR RS_STR(ESP_ARDUINO_VERSION_MAJOR) "." RS_STR(ESP_ARDUINO_VERSION_MINOR) "." RS_STR(ESP_ARDUINO_VERSION_PATCH)
#else
  #define CORE_VERSION_STR "unknown (version macros unavailable)"
#endif

static esp_reset_reason_t g_resetReason = ESP_RST_UNKNOWN;
static const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT_PIN";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC/EXCEPTION";
    case ESP_RST_INT_WDT: return "INTERRUPT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

static void printIrSnapshot(const Config& c) {
  uint8_t nT, nU;
  float gap = irCoverageGap(c, &nT, &nU);
  uint32_t revMax = 0;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) if (g_scan.revisitMaxUs[i] > revMax) revMax = g_scan.revisitMaxUs[i];
  say("INFO", "IR snapshot: %u enabled channel(s), ring cycle %lu us (avg %.0f, max %lu), cycles=%lu, window=%u us",
      (unsigned)irCountEnabled(c), UL(g_scan.cycleUs), g_scan.cycleAvgUs, UL(g_scan.cycleMaxUs), UL(g_scan.cycleCount), (unsigned)c.irWindowUs);
  say("INFO", "scan timing: longest time any receiver waited between two windows %lu us, longest late window %lu us, focus windows %lu, engine %u (ring cycle = time to open each enabled window once)",
      UL(revMax), UL(g_scan.winOverrunMaxUs), UL(g_scan.focusWindows), (unsigned)c.engine);
  say("INFO", " ch label bearing   raw det act edges per_us  hits8 rej status          (raw: pin level 1=idle for active-low; det: this window; act: smoothed; rej: windows rejected by the period gate)");
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) {
    if (!c.ch[i].enabled) continue;
    const IrChan& s = g_ir[i];
    say("INFO", " C%-2u %-5s %5.0f%s  %u   %u   %u  %5u %6lu  %u/8  %4lu %s%s", i, c.ch[i].activeLow ? "AL" : "AH", c.ch[i].bearingDeg,
        c.ch[i].bearingCal ? " " : "?", (unsigned)s.raw, (unsigned)s.detect, (unsigned)s.active, (unsigned)s.edges, UL(s.periodUs),
        popcount16(s.hist & 0xFF), UL(s.offband), CH_NAME[irChStatus(c, i)], irTrusted(c, i) ? "" : " [excluded]");
  }
  if (g_ball.valid || g_ball.anyDetect)
    say("INFO", "ball: %s, direction %.0f deg (heuristic from %u active channel(s)), confidence %.2f (heuristic), channel agreement %.2f%s",
        g_ball.valid ? "DETECTED" : "weak/unconfirmed", g_ball.bearingDeg, (unsigned)g_ball.nActive, g_ball.conf, g_ball.rInst,
        g_ball.ambiguous ? " AMBIGUOUS" : "");
  else say("INFO", "ball: not detected (this is a navigation condition, not a sensor fault)");
  say("INFO", "no distance or signal strength is available: receivers are digital (pulse seen / not seen)");
  if (nU) say("WARN", "%u trusted channel(s) have UNCALIBRATED bearings (placeholders, not measured)", (unsigned)nU);
  if (gap > c.maxGapDeg) say("WARN", "coverage gap %.0f deg exceeds %.0f deg (from configured bearings, assumed fov)", gap, c.maxGapDeg);
}

static void printIrStream(const Config& c) {
  char line[180];
  int p = snprintf(line, sizeof(line), "IR");
  for (uint8_t i = 0; i < MUX_CHANNELS && p < (int)sizeof(line) - 12; i++) {
    if (!c.ch[i].enabled) continue;
    char tag = '.';
    ChStatus st = irChStatus(c, i);
    if (st >= CH_SUSPECT_STUCK) tag = '!';
    else if (g_ir[i].active) tag = 'A';
    else if (g_ir[i].detect) tag = 'd';
    p += snprintf(line + p, sizeof(line) - (size_t)p, " %u%c%u", i, tag, (unsigned)g_ir[i].edges);
  }
  char dir[12] = "--";
  if (g_ball.anyDetect) snprintf(dir, sizeof(dir), "%.0f", g_ball.bearingDeg);   // only when justified by a detection
  say("INFO", "%s | cyc=%luus ball=%s dir=%s conf=%.2f", line, UL(g_scan.cycleUs), g_ball.valid ? "Y" : "n", dir, g_ball.conf);
}

static void printNavStatus(const Config& c, uint32_t now) {
  say("NAV", "state=%s cond=%s autonomous=%s", NAV_NAME[g_navState], condName(g_cond), g_autoEnabled ? "ENABLED" : "disabled");
  say("NAV", "ball: valid=%d dir=%.0f deg conf=%.2f (heuristic) active=0x%04X agreement=%.2f age=%lu ms", g_ball.valid, g_ball.bearingDeg,
      g_ball.conf, (unsigned)g_ball.actMask, g_ball.rInst, UL(g_ball.lastDetectMs ? elapsedMs(now, g_ball.lastDetectMs) : 0));
  if (isnan(g_hdgErr)) say("NAV", "heading: hold %s (imu %s)", c.hdgEnable ? "inactive" : "disabled", imuHeadingUsable(c, now) ? "usable" : "unavailable");
  else say("NAV", "heading: ref %.1f yaw %.1f error %.1f deg", g_refYaw, g_imu.yaw, g_hdgErr);
  say("NAV", "track: grace period %lu ms (steady signal %u ms, flickering %u ms, max %u ms), misses tolerated %u, measured miss fraction %.2f",
      UL(irEffectiveHoldMs(c)), (unsigned)c.lostHoldMs, (unsigned)c.flickerHoldMs, (unsigned)c.lostHoldMaxMs, (unsigned)g_ball.trackM, g_ball.trackFlick);
  say("NAV", "ball latency (firmware-measured, excludes the ball switching on/off): acquisition n=%lu last=%lu mean=%lu max=%lu ms | loss n=%lu last=%lu mean=%lu max=%lu ms",
      UL(g_latAcq.n), UL(g_latAcq.lastUs / 1000), UL(g_latAcq.n ? (uint32_t)(g_latAcq.sumUs / g_latAcq.n / 1000) : 0), UL(g_latAcq.maxUs / 1000),
      UL(g_latLoss.n), UL(g_latLoss.lastUs / 1000), UL(g_latLoss.n ? (uint32_t)(g_latLoss.sumUs / g_latLoss.n / 1000) : 0), UL(g_latLoss.maxUs / 1000));
  say("NAV", "request vx=%.2f vy=%.2f w=%.2f | commanded PWM FL=%d FR=%d BL=%d BR=%d (commanded, NOT measured wheel speed)",
      g_reqVx, g_reqVy, g_reqW, g_wheelOut[0], g_wheelOut[1], g_wheelOut[2], g_wheelOut[3]);
}

static void printImu(const Config& c, uint32_t now) {
  if (!g_imu.present) { say("INFO", "MPU6050: NOT FOUND (no answer at 0x68/0x69). Heading hold off, fallback in use."); return; }
  say("INFO", "MPU6050 addr=0x%02X whoami=0x%02X ok=%d fresh=%d calibrated=%d bias=%.3f dps", g_imu.addr, g_imu.whoami, g_imu.ok, imuFresh(c, now), g_imu.calibrated, g_imu.bias);
  say("INFO", " accel g: %.3f %.3f %.3f | gyro dps: %.2f %.2f %.2f", g_imu.ax, g_imu.ay, g_imu.az, g_imu.gx, g_imu.gy, g_imu.gz);
  say("INFO", " relative heading (cw+): %.1f deg %s | samples=%lu errors=%lu timeouts=%lu invalid=%lu recoveries=%lu gaps=%lu reinit-attempts=%lu",
      g_imu.yaw, g_imu.yawValid ? "" : "(INVALID)", UL(g_imu.samples), UL(g_imu.errCount), UL(g_imu.timeoutCount), UL(g_imu.invalidCount), UL(g_imu.recoveries), UL(g_imu.gaps), UL(g_imu.initAttempts));
  say("INFO", " heading is gyro-integrated and RELATIVE: it drifts and is not an absolute compass bearing");
}

static void printHealth(const Config& c, uint32_t now, const char* tag) {
  uint8_t nT, nU;
  float gap = irCoverageGap(c, &nT, &nU);
  say(tag, "================ HARDWARE HEALTH REPORT ================");
  say(tag, "firmware %s built %s %s | Arduino-ESP32 core %s | IDF %s", FW_VERSION, __DATE__, __TIME__, CORE_VERSION_STR, ESP.getSdkVersion());
  say(tag, "chip %s rev %d @ %lu MHz, flash %lu KB, free heap %lu B | reset reason: %s", ESP.getChipModel(), (int)ESP.getChipRevision(),
      UL(ESP.getCpuFreqMHz()), UL(ESP.getFlashChipSize() / 1024), UL(ESP.getFreeHeap()), resetReasonName(g_resetReason));
  say(tag, "operating condition: %s | autonomous: %s | nav: %s", condName(g_cond), g_autoEnabled ? "ENABLED" : "DISABLED (send 'a')", NAV_NAME[g_navState]);

  // --- drivetrain
  bool pwmOk = true;
  for (int i = 0; i < WHEEL_COUNT; i++) pwmOk = pwmOk && g_pwmAttached[i];
  say(tag, "[Drivetrain] GPIO + LEDC PWM init: %s (software setup only; says nothing about the wiring)", pwmOk ? "PASS" : "FAIL");
  for (int i = 0; i < WHEEL_COUNT; i++)
    say(tag, "[Drivetrain] wheel %s IN1=%u IN2=%u EN=%u reversal=%d: direction %s", WHEEL_PINS[i].name, WHEEL_PINS[i].in1, WHEEL_PINS[i].in2,
        WHEEL_PINS[i].en, c.reverse[i], (c.motorVerified & (1u << i)) ? "PASS (operator-confirmed by :mok, recorded in saved config)" : "NOT TESTED");
  say(tag, "[Drivetrain] motor/driver electrical health, actual rotation, stall: NOT MEASURABLE (no encoders, no current sensing)");

  // --- IR
  bool scanOk = g_scan.running && g_scan.cycleCount > 0 && elapsedMs(now, g_scan.lastCycleMs) < 600;
  say(tag, "[IR] acquisition: %s (%u channels enabled, cycle %lu us, window %u us, %lu cycles, overruns=%lu)",
      scanOk ? "PASS (software: scan running)" : "FAIL (scan not completing)", (unsigned)irCountEnabled(c), UL(g_scan.cycleUs), (unsigned)c.irWindowUs, UL(g_scan.cycleCount), UL(g_stats.scanOverruns));
  uint8_t nv = 0, ne = 0;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) {
    if (!c.ch[i].enabled) continue;
    ne++;
    ChStatus st = irChStatus(c, i);
    if (st == CH_VERIFIED) nv++;
    const char* word = (st == CH_VERIFIED) ? "PASS (controlled ball test)" : (st == CH_UNVERIFIED) ? "NOT TESTED" :
                       (st == CH_FAIL_STUCK || st == CH_FAIL_TEST) ? "FAIL" : "WARNING";
    say(tag, "[IR] C%u bearing %.0f%s tilt %.0f: %s [%s]%s", i, c.ch[i].bearingDeg, c.ch[i].bearingCal ? "" : " (UNCALIBRATED)", c.ch[i].tiltDownDeg, word, CH_NAME[st], irTrusted(c, i) ? "" : " excluded from navigation");
  }
  say(tag, "[IR] channels C12-C15 reserved for future receivers: %s", (c.ch[12].enabled || c.ch[13].enabled || c.ch[14].enabled || c.ch[15].enabled) ? "some ENABLED" : "disabled (not reported as failed)");
  say(tag, "[IR] physical operation: %u of %u enabled receivers verified with the real ball; the rest NOT TESTED (absence of the ball proves nothing)", (unsigned)nv, (unsigned)ne);
  say(tag, "[IR] trusted=%u (min %u), largest AZIMUTH gap %.0f deg (configured bearings + assumed fov %.0f; real coverage NOT TESTED)", (unsigned)nT, (unsigned)c.minTrusted, gap, c.fovHalfDeg);
  say(tag, "[IR] WARNING: a gap of 0 deg here does NOT mean continuous coverage. The reported ~40 mm blind zone beside each side of the chassis (between front and rear rings) and height/tilt effects are not representable in this azimuth-only model: a ball inside it may be invisible.");
  if (c.irPeriodMaxUs) say(tag, "[IR] pulse-period gate ON: accepting mean pulse periods %u..%u us (windows outside are rejected as 'offband')", (unsigned)c.irPeriodMinUs, (unsigned)c.irPeriodMaxUs);
  else say(tag, "[IR] pulse-period gate OFF: any 2+ pulse pattern below the interference rate is accepted; an unrelated 38-40 kHz source with a similar rhythm could be mistaken for the ball (measure the ball with :verify, then :irgate auto)");
  say(tag, "[IR] TSOP1738 vs 40 kHz ball compatibility: NOT TESTED (must be checked with the real ball: :learn / :verify)");
  if (c.engine == 2)
    say(tag, "[IR] detector: engine 2 (leaky evidence): acquire after %u ms (1 receiver) / %u ms (2+ neighbours) of consistent evidence, grace period %u..%u ms (flickering %u ms), focus %s, fast re-acquisition %u ms. All timings are SIMULATED; the real ball's hit rate and dropouts are NOT TESTED.",
        (unsigned)c.acqSpan1Ms, (unsigned)c.acqSpan2Ms, (unsigned)c.lostHoldMs, (unsigned)c.lostHoldMaxMs, (unsigned)c.flickerHoldMs, c.focusOn ? "on" : "off", (unsigned)c.reacqMs);
  else
    say(tag, "[IR] detector: engine 0 (legacy fallback): 8-window history, lost after %u ms. Slower than engine 2 in simulation; kept as the fallback.", (unsigned)c.lostHoldMs);

  // --- IMU
  if (!g_imu.present) say(tag, "[IMU] MPU6050: %s (no device answered at 0x68/0x69)", g_imu.hadFault ? "FAIL" : "WARNING - not found");
  else say(tag, "[IMU] MPU6050 @0x%02X who=0x%02X: %s; gyro bias %s; heading hold %s", g_imu.addr, g_imu.whoami,
           g_imu.ok ? (g_imu.whoami == 0x68 ? "PASS" : "WARNING (non-0x68 ID, clone?)") : "FAIL/RECOVERING",
           g_imu.calibrated ? "calibrated" : "NOT calibrated", imuHeadingUsable(c, now) ? "available" : "UNAVAILABLE");

  // --- storage/logging
  say(tag, "[Storage] filesystem: %s (%s); persistent logging %s; free %lu B", g_fsRwOk ? "PASS" : (g_fsMounted ? "FAIL" : "FAIL/unavailable"), g_fsDetail, g_logPersist ? "ON" : "OFF", UL(fsFreeBytes()));
  say(tag, "[Storage] RAM log queue %u/%d B, flight recorder %u/%d records", (unsigned)logqUsed(), LOGQ_SIZE - 1, (unsigned)g_frCount, FR_N);

  // --- power
  say(tag, "[Power] battery voltage: NOT MEASURABLE (no voltage monitor installed); motor current: NOT MEASURABLE");
  say(tag, "[Power] brownout/reset evidence: last reset = %s%s", resetReasonName(g_resetReason), g_resetReason == ESP_RST_BROWNOUT ? "  <-- WARNING: supply dipped, check wiring/regulator" : "");

  // --- faults
  int active = 0;
  for (int i = 0; i < F_COUNT; i++) {
    if (!g_fault[i].active) continue;
    active++;
    say(tag, "[FAULT] %s (%s, %s) since %lu ms, seen %lu x: %s -> %s", FMETA[i].code, FMETA[i].comp, sevName(FMETA[i].sev), UL(g_fault[i].firstMs), UL(g_fault[i].count), g_fault[i].evidence, FMETA[i].action);
  }
  if (!active) say(tag, "[FAULT] none active");
  say(tag, "stats: ball losses=%lu reacquired=%lu (avg %lu ms) search=%lu ms | imu timeouts=%lu degraded transitions=%lu safety stops=%lu | log dropped=%lu storage failures=%lu",
      UL(g_stats.ballLossEvents), UL(g_stats.reacquisitions), UL(g_stats.reacquisitions ? g_stats.reacquireTotalMs / g_stats.reacquisitions : 0),
      UL(g_stats.searchTotalMs), UL(g_stats.imuTimeouts), UL(g_stats.degradedTransitions), UL(g_stats.safetyStops), UL(g_stats.logDropped), UL(g_stats.storageFailures));
  say(tag, "stats: channels that failed an operator-run controlled test: mask 0x%04X | imu re-init attempts=%lu recoveries=%lu | scan overruns=%lu", (unsigned)g_stats.channelTestFailMask, UL(g_imu.initAttempts), UL(g_stats.imuRecoveries), UL(g_stats.scanOverruns));
  say(tag, "[Firmware] stack headroom (lowest ever, bytes): loop task %lu of 8192, log-writer task %lu of 8192 | loop gap max %lu us | heap min free %lu B",
      UL(uxTaskGetStackHighWaterMark(NULL)), UL(g_logTaskHandle ? uxTaskGetStackHighWaterMark(g_logTaskHandle) : 0), UL(g_loopMaxEverUs), UL(ESP.getFreeHeap()));
  say(tag, "limits: no wheel feedback (PWM shown is COMMANDED); no distance/IR strength; coverage and ball compatibility unproven until tested; software failsafes do not replace wiring/current limits");
  say(tag, "========================================================");
}
