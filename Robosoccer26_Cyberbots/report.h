// report.h - human-readable diagnostics: health report, IR sensor snapshot/stream, nav and IMU status.
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

static const char* ringChWord(const Config& c, uint8_t i) {
  if (!ringEnabled(c, i)) return "off";
  if (g_ring[i].railFault) return "SATURATED (excluded)";
  return g_ring[i].seen ? "SEES ball" : "-";
}

static void printRingSnapshot(const Config& c) {
  uint8_t nU;
  float gap = ringCoverageGap(c, &nU);
  say("INFO", "ball ring: CD74HC4067, SIG GPIO%u, %u of %u channels enabled, %u usable | window %u us, settle %u us | seen at swing >= max(%u, noise x %.1f), very close at %u | noise floor %s",
      (unsigned)P_MUX_SIG, (unsigned)ringCountEnabled(c), (unsigned)RING_CH_MAX, (unsigned)ringCountUsable(c), (unsigned)c.ringWinUs, (unsigned)c.ringSettleUs, (unsigned)c.ringSeen,
      (double)c.ringNoiseX10 / 10.0, (unsigned)c.ringNear, c.ringCalDone ? "CALIBRATED" : "NOT CALIBRATED (':ring cal noise' with the ball off)");
  for (uint8_t i = 0; i < RING_CH_MAX; i++) {
    if (!ringEnabled(c, i) && i >= RING_CH_FIXED) continue;      // C12..C15 stay hidden until fitted and switched on
    const RingCh& r = g_ring[i];
    say("INFO", " C%-2u bearing %4d deg | swing %4u (smoothed %6.1f, max %4u) low %4u high %4u | noise %4u thr %5.0f | strength %.2f | %s", (unsigned)i, (int)c.ringBearing[i],
        (unsigned)r.amp, (double)r.avg, (unsigned)r.ampMax, (unsigned)r.lo, (unsigned)r.hi, (unsigned)c.ringNoise[i], (double)ringThreshold(c, i), (double)r.s, ringChWord(c, i));
  }
  say("INFO", "coverage: largest arc no usable receiver covers ~%.0f deg (configured bearings, ASSUMED +-%u deg field of view each; an estimate, not a measurement)%s", (double)gap, (unsigned)c.ringFovHalf,
      gap > 0 ? " | a ball inside it is not seen: the robot rotates to find it" : "");
  if (g_ball.bearingKnown) say("INFO", "ball: bearing %.0f deg (0 = ahead, + = right), strongest C%u, strength %.2f, %u channel(s) see it", (double)g_ball.bearing, (unsigned)g_ball.bestCh, (double)g_ball.strength, (unsigned)g_ball.nSeen);
  else if (g_ball.valid) say("INFO", "ball: not seen now, last bearing %.0f deg (held for %u ms)", (double)g_lastBearing, (unsigned)c.lostHoldMs);
  else say("INFO", "ball: not seen (a navigation condition, not a sensor fault)");
  say("INFO", "scan: last channel %lu us, longest %lu us; full passes %lu. A channel swing is a signal STRENGTH: the bearing is only as good as the mounting angles in ':ring' (placeholders until you set them)", UL(g_ringScanLastUs), UL(g_ringScanMaxUs), UL(g_ringCycles));
}

static void printModSnapshot(const Config& c) {
  for (uint8_t i = 0; i < MOD_COUNT; i++) {
    const Module& m = g_mod[i];
    say("INFO", "IR module %u GPIO%u (%s): %s | raw level %s | %s", (unsigned)i, (unsigned)PIN_MOD[i], i == 0 ? "front: ball in the shooting position" : "position undecided: logged only",
        modEnabled(c, i) ? "on" : "off", digitalRead(PIN_MOD[i]) ? "HIGH" : "LOW", !modEnabled(c, i) ? "-" : (m.active ? "DETECTING" : "clear"));
  }
  say("INFO", "modules: output is %s when something is detected (modactivelow=%u); shooting position = module 0 detecting%s", c.modActiveLow ? "LOW" : "HIGH", (unsigned)c.modActiveLow,
      c.modNeedRing ? " AND the ring sees the ball straight ahead" : " (ring not required)");
  say("INFO", "a module reacts to ANY reflective object in its range (wall, robot), and its own LED can disturb nearby receivers (docs/RING_AND_MODULES.md)");
}

static void printRingStream(const Config& c) {
  char buf[96];
  size_t n = 0;
  for (uint8_t i = 0; i < RING_CH_MAX && n + 8 < sizeof(buf); i++) {
    if (!ringEnabled(c, i)) continue;
    n += (size_t)snprintf(buf + n, sizeof(buf) - n, "%s%u", g_ring[i].seen ? "*" : "", (unsigned)g_ring[i].amp);
    if (n + 2 < sizeof(buf)) { buf[n++] = ' '; buf[n] = 0; }
  }
  if (!n) buf[0] = 0;
  say("INFO", "RING %s| ball=%s brg=%s held=%d ready=%d mod0=%d mod1=%d", buf, g_ball.valid ? "Y" : "n", g_ball.bearingKnown ? "known" : "?", g_ball.held, g_ball.ready, g_mod[0].active, g_mod[1].active);
  if (g_ball.bearingKnown) say("INFO", "   bearing %.0f deg strongest C%u strength %.2f", (double)g_ball.bearing, (unsigned)g_ball.bestCh, (double)g_ball.strength);
}

static void printNavStatus(const Config& c, uint32_t now) {
  say("NAV", "state=%s cond=%s autonomous=%s", NAV_NAME[g_navState], condName(g_cond), g_autoEnabled ? "ENABLED" : "disabled");
  say("NAV", "ball: valid=%d seen=0x%X (%u ch) held=%d ready=%d bearing=%s%.0f deg strength=%.2f age=%lu ms", g_ball.valid, (unsigned)g_ball.seenMask, (unsigned)g_ball.nSeen, g_ball.held, g_ball.ready,
      g_ball.bearingKnown ? "" : "(last) ", (double)(g_ball.bearingKnown ? g_ball.bearing : g_lastBearing), (double)g_ball.strength, UL(g_ball.lastSeenMs ? elapsedMs(now, g_ball.lastSeenMs) : 0));
  if (isnan(g_hdgErr)) say("NAV", "heading: hold %s (imu %s)", c.hdgEnable ? "inactive" : "disabled", imuHeadingUsable(c, now) ? "usable" : "unavailable");
  else say("NAV", "heading: ref %.1f yaw %.1f error %.1f deg", g_refYaw, g_imu.yaw, g_hdgErr);
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

  // --- ball ring and IR modules
  {
    uint8_t nU;
    float gap = ringCoverageGap(c, &nU);
    const char* w = ringCountEnabled(c) == 0 ? "OFF" : ringCountUsable(c) == 0 ? "FAIL (no usable channel)" : !c.ringCalDone ? "NOT TESTED (noise floor not calibrated)" : "calibrated (software: says nothing about the ball response)";
    say(tag, "[Ring] CD74HC4067 on SIG GPIO%u, S0..S3 = GPIO%u/%u/%u/%u: %u enabled, %u usable, %s", (unsigned)P_MUX_SIG, (unsigned)P_MUX_S0, (unsigned)P_MUX_S1, (unsigned)P_MUX_S2, (unsigned)P_MUX_S3, (unsigned)ringCountEnabled(c), (unsigned)ringCountUsable(c), w);
    for (uint8_t i = 0; i < RING_CH_MAX; i++)
      if (ringEnabled(c, i) && g_ring[i].railFault) say(tag, "[Ring] C%u FAIL (saturated: swing cannot be measured; shorted to a supply rail or steady light)", (unsigned)i);
    say(tag, "[Ring] coverage gap ~%.0f deg (estimate from configured bearings; bearings are PLACEHOLDERS until set with ':ring bearing')", (double)gap);
    for (uint8_t i = 0; i < MOD_COUNT; i++)
      say(tag, "[Module %u] GPIO%u (%s): %s, %s", (unsigned)i, (unsigned)PIN_MOD[i], i == 0 ? "front, shooting position" : "position undecided", modEnabled(c, i) ? "on" : "off", !modEnabled(c, i) ? "-" : (g_mod[i].active ? "detecting now" : "clear now"));
    say(tag, "[Ring] physical operation with the real ball (range, field of view of each receiver, lighting, the modules' LEDs): NOT TESTED until you calibrate and test it (docs/RING_AND_MODULES.md)");
    say(tag, "[Ring] limits: every channel gives strength only; a ball in an uncovered arc is invisible and is found by rotating. Strength is only a proxy for distance.");
  }

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
  say(tag, "stats: shots=%lu | imu re-init attempts=%lu recoveries=%lu", UL(g_stats.shots), UL(g_imu.initAttempts), UL(g_stats.imuRecoveries));
  say(tag, "[Firmware] stack headroom (lowest ever, bytes): loop task %lu of 8192, log-writer task %lu of 8192 | loop gap max %lu us | heap min free %lu B",
      UL(uxTaskGetStackHighWaterMark(NULL)), UL(g_logTaskHandle ? uxTaskGetStackHighWaterMark(g_logTaskHandle) : 0), UL(g_loopMaxEverUs), UL(ESP.getFreeHeap()));
  say(tag, "limits: no wheel feedback (PWM shown is COMMANDED); IR strength only, no distance or bearing; ball response unproven until tested; software failsafes do not replace wiring/current limits");
  say(tag, "========================================================");
}
