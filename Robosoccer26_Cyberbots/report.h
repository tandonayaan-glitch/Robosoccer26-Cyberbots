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

static const char* irSensorWord(const Config& c, uint8_t i) {
  if (!irEnabled(c, i)) return "switched off";
  if (!irCalibrated(c, i)) return "NOT CALIBRATED";
  if (g_irs[i].railFault) return "FAULT (reading at an ADC end stop)";
  return g_irs[i].near ? "ball NEAR" : (g_irs[i].seen ? "ball seen" : "clear");
}

static void printIrSnapshot(const Config& c) {
  say("INFO", "IR sensors (2): sampled every %lu ms; seen at %u %% / near at %u %% of the clear->near span, %u-sample debounce",
      UL(IR_SAMPLE_MS), (unsigned)c.irSeenPct, (unsigned)c.irNearPct, (unsigned)c.irDebounce);
  for (uint8_t i = 0; i < IR_SENSORS; i++) {
    const IrSensor& s = g_irs[i];
    say("INFO", " sensor %u GPIO%u %s: %s | raw %u (smoothed %.0f, min %u max %u) | clear %u near %u | strength %.2f | %s", (unsigned)i, (unsigned)PIN_IR[i],
        irModeOf(c, i) ? "digital" : "analog", irEnabled(c, i) ? "on" : "off", (unsigned)s.raw, (double)s.avg, (unsigned)(s.rawMin == 0xFFFF ? 0 : s.rawMin), (unsigned)s.rawMax,
        (unsigned)irClearOf(c, i), (unsigned)irNearOf(c, i), (double)s.s, irSensorWord(c, i));
  }
  if (g_ball.valid)
    say("INFO", "ball: %s%s%s%s | sides seen 0x%X near 0x%X%s", g_ball.seenMask ? "SEEN" : "recently seen", g_ball.held ? ", in the shooting position" : "", g_ball.ready ? " and CENTRED (ready)" : "",
        g_ball.offsetKnown ? "" : " | side unknown", (unsigned)g_ball.seenMask, (unsigned)g_ball.nearMask, "");
  else say("INFO", "ball: not seen (a navigation condition, not a sensor fault)");
  if (g_ball.offsetKnown) say("INFO", "ball offset %.2f (-1 left .. +1 right, sensor 1 mounted to the %s of sensor 0)", (double)g_ball.offset, c.irOffsetSide > 0 ? "right" : "left");
  say("INFO", "a 2-pin sensor gives signal STRENGTH only: it cannot see a ball outside its own field of view, and strength is a proxy for distance only for this ball, this mounting and this lighting (docs/IR_SENSORS.md)");
}

static void printIrStream(const Config& c) {
  say("INFO", "IR s0 %4u %5.2f%s%s | s1 %4u %5.2f%s%s | ball=%s held=%d ready=%d off=%s", (unsigned)g_irs[0].raw, (double)g_irs[0].s, g_irs[0].seen ? " seen" : "", g_irs[0].near ? " near" : "",
      (unsigned)g_irs[1].raw, (double)g_irs[1].s, g_irs[1].seen ? " seen" : "", g_irs[1].near ? " near" : "", g_ball.valid ? "Y" : "n", g_ball.held, g_ball.ready,
      g_ball.offsetKnown ? (g_ball.offset > 0.15f ? "right" : (g_ball.offset < -0.15f ? "left" : "centre")) : "?");
  (void)c;
}

static void printNavStatus(const Config& c, uint32_t now) {
  say("NAV", "state=%s cond=%s autonomous=%s", NAV_NAME[g_navState], condName(g_cond), g_autoEnabled ? "ENABLED" : "disabled");
  say("NAV", "ball: valid=%d seen=0x%X near=0x%X held=%d ready=%d offset=%s%.2f age=%lu ms", g_ball.valid, (unsigned)g_ball.seenMask, (unsigned)g_ball.nearMask, g_ball.held, g_ball.ready,
      g_ball.offsetKnown ? "" : "(unknown) ", (double)g_ball.offset, UL(g_ball.lastSeenMs ? elapsedMs(now, g_ball.lastSeenMs) : 0));
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

  // --- IR sensors
  for (uint8_t i = 0; i < IR_SENSORS; i++) {
    const char* role = i == 0 ? "front centre: ball close enough to shoot" : "front offset: centred / off-centre";
    const char* word = !irEnabled(c, i) ? "OFF" : !irCalibrated(c, i) ? "NOT TESTED (not calibrated)" : g_irs[i].railFault ? "FAIL (reading at an ADC end stop)" : "calibrated (software: says nothing about the ball response)";
    say(tag, "[IR] sensor %u GPIO%u (%s): %s, %s, raw %u, clear %u near %u", (unsigned)i, (unsigned)PIN_IR[i], role, irModeOf(c, i) ? "digital" : "analog", word, (unsigned)g_irs[i].raw, (unsigned)irClearOf(c, i), (unsigned)irNearOf(c, i));
  }
  say(tag, "[IR] sensor 1 mounting side: %s", c.irOffsetSide > 0 ? "right of sensor 0" : (c.irOffsetSide < 0 ? "left of sensor 0" : "NOT SET (no sideways guidance)"));
  say(tag, "[IR] physical operation with the real ball (range, field of view, lighting, other IR sources): NOT TESTED until you calibrate and test it (docs/IR_SENSORS.md)");
  say(tag, "[IR] limits: two front sensors cannot see a ball outside their field of view; the robot finds it by rotating. Signal strength is only a proxy for distance.");

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
