// Robosoccer26_Cyberbots - RoboCup Junior soccer robot firmware (ESP32-S, Arduino-ESP32 2.x or 3.x)
//
// 4x mecanum (2x L298N) | 12x TSOP17xx via CD74HC4067 | MPU6050 | LittleFS CSV logging
// Safe by construction: motors off at boot, autonomy only after an explicit 'a', every fault
// degrades or stops instead of guessing. See docs/ for wiring review, calibration and test plan.
//
// Serial: 115200 baud, send '?' for the command list.

#ifdef HOST_TEST
  #include "mock_arduino.h"          // host-side test harness (never part of the Arduino build)
#else
  #include <Arduino.h>
  #include <Wire.h>
  #include <LittleFS.h>
  #include <esp_system.h>
  #include <esp_task_wdt.h>
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/semphr.h>
  #if __has_include(<esp_arduino_version.h>)
    #include <esp_arduino_version.h>
  #endif
#endif

#include "cli.h"

static const uint32_t WDT_TIMEOUT_MS = 3000;        // hung loop => reset (outputs return to inputs: external pull-downs!)
static const uint32_t LOOP_STALL_FATAL_MS = 1000;   // a single loop gap this long => SAFE_STOP (latched)
static uint32_t g_lastHealthMs = 0, g_lastCtlMs = 0;
#ifdef HOST_TEST
static bool g_hostWriterInline = true;
#endif

static void wdtBegin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg;
  cfg.timeout_ms = WDT_TIMEOUT_MS;
  cfg.idle_core_mask = (1u << 0);
  cfg.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
#endif
  esp_task_wdt_add(NULL);
}

#ifndef HOST_TEST
static void logTask(void*) {
  for (;;) {
    logPump(g_cfg, millis());
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
#endif

static void bootIrSettle(uint32_t ms) {            // let a few scan cycles complete before the report
  uint32_t t0 = millis();
  while (elapsedMs(millis(), t0) < ms) {
    uint32_t now = millis();
    irService(g_cfg, now);
    imuUpdate(g_cfg, now, true);
    outPump();
    delayMicroseconds(100);
  }
}

void setup() {
  motorsPinsSafe();                                // FIRST: every motor pin LOW, before anything can fail
  Serial.setTxBufferSize(2048);
  Serial.begin(SERIAL_BAUD);
  delay(100);
  g_resetReason = esp_reset_reason();
  cfgDefaults(g_cfg);
  say("BOOT", "Robosoccer26_Cyberbots %s starting. Motors are OFF; autonomy is OFF until 'a'.", FW_VERSION);
  say("BOOT", "reset reason: %s", resetReasonName(g_resetReason));

  // Storage + configuration (never formats).
  fsInit();
  char rep[64];
  CfgLoadResult cr = cfgLoad(g_cfg, rep, sizeof(rep));
  say("BOOT", "filesystem: %s | config: %s%s%s", g_fsDetail, cfgLoadText(cr), rep[0] ? " -> " : "", rep);
  if (cr != CL_OK && cr != CL_NOFILE && cr != CL_NOFS) faultRaise(F_CFG_REPLACED, "%s %s", cfgLoadText(cr), rep);
  if (!g_fsRwOk) faultRaise(F_FS_UNAVAILABLE, "%s", g_fsDetail);

  // Drivetrain PWM (duty 0, outputs still disabled).
  bool pwmOk = motorsAttachPwm(g_cfg);
  say("BOOT", "motor PWM init: %s (%u Hz, %u-bit)", pwmOk ? "OK" : "FAILED", (unsigned)g_cfg.pwmFreq, (unsigned)PWM_BITS);

  // IR acquisition.
  irBegin(g_cfg);
  say("BOOT", "IR mux configured: %u channel(s) enabled, window %u us", (unsigned)irCountEnabled(g_cfg), (unsigned)g_cfg.irWindowUs);

  // IMU: probe, configure, measure gyro bias while stationary.
  bool imuOk = imuInit(g_cfg, true);
  say("BOOT", "MPU6050: %s", imuOk ? "found, calibrating gyro bias (keep the robot still)" : "NOT FOUND - heading hold will be unavailable");
  if (imuOk) {
    imuStartCalibration();
    uint32_t t0 = millis();
    while (g_imu.calState == 1 && elapsedMs(millis(), t0) < 4000) {
      uint32_t now = millis();
      irService(g_cfg, now);
      imuUpdate(g_cfg, now, true);
      outPump();
      delayMicroseconds(100);
    }
  }

  logInit(g_cfg);
  if (g_resetReason == ESP_RST_BROWNOUT) faultRaise(F_BROWNOUT_RESET, "previous reset was a brownout: supply dipped below threshold");
  if (g_resetReason == ESP_RST_PANIC || g_resetReason == ESP_RST_INT_WDT || g_resetReason == ESP_RST_TASK_WDT || g_resetReason == ESP_RST_WDT)
    faultRaise(F_ABNORMAL_RESET, "previous reset: %s", resetReasonName(g_resetReason));

  bootIrSettle(700);
  uint32_t now = millis();
  irHealth(g_cfg, now);
  imuHealth(g_cfg, now);
  condEvaluate();
  logEvent("E", "boot");

  motorsSetEnabled(true);                          // outputs allowed, but every target is zero
  motorsDrive(g_cfg, 0, 0, 0, g_cfg.maxManual);
  g_motorsLastUpdateMs = millis();
  navSetState(g_cond == COND_SAFE_STOP ? NAV_SAFE_STOP : NAV_IDLE, "boot complete");

  printHealth(g_cfg, now, "BOOT");
  say("BOOT", "autonomous mode: DISABLED. Bench-test first ('t'), then send 'a'. '?' lists commands.");
  g_bootPhase = false;
  outFlushBlocking(1500);

#ifndef HOST_TEST
  xTaskCreatePinnedToCore(logTask, "logw", 8192, nullptr, 1, nullptr, 0);   // flash writes live off the control core
#endif
  wdtBegin();
  g_lastLoopUs = micros();
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t now = millis();
  esp_task_wdt_reset();

  uint32_t gap = (uint32_t)(nowUs - g_lastLoopUs);
  g_lastLoopUs = nowUs;
  if (gap > g_loopMaxUs) g_loopMaxUs = gap;
  if (gap > g_loopMaxEverUs) g_loopMaxEverUs = gap;
  if (gap > LOOP_STALL_FATAL_MS * 1000u) {
    motorsStopNow();                               // we were blind for a full second: do not trust any old command
    faultRaise(F_LOOP_STALL, "control loop was blocked for %lu ms", UL(gap / 1000));
  }

  cliPoll(now);
  irService(g_cfg, now);
  imuUpdate(g_cfg, now, motorsAllZero() && elapsedMs(now, g_motorsStoppedSinceMs) > 1000);

  if (elapsedMs(now, g_lastHealthMs) >= 50) {      // 20 Hz health monitor
    g_lastHealthMs = now;
    irHealth(g_cfg, now);
    imuHealth(g_cfg, now);
    faultUpdate(F_FS_UNAVAILABLE, !g_fsRwOk, g_cfg.recoverVerifyMs, g_fsDetail);
    condEvaluate();
  }
  if (elapsedMs(now, g_lastCtlMs) >= 10) {         // 100 Hz control tick
    g_lastCtlMs = now;
    navTick(g_cfg, now);
    motorsUpdate(g_cfg, now);
  }

  frService(g_cfg, now);
  logService(g_cfg, now);
  exportService();
  streamService(g_cfg, now);
#ifdef HOST_TEST
  if (g_hostWriterInline) logPump(g_cfg, now);
#endif
  outPump();
  yield();
}
