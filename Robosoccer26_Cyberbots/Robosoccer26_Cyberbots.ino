// Robosoccer26_Cyberbots - RoboCup Junior soccer robot firmware for the ARDUINO MEGA 2560 (ATmega2560).
// (The ESP32-S version is kept in git: commit 1963a3f.)
//
// 4x mecanum (2x L298N) | 2 phototransistor receivers on A0 / A1 (more can be added on A2..A7) | 2 IR obstacle modules (D34, D35) | HC-05 on Serial1 (D18/D19) | underside sensor D40 | MPU6050 on I2C (D20/D21)
// Safe by construction: motors off at boot, autonomy starts by itself after a clean boot (autostart, default on), never with a fault, every fault
// degrades or stops instead of guessing. See docs/ for wiring, power, calibration and the test plan.
//
// Board: Arduino Mega or Mega 2560 (ATmega2560), Arduino AVR core 1.8.3 or newer. No external libraries (Wire and EEPROM ship with the core).
// Serial: 115200 baud, send '?' for the command list.

#ifdef HOST_TEST
  #include "mock_arduino.h"          // host-side test harness (never part of the Arduino build)
#else
  #include <Arduino.h>
  #include <Wire.h>
  #include <EEPROM.h>
  #include <avr/wdt.h>
#endif

#include "cli.h"

// Optional hardware watchdog. OFF by default: the stock Arduino Mega 2560 bootloader (STK500v2) does not clear the watchdog after a watchdog reset, which
// can leave the board in a reset loop that only an ISP programmer fixes. Enable (-DRS_USE_WATCHDOG=1) only with a bootloader that handles it (e.g. Optiboot-Mega).
#ifndef RS_USE_WATCHDOG
#define RS_USE_WATCHDOG 0
#endif
static const uint32_t LOOP_STALL_FATAL_MS = 1000;   // a single loop gap this long => SAFE_STOP (latched)
static uint32_t g_lastHealthMs = 0, g_lastCtlMs = 0;
static bool g_ctlBallValid = false;           // last ball state seen by the control tick (an edge triggers an early tick)

static void bootRingSettle(uint32_t ms) {            // let the receivers settle before the report
  uint32_t t0 = millis();
  while (elapsedMs(millis(), t0) < ms) {
    uint32_t now = millis();
    ringService(g_cfg, now);
    imuUpdate(g_cfg, now, true);
    outPump();
  }
}

void setup() {
  motorsPinsSafe();                                // FIRST: every motor pin LOW, before anything can fail
  Serial.begin(SERIAL_BAUD);
  delay(100);
  cfgDefaults(g_cfg);
  say("BOOT", "Robosoccer26_Cyberbots %s starting. Motors are OFF during initialisation.", FW_VERSION);

  // Configuration from EEPROM (never writes at boot).
  char rep[40];
  CfgLoadResult cr = cfgLoad(g_cfg, rep, sizeof(rep));
  say("BOOT", "config: %S%s%s", cfgLoadText(cr), rep[0] ? " -> " : "", rep);
  if (cr != CL_OK && cr != CL_NOFILE) faultRaise(F_CFG_REPLACED, "%s", rep[0] ? rep : "config replaced");

  recInit();
  recAdd(REC_BOOT, 0);
  btBegin();
  say("BOOT", "Bluetooth (HC-05) on Serial1 D18/D19 at %lu baud: %S", UL(BT_BAUD), g_cfg.btEnable ? PSTR("listening") : PSTR("disabled"));

  // Drivetrain PWM check (duty 0, outputs still disabled).
  bool pwmOk = motorsAttachPwm(g_cfg);
  say("BOOT", "motor PWM pins: %S (D%u D%u D%u D%u)", pwmOk ? PSTR("OK") : PSTR("NOT PWM CAPABLE"), (unsigned)WHEEL_PINS[0].en, (unsigned)WHEEL_PINS[1].en, (unsigned)WHEEL_PINS[2].en, (unsigned)WHEEL_PINS[3].en);

  // Ball receivers (analog inputs A0, A1, ...) and the two IR modules (D34 / D35).
  ringBegin();
  say("BOOT", "ball receivers: %u of %u enabled, noise floor %S | IR modules: front D%u %S, second D%u %S", (unsigned)ringCountEnabled(g_cfg), (unsigned)RING_CH_MAX,
      g_cfg.ringCalDone ? PSTR("calibrated") : PSTR("NOT CALIBRATED"), (unsigned)PIN_MOD[0], modEnabled(g_cfg, 0) ? PSTR("on") : PSTR("off"), (unsigned)PIN_MOD[1], modEnabled(g_cfg, 1) ? PSTR("on") : PSTR("off"));

  // IMU: probe, configure, measure gyro bias while stationary.
  bool imuOk = imuInit(g_cfg, true);
  say("BOOT", "MPU6050: %S", imuOk ? PSTR("found, calibrating gyro bias (keep the robot still)") : PSTR("NOT FOUND - heading hold will be unavailable"));
  if (imuOk) {
    imuStartCalibration();
    uint32_t t0 = millis();
    while (g_imu.calState == 1 && elapsedMs(millis(), t0) < 4000) {
      uint32_t now = millis();
      ringService(g_cfg, now);
      imuUpdate(g_cfg, now, true);
      outPump();
    }
  }

  bootRingSettle(700);
  uint32_t now = millis();
  ringHealth(g_cfg, now);
  imuHealth(g_cfg, now);
  condEvaluate();

  motorsSetEnabled(true);                          // outputs allowed, but every target is zero
  motorsDrive(g_cfg, 0, 0, 0, g_cfg.maxManual);
  g_motorsLastUpdateMs = millis();
  navSetState(g_cond == COND_SAFE_STOP ? NAV_SAFE_STOP : NAV_IDLE, PSTR("boot complete"));

  printHealth(g_cfg, now, PSTR("BOOT"));
  if (g_cfg.autoStart && g_cond != COND_SAFE_STOP) {
    g_autoStartPending = true; g_autoStartAtMs = now + g_cfg.autoStartDelayMs;
    say("BOOT", "autostart: autonomous mode begins in %u ms if all checks still pass. Cancel: X (Bluetooth), '!', 's' or M; disable: ':set autostart 0' + ':save'.", (unsigned)g_cfg.autoStartDelayMs);
  } else if (g_cfg.autoStart) say("BOOT", "autostart SKIPPED: robot is in SAFE_STOP (see 'h'). Motors stay off.");
  else say("BOOT", "autonomous mode: DISABLED (autostart 0). Send 'a' or U to start. '?' lists commands.");
  g_bootPhase = false;
  outFlushBlocking(1500);
  g_loopMaxEverUs = 0;                                  // statistics start after the boot-time blocking work

#if RS_USE_WATCHDOG
  wdt_enable(WDTO_2S);
#endif
  g_lastLoopUs = micros();
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t now = millis();
#if RS_USE_WATCHDOG
  wdt_reset();
#endif

  uint32_t gap = (uint32_t)(nowUs - g_lastLoopUs);
  g_lastLoopUs = nowUs;
  if (gap > g_loopMaxUs) g_loopMaxUs = gap;
  if (gap > g_loopMaxEverUs) g_loopMaxEverUs = gap;
  if (gap > LOOP_STALL_FATAL_MS * 1000u) {
    motorsStopNow();                               // we were blind for a full second: do not trust any old command
    faultRaise(F_LOOP_STALL, "loop blocked for %lu ms", UL(gap / 1000));
  }

  if (g_autoStartPending) {                        // one-shot automatic start; any operator action or fault cancels it
    if (g_estop || g_remoteManual || g_autoEnabled || g_navState != NAV_IDLE || g_cond == COND_SAFE_STOP) { g_autoStartPending = false; if (!g_autoEnabled) say("BOOT", "autostart cancelled: %S. Motors stay off.", g_cond == COND_SAFE_STOP ? PSTR("fault / SAFE_STOP (see 'h')") : PSTR("operator action")); }
    else if (deadlineReached(now, g_autoStartAtMs)) {
      g_autoStartPending = false;
      if (!navEnableAuto(g_cfg, now)) say("BOOT", "autostart REFUSED by the safety checks above: motors stay off");
    }
  }
  btPoll(g_cfg, now);                              // remote commands first: the emergency stop has the highest priority
  cliPoll(now);
  ringService(g_cfg, now);
  ringCalPoll(g_cfg, now);
  goalService(g_cfg, now);
  imuUpdate(g_cfg, now, motorsAllZero() && elapsedMs(now, g_motorsStoppedSinceMs) > 1000);

  if (elapsedMs(now, g_lastHealthMs) >= 50) {      // 20 Hz health monitor
    g_lastHealthMs = now;
    ringHealth(g_cfg, now);
    imuHealth(g_cfg, now);
    condEvaluate();
  }
  if (g_ball.valid != g_ctlBallValid) {           // acquisition / loss: do not wait for the next 10 ms tick
    g_ctlBallValid = g_ball.valid;
    g_lastCtlMs = now - 10u;
  }
  if (elapsedMs(now, g_lastCtlMs) >= 10) {         // 100 Hz control tick
    g_lastCtlMs = now;
    navTick(g_cfg, now);
    motorsUpdate(g_cfg, now);
  }

  reportService(g_cfg, now);
  logService(g_cfg, now);
  streamService(g_cfg, now);
  outPump();
}

#ifdef HOST_TEST
#undef snprintf
#undef say
#endif
