// Robosoccer26_Cyberbots - RoboCup Junior soccer robot firmware for the ARDUINO MEGA 2560 (ATmega2560).
// (An earlier controller version, on a different board, is kept in git: commit 1963a3f.)
//
// 4x mecanum (2x L298N) | 2 phototransistor receivers on A0 / A1 (A2..A5 optional) | 2 IR obstacle modules (D34, D35) | HC-05 on Serial1 (D18/D19) | bottom IR sensor D34 | MPU6050 on I2C (D20/D21)
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

void loop();                                       // (setup() fast-forwards the start-up in the host tests only)

void setup() {
  motorsPinsSafe();                                // FIRST: every motor pin LOW, before anything can fail
  memset(&g_imu, 0, sizeof(g_imu));                // (already zero after a real reset; the host tests call setup() more than once)
  Serial.begin(SERIAL_BAUD);
  delay(100);
  cfgDefaults(g_cfg);
  say("BOOT", "Robosoccer26_Cyberbots %s starting. Motors are OFF; the start-up stages follow: MOTOR INIT, IMU CHECK, IR SETTLING, IR CHECK, SAFETY CHECK, READY.", FW_VERSION);

  // Configuration from EEPROM (never writes at boot).
  char rep[40];
  CfgLoadResult cr = cfgLoad(g_cfg, rep, sizeof(rep));
  say("BOOT", "config: %S%s%s", cfgLoadText(cr), rep[0] ? " -> " : "", rep);
  if (cr == CL_BADVERSION || cr == CL_BADSIZE || cr == CL_BADCRC) say("WARN", "the saved settings were NOT usable and defaults are in use: the receiver noise floor is measured again at start-up (autocal), other calibrations must be repeated");
  if (cr != CL_OK && cr != CL_NOFILE && cr != CL_MIGRATED) faultRaise(F_CFG_REPLACED, "%s", rep[0] ? rep : "config replaced");

  recInit();
  recAdd(REC_BOOT, 0);
  btBegin(g_cfg);
  authBegin();
  say("BOOT", "Bluetooth (HC-05) on Serial1 D18/D19 at %lu baud: %S | authentication %S, key %S", UL(btBaudNow(g_cfg)), g_cfg.btEnable ? PSTR("listening") : PSTR("disabled"),
      g_auth.enabled ? PSTR("ENFORCED") : PSTR("OFF"), g_auth.keySet ? PSTR("set") : PSTR("NOT SET (use :authkey over USB; the remote can only send X and P until then)"));

  // Drivetrain PWM check (duty 0, outputs still disabled), receivers, IMU probe. Nothing below waits for a device: the slow parts (gyro bias, IR settling) run as start-up stages in loop().
  bool pwmOk = motorsAttachPwm(g_cfg);
  say("BOOT", "motor PWM pins: %S (D%u D%u D%u D%u)", pwmOk ? PSTR("OK") : PSTR("NOT PWM CAPABLE"), (unsigned)WHEEL_PINS[0].en, (unsigned)WHEEL_PINS[1].en, (unsigned)WHEEL_PINS[2].en, (unsigned)WHEEL_PINS[3].en);
  ringBegin();
  say("BOOT", "ball receivers: %u of %u enabled, noise floor %S | IR modules: front D%u %S, second D%u %S", (unsigned)ringCountEnabled(g_cfg), (unsigned)RING_CH_MAX,
      g_cfg.ringCalDone ? PSTR("calibrated") : PSTR("NOT CALIBRATED (measured automatically at start-up if the receivers are quiet)"), (unsigned)PIN_MOD[0], modEnabled(g_cfg, 0) ? PSTR("on") : PSTR("off"), (unsigned)PIN_MOD[1], modEnabled(g_cfg, 1) ? PSTR("on") : PSTR("off"));
  bool imuOk = imuInit(g_cfg, true);
  if (imuOk) imuStartCalibration();                // measured by the IMU CHECK stage while loop() keeps running

  motorsSetEnabled(true);                          // outputs allowed, but every target is zero and the start-up gate refuses movement until READY
  motorsDrive(g_cfg, 0, 0, 0, g_cfg.maxManual);
  g_motorsLastUpdateMs = millis();
  condEvaluate();
  navSetState(g_cond == COND_SAFE_STOP ? NAV_SAFE_STOP : NAV_IDLE, PSTR("boot complete"));
  startupBegin(millis());
  outFlushBlocking(300);
  g_bootPhase = false;
  g_loopMaxEverUs = 0;                             // statistics start after the boot-time blocking work

#if RS_USE_WATCHDOG
  wdt_enable(WDTO_2S);
#endif
  g_lastLoopUs = micros();
#ifdef HOST_TEST
  if (!g_hostStartupAsync) {                       // host tests: run the stages to the end so a test starts from READY (startup tests set g_hostStartupAsync)
    for (int i = 0; i < 20000 && !startupFinished(); i++) { loop(); mockAdvance(1000); }
    if (!g_hostAutostart) g_autoStartPending = false;    // the tests that are not about the automatic start keep the old 'idle after setup()' behaviour
  }
#endif
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t now = millis();
#if RS_USE_WATCHDOG
  wdt_reset();
#endif

  uint32_t gap = (uint32_t)(nowUs - g_lastLoopUs);
  g_lastLoopUs = nowUs;
  g_tele.loopAvgUs = (g_tele.loopAvgUs * 15u + gap) / 16u;
  if (gap > g_loopMaxUs) g_loopMaxUs = gap;
  if (gap > g_loopMaxEverUs) g_loopMaxEverUs = gap;
  if (gap > LOOP_STALL_FATAL_MS * 1000u) {
    motorsStopNow();                               // we were blind for a full second: do not trust any old command
    faultRaise(F_LOOP_STALL, "loop blocked for %lu ms", UL(gap / 1000));
  }

  startupService(g_cfg, now);                      // the start-up stages (non-blocking)
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
  yawCheckService(g_cfg, now);
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
    motionService(g_cfg, now);
  }

  reportService(g_cfg, now);
  logService(g_cfg, now);
  streamService(g_cfg, now);
  teleService(g_cfg, now);
  outPump();
}

#ifdef HOST_TEST
#undef snprintf
#undef say
#endif
