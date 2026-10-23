// startup.h - the non-blocking start-up state machine. setup() only makes every motor output safe and probes the hardware; the slow parts run here, one small step per
// loop() pass, so USB commands (and the emergency stop '!') are served from the first millisecond and nothing waits for a device.
//
//   MOTOR INIT    all motor pins LOW / PWM 0 (done first in setup()); the PWM pins are checked.            -> FAULT if an enable pin is not a PWM pin
//   IMU CHECK     MPU6050 present? gyro bias measured while the robot is still (bounded: 6 s).             -> missing / rejected = DEGRADED, never a hang
//   IR SETTLING   startsettle ms (2 s): the receivers are scanned but their readings are IGNORED for every start-up decision. This is software only: it does
//                 NOT switch any infrared LED off (the obstacle modules' emitters and any fixed IR LEDs have no control pin on this robot).
//   IR CHECK      startcal ms (1 s), robot still: is any receiver seeing a signal? A signal now is the ball or an external IR source (sunlight, another robot,
//                 stage lights): it cannot be told apart, so nothing is "calibrated" from it and the automatic attack start is deferred (irwait) until it clears.
//                 If the receivers are quiet and no noise floor was saved, the floor is measured now (RAM only; ':save' keeps it) - this is what lets a robot whose
//                 saved calibration was lost (blank EEPROM, replaced config) move again without a manual step.
//   SAFETY CHECK  faults evaluated, health report requested.
//   READY         movement allowed. READY (degraded) = movement allowed but something is missing (IMU, uncalibrated receivers ...), listed in the message.
//   FAULT         a drivetrain fault: movement refused until it is cleared (':recover').
// Movement commands (manual keys, motor tests, attack, defend) are refused until READY; the emergency stop, diagnostics and settings work at all times.
// Nothing here ever starts a motor by itself except the optional automatic attack start (autostart), which needs READY (not degraded), a working IMU and quiet receivers.
#pragma once
#include "nav.h"

static void startupHealthReport();                  // report.h: asks the paced reporter for the health report
enum StartState : uint8_t { ST_MOTOR_INIT, ST_IMU_CHECK, ST_IR_SETTLE, ST_IR_CHECK, ST_SAFETY_CHECK, ST_READY, ST_READY_DEGRADED, ST_FAULT, ST_COUNT };
static const char START_NAME[ST_COUNT][17] PROGMEM = {"MOTOR INIT", "IMU CHECK", "IR SETTLING", "IR CHECK", "SAFETY CHECK", "READY", "READY (degraded)", "FAULT"};

struct StartupInfo {
  StartState st;
  uint32_t since, begunMs, readyMs;
  bool entered, imuOk, irQuiet, autoCal, armed, waitingIr, waitImu;
  uint16_t maxSwing[RING_CH_MAX];
  char why[48];
};
static StartupInfo g_start;

__attribute__((unused)) static bool startupFinished() { return g_start.st == ST_READY || g_start.st == ST_READY_DEGRADED || g_start.st == ST_FAULT; }
static bool startupMotionAllowed() { return g_start.st == ST_READY || g_start.st == ST_READY_DEGRADED; }

// Called by every command that could move a motor. Prints why not.
static bool startupGate(const char* what) {
  if (startupMotionAllowed()) return true;
  if (g_start.st == ST_FAULT) say("SAFETY", "%s REFUSED: start-up ended in FAULT (%s). Fix it, then ':recover'.", what, g_start.why);
  else say("SAFETY", "%s REFUSED: start-up is still running (%S, %lu ms in). The emergency stop and diagnostics work; movement is enabled at READY.", what, (PGM_P)START_NAME[g_start.st], UL(elapsedMs(millis(), g_start.since)));
  return false;
}

static void startGo(StartState s, uint32_t now) { g_start.st = s; g_start.since = now ? now : 1u; g_start.entered = false; }

static void startupBegin(uint32_t now) {
  memset(&g_start, 0, sizeof(g_start));
  g_start.begunMs = now;
  startGo(ST_MOTOR_INIT, now);
}

static void startupFault(uint32_t now, const char* why) {
  strncpy(g_start.why, why, sizeof(g_start.why) - 1);
  say("START", "FAULT: %s. Motors stay stopped.", why);
  startGo(ST_FAULT, now);
}

// Which enabled receivers saw a swing above their detection threshold during the IR CHECK window.
static uint16_t startIrActiveMask(const Config& c) {
  uint16_t m = 0;
  for (uint8_t i = 0; i < RING_CH_MAX; i++) if (ringEnabled(c, i) && (float)g_start.maxSwing[i] >= ringThreshold(c, i)) m |= (uint16_t)(1u << i);
  return m;
}
static void startMaskText(uint16_t m, char* out, size_t n) {
  out[0] = 0;
  for (uint8_t i = 0; i < RING_CH_MAX; i++) if ((m >> i) & 1u) { char t[6]; snprintf(t, sizeof(t), "%sA%u", out[0] ? "," : "", (unsigned)i); if (strlen(out) + strlen(t) < n) strcat(out, t); }
}

static void startupArmAutostart(const Config& c, uint32_t now) {
  if (!c.autoStart) { say("START", "autostart is OFF (':set autostart 1' to enable). Send 'a' / 'attack' to start the attack, 'defend' to defend."); return; }
  if (!g_start.imuOk) { g_start.waitImu = true; say("START", "autostart waits for the MPU6050 to start (not ready yet). 'a' starts the attack on request."); return; }
  if (!startupMotionAllowed() || g_cond == COND_SAFE_STOP || !c.ringCalDone) {
    say("START", "autostart NOT armed (%S%S). Manual keys and diagnostics work; 'a' starts the attack on request.", g_cond == COND_SAFE_STOP ? PSTR(" safety stop, see 'h'") : PSTR(""), !c.ringCalDone ? PSTR(" receivers not calibrated") : PSTR(""));
    return;
  }
  g_autoStartPending = true; g_autoStartAtMs = now + c.autoStartDelayMs;
  g_start.armed = true;
  say("START", "autostart armed: the attack begins in %u ms at the autonomous speed limit (maxauto %u of 255). Cancel with X, '!', 's' or 'm'; disable with ':set autostart 0'.", (unsigned)c.autoStartDelayMs, (unsigned)c.maxAuto);
}

static void startupService(Config& c, uint32_t now) {
  StartupInfo& s = g_start;
  if (!startupFinished() && outUsed() > OUTQ_SIZE / 4) return;   // one message at a time: wait until the previous lines have left the UART (no dropped stage messages)
  switch (s.st) {
    case ST_MOTOR_INIT:
      say("START", "MOTOR INIT: all four motors commanded stopped (direction pins LOW, PWM 0 on D%u D%u D%u D%u).", (unsigned)WHEEL_PINS[0].en, (unsigned)WHEEL_PINS[1].en, (unsigned)WHEEL_PINS[2].en, (unsigned)WHEEL_PINS[3].en);
      if (drivetrainFault()) startupFault(now, "a motor enable pin is not a PWM pin (see 'h')");
      else startGo(ST_IMU_CHECK, now);
      break;

    case ST_IMU_CHECK:
      if (!s.entered) { s.entered = true; say("START", "IMU CHECK: looking for the MPU6050 on D20/D21 and measuring the gyro bias - keep the robot still."); }
      if (!g_imu.present) {
        say("START", "IMU CHECK: MPU6050 NOT FOUND (wiring D20/D21, power, AD0). Heading hold is unavailable; ball seeking and manual control still work. Retrying in the background.");
        s.imuOk = false; startGo(ST_IR_SETTLE, now);
      } else if (g_imu.calState == 2) {
        say("START", "IMU CHECK: ok, gyro bias %.3f dps measured, relative yaw reference set (a gyro gives a RELATIVE heading that drifts).", (double)g_imu.bias);
        s.imuOk = true; startGo(ST_IR_SETTLE, now);
      } else if (g_imu.calState == 3 || elapsedMs(now, s.since) > 6000) {
        say("START", "IMU CHECK: gyro calibration %S. Heading hold is unavailable until ':gcal' succeeds with the robot completely still.", g_imu.calState == 3 ? PSTR("REJECTED (the robot moved)") : PSTR("TIMED OUT"));
        s.imuOk = false; startGo(ST_IR_SETTLE, now);
      }
      break;

    case ST_IR_SETTLE:
      if (!s.entered) {
        s.entered = true;
        say("START", "IR SETTLING: ignoring the receivers for %u ms (software only - no infrared LED is switched off by this firmware). Keep the robot still and the ball away.", (unsigned)c.startSettleMs);
      }
      if (elapsedMs(now, s.since) >= c.startSettleMs) { memset(s.maxSwing, 0, sizeof(s.maxSwing)); startGo(ST_IR_CHECK, now); }
      break;

    case ST_IR_CHECK: {
      if (!s.entered) { s.entered = true; say("START", "IR CHECK: measuring the receivers for %u ms to see whether an infrared signal (ball or external source) is present.", (unsigned)c.startCalMs); }
      for (uint8_t i = 0; i < RING_CH_MAX; i++) if (ringEnabled(c, i) && g_ring[i].amp > s.maxSwing[i]) s.maxSwing[i] = g_ring[i].amp;
      if (elapsedMs(now, s.since) < c.startCalMs) break;
      uint16_t act = startIrActiveMask(c);
      char t[24];
      if (ringCountEnabled(c) == 0) { s.irQuiet = true; say("START", "IR CHECK: no receiver is enabled (':ring on <n>')."); }
      else if (act) {
        startMaskText(act, t, sizeof(t));
        s.irQuiet = true;
        if (!c.ringCalDone && c.autoCalNoise) { for (uint8_t i = 0; i < RING_CH_MAX; i++) if (ringEnabled(c, i)) c.ringNoise[i] = 0; c.ringCalDone = 1; s.autoCal = true; }
        say("START", "IR CHECK: receivers %s see an IR signal while still (ball, sunlight, lamps, another robot). %S", t, PSTR("Noise floor not measured: minimum threshold used. Seeking starts anyway."));
      } else {
        s.irQuiet = true;
        if (!c.ringCalDone && c.autoCalNoise) {
          for (uint8_t i = 0; i < RING_CH_MAX; i++) if (ringEnabled(c, i)) c.ringNoise[i] = s.maxSwing[i];
          c.ringCalDone = 1; s.autoCal = true;
          say("START", "IR CHECK: receivers quiet. No saved noise floor, so it was measured now (RAM only; ':save' keeps it). Threshold = max(ringseen %u, noise x %.1f).", (unsigned)c.ringSeen, (double)c.ringNoiseX10 / 10.0);
        } else say("START", "IR CHECK: receivers quiet%S.", c.ringCalDone ? PSTR(" (saved noise floor in use)") : PSTR(" (noise floor NOT calibrated and autocal is off)"));
      }
      startGo(ST_SAFETY_CHECK, now);
      break;
    }

    case ST_SAFETY_CHECK: {
      condEvaluate();
      say("START", "SAFETY CHECK: condition %S, %u active fault(s).", condName(g_cond), (unsigned)popcount16(faultMask()));
      startupHealthReport();
      if (drivetrainFault()) { startupFault(now, "drivetrain fault (see 'h')"); break; }
      bool degraded = !s.imuOk || g_cond != COND_NORMAL || !c.ringCalDone || !s.irQuiet;
      s.readyMs = now;
      if (degraded) {
        snprintf(s.why, sizeof(s.why), "%S%S%S%S", !s.imuOk ? PSTR("IMU ") : PSTR(""), g_cond != COND_NORMAL ? PSTR("fault ") : PSTR(""), !c.ringCalDone ? PSTR("uncalibrated-IR ") : PSTR(""), !s.irQuiet ? PSTR("IR-signal") : PSTR(""));
        say("START", "READY (degraded: %s). Movement commands are enabled; check 'h'.", s.why);
        startGo(ST_READY_DEGRADED, now);
      } else {
        say("START", "READY. Commands: '?' lists them. Motor tests need ':mtest <wheel>' / ':dtest' then ':confirm' with the wheels lifted.");
        startGo(ST_READY, now);
      }
      s.waitingIr = false;
      startupArmAutostart(c, now);
      g_bootPhase = false;
      break;
    }

    case ST_READY:
    case ST_READY_DEGRADED:
      if (s.waitImu && imuHeadingUsable(c, now)) {          // the MPU6050 came up after the start-up: now the automatic start may begin
        s.waitImu = false; s.imuOk = true;
        if (s.st == ST_READY_DEGRADED && g_cond == COND_NORMAL && c.ringCalDone && s.irQuiet) s.st = ST_READY;
        say("START", "MPU6050 is now working.");
        startupArmAutostart(c, now);
      }
      if (s.waitingIr) {                                         // an IR signal was present at the start: give the automatic start a bounded chance
        static uint32_t lastQuietMs = 0;
        if (g_ball.seenMask != 0) lastQuietMs = 0; else if (!lastQuietMs) lastQuietMs = now ? now : 1u;
        if (lastQuietMs && elapsedMs(now, lastQuietMs) >= 1000) {
          s.waitingIr = false; s.irQuiet = true;
          if (!c.ringCalDone && c.autoCalNoise) { for (uint8_t i = 0; i < RING_CH_MAX; i++) if (ringEnabled(c, i)) c.ringNoise[i] = g_ring[i].amp > 0 ? g_ring[i].amp : 0; c.ringCalDone = 1; s.autoCal = true; }
          say("START", "the infrared signal cleared.");
          if (s.st == ST_READY_DEGRADED && s.imuOk && g_cond == COND_NORMAL && c.ringCalDone) { s.st = ST_READY; }
          startupArmAutostart(c, now);
        } else if (elapsedMs(now, s.readyMs) > c.irWaitMs) { s.waitingIr = false; say("START", "autostart given up: the infrared signal did not clear within %u ms. Use 'a' when ready.", (unsigned)c.irWaitMs); }
      }
      break;

    case ST_FAULT:
      if (!drivetrainFault() && elapsedMs(now, s.since) > 300) { say("START", "the drivetrain fault was cleared: re-running the safety check."); startGo(ST_SAFETY_CHECK, now); }
      break;
    default: break;
  }
}

static void printStartup(uint32_t now) {
  say("START", "state %S for %lu ms | IMU %S | IR %S%S | autostart %S | why: %s", (PGM_P)START_NAME[g_start.st], UL(elapsedMs(now, g_start.since)), g_start.imuOk ? PSTR("ok") : PSTR("NOT usable"),
      g_start.irQuiet ? PSTR("quiet") : PSTR("signal present"), g_start.autoCal ? PSTR(", noise floor measured automatically (RAM only)") : PSTR(""),
      g_start.armed ? PSTR("armed") : (g_autoStartPending ? PSTR("pending") : PSTR("not armed")), g_start.why[0] ? g_start.why : "-");
}
