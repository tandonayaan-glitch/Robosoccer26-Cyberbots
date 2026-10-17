// nav.h - navigation state machine: manual, autonomous ball seeking, search, motor test, safe stop.
//
// Frame convention: bearing 0 = robot forward, positive = clockwise (to the robot's right).
// Body request: vx = strafe right, vy = forward, w = rotate clockwise.
#pragma once
#include <strings.h>
#include "logger.h"

static bool g_autoEnabled = false;          // RAM only: every reset starts with autonomy OFF
static uint32_t g_navSince = 0;
static uint32_t g_manualUntilMs = 0;
static uint8_t g_searchAttempt = 0;
static int8_t g_searchDir = 1;
static uint32_t g_searchStartMs = 0, g_lostAtMs = 0, g_pauseUntilMs = 0;
static bool g_lostPending = false;          // a loss happened and has not been re-acquired yet
static float g_refYaw = 0;
static bool g_centered = false;
static float g_lastBearing = 0;
static uint32_t g_lastNavTick = 0;

struct MotorTest {
  uint8_t state;       // 0 idle, 1 armed (needs :confirm), 2 forward, 3 pause, 4 reverse, 5 settle
  uint8_t wheel;
  uint32_t until;
};
static MotorTest g_mtest;

static void navSetState(NavState s, const char* why) {
  if (g_navState == s) return;
  uint32_t now = millis();
  if (g_navState == NAV_SEARCH) g_stats.searchTotalMs += elapsedMs(now, g_searchStartMs);
  say("NAV", "%s -> %s (%s)", NAV_NAME[g_navState], NAV_NAME[s], why);
  char note[24];
  snprintf(note, sizeof(note), "%s>%s", NAV_NAME[g_navState], NAV_NAME[s]);
  g_navState = s;
  g_navSince = now;
  logEvent("N", note);
}

static bool drivetrainFault() { return faultActive(F_MOTOR_INIT) || faultActive(F_LOOP_STALL); }
static bool sensingFault() {
  return faultActive(F_IR_NO_CHANNELS) || faultActive(F_IR_SCAN_STALL) || faultActive(F_IR_TRUSTED_LOW);
}

// 1.0 when every sensing capability is nominal, lower when readings are less reliable.
static float navSensorScale(const Config& c) {
  float s = 1.0f;
  if (g_cond == COND_DEGRADED) s = c.degradedScale;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++)
    if (c.ch[i].enabled && g_ir[i].noisy) { s = s < c.degradedScale ? s : c.degradedScale; break; }
  return s;
}

static uint8_t navCalibratedTrusted(const Config& c) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) if (irTrusted(c, i) && c.ch[i].bearingCal) n++;
  return n;
}

static void navStop(const char* why) {
  motorsStopNow();
  g_autoEnabled = false;
  g_mtest.state = 0;
  g_hdgErr = NAN;
  if (g_cond == COND_SAFE_STOP) navSetState(NAV_SAFE_STOP, why);
  else navSetState(NAV_IDLE, why);
}

static void navMarkLoss(const char* where) {
  g_stats.ballLossEvents++;
  g_lostAtMs = millis();
  g_lostPending = true;
  say("NAV", "ball lost (%s): loss #%lu, last bearing %.0f deg", where, UL(g_stats.ballLossEvents), g_lastBearing);
  logEvent("N", "ball_lost");
}

static bool navEnableAuto(const Config& c, uint32_t now) {
  if (g_cond == COND_SAFE_STOP || sensingFault() || drivetrainFault()) {
    say("SAFETY", "autonomous REFUSED: robot is in SAFE_STOP (see 'h'); fix the fault and use :recover");
    return false;
  }
  if (g_mtest.state) { say("SAFETY", "autonomous REFUSED: motor test in progress"); return false; }
  uint8_t cal = navCalibratedTrusted(c);
  if (cal < c.minTrusted) {
    say("SAFETY", "autonomous REFUSED: only %u trusted channels have a calibrated bearing (need %u). See docs/CALIBRATION.md, ':ch <n> bear <deg>'.",
        (unsigned)cal, (unsigned)c.minTrusted);
    return false;
  }
  g_autoEnabled = true;
  g_searchAttempt = 0;
  g_lostPending = false;
  g_centered = false;
  if (imuHeadingUsable(c, now)) { g_refYaw = g_imu.yaw; }
  motorsStopNow();
  navSetState(NAV_SEEK, "autonomous enabled by operator");
  if (c.motorVerified != 0x0F) say("WARN", "drivetrain direction NOT operator-verified for all wheels (mask 0x%X of 0xF): bench-test first (see 'h')", (unsigned)c.motorVerified);
  if (!imuHeadingUsable(c, now)) say("WARN", "heading hold unavailable: using turn-toward-ball fallback");
  if (g_cond == COND_DEGRADED) say("WARN", "running DEGRADED: reduced speed (x%.2f)", navSensorScale(c));
  return true;
}

static bool navHeading(const Config& c, uint32_t now, float* w) {
  if (!imuHeadingUsable(c, now)) { g_hdgErr = NAN; return false; }
  float err = angDiffDeg(g_refYaw, g_imu.yaw);     // reference - current: >0 means turn clockwise
  g_hdgErr = err;
  *w = (absf(err) < c.hdgDeadbandDeg) ? 0.0f : clampf(c.hdgKp * err, -c.hdgMaxRot, c.hdgMaxRot);
  return true;
}

static void navApproach(const Config& c, uint32_t now) {
  float th = g_ball.bearingDeg;
  g_lastBearing = th;
  float scale = navSensorScale(c);
  float speed;
  bool healthy = (g_cond == COND_NORMAL) && !g_ball.ambiguous;
  if (g_ball.conf >= c.aggressiveConf && healthy) {
    speed = c.attackSpeed;                                  // aggressive only with high confidence AND healthy sensing
  } else {
    float t = clampf((g_ball.conf - c.acqConf) / (1.0f - c.acqConf), 0.0f, 1.0f);
    speed = c.slowSpeed + t * (c.approachSpeed - c.slowSpeed);
  }
  if (g_ball.ambiguous || elapsedMs(now, g_ball.lastDetectMs) > c.lostHoldMs / 2) speed = speed < c.slowSpeed ? speed : c.slowSpeed;
  if (absf(th) > 120.0f) speed = speed < c.slowSpeed ? speed : c.slowSpeed;   // ball behind: creep
  speed *= scale;

  // Centre-band hysteresis: forward drive when centred, strafe when off-centre.
  if (g_centered && absf(th) > c.centerDeg * 1.3f) g_centered = false;
  else if (!g_centered && absf(th) < c.centerDeg * 0.7f) g_centered = true;
  float s = sinf(th * DEG2RAD), co = cosf(th * DEG2RAD);
  float vx = speed * s, vy = speed * co;
  if (g_centered) { vy = speed; vx = speed * s * 0.5f; }

  float w = 0;
  if (!navHeading(c, now, &w)) {
    w = clampf(c.turnKp * th, -c.hdgMaxRot, c.hdgMaxRot);   // fallback: robot-relative turn toward the ball
    if (absf(th) > c.fallbackAlignDeg) { vx *= 0.5f; vy *= 0.5f; }
  }
  motorsDrive(c, vx, vy, w, c.maxAuto);
}

static void navEnterSearch(const char* why) {
  g_searchStartMs = millis();
  g_searchDir = (g_lastBearing >= 0) ? 1 : -1;
  navSetState(NAV_SEARCH, why);
}

// 100 Hz control tick (called with the loop's timestamp).
static void navTick(const Config& c, uint32_t now) {
  g_lastNavTick = now;
  if (g_ball.lostEvent) {
    g_ball.lostEvent = false;
    if (g_autoEnabled && g_navState == NAV_APPROACH) navMarkLoss("approach");
  }
  if (g_autoEnabled && (g_cond == COND_SAFE_STOP || drivetrainFault())) navStop("safe stop");

  switch (g_navState) {
    case NAV_IDLE:
    case NAV_SAFE_STOP:
      g_hdgErr = NAN;
      motorsDrive(c, 0, 0, 0, c.maxManual);
      break;

    case NAV_MANUAL:
      if (deadlineReached(now, g_manualUntilMs)) { motorsDrive(c, 0, 0, 0, c.maxManual); navSetState(g_cond == COND_SAFE_STOP ? NAV_SAFE_STOP : NAV_IDLE, "manual timeout"); }
      break;

    case NAV_SEEK:
      if (g_ball.valid) { g_refYaw = g_imu.yaw; navSetState(NAV_APPROACH, "ball acquired"); }
      else if (elapsedMs(now, g_navSince) > 1000) { g_searchAttempt = 1; navEnterSearch("no ball at start"); }
      else motorsDrive(c, 0, 0, 0, c.maxAuto);
      break;

    case NAV_APPROACH:
      if (!g_ball.valid) { g_searchAttempt = 1; navEnterSearch("ball lost"); }
      else navApproach(c, now);
      break;

    case NAV_SEARCH:
      g_hdgErr = NAN;
      if (g_ball.valid) {
        if (g_lostPending) {
          g_stats.reacquisitions++;
          g_stats.reacquireTotalMs += elapsedMs(now, g_lostAtMs);
          g_lostPending = false;
          say("NAV", "ball re-acquired after %lu ms", UL(elapsedMs(now, g_lostAtMs)));
        }
        if (imuHeadingUsable(c, now)) g_refYaw = g_imu.yaw;
        navSetState(NAV_APPROACH, "ball re-acquired");
      } else if (elapsedMs(now, g_searchStartMs) > c.searchTimeoutMs) {
        motorsDrive(c, 0, 0, 0, c.maxAuto);
        if (g_searchAttempt >= c.searchAttempts) navSetState(NAV_WAIT, "search attempts exhausted");
        else { g_searchAttempt++; g_pauseUntilMs = now + c.searchPauseMs; navSetState(NAV_SEARCH_PAUSE, "search timeout"); }
      } else {
        motorsDrive(c, 0, 0, (float)g_searchDir * c.searchSpin * navSensorScale(c), c.maxAuto);   // slow rotation
      }
      break;

    case NAV_SEARCH_PAUSE:
      motorsDrive(c, 0, 0, 0, c.maxAuto);
      if (g_ball.valid) navSetState(NAV_SEARCH, "ball seen during pause");
      else if (deadlineReached(now, g_pauseUntilMs)) { g_searchDir = (int8_t)-g_searchDir; g_searchStartMs = now; navSetState(NAV_SEARCH, "retry search, opposite direction"); }
      break;

    case NAV_WAIT:                                   // stopped, still watching; never spins forever
      motorsDrive(c, 0, 0, 0, c.maxAuto);
      if (g_ball.valid) { if (imuHeadingUsable(c, now)) g_refYaw = g_imu.yaw; navSetState(NAV_APPROACH, "ball appeared"); }
      else if (elapsedMs(now, g_navSince) > 60000) { g_autoEnabled = false; navSetState(NAV_IDLE, "no ball for 60 s: autonomous disabled"); }
      break;

    case NAV_TEST: {
      MotorTest& t = g_mtest;
      if (t.state == 1) {
        if (deadlineReached(now, t.until)) { t.state = 0; navSetState(NAV_IDLE, "motor test confirmation timed out"); }
        break;
      }
      for (int i = 0; i < WHEEL_COUNT; i++) motorsSetWheelTarget((uint8_t)i, 0);
      if (t.state == 2) motorsSetWheelTarget(t.wheel, (float)c.testPwm);
      if (t.state == 4) motorsSetWheelTarget(t.wheel, -(float)c.testPwm);
      if (deadlineReached(now, t.until)) {
        if (t.state == 2) { t.state = 3; t.until = now + 600; }
        else if (t.state == 3) { t.state = 4; t.until = now + 1000; }
        else if (t.state == 4) { t.state = 5; t.until = now + 600; }
        else if (t.state == 5) {
          say("TEST", "%s test finished. Did the wheel turn FORWARD first (robot-forward direction) and then BACKWARD? If forward was wrong: ':mrev %s'; if correct: ':mok %s'.",
              WHEEL_PINS[t.wheel].name, WHEEL_PINS[t.wheel].name, WHEEL_PINS[t.wheel].name);
          t.state = 0;
          navSetState(NAV_IDLE, "motor test complete");
        }
      }
      break;
    }
  }
}

// Manual keys. Manual control always cancels autonomy until 'a' is sent again.
static void navManual(const Config& c, char key, uint32_t now) {
  if (drivetrainFault()) { say("SAFETY", "manual motion REFUSED: drivetrain fault active (use :recover after fixing)"); return; }
  g_autoEnabled = false;
  g_mtest.state = 0;
  g_hdgErr = NAN;
  float vx = 0, vy = 0, w = 0;
  switch (key) {
    case 'f': vy = 1; break;
    case 'b': vy = -1; break;
    case 'l': vx = -1; break;      // strafe left
    case 'r': vx = 1; break;       // strafe right
    case 'q': w = -1; break;       // counter-clockwise
    case 'e': w = 1; break;        // clockwise
    default: return;
  }
  if (g_cond == COND_SAFE_STOP) sayRL(1, 5000, "SAFETY", "manual motion in SAFE_STOP (sensing fault): operator responsibility, autonomy stays off");
  motorsDrive(c, vx, vy, w, c.maxManual);
  g_manualUntilMs = now + c.manualHoldMs;
  navSetState(NAV_MANUAL, "manual command");
}

static int wheelByName(const char* s) {
  for (int i = 0; i < WHEEL_COUNT; i++) if (strcasecmp(s, WHEEL_PINS[i].name) == 0) return i;
  return -1;
}

static bool navMotorTestArm(const Config& c, int wheel, uint32_t now) {
  if (wheel < 0) return false;
  if (drivetrainFault()) { say("SAFETY", "motor test REFUSED: drivetrain fault active"); return false; }
  g_autoEnabled = false;
  motorsStopNow();
  g_mtest.state = 1;
  g_mtest.wheel = (uint8_t)wheel;
  g_mtest.until = now + 15000;
  navSetState(NAV_TEST, "motor test armed");
  say("TEST", "MOTOR TEST %s: LIFT THE WHEELS OFF THE GROUND and secure the robot, keep a hand on the motor-power switch. Then send ':confirm' within 15 s (PWM %u, 1 s forward then 1 s backward).",
      WHEEL_PINS[wheel].name, (unsigned)c.testPwm);
  return true;
}

static bool navMotorTestConfirm(uint32_t now) {
  if (g_mtest.state != 1) { say("TEST", "nothing to confirm (use :mtest <FL|FR|BL|BR> first)"); return false; }
  g_mtest.state = 2;
  g_mtest.until = now + 1000;
  say("TEST", "running %s forward... (send 's' or '!' to abort)", WHEEL_PINS[g_mtest.wheel].name);
  return true;
}

// Safety-condition hook called from health.h
static void onCondChange(Cond from, Cond to) {
  if (to == COND_SAFE_STOP) {
    g_autoEnabled = false;
    g_mtest.state = 0;
    motorsStopNow();
    say("SAFETY", "SAFE_STOP entered: autonomous movement disabled, motors stopped. Diagnostics stay available ('h' for cause).");
    logEvent("E", "safe_stop");
    frTrigger("safe_stop");
    navSetState(NAV_SAFE_STOP, "critical fault");
  } else if (from == COND_SAFE_STOP) {
    say("SAFETY", "SAFE_STOP cleared (%s). Autonomous remains DISABLED: send 'a' after reviewing 'h'.", condName(to));
    if (g_navState == NAV_SAFE_STOP) navSetState(NAV_IDLE, "fault cleared");
  } else {
    say("INFO", "operating condition %s -> %s", condName(from), condName(to));
  }
}
