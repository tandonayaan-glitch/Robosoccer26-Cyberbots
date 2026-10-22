// nav.h - navigation state machine: manual, autonomous ball seeking, search, approach/alignment, shot, motor test, safe stop.
//
// The ball is known only through the two front IR sensors (ir2.h): seen / near per sensor and a left-right offset. The robot has no
// 360-degree view: finding a ball outside the sensors' field of view is done by rotating (search).
// Frame convention: offset -1 = ball to the left, +1 = to the right, 0 = centred. Heading 0 = robot forward, clockwise positive.
// Body request: vx = strafe right, vy = forward, w = rotate clockwise.
#pragma once
#include <strings.h>
#include "logger.h"

static bool g_autoEnabled = false;          // RAM only: every reset starts with autonomy OFF
static uint32_t g_navSince = 0;
static uint32_t g_manualUntilMs = 0;
static uint8_t g_searchAttempt = 0;
static int8_t g_searchDir = 1;
static uint32_t g_searchStartMs = 0, g_lostAtMs = 0, g_pauseUntilMs = 0, g_searchBrakeUntilMs = 0;
static bool g_lostPending = false;          // a loss happened and has not been re-acquired yet
static float g_refYaw = 0;
static float g_lastOffset = 0;               // last known side of the ball (-1 left .. +1 right): search turns that way first
static uint32_t g_lastNavTick = 0;

// Goal heading. The robot is placed pointing at the goal; goalRegisterMs after power-on (gyro calibrated, robot still) its heading is registered
// as "straight at the goal". The registered heading drifts with the gyro; ':goal set' registers it again.
static float g_goalYaw = 0;
static uint32_t g_goalEpoch = 0;                // g_imu.yawEpoch when the heading was registered
static bool g_goalRefSet = false, g_goalWaitNoted = false, g_goalAutoOff = false;   // g_goalAutoOff: ':goal clear' stops the automatic registration
static uint32_t g_shootStartMs = 0, g_lastShotMs = 0, g_heldLostSinceMs = 0;
static uint8_t g_shootPhase = 0;                 // 0 aim, 1 reverse, 2 drive
static bool g_shotDone = false, g_needRelease = false;   // g_needRelease: after a shot the ball must leave the front sensor before another shot can start
static bool g_goalLostNoted = false, g_shootLaunched = false;
static uint32_t g_shotT0 = 0, g_shotLaunchMs = 0;   // when the shot started / when the wheels turned forward (for the log line)

// Half-angle (deg) inside which a shot still scores: the ball (centre) must pass within (goal width - ball diameter) / 2 of the goal centre
// at the goal line. Clamped to 2..20 deg. The shot is fired at half of it, as margin for gyro drift and ball deviation.
static float shootAimTolDeg(const Config& c) {
  float half = ((float)c.goalWidthMm - (float)c.ballDiaMm) * 0.5f;
  float deg = atanf(half / (float)c.goalDistMm) * RAD2DEG;
  return clampf(deg, 2.0f, 20.0f);
}

static bool navGoalRegister(const Config& c, uint32_t now) {
  if (!imuHeadingUsable(c, now)) return false;
  g_goalYaw = g_imu.yaw; g_goalEpoch = g_imu.yawEpoch; g_goalRefSet = true; g_goalAutoOff = true; g_goalLostNoted = false;   // registered once; never silently re-registered
  return true;
}

// The registered heading is only meaningful while the gyro reference has not been reset since (':zero', recalibration, an IMU outage).
static bool goalValid() { return g_goalRefSet && g_goalEpoch == g_imu.yawEpoch; }

// Every loop: registers the goal heading once, goalRegisterMs after power-on, as soon as the gyro is calibrated and delivering.
static void goalService(const Config& c, uint32_t now) {
  if (g_goalRefSet && !goalValid() && !g_goalLostNoted) { g_goalLostNoted = true; say("WARN", "goal heading INVALID: the gyro reference was reset since it was registered; point the robot at the goal and use ':goal set'"); }
  if (g_goalRefSet || g_goalAutoOff || now < c.goalRegisterMs) return;
  if (navGoalRegister(c, now)) say("NAV", "goal heading registered at %.1f deg of gyro yaw (%lu ms after power-on); aim window +-%.1f deg at %u mm, goal %u mm wide", (double)g_goalYaw, UL(now), (double)shootAimTolDeg(c) * 0.5, (unsigned)c.goalDistMm, (unsigned)c.goalWidthMm);
  else if (!g_goalWaitNoted) { g_goalWaitNoted = true; say("WARN", "goal heading NOT registered yet: gyro not calibrated/usable (keep the robot still); it keeps trying"); }
}

struct MotorTest {
  uint8_t state;       // 0 idle, 1 armed (needs :confirm), 2 forward/step, 3 pause, 4 reverse, 5 settle
  uint8_t wheel;
  uint8_t drive;       // 1 = whole-drivetrain test (6 body motions), 0 = single wheel
  uint8_t step;
  uint32_t until;
};
static const char* DT_STEP_NAME[6] = {"FORWARD", "BACKWARD", "STRAFE LEFT", "STRAFE RIGHT", "ROTATE CLOCKWISE", "ROTATE COUNTER-CLOCKWISE"};
static const float DT_VEC[6][3] = {{0, 1, 0}, {0, -1, 0}, {-1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, -1}};
static MotorTest g_mtest;

static void navSetState(NavState s, const char* why) {
  if (g_navState == s) return;
  uint32_t now = millis();
  if (g_navState == NAV_SEARCH) g_stats.searchTotalMs += elapsedMs(now, g_searchStartMs);
  if (s != NAV_SHOOT) g_rampBoost = false;
  say("NAV", "%s -> %s (%s)", NAV_NAME[g_navState], NAV_NAME[s], why);
  char note[24];
  snprintf(note, sizeof(note), "%s>%s", NAV_NAME[g_navState], NAV_NAME[s]);
  g_navState = s;
  g_navSince = now;
  logEvent("N", note);
}

static bool drivetrainFault() { return faultActive(F_MOTOR_INIT) || faultActive(F_LOOP_STALL); }
static bool sensingFault() { return faultActive(F_IR_FRONT); }

// 1.0 when sensing and the drivetrain are nominal, lower when degraded.
static float navSensorScale(const Config& c) {
  return g_cond == COND_DEGRADED ? c.degradedScale : 1.0f;
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
  say("NAV", "ball lost (%s): loss #%lu, last side %s (%.2f)", where, UL(g_stats.ballLossEvents), g_lastOffset > 0.15f ? "right" : (g_lastOffset < -0.15f ? "left" : "centre/unknown"), g_lastOffset);
  logEvent("N", "ball_lost");
}

static bool navEnableAuto(const Config& c, uint32_t now) {
  if (g_cond == COND_SAFE_STOP || sensingFault() || drivetrainFault()) {
    say("SAFETY", "autonomous REFUSED: robot is in SAFE_STOP (see 'h'); fix the fault and use :recover");
    return false;
  }
  if (g_mtest.state) { say("SAFETY", "autonomous REFUSED: motor test in progress"); return false; }
  if (!irEnabled(c, 0)) { say("SAFETY", "autonomous REFUSED: IR sensor 0 (front centre) is switched off (':set irenable 3')"); return false; }
  if (!irCalibrated(c, 0)) { say("SAFETY", "autonomous REFUSED: IR sensor 0 is not calibrated. Run ':ir cal clear 0' (nothing in front) and ':ir cal near 0' (ball in the shooting position), then ':save'. See docs/IR_SENSORS.md."); return false; }
  g_autoEnabled = true;
  g_searchAttempt = 0;
  g_lostPending = false;
  if (imuHeadingUsable(c, now)) { g_refYaw = g_imu.yaw; }
  motorsStopNow();
  navSetState(NAV_SEEK, "autonomous enabled by operator");
  if (c.motorVerified != 0x0F) say("WARN", "drivetrain direction NOT operator-verified for all wheels (mask 0x%X of 0xF): bench-test first (see 'h')", (unsigned)c.motorVerified);
  if (!irEnabled(c, 1) || !irCalibrated(c, 1)) say("WARN", "IR sensor 1 is %s: no centred/off-centre judgement, the shooting position uses sensor 0 only", irEnabled(c, 1) ? "not calibrated" : "switched off");
  else if (c.irOffsetSide == 0) say("WARN", "IR sensor 1 mounting side not set (':set iroffset 1' = right of sensor 0, '-1' = left): no sideways guidance");
  if (!imuHeadingUsable(c, now)) say("WARN", "heading hold unavailable: the robot may rotate while it strafes; shots need the gyro");
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

// Approach and alignment, from the two front sensors only.
//   far/medium: drive forward, slowing as the signal strengthens
//   only ONE sensor sees the ball: cautious (slow forward) and a small sideways move towards the ball's side (needs irOffsetSide)
//   both see it: sideways correction from the strength difference, until centred
//   shooting position (sensor 0 near): no forward motion; only a small sideways correction if still off-centre
static void navApproach(const Config& c, uint32_t now) {
  if (g_ball.offsetKnown) g_lastOffset = g_ball.offset;
  float scale = navSensorScale(c);
  bool seenNow = g_ball.seenMask != 0;
  float s = g_ball.s0 > g_ball.s1 ? g_ball.s0 : g_ball.s1;
  float nearFrac = (float)c.irNearPct / 100.0f;
  float t = clampf(s / nearFrac, 0.0f, 1.0f);
  float vy = c.approachSpeed + t * (c.slowSpeed - c.approachSpeed);
  float vx = 0;
  bool oneOfTwo = (g_ball.seenMask == 1 || g_ball.seenMask == 2) && irUsable(c, 0) && irUsable(c, 1);
  if (g_ball.offsetKnown && seenNow && absf(g_ball.offset) > 0.5f * c.centerBand) vx = c.alignSpeed * g_ball.offset;
  if (oneOfTwo) vy = vy < c.slowSpeed ? vy : c.slowSpeed;                 // only one sensor sees it: cautious
  if (!seenNow) { vy = 0.5f * c.slowSpeed; vx = 0; }                       // ball not seen right now (inside the loss grace): creep, do not guess
  if (g_ball.held) vy = 0;                                                  // shooting position reached: no more forward motion
  if (g_ball.ready) vx = 0;
  vx *= scale; vy *= scale;

  float w = 0;
  navHeading(c, now, &w);                                                   // keep the heading the approach started with (no turning toward the ball: there is no bearing)
  motorsDrive(c, vx, vy, w, c.maxAuto);
}

// Start a shot when the ball has been in the shooting position (sensor 0 near and centred) for shootHoldMs and the goal heading is registered.
// Returns true when the state changed.
static bool navShootCheck(const Config& c, uint32_t now) {
  bool fresh = g_irLastSampleMs && elapsedMs(now, g_irLastSampleMs) <= 100;   // a stalled sensor loop cannot start a shot
  if (fresh && !g_ball.held) g_needRelease = false;                 // the ball has left the sensor: the next ball may be shot
  if (!c.shootEnable || !g_autoEnabled || g_cond != COND_NORMAL || !goalValid() || !imuHeadingUsable(c, now) || !fresh || !g_ball.ready) return false;
  if (g_needRelease) return false;                                  // still the same ball: wait until it has left the sensor
  if (g_ball.readySinceMs == 0 || elapsedMs(now, g_ball.readySinceMs) < c.shootHoldMs) return false;
  if (g_shotDone && elapsedMs(now, g_lastShotMs) < c.shootCooldownMs) return false;
  g_shootPhase = 0; g_shootStartMs = now; g_shotT0 = now; g_heldLostSinceMs = 0; g_rampBoost = true;   // no acceleration limit during the whole shot
  navSetState(NAV_SHOOT, "ball in the shooting position: aim at the goal heading, reverse, then full speed forward");
  return true;
}

static void navEnterSearch(const char* why, bool brake = false) {
  g_searchStartMs = millis();
  // After a loss the wheels are still driving the old approach. Come to rest first (the ramp limits how fast that can be),
  // then rotate: going straight from a strafe to a spin would reverse two wheels through the dead-time pause for nothing.
  g_searchBrakeUntilMs = brake ? g_searchStartMs + 20u + (uint32_t)g_cfg.maxAuto * 1000u / (g_cfg.decelPwmPerS ? g_cfg.decelPwmPerS : 1u) : g_searchStartMs;
  g_searchDir = (g_lastOffset >= 0) ? 1 : -1;
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
  if (g_navState == NAV_SEEK || g_navState == NAV_APPROACH || g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE || g_navState == NAV_WAIT) navShootCheck(c, now);

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
      if (g_ball.valid) { g_refYaw = g_imu.yaw; navSetState(NAV_APPROACH, "ball acquired"); navApproach(c, now); }
      else if (elapsedMs(now, g_navSince) > 1000) { g_searchAttempt = 1; navEnterSearch("no ball at start"); }
      else motorsDrive(c, 0, 0, 0, c.maxAuto);
      break;

    case NAV_APPROACH:
      if (!g_ball.valid) { g_searchAttempt = 1; navEnterSearch("ball lost", true); motorsDrive(c, 0, 0, 0, c.maxAuto); }
      else navApproach(c, now);
      break;

    case NAV_SHOOT: {
      if (!g_autoEnabled) break;
      float w = 0, vx = 0, vy = 0;
      uint8_t limit = c.maxAuto;
      if (!goalValid() || !imuHeadingUsable(c, now)) {              // the heading reference is gone: stop rather than drive along a wrong heading
        motorsDrive(c, 0, 0, 0, c.maxAuto);
        g_lastShotMs = now; g_shotDone = true; g_needRelease = true;
        navSetState(NAV_SEEK, "shot cancelled: heading reference lost");
        break;
      }
      float sc = navSensorScale(c);
      uint32_t el = elapsedMs(now, g_shootStartMs);
      float err = angDiffDeg(g_goalYaw, g_imu.yaw);
      float hold = absf(err) < c.hdgDeadbandDeg ? 0.0f : clampf(c.hdgKp * err, -c.hdgMaxRot, c.hdgMaxRot);   // keep the heading
      if (g_shootPhase == 0) {                                     // aim (and, in parallel mode, reverse at the same time)
        if (!g_ball.held) { if (!g_heldLostSinceMs) g_heldLostSinceMs = now ? now : 1u; } else g_heldLostSinceMs = 0;
        if (el > c.aimTimeoutMs || (g_heldLostSinceMs && elapsedMs(now, g_heldLostSinceMs) > 400)) {
          g_lastShotMs = now; g_shotDone = true; g_needRelease = true;
          motorsDrive(c, 0, 0, 0, c.maxAuto);
          navSetState(NAV_SEEK, g_heldLostSinceMs ? "shot cancelled: ball no longer held" : "shot cancelled: aim timeout");
          break;
        }
        bool aligned = absf(err) <= 0.5f * shootAimTolDeg(c);
        bool backDone = el >= c.shootBackMs;
        if (c.shootParallel) {
          w = aligned ? hold : (err > 0 ? 1.0f : -1.0f) * clampf(0.02f * absf(err), 0.12f, c.aimSpeed) * sc;
          vy = backDone ? 0.0f : -c.shootBackSpeed * sc;
          if (aligned && backDone) { g_shootPhase = 2; g_shootStartMs = now; el = 0; g_shootLaunched = false; }
        } else if (aligned) { g_shootPhase = 1; g_shootStartMs = now; el = 0; }
        else w = (err > 0 ? 1.0f : -1.0f) * clampf(0.02f * absf(err), 0.12f, c.aimSpeed) * sc;
      }
      if (g_shootPhase == 1) {                                     // sequential mode: reverse a little with the heading held
        w = hold;
        vy = -c.shootBackSpeed * sc;
        if (el >= c.shootBackMs) { g_shootPhase = 2; g_shootStartMs = now; el = 0; g_shootLaunched = false; }
      }
      if (g_shootPhase == 2) {                                     // full power straight ahead along the goal heading
        w = hold; vy = sc; limit = c.shootPwm > c.maxAuto ? c.shootPwm : c.maxAuto;
        if (!g_shootLaunched) {                                    // the reverse-to-forward dwell must not eat the drive time:
          float fwd = 0; for (int i = 0; i < WHEEL_COUNT; i++) fwd += g_wheelCur[i];   // reversing: sum < 0, in-place rotation: sum 0, forward: sum > 0
          if (fwd > 1.0f) g_shootLaunched = true;                  // the clock starts when the wheels actually turn forward
          if (g_shootLaunched) { g_shootStartMs = now; el = 0; g_shotLaunchMs = now; }
          else if (el > 600) { g_shootLaunched = true; g_shootStartMs = now; el = 0; g_shotLaunchMs = now; }   // never wait forever (blocked output, obstacle cap)
        }
      }
            motorsDrive(c, vx, vy, w, limit);
      if (g_shootPhase == 2 && g_shootLaunched && el >= c.shootDriveMs) {
        g_lastShotMs = now; g_shotDone = true; g_needRelease = true; g_stats.shots++;
        say("NAV", "shot %lu: held-to-launch %lu ms, drive %lu ms, heading error at launch within +-%.1f deg", UL(g_stats.shots), UL(elapsedMs(g_shotLaunchMs, g_shotT0)), UL(el), (double)(0.5f * shootAimTolDeg(c)));
        navSetState(NAV_SEEK, "shot finished");
      }
      break;
    }

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
        navApproach(c, now);
      } else if (elapsedMs(now, g_searchStartMs) > c.searchTimeoutMs) {
        motorsDrive(c, 0, 0, 0, c.maxAuto);
        if (g_searchAttempt >= c.searchAttempts) navSetState(NAV_WAIT, "search attempts exhausted");
        else { g_searchAttempt++; g_pauseUntilMs = now + c.searchPauseMs; navSetState(NAV_SEARCH_PAUSE, "search timeout"); }
      } else if (!deadlineReached(now, g_searchBrakeUntilMs)) {
        motorsDrive(c, 0, 0, 0, c.maxAuto);                                                        // wheels coming to rest
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
      if (g_ball.valid) { if (imuHeadingUsable(c, now)) g_refYaw = g_imu.yaw; navSetState(NAV_APPROACH, "ball appeared"); navApproach(c, now); }
      else if (elapsedMs(now, g_navSince) > 60000) { g_autoEnabled = false; navSetState(NAV_IDLE, "no ball for 60 s: autonomous disabled"); }
      break;

    case NAV_TEST: {
      MotorTest& t = g_mtest;
      if (t.state == 1) {
        if (deadlineReached(now, t.until)) { t.state = 0; navSetState(NAV_IDLE, "motor test confirmation timed out"); }
        break;
      }
      if (t.drive) {                                   // complete drivetrain test: six body motions
        if (t.state == 2) motorsDrive(c, DT_VEC[t.step][0], DT_VEC[t.step][1], DT_VEC[t.step][2], c.testPwm);
        else motorsDrive(c, 0, 0, 0, c.testPwm);
        if (deadlineReached(now, t.until)) {
          if (t.state == 2) { t.state = 3; t.until = now + 700; }
          else if (++t.step >= 6) {
            say("TEST", "drivetrain test finished. Each motion should have matched its label. If strafe was mirrored use ':roller O'; a wrong single wheel -> ':mrev <wheel>'.");
            t.state = 0; motorsDrive(c, 0, 0, 0, c.testPwm);
            navSetState(NAV_IDLE, "drivetrain test complete");
          } else { t.state = 2; t.until = now + 900; say("TEST", "step %u/6: %s", (unsigned)t.step + 1, DT_STEP_NAME[t.step]); }
        }
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

static bool navMotorTestArm(const Config& c, int wheel, uint32_t now, bool drive = false) {
  if (wheel < 0) return false;
  if (drivetrainFault()) { say("SAFETY", "motor test REFUSED: drivetrain fault active"); return false; }
  g_autoEnabled = false;
  motorsStopNow();
  g_mtest.state = 1;
  g_mtest.wheel = (uint8_t)wheel;
  g_mtest.drive = drive ? 1 : 0;
  g_mtest.step = 0;
  g_mtest.until = now + 15000;
  navSetState(NAV_TEST, "motor test armed");
  if (drive) {
    say("TEST", "DRIVETRAIN TEST: LIFT THE WHEELS OFF THE GROUND and secure the robot. Send ':confirm' within 15 s: six 0.9 s motions at PWM %u (forward, backward, strafe left/right, rotate cw/ccw) with pauses; 's' aborts.", (unsigned)c.testPwm);
    return true;
  }
  say("TEST", "MOTOR TEST %s: LIFT THE WHEELS OFF THE GROUND and secure the robot, keep a hand on the motor-power switch. Then send ':confirm' within 15 s (PWM %u, 1 s forward then 1 s backward).",
      WHEEL_PINS[wheel].name, (unsigned)c.testPwm);
  return true;
}

static bool navMotorTestConfirm(uint32_t now) {
  if (g_mtest.state != 1) { say("TEST", "nothing to confirm (use :mtest <FL|FR|BL|BR> first)"); return false; }
  g_mtest.state = 2;
  if (g_mtest.drive) { g_mtest.step = 0; g_mtest.until = now + 900; say("TEST", "step 1/6: %s", DT_STEP_NAME[0]); return true; }
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
