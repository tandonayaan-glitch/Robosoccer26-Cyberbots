// nav.h - navigation state machine: manual, autonomous ball seeking, search, approach/alignment, shot, motor test, safe stop.
//
// The ball is known through the phototransistor ring (ring.h): a bearing and a strength while a channel sees it, plus the front IR module
// ("in the shooting position"). The ring has blind arcs between its receivers: a ball that disappears into one is looked for by rotating
// the way it was last moving / towards the nearest uncovered arc (search).
// Frame convention: bearing 0 = ahead, + = to the right (clockwise), +-180 = behind. Heading 0 = robot forward, clockwise positive.
// Body request: vx = strafe right, vy = forward, w = rotate clockwise.
#pragma once
#include "logger.h"

static bool g_autoEnabled = false;          // RAM only: every reset starts with autonomy OFF
static bool g_estop = false;                // emergency-stop LATCH (RAM only): while set, no movement and no autonomy can be started from anywhere
static bool g_autoStartPending = false;       // boot decided to start autonomy by itself once the start delay has passed
static uint32_t g_autoStartAtMs = 0;
static bool g_defendEnabled = false;          // defend mode is active (defend.h); exclusive with autonomous attack and manual driving
static void defendTick(const Config& c, uint32_t now);
static bool startupGate(const char* what);
static void autoDetailsReport();                              // report.h: full sensor report, printed when the autonomous mode starts
static void hdgTestTick(const Config& c, uint32_t now);            // startup.h: false (with the reason printed) until the start-up reached READY
static bool g_remoteManual = false;         // the remote asked for manual mode (M); cleared by U, X and a reset
static uint32_t g_autoStartMs = 0, g_approachSinceMs = 0, g_reSince = 0, g_reLockUntil = 0, g_waitHoldUntil = 0;
static uint8_t g_rePhase = 0, g_reCount = 0, g_appTimeouts = 0;
static uint32_t g_navSince = 0;
static uint32_t g_manualUntilMs = 0;
static uint8_t g_searchAttempt = 0;
static int8_t g_searchDir = 1;
static uint32_t g_searchStartMs = 0, g_lostAtMs = 0, g_pauseUntilMs = 0, g_searchBrakeUntilMs = 0;
static bool g_lostPending = false;          // a loss happened and has not been re-acquired yet
static float g_refYaw = 0;
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
// Floor test of the heading controller (':hdgtest'): measured yaw response to a rotation, then a straight drive without and with the correction.
struct HdgTest { uint8_t state, step; uint32_t until; float yaw0, res[4]; };
static HdgTest g_hdgt;                                 // state 0 idle, 1 armed (needs :confirm), 2 running a step, 3 settling after a step
// Hand-rotation check of the yaw SIGN (':yawcheck'): the operator turns the robot clockwise by hand; nothing moves by itself.
struct YawCheck { uint8_t state; uint32_t until, quietSince; float yaw0; };
static YawCheck g_yawck;
static float g_manVx = 0, g_manVy = 0;                 // the translation of the manual command being held straight
static bool g_manHold = false;
static const char DT_STEP_NAME[6][26] PROGMEM = {"FORWARD", "BACKWARD", "STRAFE LEFT", "STRAFE RIGHT", "ROTATE CLOCKWISE", "ROTATE COUNTER-CLOCKWISE"};
static const float DT_VEC[6][3] PROGMEM = {{0, 1, 0}, {0, -1, 0}, {-1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, -1}};
static MotorTest g_mtest;

static void navSetState(NavState s, PGM_P why) {
  if (g_navState == s) return;
  uint32_t now = millis();
  if (g_navState == NAV_SEARCH) g_stats.searchTotalMs += elapsedMs(now, g_searchStartMs);
  if (s != NAV_SHOOT) g_rampBoost = false;
  say("NAV", "%S -> %S (%S)", (PGM_P)NAV_NAME[g_navState], (PGM_P)NAV_NAME[s], why);
  char note[28];
  snprintf(note, sizeof(note), "%S>%S", (PGM_P)NAV_NAME[g_navState], (PGM_P)NAV_NAME[s]);
  if (s == NAV_APPROACH && g_navState != NAV_APPROACH) g_approachSinceMs = now ? now : 1u;
  recAdd(REC_NAV, (uint16_t)(((uint16_t)g_navState << 8) | (uint8_t)s));
  g_navState = s;
  g_navSince = now;
  logEvent("N", note);
}

static bool drivetrainFault() { return faultActive(F_MOTOR_INIT) || faultActive(F_LOOP_STALL); }
static bool sensingFault() { return faultActive(F_RING_NONE) || faultActive(F_RING_STALL); }

// 1.0 when sensing and the drivetrain are nominal, lower when degraded.
static float navSensorScale(const Config& c) {
  return g_cond == COND_DEGRADED ? c.degradedScale : 1.0f;
}

static void navStop(PGM_P why) {
  motorsStopNow();
  g_autoEnabled = false;
  g_defendEnabled = false;
  g_mtest.state = 0;
  g_hdgt.state = 0; g_yawck.state = 0; g_manHold = false;
  g_hdgErr = NAN;
  if (g_cond == COND_SAFE_STOP) navSetState(NAV_SAFE_STOP, why);
  else navSetState(NAV_IDLE, why);
}

// Emergency stop: everything stops NOW and a latch keeps the robot from moving until it is cleared deliberately (remote 'R' or ':recover').
static void navEstop(PGM_P why) {
  g_estop = true;
  g_remoteManual = false;
  navStop(why);
  motorsStopNow();
  recAdd(REC_ESTOP, 1);
}
static bool navEstopRefuses(PGM_P what) {
  if (!g_estop) return false;
  say("SAFETY", "%S REFUSED: emergency stop is LATCHED (remote 'R' or ':recover' clears it, then choose manual or autonomous explicitly)", what);
  return true;
}

static void navMarkLoss(PGM_P where) {
  g_stats.ballLossEvents++;
  g_lostAtMs = millis();
  g_lostPending = true;
  say("NAV", "ball lost (%S): loss #%lu, last bearing %.0f deg%s, moving %.0f deg/s", where, UL(g_stats.ballLossEvents), (double)g_lastBearing, ringInBlindSpot(g_cfg, g_lastBearing) ? " (inside a blind arc)" : "", (double)g_bearingRate);
  logEvent("N", "ball_lost");
  recAdd(REC_BALL_LOST, (uint16_t)(int16_t)g_lastBearing);
}

static bool navEnableAuto(const Config& c, uint32_t now) {
  if (navEstopRefuses(PSTR("autonomous"))) return false;
  if (!startupGate("autonomous")) return false;
  if (g_cond == COND_SAFE_STOP || sensingFault() || drivetrainFault()) {
    say("SAFETY", "autonomous REFUSED: robot is in SAFE_STOP (see 'h'); fix the fault and use :recover");
    return false;
  }
  if (g_mtest.state) { say("SAFETY", "autonomous REFUSED: motor test in progress"); return false; }
  if (ringCountUsable(c) == 0) { say("SAFETY", "autonomous REFUSED: no usable ball-ring channel (':ring' shows which are off or saturated)"); return false; }
  if (!c.ringCalDone) { say("SAFETY", "autonomous REFUSED: ring noise floor not calibrated. Ball OFF, then ':ring cal noise' and ':save' (docs/RING_AND_MODULES.md)."); return false; }
  if (!modEnabled(c, 0)) { say("SAFETY", "autonomous REFUSED: IR module 0 (front, shooting position) is switched off (':mod on 0'): the robot could not tell when the ball is in place"); return false; }
  g_defendEnabled = false;
  g_remoteManual = false;
  g_autoEnabled = true;
  g_autoStartMs = now ? now : 1u;
  g_reCount = 0; g_appTimeouts = 0; g_reLockUntil = 0; g_waitHoldUntil = 0;
  g_searchAttempt = 0;
  g_lostPending = false;
  if (imuHeadingUsable(c, now)) { g_refYaw = g_imu.yaw; }
  motorsStopNow();
  navSetState(NAV_SEEK, PSTR("autonomous enabled by operator"));
  if (c.motorVerified != 0x0F) say("WARN", "drivetrain direction NOT operator-verified for all wheels (mask 0x%X of 0xF): bench-test first (see 'h')", (unsigned)c.motorVerified);
  { uint8_t nU; float gap = ringCoverageGap(c, &nU);
    if (gap > 0.0f) say("WARN", "ring blind arc ~%.0f deg (estimate from configured bearings): a ball inside it is only found by rotating", (double)gap); }
  if (c.ringNear == 375) say("WARN", "ringnear is the untouched default: strength (and so the slow-down) is a guess until ':ring cal near' is done with the ball in the shooting position");
  if (!imuHeadingUsable(c, now)) say("WARN", "heading hold unavailable: the robot may rotate while it strafes; shots need the gyro");
  if (g_cond == COND_DEGRADED) say("WARN", "running DEGRADED: reduced speed (x%.2f)", navSensorScale(c));
  autoDetailsReport();
  return true;
}

static bool navHeading(const Config& c, uint32_t now, float* w) {
  if (!imuHeadingUsable(c, now)) { g_hdgErr = NAN; return false; }
  float err = angDiffDeg(g_refYaw, g_imu.yaw);     // reference - current: >0 means turn clockwise
  g_hdgErr = err;
  *w = (absf(err) < c.hdgDeadbandDeg) ? 0.0f : clampf(c.hdgKp * err, -c.hdgMaxRot, c.hdgMaxRot);
  return true;
}

// Approach and alignment from the ring bearing.
//   ball far off to one side or behind (|bearing| > turndeg): turn on the spot towards it first (a little creep forward only when it is ahead)
//   roughly ahead: drive forward, slowing as the signal strengthens, steering gently towards the bearing
//   close (strong signal): keep the heading and strafe to centre the ball instead of turning
//   front module detecting (shooting position): no forward motion; only a small sideways correction if still off-centre
//   ball not seen right now (inside the loss grace): creep, do not guess
static void navApproach(const Config& c, uint32_t now) {
  float scale = navSensorScale(c);
  bool seenNow = g_ball.bearingKnown;
  float brg = seenNow ? g_ball.bearing : g_lastBearing;
  float t = clampf(g_ball.strength, 0.0f, 1.0f);
  float vy = c.approachSpeed + t * (c.slowSpeed - c.approachSpeed);
  float vx = 0, w = 0;
  bool turning = false;
  if (!seenNow) { vy = 0.5f * c.slowSpeed; }
  else if (absf(brg) > c.turnDeg) {                                          // turn towards it first
    w = clampf(c.turnKp * brg, -c.turnMax, c.turnMax);
    if (absf(w) < 0.12f) w = (brg > 0 ? 0.12f : -0.12f);                     // enough to move the wheels
    vy = absf(brg) < 60.0f ? 0.5f * c.slowSpeed : 0.0f;
    turning = true;
  } else if (absf(brg) > 0.5f * c.centerDeg) {
    if (t >= 0.5f || g_ball.held) vx = c.alignSpeed * clampf(brg / c.turnDeg, -1.0f, 1.0f);   // close: strafe, keep the heading
    else { w = clampf(c.turnKp * brg, -c.turnMax, c.turnMax); turning = true; }                // far: steer
  }
  if (g_ball.held) vy = 0;                                                  // shooting position reached: no more forward motion
  if (g_ball.ready) vx = 0;
  vx *= scale; vy *= scale;

  if (turning) {                                                            // the robot follows the ball: the heading to hold is wherever it points after the turn
    if (imuHeadingUsable(c, now)) g_refYaw = g_imu.yaw;
    g_hdgErr = NAN;
    w *= scale;
  } else navHeading(c, now, &w);
  motorsDrive(c, vx, vy, w, c.maxAuto);
}

// Start a shot when the ball has been in the shooting position (sensor 0 near and centred) for shootHoldMs and the goal heading is registered.
// Returns true when the state changed.
static bool navShootCheck(const Config& c, uint32_t now) {
  bool fresh = g_ringLastScanMs && elapsedMs(now, g_ringLastScanMs) <= 100;   // a stalled sensor loop cannot start a shot
  if (fresh && !g_ball.held) g_needRelease = false;                 // the ball has left the front module: the next ball may be shot
  bool goalOk = goalValid() && imuHeadingUsable(c, now);
  if (!c.shootEnable || !g_autoEnabled || g_cond != COND_NORMAL || (c.shootNeedGoal && !goalOk) || !fresh || !g_ball.ready) return false;
  if (!deadlineReached(now, g_reLockUntil)) return false;           // reverse-and-reattack tries used up: stay put for reCooldownMs
  if (g_needRelease) return false;                                  // still the same ball: wait until it has left the sensor
  if (g_ball.readySinceMs == 0 || elapsedMs(now, g_ball.readySinceMs) < c.shootHoldMs) return false;
  if (g_shotDone && elapsedMs(now, g_lastShotMs) < c.shootCooldownMs) return false;
  g_shootPhase = 0; g_shootStartMs = now; g_shotT0 = now; g_heldLostSinceMs = 0; g_rampBoost = true;   // no acceleration limit during the whole shot
  navSetState(NAV_SHOOT, PSTR("ball in the shooting position: aim at the goal heading, reverse, then full speed forward"));
  return true;
}

// Which way to rotate to find a ball that has disappeared. Priority: the way it was moving across the view (it left through that side), then
// the nearest uncovered arc on either side of where it was last seen (it can only have slipped into a blind spot), then the side of its last bearing.
static int8_t navPickSearchDir(const Config& c) {
  if (absf(g_bearingRate) >= 20.0f) return g_bearingRate > 0 ? 1 : -1;
  for (int d = 5; d <= 120; d += 5) {
    bool r = ringInBlindSpot(c, g_lastBearing + (float)d), l = ringInBlindSpot(c, g_lastBearing - (float)d);
    if (r != l) return r ? 1 : -1;
    if (r && l) break;
  }
  return g_lastBearing >= 0 ? 1 : -1;
}

static void navEnterSearch(PGM_P why, bool brake = false) {
  g_searchStartMs = millis();
  // After a loss the wheels are still driving the old approach. Come to rest first (the ramp limits how fast that can be),
  // then rotate: going straight from a strafe to a spin would reverse two wheels through the dead-time pause for nothing.
  g_searchBrakeUntilMs = brake ? g_searchStartMs + 20u + (uint32_t)g_cfg.maxAuto * 1000u / (g_cfg.decelPwmPerS ? g_cfg.decelPwmPerS : 1u) : g_searchStartMs;
  g_searchDir = navPickSearchDir(g_cfg);
  navSetState(NAV_SEARCH, why);
}

// 100 Hz control tick (called with the loop's timestamp).
static void navTick(const Config& c, uint32_t now) {
  g_lastNavTick = now;
  if (g_ball.lostEvent) {
    g_ball.lostEvent = false;
    if (g_autoEnabled && g_navState == NAV_APPROACH) navMarkLoss(PSTR("approach"));
  }
  if (g_autoEnabled && (g_cond == COND_SAFE_STOP || drivetrainFault())) navStop(PSTR("safe stop"));
  if (g_navState == NAV_SEEK || g_navState == NAV_APPROACH || g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE || g_navState == NAV_WAIT) navShootCheck(c, now);

  switch (g_navState) {
    case NAV_IDLE:
    case NAV_SAFE_STOP:
      g_hdgErr = NAN;
      motorsDrive(c, 0, 0, 0, c.maxManual);
      break;

    case NAV_MANUAL:
      if (g_manHold && !deadlineReached(now, g_manualUntilMs)) {          // straight / strafe command: keep the gyro heading (bounded by hdgmax; off with ':set hdgmanual 0')
        float wm = 0;
        if (navHeading(c, now, &wm)) motorsDrive(c, g_manVx, g_manVy, wm, c.maxManual); else g_manHold = false;     // IMU stale or invalid: plain manual driving
      }
      if (deadlineReached(now, g_manualUntilMs)) { motorsDrive(c, 0, 0, 0, c.maxManual); navSetState(g_cond == COND_SAFE_STOP ? NAV_SAFE_STOP : NAV_IDLE, "manual timeout"); }
      break;

    case NAV_SEEK:
      if (g_ball.valid) { g_refYaw = g_imu.yaw; navSetState(NAV_APPROACH, PSTR("ball acquired")); navApproach(c, now); }
      else if (elapsedMs(now, g_navSince) > 1000) { g_searchAttempt = 1; navEnterSearch(PSTR("no ball at start")); }
      else motorsDrive(c, 0, 0, 0, c.maxAuto);
      break;

    case NAV_APPROACH:
      if (!g_ball.valid) { g_searchAttempt = 1; navEnterSearch(PSTR("ball lost"), true); motorsDrive(c, 0, 0, 0, c.maxAuto); }
      else if (g_ball.held) { g_appTimeouts = 0; g_approachSinceMs = now ? now : 1u; navApproach(c, now); }   // reached it: the timer restarts
      else if (elapsedMs(now, g_approachSinceMs) > c.approachTimeoutMs) {
        // Approaching for this long without the front module ever reporting the ball: a false detection (or a ball that keeps rolling away). Search again;
        // after three such timeouts in a row stop and pause instead of chasing it forever.
        g_stats.ballLossEvents++;
        if (++g_appTimeouts >= 3) { g_appTimeouts = 0; g_waitHoldUntil = now + c.reCooldownMs; motorsDrive(c, 0, 0, 0, c.maxAuto); navSetState(NAV_WAIT, PSTR("approach never reached the ball 3 times: pausing")); }
        else { g_searchAttempt = 1; navEnterSearch(PSTR("approach timeout: not reached, treating as a false detection"), true); motorsDrive(c, 0, 0, 0, c.maxAuto); }
      }
      else navApproach(c, now);
      break;

    case NAV_SHOOT: {
      if (!g_autoEnabled) break;
      float w = 0, vx = 0, vy = 0;
      uint8_t limit = c.maxAuto;
      bool goalOk = goalValid() && imuHeadingUsable(c, now);
      if (c.shootNeedGoal && !goalOk) {                             // the heading reference is gone: stop rather than drive along a wrong heading
        motorsDrive(c, 0, 0, 0, c.maxAuto);
        g_lastShotMs = now; g_shotDone = true; g_needRelease = true;
        navSetState(NAV_SEEK, PSTR("shot cancelled: heading reference lost"));
        break;
      }
      float sc = navSensorScale(c);
      uint32_t el = elapsedMs(now, g_shootStartMs);
      float err = goalOk ? angDiffDeg(g_goalYaw, g_imu.yaw) : 0.0f;    // no goal heading (shootneedgoal 0): a straight drive burst along the current heading
      float hold = absf(err) < c.hdgDeadbandDeg ? 0.0f : clampf(c.hdgKp * err, -c.hdgMaxRot, c.hdgMaxRot);   // keep the heading
      if (g_shootPhase == 0) {                                     // aim (and, in parallel mode, reverse at the same time)
        if (!g_ball.held) { if (!g_heldLostSinceMs) g_heldLostSinceMs = now ? now : 1u; } else g_heldLostSinceMs = 0;
        if (el > c.aimTimeoutMs || (g_heldLostSinceMs && elapsedMs(now, g_heldLostSinceMs) > 400)) {
          g_lastShotMs = now; g_shotDone = true; g_needRelease = true;
          motorsDrive(c, 0, 0, 0, c.maxAuto);
          navSetState(NAV_SEEK, g_heldLostSinceMs ? PSTR("shot cancelled: ball no longer held") : PSTR("shot cancelled: aim timeout"));
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
        g_lastShotMs = now; g_shotDone = true; g_needRelease = true; g_stats.shots++; recAdd(REC_SHOT, (uint16_t)g_stats.shots);
        say("NAV", "shot %lu: held-to-launch %lu ms, drive %lu ms, heading error at launch within +-%.1f deg", UL(g_stats.shots), UL(elapsedMs(g_shotLaunchMs, g_shotT0)), UL(el), (double)(0.5f * shootAimTolDeg(c)));
        g_rePhase = 0; g_reSince = now ? now : 1u;
        navSetState(NAV_REVERSE, PSTR("shot finished: judging the result"));
      }
      break;
    }

    // Did the push work? Judge after reJudgeMs: the ball gone = success. The ball still in front = the push failed: reverse a short bounded distance, settle,
    // then look again (normal tracking picks the ball up and attacks again), at most reMax times in a row, then stay put for reCooldownMs. Every phase is
    // non-blocking and is interrupted by: ball gone / moved, E-stop, a fault (the generic checks above), or any remote/CLI command (they call navStop).
    case NAV_REVERSE: {
      float sc = navSensorScale(c);
      uint32_t el = elapsedMs(now, g_reSince);
      if (g_rePhase == 0) {                                        // judge (standing still)
        motorsDrive(c, 0, 0, 0, c.maxAuto);
        bool still = g_ball.held || (underUsable(c) && g_under.active);
        if (!g_ball.valid || (!still && el >= 60)) { g_reCount = 0; g_shotDone = false; navSetState(NAV_SEEK, PSTR("attack worked: the ball is gone from the front")); break; }
        if (el >= c.reJudgeMs) {
          if (g_reCount >= c.reMax) {
            g_reCount = 0; g_reLockUntil = now + c.reCooldownMs; g_waitHoldUntil = g_reLockUntil;
            navSetState(NAV_WAIT, PSTR("attack retries used up: pausing before the next attempt"));
          } else { g_reCount++; recAdd(REC_REATTACK, g_reCount); g_rePhase = 1; g_reSince = now; say("NAV", "push failed (ball still in front): reversing, try %u of %u", (unsigned)g_reCount, (unsigned)c.reMax); }
        }
      } else if (g_rePhase == 1) {                                 // reverse a short, bounded distance
        if (!g_ball.valid) { g_stats.ballLossEvents++; g_searchAttempt = 1; navEnterSearch(PSTR("ball lost while reversing"), true); break; }
        motorsDrive(c, 0, -c.reBackSpeed * sc, 0, c.maxAuto);
        bool moved = g_ball.bearingKnown && absf(g_ball.bearing) > 1.5f * c.turnDeg;       // the ball moved away from the front: reposition at once
        if (el >= c.reBackMs || moved) { g_rePhase = 2; g_reSince = now; }
      } else {                                                     // settle, then re-evaluate with normal tracking
        motorsDrive(c, 0, 0, 0, c.maxAuto);
        if (el >= c.reSettleMs) { g_shotDone = false; g_needRelease = false; navSetState(NAV_SEEK, PSTR("re-evaluating the ball after the reverse")); }
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
        navSetState(NAV_APPROACH, PSTR("ball re-acquired"));
        navApproach(c, now);
      } else if (elapsedMs(now, g_searchStartMs) > c.searchTimeoutMs) {
        motorsDrive(c, 0, 0, 0, c.maxAuto);
        if (g_searchAttempt >= c.searchAttempts) navSetState(NAV_WAIT, PSTR("search attempts exhausted"));
        else { g_searchAttempt++; g_pauseUntilMs = now + c.searchPauseMs; navSetState(NAV_SEARCH_PAUSE, PSTR("search timeout")); }
      } else if (!deadlineReached(now, g_searchBrakeUntilMs)) {
        motorsDrive(c, 0, 0, 0, c.maxAuto);                                                        // wheels coming to rest
      } else {
        motorsDrive(c, 0, 0, (float)g_searchDir * c.searchSpin * navSensorScale(c), c.maxAuto);   // slow rotation
      }
      break;

    case NAV_SEARCH_PAUSE:
      motorsDrive(c, 0, 0, 0, c.maxAuto);
      if (g_ball.valid) navSetState(NAV_SEARCH, PSTR("ball seen during pause"));
      else if (deadlineReached(now, g_pauseUntilMs)) { g_searchDir = (int8_t)-g_searchDir; g_searchStartMs = now; navSetState(NAV_SEARCH, PSTR("retry search, opposite direction")); }
      break;

    case NAV_WAIT:                                   // stopped, still watching; never spins forever
      motorsDrive(c, 0, 0, 0, c.maxAuto);
      if (g_ball.valid) { if (deadlineReached(now, g_waitHoldUntil)) { if (imuHeadingUsable(c, now)) g_refYaw = g_imu.yaw; navSetState(NAV_APPROACH, PSTR("ball appeared")); navApproach(c, now); } }   // (after a failed attack or approach the pause is kept)
      else if (elapsedMs(now, g_ball.lastSeenMs ? g_ball.lastSeenMs : g_autoStartMs) > c.giveUpMs) { g_autoEnabled = false; navSetState(NAV_IDLE, PSTR("no ball for giveupms: autonomous disabled")); }
      else if (elapsedMs(now, g_navSince) > c.researchMs) { g_searchAttempt = 1; navEnterSearch(PSTR("fresh search after the pause")); }   // never stay blind for good
      break;

    case NAV_DEFEND: defendTick(c, now); break;

    case NAV_COUNT: break;

    case NAV_TEST: {
      if (g_hdgt.state) { hdgTestTick(c, now); break; }
      MotorTest& t = g_mtest;
      if (t.state == 1) {
        if (deadlineReached(now, t.until)) { t.state = 0; navSetState(NAV_IDLE, PSTR("motor test confirmation timed out")); }
        break;
      }
      if (t.drive) {                                   // complete drivetrain test: six body motions
        if (t.state == 2) motorsDrive(c, pgm_read_float(&DT_VEC[t.step][0]), pgm_read_float(&DT_VEC[t.step][1]), pgm_read_float(&DT_VEC[t.step][2]), c.testPwm);
        else motorsDrive(c, 0, 0, 0, c.testPwm);
        if (deadlineReached(now, t.until)) {
          if (t.state == 2) {
            t.state = 3; t.until = now + 700;
            say("TEST", "  %u/6 %S: req vx=%.0f vy=%.0f w=%.0f -> cmd PWM FL %d FR %d BL %d BR %d | %S. COMMAND only: you confirm the wheels really turned that way.",
                (unsigned)t.step + 1, (PGM_P)DT_STEP_NAME[t.step], (double)pgm_read_float(&DT_VEC[t.step][0]), (double)pgm_read_float(&DT_VEC[t.step][1]), (double)pgm_read_float(&DT_VEC[t.step][2]),
                (int)g_wheelOut[W_FL], (int)g_wheelOut[W_FR], (int)g_wheelOut[W_BL], (int)g_wheelOut[W_BR], g_estop ? PSTR("E-STOP LATCHED") : (drivetrainFault() ? PSTR("FAULT") : PSTR("running")));
          }
          else if (++t.step >= 6) {
            say("TEST", "drivetrain test finished. Each motion should have matched its label. If strafe was mirrored use ':roller O'; a wrong single wheel -> ':mrev <wheel>'.");
            t.state = 0; motorsDrive(c, 0, 0, 0, c.testPwm);
            navSetState(NAV_IDLE, PSTR("drivetrain test complete"));
          } else { t.state = 2; t.until = now + 900; say("TEST", "step %u/6: %S", (unsigned)t.step + 1, (PGM_P)DT_STEP_NAME[t.step]); }
        }
        break;
      }
      for (int i = 0; i < WHEEL_COUNT; i++) motorsSetWheelTarget((uint8_t)i, 0);
      if (t.state == 2) motorsSetWheelTarget(t.wheel, (float)c.testPwm);
      if (t.state == 4) motorsSetWheelTarget(t.wheel, -(float)c.testPwm);
      if (deadlineReached(now, t.until)) {
        if (t.state == 2) { t.state = 3; t.until = now + 600; say("TEST", "  %s forward: commanded PWM %d (requested %u) | stop/fault state: %S", WHEEL_PINS[t.wheel].name, (int)g_wheelOut[t.wheel], (unsigned)c.testPwm, g_estop ? PSTR("E-STOP LATCHED") : (drivetrainFault() ? PSTR("FAULT") : PSTR("running"))); }
        else if (t.state == 3) { t.state = 4; t.until = now + 1000; }
        else if (t.state == 4) { t.state = 5; t.until = now + 600; }
        else if (t.state == 5) {
          say("TEST", "%s test finished. Did it turn FORWARD first, then BACKWARD? Wrong: ':mrev %s'; right: ':mok %s'.",
              WHEEL_PINS[t.wheel].name, WHEEL_PINS[t.wheel].name, WHEEL_PINS[t.wheel].name);
          t.state = 0;
          navSetState(NAV_IDLE, PSTR("motor test complete"));
        }
      }
      break;
    }
  }
}

// Manual keys. Manual control always cancels autonomy until 'a' is sent again.
static void navManual(const Config& c, char key, uint32_t now, uint16_t holdMs = 0) {
  if (navEstopRefuses(PSTR("manual motion"))) return;
  if (!startupGate("manual motion")) return;
  if (drivetrainFault()) { say("SAFETY", "manual motion REFUSED: drivetrain fault active (use :recover after fixing)"); return; }
  g_autoEnabled = false;
  g_defendEnabled = false;
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
  if ((vx != 0 || vy != 0) && w == 0 && c.hdgManual && imuHeadingUsable(c, now)) {            // translation: hold the heading it had when the key was first pressed
    bool cont = g_navState == NAV_MANUAL && g_manHold && absf(vx - g_manVx) < 0.01f && absf(vy - g_manVy) < 0.01f && !deadlineReached(now, g_manualUntilMs);
    if (!cont) g_refYaw = g_imu.yaw;
    g_manHold = true; g_manVx = vx; g_manVy = vy;
  } else g_manHold = false;
  motorsDrive(c, vx, vy, w, c.maxManual);
  g_manualUntilMs = now + (holdMs ? holdMs : c.manualHoldMs);
  navSetState(NAV_MANUAL, PSTR("manual command"));
}

static int wheelByName(const char* s) {
  for (int i = 0; i < WHEEL_COUNT; i++) if (strcasecmp(s, WHEEL_PINS[i].name) == 0) return i;
  return -1;
}

static bool navHdgTestArm(const Config& c, uint32_t now) {
  if (navEstopRefuses(PSTR("heading test"))) return false;
  if (!startupGate("heading test")) return false;
  if (!imuHeadingUsable(c, now)) { say("TEST", "heading test REFUSED: no usable, calibrated MPU6050 ('gyro' / ':gcal' with the robot still)"); return false; }
  if (drivetrainFault()) { say("SAFETY", "heading test REFUSED: drivetrain fault active"); return false; }
  g_autoEnabled = false; g_defendEnabled = false; g_mtest.state = 0;
  motorsStopNow();
  g_hdgt.state = 1; g_hdgt.step = 0; g_hdgt.until = now + 15000;
  navSetState(NAV_TEST, PSTR("heading test armed"));
  say("TEST", "HEADING TEST: put the robot ON THE FLOOR with about 1 m free in front, hand on the power switch. ':confirm' within 15 s starts: a clockwise pulse, a counter-clockwise pulse, then a straight drive without and with the heading correction, all at PWM %u (about 7 s). 's' or '!' aborts.", (unsigned)c.testPwm);
  say("TEST", "A robot held off the ground cannot rotate, so a lifted-wheel run only shows 'no yaw response'.");
  return true;
}

static void hdgTestTick(const Config& c, uint32_t now) {
  HdgTest& h = g_hdgt;
  if (h.state == 1) { if (deadlineReached(now, h.until)) { h.state = 0; navSetState(NAV_IDLE, PSTR("heading test confirmation timed out")); } return; }
  if (!imuHeadingUsable(c, now)) {                                       // the IMU went away while a correction was being tested: stop at once
    motorsStopNow(); h.state = 0; navSetState(NAV_IDLE, PSTR("heading test aborted"));
    say("TEST", "heading test ABORTED: the IMU data became stale or invalid. Motors stopped.");
    return;
  }
  static const float VEC[4][3] PROGMEM = {{0, 0, 0.5f}, {0, 0, -0.5f}, {0, 0.5f, 0}, {0, 0.5f, 0}};
  static const uint16_t DUR[4] PROGMEM = {500, 500, 1200, 1200};
  if (h.state == 2) {
    float w = pgm_read_float(&VEC[h.step][2]);
    if (h.step == 3) { float wc = 0; if (navHeading(c, now, &wc)) w = wc; }          // step 4: straight WITH the correction (reference = yaw at the start of the step)
    motorsDrive(c, pgm_read_float(&VEC[h.step][0]), pgm_read_float(&VEC[h.step][1]), w, c.testPwm);
    if (deadlineReached(now, h.until)) { motorsDrive(c, 0, 0, 0, c.testPwm); h.state = 3; h.until = now + 900; }
  } else if (h.state == 3) {
    motorsDrive(c, 0, 0, 0, c.testPwm);
    if (deadlineReached(now, h.until)) {
      h.res[h.step] = -angDiffDeg(h.yaw0, g_imu.yaw);                    // yaw change during the step (+ = clockwise)
      say("TEST", "step %u/4 %S: requested vx=%.1f vy=%.1f w=%.1f at PWM %u -> yaw change %+.1f deg (measured by the gyro)", (unsigned)h.step + 1,
          h.step == 0 ? PSTR("clockwise pulse") : (h.step == 1 ? PSTR("counter-clockwise pulse") : (h.step == 2 ? PSTR("straight, NO correction") : PSTR("straight WITH correction"))),
          (double)pgm_read_float(&VEC[h.step][0]), (double)pgm_read_float(&VEC[h.step][1]), (double)pgm_read_float(&VEC[h.step][2]), (unsigned)c.testPwm, (double)h.res[h.step]);
      if (++h.step >= 4) {
        h.state = 0; navSetState(NAV_IDLE, PSTR("heading test complete"));
        bool resp = absf(h.res[0]) > 2.0f || absf(h.res[1]) > 2.0f;
        say("TEST", "RESULT sign: %S", !resp ? PSTR("NO yaw response (robot lifted, wheels slipping, or the IMU is not rigidly mounted) - nothing can be concluded") :
            ((h.res[0] > 0 && h.res[1] < 0) ? PSTR("OK - a clockwise command gives positive yaw") : PSTR("WRONG - a clockwise command gave negative yaw: check ':roller', ':rev' and ':set yawsign'")));
        say("TEST", "RESULT straight drive: drift %+.1f deg without correction, %+.1f deg with it (limit %.2f of full rotation, deadband %.1f deg). One run is an observation, not proof; repeat it. Gyro heading is RELATIVE and drifts.", (double)h.res[2], (double)h.res[3], (double)c.hdgMaxRot, (double)c.hdgDeadbandDeg);
      } else {
        h.state = 2; h.until = now + pgm_read_word(&DUR[h.step]); h.yaw0 = g_imu.yaw;
        if (h.step == 3) g_refYaw = g_imu.yaw;
      }
    }
  }
}

static void yawCheckService(const Config& c, uint32_t now) {
  YawCheck& y = g_yawck;
  if (!y.state) return;
  float d = -angDiffDeg(y.yaw0, g_imu.yaw);                              // yaw now - yaw at the start
  bool still = absf(g_imu.gz - g_imu.bias) < 3.0f;
  if (still) { if (!y.quietSince) y.quietSince = now ? now : 1u; } else y.quietSince = 0;
  bool done = absf(d) >= 30.0f && y.quietSince && elapsedMs(now, y.quietSince) >= 500;
  if (!imuHeadingUsable(c, now)) { y.state = 0; say("TEST", "yaw check ABORTED: the IMU data became stale or invalid."); return; }
  if (done || deadlineReached(now, y.until)) {
    y.state = 0;
    if (absf(d) < 30.0f) say("TEST", "yaw check: only %+.0f deg measured - turn the robot CLOCKWISE (seen from above) by 60-90 degrees and hold it still, then repeat ':yawcheck'. (Or the IMU does not respond.)", (double)d);
    else if (d > 0) say("TEST", "yaw check: yaw changed %+.0f deg for a clockwise turn: the SIGN IS OK (yawsign %d). If you actually turned it counter-clockwise this proves nothing.", (double)d, (int)c.imuYawSign);
    else say("TEST", "yaw check: yaw changed %+.0f deg: NEGATIVE for what should have been a clockwise turn. If you are sure it was clockwise (seen from above), the sign is WRONG: ':set yawsign %d' and repeat. Otherwise repeat the turn clockwise.", (double)d, (int)(-c.imuYawSign));
  }
}
static bool yawCheckArm(const Config& c, uint32_t now) {
  if (!imuHeadingUsable(c, now)) { say("TEST", "yaw check REFUSED: no usable, calibrated MPU6050"); return false; }
  if (!motorsAllZero()) { say("TEST", "yaw check REFUSED: the motors are not stopped"); return false; }
  g_yawck.state = 1; g_yawck.yaw0 = g_imu.yaw; g_yawck.until = now + 15000; g_yawck.quietSince = 0;
  say("TEST", "YAW CHECK: within 15 s pick the robot up and turn it CLOCKWISE (seen from above) by 60-90 degrees, then hold it still. Nothing will move by itself.");
  return true;
}

static bool navMotorTestArm(const Config& c, int wheel, uint32_t now, bool drive = false) {
  if (wheel >= 0 && navEstopRefuses(PSTR("motor test"))) return false;
  if (wheel < 0) return false;
  if (!startupGate("motor test")) return false;
  if (drivetrainFault()) { say("SAFETY", "motor test REFUSED: drivetrain fault active"); return false; }
  g_autoEnabled = false;
  motorsStopNow();
  g_mtest.state = 1;
  g_mtest.wheel = (uint8_t)wheel;
  g_mtest.drive = drive ? 1 : 0;
  g_mtest.step = 0;
  g_mtest.until = now + 15000;
  navSetState(NAV_TEST, PSTR("motor test armed"));
  if (drive) {
    say("TEST", "DRIVETRAIN TEST: LIFT THE WHEELS and secure the robot. ':confirm' within 15 s runs six 0.9 s motions at PWM %u; 's' aborts.", (unsigned)c.testPwm);
    return true;
  }
  say("TEST", "MOTOR TEST %s: LIFT THE WHEELS, hand on the power switch. ':confirm' within 15 s (PWM %u, 1 s forward, 1 s backward).",
      WHEEL_PINS[wheel].name, (unsigned)c.testPwm);
  return true;
}

static bool navMotorTestConfirm(uint32_t now) {
  if (g_hdgt.state == 1) {                                               // the heading test
    g_hdgt.state = 2; g_hdgt.step = 0; g_hdgt.until = now + 500; g_hdgt.yaw0 = g_imu.yaw;
    say("TEST", "running the heading test... ('s' or '!' aborts)");
    return true;
  }
  if (g_mtest.state != 1) { say("TEST", "nothing to confirm (use :mtest <FL|FR|BL|BR> first)"); return false; }
  g_mtest.state = 2;
  if (g_mtest.drive) { g_mtest.step = 0; g_mtest.until = now + 900; say("TEST", "step 1/6: %S", (PGM_P)DT_STEP_NAME[0]); return true; }
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
    recAdd(REC_SAFE_STOP, 0);
    navSetState(NAV_SAFE_STOP, PSTR("critical fault"));
  } else if (from == COND_SAFE_STOP) {
    say("SAFETY", "SAFE_STOP cleared (%S). Autonomous remains DISABLED: send 'a' after reviewing 'h'.", condName(to));
    if (g_navState == NAV_SAFE_STOP) navSetState(NAV_IDLE, PSTR("fault cleared"));
  } else {
    say("INFO", "operating condition %S -> %S", condName(from), condName(to));
  }
}

#include "defend.h"
#include "diag.h"
#include "startup.h"
