// defend.h - DEFENDING MODE ('defend'): the robot holds a defensive heading, watches the ball and moves SIDEWAYS to get in front of an approaching ball.
//
// What the hardware can and cannot do (nothing here pretends otherwise):
//  * There is NO position sensor (no encoders, no distance sensor, no field marker). "Position" therefore means the registered HEADING (gyro, relative) plus an
//    open-loop ESTIMATE of the sideways offset from where defend mode started, integrated from the commanded strafe time (g_defLat). That estimate only limits how far the
//    robot may wander; it is never a measurement.
//  * The receivers give a bearing and a signal STRENGTH, no distance. "Approaching" means the strength has been rising over time-stamped observations; it is not a speed
//    or a distance, and it does not prove the ball is heading for the goal.
//  * No goalkeeper mechanism and no kicker exist: blocking means standing in front of the ball with the wheels. Nothing guarantees the ball stops.
//
// States (every transition is logged with its reason; minimum dwell + confidence hysteresis stop rapid flipping on noisy readings):
//   POSITION  restore the defensive heading, drift back towards the start offset
//   SEARCH    stand still and watch (the ring covers the circle; no spinning)
//   TRACK     ball confirmed: hold, follow it with the readings
//   PREDICT   the time-stamped observations support a trend (approaching and/or sideways drift): commit a direction
//   INTERCEPT strafe towards where the ball is (plus the measured lead), bounded in time and in total offset
//   BLOCK     in front of the ball: stop, trim the alignment slowly, check the ball is still there
//   LOST      the ball disappeared: stand still for deflost
//   RECOVER   bounded turn towards the last bearing (needs the gyro to come back), then POSITION
//   FAULT     a sensing or drivetrain fault: motors stopped until the operator acts
#pragma once
#include "nav.h"

enum DefState : uint8_t { DEF_POSITION, DEF_SEARCH, DEF_TRACK, DEF_PREDICT, DEF_INTERCEPT, DEF_BLOCK, DEF_LOST, DEF_RECOVER, DEF_FAULT, DEF_COUNT };
static const char DEF_NAME[DEF_COUNT][11] PROGMEM = {"POSITION", "SEARCH", "TRACK", "PREDICT", "INTERCEPT", "BLOCK", "LOST", "RECOVER", "FAULT_STOP"};

struct DefObs { uint32_t ms; float bearing, strength; };
static const uint8_t DEF_OBS_N = 8;
static DefObs g_defObs[DEF_OBS_N];
static uint8_t g_defObsCount = 0, g_defObsHead = 0;
static uint32_t g_defObsLastMs = 0;

static DefState g_defState = DEF_POSITION;
static uint32_t g_defSince = 0, g_defHaveSince = 0, g_defStrafeSince = 0, g_defLastTickMs = 0, g_defStrafeRestMs = 0, g_defObstacleHoldUntil = 0;
static float g_defHeading = 0.0f;       // gyro yaw to hold (relative)
static float g_defLat = 0.0f;           // ESTIMATED sideways offset from the start, in "ms at defspeed" (+ = right); integrated from the commanded strafe
static int8_t g_defDir = 0;             // committed strafe direction (+1 right, -1 left)
static bool g_defPredOk = false, g_defApproaching = false, g_defReceding = false;
static float g_defStrSlope = 0.0f, g_defBrgRate = 0.0f, g_defLeadDeg = 0.0f;
static uint8_t g_defRecoverTries = 0;
static uint32_t g_defTransitions = 0;
static float g_defTarget = 0.0f;
static bool g_defRecoverTurned = false;

static void defSet(DefState s, PGM_P why) {
  if (g_defState == s) return;
  say("DEFEND", "%S -> %S (%S)", (PGM_P)DEF_NAME[g_defState], (PGM_P)DEF_NAME[s], why);
  g_defState = s; g_defSince = millis(); g_defTransitions++;
  recAdd(REC_MODE, (uint16_t)(10 + s));
}

static bool defendEnable(const Config& c, uint32_t now) {
  if (navEstopRefuses(PSTR("defend"))) return false;
  if (!startupGate("defend")) return false;
  if (g_cond == COND_SAFE_STOP || sensingFault() || drivetrainFault()) { say("SAFETY", "defend REFUSED: robot is in SAFE_STOP or has a sensing/drivetrain fault (see 'status')"); return false; }
  if (g_mtest.state) { say("SAFETY", "defend REFUSED: motor test in progress"); return false; }
  if (ringCountUsable(c) == 0) { say("SAFETY", "defend REFUSED: no usable ball receiver"); return false; }
  if (!c.ringCalDone) { say("SAFETY", "defend REFUSED: receiver noise floor not calibrated (remove the ball, 'calibrate', then ':save')"); return false; }
  if (!c.bearingsSet) { say("SAFETY", "defend REFUSED: the receiver mounting angles are still the default guess. Enter them with ':ring bearing <0-5> <deg>' (docs/BALL_SENSING.md)"); return false; }
  g_autoEnabled = false; g_remoteManual = false;
  motorsStopNow();
  g_defendEnabled = true;
  g_defObsCount = g_defObsHead = 0; g_defLat = 0; g_defDir = 0; g_defHaveSince = 0; g_defRecoverTries = 0; g_defPredOk = g_defApproaching = g_defReceding = false;
  g_defHeading = g_imu.yaw; g_refYaw = g_imu.yaw; g_defLastTickMs = now;
  g_defState = DEF_POSITION; g_defSince = now; g_defTransitions = 0;
  navSetState(NAV_DEFEND, PSTR("defend mode requested"));
  recAdd(REC_MODE, 3);
  say("DEFEND", "defend mode ON. Heading hold %S. Sideways offset limit +-%u ms-at-speed (estimate). Stop with 'stop'; emergency stop X / '!'.",
      imuHeadingUsable(c, now) ? PSTR("active") : PSTR("UNAVAILABLE (no usable gyro: the robot cannot restore its heading)"), (unsigned)c.defLatMs);
  return true;
}

// Least-squares slopes over the observations of the last 800 ms. Needs >= 4 samples spanning >= 300 ms; otherwise no trend is reported at all.
static void defAnalyse(const Config& c, uint32_t now) {
  g_defPredOk = false; g_defApproaching = g_defReceding = false; g_defStrSlope = g_defBrgRate = g_defLeadDeg = 0.0f;
  float t[DEF_OBS_N], br[DEF_OBS_N], st[DEF_OBS_N];
  uint8_t n = 0;
  for (uint8_t k = 0; k < g_defObsCount; k++) {
    const DefObs& o = g_defObs[(uint8_t)((g_defObsHead + DEF_OBS_N - 1 - k) % DEF_OBS_N)];     // newest first
    if (elapsedMs(now, o.ms) > 800) break;
    if (g_ball.seenSinceMs && (int32_t)(o.ms - g_ball.seenSinceMs) < 250) break;                 // the first 250 ms after the ball appears are the smoothing filter settling, not an approach
    t[n] = -(float)elapsedMs(now, o.ms) / 1000.0f; br[n] = o.bearing; st[n] = o.strength; n++;
  }
  if (n < 4 || t[0] - t[n - 1] < 0.3f) return;
  for (uint8_t i = 1; i < n; i++) br[i] = br[0] + wrapDeg180(br[i] - br[0]);                     // unwrap relative to the newest
  float mt = 0, mb = 0, ms = 0;
  for (uint8_t i = 0; i < n; i++) { mt += t[i]; mb += br[i]; ms += st[i]; }
  mt /= n; mb /= n; ms /= n;
  float stt = 0, stb = 0, sts = 0;
  for (uint8_t i = 0; i < n; i++) { stt += (t[i] - mt) * (t[i] - mt); stb += (t[i] - mt) * (br[i] - mb); sts += (t[i] - mt) * (st[i] - ms); }
  if (stt < 1e-6f) return;
  g_defBrgRate = stb / stt; g_defStrSlope = sts / stt;
  g_defPredOk = true;
  g_defApproaching = g_defStrSlope >= c.defSlope;
  g_defReceding = g_defStrSlope <= -c.defSlope;
  g_defLeadDeg = clampf(g_defBrgRate * (float)c.defLeadMs / 1000.0f, -25.0f, 25.0f);
}

static void defendTick(const Config& c, uint32_t now) {
  uint32_t dtMs = elapsedMs(now, g_defLastTickMs); g_defLastTickMs = now; if (dtMs > 100) dtMs = 100;
  if (!g_defendEnabled) return;
  float vx = 0, w = 0;
  if (g_defState != DEF_FAULT && (g_cond == COND_SAFE_STOP || drivetrainFault() || sensingFault() || ringCountUsable(c) == 0)) {
    motorsDrive(c, 0, 0, 0, c.maxAuto);
    defSet(DEF_FAULT, PSTR("sensing or drivetrain fault: motors stopped"));
    return;
  }
  if (g_defState == DEF_FAULT) { motorsDrive(c, 0, 0, 0, c.maxAuto); return; }

  // time-stamped observations (every >= 100 ms while a bearing is known)
  if (g_ball.bearingKnown && elapsedMs(now, g_defObsLastMs) >= 100) {
    g_defObsLastMs = now;
    g_defObs[g_defObsHead] = {now, g_ball.bearing, g_ball.strength};
    g_defObsHead = (uint8_t)((g_defObsHead + 1) % DEF_OBS_N); if (g_defObsCount < DEF_OBS_N) g_defObsCount++;
  }
  if (!g_ball.bearingKnown && elapsedMs(now, g_defObsLastMs) > 800) g_defObsCount = 0;      // stale history is never used
  defAnalyse(c, now);

  // confidence with hysteresis: on at defconfon, off below defconfoff
  bool tracking = g_defState == DEF_TRACK || g_defState == DEF_PREDICT || g_defState == DEF_INTERCEPT || g_defState == DEF_BLOCK;
  bool have = g_ball.valid && g_ball.bearingKnown && !g_ball.ambiguous && g_ball.confidence >= (tracking ? c.defConfOff : c.defConfOn);
  if (have) { if (!g_defHaveSince) g_defHaveSince = now ? now : 1u; } else g_defHaveSince = 0;
  bool confirmed = have && elapsedMs(now, g_defHaveSince) >= c.defConfirmMs;
  uint32_t inState = elapsedMs(now, g_defSince);
  bool dwell = inState >= c.defDwellMs;
  float scale = navSensorScale(c);
  float brg = g_ball.bearingKnown ? g_ball.bearing : g_lastBearing;
  bool obstacle = modEnabled(c, 1) && g_mod[1].active;
  bool headingOk = imuHeadingUsable(c, now);
  g_refYaw = g_defHeading;

  auto strafeAllowed = [&](int dir) {
    if (obstacle) return false;
    if (!deadlineReached(now, g_defObstacleHoldUntil)) return false;
    if (dir > 0 && g_defLat >= (float)c.defLatMs) return false;
    if (dir < 0 && g_defLat <= -(float)c.defLatMs) return false;
    return true;
  };

  switch (g_defState) {
    case DEF_POSITION: {
      bool headDone = true, latDone = true;
      if (headingOk) {
        float err = angDiffDeg(g_defHeading, g_imu.yaw);
        if (absf(err) > c.hdgDeadbandDeg * 2.0f) { w = clampf(c.hdgKp * err, -c.defTurnSpeed, c.defTurnSpeed); if (absf(w) < 0.12f) w = err > 0 ? 0.12f : -0.12f; headDone = false; }
      }
      if (headDone && absf(g_defLat) > 100.0f && !confirmed) { vx = (g_defLat > 0 ? -1.0f : 1.0f) * c.defAlignSpeed; latDone = false; }   // drift back towards the start offset
      if (confirmed && absf(brg) <= c.defFrontArcDeg) { w = vx = 0; defSet(DEF_TRACK, PSTR("ball confirmed while positioning")); }
      else if (headDone && latDone) defSet(DEF_SEARCH, PSTR("defensive heading restored"));
      else if (inState > c.defPosMs) { w = vx = 0; defSet(DEF_SEARCH, PSTR("position timeout: holding where it is")); }
      break;
    }
    case DEF_SEARCH:
      if (confirmed) {
        if (absf(brg) <= c.defFrontArcDeg) defSet(DEF_TRACK, PSTR("ball confirmed"));
        else if (dwell) { g_defRecoverTries = 0; defSet(DEF_RECOVER, PSTR("ball confirmed behind the robot")); }
      }
      break;
    case DEF_TRACK:
      if (!have && dwell) { defSet(DEF_LOST, PSTR("ball signal lost or confidence below defconfoff")); break; }
      if (!dwell) break;
      if (absf(brg) > c.defFrontArcDeg) { g_defRecoverTries = 0; defSet(DEF_RECOVER, PSTR("ball is behind the robot: it cannot be blocked from here")); break; }
      if (g_ball.held || (g_ball.strength >= c.defBlockStrength && absf(brg) <= c.defAlignDeg)) { defSet(DEF_BLOCK, PSTR("ball in front at the block position")); break; }
      if (g_defPredOk && g_defApproaching) { defSet(DEF_PREDICT, PSTR("strength rising over the observations: ball approaching")); break; }
      if (absf(brg) > c.defAlignDeg && g_ball.strength >= 0.4f) { g_defDir = brg > 0 ? 1 : -1; defSet(DEF_INTERCEPT, PSTR("trend unknown: conservative shadowing of a close ball")); }
      break;
    case DEF_PREDICT: {
      g_defTarget = brg + (g_defPredOk ? g_defLeadDeg : 0.0f);
      g_defDir = g_defTarget > 0 ? 1 : -1;
      if (!have) { defSet(DEF_LOST, PSTR("ball lost while predicting")); break; }
      if (g_defPredOk && g_defReceding) { defSet(DEF_TRACK, PSTR("strength falling: ball no longer approaching")); break; }
      if (inState >= c.defConfirmMs) {
        if (absf(g_defTarget) > c.defAlignDeg) defSet(DEF_INTERCEPT, PSTR("intercept direction committed"));
        else defSet(DEF_BLOCK, PSTR("ball already in line: hold and block"));
      }
      break;
    }
    case DEF_INTERCEPT: {
      g_defTarget = brg + (g_defPredOk ? g_defLeadDeg : 0.0f);
      if (!have) { vx = 0; defSet(DEF_LOST, PSTR("ball lost while intercepting")); break; }
      if (absf(brg) > c.defFrontArcDeg) { g_defRecoverTries = 0; defSet(DEF_RECOVER, PSTR("ball went behind the robot")); break; }
      if (g_ball.held) { defSet(DEF_BLOCK, PSTR("ball at the front sensor")); break; }
      if (absf(g_defTarget) <= c.defAlignDeg * 0.75f) { defSet(DEF_BLOCK, PSTR("aligned with the ball")); break; }          // exit threshold < entry threshold: hysteresis
      int dir = g_defTarget > 0 ? 1 : -1;
      if (dir != g_defDir && absf(g_defTarget) < 1.5f * c.defAlignDeg) dir = g_defDir;                                     // do not flip direction on a small overshoot
      g_defDir = (int8_t)dir;
      if (obstacle) { g_defObstacleHoldUntil = now + 500u; defSet(DEF_TRACK, PSTR("obstacle sensor 1 active: not driving into it")); break; }
      if (!strafeAllowed(dir)) { defSet(DEF_TRACK, PSTR("sideways limit reached (deflatms): holding")); break; }
      if (!g_defStrafeSince) g_defStrafeSince = now ? now : 1u;
      if (elapsedMs(now, g_defStrafeSince) > c.defStrafeMs) { g_defStrafeSince = 0; g_defStrafeRestMs = now; defSet(DEF_TRACK, PSTR("continuous strafe limit (defstrafems): pausing")); break; }
      float v = c.defAlignSpeed + (c.defSpeed - c.defAlignSpeed) * clampf(absf(g_defTarget) / 45.0f, 0.0f, 1.0f);
      v *= 0.6f + 0.4f * (float)g_ball.confidence / 100.0f;
      vx = (float)dir * v;
      break;
    }
    case DEF_BLOCK:
      if (!have && inState > c.defDwellMs) { defSet(DEF_LOST, PSTR("ball no longer detected after the block attempt")); break; }
      if (absf(brg) > 1.5f * c.defAlignDeg && !g_ball.held && dwell) { defSet(DEF_TRACK, PSTR("ball moved away from the block position")); break; }
      if (g_defPredOk && g_defReceding && g_ball.strength < 0.5f * c.defBlockStrength && dwell) { defSet(DEF_TRACK, PSTR("ball moving away")); break; }
      if (!g_ball.held && absf(brg) > 0.5f * c.defAlignDeg) {                                // slow final alignment
        int dir = brg > 0 ? 1 : -1;
        if (strafeAllowed(dir)) vx = (float)dir * 0.6f * c.defAlignSpeed;
      }
      break;
    case DEF_LOST:
      if (confirmed) { defSet(DEF_TRACK, PSTR("ball reacquired")); break; }
      if (inState >= c.defLostMs) {
        if (g_defRecoverTries < 2 && headingOk && absf(g_lastBearing) > c.defAlignDeg * 2.0f) { g_defRecoverTries++; defSet(DEF_RECOVER, PSTR("ball not found: turning towards where it was last seen")); }
        else defSet(DEF_POSITION, PSTR("ball not found: back to the defensive position"));
      }
      break;
    case DEF_RECOVER: {
      if (confirmed && absf(brg) <= c.defFrontArcDeg) { defSet(DEF_TRACK, PSTR("ball reacquired")); break; }
      if (inState < c.defRecoverMs && headingOk && g_defRecoverTries < 3 && absf(g_lastBearing) > c.defAlignDeg * 2.0f) {
        w = (g_lastBearing > 0 ? 1.0f : -1.0f) * c.defTurnSpeed; g_defRecoverTurned = true;
      } else { g_defRecoverTurned = false; defSet(DEF_POSITION, PSTR("recover finished: restoring the defensive heading")); }
      break;
    }
    default: break;
  }

  // hold the heading while the robot is not rotating on purpose
  bool turningOnPurpose = (g_defState == DEF_RECOVER && w != 0.0f) || (g_defState == DEF_POSITION && w != 0.0f);
  if (!turningOnPurpose && headingOk && !(g_defState == DEF_FAULT)) navHeading(c, now, &w);
  if (g_defState == DEF_INTERCEPT && vx == 0.0f) g_defStrafeSince = 0;
  if (g_defState != DEF_INTERCEPT && vx == 0.0f && elapsedMs(now, g_defStrafeRestMs) > 300) g_defStrafeSince = 0;
  g_defLat += (vx / (c.defSpeed > 0.01f ? c.defSpeed : 1.0f)) * (float)dtMs;
  motorsDrive(c, vx * scale, 0, w * scale, c.maxAuto);
}
