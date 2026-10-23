// diag.h - operating-mode naming and motor-COMMAND diagnostics.
// Everything here is about what the firmware COMMANDED. There are no encoders and no current sensors: a commanded PWM is not proof that a wheel turned.
#pragma once
#include "nav.h"

struct MotionDiag {
  int8_t dir[WHEEL_COUNT];             // commanded direction now: -1 backward, 0 stopped, +1 forward (logical wheel direction, before the reversal setting)
  uint32_t sinceMs[WHEEL_COUNT];       // since when the wheel has had this command
  uint32_t moveSinceMs;                // when the robot started being commanded to move without a break (0 = not moving)
  uint32_t turnSinceMs;                // commanded rotation without translation, continuously
  uint32_t wdTrips, staleEvents;
  uint32_t lastTripMs, lastStaleMs;
  bool wdStopped;                      // the motion watchdog was the last thing that stopped the motors
};
static MotionDiag g_motion;

static bool modeCommandsMotion() { return g_autoEnabled || g_defendEnabled || g_navState == NAV_MANUAL || g_navState == NAV_TEST; }

// The five operating modes the laptop sees (plus FAULT). A latched emergency stop and a SAFE_STOP condition both read FAULT; `safetyText()` says which.
static PGM_P modeName() {
  if (g_estop || g_cond == COND_SAFE_STOP || drivetrainFault()) return PSTR("FAULT");
  if (g_imu.calState == 1 || g_ringCalActive || g_imu.driftState == 1) return PSTR("CALIBRATION");
  if (g_defendEnabled) return g_defState == DEF_FAULT ? PSTR("FAULT") : PSTR("DEFEND");
  if (g_autoEnabled) return PSTR("ATTACK");
  if (g_remoteManual || g_navState == NAV_MANUAL) return PSTR("MANUAL");
  return PSTR("IDLE");
}
static PGM_P safetyText() {
  if (g_estop) return PSTR("ESTOP_LATCHED");
  if (g_cond == COND_SAFE_STOP) return PSTR("SAFE_STOP");
  if (drivetrainFault()) return PSTR("DRIVETRAIN_FAULT");
  if (g_defendEnabled && g_defState == DEF_FAULT) return PSTR("DEFEND_FAULT_STOP");
  if (g_motion.wdStopped) return PSTR("MOTION_WATCHDOG");
  if (g_cond == COND_DEGRADED) return PSTR("DEGRADED");
  return PSTR("OK");
}
// Name of the first active fault (empty when none).
static PGM_P firstFaultName() {
  for (uint8_t i = 0; i < F_COUNT; i++) if (g_fault[i].active) return faultCode(i);
  return PSTR("");
}

// Called every control tick after motorsUpdate().
static void motionService(const Config& c, uint32_t now) {
  bool any = false;
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    int8_t d = g_wheelTgt[i] > 0 ? 1 : (g_wheelTgt[i] < 0 ? -1 : 0);
    if (d != g_motion.dir[i] || g_motion.sinceMs[i] == 0) { g_motion.dir[i] = d; g_motion.sinceMs[i] = now ? now : 1u; }
    if (d != 0) any = true;
  }
  // 1. a motor command left behind by a mode that should not be moving anything: clear it and count it
  if (any && !modeCommandsMotion()) {
    g_motion.staleEvents++; g_motion.lastStaleMs = now;
    motorsStopNow();
    sayRL(3, 2000, "SAFETY", "stale motor command cleared (mode %S commands no motion)", modeName());
    any = false;
  }
  // 2. continuous motion limit
  if (any) { if (!g_motion.moveSinceMs) g_motion.moveSinceMs = now ? now : 1u; } else g_motion.moveSinceMs = 0;
  bool turning = any && absf(g_reqW) > 0.05f && absf(g_reqVx) < 0.05f && absf(g_reqVy) < 0.05f;
  if (turning) { if (!g_motion.turnSinceMs) g_motion.turnSinceMs = now ? now : 1u; } else g_motion.turnSinceMs = 0;
  if (any) g_motion.wdStopped = false;
  if (c.maxMoveSec && g_motion.moveSinceMs && elapsedMs(now, g_motion.moveSinceMs) > (uint32_t)c.maxMoveSec * 1000u) {
    g_motion.wdTrips++; g_motion.lastTripMs = now; g_motion.wdStopped = true;
    say("SAFETY", "motion watchdog: motors were commanded for %u s without a break (maxmovesec). Stopped; autonomy/defend ended.", (unsigned)c.maxMoveSec);
    navStop(PSTR("continuous motion limit"));
    g_motion.moveSinceMs = 0;
  }
}

// One-screen answer to "what are the motors being told to do".
static void printMotion(const Config& c, uint32_t now) {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    say("MOTOR", "%s: command %S, requested %d PWM, ramped output %d PWM, commanded this way for %lu ms%S", WHEEL_PINS[i].name,
        g_motion.dir[i] > 0 ? PSTR("FORWARD") : (g_motion.dir[i] < 0 ? PSTR("BACKWARD") : PSTR("STOPPED")), (int)g_wheelTgt[i], (int)g_wheelOut[i],
        UL(g_motion.sinceMs[i] ? elapsedMs(now, g_motion.sinceMs[i]) : 0), c.wheelScale[i] != 100 ? PSTR(" (trim applied)") : PSTR(""));
  say("MOTOR", "body request vx=%.2f vy=%.2f w=%.2f | moving continuously for %lu ms | rotating without translation for %lu ms | limit %u s",
      (double)g_reqVx, (double)g_reqVy, (double)g_reqW, UL(g_motion.moveSinceMs ? elapsedMs(now, g_motion.moveSinceMs) : 0), UL(g_motion.turnSinceMs ? elapsedMs(now, g_motion.turnSinceMs) : 0), (unsigned)c.maxMoveSec);
  say("MOTOR", "watchdog trips %lu | stale commands cleared %lu | mode %S | safety %S | NOTE: commands only - no encoder or current sensor exists, so actual wheel motion is not measured", UL(g_motion.wdTrips), UL(g_motion.staleEvents), modeName(), safetyText());
}
