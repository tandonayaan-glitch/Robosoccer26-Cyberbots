// ringcli.h - ':ring' (ball ring) and ':mod' (IR obstacle modules) commands.
#pragma once

static bool parseChannel(const char* a, uint8_t* ch) {
  float v;
  if (!parseF(a, &v) || v < 0 || v >= RING_CH_MAX || v != (float)(int)v) return false;
  *ch = (uint8_t)v;
  return true;
}

// Finishes a noise calibration (called every loop with the live config).
static void ringCalPoll(Config& c, uint32_t now) {
  if (!g_ringCalActive || !deadlineReached(now, g_ringCalEndMs)) return;
  g_ringCalActive = false;
  uint8_t n = 0;
  for (uint8_t i = 0; i < RING_CH_MAX; i++) {
    if (!ringEnabled(c, i)) continue;
    c.ringNoise[i] = g_ringCalMax[i]; n++;
    if (g_ringCalMax[i] >= c.ringSeen) say("WARN", "ring cal: A%u swing %u with the ball off reaches the 'seen' level %u: another IR source or a noisy channel", (unsigned)i, (unsigned)g_ringCalMax[i], (unsigned)c.ringSeen);
  }
  c.ringCalDone = 1;
  say("INFO", "ring noise floor stored for %u channels. Threshold = max(ringseen %u, noise x %.1f). ':save' keeps it. Calibration finished (gyro %S): send 'a' to start the autonomous mode.", (unsigned)n, (unsigned)c.ringSeen, (double)c.ringNoiseX10 / 10.0, g_imu.calibrated ? PSTR("done") : PSTR("not done"));
}

// ':ring' | ':ring on|off <ch>' | ':ring bearing <ch> <deg>' | ':ring cal noise' | ':ring cal near' | ':ring reset'
static void cmdRing(Config& c, const char* a1, const char* a2, const char* a3) {
  uint8_t ch;
  float v;
  if (!a1) { reportRequest(REP_RINGONLY); return; }
  if (STREQ(a1, "reset")) { ringResetStats(); say("INFO", "ring statistics cleared"); return; }
  if ((STREQ(a1, "on") || STREQ(a1, "off")) && parseChannel(a2, &ch)) {
    if (STREQ(a1, "on")) c.ringEnable |= (uint16_t)(1u << ch); else c.ringEnable &= (uint16_t)~(1u << ch);
    g_ring[ch].init = false; g_ring[ch].seen = false; g_ring[ch].railFault = false; g_ring[ch].railSinceMs = 0;
    say("INFO", "receiver A%u %S (bearing %d deg)", (unsigned)ch, ringEnabled(c, ch) ? PSTR("ON") : PSTR("OFF"), (int)c.ringBearing[ch]);
    if (c.ringCalDone) say("WARN", "the noise floor does not cover a newly switched-on receiver: repeat ':ring cal noise'");
    return;
  }
  if (STREQ(a1, "bearing") && parseChannel(a2, &ch) && parseF(a3, &v) && v >= -180 && v <= 180) {
    c.ringBearing[ch] = (int16_t)(v < 0 ? v - 0.5f : v + 0.5f);
    c.bearingsSet = 1;
    say("INFO", "receiver A%u looks %d deg (0 = ahead, + = right, +-180 = behind). ':save' keeps it.", (unsigned)ch, (int)c.ringBearing[ch]);
    return;
  }
  if (STREQ(a1, "cal") && a2 && STREQ(a2, "noise")) {
    if (ringCountEnabled(c) == 0) { say("WARN", "ring cal noise: no channel is enabled"); return; }
    memset(g_ringCalMax, 0, sizeof(g_ringCalMax));
    g_ringCalActive = true; g_ringCalEndMs = millis() + 2000u;
    say("INFO", "ring cal noise: measuring for 2 s. The BALL MUST BE OFF (or far away / asleep) and no other infrared source may be near. Keep the robot still.");
    return;
  }
  if (STREQ(a1, "cal") && a2 && STREQ(a2, "near")) {
    if (!g_ball.bearingKnown) { say("WARN", "ring cal near: no channel sees the ball. Put the ball in the shooting position, in front of the robot, emitting."); return; }
    uint16_t v2 = (uint16_t)(g_ring[g_ball.bestCh].avg + 0.5f);
    if (v2 < 2u * c.ringSeen) { say("WARN", "ring cal near: the strongest receiver (A%u) swing is only %u, less than twice the 'seen' threshold %u: no usable range yet", (unsigned)g_ball.bestCh, (unsigned)v2, (unsigned)c.ringSeen); return; }
    c.ringNear = v2;
    say("INFO", "ringnear = %u (swing of A%u with the ball in the shooting position). ':save' keeps it.", (unsigned)v2, (unsigned)g_ball.bestCh);
    return;
  }
  say("WARN", "usage: :ring | :ring on|off <0-7> | :ring bearing <0-7> <deg> | :ring cal noise | :ring cal near | :ring reset");
}

// ':mod' | ':mod on|off <0|1>' | ':mod level <0|1> low|high'
static void cmdMod(Config& c, const char* a1, const char* a2, const char* a3) {
  if (!a1) { reportRequest(REP_MOD); return; }
  if ((STREQ(a1, "on") || STREQ(a1, "off")) && a2 && (a2[0] == '0' || a2[0] == '1') && !a2[1]) {
    uint8_t i = (uint8_t)(a2[0] - '0');
    if (STREQ(a1, "on")) c.modEnable |= (uint8_t)(1u << i); else c.modEnable &= (uint8_t)~(1u << i);
    say("INFO", "module %u (D%u) %S", (unsigned)i, (unsigned)PIN_MOD[i], modEnabled(c, i) ? PSTR("ON") : PSTR("OFF"));
    return;
  }
  if (STREQ(a1, "level") && a2 && (a2[0] == '0' || a2[0] == '1') && !a2[1] && a3 && (STREQ(a3, "low") || STREQ(a3, "high"))) {
    uint8_t i = (uint8_t)(a2[0] - '0');
    if (STREQ(a3, "low")) c.modActiveLow |= (uint8_t)(1u << i); else c.modActiveLow &= (uint8_t)~(1u << i);
    say("INFO", "module %u output is %S when something is detected. ':save' keeps it.", (unsigned)i, ((c.modActiveLow >> i) & 1u) ? PSTR("LOW") : PSTR("HIGH"));
    return;
  }
  say("WARN", "usage: :mod | :mod on|off <0|1> | :mod level <0|1> low|high");
}
