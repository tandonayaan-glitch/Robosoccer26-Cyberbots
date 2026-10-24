// report.h - human-readable diagnostics: health report, ring / module snapshot and stream, nav and IMU status.
#pragma once
#include "nav.h"
#include "bt.h"

#if defined(__AVR__)
extern int __heap_start, *__brkval;
static int freeRam() { int v; return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval); }
#else
static int freeRam() { return 4096; }
#endif

static PGM_P ringChWord(const Config& c, uint8_t i) {
  if (!ringEnabled(c, i)) return PSTR("off");
  if (g_ring[i].railFault) return PSTR("SATURATED (excluded)");
  return g_ring[i].seen ? PSTR("SEES ball") : PSTR("-");
}

static void printRingSnapshot(const Config& c) {
  uint8_t nU;
  float gap = ringCoverageGap(c, &nU);
  say("INFO", "ball receivers: %u of %u analog inputs on, %u usable | window %u us | seen at swing >= max(%u, noise x %.1f), very close at %u (10-bit counts)",
      (unsigned)ringCountEnabled(c), (unsigned)RING_CH_MAX, (unsigned)ringCountUsable(c), (unsigned)c.ringWinUs, (unsigned)c.ringSeen, (double)c.ringNoiseX10 / 10.0, (unsigned)c.ringNear);
  say("INFO", "noise floor %S", c.ringCalDone ? PSTR("CALIBRATED") : PSTR("NOT CALIBRATED (':ring cal noise' with the ball off)"));
  for (uint8_t i = 0; i < RING_CH_MAX; i++) {
    if (!ringEnabled(c, i) && i >= RING_CH_FIXED) continue;      // A2..A7 stay hidden until switched on
    const RingCh& r = g_ring[i];
    say("INFO", " A%u brg %4d | swing %3u (avg %5.1f, max %3u) lo %4u hi %4u | noise %3u thr %4.0f | str %.2f | %S", (unsigned)i, (int)c.ringBearing[i],
        (unsigned)r.amp, (double)r.avg, (unsigned)r.ampMax, (unsigned)r.lo, (unsigned)r.hi, (unsigned)c.ringNoise[i], (double)ringThreshold(c, i), (double)r.s, ringChWord(c, i));
  }
  say("INFO", "coverage: largest arc no usable receiver covers ~%.0f deg (configured bearings, ASSUMED +-%u deg view each; an estimate)", (double)gap, (unsigned)c.ringFovHalf);
  if (g_ball.bearingKnown) say("INFO", "ball: bearing %.0f deg (0 = ahead, + = right), strongest A%u, strength %.2f, %u receiver(s)", (double)g_ball.bearing, (unsigned)g_ball.bestCh, (double)g_ball.strength, (unsigned)g_ball.nSeen);
  else if (g_ball.valid) say("INFO", "ball: not seen now, last bearing %.0f deg (held %u ms)", (double)g_lastBearing, (unsigned)c.lostHoldMs);
  else say("INFO", "ball: not seen (a navigation condition, not a sensor fault)");
  say("INFO", "scan: last channel %lu us, longest %lu us, full passes %lu. Swing = signal STRENGTH; the bearing is only as good as the mounting angles (':ring bearing')", UL(g_ringScanLastUs), UL(g_ringScanMaxUs), UL(g_ringCycles));
}

static void printModSnapshot(const Config& c) {
  for (uint8_t i = 0; i < MOD_COUNT; i++) {
    const Module& m = g_mod[i];
    say("INFO", "IR module %u D%u (%S): %S | pin %S | active when %S | %S", (unsigned)i, (unsigned)PIN_MOD[i], i == 0 ? PSTR("front: shooting position") : PSTR("second module: logged only"),
        modEnabled(c, i) ? PSTR("on") : PSTR("off"), digitalRead(PIN_MOD[i]) ? PSTR("HIGH") : PSTR("LOW"), ((c.modActiveLow >> i) & 1u) ? PSTR("LOW") : PSTR("HIGH"),
        !modEnabled(c, i) ? PSTR("-") : (m.active ? PSTR("DETECTING") : PSTR("clear")));
  }
  say("INFO", "shooting position = module 0 detecting%S", c.modNeedRing ? PSTR(" AND the ring sees the ball straight ahead") : PSTR(" (ring not required)"));
  say("INFO", "a module reacts to ANY reflective object in range (wall, robot); its own LED can disturb nearby receivers (docs/RING_AND_MODULES.md)");
}

static void printRingStream(const Config& c) {
  char buf[64];
  size_t n = 0;
  buf[0] = 0;
  for (uint8_t i = 0; i < RING_CH_MAX && n + 8 < sizeof(buf); i++) {
    if (!ringEnabled(c, i)) continue;
    n += (size_t)snprintf(buf + n, sizeof(buf) - n, "%s%u ", g_ring[i].seen ? "*" : "", (unsigned)g_ring[i].amp);
  }
  say("INFO", "RING %s| ball=%c brg=%S held=%d ready=%d mod0=%d mod1=%d", buf, g_ball.valid ? 'Y' : 'n', g_ball.bearingKnown ? PSTR("known") : PSTR("?"), g_ball.held, g_ball.ready, g_mod[0].active, g_mod[1].active);
  if (g_ball.bearingKnown) say("INFO", "   bearing %.0f deg strongest A%u strength %.2f", (double)g_ball.bearing, (unsigned)g_ball.bestCh, (double)g_ball.strength);
}

static void printNavStatus(const Config& c, uint32_t now) {
  say("NAV", "state=%S cond=%S autonomous=%S", (PGM_P)NAV_NAME[g_navState], condName(g_cond), g_autoEnabled ? PSTR("ENABLED") : PSTR("disabled"));
  say("NAV", "ball: valid=%d seen=0x%X (%u ch) held=%d ready=%d bearing=%S%.0f deg strength=%.2f age=%lu ms", g_ball.valid, (unsigned)g_ball.seenMask, (unsigned)g_ball.nSeen, g_ball.held, g_ball.ready,
      g_ball.bearingKnown ? PSTR("") : PSTR("(last) "), (double)(g_ball.bearingKnown ? g_ball.bearing : g_lastBearing), (double)g_ball.strength, UL(g_ball.lastSeenMs ? elapsedMs(now, g_ball.lastSeenMs) : 0));
  if (isnan(g_hdgErr)) say("NAV", "heading: hold %S (imu %S)", c.hdgEnable ? PSTR("inactive") : PSTR("disabled"), imuHeadingUsable(c, now) ? PSTR("usable") : PSTR("unavailable"));
  else say("NAV", "heading: ref %.1f yaw %.1f error %.1f deg", (double)g_refYaw, (double)g_imu.yaw, (double)g_hdgErr);
  say("NAV", "request vx=%.2f vy=%.2f w=%.2f | commanded PWM FL=%d FR=%d BL=%d BR=%d (commanded, NOT measured)",
      (double)g_reqVx, (double)g_reqVy, (double)g_reqW, (int)g_wheelOut[0], (int)g_wheelOut[1], (int)g_wheelOut[2], (int)g_wheelOut[3]);
}

static void printImu(const Config& c, uint32_t now) {
  if (!g_imu.present) { say("INFO", "MPU6050: NOT FOUND (no answer at 0x68/0x69). Heading hold off, fallback in use."); return; }
  say("INFO", "MPU6050 addr=0x%02X whoami=0x%02X ok=%d fresh=%d calibrated=%d bias=%.3f dps", (unsigned)g_imu.addr, (unsigned)g_imu.whoami, g_imu.ok, imuFresh(c, now), g_imu.calibrated, (double)g_imu.bias);
  say("INFO", " accel g: %.3f %.3f %.3f | gyro dps: %.2f %.2f %.2f", (double)g_imu.ax, (double)g_imu.ay, (double)g_imu.az, (double)g_imu.gx, (double)g_imu.gy, (double)g_imu.gz);
  say("INFO", " relative heading (cw+): %.1f deg %S | samples=%lu errors=%lu timeouts=%lu invalid=%lu recoveries=%lu", (double)g_imu.yaw, g_imu.yawValid ? PSTR("valid") : PSTR("INVALID"),
      UL(g_imu.samples), UL(g_imu.errCount), UL(g_imu.timeoutCount), UL(g_imu.invalidCount), UL(g_imu.recoveries));
}

// Full health report. PASS only where a check really passed; NOT TESTED / NOT MEASURABLE are said plainly. `tag` is a flash string.
static void printHealth(const Config& c, uint32_t now, PGM_P tag) {
  sayT(tag, "================ HARDWARE HEALTH REPORT ================");
  sayT(tag, "firmware %s | Arduino Mega 2560 (ATmega2560, 16 MHz, 8 KB RAM, 256 KB flash) | free RAM %d B | uptime %lu ms", FW_VERSION, freeRam(), UL(now));
  sayT(tag, "operating condition: %S | autonomous: %S | nav: %S", condName(g_cond), g_autoEnabled ? PSTR("ENABLED") : PSTR("disabled"), (PGM_P)NAV_NAME[g_navState]);

  bool pwmOk = true;
  for (int i = 0; i < WHEEL_COUNT; i++) pwmOk = pwmOk && g_pwmAttached[i];
  sayT(tag, "[Drivetrain] motor enable pins are hardware PWM pins: %S (software check only; says nothing about the wiring)", pwmOk ? PSTR("PASS") : PSTR("FAIL"));
  for (int i = 0; i < WHEEL_COUNT; i++)
    sayT(tag, "[Drivetrain] wheel %s IN1=D%u IN2=D%u EN=D%u reversal=%d: direction %S", WHEEL_PINS[i].name, (unsigned)WHEEL_PINS[i].in1, (unsigned)WHEEL_PINS[i].in2,
         (unsigned)WHEEL_PINS[i].en, c.reverse[i], (c.motorVerified & (1u << i)) ? PSTR("operator-confirmed by :mok") : PSTR("NOT TESTED"));
  sayT(tag, "[Drivetrain] motor/driver electrical health, actual rotation, stall: NOT MEASURABLE (no encoders, no current sensing)");

  {
    uint8_t nU;
    float gap = ringCoverageGap(c, &nU);
    PGM_P w = ringCountEnabled(c) == 0 ? PSTR("OFF") : ringCountUsable(c) == 0 ? PSTR("FAIL (no usable receiver)") : !c.ringCalDone ? PSTR("NOT TESTED (noise floor not calibrated)") : PSTR("calibrated (software: says nothing about the ball response)");
    sayT(tag, "[Ring] receivers on A0..A7, each its own analog input: %u enabled, %u usable, %S", (unsigned)ringCountEnabled(c), (unsigned)ringCountUsable(c), w);
    for (uint8_t i = 0; i < RING_CH_MAX; i++)
      if (ringEnabled(c, i) && g_ring[i].railFault) sayT(tag, "[Ring] A%u FAIL (saturated: swing cannot be measured; short to the 5 V rail or steady light)", (unsigned)i);
    sayT(tag, "[Ring] coverage gap ~%.0f deg (estimate; bearings are PLACEHOLDERS until set with ':ring bearing')", (double)gap);
    for (uint8_t i = 0; i < MOD_COUNT; i++)
      sayT(tag, "[Module %u] D%u (%S): %S, %S", (unsigned)i, (unsigned)PIN_MOD[i], i == 0 ? PSTR("front, shooting position") : PSTR("second module"), modEnabled(c, i) ? PSTR("on") : PSTR("off"),
           !modEnabled(c, i) ? PSTR("-") : (g_mod[i].active ? PSTR("detecting now") : PSTR("clear now")));
    sayT(tag, "[Ring] physical response to the real ball (range, field of view, lighting, other IR sources): NOT TESTED (docs/RING_AND_MODULES.md)");
  }

  if (!g_imu.present) sayT(tag, "[IMU] MPU6050: %S (no device answered at 0x68/0x69)", g_imu.hadFault ? PSTR("FAIL") : PSTR("WARNING - not found"));
  else sayT(tag, "[IMU] MPU6050 @0x%02X who=0x%02X: %S; gyro bias %S; heading hold %S", (unsigned)g_imu.addr, (unsigned)g_imu.whoami,
            g_imu.ok ? (g_imu.whoami == 0x68 ? PSTR("PASS") : PSTR("WARNING (non-0x68 ID, clone?)")) : PSTR("FAIL/RECOVERING"),
            g_imu.calibrated ? PSTR("calibrated") : PSTR("NOT calibrated"), imuHeadingUsable(c, now) ? PSTR("available") : PSTR("UNAVAILABLE"));

  sayT(tag, "[Storage] configuration in EEPROM (CRC protected). No flash logging on the Mega: CSV rows stream over Serial with ':diag on'");
  sayT(tag, "[Power] battery voltage: NOT MEASURABLE (no voltage monitor installed); motor current: NOT MEASURABLE");

  int active = 0;
  for (int i = 0; i < F_COUNT; i++) {
    if (!g_fault[i].active) continue;
    active++;
    sayT(tag, "[FAULT] %S (%S, %S) since %lu ms, seen %lu x: %s", faultCode((uint8_t)i), (PGM_P)FMETA[i].comp, sevName(faultSev((uint8_t)i)), UL(g_fault[i].firstMs), UL(g_fault[i].count), g_fault[i].evidence);
    sayT(tag, "        -> %S", (PGM_P)FMETA[i].action);
  }
  if (!active) sayT(tag, "[FAULT] none active");
  sayT(tag, "stats: ball losses=%lu reacquired=%lu (avg %lu ms) search=%lu ms | imu timeouts=%lu degraded=%lu safety stops=%lu | output lines dropped=%lu",
       UL(g_stats.ballLossEvents), UL(g_stats.reacquisitions), UL(g_stats.reacquisitions ? g_stats.reacquireTotalMs / g_stats.reacquisitions : 0),
       UL(g_stats.searchTotalMs), UL(g_stats.imuTimeouts), UL(g_stats.degradedTransitions), UL(g_stats.safetyStops), UL(g_outDropped));
  sayT(tag, "stats: shots=%lu | imu re-init attempts=%lu recoveries=%lu | loop gap max %lu us", UL(g_stats.shots), UL(g_imu.initAttempts), UL(g_stats.imuRecoveries), UL(g_loopMaxEverUs));
  sayT(tag, "limits: no wheel feedback (PWM shown is COMMANDED); ring = strength only; ball response unproven until tested; software failsafes do not replace wiring/current limits");
  sayT(tag, "========================================================");
}

// ':raw' - what the hardware actually delivers right now, to find a missing or mis-wired part: every analog input, both module pins, the IMU, free RAM.
static void printRaw(const Config& c) {
  char line[100]; size_t n = 0; line[0] = 0;
  for (uint8_t i = 0; i < RING_CH_MAX; i++) n += (size_t)snprintf(line + n, sizeof(line) - n, "A%u=%u ", (unsigned)i, (unsigned)analogRead(PIN_RECV[i]));
  (void)analogRead(PIN_RECV[g_ringNext]);                            // leave the ADC on the input the running scan expects
  say("INFO", "raw analog (0..1023): %s", line);
  say("INFO", "  an input with nothing connected floats and shows random values; a receiver in the dark sits near 0, in strong light up to ~960");
  for (uint8_t i = 0; i < MOD_COUNT; i++)
    say("INFO", "raw module %u (D%u): pin %S | enabled %S | active when %S | decoded %S", (unsigned)i, (unsigned)PIN_MOD[i], digitalRead(PIN_MOD[i]) ? PSTR("HIGH") : PSTR("LOW"),
        modEnabled(c, i) ? PSTR("yes") : PSTR("no"), ((c.modActiveLow >> i) & 1u) ? PSTR("LOW") : PSTR("HIGH"), g_mod[i].active ? PSTR("DETECTING") : PSTR("clear"));
  say("INFO", "raw underside sensor (D40): pin %S | enabled %S | active when %S | decoded %S%S", digitalRead(P_UNDER) ? PSTR("HIGH") : PSTR("LOW"), c.underEnable ? PSTR("yes") : PSTR("no"),
      c.underActiveLow ? PSTR("LOW") : PSTR("HIGH"), g_under.active ? PSTR("BALL") : PSTR("clear"), g_under.suspectStuck ? PSTR(" (suspect: stuck)") : (g_under.suspectMiss ? PSTR(" (suspect: never answers)") : PSTR("")));
  say("INFO", "IMU: %S%S | free RAM %d B | EEPROM records %u", g_imu.present ? PSTR("present at 0x") : PSTR("NOT FOUND (no answer on D20/D21 at 0x68/0x69)"), g_imu.present ? (g_imu.addr == 0x69 ? PSTR("69") : PSTR("68")) : PSTR(""), freeRam(), (unsigned)g_recCount);
}

// ':pins' - the compiled pin map.
static void printPins() {
  say("INFO", "motors  FR: IN1 D%u IN2 D%u ENA D%u | BR: IN3 D%u IN4 D%u ENB D%u (right L298N)", (unsigned)P_FR_IN1, (unsigned)P_FR_IN2, (unsigned)P_FR_EN, (unsigned)P_BR_IN1, (unsigned)P_BR_IN2, (unsigned)P_BR_EN);
  say("INFO", "motors  FL: IN1 D%u IN2 D%u ENA D%u | BL: IN3 D%u IN4 D%u ENB D%u (left L298N)", (unsigned)P_FL_IN1, (unsigned)P_FL_IN2, (unsigned)P_FL_EN, (unsigned)P_BL_IN1, (unsigned)P_BL_IN2, (unsigned)P_BL_EN);
  say("INFO", "receivers A0..A5 fitted (A6, A7 spare) | modules: front D%u, second D%u (not fitted yet) | underside sensor D%u | MPU6050 SDA D%u SCL D%u", (unsigned)P_MOD0, (unsigned)P_MOD1, (unsigned)P_UNDER, (unsigned)P_SDA, (unsigned)P_SCL);
  say("INFO", "HC-05 on Serial1: TX1 D%u -> divider -> HC-05 RXD, HC-05 TXD -> RX1 D%u | USB serial D0/D1", (unsigned)P_BT_TX, (unsigned)P_BT_RX);
}

// ':status' - one screen: is everything there, and what is the robot doing.
static void printStatus(const Config& c, uint32_t now) {
  say("INFO", "MODE %S (%S) | safety %S%S%S | uptime %lu s | condition %S | nav %S | free RAM %d B", modeName(), g_defendEnabled ? (PGM_P)DEF_NAME[g_defState] : (PGM_P)NAV_NAME[g_navState], safetyText(),
      firstFaultName()[0] ? PSTR(" | fault ") : PSTR(""), firstFaultName(), UL(now / 1000), condName(g_cond), (PGM_P)NAV_NAME[g_navState], freeRam());
  say("INFO", "motors commanded (FL FR BL BR): %d %d %d %d PWM | last command received: '%c' | ball %S bearing %.0f deg confidence %u", (int)g_wheelOut[W_FL], (int)g_wheelOut[W_FR], (int)g_wheelOut[W_BL], (int)g_wheelOut[W_BR],
      g_btLast ? g_btLast : '-', g_ball.valid ? PSTR("seen") : PSTR("not seen"), (double)(g_ball.bearingKnown ? g_ball.bearing : 0.0f), (unsigned)g_ball.confidence);
  say("INFO", "receivers: %u on, %u usable, noise floor %S | ball %S | modules: front %S, second %S | IMU %S%S", (unsigned)ringCountEnabled(c), (unsigned)ringCountUsable(c),
      c.ringCalDone ? PSTR("calibrated") : PSTR("NOT calibrated"), g_ball.valid ? PSTR("seen") : PSTR("not seen"),
      !modEnabled(c, 0) ? PSTR("off") : (g_mod[0].active ? PSTR("DETECTING") : PSTR("clear")), !modEnabled(c, 1) ? PSTR("off") : (g_mod[1].active ? PSTR("DETECTING") : PSTR("clear")),
      g_imu.present ? PSTR("present") : PSTR("MISSING"), g_imu.present ? (imuHeadingUsable(c, now) ? PSTR(", heading usable") : PSTR(", heading NOT usable")) : PSTR(""));
  say("INFO", "mode %S | e-stop latch %S | underside sensor %S | Bluetooth %S | attack tries %u", btModeName(), g_estop ? PSTR("LATCHED") : PSTR("clear"),
      !c.underEnable ? PSTR("off") : (!underUsable(c) ? PSTR("IGNORED (suspect)") : (g_under.active ? PSTR("ball here") : PSTR("clear"))), g_btLink ? PSTR("link up") : PSTR("no link"), (unsigned)g_reCount);
  say("INFO", "active faults: %u (mask 0x%X) | shots %lu | ball losses %lu | records %u | output lines dropped %lu", (unsigned)popcount16(faultMask()), (unsigned)faultMask(), UL(g_stats.shots), UL(g_stats.ballLossEvents), (unsigned)g_recCount, UL(g_outDropped));
}

// 'sensors': receivers, modules, underside sensor and IMU in one block.
static void printSensors(const Config& c, uint32_t now) {
  say("SENS", "receivers (A0..A5): swing / filtered / noise floor / threshold (0..1023 ADC counts; '-' = channel off)");
  for (uint8_t i = 0; i < 6; i++) {
    if (!ringEnabled(c, i)) { say("SENS", "  A%u  off", (unsigned)i); continue; }
    say("SENS", "  A%u  level %u..%u swing %u filtered %.1f noise %u threshold %.0f | %S%S | looks %d deg%S", (unsigned)i, (unsigned)g_ring[i].lo, (unsigned)g_ring[i].hi, (unsigned)g_ring[i].amp, (double)g_ring[i].avg, (unsigned)c.ringNoise[i],
        (double)ringThreshold(c, i), g_ring[i].seen ? PSTR("SEES BALL") : PSTR("quiet"), g_ring[i].railFault ? PSTR(" RAIL-FAULT") : PSTR(""), (int)c.ringBearing[i], c.bearingsSet ? PSTR("") : PSTR(" (default guess)"));
  }
  say("SENS", "ball %S bearing %.1f deg confidence %u strength %.2f centered %S | noise floor %S | bearing map %S", g_ball.valid ? PSTR("DETECTED") : PSTR("none"), (double)(g_ball.bearingKnown ? g_ball.bearing : 0.0f), (unsigned)g_ball.confidence, (double)g_ball.strength,
      g_ball.centered ? PSTR("yes") : PSTR("no"), c.ringCalDone ? PSTR("calibrated") : PSTR("NOT calibrated"), c.bearingsSet ? PSTR("set by operator") : PSTR("default guess - UNVERIFIED"));
  say("SENS", "obstacle modules: D34 %S, D35 %S | underside D40 %S (%S) | shooting position %S", !modEnabled(c, 0) ? PSTR("off") : (g_mod[0].active ? PSTR("DETECTING") : PSTR("clear")),
      !modEnabled(c, 1) ? PSTR("off") : (g_mod[1].active ? PSTR("DETECTING") : PSTR("clear")), !c.underEnable ? PSTR("off") : (g_under.active ? PSTR("BALL") : PSTR("clear")),
      underUsable(c) ? PSTR("trusted") : PSTR("ignored"), g_ball.ready ? PSTR("YES") : PSTR("no"));
  say("SENS", "IMU %S: accel %.2f %.2f %.2f g | gyro %.2f %.2f %.2f dps | roll %.1f pitch %.1f yaw %.1f deg (relative ESTIMATES)", g_imu.present && g_imu.ok ? PSTR("ok") : PSTR("NOT delivering data"),
      (double)g_imu.ax, (double)g_imu.ay, (double)g_imu.az, (double)g_imu.gx, (double)g_imu.gy, (double)g_imu.gz, (double)g_imu.roll, (double)g_imu.pitch, (double)g_imu.yaw);
  (void)now;
}

// 'gyro': calibration quality and the stationary drift diagnostic.
static void printGyro(const Config& c, uint32_t now) {
  say("GYRO", "MPU6050 %S | gyro bias calibration %S | bias in use %.3f dps", g_imu.present ? (g_imu.ok ? PSTR("ok") : PSTR("present but not delivering")) : PSTR("MISSING"),
      g_imu.calibrated ? PSTR("DONE") : (g_imu.calState == 1 ? PSTR("running") : (g_imu.calState == 3 ? PSTR("REJECTED (robot moved)") : PSTR("not done"))), (double)g_imu.bias);
  if (g_imu.calAtMs) say("GYRO", "  accepted calibration: mean %.3f dps, noise sd %.3f dps (limit %.2f), bias standard error %.4f dps, %lu s ago; stationary-limit |bias| < %.1f dps",
      (double)g_imu.calMean, (double)g_imu.calSd, (double)c.imuStillDps, (double)imuBiasSeDps(), UL(elapsedMs(now, g_imu.calAtMs) / 1000), (double)c.imuMaxBiasDps);
  say("GYRO", "  yaw %.2f deg RELATIVE to the last zero; estimated uncertainty %.2f deg (bias standard error only - excludes bias instability, so run 'gyro drift'); roll %.1f pitch %.1f (gravity-corrected ESTIMATES)",
      (double)g_imu.yaw, (double)imuYawUncertDeg(now), (double)g_imu.roll, (double)g_imu.pitch);
  if (g_imu.driftState == 1) say("GYRO", "  drift test RUNNING: %lu of %lu ms (keep the robot still)", UL(elapsedMs(now, g_imu.driftStartMs)), UL(g_imu.driftDurMs));
  else if (g_imu.driftState == 2) {
    say("GYRO", "  drift test DONE (%lu ms): mean raw gz %.3f dps vs calibrated bias %.3f: difference %.3f dps = measured BIAS ERROR (noise sd %.3f)", UL(g_imu.driftDurMs),
        (double)g_imu.driftMeanDps, (double)g_imu.calMean, (double)(g_imu.driftMeanDps - g_imu.bias), (double)g_imu.driftSdDps);
    say("GYRO", "  integrated yaw changed %.2f deg = %.2f deg/min = measured HEADING DRIFT of this run (not a guarantee for a whole match)", (double)g_imu.driftDeltaDeg, (double)g_imu.driftRateDegMin);
  }
  else if (g_imu.driftState == 3) say("GYRO", "  drift test FAILED: %S", g_imu.driftWhy == 1 ? PSTR("the motors moved") : (g_imu.driftWhy == 2 ? PSTR("the gyro was not still") : (g_imu.driftWhy == 4 ? PSTR("the robot was tilted or shaken") : (g_imu.driftWhy == 5 ? PSTR("stopped by the operator") : PSTR("sensor lost")))));
  else say("GYRO", "  no drift test run since boot: 'gyro drift [s]' (robot still, motors stopped)");
  say("GYRO", "  The gyro alone gives a RELATIVE heading that drifts; the accelerometer cannot correct yaw. Nothing here is an absolute heading.");
}

// ---- paced reports -----------------------------------------------------------------------------------
// Every long report is requested with reportRequest() and printed by reportService() from the main loop in as many passes as the small output queue needs.
enum RepKind : uint8_t { REP_NONE, REP_HELP, REP_HEALTH, REP_RING, REP_NAV, REP_IMU, REP_TEST, REP_MOD, REP_RINGONLY, REP_LOG, REP_RAW, REP_PINS, REP_STATUS, REP_SENSORS, REP_GYRO, REP_MOTORS, REP_ALL };
static uint8_t g_repKind = REP_NONE;
static void printHelp();
static void cmdTestModeText();
static void reportRun(uint8_t kind, const Config& c, uint32_t now) {
  switch (kind) {
    case REP_HELP: printHelp(); break;
    case REP_HEALTH: printHealth(c, now, PSTR("INFO")); break;
    case REP_RING: printRingSnapshot(c); printModSnapshot(c); break;
    case REP_NAV: printNavStatus(c, now); break;
    case REP_IMU: printImu(c, now); break;
    case REP_TEST: cmdTestModeText(); break;
    case REP_MOD: printModSnapshot(c); break;
    case REP_RINGONLY: printRingSnapshot(c); break;
    case REP_LOG: recDump(); break;
    case REP_RAW: printRaw(c); break;
    case REP_PINS: printPins(); break;
    case REP_STATUS: printStatus(c, now); break;
    case REP_SENSORS: printSensors(c, now); break;
    case REP_GYRO: printGyro(c, now); break;
    case REP_MOTORS: printMotion(c, now); break;
    case REP_ALL: printStatus(c, now); printSensors(c, now); printGyro(c, now); printMotion(c, now); break;
    default: break;
  }
}
static void reportRequest(uint8_t kind) { g_repKind = kind; g_repSkip = 0; }
static void reportService(const Config& c, uint32_t now) {
  if (g_repKind == REP_NONE || outUsed() > OUTQ_SIZE / 8) return;     // wait until the queue has drained
  g_repActive = true; g_repLine = 0; g_repOverflow = false;
  reportRun(g_repKind, c, now);
  g_repActive = false;
  if (!g_repOverflow) { g_repKind = REP_NONE; g_repSkip = 0; }
}
