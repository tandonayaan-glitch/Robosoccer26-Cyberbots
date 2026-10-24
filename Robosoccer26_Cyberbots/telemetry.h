// telemetry.h - JSON-lines telemetry from the Mega to the laptop over the HC-05.
//
// One frame = one line of JSON ending in '\n', protocol version "v":1, sequence "n" (wraps at 65535), uptime "ms". Four frame kinds, each below 384 bytes:
//   "k":"c"  core      mode, safety, link, ball, obstacle / underside sensors, motor commands
//   "k":"r"  receivers raw level, swing, filtered, noise floor, threshold for A0..A5 (-1 = channel not enabled)
//   "k":"i"  imu       accelerometer, gyro, roll/pitch/yaw ESTIMATES, gyro bias, drift diagnostic
//   "k":"d"  diag      motor command directions and durations, loop timing, watchdog, fault mask, error counters, telemetry/link counters
// Values the hardware cannot provide are never invented: they are null or absent. There is NO distance, NO motor current, NO wheel speed and NO absolute heading
// here; the motor figures are the PWM the firmware COMMANDED.
//
// Pacing: a frame is built only when the transmit queue is empty, and the rate is limited so telemetry uses at most ~60 % of the UART's capacity. At the HC-05's
// factory 9600 baud that is about 2-3 frames per second in total; raise the HC-05 baud (docs/BLUETOOTH.md) for more. The control loop never waits for it.
#pragma once

struct TeleState {
  uint16_t seq;
  uint32_t lastMs, sent, dropped, lastLen, slot;
  uint8_t lastKind;
  bool usb;
  uint32_t loopAvgUs;
};
static TeleState g_tele;
// An explicit terminal request keeps showing that sensor group until another view is selected.
// 0 follows the configured schedule; 'a' cycles all four frame kinds.
static char g_teleBuf[384];
static const uint16_t TELE_BUF = sizeof(g_teleBuf);

// Appends to g_teleBuf; once the buffer is full, further appends are no-ops and the frame is discarded (never sent truncated).
static size_t g_teleN;
#define JADD(fmt, ...) do { if (g_teleN < TELE_BUF) g_teleN += (size_t)snprintf(g_teleBuf + g_teleN, TELE_BUF - g_teleN, fmt, ##__VA_ARGS__); } while (0)

static void jsonOpen(char kind, uint32_t now) {
  g_teleN = 0;
  JADD("{\"v\":1,\"k\":\"%c\",\"n\":%u,\"ms\":%lu", kind, (unsigned)g_tele.seq, UL(now));
}
static void jsonFloatOrNull(const char* key, bool ok, float v, uint8_t prec) {
  if (!ok || !isfinite(v)) { JADD(",\"%s\":null", key); return; }
  JADD(",\"%s\":%.*f", key, (int)prec, (double)v);
}

static void teleBuildCore(const Config& c, uint32_t now) {
  jsonOpen('c', now);
  JADD(",\"mode\":\"%S\",\"sub\":\"%S\",\"safety\":\"%S\",\"fault\":\"%S\",\"cond\":\"%S\"", modeName(),
       g_defendEnabled ? (PGM_P)DEF_NAME[g_defState] : (PGM_P)NAV_NAME[g_navState], safetyText(), firstFaultName(), condName(g_cond));
  JADD(",\"bt\":%u,\"estop\":%u,\"cmd\":\"%c\"", g_btLink ? 1u : 0u, g_estop ? 1u : 0u, g_btLast ? g_btLast : '-');
  JADD(",\"ball\":%u", g_ball.valid ? 1u : 0u);
  jsonFloatOrNull("bearing_deg", g_ball.bearingKnown && !g_ball.ambiguous, g_ball.bearing, 1);   // null when the receivers disagree
  JADD(",\"conf\":%u,\"ambiguous\":%u,\"centered\":%u", (unsigned)g_ball.confidence, g_ball.ambiguous ? 1u : 0u, g_ball.centered ? 1u : 0u);
  JADD(",\"obst\":[%u,%u],\"under\":%u,\"under_ok\":%u,\"shot_pos\":%u", g_mod[0].active ? 1u : 0u, g_mod[1].active ? 1u : 0u, g_under.active ? 1u : 0u,
       underUsable(c) ? 1u : 0u, g_ball.ready ? 1u : 0u);
  JADD(",\"motor_command_pwm\":[%d,%d,%d,%d]}", (int)g_wheelOut[W_FL], (int)g_wheelOut[W_FR], (int)g_wheelOut[W_BL], (int)g_wheelOut[W_BR]);
}

static void teleBuildRecv(const Config& c, uint32_t now) {
  jsonOpen('r', now);
  const char* keys[5] = {"raw_level", "swing", "filtered", "noise_floor", "threshold"};
  for (uint8_t k = 0; k < 5; k++) {
    JADD(",\"%s\":[", keys[k]);
    for (uint8_t i = 0; i < 6; i++) {
      long v = -1;
      if (ringEnabled(c, i)) {
        if (k == 0) v = (long)((g_ring[i].lo + g_ring[i].hi) / 2);
        else if (k == 1) v = (long)g_ring[i].amp;
        else if (k == 2) v = (long)(g_ring[i].avg + 0.5f);
        else if (k == 3) v = (long)c.ringNoise[i];
        else v = (long)(ringThreshold(c, i) + 0.5f);
      }
      JADD("%s%ld", i ? "," : "", v);
    }
    JADD("]");
  }
  JADD(",\"enabled\":%u,\"usable\":%u,\"seen\":%u,\"best\":%u,\"strength\":%.2f,\"cal\":%u,\"map\":\"%S\"}", (unsigned)(c.ringEnable & 0x3F), (unsigned)ringCountUsable(c),
       (unsigned)g_ball.seenMask, (unsigned)g_ball.bestCh, (double)g_ball.strength, c.ringCalDone ? 1u : 0u, c.bearingsSet ? PSTR("operator") : PSTR("default_guess"));
}

static void teleBuildImu(const Config& c, uint32_t now) {
  (void)c;
  jsonOpen('i', now);
  bool ok = g_imu.present && g_imu.ok;
  JADD(",\"imu\":%u,\"imu_cal\":%u", ok ? 1u : 0u, (unsigned)g_imu.calState);       // imu_cal: 0 none, 1 running, 2 done, 3 rejected
  jsonFloatOrNull("ax_g", ok, g_imu.ax, 3); jsonFloatOrNull("ay_g", ok, g_imu.ay, 3); jsonFloatOrNull("az_g", ok, g_imu.az, 3);
  jsonFloatOrNull("gx_dps", ok, g_imu.gx, 2); jsonFloatOrNull("gy_dps", ok, g_imu.gy, 2); jsonFloatOrNull("gz_dps", ok, g_imu.gz, 2);
  jsonFloatOrNull("roll_est_deg", ok && g_imu.tiltValid, g_imu.roll, 1); jsonFloatOrNull("pitch_est_deg", ok && g_imu.tiltValid, g_imu.pitch, 1);
  jsonFloatOrNull("yaw_rel_est_deg", ok && g_imu.yawValid, g_imu.yaw, 1);
  jsonFloatOrNull("yaw_unc_est_deg", g_imu.calibrated, imuYawUncertDeg(now), 2);
  jsonFloatOrNull("gyro_bias_dps", g_imu.calibrated, g_imu.bias, 3);
  JADD(",\"drift\":%u", (unsigned)g_imu.driftState);
  jsonFloatOrNull("drift_deg_min", g_imu.driftState == 2, g_imu.driftRateDegMin, 2);
  jsonFloatOrNull("drift_bias_dps", g_imu.driftState == 2, g_imu.driftMeanDps, 3);
  JADD("}");
}

static void teleBuildDiag(const Config& c, uint32_t now) {
  jsonOpen('d', now);
  JADD(",\"wheel_dir\":[%d,%d,%d,%d],\"moving_ms\":%lu,\"turning_ms\":%lu,\"wd_trips\":%lu,\"stale_cmds\":%lu", (int)g_motion.dir[W_FL], (int)g_motion.dir[W_FR], (int)g_motion.dir[W_BL], (int)g_motion.dir[W_BR],
       UL(g_motion.moveSinceMs ? elapsedMs(now, g_motion.moveSinceMs) : 0), UL(g_motion.turnSinceMs ? elapsedMs(now, g_motion.turnSinceMs) : 0), UL(g_motion.wdTrips), UL(g_motion.staleEvents));
  JADD(",\"loop_avg_us\":%lu,\"loop_max_us\":%lu,\"hw_wdt\":%u,\"faults\":%u,\"imu_err\":%lu,\"calibrated\":%u", UL(g_tele.loopAvgUs), UL(g_loopMaxEverUs),
       (unsigned)RS_USE_WATCHDOG, (unsigned)faultMask(), UL(g_imu.errCount), (unsigned)(c.ringCalDone ? 1 : 0));
  JADD(",\"tx_drops\":%lu,\"cmds\":%lu,\"cmd_unknown\":%lu,\"def_state\":\"%S\"}", UL(g_btReplyDrops + g_tele.dropped), UL(g_btCmds), UL(g_btUnknown), (PGM_P)DEF_NAME[g_defState]);
}

static bool teleSend(const Config& c, uint32_t now, char kind, bool force) {
  if (!c.btEnable && !g_tele.usb) return false;
  if (kind == 'c') teleBuildCore(c, now); else if (kind == 'r') teleBuildRecv(c, now); else if (kind == 'i') teleBuildImu(c, now); else teleBuildDiag(c, now);
  g_tele.seq++;
  if (g_teleN >= TELE_BUF - 1) { g_tele.dropped++; return false; }               // would not fit: discarded, never sent cut off
  if (g_tele.usb && outUsed() < OUTQ_SIZE / 4) say("TELE", "%s", g_teleBuf);
  if (!c.btEnable || !btAuthOk()) return true;
  if (!btTxPut(g_teleBuf, g_teleN, true)) { g_tele.dropped++; (void)force; return false; }
  g_tele.sent++; g_tele.lastLen = (uint32_t)g_teleN + 1u; g_tele.lastKind = (uint8_t)kind; g_tele.lastMs = now;
  return true;
}
static void teleSendNow(const Config& c, uint32_t now, char kind) { teleSend(c, now, kind, true); }

static void teleService(const Config& c, uint32_t now) {
  if (!c.teleEnable && !g_teleView) return;
  if (!btAuthOk()) return;                                                        // no telemetry to a peer that has not authenticated
  if (btTxUsed() != 0) return;                                                    // never queue frames behind frames
  uint32_t period = 1000u / (c.teleHz ? c.teleHz : 1u);
  uint32_t cap = btBaudNow(c) / 10u;                                              // bytes per second the UART can carry (8N1)
  uint32_t budgetGap = g_tele.lastLen ? (g_tele.lastLen * 1000u * 10u) / (cap * 6u) : 0u;   // keep telemetry under ~60 % of the capacity
  uint32_t gap = period > budgetGap ? period : budgetGap;
  if (g_tele.lastMs && elapsedMs(now, g_tele.lastMs) < gap) return;
  char kind = 'c';
  if (g_teleView == 'r' || g_teleView == 'i') kind = g_teleView;
  else if (g_teleView == 'a') { static const char order[4] = {'c', 'r', 'i', 'd'}; kind = order[g_tele.slot % 4u]; }
  else {
    uint8_t every = c.teleDetailEvery ? c.teleDetailEvery : 1u;
    if (g_tele.slot % every == 0 && g_tele.slot != 0) { uint8_t m = (uint8_t)((g_tele.slot / every) % 3u); kind = m == 1 ? 'r' : (m == 2 ? 'i' : 'd'); }
  }
  g_tele.slot++;
  if (!g_tele.lastMs) g_tele.lastMs = now ? now : 1u;
  teleSend(c, now, kind, false);
}

static void printTele(const Config& c, uint32_t now) {
  uint32_t cap = btBaudNow(c) / 10u;
  say("INFO", "telemetry: %S over Bluetooth, requested %u Hz, detail frame every %u; frames sent %lu, dropped %lu, last frame %lu bytes (%S), %lu ms ago | link capacity %lu B/s at %lu baud | USB echo %S",
      c.teleEnable ? PSTR("ON") : PSTR("OFF"), (unsigned)c.teleHz, (unsigned)c.teleDetailEvery, UL(g_tele.sent), UL(g_tele.dropped), UL(g_tele.lastLen),
      g_tele.lastKind == 'r' ? PSTR("receivers") : (g_tele.lastKind == 'i' ? PSTR("imu") : (g_tele.lastKind == 'd' ? PSTR("diag") : PSTR("core"))), UL(g_tele.lastMs ? elapsedMs(now, g_tele.lastMs) : 0), UL(cap), UL(btBaudNow(c)), g_tele.usb ? PSTR("ON") : PSTR("off"));
}
