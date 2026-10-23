// telemetry_parse.h - portable (no Arduino dependency) parser and link monitor for the robot's JSON-lines telemetry (see docs/TELEMETRY.md).
// Used by remote_esp32.ino and unit-tested on the host (test/test_main.cpp). Frames are single-level JSON objects: numbers, strings, null and flat numeric arrays.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const size_t TELE_MAX_LINE = 320;          // longer lines are discarded (the robot never sends more than 300 bytes)

// ---- tiny JSON accessors (flat objects only) ----
static inline const char* jFind(const char* js, const char* key) {
  char pat[40];
  snprintf(pat, sizeof(pat), "\"%s\":", key);
  const char* p = strstr(js, pat);
  return p ? p + strlen(pat) : nullptr;
}
static inline bool jNum(const char* js, const char* key, float* out) {          // false when absent or null
  const char* p = jFind(js, key);
  if (!p || *p == 'n' || *p == '"' || *p == '[') return false;
  char* e = nullptr;
  float v = strtof(p, &e);
  if (e == p) return false;
  *out = v;
  return true;
}
static inline bool jStr(const char* js, const char* key, char* out, size_t n) {
  const char* p = jFind(js, key);
  if (!p || *p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < n) out[i++] = *p++;
  out[i] = 0;
  return *p == '"';
}
static inline int jArr(const char* js, const char* key, float* out, int max) {  // number of elements read (null counts as NAN)
  const char* p = jFind(js, key);
  if (!p || *p != '[') return 0;
  p++;
  int n = 0;
  while (*p && *p != ']' && n < max) {
    if (strncmp(p, "null", 4) == 0) { out[n++] = NAN; p += 4; }
    else { char* e = nullptr; float v = strtof(p, &e); if (e == p) return n; out[n++] = v; p = e; }
    if (*p == ',') p++;
  }
  return n;
}

// ---- snapshot of the latest values from the three frame kinds ----
struct TeleSnapshot {
  // core ('c')
  char mode[12], sub[12], safety[20], fault[14], lastCmd;
  bool bt, estop, ball, ambiguous, centered, underActive, underOk, shotPos, bearingValid;
  float bearing, conf; float obst[2]; float motorPwm[4];
  // receivers ('r')
  float rawLevel[6], swing[6], filtered[6], noise[6], thresh[6]; float strength, best; char bearingMap[16]; bool cal;
  // imu/diag ('i')
  bool imuOk; int imuCal, drift; float acc[3], gyro[3], roll, pitch, yaw, yawUnc, gyroBias, driftRate, driftBias; bool rollValid, yawValid, driftValid;
  float wheelDir[4]; float movingMs, turningMs, wdTrips, staleCmds, loopAvgUs, loopMaxUs; int faults;
  // link bookkeeping (host clock in ms)
  char lastRaw[4][TELE_MAX_LINE + 2];   // the last good line of each kind (c, r, i, d) for the RAW view
  float txDrops; uint32_t lastCoreMs, lastRecvMs, lastImuMs, lastDiagMs, lastAnyMs; uint16_t lastSeq; bool haveSeq;
  uint32_t frames, bad, lost;
};

static inline void teleInit(TeleSnapshot* s) { memset(s, 0, sizeof(*s)); s->bearing = s->roll = s->pitch = s->yaw = NAN; }

// Parses one line (without the newline). Returns 'c', 'r' or 'i' for a good frame of protocol version 1, otherwise 0 (and counts it as bad).
static inline char teleParseLine(TeleSnapshot* s, const char* line, uint32_t nowMs) {
  size_t len = strlen(line);
  if (len < 10 || len > TELE_MAX_LINE || line[0] != '{' || line[len - 1] != '}') { s->bad++; return 0; }
  float v = 0, seq = 0, ms = 0;
  char k[4];
  if (!jNum(line, "v", &v) || (int)v != 1 || !jStr(line, "k", k, sizeof(k)) || !jNum(line, "n", &seq) || !jNum(line, "ms", &ms)) { s->bad++; return 0; }
  if (k[0] != 'c' && k[0] != 'r' && k[0] != 'i' && k[0] != 'd') { s->bad++; return 0; }
  uint16_t n = (uint16_t)seq;
  if (s->haveSeq) { uint16_t gap = (uint16_t)(n - (uint16_t)(s->lastSeq + 1)); if (gap && gap < 1000) s->lost += gap; }
  s->lastSeq = n; s->haveSeq = true; s->frames++; s->lastAnyMs = nowMs ? nowMs : 1u;
  float f;
  { int ki = k[0] == 'c' ? 0 : (k[0] == 'r' ? 1 : (k[0] == 'i' ? 2 : 3)); strncpy(s->lastRaw[ki], line, TELE_MAX_LINE); s->lastRaw[ki][TELE_MAX_LINE] = 0; }
  if (k[0] == 'c') {
    jStr(line, "mode", s->mode, sizeof(s->mode)); jStr(line, "sub", s->sub, sizeof(s->sub)); jStr(line, "safety", s->safety, sizeof(s->safety)); jStr(line, "fault", s->fault, sizeof(s->fault));
    char cm[4]; s->lastCmd = jStr(line, "cmd", cm, sizeof(cm)) ? cm[0] : '-';
    s->bt = jNum(line, "bt", &f) && f != 0; s->estop = jNum(line, "estop", &f) && f != 0; s->ball = jNum(line, "ball", &f) && f != 0;
    s->ambiguous = jNum(line, "ambiguous", &f) && f != 0; s->centered = jNum(line, "centered", &f) && f != 0;
    s->bearingValid = jNum(line, "bearing_deg", &s->bearing); if (!s->bearingValid) s->bearing = NAN;
    s->conf = jNum(line, "conf", &f) ? f : 0;
    jArr(line, "obst", s->obst, 2);
    s->underActive = jNum(line, "under", &f) && f != 0; s->underOk = jNum(line, "under_ok", &f) && f != 0; s->shotPos = jNum(line, "shot_pos", &f) && f != 0;
    jArr(line, "motor_command_pwm", s->motorPwm, 4);
    s->lastCoreMs = nowMs ? nowMs : 1u;
    return 'c';
  }
  if (k[0] == 'r') {
    jArr(line, "raw_level", s->rawLevel, 6); jArr(line, "swing", s->swing, 6); jArr(line, "filtered", s->filtered, 6); jArr(line, "noise_floor", s->noise, 6); jArr(line, "threshold", s->thresh, 6);
    s->strength = jNum(line, "strength", &f) ? f : 0; s->best = jNum(line, "best", &f) ? f : 0; jStr(line, "map", s->bearingMap, sizeof(s->bearingMap)); s->cal = jNum(line, "cal", &f) && f != 0;
    s->lastRecvMs = nowMs ? nowMs : 1u;
    return 'r';
  }
  if (k[0] == 'i') {
    s->imuOk = jNum(line, "imu", &f) && f != 0; s->imuCal = jNum(line, "imu_cal", &f) ? (int)f : 0; s->drift = jNum(line, "drift", &f) ? (int)f : 0;
    const char* ak[3] = {"ax_g", "ay_g", "az_g"}; const char* gk[3] = {"gx_dps", "gy_dps", "gz_dps"};
    for (int i = 0; i < 3; i++) { if (!jNum(line, ak[i], &s->acc[i])) s->acc[i] = NAN; if (!jNum(line, gk[i], &s->gyro[i])) s->gyro[i] = NAN; }
    s->rollValid = jNum(line, "roll_est_deg", &s->roll); if (!s->rollValid) s->roll = NAN;
    if (!jNum(line, "pitch_est_deg", &s->pitch)) s->pitch = NAN;
    s->yawValid = jNum(line, "yaw_rel_est_deg", &s->yaw); if (!s->yawValid) s->yaw = NAN;
    if (!jNum(line, "yaw_unc_est_deg", &s->yawUnc)) s->yawUnc = NAN;
    if (!jNum(line, "gyro_bias_dps", &s->gyroBias)) s->gyroBias = NAN;
    s->driftValid = jNum(line, "drift_deg_min", &s->driftRate); jNum(line, "drift_bias_dps", &s->driftBias);
    s->lastImuMs = nowMs ? nowMs : 1u;
    return 'i';
  }
  if (k[0] == 'd') {
    jArr(line, "wheel_dir", s->wheelDir, 4);
    s->movingMs = jNum(line, "moving_ms", &f) ? f : 0; s->turningMs = jNum(line, "turning_ms", &f) ? f : 0; s->wdTrips = jNum(line, "wd_trips", &f) ? f : 0; s->staleCmds = jNum(line, "stale_cmds", &f) ? f : 0;
    s->loopAvgUs = jNum(line, "loop_avg_us", &f) ? f : 0; s->loopMaxUs = jNum(line, "loop_max_us", &f) ? f : 0; s->faults = jNum(line, "faults", &f) ? (int)f : 0;
    s->txDrops = jNum(line, "tx_drops", &f) ? f : 0;
    s->lastDiagMs = nowMs ? nowMs : 1u;
    return 'd';
  }
  s->bad++;
  return 0;
}

// Assembles bytes into lines. Bounded: an over-long line is dropped as a whole (counted in `dropped`) and the next newline resyncs.
struct TeleLineBuf {
  char buf[TELE_MAX_LINE + 2]; size_t n; bool overflow; uint32_t dropped;
};
static inline void teleLineInit(TeleLineBuf* b) { memset(b, 0, sizeof(*b)); }
// Returns true when `out` holds a complete line (no newline) ready for teleParseLine().
static inline bool teleLineFeed(TeleLineBuf* b, char ch, char* out, size_t outN) {
  if (ch == '\r') return false;
  if (ch == '\n') {
    bool ok = !b->overflow && b->n > 0;
    if (b->overflow) b->dropped++;
    size_t n = b->n; b->n = 0; b->overflow = false;
    if (!ok) return false;
    if (n >= outN) return false;
    memcpy(out, b->buf, n); out[n] = 0;
    return true;
  }
  if (b->n < TELE_MAX_LINE) b->buf[b->n++] = ch; else b->overflow = true;
  return false;
}

// ---- link health from the host's point of view ----
enum TeleLink { TL_NO_DATA, TL_OK, TL_STALE, TL_LOST };
static inline TeleLink teleLinkState(const TeleSnapshot* s, uint32_t nowMs, bool btConnected, uint32_t staleMs = 1500, uint32_t lostMs = 5000) {
  if (!btConnected) return TL_LOST;
  if (!s->lastAnyMs) return TL_NO_DATA;
  uint32_t age = nowMs - s->lastAnyMs;
  if (age > lostMs) return TL_LOST;
  if (age > staleMs) return TL_STALE;
  return TL_OK;
}
static inline const char* teleLinkText(TeleLink l) { return l == TL_OK ? "OK" : (l == TL_STALE ? "STALE" : (l == TL_NO_DATA ? "NO DATA YET" : "LINK LOST")); }

// ---- console viewer: renders the snapshot into `out`; every value shows 'n/a' when the robot did not provide it, and a section older than staleMs is marked STALE ----
static inline void teleFmt(char* o, size_t n, float v, int prec) { if (isnan(v)) snprintf(o, n, "n/a"); else snprintf(o, n, "%.*f", prec, (double)v); }
static inline size_t teleRender(const TeleSnapshot* s, uint32_t nowMs, bool btConnected, char* out, size_t outN) {
  TeleLink l = teleLinkState(s, nowMs, btConnected);
  auto age = [&](uint32_t t) -> const char* { return (!t || nowMs - t > 1500) ? " [STALE/none]" : ""; };
  char a[12], b[12], c[12];
  size_t o = 0;
  o += snprintf(out + o, outN - o, "== robot link: %s | frames %lu, bad %lu, lost %lu ==\n", teleLinkText(l), (unsigned long)s->frames, (unsigned long)s->bad, (unsigned long)s->lost);
  if (l == TL_LOST || l == TL_NO_DATA) { o += snprintf(out + o, outN - o, "(no current telemetry: values below are the last known ones, if any)\n"); }
  o += snprintf(out + o, outN - o, "mode %s/%s safety %s fault '%s' estop %d robot-sees-remote %d last-cmd %c%s\n", s->mode, s->sub, s->safety, s->fault, s->estop, s->bt, s->lastCmd ? s->lastCmd : '-', age(s->lastCoreMs));
  teleFmt(a, sizeof(a), s->bearing, 1);
  o += snprintf(out + o, outN - o, "ball %d bearing %s deg conf %.0f%s%s | obstacle %.0f %.0f | underside %d (trusted %d) shot-position %d\n", s->ball, a, s->conf, s->ambiguous ? " AMBIGUOUS" : "", s->centered ? " centered" : "",
                s->obst[0], s->obst[1], s->underActive, s->underOk, s->shotPos);
  o += snprintf(out + o, outN - o, "motor command PWM FL %.0f FR %.0f BL %.0f BR %.0f (commands, not measured wheel speed)\n", s->motorPwm[0], s->motorPwm[1], s->motorPwm[2], s->motorPwm[3]);
  o += snprintf(out + o, outN - o, "receivers swing  %.0f %.0f %.0f %.0f %.0f %.0f | threshold %.0f %.0f %.0f %.0f %.0f %.0f | bearing map %s%s\n", s->swing[0], s->swing[1], s->swing[2], s->swing[3], s->swing[4], s->swing[5],
                s->thresh[0], s->thresh[1], s->thresh[2], s->thresh[3], s->thresh[4], s->thresh[5], s->bearingMap, age(s->lastRecvMs));
  teleFmt(a, sizeof(a), s->roll, 1); teleFmt(b, sizeof(b), s->pitch, 1); teleFmt(c, sizeof(c), s->yaw, 1);
  o += snprintf(out + o, outN - o, "imu %s | roll %s pitch %s yaw %s deg (relative ESTIMATES) | gyro bias ", s->imuOk ? "ok" : "NOT OK", a, b, c);
  teleFmt(a, sizeof(a), s->gyroBias, 3); teleFmt(b, sizeof(b), s->yawUnc, 2);
  o += snprintf(out + o, outN - o, "%s dps | yaw uncertainty est %s deg | drift test %d", a, b, s->drift);
  if (s->driftValid) o += snprintf(out + o, outN - o, " = %.2f deg/min", (double)s->driftRate);
  o += snprintf(out + o, outN - o, "%s\n", age(s->lastImuMs));
  return o;
}

// ---- selectable views (the ESP32 console words `ir`, `imu`, `motors`, `ball`, `safety`, `raw`, and `all` / `back` for the normal full view) ----
enum TeleView : uint8_t { V_ALL, V_IR, V_IMU, V_MOTORS, V_BALL, V_SAFETY, V_RAW, V_COUNT };
static inline const char* teleViewName(uint8_t v) {
  static const char* n[V_COUNT] = {"ALL", "IR sensors", "IMU / gyro", "MOTORS", "BALL", "SAFETY", "RAW frames"};
  return v < V_COUNT ? n[v] : "?";
}
// Which frame kinds the robot should be asked for (`tele now <kind>`) so the selected view is fresh: always the core frame plus one more.
static inline char teleViewExtraKind(uint8_t v) { return v == V_IR ? 'r' : (v == V_IMU ? 'i' : (v == V_MOTORS ? 'd' : (v == V_ALL ? 'r' : 0))); }

static inline const char* teleAge(const TeleSnapshot* s, uint32_t t, uint32_t now) { (void)s; return (!t || now - t > 2500) ? "  [STALE or never received]" : ""; }

static inline size_t teleRenderView(const TeleSnapshot* s, uint8_t view, uint32_t nowMs, bool sessionOk, char* out, size_t outN) {
  if (view == V_ALL || view >= V_COUNT) return teleRender(s, nowMs, sessionOk, out, outN);
  TeleLink l = teleLinkState(s, nowMs, sessionOk);
  size_t o = 0;
  char a[16], b[16], c[16];
  o += snprintf(out + o, outN - o, "== view: %s | robot link: %s | type 'back' for the normal view, 'help' for commands ==\n", teleViewName(view), teleLinkText(l));
  if (l == TL_LOST || l == TL_NO_DATA) { o += snprintf(out + o, outN - o, "(no current data from the robot: values are the last ones received, if any)\n"); }
  if (view == V_IR) {
    o += snprintf(out + o, outN - o, "receiver      A0   A1   A2   A3   A4   A5%s\n", teleAge(s, s->lastRecvMs, nowMs));
    o += snprintf(out + o, outN - o, "level        %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f   (ADC 0..1023, middle of the window)\n", s->rawLevel[0], s->rawLevel[1], s->rawLevel[2], s->rawLevel[3], s->rawLevel[4], s->rawLevel[5]);
    o += snprintf(out + o, outN - o, "swing        %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f   (peak-to-peak: the ball's signal)\n", s->swing[0], s->swing[1], s->swing[2], s->swing[3], s->swing[4], s->swing[5]);
    o += snprintf(out + o, outN - o, "filtered     %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f\n", s->filtered[0], s->filtered[1], s->filtered[2], s->filtered[3], s->filtered[4], s->filtered[5]);
    o += snprintf(out + o, outN - o, "noise floor  %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f   (-1 = channel off)\n", s->noise[0], s->noise[1], s->noise[2], s->noise[3], s->noise[4], s->noise[5]);
    o += snprintf(out + o, outN - o, "threshold    %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f   (swing above this = sees the ball)\n", s->thresh[0], s->thresh[1], s->thresh[2], s->thresh[3], s->thresh[4], s->thresh[5]);
    o += snprintf(out + o, outN - o, "obstacle modules: D34 %s, D35 %s | underside D40: %s (%s) | shooting position: %s%s\n", s->obst[0] != 0 ? "DETECTING" : "clear", s->obst[1] != 0 ? "DETECTING" : "clear",
                  s->underActive ? "BALL" : "clear", s->underOk ? "trusted" : "IGNORED", s->shotPos ? "YES" : "no", teleAge(s, s->lastCoreMs, nowMs));
    o += snprintf(out + o, outN - o, "bearing map: %s | noise floor calibrated: %s\n", s->bearingMap, s->cal ? "yes" : "NO");
  } else if (view == V_IMU) {
    teleFmt(a, sizeof(a), s->acc[0], 3); teleFmt(b, sizeof(b), s->acc[1], 3); teleFmt(c, sizeof(c), s->acc[2], 3);
    o += snprintf(out + o, outN - o, "imu %s | calibration %d (0 none, 1 running, 2 done, 3 rejected)%s\n", s->imuOk ? "ok" : "NOT delivering data", s->imuCal, teleAge(s, s->lastImuMs, nowMs));
    o += snprintf(out + o, outN - o, "accel g   x %s  y %s  z %s\n", a, b, c);
    teleFmt(a, sizeof(a), s->gyro[0], 2); teleFmt(b, sizeof(b), s->gyro[1], 2); teleFmt(c, sizeof(c), s->gyro[2], 2);
    o += snprintf(out + o, outN - o, "gyro dps  x %s  y %s  z %s\n", a, b, c);
    teleFmt(a, sizeof(a), s->roll, 1); teleFmt(b, sizeof(b), s->pitch, 1); teleFmt(c, sizeof(c), s->yaw, 1);
    o += snprintf(out + o, outN - o, "roll %s  pitch %s  yaw %s deg  (relative ESTIMATES; yaw from the gyro only and drifts)\n", a, b, c);
    teleFmt(a, sizeof(a), s->gyroBias, 3); teleFmt(b, sizeof(b), s->yawUnc, 2);
    o += snprintf(out + o, outN - o, "gyro bias %s dps | yaw uncertainty estimate %s deg | drift test state %d", a, b, s->drift);
    if (s->driftValid) o += snprintf(out + o, outN - o, " -> %.2f deg/min", (double)s->driftRate);
    o += snprintf(out + o, outN - o, "\n");
  } else if (view == V_MOTORS) {
    o += snprintf(out + o, outN - o, "motor COMMAND PWM   FL %4.0f  FR %4.0f  BL %4.0f  BR %4.0f   (commands only: no encoder or current sensor exists)%s\n", s->motorPwm[0], s->motorPwm[1], s->motorPwm[2], s->motorPwm[3], teleAge(s, s->lastCoreMs, nowMs));
    o += snprintf(out + o, outN - o, "commanded direction FL %+.0f  FR %+.0f  BL %+.0f  BR %+.0f   (-1 backward, 0 stopped, +1 forward)%s\n", s->wheelDir[0], s->wheelDir[1], s->wheelDir[2], s->wheelDir[3], teleAge(s, s->lastDiagMs, nowMs));
    o += snprintf(out + o, outN - o, "moving for %.0f ms | rotating without translation for %.0f ms | watchdog trips %.0f | stale commands cleared %.0f\n", s->movingMs, s->turningMs, s->wdTrips, s->staleCmds);
  } else if (view == V_BALL) {
    teleFmt(a, sizeof(a), s->bearing, 1);
    o += snprintf(out + o, outN - o, "ball %s | bearing %s deg (0 = ahead, + = right) | confidence %.0f%s%s%s\n", s->ball ? "SEEN" : "not seen", a, s->conf, s->ambiguous ? " | AMBIGUOUS (receivers disagree)" : "", s->centered ? " | centered" : "", teleAge(s, s->lastCoreMs, nowMs));
    o += snprintf(out + o, outN - o, "strength %.2f (1.0 = very close; not a distance) | strongest receiver A%.0f\n", s->strength, s->best);
  } else if (view == V_SAFETY) {
    o += snprintf(out + o, outN - o, "mode %s / %s | safety %s | fault '%s' | e-stop %s | robot hears remote: %s | last command %c%s\n", s->mode, s->sub, s->safety, s->fault, s->estop ? "LATCHED" : "clear", s->bt ? "yes" : "no", s->lastCmd ? s->lastCmd : '-', teleAge(s, s->lastCoreMs, nowMs));
    o += snprintf(out + o, outN - o, "fault mask %d | loop avg %.0f us, max %.0f us | telemetry frames %lu, bad %lu, lost %lu | robot tx drops %.0f%s\n", s->faults, s->loopAvgUs, s->loopMaxUs,
                  (unsigned long)s->frames, (unsigned long)s->bad, (unsigned long)s->lost, s->txDrops, teleAge(s, s->lastDiagMs, nowMs));
  } else {   // V_RAW
    const char* names = "crid";
    for (int i = 0; i < 4; i++) o += snprintf(out + o, outN - o, "[%c] %s\n", names[i], s->lastRaw[i][0] ? s->lastRaw[i] : "(none received)");
  }
  return o;
}
