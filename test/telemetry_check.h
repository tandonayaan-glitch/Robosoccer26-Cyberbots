// telemetry_check.h - TEST-ONLY C++ reader for the robot's JSON-lines telemetry (docs/TELEMETRY.md), used by the firmware simulation tests to check the frames the
// Mega produces. The real client is Python (laptop/robot_protocol.py), which is tested against the same frames with Python's json module.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const size_t TELE_MAX_LINE = 400;          // longer lines are discarded (the robot sends at most 383 bytes)

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
