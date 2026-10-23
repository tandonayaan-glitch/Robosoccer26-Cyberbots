// logger.h - diagnostics over Serial. The ESP32 version wrote CSV files to flash and kept a RAM flight recorder; the Mega has 8 KB of RAM and no
// filesystem, so the same CSV row format is STREAMED over the USB serial port while ':diag on' is active (capture it in the Serial Monitor / a terminal).
// Events (faults, recoveries, navigation transitions) are streamed as rows of type E/F/R/N when ':diag on' is active.
#pragma once
#include "ring.h"
#include "motors.h"
#include "imu.h"
#include "storage.h"

#define FW_VERSION "2.0.0-mega"
#define LOG_SCHEMA 3
// t_ms,type,cond,nav,ringmask,ball,bearing_deg,strength,flags,imu,yaw_deg,herr_deg,gz_dps,vx,vy,w,pwm_fl,pwm_fr,pwm_bl,pwm_br,faults,loop_us,note

enum NavState : uint8_t {
  NAV_IDLE, NAV_MANUAL, NAV_SEEK, NAV_APPROACH, NAV_SEARCH, NAV_SEARCH_PAUSE, NAV_WAIT, NAV_TEST, NAV_SAFE_STOP, NAV_SHOOT, NAV_REVERSE, NAV_DEFEND, NAV_COUNT
};
static const char NAV_NAME[NAV_COUNT][13] PROGMEM = {"IDLE", "MANUAL", "SEEK", "APPROACH", "SEARCH", "SEARCH_PAUSE", "WAIT", "TEST", "SAFE_STOP", "SHOOT", "REVERSE", "DEFEND"};
static NavState g_navState = NAV_IDLE;
static float g_hdgErr = NAN;               // degrees; NaN when heading hold is not active
static uint32_t g_loopMaxUs = 0;           // longest loop() iteration since the last snapshot
static uint32_t g_loopMaxEverUs = 0;

static bool g_csvOn = false;               // ':diag on' - CSV rows over Serial
static uint32_t g_lastSnapMs = 0;

static void logHeader() {
  say("LOG", "#schema=3;fw=2.0.0-mega");
  say("LOG", "t_ms,type,cond,nav,ringmask,ball,bearing_deg,strength,flags,imu,yaw_deg,herr_deg,gz_dps,vx,vy,w,pwm_fl,pwm_fr,pwm_bl,pwm_br,faults,loop_us,note");
}

// One CSV row (a raw line, no [TAG] prefix, so the stream can be saved as a .csv). `type`: S snapshot, E event, F fault, R recovery, N navigation transition.
static void logRow(char type, const char* note) {
  char b[OUT_LINE_MAX];
  char yaw[10] = "", herr[10] = "", gz[10] = "", brg[10] = "";
  if (g_imu.present && g_imu.yawValid) snprintf(yaw, sizeof(yaw), "%.1f", (double)g_imu.yaw);
  if (!isnan(g_hdgErr)) snprintf(herr, sizeof(herr), "%.1f", (double)g_hdgErr);
  if (g_imu.present && g_imu.ok) snprintf(gz, sizeof(gz), "%.1f", (double)g_imu.gz);
  if (g_ball.bearingKnown) snprintf(brg, sizeof(brg), "%.1f", (double)g_ball.bearing);
  uint8_t flags = (uint8_t)((g_ball.held ? 1 : 0) | (g_ball.ready ? 2 : 0) | (g_mod[0].active ? 4 : 0) | (g_mod[1].active ? 8 : 0) | (g_under.active ? 16 : 0));
  snprintf(b, sizeof(b), "%lu,%c,%u,%u,%04X,%u,%s,%.2f,%u,%u,%s,%s,%s,%.2f,%.2f,%.2f,%d,%d,%d,%d,%04X,%lu,",
           UL(millis()), type, (unsigned)g_cond, (unsigned)g_navState, (unsigned)g_ball.seenMask, g_ball.valid ? 1u : 0u, brg, (double)g_ball.strength, (unsigned)flags,
           g_imu.present ? (g_imu.ok ? 2u : 1u) : 0u, yaw, herr, gz, (double)g_reqVx, (double)g_reqVy, (double)g_reqW,
           (int)g_wheelOut[W_FL], (int)g_wheelOut[W_FR], (int)g_wheelOut[W_BL], (int)g_wheelOut[W_BR], (unsigned)faultMask(), UL(g_loopMaxUs));
  size_t l = strlen(b);
  size_t room = sizeof(b) - l - 2;
  for (size_t i = 0; note && note[i] && i < room; i++) { char ch = note[i]; b[l++] = (ch == ',' || ch == '\n' || ch == '\r') ? ';' : ch; }
  b[l++] = '\n'; b[l] = 0;
  outPutLine(b);
}

static void logEvent(const char* type, const char* note) {
  if (g_csvOn) logRow(type[0], note);
}

// Periodic snapshot rows while ':diag on' is active (called every loop; cheap otherwise).
static void logService(const Config& c, uint32_t nowMs) {
  if (!g_csvOn) return;
  if (elapsedMs(nowMs, g_lastSnapMs) < c.snapshotMs) return;
  g_lastSnapMs = nowMs;
  logRow('S', "");
  g_loopMaxUs = 0;
}

// ---- event records in the EEPROM ("records") -----------------------------------------------------------------------------------
// The Mega has no filesystem, but its 4 KB EEPROM can keep a few hundred short event records across power cycles: boot, faults, recoveries, navigation
// transitions, ball losses, shots, SAFE_STOP, configuration saves. They are read back with ':log' (and saved to records.txt on the PC with tools/get_records.py).
// Layout: bytes 0..511 hold the configuration, bytes 512..4095 a ring of 12-byte records. A record is only written when something happens (a few per
// second at most), each EEPROM cell is rewritten once per full turn of the ring (about 300 events), so wear is not a concern.
#define REC_BASE 512
#define REC_BYTES 12
#define REC_SLOTS ((4096 - REC_BASE) / REC_BYTES)
struct Rec { uint8_t magic, type; uint16_t arg; uint32_t t; uint16_t seq, chk; };
static uint16_t g_recNext = 0, g_recSeq = 1, g_recCount = 0;
static uint32_t g_recDropped = 0, g_recWindowMs = 0;
static uint8_t g_recInWindow = 0;
static bool g_recLive = false;

static uint16_t recChk(const Rec& r) { return (uint16_t)(0x5A5Au ^ r.type ^ r.arg ^ (uint16_t)r.t ^ (uint16_t)(r.t >> 16) ^ r.seq); }
static bool recRead(uint16_t slot, Rec* r) {
  EEPROM.get(REC_BASE + (int)slot * REC_BYTES, *r);
  return r->magic == 0xA5 && r->chk == recChk(*r);
}
static void recInit() {                       // find the newest record after a power cycle
  uint16_t best = 0, bestSeq = 0, n = 0;
  bool any = false;
  for (uint16_t s = 0; s < REC_SLOTS; s++) {
    Rec r;
    if (!recRead(s, &r)) continue;
    n++;
    if (!any || (int16_t)(r.seq - bestSeq) > 0) { best = s; bestSeq = r.seq; any = true; }
  }
  g_recCount = n;
  g_recNext = any ? (uint16_t)((best + 1) % REC_SLOTS) : 0;
  g_recSeq = any ? (uint16_t)(bestSeq + 1) : 1;
}

static PGM_P recName(uint8_t t) {
  switch (t) {
    case REC_BOOT: return PSTR("BOOT");
    case REC_FAULT: return PSTR("FAULT");
    case REC_RECOVER: return PSTR("RECOVER");
    case REC_NAV: return PSTR("NAV");
    case REC_BALL_LOST: return PSTR("BALL_LOST");
    case REC_SHOT: return PSTR("SHOT");
    case REC_SAFE_STOP: return PSTR("SAFE_STOP");
    case REC_CFG_SAVED: return PSTR("CFG_SAVED");
    case REC_ESTOP: return PSTR("ESTOP");
    case REC_MODE: return PSTR("MODE");
    case REC_REATTACK: return PSTR("REATTACK");
    default: return PSTR("NOTE");
  }
}
// One record as a text line: "seq,t_ms,EVENT,detail" (no tag, so a saved dump is a plain CSV-like text file).
static void recPrint(const Rec& r) {
  char b[OUT_LINE_MAX];
  int p = snprintf(b, sizeof(b), "%u,%lu,%S,", (unsigned)r.seq, UL(r.t), recName(r.type));
  char* d = b + p; size_t room = sizeof(b) - (size_t)p - 2;
  switch (r.type) {
    case REC_FAULT: case REC_RECOVER: snprintf(d, room, "%S", r.arg < F_COUNT ? faultCode((uint8_t)r.arg) : PSTR("?")); break;
    case REC_NAV: { uint8_t f = (uint8_t)(r.arg >> 8), to = (uint8_t)r.arg; snprintf(d, room, "%S>%S", f < NAV_COUNT ? (PGM_P)NAV_NAME[f] : PSTR("?"), to < NAV_COUNT ? (PGM_P)NAV_NAME[to] : PSTR("?")); break; }
    case REC_BALL_LOST: snprintf(d, room, "last_bearing_deg=%d", (int)(int16_t)r.arg); break;
    case REC_SHOT: snprintf(d, room, "shot_number=%u", (unsigned)r.arg); break;
    case REC_MODE: snprintf(d, room, "%S", r.arg == 2 ? PSTR("AUTO") : (r.arg == 1 ? PSTR("MANUAL") : PSTR("IDLE"))); break;
    case REC_ESTOP: snprintf(d, room, "%S", r.arg ? PSTR("latched") : PSTR("cleared")); break;
    case REC_REATTACK: snprintf(d, room, "try=%u", (unsigned)r.arg); break;
    default: snprintf(d, room, "%u", (unsigned)r.arg); break;
  }
  size_t l = strlen(b); b[l] = '\n'; b[l + 1] = 0;
  outPutLine(b);
}
static void recAdd(uint8_t type, uint16_t arg) {
  uint32_t now = millis();
  if (type != REC_BOOT && type != REC_SAFE_STOP && type != REC_FAULT) {         // flood guard: at most 6 routine records per second
    if (elapsedMs(now, g_recWindowMs) >= 1000) { g_recWindowMs = now; g_recInWindow = 0; }
    if (g_recInWindow >= 6) { g_recDropped++; return; }
    g_recInWindow++;
  }
  Rec r; memset(&r, 0, sizeof(r));
  r.magic = 0xA5; r.type = type; r.arg = arg; r.t = now; r.seq = g_recSeq++;
  r.chk = recChk(r);
  EEPROM.put(REC_BASE + (int)g_recNext * REC_BYTES, r);
  g_recNext = (uint16_t)((g_recNext + 1) % REC_SLOTS);
  if (g_recCount < REC_SLOTS) g_recCount++;
  if (g_recLive) recPrint(r);
}
static void recClear() {
  Rec r; memset(&r, 0xFF, sizeof(r));
  for (uint16_t s = 0; s < REC_SLOTS; s++) EEPROM.put(REC_BASE + (int)s * REC_BYTES, r);
  g_recNext = 0; g_recSeq = 1; g_recCount = 0;
}
// The dump (oldest first), as a paced report: skipped lines cost nothing (see report.h).
static void recDump() {
  say("LOG", "#RECORDS %u stored, %u slots, oldest first. Columns: seq,t_ms,event,detail (t_ms counts from the last BOOT line above it)", (unsigned)g_recCount, (unsigned)REC_SLOTS);
  for (uint16_t i = 0; i < g_recCount; i++) {
    Rec r;
    uint16_t slot = (uint16_t)((g_recNext + REC_SLOTS - g_recCount + i) % REC_SLOTS);
    if (!recRead(slot, &r)) continue;
    if (g_repActive) { if (g_repOverflow) return; if (g_repLine < g_repSkip) { g_repLine++; continue; } }   // printed by an earlier pass: no formatting work
    recPrint(r);
  }
  say("LOG", "#END-RECORDS");
}
