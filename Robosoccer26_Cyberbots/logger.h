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
  NAV_IDLE, NAV_MANUAL, NAV_SEEK, NAV_APPROACH, NAV_SEARCH, NAV_SEARCH_PAUSE, NAV_WAIT, NAV_TEST, NAV_SAFE_STOP, NAV_SHOOT
};
static const char NAV_NAME[][13] PROGMEM = {"IDLE", "MANUAL", "SEEK", "APPROACH", "SEARCH", "SEARCH_PAUSE", "WAIT", "TEST", "SAFE_STOP", "SHOOT"};
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
  uint8_t flags = (uint8_t)((g_ball.held ? 1 : 0) | (g_ball.ready ? 2 : 0) | (g_mod[0].active ? 4 : 0) | (g_mod[1].active ? 8 : 0));
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
