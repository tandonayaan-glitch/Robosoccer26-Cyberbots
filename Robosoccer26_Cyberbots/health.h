// health.h - fault registry, operating condition (NORMAL / DEGRADED / SAFE_STOP) and statistics.
#pragma once
#include "util.h"

enum Severity : uint8_t { SEV_INFO, SEV_WARN, SEV_DEGRADED, SEV_CRITICAL };
enum Cond : uint8_t { COND_NORMAL, COND_DEGRADED, COND_SAFE_STOP };

enum FaultCode : uint8_t {
  F_IR_FRONT, F_IR_ALIGN,
  F_IMU_MISSING, F_IMU_TIMEOUT, F_IMU_INVALID,
  F_FS_UNAVAILABLE, F_FS_WRITE,
  F_CFG_REPLACED, F_BROWNOUT_RESET, F_ABNORMAL_RESET,
  F_MOTOR_INIT, F_LOOP_STALL, F_LOG_DROPS,
  F_COUNT
};

struct FaultMeta {
  const char* code;      // stable identifier used in logs
  const char* comp;      // component / capability
  Severity sev;
  bool latched;          // true: stays active until the operator runs ":recover"
  const char* action;    // what the firmware does about it
  const char* certainty; // how sure the evidence is (confirmed vs suspected)
};

static const FaultMeta FMETA[F_COUNT] = {
  {"IR_FRONT",        "IR sensor 0",     SEV_CRITICAL, false, "ball seeking disabled, motors stopped", "symptom measured (reading held at an ADC end stop); cause (wiring/supply/part) unproven"},
  {"IR_ALIGN",        "IR sensor 1",     SEV_DEGRADED, false, "sensor 1 excluded: no sideways guidance, shooting position uses sensor 0 only", "symptom measured (reading held at an ADC end stop); cause unproven"},
  {"IMU_MISSING",     "MPU6050",         SEV_DEGRADED, false, "heading hold disabled, turn-toward-ball fallback", "no I2C ACK; wiring vs dead chip not distinguishable"},
  {"IMU_TIMEOUT",     "MPU6050",         SEV_DEGRADED, false, "heading hold disabled, retrying init", "bus errors observed; cause unproven"},
  {"IMU_INVALID",     "MPU6050",         SEV_DEGRADED, false, "invalid samples rejected, heading hold disabled", "frames failed plausibility checks; cause unproven"},
  {"FS_UNAVAILABLE",  "Storage",         SEV_DEGRADED, false, "persistent logging off; Serial + RAM recorder continue", "mount failed; may be a missing LittleFS partition"},
  {"FS_WRITE",        "Storage",         SEV_DEGRADED, false, "persistent logging paused/off; control unaffected", "write or read-back failed"},
  {"CFG_REPLACED",    "Configuration",   SEV_WARN,     false, "safe defaults used for invalid fields", "certain"},
  {"BROWNOUT_RESET",  "Power",           SEV_WARN,     false, "autonomous stays disabled until 'a'; check supply", "reset-reason register: certain it was a brownout reset"},
  {"ABNORMAL_RESET",  "Firmware/CPU",    SEV_WARN,     false, "autonomous stays disabled until 'a'; see log", "reset-reason register"},
  {"MOTOR_INIT",      "Drivetrain",      SEV_CRITICAL, true,  "motion disabled until :recover", "certain: LEDC attach returned failure"},
  {"LOOP_STALL",      "Control loop",    SEV_CRITICAL, true,  "motors stopped; run :recover after cause is removed", "measured loop gap"},
  {"LOG_DROPS",       "Logging",         SEV_WARN,     false, "records dropped; diagnostics reduced", "certain: counter"},
};

struct FaultState {
  bool active;
  uint32_t count;
  uint32_t firstMs, lastMs, lastPrintMs;
  char evidence[72];
};
static FaultState g_fault[F_COUNT];
static Cond g_cond = COND_NORMAL;

struct Stats {
  uint32_t ballLossEvents, reacquisitions, reacquireTotalMs, searchTotalMs, shots;
  uint32_t imuTimeouts, imuRecoveries, degradedTransitions, safetyStops;
  uint32_t logDropped, storageFailures;
};
static Stats g_stats;

// Implemented later (logger.h / main): record the fault in CSV + flight recorder.
static void logEvent(const char* type, const char* note);
static void frTrigger(const char* why);
static void onCondChange(Cond from, Cond to);

static const char* condName(Cond c) {
  return c == COND_NORMAL ? "NORMAL" : (c == COND_DEGRADED ? "DEGRADED" : "SAFE_STOP");
}
static const char* sevName(Severity s) {
  return s == SEV_INFO ? "INFO" : (s == SEV_WARN ? "WARN" : (s == SEV_DEGRADED ? "DEGRADED" : "CRITICAL"));
}

static bool faultActive(FaultCode c) { return g_fault[c].active; }
static uint16_t faultMask() {
  uint16_t m = 0;
  for (int i = 0; i < F_COUNT; i++) if (g_fault[i].active) m |= (uint16_t)(1u << i);
  return m;
}

static void condEvaluate() {
  Cond n = COND_NORMAL;
  for (int i = 0; i < F_COUNT; i++) {
    if (!g_fault[i].active) continue;
    if (FMETA[i].sev == SEV_CRITICAL) { n = COND_SAFE_STOP; break; }
    if (FMETA[i].sev == SEV_DEGRADED) n = COND_DEGRADED;
  }
  if (n != g_cond) {
    Cond old = g_cond;
    g_cond = n;
    if (n == COND_DEGRADED && old == COND_NORMAL) g_stats.degradedTransitions++;
    onCondChange(old, n);
  }
}

static void faultRaise(FaultCode c, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
static void faultRaise(FaultCode c, const char* fmt, ...) {
  FaultState& f = g_fault[c];
  uint32_t now = millis();
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(f.evidence, sizeof(f.evidence), fmt, ap);
  va_end(ap);
  f.lastMs = now;
  if (!f.active) {
    f.active = true;
    f.count++;
    f.firstMs = now;
    f.lastPrintMs = now;
    char note[96];
    snprintf(note, sizeof(note), "%s;%s", FMETA[c].code, f.evidence);
    logEvent("F", note);
    if (FMETA[c].sev >= SEV_DEGRADED) frTrigger(FMETA[c].code);
    if (FMETA[c].sev == SEV_CRITICAL) g_stats.safetyStops++;
    condEvaluate();
    // timestamp (prefix), severity, component, code, evidence, certainty, action, current mode, recovery
    say(FMETA[c].sev == SEV_CRITICAL ? "FAULT" : "WARN",
        "%s sev=%s comp=%s evidence=\"%s\" certainty=\"%s\" action=\"%s\" mode=%s recovery=%s",
        FMETA[c].code, sevName(FMETA[c].sev), FMETA[c].comp, f.evidence, FMETA[c].certainty, FMETA[c].action,
        condName(g_cond), FMETA[c].latched ? "needs :recover" : "automatic when verified");
  } else if (elapsedMs(now, f.lastPrintMs) >= 10000) {
    f.lastPrintMs = now;
    say("WARN", "%s still active: %s", FMETA[c].code, f.evidence);
  }
}

static void faultClear(FaultCode c, const char* how) {
  FaultState& f = g_fault[c];
  if (!f.active) return;
  f.active = false;
  say("INFO", "%s cleared (%s)", FMETA[c].code, how);
  char note[64];
  snprintf(note, sizeof(note), "%s;%s", FMETA[c].code, how);
  logEvent("R", note);
  condEvaluate();
}

// Raise while `bad`; clear only after the condition has been absent for `verifyMs`
// (a transient error is not declared recovered until it has stayed away).
static uint32_t g_okSince[F_COUNT];
static bool g_okTiming[F_COUNT];
static void faultUpdate(FaultCode c, bool bad, uint32_t verifyMs, const char* evidence) {
  uint32_t now = millis();
  if (bad) {
    g_okTiming[c] = false;
    faultRaise(c, "%s", evidence);
  } else if (g_fault[c].active && !FMETA[c].latched) {
    if (!g_okTiming[c]) { g_okTiming[c] = true; g_okSince[c] = now; }
    else if (elapsedMs(now, g_okSince[c]) >= verifyMs) { g_okTiming[c] = false; faultClear(c, "condition absent for the verification period"); }
  }
}
