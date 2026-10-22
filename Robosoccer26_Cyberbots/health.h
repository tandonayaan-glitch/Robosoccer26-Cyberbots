// health.h - fault registry, operating condition (NORMAL / DEGRADED / SAFE_STOP) and statistics.
#pragma once
#include "util.h"

enum Severity : uint8_t { SEV_INFO, SEV_WARN, SEV_DEGRADED, SEV_CRITICAL };
enum Cond : uint8_t { COND_NORMAL, COND_DEGRADED, COND_SAFE_STOP };

enum FaultCode : uint8_t {
  F_RING_NONE, F_RING_STALL, F_RING_CHANNEL, F_RING_GAP,
  F_IMU_MISSING, F_IMU_TIMEOUT, F_IMU_INVALID,
  F_CFG_REPLACED,
  F_MOTOR_INIT, F_LOOP_STALL,
  F_COUNT
};

// Fixed-size char arrays so the whole table lives in flash (PROGMEM) without one declaration per string.
struct FaultMeta {
  char code[14];         // stable identifier used in the output
  char comp[14];         // component / capability
  uint8_t sev;
  uint8_t latched;       // 1: stays active until the operator runs ":recover"
  char action[88];       // what the firmware does about it
  char certainty[96];    // how sure the evidence is (confirmed vs suspected)
};

static const FaultMeta FMETA[F_COUNT] PROGMEM = {
  {"RING_NONE",       "Ball ring",   SEV_CRITICAL, 0, "ball seeking disabled, motors stopped", "no enabled channel is usable (all off or saturated); cause (wiring/supply/part) unproven"},
  {"RING_STALL",      "Ball ring",   SEV_CRITICAL, 0, "ball seeking disabled, motors stopped", "no channel scan completed for 600 ms: the control loop or the scan is stuck"},
  {"RING_CHANNEL",    "Ball ring",   SEV_DEGRADED, 0, "listed channels excluded from the ball bearing", "symptom measured (reading held at the top of the ADC range); cause unproven"},
  {"RING_GAP",        "Ball ring",   SEV_DEGRADED, 0, "reduced speed; the robot rotates to find a ball in the uncovered arc", "ESTIMATE from configured bearings and an assumed field of view, not a measurement"},
  {"IMU_MISSING",     "MPU6050",     SEV_DEGRADED, 0, "heading hold disabled, turn-toward-ball fallback", "no I2C ACK; wiring vs dead chip not distinguishable"},
  {"IMU_TIMEOUT",     "MPU6050",     SEV_DEGRADED, 0, "heading hold disabled, retrying init", "bus errors observed; cause unproven"},
  {"IMU_INVALID",     "MPU6050",     SEV_DEGRADED, 0, "invalid samples rejected, heading hold disabled", "frames failed plausibility checks; cause unproven"},
  {"CFG_REPLACED",    "Config",      SEV_WARN,     0, "safe defaults used for invalid fields", "certain"},
  {"MOTOR_INIT",      "Drivetrain",  SEV_CRITICAL, 1, "motion disabled until :recover", "certain: a motor enable pin is not a PWM pin"},
  {"LOOP_STALL",      "Control loop", SEV_CRITICAL, 1, "motors stopped; run :recover after the cause is removed", "measured loop gap"},
};
static inline PGM_P faultCode(uint8_t i) { return (PGM_P)FMETA[i].code; }
static inline uint8_t faultSev(uint8_t i) { return pgm_read_byte(&FMETA[i].sev); }
static inline bool faultLatched(uint8_t i) { return pgm_read_byte(&FMETA[i].latched) != 0; }

struct FaultState {
  bool active;
  uint32_t count;
  uint32_t firstMs, lastMs, lastPrintMs;
  char evidence[48];
};
static FaultState g_fault[F_COUNT];
static Cond g_cond = COND_NORMAL;

struct Stats {
  uint32_t ballLossEvents, reacquisitions, reacquireTotalMs, searchTotalMs, shots;
  uint32_t imuTimeouts, imuRecoveries, degradedTransitions, safetyStops;
};
static Stats g_stats;

// Event records kept in the EEPROM (logger.h): what happened, in order, survives a power cycle.
enum RecType : uint8_t { REC_BOOT = 1, REC_FAULT, REC_RECOVER, REC_NAV, REC_BALL_LOST, REC_SHOT, REC_SAFE_STOP, REC_CFG_SAVED, REC_NOTE };
static void recAdd(uint8_t type, uint16_t arg);

// Implemented later (logger.h / nav.h).
static void logEvent(const char* type, const char* note);
static void onCondChange(Cond from, Cond to);

static PGM_P condName(Cond c) {
  return c == COND_NORMAL ? PSTR("NORMAL") : (c == COND_DEGRADED ? PSTR("DEGRADED") : PSTR("SAFE_STOP"));
}
static PGM_P sevName(uint8_t s) {
  return s == SEV_INFO ? PSTR("INFO") : (s == SEV_WARN ? PSTR("WARN") : (s == SEV_DEGRADED ? PSTR("DEGRADED") : PSTR("CRITICAL")));
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
    if (faultSev((uint8_t)i) == SEV_CRITICAL) { n = COND_SAFE_STOP; break; }
    if (faultSev((uint8_t)i) == SEV_DEGRADED) n = COND_DEGRADED;
  }
  if (n != g_cond) {
    Cond old = g_cond;
    g_cond = n;
    if (n == COND_DEGRADED && old == COND_NORMAL) g_stats.degradedTransitions++;
    onCondChange(old, n);
  }
}

// fmt is in flash; the evidence text is kept (48 bytes) for the repeat reminders and the health report.
static void faultRaiseV(FaultCode c, PGM_P fmt, va_list ap) {
  FaultState& f = g_fault[c];
  uint32_t now = millis();
  rsFormat(f.evidence, sizeof(f.evidence), fmt, RS_HAS_FLASH != 0, ap);
  f.lastMs = now;
  if (!f.active) {
    f.active = true;
    f.count++;
    f.firstMs = now;
    f.lastPrintMs = now;
    char note[40];
    snprintf(note, sizeof(note), "%S;%s", faultCode((uint8_t)c), f.evidence);
    logEvent("F", note);
    recAdd(REC_FAULT, (uint16_t)c);
    if (faultSev((uint8_t)c) == SEV_CRITICAL) g_stats.safetyStops++;
    condEvaluate();
    // severity, component, code, evidence, certainty, action, current mode, recovery
    PGM_P tag = faultSev((uint8_t)c) == SEV_CRITICAL ? PSTR("FAULT") : PSTR("WARN");
    sayP(tag, PSTR("%S sev=%S comp=%S evidence=\"%s\""), faultCode((uint8_t)c), sevName(faultSev((uint8_t)c)), (PGM_P)FMETA[c].comp, f.evidence);
    sayP(tag, PSTR("  certainty: %S | action: %S | mode=%S recovery=%S"), (PGM_P)FMETA[c].certainty, (PGM_P)FMETA[c].action, condName(g_cond),
         faultLatched((uint8_t)c) ? PSTR("needs :recover") : PSTR("automatic when verified"));
  } else if (elapsedMs(now, f.lastPrintMs) >= 10000) {
    f.lastPrintMs = now;
    say("WARN", "%S still active: %s", faultCode((uint8_t)c), f.evidence);
  }
}
static void faultRaiseP(FaultCode c, PGM_P fmt, ...) {
  va_list ap; va_start(ap, fmt);
  faultRaiseV(c, fmt, ap);
  va_end(ap);
}
#define faultRaise(c, fmt, ...) faultRaiseP((c), PSTR(fmt), ##__VA_ARGS__)

static void faultClear(FaultCode c, const char* how) {
  FaultState& f = g_fault[c];
  if (!f.active) return;
  f.active = false;
  say("INFO", "%S cleared (%s)", faultCode((uint8_t)c), how);
  char note[40];
  snprintf(note, sizeof(note), "%S;%s", faultCode((uint8_t)c), how);
  logEvent("R", note);
  recAdd(REC_RECOVER, (uint16_t)c);
  condEvaluate();
}

// Raise while `bad`; clear only after the condition has been absent for `verifyMs`
// (a transient error is not declared recovered until it has stayed away).
static uint32_t g_okSince[F_COUNT];
static bool g_okTiming[F_COUNT];
static void faultUpdateCore(FaultCode c, bool bad, uint32_t verifyMs, PGM_P fmt, const char* ramEvidence) {
  uint32_t now = millis();
  if (bad) {
    g_okTiming[c] = false;
    if (ramEvidence) faultRaiseP(c, PSTR("%s"), ramEvidence); else faultRaiseP(c, fmt);
  } else if (g_fault[c].active && !faultLatched((uint8_t)c)) {
    if (!g_okTiming[c]) { g_okTiming[c] = true; g_okSince[c] = now; }
    else if (elapsedMs(now, g_okSince[c]) >= verifyMs) { g_okTiming[c] = false; faultClear(c, "condition absent for the verification period"); }
  }
}
static void faultUpdate(FaultCode c, bool bad, uint32_t verifyMs, const char* ramEvidence) { faultUpdateCore(c, bad, verifyMs, nullptr, ramEvidence); }
#define faultUpdateP(c, bad, ms, lit) faultUpdateCore((c), (bad), (ms), PSTR(lit), nullptr)
