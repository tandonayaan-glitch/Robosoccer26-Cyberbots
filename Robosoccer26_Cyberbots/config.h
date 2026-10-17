// config.h - central, versioned, validated configuration.
// GPIO pins are NOT part of this structure (see pins.h); configuration can never move a pin.
#pragma once
#include <stddef.h>
#include "pins.h"
#include "util.h"

#define CFG_MAGIC 0x52534342u   // "RSCB"
#define CFG_VERSION 5

// Per-IR-channel calibration record (CD74HC4067 channel == array index).
struct ChCfg {
  uint8_t enabled;      // scanned at all?
  uint8_t activeLow;    // 1: receiver output is LOW while a modulated signal is present (TSOP17xx family)
  uint8_t trusted;      // operator allows this channel in navigation
  uint8_t bearingCal;   // 1 once the bearing has been measured/confirmed by the operator
  float bearingDeg;     // horizontal bearing, 0 = robot forward, + = clockwise (to the right)
  float tiltDownDeg;    // mounting tilt below horizontal (metadata only, used for documentation)
  uint8_t verified;     // operator-run controlled ball test passed (persisted operator record)
  uint8_t verifyFails;  // controlled ball tests that failed (0..5)
  uint8_t pad[2];
};

struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  // ---- drivetrain ----
  uint8_t reverse[WHEEL_COUNT];  // software reversal per wheel (FL,FR,BL,BR)
  uint8_t rollerO;               // 0 = "X" roller layout (default), 1 = "O" layout (flips strafe sign)
  uint8_t motorVerified;         // bitmask: wheel direction confirmed by operator test
  uint16_t pwmFreq;              // Hz (applied at boot)
  uint8_t maxManual;             // PWM counts 0..255
  uint8_t maxAuto;
  uint8_t minPwm;                // smallest non-zero PWM that reliably starts a wheel (to be calibrated)
  uint8_t testPwm;
  uint16_t accelPwmPerS;
  uint16_t decelPwmPerS;
  uint16_t reverseDeadMs;        // pause at zero when a wheel changes direction
  uint16_t manualHoldMs;         // manual motion auto-stops after this long without a repeated key

  // ---- IR acquisition ----
  uint16_t irWindowUs;           // observation window per channel
  uint8_t irMinEdges;            // active-level pulses needed in one window to call it a detection
  uint8_t irNoisyPerMs;          // more than this many pulses per ms => interference, not a ball
  uint8_t irSettleUs;            // mux settle time before a window opens
  uint8_t irHistMinOn;           // detections in the last 8 windows to switch a channel "active"
  uint8_t irFullHits;            // hits (of the last 8 windows) at which a channel counts with full weight
  uint8_t acqCycles;             // consecutive scan cycles above acqConf needed to acquire the ball
  uint16_t irPeriodMinUs;        // pulse-period gate (0/0 = OFF): accept only mean pulse periods in [min,max] us
  uint16_t irPeriodMaxUs;        // set from a measured ball with ":irgate auto"; rejects unrelated modulated sources
  uint32_t irStuckMs;            // output held active this long => suspect
  uint32_t irStuckFailMs;        // ... this long => confirmed fault
  float fovHalfDeg;              // assumed half acceptance angle per receiver (coverage maths; ASSUMPTION)
  float maxGapDeg;               // widest tolerated uncovered gap before DEGRADED
  uint8_t minTrusted;            // fewer trusted channels than this => ball seeking unavailable

  // ---- navigation ----
  float acqConf;
  uint16_t lostHoldMs;
  float centerDeg;
  float approachSpeed;           // 0..1 of maxAuto
  float attackSpeed;
  float slowSpeed;
  float aggressiveConf;
  float searchSpin;
  uint32_t searchTimeoutMs;
  uint8_t searchAttempts;
  uint16_t searchPauseMs;
  float degradedScale;
  float turnKp;                  // fallback turn-toward-ball gain (per degree)
  float fallbackAlignDeg;

  // ---- heading / IMU ----
  uint8_t hdgEnable;
  float hdgKp;                   // rotation command (0..1) per degree of heading error
  float hdgMaxRot;
  float hdgDeadbandDeg;
  int8_t imuYawSign;             // -1: gyro Z (CCW+) -> clockwise-positive yaw (chip flat, Z up)
  uint16_t imuStaleMs;
  uint16_t imuCalSamples;
  float imuStillDps;
  uint8_t imuAddrPref;           // 0 = auto (0x68 then 0x69), else 0x68 / 0x69

  // ---- logging ----
  uint16_t snapshotMs;
  uint16_t detailMs;             // 0 = detailed per-channel rows off
  uint32_t fileMaxBytes;
  uint8_t maxFiles;
  uint32_t fsReserveBytes;
  uint16_t flushMs;
  uint16_t frPeriodMs;
  uint16_t frPostMs;
  uint16_t streamMs;             // 'I' live IR view period

  // ---- recovery ----
  uint16_t recoverVerifyMs;

  ChCfg ch[MUX_CHANNELS];
  uint32_t crc;                  // CRC32 of every byte before this field
};

// Settable scalar parameters (Serial ":set"). Range-checked on set and on load.
enum ParamType : uint8_t { PT_U8, PT_U16, PT_U32, PT_F32, PT_I8 };
struct ParamDef { const char* name; ParamType type; size_t off; float lo; float hi; };

#define PD(n, t, f, lo, hi) {n, t, offsetof(Config, f), (float)(lo), (float)(hi)}
static const ParamDef PARAMS[] = {
  PD("pwmfreq", PT_U16, pwmFreq, 1000, 20000),
  PD("maxmanual", PT_U8, maxManual, 20, 255),
  PD("maxauto", PT_U8, maxAuto, 20, 255),
  PD("minpwm", PT_U8, minPwm, 0, 120),
  PD("testpwm", PT_U8, testPwm, 30, 255),
  PD("accel", PT_U16, accelPwmPerS, 50, 5000),
  PD("decel", PT_U16, decelPwmPerS, 50, 10000),
  PD("revdead", PT_U16, reverseDeadMs, 0, 500),
  PD("manualhold", PT_U16, manualHoldMs, 100, 5000),
  PD("irwindow", PT_U16, irWindowUs, 1000, 20000),
  PD("irminedges", PT_U8, irMinEdges, 1, 10),
  PD("irnoisy", PT_U8, irNoisyPerMs, 2, 50),
  PD("irsettle", PT_U8, irSettleUs, 2, 100),
  PD("irhistmin", PT_U8, irHistMinOn, 1, 8),
  PD("irfullhits", PT_U8, irFullHits, 2, 8),
  PD("acqcycles", PT_U8, acqCycles, 1, 6),
  PD("irperlo", PT_U16, irPeriodMinUs, 0, 20000),
  PD("irperhi", PT_U16, irPeriodMaxUs, 0, 20000),
  PD("irstuck", PT_U32, irStuckMs, 500, 30000),
  PD("irstuckfail", PT_U32, irStuckFailMs, 2000, 120000),
  PD("fov", PT_F32, fovHalfDeg, 10, 90),
  PD("maxgap", PT_F32, maxGapDeg, 30, 180),
  PD("mintrusted", PT_U8, minTrusted, 1, 12),
  PD("acqconf", PT_F32, acqConf, 0.2, 0.95),
  PD("losthold", PT_U16, lostHoldMs, 100, 3000),
  PD("center", PT_F32, centerDeg, 5, 60),
  PD("approach", PT_F32, approachSpeed, 0.15, 1.0),
  PD("attack", PT_F32, attackSpeed, 0.15, 1.0),
  PD("slow", PT_F32, slowSpeed, 0.1, 0.6),
  PD("aggrconf", PT_F32, aggressiveConf, 0.5, 0.99),
  PD("searchspin", PT_F32, searchSpin, 0.1, 0.6),
  PD("searchtime", PT_U32, searchTimeoutMs, 2000, 60000),
  PD("searchtries", PT_U8, searchAttempts, 1, 10),
  PD("searchpause", PT_U16, searchPauseMs, 500, 10000),
  PD("degscale", PT_F32, degradedScale, 0.2, 1.0),
  PD("turnkp", PT_F32, turnKp, 0.002, 0.05),
  PD("alignfallback", PT_F32, fallbackAlignDeg, 15, 90),
  PD("hdgenable", PT_U8, hdgEnable, 0, 1),
  PD("hdgkp", PT_F32, hdgKp, 0.0, 0.1),
  PD("hdgmax", PT_F32, hdgMaxRot, 0.05, 0.5),
  PD("hdgdead", PT_F32, hdgDeadbandDeg, 0, 10),
  PD("yawsign", PT_I8, imuYawSign, -1, 1),
  PD("imustale", PT_U16, imuStaleMs, 20, 1000),
  PD("imucal", PT_U16, imuCalSamples, 100, 2000),
  PD("imustill", PT_F32, imuStillDps, 0.5, 20),
  PD("imuaddr", PT_U8, imuAddrPref, 0, 0x69),
  PD("snapms", PT_U16, snapshotMs, 50, 5000),
  PD("detailms", PT_U16, detailMs, 0, 5000),
  PD("filemax", PT_U32, fileMaxBytes, 4096, 262144),
  PD("maxfiles", PT_U8, maxFiles, 2, 16),
  PD("fsreserve", PT_U32, fsReserveBytes, 16384, 1048576),
  PD("flushms", PT_U16, flushMs, 500, 10000),
  PD("frms", PT_U16, frPeriodMs, 20, 1000),
  PD("frpost", PT_U16, frPostMs, 0, 10000),
  PD("streamms", PT_U16, streamMs, 100, 5000),
  PD("recoverms", PT_U16, recoverVerifyMs, 500, 30000),
};
static const size_t PARAM_COUNT = sizeof(PARAMS) / sizeof(PARAMS[0]);

static float paramGet(const Config& c, const ParamDef& p) {
  const uint8_t* b = (const uint8_t*)&c + p.off;
  switch (p.type) {
    case PT_U8: return (float)*b;
    case PT_I8: return (float)*(const int8_t*)b;
    case PT_U16: { uint16_t v; memcpy(&v, b, 2); return (float)v; }
    case PT_U32: { uint32_t v; memcpy(&v, b, 4); return (float)v; }
    case PT_F32: { float v; memcpy(&v, b, 4); return v; }
  }
  return 0;
}
static void paramSet(Config& c, const ParamDef& p, float val) {
  uint8_t* b = (uint8_t*)&c + p.off;
  switch (p.type) {
    case PT_U8: *b = (uint8_t)(val + 0.5f); break;
    case PT_I8: { int8_t v = (int8_t)(val < 0 ? val - 0.5f : val + 0.5f); memcpy(b, &v, 1); break; }
    case PT_U16: { uint16_t v = (uint16_t)(val + 0.5f); memcpy(b, &v, 2); break; }
    case PT_U32: { uint32_t v = (uint32_t)(val + 0.5f); memcpy(b, &v, 4); break; }
    case PT_F32: memcpy(b, &val, 4); break;
  }
}
static const ParamDef* paramFind(const char* name) {
  for (size_t i = 0; i < PARAM_COUNT; i++)
    if (strcmp(PARAMS[i].name, name) == 0) return &PARAMS[i];
  return nullptr;
}
static bool paramInRange(const ParamDef& p, float v) {
  return isfinite(v) && v >= p.lo && v <= p.hi;
}

static void cfgDefaults(Config& c) {
  memset(&c, 0, sizeof(c));   // zero padding too, so the CRC is reproducible
  c.magic = CFG_MAGIC;
  c.version = CFG_VERSION;
  c.size = (uint16_t)sizeof(Config);
  c.rollerO = 0;
  c.pwmFreq = 5000;
  c.maxManual = 150;
  c.maxAuto = 170;
  c.minPwm = 40;
  c.testPwm = 100;
  c.accelPwmPerS = 500;
  c.decelPwmPerS = 1200;
  c.reverseDeadMs = 60;
  c.manualHoldMs = 600;
  c.irWindowUs = 3000;
  c.irMinEdges = 2;
  c.irNoisyPerMs = 3;     // ball ~1.2 pulses/ms; motor-PWM/interference is several per ms
  c.irSettleUs = 10;
  c.irHistMinOn = 2;
  c.irFullHits = 8;
  c.acqCycles = 2;
  c.irPeriodMinUs = 0;     // gate OFF until the real ball has been measured
  c.irPeriodMaxUs = 0;
  c.irStuckMs = 2000;
  c.irStuckFailMs = 15000;
  c.fovHalfDeg = 35.0f;
  c.maxGapDeg = 100.0f;
  c.minTrusted = 4;
  c.acqConf = 0.45f;
  c.lostHoldMs = 500;
  c.centerDeg = 20.0f;
  c.approachSpeed = 0.7f;
  c.attackSpeed = 1.0f;
  c.slowSpeed = 0.3f;
  c.aggressiveConf = 0.75f;
  c.searchSpin = 0.25f;
  c.searchTimeoutMs = 8000;
  c.searchAttempts = 3;
  c.searchPauseMs = 2000;
  c.degradedScale = 0.6f;
  c.turnKp = 0.015f;
  c.fallbackAlignDeg = 45.0f;
  c.hdgEnable = 1;
  c.hdgKp = 0.02f;
  c.hdgMaxRot = 0.25f;
  c.hdgDeadbandDeg = 2.0f;
  c.imuYawSign = -1;
  c.imuStaleMs = 150;
  c.imuCalSamples = 400;
  c.imuStillDps = 3.0f;
  c.imuAddrPref = 0;
  c.snapshotMs = 250;
  c.detailMs = 0;
  c.fileMaxBytes = 65536;
  c.maxFiles = 4;
  c.fsReserveBytes = 65536;
  c.flushMs = 2000;
  c.frPeriodMs = 100;
  c.frPostMs = 2000;
  c.streamMs = 500;
  c.recoverVerifyMs = 3000;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) {
    c.ch[i].enabled = (i < IR_PLANNED_CHANNELS) ? 1 : 0;   // C12..C15 stay off until hardware exists
    c.ch[i].activeLow = 1;
    c.ch[i].trusted = 1;
    c.ch[i].bearingCal = 0;                                // placeholder bearings are NOT measured
    c.ch[i].bearingDeg = (float)((i % 12) * 30);     // placeholder; spare channels reuse 0..90
    c.ch[i].tiltDownDeg = 22.0f;                           // proposed starting experiment (20-25 deg)
  }
}

static void cfgSeal(Config& c) {
  c.crc = crc32Update(0, (const uint8_t*)&c, offsetof(Config, crc));
}
static bool cfgCrcOk(const Config& c) {
  return c.crc == crc32Update(0, (const uint8_t*)&c, offsetof(Config, crc));
}

static void repNote(char* rep, size_t n, const char* what) {
  size_t l = strlen(rep);
  if (l + strlen(what) + 2 >= n) return;
  if (l) { rep[l++] = ','; rep[l] = 0; }
  strcat(rep, what);
}

// Replace every out-of-range / non-finite field with its safe default. Returns the number of
// fields replaced; `rep` receives a comma-separated list of the names.
static int cfgValidate(Config& c, char* rep, size_t repN) {
  static Config d;                         // loop-task only; keeps 0.5 KB off the stack
  cfgDefaults(d);
  int bad = 0;
  if (repN) rep[0] = 0;
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    const ParamDef& p = PARAMS[i];
    if (!paramInRange(p, paramGet(c, p))) { paramSet(c, p, paramGet(d, p)); bad++; repNote(rep, repN, p.name); }
  }
  for (int i = 0; i < WHEEL_COUNT; i++)
    if (c.reverse[i] > 1) { c.reverse[i] = 0; bad++; repNote(rep, repN, "reverse"); }
  if (c.rollerO > 1) { c.rollerO = 0; bad++; repNote(rep, repN, "roller"); }
  if (c.motorVerified & 0xF0) { c.motorVerified &= 0x0F; bad++; repNote(rep, repN, "motorverified"); }
  if (c.imuAddrPref != 0 && c.imuAddrPref != 0x68 && c.imuAddrPref != 0x69) { c.imuAddrPref = 0; bad++; repNote(rep, repN, "imuaddr"); }
  if (c.imuYawSign != -1 && c.imuYawSign != 1) { c.imuYawSign = -1; bad++; repNote(rep, repN, "yawsign"); }
  // Cross-field constraints: a start threshold above the speed limit would make motion impossible
  // or make "slow" faster than "fast".
  if (c.minPwm >= c.maxManual || c.minPwm >= c.maxAuto) {
    c.minPwm = d.minPwm; c.maxManual = d.maxManual; c.maxAuto = d.maxAuto; bad++; repNote(rep, repN, "minpwm/max");
  }
  if (c.testPwm < c.minPwm || c.testPwm > c.maxManual) { c.testPwm = d.testPwm; bad++; repNote(rep, repN, "testpwm"); }
  if (c.slowSpeed > c.attackSpeed) { c.slowSpeed = d.slowSpeed; c.attackSpeed = d.attackSpeed; bad++; repNote(rep, repN, "slow/attack"); }
  bool gateOff = (c.irPeriodMinUs == 0 && c.irPeriodMaxUs == 0);
  if (!gateOff && !(c.irPeriodMinUs >= 100 && c.irPeriodMaxUs > c.irPeriodMinUs)) { c.irPeriodMinUs = c.irPeriodMaxUs = 0; bad++; repNote(rep, repN, "irperiodgate"); }
  if (c.irStuckFailMs <= c.irStuckMs) { c.irStuckMs = d.irStuckMs; c.irStuckFailMs = d.irStuckFailMs; bad++; repNote(rep, repN, "irstuck"); }
  for (int i = 0; i < MUX_CHANNELS; i++) {
    ChCfg& h = c.ch[i];
    ChCfg& dh = d.ch[i];
    bool badCh = false;
    if (h.enabled > 1) { h.enabled = dh.enabled; badCh = true; }
    if (h.activeLow > 1) { h.activeLow = dh.activeLow; badCh = true; }
    if (h.trusted > 1) { h.trusted = dh.trusted; badCh = true; }
    if (h.bearingCal > 1) { h.bearingCal = 0; badCh = true; }
    if (h.verified > 1) { h.verified = 0; badCh = true; }
    if (h.verifyFails > 5) { h.verifyFails = 0; badCh = true; }
    if (!isfinite(h.bearingDeg) || h.bearingDeg < 0.0f || h.bearingDeg >= 360.0f) {
      h.bearingDeg = dh.bearingDeg; h.bearingCal = 0; badCh = true;
    }
    if (!isfinite(h.tiltDownDeg) || h.tiltDownDeg < 0.0f || h.tiltDownDeg > 90.0f) { h.tiltDownDeg = dh.tiltDownDeg; badCh = true; }
    if (badCh) { bad++; repNote(rep, repN, "channel"); }
  }
  return bad;
}

// The live configuration. Modules take `const Config&` where practical; this is the one instance.
static Config g_cfg;
