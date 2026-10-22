// config.h - central, versioned, validated configuration.
// GPIO pins are NOT part of this structure (see pins.h); configuration can never move a pin.
#pragma once
#include <stddef.h>
#include "pins.h"
#include "util.h"

#define CFG_MAGIC 0x52534342u   // "RSCB"
#define CFG_VERSION 17

struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  // ---- drivetrain ----
  uint8_t reverse[WHEEL_COUNT];  // software reversal per wheel (FL,FR,BL,BR)
  uint8_t rollerO;               // 0 = "X" roller layout (default), 1 = "O" layout (flips strafe sign)
  uint8_t motorVerified;         // bitmask: wheel direction confirmed by operator test
  uint8_t maxManual;             // PWM counts 0..255
  uint8_t maxAuto;
  uint8_t minPwm;                // smallest non-zero PWM that reliably starts a wheel (to be calibrated)
  uint8_t testPwm;
  uint16_t accelPwmPerS;
  uint16_t decelPwmPerS;
  uint16_t reverseDeadMs;        // pause at zero when a wheel changes direction
  uint16_t manualHoldMs;         // manual motion auto-stops after this long without a repeated key

  // ---- ball receivers: phototransistors, each on its own analog input (A0, A1 fitted; A2..A7 optional), 10-bit ADC ----
  uint16_t ringEnable;           // bit n = receiver on analog input An is fitted and scanned. A0 and A1 on by default; more once you fit them
  int16_t ringBearing[RING_CH_MAX]; // where each receiver looks, degrees: 0 = straight ahead, + clockwise (right), +-180 = behind. PLACEHOLDERS until you measure (':ring bearing')
  uint16_t ringNoise[RING_CH_MAX];  // per-channel noise floor in ADC counts (0..1023) (peak-to-peak with no ball emitting), set by ':ring cal noise'
  uint8_t ringCalDone;           // 1 = ':ring cal noise' has been run on this robot (autonomy is refused otherwise)
  uint16_t ringWinUs;            // each channel is sampled this long (microseconds) and the peak-to-peak swing is taken: must cover one ball burst period
  uint16_t ringSeen;             // a channel "sees" the ball at this peak-to-peak swing (10-bit ADC counts), or above its noise floor times ringnoisex, whichever is more
  uint8_t ringNoiseX10;          // noise-floor multiplier x10 (30 = 3.0x)
  uint16_t ringNear;             // swing at which the ball counts as "very close" (strength 1.0); ':ring cal near' sets it from the shooting position
  uint8_t ringDebounce;          // consecutive scans needed to change a channel's state
  uint8_t ringFovHalf;           // ASSUMED half field of view of one receiver, degrees: used only for the coverage-gap estimate
  uint8_t ringBlend;             // channels within this many degrees of the strongest one are averaged into the bearing
  uint16_t ringMaxGap;           // largest tolerated uncovered arc in degrees (estimate from the configured bearings), above it: DEGRADED. 360 = never (only two receivers are fitted)
  uint16_t ringRailMs;           // a channel held saturated this long is excluded and reported (shorted to a rail / sunlight saturation)
  uint16_t ringRailHi;           // "saturated" = lowest reading of the window at or above this ADC value (0..1023). The emitter node of a lit phototransistor tops out near 4.8 V = about 980 counts; 1000 flags a channel stuck at the 5 V rail
  // ---- IR obstacle modules (digital OUT): module 0 = front, ball in the shooting position; module 1 = undecided position, logged only ----
  uint8_t modEnable;             // bit0: module 0 in use (default), bit1: module 1 in use
  uint8_t modActiveLow;          // bit n set = module n output goes LOW when something is detected (typical for these boards; NOT assumed: check with ':mod' and set with ':mod level <n> low|high')
  uint8_t modDebounce;           // consecutive 5 ms samples needed to change a module's state
  uint8_t modNeedRing;           // 1 = the shooting position also needs the ring to see the ball near the front (a wall or a robot also trips the module)
  // ---- navigation ----
  uint16_t lostHoldMs;           // ball stays "valid" this long after the ring stopped seeing it
  float centerDeg;               // ball bearing within +- this many degrees of straight ahead counts as centred
  float turnDeg;                 // bearing beyond this: turn on the spot towards the ball before driving
  float turnKp;                  // rotation request per degree of bearing error
  float turnMax;                 // largest rotation request used to follow the ball
  float approachSpeed;           // 0..1 of maxAuto; forward speed while the ball is far
  float slowSpeed;               // forward speed close to the ball
  float alignSpeed;              // largest sideways speed used to centre the ball (strafe)
  float searchSpin;
  uint32_t searchTimeoutMs;
  uint8_t searchAttempts;
  uint16_t searchPauseMs;
  float degradedScale;

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

  // ---- goal heading registered by the gyro after power-on ----
  uint16_t goalRegisterMs;       // the robot's heading at this time after power-on is registered as "straight at the goal"
  uint16_t goalWidthMm;          // inner width of the goal (RoboCupJunior: 450 mm in some rule sets, 600 mm in others: check yours)
  uint16_t goalDistMm;           // expected distance from the shooting spot to the goal line (operator estimate)
  uint8_t ballDiaMm;             // ball diameter (golf ball ~43 mm)
  float aimSpeed;                // rotation speed while aiming, 0..1 of full
  uint16_t aimTimeoutMs;         // give up aiming after this long

  // ---- shot: aim at the registered goal heading, reverse a little, then drive forward at full speed (NO kicker is installed) ----
  uint8_t shootEnable;           // 0 = never shoot (default)
  uint8_t shootPwm;              // wheel PWM limit during the drive phase (the shot ignores the autonomous limit maxauto; 255 = full power)
  uint8_t shootParallel;         // 1 = turn toward the goal heading WHILE reversing (faster); 0 = aim first, then reverse
  uint16_t shootBackMs;          // reverse phase length
  float shootBackSpeed;          // reverse speed, 0..1 of full
  uint16_t shootDriveMs;         // full-speed forward phase length
  uint16_t shootHoldMs;          // the ball must stay in the shooting position this long before the shot starts
  uint16_t shootCooldownMs;      // minimum time between shots

  // ---- serial reporting ----
  uint16_t snapshotMs;           // period of the CSV rows after ':diag on'
  uint16_t streamMs;             // 'I' live ring view period

  // ---- recovery ----
  uint16_t recoverVerifyMs;

  uint32_t crc;                  // CRC32 of every byte before this field
};

// Settable scalar parameters (Serial ":set"). Range-checked on set and on load.
enum ParamType : uint8_t { PT_U8, PT_U16, PT_U32, PT_F32, PT_I8 };
struct ParamDef { char name[16]; uint8_t type; uint16_t off; float lo; float hi; };   // lives in flash (PROGMEM)

#define PD(n, t, f, lo, hi) {n, (uint8_t)(t), (uint16_t)offsetof(Config, f), (float)(lo), (float)(hi)}
static const ParamDef PARAMS[] PROGMEM = {
  PD("maxmanual", PT_U8, maxManual, 20, 255),
  PD("maxauto", PT_U8, maxAuto, 20, 255),
  PD("minpwm", PT_U8, minPwm, 0, 120),
  PD("testpwm", PT_U8, testPwm, 30, 255),
  PD("accel", PT_U16, accelPwmPerS, 50, 5000),
  PD("decel", PT_U16, decelPwmPerS, 50, 10000),
  PD("revdead", PT_U16, reverseDeadMs, 0, 500),
  PD("manualhold", PT_U16, manualHoldMs, 100, 5000),
  PD("ringenable", PT_U16, ringEnable, 0, 65535),
  PD("ringwinus", PT_U16, ringWinUs, 400, 3000),
  PD("ringseen", PT_U16, ringSeen, 3, 500),
  PD("ringnoisex", PT_U8, ringNoiseX10, 10, 100),
  PD("ringnear", PT_U16, ringNear, 20, 1023),
  PD("ringdebounce", PT_U8, ringDebounce, 1, 10),
  PD("ringfov", PT_U8, ringFovHalf, 10, 90),
  PD("ringblend", PT_U8, ringBlend, 10, 120),
  PD("ringmaxgap", PT_U16, ringMaxGap, 30, 360),
  PD("ringrailms", PT_U16, ringRailMs, 200, 20000),
  PD("ringrailhi", PT_U16, ringRailHi, 500, 1023),
  PD("modenable", PT_U8, modEnable, 0, 3),
  PD("modactivelow", PT_U8, modActiveLow, 0, 3),
  PD("moddebounce", PT_U8, modDebounce, 1, 20),
  PD("modneedring", PT_U8, modNeedRing, 0, 1),
  PD("losthold", PT_U16, lostHoldMs, 40, 3000),
  PD("centerdeg", PT_F32, centerDeg, 3, 60),
  PD("turndeg", PT_F32, turnDeg, 8, 90),
  PD("turnkp", PT_F32, turnKp, 0.002, 0.05),
  PD("turnmax", PT_F32, turnMax, 0.1, 0.6),
  PD("approach", PT_F32, approachSpeed, 0.15, 1.0),
  PD("slow", PT_F32, slowSpeed, 0.1, 0.6),
  PD("alignspeed", PT_F32, alignSpeed, 0.05, 0.5),
  PD("searchspin", PT_F32, searchSpin, 0.1, 0.6),
  PD("searchtime", PT_U32, searchTimeoutMs, 2000, 60000),
  PD("searchtries", PT_U8, searchAttempts, 1, 10),
  PD("searchpause", PT_U16, searchPauseMs, 500, 10000),
  PD("degscale", PT_F32, degradedScale, 0.2, 1.0),
  PD("hdgenable", PT_U8, hdgEnable, 0, 1),
  PD("hdgkp", PT_F32, hdgKp, 0.0, 0.1),
  PD("hdgmax", PT_F32, hdgMaxRot, 0.05, 0.5),
  PD("hdgdead", PT_F32, hdgDeadbandDeg, 0, 10),
  PD("yawsign", PT_I8, imuYawSign, -1, 1),
  PD("imustale", PT_U16, imuStaleMs, 20, 1000),
  PD("imucal", PT_U16, imuCalSamples, 100, 2000),
  PD("imustill", PT_F32, imuStillDps, 0.5, 20),
  PD("imuaddr", PT_U8, imuAddrPref, 0, 0x69),
  PD("goalregms", PT_U16, goalRegisterMs, 1000, 30000),
  PD("goalwidth", PT_U16, goalWidthMm, 200, 1000),
  PD("goaldist", PT_U16, goalDistMm, 200, 3000),
  PD("balldia", PT_U8, ballDiaMm, 20, 80),
  PD("aimspeed", PT_F32, aimSpeed, 0.1, 0.6),
  PD("aimtimeout", PT_U16, aimTimeoutMs, 500, 10000),
  PD("shootenable", PT_U8, shootEnable, 0, 1),
  PD("shootpwm", PT_U8, shootPwm, 80, 255),
  PD("shootparallel", PT_U8, shootParallel, 0, 1),
  PD("shootbackms", PT_U16, shootBackMs, 0, 1500),
  PD("shootbackspeed", PT_F32, shootBackSpeed, 0.1, 1.0),
  PD("shootdrivems", PT_U16, shootDriveMs, 100, 1500),
  PD("shoothold", PT_U16, shootHoldMs, 50, 3000),
  PD("shootcooldown", PT_U16, shootCooldownMs, 500, 20000),
  PD("snapms", PT_U16, snapshotMs, 50, 5000),
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
static void paramAt(size_t i, ParamDef* out) { memcpy_P(out, &PARAMS[i], sizeof(ParamDef)); }
static bool paramFind(const char* name, ParamDef* out) {
  for (size_t i = 0; i < PARAM_COUNT; i++)
    if (strcmp_P(name, (PGM_P)PARAMS[i].name) == 0) { paramAt(i, out); return true; }
  return false;
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
  c.maxManual = 150;
  c.maxAuto = 170;
  c.minPwm = 40;
  c.testPwm = 100;
  c.accelPwmPerS = 500;
  c.decelPwmPerS = 1200;
  c.reverseDeadMs = 60;
  c.manualHoldMs = 600;
  c.ringEnable = 0x0003;         // A0 and A1 (the two fitted phototransistors)
  for (int i = 0; i < RING_CH_MAX; i++) {
    // PLACEHOLDER: the firmware does not know where the receivers look. All channels default to straight ahead (0 deg): the robot then sees THAT a ball is
    // in front but cannot tell left from right. Set the real mounting angles with ':ring bearing <ch> <deg>' (0 = ahead, + = right, +-180 = behind).
    c.ringBearing[i] = 0;
    c.ringNoise[i] = 0;
  }
  c.ringCalDone = 0;             // autonomy is refused until ':ring cal noise' has been run
  c.ringWinUs = 900; c.ringSeen = 20; c.ringNoiseX10 = 30; c.ringNear = 375; c.ringDebounce = 2;
  c.ringFovHalf = 45; c.ringBlend = 100; c.ringMaxGap = 360; c.ringRailMs = 2000; c.ringRailHi = 1000;
  c.modEnable = 1; c.modActiveLow = 3; c.modDebounce = 3; c.modNeedRing = 1;
  c.lostHoldMs = 300;
  c.centerDeg = 15.0f; c.turnDeg = 25.0f; c.turnKp = 0.012f; c.turnMax = 0.35f;
  c.approachSpeed = 0.5f;
  c.slowSpeed = 0.25f;
  c.alignSpeed = 0.2f;
  c.searchSpin = 0.25f;
  c.searchTimeoutMs = 8000;
  c.searchAttempts = 3;
  c.searchPauseMs = 2000;
  c.degradedScale = 0.6f;
  c.hdgEnable = 1;
  c.hdgKp = 0.02f;
  c.hdgMaxRot = 0.25f;
  c.hdgDeadbandDeg = 2.0f;
  c.imuYawSign = -1;
  c.imuStaleMs = 150;
  c.imuCalSamples = 400;
  c.imuStillDps = 3.0f;
  c.imuAddrPref = 0;
  c.goalRegisterMs = 5000; c.goalWidthMm = 450; c.goalDistMm = 1000; c.ballDiaMm = 43; c.aimSpeed = 0.5f; c.aimTimeoutMs = 3000;
  c.shootEnable = 0; c.shootPwm = 255; c.shootParallel = 1; c.shootBackMs = 150; c.shootBackSpeed = 0.7f; c.shootDriveMs = 400; c.shootHoldMs = 100; c.shootCooldownMs = 3000;
  c.snapshotMs = 250;
  c.streamMs = 500;
  c.recoverVerifyMs = 3000;
}

static void cfgSeal(Config& c) {
  c.crc = crc32Update(0, (const uint8_t*)&c, offsetof(Config, crc));
}
static bool cfgCrcOk(const Config& c) {
  return c.crc == crc32Update(0, (const uint8_t*)&c, offsetof(Config, crc));
}

// Appends a field name (in flash) to the comma-separated report. Use the REPN macro with a literal.
static void repNoteP(char* rep, size_t n, PGM_P what) {
  size_t l = strlen(rep);
  if (l + strlen_P(what) + 2 >= n) return;
  if (l) { rep[l++] = ','; rep[l] = 0; }
  strcpy_P(rep + l, what);
}
#define repNote(rep, n, lit) repNoteP((rep), (n), PSTR(lit))

// Replace every out-of-range / non-finite field with its safe default. Returns the number of
// fields replaced; `rep` receives a comma-separated list of the names.
static int cfgValidate(Config& c, char* rep, size_t repN) {
  Config d;                                // on the stack: 8 KB of RAM leave no room for a static copy
  cfgDefaults(d);
  int bad = 0;
  if (repN) rep[0] = 0;
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    ParamDef p;
    paramAt(i, &p);
    if (!paramInRange(p, paramGet(c, p))) {
      paramSet(c, p, paramGet(d, p)); bad++;
      size_t l = strlen(rep);
      if (l + strlen(p.name) + 2 < repN) { if (l) { rep[l++] = ','; rep[l] = 0; } strcpy(rep + l, p.name); }
    }
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
  if (c.slowSpeed > c.approachSpeed) { c.slowSpeed = d.slowSpeed; c.approachSpeed = d.approachSpeed; bad++; repNote(rep, repN, "slow/approach"); }
  if (c.ringSeen >= c.ringNear) { c.ringSeen = d.ringSeen; c.ringNear = d.ringNear; bad++; repNote(rep, repN, "ringseen/ringnear"); }
  if (c.centerDeg >= c.turnDeg) { c.centerDeg = d.centerDeg; c.turnDeg = d.turnDeg; bad++; repNote(rep, repN, "centerdeg/turndeg"); }
  for (int i = 0; i < RING_CH_MAX; i++) {
    if (c.ringBearing[i] < -180 || c.ringBearing[i] > 180) { c.ringBearing[i] = d.ringBearing[i]; bad++; repNote(rep, repN, "ringbearing"); }
    if (c.ringNoise[i] > 1023) { c.ringNoise[i] = 0; bad++; repNote(rep, repN, "ringnoise"); }
  }
  if (c.ringCalDone > 1) { c.ringCalDone = 0; bad++; repNote(rep, repN, "ringcaldone"); }
  if (c.ballDiaMm * 2 >= c.goalWidthMm) { c.ballDiaMm = d.ballDiaMm; c.goalWidthMm = d.goalWidthMm; bad++; repNote(rep, repN, "balldia/goalwidth"); }
  return bad;
}

// The live configuration. Modules take `const Config&` where practical; this is the one instance.
static Config g_cfg;
