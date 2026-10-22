// ir2.h - the two front IR pairs (IR emitter LED + 2-pin IR receiver each), reflected-light ball detection.
//
//   pair 0: front CENTRE (receiver GPIO35, emitter GPIO4 when pulsed by the firmware): how strong the reflection is -> "seen", and
//           "near" = the ball is close enough to be in the shooting position
//   pair 1: front, slightly OFFSET to one side (receiver GPIO36, emitter GPIO13): with pair 0, whether the ball is centred or to one side
//
// This detects light REFLECTED from the ball, not an opponent goal and not the ball's own emission. With emitter pulsing on (iremit = 1)
// every cycle reads the receiver with its emitter off ("dark": ambient light, the ball's own IR) and on ("lit"); the signal is
// 2048 + (lit - dark), so ambient light and any constant emission cancel and only the reflection of OUR emitter remains.
//
// What the firmware does NOT assume: whether a receiver is read as analog or digital (irMode0/1), or which level means "ball". Each receiver
// is calibrated on the robot (':ir cal clear' with nothing in front, ':ir cal near' with the ball in the shooting position); the strength is
// (signal - clear) / (near - clear), so either polarity works. A 2-pin receiver gives strength only: it cannot report the ball's position or
// distance by itself, and it cannot see a ball outside its field of view: see docs/IR_SENSORS.md.
#pragma once
#include "config.h"
#include "health.h"

static const uint32_t IR_SAMPLE_MS = 5;           // both sensors are read every 5 ms
static const uint16_t IR_RAIL_LO = 8, IR_RAIL_HI = 4087;

struct IrSensor {
  bool init;                 // first sample taken
  uint16_t raw;              // last SIGNAL (what is calibrated): the reading, or 2048 + (lit - dark) with emitter pulsing
  uint16_t dark, lit;        // last readings with the emitter off / on (iremit = 1 only)
  float avg;                 // smoothed reading
  float s;                   // strength: 0 = "clear" value, 1 = "near" value (negative or above 1 are possible)
  bool seen, near;           // debounced states
  uint8_t seenCnt, nearCnt;  // consecutive samples that disagree with the state (debounce)
  uint16_t rawMin, rawMax;   // since the last ':ir reset'
  uint32_t samples;
  uint32_t lastSeenMs;
  uint32_t railSinceMs;      // 0 = not at an ADC end stop
  bool railFault;            // analog reading held at an end stop (wiring: shorted / open / unpowered)
  uint32_t offRailSinceMs;
};
static IrSensor g_irs[IR_SENSORS];

struct Ball {
  bool valid;                // a sensor sees the ball, or did within lostHoldMs
  bool held;                 // sensor 0 reports "near" (and sensor 1 sees it, if irNeedBoth and sensor 1 is usable)
  bool centered, ready;      // ready = held and centred: the shooting position
  bool offsetKnown;          // both sensors usable, mounting side set, ball seen
  float offset;              // -1 = far left ... +1 = far right (robot frame); only meaningful when offsetKnown
  uint8_t seenMask, nearMask;
  float s0, s1;              // strengths of the usable sensors (0 when unusable)
  uint32_t lastSeenMs, readySinceMs;
  bool lostEvent;            // set when the ball is lost; the navigation clears it
};
static Ball g_ball;
static uint32_t g_irLastSampleMs = 0;
static bool g_emitDriven = false;                 // the emitter pins are configured as outputs (iremit = 1)

// ---- calibration accessors --------------------------------------------------------------------
static uint16_t irClearOf(const Config& c, uint8_t i) { return i == 0 ? c.irClear0 : c.irClear1; }
static uint16_t irNearOf(const Config& c, uint8_t i) { return i == 0 ? c.irNear0 : c.irNear1; }
static uint8_t irModeOf(const Config& c, uint8_t i) { return i == 0 ? c.irMode0 : c.irMode1; }
static bool irEnabled(const Config& c, uint8_t i) { return ((c.irEnable >> i) & 1u) != 0; }
static bool irCalibrated(const Config& c, uint8_t i) {
  uint16_t a = irClearOf(c, i), b = irNearOf(c, i);
  return (a > b ? a - b : b - a) >= 50;
}
// Usable for navigation: switched on, calibrated, and not reporting a wiring fault.
static bool irUsable(const Config& c, uint8_t i) { return irEnabled(c, i) && irCalibrated(c, i) && !g_irs[i].railFault; }

// ---- pure rules (unit tested) -----------------------------------------------------------------------
// Strength of a reading: 0 at `clear`, 1 at `near`, linear in between and beyond (either polarity).
static float irStrength(float avg, uint16_t clearRaw, uint16_t nearRaw) {
  float span = (float)nearRaw - (float)clearRaw;
  if (span > -50.0f && span < 50.0f) return 0.0f;
  return (avg - (float)clearRaw) / span;
}
// Hysteresis: switch on at `onPct` of the span, off again below `offFrac` of that.
static bool irHyst(bool prev, float s, float onPct, float offFrac) {
  float on = onPct / 100.0f;
  return prev ? (s >= on * offFrac) : (s >= on);
}
// Sideways position of the ball (-1 left ... +1 right) from the two sensors. `side` = +1 when sensor 1 is mounted to the right of
// sensor 0 (seen from behind the robot), -1 when to the left. Both strengths seen: the one that is stronger shows the side;
// only sensor 1 sees it: the ball is on sensor 1's side; only sensor 0 sees it: on the other side.
static bool irOffsetFrom(bool seen0, bool seen1, float s0, float s1, int8_t side, float* offset) {
  if (side == 0 || (!seen0 && !seen1)) return false;
  float e;
  if (seen0 && seen1) { float t = s0 + s1; e = t > 0.0f ? (s1 - s0) / t : 0.0f; }
  else e = seen1 ? 1.0f : -1.0f;
  *offset = clampf(e * (float)side, -1.0f, 1.0f);
  return true;
}

static uint16_t irReadRaw(const Config& c, uint8_t i) {
  if (irModeOf(c, i)) return digitalRead(PIN_IR[i]) ? 4095 : 0;
  return (uint16_t)analogRead(PIN_IR[i]);
}

static void irBegin() {
  for (uint8_t i = 0; i < IR_SENSORS; i++) pinMode(PIN_IR[i], INPUT);   // input-only pins: no pull resistors exist
  memset(g_irs, 0, sizeof(g_irs));
  memset(&g_ball, 0, sizeof(g_ball));
  for (uint8_t i = 0; i < IR_SENSORS; i++) g_irs[i].rawMin = 0xFFFF;
  g_irLastSampleMs = 0;
  g_emitDriven = false;                              // the emitter pins are not touched until iremit = 1
}

// Emitter pins: outputs, LOW (off) between pulses, only while iremit = 1; released to inputs again when it is switched off.
static void irEmitSync(const Config& c) {
  if (c.irEmit && !g_emitDriven) {
    for (uint8_t i = 0; i < IR_SENSORS; i++) { digitalWrite(PIN_IR_EMIT[i], LOW); pinMode(PIN_IR_EMIT[i], OUTPUT); digitalWrite(PIN_IR_EMIT[i], LOW); }
    g_emitDriven = true;
    for (uint8_t i = 0; i < IR_SENSORS; i++) g_irs[i].init = false;          // the signal changes meaning: restart the smoothing
  } else if (!c.irEmit && g_emitDriven) {
    for (uint8_t i = 0; i < IR_SENSORS; i++) { digitalWrite(PIN_IR_EMIT[i], LOW); pinMode(PIN_IR_EMIT[i], INPUT); }
    g_emitDriven = false;
    for (uint8_t i = 0; i < IR_SENSORS; i++) g_irs[i].init = false;
  }
}

static void irStep(const Config& c, uint8_t i, uint16_t raw, uint32_t nowMs) {
  IrSensor& s = g_irs[i];
  s.raw = raw;
  s.samples++;
  if (raw < s.rawMin) s.rawMin = raw;
  if (raw > s.rawMax) s.rawMax = raw;
  bool digital = irModeOf(c, i) != 0;
  if (!s.init) { s.init = true; s.avg = (float)raw; } else s.avg = digital ? (float)raw : s.avg * 0.5f + (float)raw * 0.5f;

  // Wiring check (analog only): a reading held at the end stop on the CLEAR side of the calibration means shorted, open or unpowered
  // (a rising sensor stuck at 0, a falling one stuck at 4095). The end stop on the "near" side is legitimate: a ball touching the
  // sensor can saturate it. If the clear reading itself lies at that end stop the check is off.
  uint16_t clr = irClearOf(c, i), nr = irNearOf(c, i);
  bool rising = nr > clr;
  bool clearAtRail = rising ? (clr <= IR_RAIL_LO + 30) : (clr >= IR_RAIL_HI - 30);
  bool atRail = !c.irEmit && !digital && irCalibrated(c, i) && !clearAtRail && (rising ? raw <= IR_RAIL_LO : raw >= IR_RAIL_HI);   // no wiring check in emitter mode: a dark receiver cannot be told from a dead one
  if (atRail) {
    s.offRailSinceMs = 0;
    if (!s.railSinceMs) s.railSinceMs = nowMs ? nowMs : 1u;
    if (elapsedMs(nowMs, s.railSinceMs) >= c.irRailMs) s.railFault = true;
  } else {
    s.railSinceMs = 0;
    if (s.railFault) {
      if (!s.offRailSinceMs) s.offRailSinceMs = nowMs ? nowMs : 1u;
      if (elapsedMs(nowMs, s.offRailSinceMs) >= 500) { s.railFault = false; s.offRailSinceMs = 0; }
    }
  }

  if (!irCalibrated(c, i)) { s.s = 0.0f; s.seen = s.near = false; s.seenCnt = s.nearCnt = 0; return; }
  s.s = irStrength(s.avg, clr, nr);
  bool wantSeen = irHyst(s.seen, s.s, (float)c.irSeenPct, 0.6f);
  bool wantNear = irHyst(s.near, s.s, (float)c.irNearPct, 0.85f);
  if (wantSeen != s.seen) { if (++s.seenCnt >= c.irDebounce) { s.seen = wantSeen; s.seenCnt = 0; } } else s.seenCnt = 0;
  if (wantNear != s.near) { if (++s.nearCnt >= c.irDebounce) { s.near = wantNear; s.nearCnt = 0; } } else s.nearCnt = 0;
  if (s.seen) s.lastSeenMs = nowMs;
}

static void irUpdateBall(const Config& c, uint32_t nowMs) {
  bool u0 = irUsable(c, 0), u1 = irUsable(c, 1);
  uint8_t seen = 0, nr = 0;
  if (u0 && g_irs[0].seen) seen |= 1u;
  if (u1 && g_irs[1].seen) seen |= 2u;
  if (u0 && g_irs[0].near) nr |= 1u;
  if (u1 && g_irs[1].near) nr |= 2u;
  g_ball.seenMask = seen; g_ball.nearMask = nr;
  g_ball.s0 = u0 ? g_irs[0].s : 0.0f;
  g_ball.s1 = u1 ? g_irs[1].s : 0.0f;
  if (seen) g_ball.lastSeenMs = nowMs ? nowMs : 1u;
  bool valid = seen != 0 || (g_ball.lastSeenMs && elapsedMs(nowMs, g_ball.lastSeenMs) <= c.lostHoldMs);
  if (g_ball.valid && !valid) g_ball.lostEvent = true;
  g_ball.valid = valid;

  float off;
  if (u0 && u1 && irOffsetFrom((seen & 1u) != 0, (seen & 2u) != 0, g_ball.s0, g_ball.s1, c.irOffsetSide, &off)) {
    g_ball.offset = g_ball.offsetKnown ? 0.5f * g_ball.offset + 0.5f * off : off;
    g_ball.offsetKnown = true;
  } else if (!valid) g_ball.offsetKnown = false;           // forget the side only once the ball is gone

  g_ball.held = u0 && (nr & 1u) && (!c.irNeedBoth || !u1 || (seen & 2u));
  g_ball.centered = !g_ball.offsetKnown || absf(g_ball.offset) <= c.centerBand;
  bool ready = g_ball.held && g_ball.centered;
  if (ready && !g_ball.ready) g_ball.readySinceMs = nowMs ? nowMs : 1u;
  if (!ready) g_ball.readySinceMs = 0;
  g_ball.ready = ready;
}

// Every loop iteration: cheap unless 5 ms have passed. With emitter pulsing each enabled pair costs one settle time (irsettle, 200 us).
static void irService(const Config& c, uint32_t nowMs) {
  if (g_irLastSampleMs && elapsedMs(nowMs, g_irLastSampleMs) < IR_SAMPLE_MS) return;
  g_irLastSampleMs = nowMs ? nowMs : 1u;
  irEmitSync(c);
  if (!c.irEmit) {
    for (uint8_t i = 0; i < IR_SENSORS; i++) if (irEnabled(c, i)) { g_irs[i].dark = g_irs[i].lit = 0; irStep(c, i, irReadRaw(c, i), nowMs); }
  } else {
    uint16_t dark[IR_SENSORS] = {0, 0};
    for (uint8_t i = 0; i < IR_SENSORS; i++) if (irEnabled(c, i)) dark[i] = irReadRaw(c, i);       // both receivers, all emitters off
    for (uint8_t i = 0; i < IR_SENSORS; i++) {
      if (!irEnabled(c, i)) continue;
      digitalWrite(PIN_IR_EMIT[i], HIGH);                                                           // only THIS pair's emitter: no cross-talk between the pairs
      delayMicroseconds(c.irSettleUs);
      uint16_t lit = irReadRaw(c, i);
      digitalWrite(PIN_IR_EMIT[i], LOW);
      int d = (int)lit - (int)dark[i];
      if (irModeOf(c, i)) d /= 2;                                                                   // digital receiver: 0 / 4095 -> +-2047
      int sig = 2048 + d;
      g_irs[i].dark = dark[i]; g_irs[i].lit = lit;
      irStep(c, i, (uint16_t)(sig < 0 ? 0 : (sig > 4095 ? 4095 : sig)), nowMs);
    }
  }
  irUpdateBall(c, nowMs);
}

// 20 Hz: wiring faults. A sensor that is not enabled or not calibrated is "not tested", never "failed".
static void irHealth(const Config& c, uint32_t nowMs) {
  (void)nowMs;
  char ev[72];
  snprintf(ev, sizeof(ev), "sensor 0 (GPIO%u) held at an ADC end stop (raw %u) for %u ms", (unsigned)PIN_IR[0], (unsigned)g_irs[0].raw, (unsigned)c.irRailMs);
  faultUpdate(F_IR_FRONT, irEnabled(c, 0) && g_irs[0].railFault, c.recoverVerifyMs, ev);
  snprintf(ev, sizeof(ev), "sensor 1 (GPIO%u) held at an ADC end stop (raw %u) for %u ms", (unsigned)PIN_IR[1], (unsigned)g_irs[1].raw, (unsigned)c.irRailMs);
  faultUpdate(F_IR_ALIGN, irEnabled(c, 1) && g_irs[1].railFault, c.recoverVerifyMs, ev);
}

static void irResetStats() {
  for (uint8_t i = 0; i < IR_SENSORS; i++) { g_irs[i].rawMin = 0xFFFF; g_irs[i].rawMax = 0; g_irs[i].samples = 0; }
}
