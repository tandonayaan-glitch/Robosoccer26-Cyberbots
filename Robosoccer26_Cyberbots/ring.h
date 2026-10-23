// ring.h - ball sensing: phototransistor receivers, each on its own analog input (A0, A1, ...), plus two digital IR obstacle modules.
// (The word "ring" in the names is historical: it is the set of ball receivers, two fitted today, up to eight supported.)
//
//   receivers  A0 (receiver 0), A1 (receiver 1) and optionally A2..A7: phototransistors that see the BALL'S OWN infrared emission. Each gives a signal STRENGTH;
//           the bearing of the ball is estimated from which receivers see it and the mounting angle configured for each (ringBearing).
//   module 0  (D34, front): digital "something is in front, at the distance set by its trimmer" = the ball is in the shooting position.
//   module 1  (D35, second module): read and logged only.
//
// Reading one receiver (NON-BLOCKING state machine, one analogRead of about 112 us per call): one discarded conversion after the ADC input changes, then sample
// the receiver's analog input (10-bit, 0..1023) for ringWinUs and keep the PEAK-TO-PEAK swing.
// The ball emits bursts (the RCJ ball: a 40 kHz carrier gated into ~190 us bursts about every 830 us), so a short sample window can miss a burst;
// the window must cover at least one burst period. The swing is also immune to steady light (sunlight, the IR transmitter LEDs and the modules' unmodulated LEDs): a constant
// level has no swing. What the phototransistor circuit does with the 40 kHz carrier (its response time vs. the load resistor) decides how much
// swing arrives at the pin: that is measured on the robot, not assumed (docs/RING_AND_MODULES.md).
#pragma once
#include "config.h"
#include "health.h"

static const uint32_t MOD_SAMPLE_MS = 5;

struct RingCh {
  bool init;
  uint16_t lo, hi;           // last window: lowest / highest reading (0..1023)
  uint16_t amp;              // last peak-to-peak swing
  float avg;                 // smoothed swing
  float s;                   // strength: avg / ringNear (1.0 = "very close")
  bool seen;                 // debounced
  uint8_t cnt;               // consecutive scans that disagree with `seen`
  uint32_t scans, lastSeenMs;
  uint32_t railSinceMs;      // 0 = not saturated
  bool railFault;
  uint32_t offRailSinceMs;
  uint16_t ampMax;           // largest swing since ':ring reset'
};
static RingCh g_ring[RING_CH_MAX];

struct Module {
  bool raw;                  // detecting, before debounce
  bool active;               // debounced
  uint8_t cnt;
  uint32_t sinceMs, samples;
};
static Module g_mod[MOD_COUNT];
static uint8_t g_modPinMode[MOD_COUNT];            // INPUT or INPUT_PULLUP currently set on each module pin

struct Ball {
  bool valid;                // the ring sees the ball, or did within lostHoldMs
  bool bearingKnown;         // seen right now
  float bearing;             // degrees, 0 = ahead, + right (valid while bearingKnown; the last value is kept in g_lastBearing)
  float strength;            // strongest channel's strength
  uint8_t bestCh;
  uint16_t seenMask;         // which ring channels see the ball
  uint8_t nSeen;
  bool ambiguous;            // strong receivers disagree about the direction (more than ringblend degrees apart): a confident direction is NOT reported
  bool held;                 // front module reports the ball in the shooting position
  bool centered, ready;      // ready = held and the ring says it is straight ahead (or modneedring = 0) and the underside sensor agrees (when it is usable)
  bool positioned;           // the underside sensor says the ball is in the attack position, or it is not usable (then it does not vote)
  uint8_t confidence;        // 0..100 diagnostic figure: signal strength, number of receivers that agree, how long it has been seen
  uint32_t lastSeenMs, readySinceMs, seenSinceMs;
  bool lostEvent;            // set when the ball is lost; the navigation clears it
};
static Ball g_ball;

// Underside ball-position sensor (D40). Type and active level are NOT documented: the level is a setting. It confirms a ball position, it never replaces the receivers.
// Suspect (= ignored, the shooting position is then decided by the other sensors) when:
//   stuck     : it reports "ball" for underStuckMs while the receivers see no ball at all (stuck active, or the wrong polarity with the pull-up on)
//   no answer : everything else says "ball in place" for underStuckMs but it never reports it (disconnected, wrong polarity, mis-aimed)
struct Under { bool raw, active, suspectStuck, suspectMiss; uint8_t cnt, pinMode; uint32_t activeNoBallSinceMs, missSinceMs, samples; };
static Under g_under;
static bool underUsable(const Config& c) { return c.underEnable && !g_under.suspectStuck && !g_under.suspectMiss; }
static float g_lastBearing = 0.0f, g_bearingRate = 0.0f;   // last seen bearing, and how fast it was changing (deg/s, + = moving to the right)
static uint32_t g_lastBearingMs = 0, g_rateRefMs = 0;
static float g_rateRef = 0.0f;
static float g_bHist[3];                       // bearing history, one sample per 50 ms (see ringUpdateBall)
static uint32_t g_bHistMs = 0;
static uint32_t g_ringLastScanMs = 0;                       // last completed channel scan (for the freshness checks)
static uint32_t g_ringCycleMs = 0, g_ringCycles = 0;        // last completed full pass over the enabled channels
static uint8_t g_ringNext = 0;                              // next channel to scan
static uint32_t g_modLastMs = 0;
static uint32_t g_ringScanMaxUs = 0, g_ringScanLastUs = 0;
enum RingPhase : uint8_t { RP_IDLE, RP_SAMPLE };
static uint8_t g_ringPhase = RP_IDLE;                       // state of the non-blocking channel scan
static uint32_t g_ringT0 = 0, g_ringScanT0 = 0;             // micros() when the window opened / when the receiver's scan began
static uint16_t g_ringLo = 0, g_ringHi = 0;
static uint8_t g_ringNSamp = 0;

// ---- noise calibration (':ring cal noise') ----------------------------------------------------------------------
static bool g_ringCalActive = false;
static uint32_t g_ringCalEndMs = 0;
static uint16_t g_ringCalMax[RING_CH_MAX];

static bool ringEnabled(const Config& c, uint8_t i) { return i < RING_CH_MAX && ((c.ringEnable >> i) & 1u) != 0; }
static bool ringUsable(const Config& c, uint8_t i) { return ringEnabled(c, i) && !g_ring[i].railFault; }
static uint8_t ringCountEnabled(const Config& c) { uint8_t n = 0; for (uint8_t i = 0; i < RING_CH_MAX; i++) if (ringEnabled(c, i)) n++; return n; }
static uint8_t ringCountUsable(const Config& c) { uint8_t n = 0; for (uint8_t i = 0; i < RING_CH_MAX; i++) if (ringUsable(c, i)) n++; return n; }
static bool modEnabled(const Config& c, uint8_t i) { return i < MOD_COUNT && ((c.modEnable >> i) & 1u) != 0; }

// ---- pure rules (unit tested) -----------------------------------------------------------------------------------
// Swing above which a channel counts as seeing the ball: the fixed minimum, or the channel's own noise floor times the multiplier.
static float ringThreshold(const Config& c, uint8_t ch) {
  float t = (float)c.ringSeen;
  float n = (float)c.ringNoise[ch] * ((float)c.ringNoiseX10 / 10.0f);
  return n > t ? n : t;
}
// Hysteresis: on at `thr`, off again below 0.6 of it.
static bool ringHyst(bool prev, float amp, float thr) { return prev ? (amp >= 0.6f * thr) : (amp >= thr); }

// Bearing of the ball from the channels that see it: the strongest channel and every seen channel within `blendDeg` of it, averaged as
// vectors weighted by their swing. `amp[]`/`seen[]` indexed by channel, bearings in degrees. Returns false when nothing sees the ball.
static bool ringBearingFrom(const float* amp, const bool* seen, const int16_t* bearing, uint8_t n, float blendDeg, float* out, uint8_t* best) {
  int b = -1;
  for (uint8_t i = 0; i < n; i++) if (seen[i] && (b < 0 || amp[i] > amp[b])) b = i;
  if (b < 0) return false;
  float sx = 0, sy = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (!seen[i] || absf(angDiffDeg((float)bearing[b], (float)bearing[i])) > blendDeg) continue;
    float a = (float)bearing[i] * DEG2RAD;
    sx += amp[i] * sinf(a); sy += amp[i] * cosf(a);
  }
  *out = (sx == 0.0f && sy == 0.0f) ? (float)bearing[b] : wrapDeg180(atan2f(sx, sy) * RAD2DEG);
  *best = (uint8_t)b;
  return true;
}

// Largest arc (degrees) that no usable receiver covers, from the CONFIGURED bearings and the ASSUMED half field of view. A planning figure only.
static float ringCoverageGap(const Config& c, uint8_t* nUsable) {
  float b[RING_CH_MAX];
  uint8_t n = 0;
  for (uint8_t i = 0; i < RING_CH_MAX; i++) if (ringUsable(c, i)) b[n++] = wrapDeg360((float)c.ringBearing[i]);
  if (nUsable) *nUsable = n;
  if (n == 0) return 360.0f;
  for (uint8_t i = 1; i < n; i++) { float v = b[i]; int j = i - 1; while (j >= 0 && b[j] > v) { b[j + 1] = b[j]; j--; } b[j + 1] = v; }
  float worst = 0;
  for (uint8_t i = 0; i < n; i++) {
    float next = (i + 1 < n) ? b[i + 1] : b[0] + 360.0f;
    float gap = (n == 1) ? 360.0f - 2.0f * (float)c.ringFovHalf : (next - b[i]) - 2.0f * (float)c.ringFovHalf;
    if (gap > worst) worst = gap;
  }
  return worst;
}
// Is `deg` inside an arc that no usable receiver covers (a blind spot)?
static bool ringInBlindSpot(const Config& c, float deg) {
  for (uint8_t i = 0; i < RING_CH_MAX; i++)
    if (ringUsable(c, i) && absf(angDiffDeg((float)c.ringBearing[i], deg)) <= (float)c.ringFovHalf) return false;
  return true;
}

// ---- hardware ---------------------------------------------------------------------------------------------------
// Start-up (called once from setup(), after the motor pins are safe): the receiver pins are plain analog inputs (no pull-up), the module pins plain digital inputs.
static void ringBegin() {
  for (uint8_t i = 0; i < RING_CH_MAX; i++) pinMode(PIN_RECV[i], INPUT);   // analog inputs: no pull-up (an open input floats and reads random values: wire every fitted receiver)
  for (uint8_t i = 0; i < MOD_COUNT; i++) { pinMode(PIN_MOD[i], INPUT); g_modPinMode[i] = INPUT; }   // pull-up chosen per module polarity in modService()
  memset(g_ring, 0, sizeof(g_ring));
  memset(g_mod, 0, sizeof(g_mod));
  memset(&g_under, 0, sizeof(g_under));
  memset(&g_ball, 0, sizeof(g_ball));
  g_lastBearing = 0; g_bearingRate = 0; g_lastBearingMs = 0; g_rateRefMs = 0; g_rateRef = 0; g_bHistMs = 0;
  g_ringNext = 0; g_ringLastScanMs = 0; g_ringCycleMs = 0; g_ringCycles = 0; g_modLastMs = 0;
  g_ringCalActive = false;
  g_ringPhase = RP_IDLE;
}

static void ringStep(const Config& c, uint8_t ch, uint16_t lo, uint16_t hi, uint32_t nowMs) {
  RingCh& r = g_ring[ch];
  r.lo = lo; r.hi = hi;
  r.amp = (uint16_t)(hi - lo);
  r.scans++;
  if (r.amp > r.ampMax) r.ampMax = r.amp;
  if (!r.init) { r.init = true; r.avg = (float)r.amp; } else r.avg = 0.5f * r.avg + 0.5f * (float)r.amp;
  r.s = r.avg / (float)c.ringNear;

  // Wiring/saturation: a channel whose LOWEST reading in the window stays at/above ringrailhi is shorted to a supply rail, or blinded by steady light.
  bool sat = lo >= c.ringRailHi;
  if (sat) {
    r.offRailSinceMs = 0;
    if (!r.railSinceMs) r.railSinceMs = nowMs ? nowMs : 1u;
    if (elapsedMs(nowMs, r.railSinceMs) >= c.ringRailMs) r.railFault = true;
  } else {
    r.railSinceMs = 0;
    if (r.railFault) {
      if (!r.offRailSinceMs) r.offRailSinceMs = nowMs ? nowMs : 1u;
      if (elapsedMs(nowMs, r.offRailSinceMs) >= 500) { r.railFault = false; r.offRailSinceMs = 0; }
    }
  }

  bool want = !r.railFault && ringHyst(r.seen, r.avg, ringThreshold(c, ch));
  if (want != r.seen) { if (++r.cnt >= c.ringDebounce) { r.seen = want; r.cnt = 0; } } else r.cnt = 0;
  if (r.seen) r.lastSeenMs = nowMs ? nowMs : 1u;
}

static void ringUpdateBall(const Config& c, uint32_t nowMs) {
  float amp[RING_CH_MAX]; bool seen[RING_CH_MAX];
  uint16_t mask = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < RING_CH_MAX; i++) {
    bool u = ringUsable(c, i) && g_ring[i].seen;
    seen[i] = u; amp[i] = u ? g_ring[i].avg : 0.0f;
    if (u) { mask |= (uint16_t)(1u << i); n++; }
  }
  g_ball.seenMask = mask; g_ball.nSeen = n;
  float brg; uint8_t best;
  bool known = ringBearingFrom(amp, seen, c.ringBearing, RING_CH_MAX, (float)c.ringBlend, &brg, &best);
  g_ball.bearingKnown = known;
  g_ball.ambiguous = false;
  for (uint8_t i = 0; known && i < RING_CH_MAX; i++)                // conflicting pattern: another receiver nearly as strong looks the other way
    if (seen[i] && i != best && amp[i] >= 0.5f * amp[best] && absf(angDiffDeg((float)c.ringBearing[best], (float)c.ringBearing[i])) > (float)c.ringBlend) g_ball.ambiguous = true;
  if (known) {
    g_ball.bearing = brg; g_ball.bestCh = best; g_ball.strength = g_ring[best].s;
    // Last bearing and its rate (where the ball went when it disappears) only from a healthy signal: near the detection threshold channels drop out
    // one by one and the bearing drifts. angDiffDeg(a, b) = a - b; the rate is deg/s, + = towards the right.
    if (g_ring[best].avg >= 3.0f * ringThreshold(c, best)) {
      if (!g_rateRefMs || elapsedMs(nowMs, g_lastBearingMs) > 500) { g_rateRef = brg; g_rateRefMs = nowMs ? nowMs : 1u; g_bearingRate = 0; }
      else if (elapsedMs(nowMs, g_rateRefMs) >= 60) {
        float dt = (float)elapsedMs(nowMs, g_rateRefMs) * 0.001f;
        g_bearingRate = 0.6f * g_bearingRate + 0.4f * (angDiffDeg(brg, g_rateRef) / dt);
        g_rateRef = brg; g_rateRefMs = nowMs ? nowMs : 1u;
      }
      // g_lastBearing is the bearing as it was about 100-150 ms earlier: when the ball disappears its signal fades receiver by receiver, and the bearing
      // computed during the fade drifts towards the receiver that fades last. A sample from before the fade is the honest "last known" direction.
      if (!g_bHistMs || elapsedMs(nowMs, g_lastBearingMs) > 500) { g_bHist[0] = g_bHist[1] = g_bHist[2] = brg; g_bHistMs = nowMs ? nowMs : 1u; }
      else if (elapsedMs(nowMs, g_bHistMs) >= 50) { g_bHist[2] = g_bHist[1]; g_bHist[1] = g_bHist[0]; g_bHist[0] = brg; g_bHistMs = nowMs ? nowMs : 1u; }
      g_lastBearing = g_bHist[2]; g_lastBearingMs = nowMs ? nowMs : 1u;
    }
    g_ball.lastSeenMs = nowMs ? nowMs : 1u;
  } else g_ball.strength = 0.0f;
  bool valid = known || (g_ball.lastSeenMs && elapsedMs(nowMs, g_ball.lastSeenMs) <= c.lostHoldMs);
  if (g_ball.valid && !valid) g_ball.lostEvent = true;
  g_ball.valid = valid;

  bool ringFront = known && absf(g_ball.bearing) <= c.centerDeg;
  g_ball.held = modEnabled(c, 0) && g_mod[0].active;
  g_ball.centered = ringFront || (!c.modNeedRing && g_ball.held);
  bool other = g_ball.held && (c.modNeedRing ? ringFront : true);          // "ball in place" according to everything except the underside sensor
  if (!c.underEnable) { g_under.suspectStuck = g_under.suspectMiss = false; g_under.activeNoBallSinceMs = g_under.missSinceMs = 0; }
  else {
    if (g_under.active && !g_ball.valid) { if (!g_under.activeNoBallSinceMs) g_under.activeNoBallSinceMs = nowMs ? nowMs : 1u; if (elapsedMs(nowMs, g_under.activeNoBallSinceMs) > c.underStuckMs) g_under.suspectStuck = true; }
    else { g_under.activeNoBallSinceMs = 0; if (!g_under.active) g_under.suspectStuck = false; }
    if (other && !g_under.active) { if (!g_under.missSinceMs) g_under.missSinceMs = nowMs ? nowMs : 1u; if (elapsedMs(nowMs, g_under.missSinceMs) > c.underStuckMs) g_under.suspectMiss = true; }
    else { g_under.missSinceMs = 0; if (other && g_under.active) g_under.suspectMiss = false; }      // it answered when it should: trusted again
  }
  g_ball.positioned = !underUsable(c) || g_under.active;
  bool ready = other && (!c.underNeed || g_ball.positioned);
  if (known) { if (!g_ball.seenSinceMs) g_ball.seenSinceMs = nowMs ? nowMs : 1u; } else g_ball.seenSinceMs = 0;
  { int cf = 0;
    if (known) { cf = (int)(clampf(g_ball.strength, 0.0f, 1.0f) * 60.0f) + (g_ball.nSeen >= 2 && !g_ball.ambiguous ? 20 : 0) + (elapsedMs(nowMs, g_ball.seenSinceMs) >= 100 ? 20 : 0); if (g_ball.ambiguous && cf > 30) cf = 30; }
    g_ball.confidence = (uint8_t)(cf > 100 ? 100 : cf); }
  if (ready && !g_ball.ready) g_ball.readySinceMs = nowMs ? nowMs : 1u;
  if (!ready) g_ball.readySinceMs = 0;
  g_ball.ready = ready;
}

static void underService(const Config& c) {
  if (!c.underEnable) { g_under.raw = g_under.active = false; g_under.cnt = 0; return; }
  // Active-LOW (the default) gets the internal pull-up: a disconnected sensor reads HIGH = "no ball". (No pull-down exists on the AVR for active-HIGH.)
  uint8_t want = c.underActiveLow ? INPUT_PULLUP : INPUT;
  if (g_under.pinMode != want + 1u) { pinMode(P_UNDER, want); g_under.pinMode = (uint8_t)(want + 1u); }
  bool level = digitalRead(P_UNDER) != 0;
  g_under.raw = c.underActiveLow ? !level : level;
  g_under.samples++;
  if (g_under.raw != g_under.active) { if (++g_under.cnt >= c.underDebounce) { g_under.active = g_under.raw; g_under.cnt = 0; } } else g_under.cnt = 0;
}

static void modService(const Config& c, uint32_t nowMs) {
  if (g_modLastMs && elapsedMs(nowMs, g_modLastMs) < MOD_SAMPLE_MS) return;
  g_modLastMs = nowMs ? nowMs : 1u;
  underService(c);
  for (uint8_t i = 0; i < MOD_COUNT; i++) {
    Module& m = g_mod[i];
    if (!modEnabled(c, i)) { m.raw = m.active = false; m.cnt = 0; continue; }
    bool activeLow = ((c.modActiveLow >> i) & 1u) != 0;
    // An active-LOW module gets the ATmega's internal pull-up: a module that is not connected (or whose wire fell off) then reads HIGH = "nothing detected"
    // instead of random noise. A real module output overrides the weak pull-up. (There is no pull-down on the AVR: an active-HIGH module that is missing floats.)
    uint8_t want = activeLow ? INPUT_PULLUP : INPUT;
    if (g_modPinMode[i] != want) { pinMode(PIN_MOD[i], want); g_modPinMode[i] = want; }
    bool level = digitalRead(PIN_MOD[i]) != 0;
    m.raw = activeLow ? !level : level;
    m.samples++;
    if (m.raw != m.active) { if (++m.cnt >= c.modDebounce) { m.active = m.raw; m.cnt = 0; m.sinceMs = nowMs ? nowMs : 1u; } } else m.cnt = 0;
  }
}

// Every loop iteration: the modules every 5 ms, and ONE step of the receiver scan (at most one analogRead, about 112 us). The scan never waits:
//   IDLE   -> pick the next enabled receiver; discard one conversion (the ADC sample-and-hold still holds the previous input) and open the window
//   SAMPLE -> one conversion per call into the window's lowest / highest; when the window (ringwinus, at least 2 samples) has elapsed, finish the receiver
static void ringService(const Config& c, uint32_t nowMs) {
  modService(c, nowMs);
  if (g_ringPhase == RP_IDLE) {
    uint8_t tries = 0;
    while (tries++ < RING_CH_MAX && !ringEnabled(c, g_ringNext)) g_ringNext = (uint8_t)((g_ringNext + 1) % RING_CH_MAX);
    if (ringEnabled(c, g_ringNext)) {
      g_ringScanT0 = micros();
      (void)analogRead(PIN_RECV[g_ringNext]);
      g_ringT0 = micros();
      g_ringLo = 1023; g_ringHi = 0; g_ringNSamp = 0;
      g_ringPhase = RP_SAMPLE;
    }
  } else {
    uint8_t ch = g_ringNext;
    uint16_t v = (uint16_t)analogRead(PIN_RECV[ch]);
    if (v < g_ringLo) g_ringLo = v;
    if (v > g_ringHi) g_ringHi = v;
    if (g_ringNSamp < 255) g_ringNSamp++;
    uint32_t t = micros();
    if (((uint32_t)(t - g_ringT0) >= c.ringWinUs && g_ringNSamp >= 2) || g_ringNSamp >= 250) {
      g_ringScanLastUs = (uint32_t)(t - g_ringScanT0);
      if (g_ringScanLastUs > g_ringScanMaxUs) g_ringScanMaxUs = g_ringScanLastUs;
      ringStep(c, ch, g_ringLo, g_ringHi, nowMs);
      if (g_ringCalActive && (uint16_t)(g_ringHi - g_ringLo) > g_ringCalMax[ch]) g_ringCalMax[ch] = (uint16_t)(g_ringHi - g_ringLo);
      g_ringLastScanMs = nowMs ? nowMs : 1u;
      uint8_t nx = (uint8_t)((ch + 1) % RING_CH_MAX);
      if (nx <= ch) { g_ringCycleMs = nowMs ? nowMs : 1u; g_ringCycles++; }          // wrapped round
      g_ringNext = nx;
      g_ringPhase = RP_IDLE;
    }
  }
  ringUpdateBall(c, nowMs);
}

// 20 Hz: wiring / coverage faults. A channel that is switched off is "not tested", never "failed".
static void ringHealth(const Config& c, uint32_t nowMs) {
  uint8_t nUse = ringCountUsable(c), nEn = ringCountEnabled(c);
  char ev[48];
  snprintf(ev, sizeof(ev), "no usable receiver (%u on, %u excluded)", (unsigned)nEn, (unsigned)(nEn - nUse));
  faultUpdate(F_RING_NONE, nUse == 0, c.recoverVerifyMs, ev);
  snprintf(ev, sizeof(ev), "no ring scan for %lu ms (cycles=%lu)", UL(elapsedMs(nowMs, g_ringLastScanMs)), UL(g_ringCycles));
  faultUpdate(F_RING_STALL, nEn > 0 && g_ringLastScanMs != 0 && elapsedMs(nowMs, g_ringLastScanMs) > 600, c.recoverVerifyMs, ev);
  char list[40] = "";
  bool any = false;
  for (uint8_t i = 0; i < RING_CH_MAX; i++) {
    if (!ringEnabled(c, i) || !g_ring[i].railFault) continue;
    char one[12];
    snprintf(one, sizeof(one), "%sA%u", any ? " " : "", (unsigned)i);
    if (strlen(list) + strlen(one) < sizeof(list)) strcat(list, one);
    any = true;
  }
  char evc[48];
  snprintf(evc, sizeof(evc), "saturated: %s", list);
  faultUpdate(F_RING_CHANNEL, any, c.recoverVerifyMs, evc);
  uint8_t nU2;
  float gap = ringCoverageGap(c, &nU2);
  snprintf(ev, sizeof(ev), "gap ~%.0f deg > %u (fov +-%u)", (double)gap, (unsigned)c.ringMaxGap, (unsigned)c.ringFovHalf);
  faultUpdate(F_RING_GAP, nU2 > 0 && gap > (float)c.ringMaxGap, c.recoverVerifyMs, ev);
}

static void ringResetStats() {
  for (uint8_t i = 0; i < RING_CH_MAX; i++) { g_ring[i].ampMax = 0; g_ring[i].scans = 0; }
  g_ringScanMaxUs = 0;
}
