// irscan.h - IR receiver acquisition through the CD74HC4067, per-channel health, direction estimate.
//
// Why windows: a TSOP17xx demodulates a 38 kHz carrier and outputs the *envelope* of the ball's
// bursts. The competition ball pulses its carrier at a low repetition rate (about 1.2 kHz for the
// commonly used ball - to be confirmed against your ball). One mux channel is read at a time, so a
// channel must be observed for several pulse periods before "no pulses" means "no ball".
// The window (irWindowUs, default 3 ms = ~3.6 periods at 1.2 kHz) is timed by micros() while an
// edge interrupt on GPIO34 counts pulses, so loop() keeps running motors during the window.
// One full scan = enabled_channels * (window + settle). Nothing here measures signal strength:
// the receiver output is digital, so only "pulses seen / not seen" and pulse counts are reported.
#pragma once
#include "config.h"
#include "health.h"

enum WinState : uint8_t { WS_NONE, WS_DETECT, WS_STUCK, WS_NOISY, WS_INVALID };
enum ChStatus : uint8_t {
  CH_DISABLED, CH_UNVERIFIED, CH_VERIFIED, CH_SUSPECT_STUCK, CH_SUSPECT_NOISY, CH_SUSPECT_TEST,
  CH_FAIL_STUCK, CH_FAIL_TEST
};
static const char* WS_NAME[] = {"none", "DETECT", "STUCK", "NOISY", "invalid"};
static const char* CH_NAME[] = {"DISABLED", "UNVERIFIED", "VERIFIED", "SUSPECT_STUCK", "SUSPECT_NOISY",
                                "SUSPECT_TEST", "FAIL_STUCK", "FAIL_TEST"};

struct IrChan {
  uint8_t raw;            // pin level at the end of the last window (0/1, as read)
  uint8_t winState;       // WinState of the last window
  uint8_t detect;         // last window was a detection
  uint8_t active;         // smoothed (hysteresis) detection
  uint16_t edges;         // active-level pulses in the last window
  uint32_t activeUs;      // time the output was at its active level in the last window
  uint32_t periodUs;      // measured mean pulse period of the last window (0 if < 2 pulses)
  uint16_t hist;          // 1 bit per window, newest = bit 0
  uint32_t windows, detCount, overruns;
  uint32_t lastWindowMs, lastDetectMs;
  uint32_t stuckSinceMs;
  uint8_t noisyScore, cleanStreak, stuckSuspect, stuckFail, noisy;
};
static IrChan g_ir[MUX_CHANNELS];

// ---- edge counter (ISR) -----------------------------------------------------------
#define IR_EDGE_CAP 250
static volatile bool g_isrGate = false;
static volatile uint8_t g_isrActiveLevel = 0;
static volatile bool g_isrInActive = false;
static volatile uint16_t g_isrEdges = 0;
static volatile uint32_t g_isrActAcc = 0, g_isrActStart = 0, g_isrFirst = 0, g_isrLast = 0;

static void IRAM_ATTR irIsr() {
  if (!g_isrGate) return;
  bool isActive = (digitalRead(PIN_MUX_SIG) == g_isrActiveLevel);
  uint32_t t = micros();
  if (isActive) {
    if (!g_isrInActive) {
      g_isrInActive = true;
      g_isrActStart = t;
      if (g_isrEdges == 0) g_isrFirst = t;
      g_isrLast = t;
      g_isrEdges++;
      if (g_isrEdges >= IR_EDGE_CAP) g_isrGate = false;   // flood guard: bounded interrupt load
    }
  } else if (g_isrInActive) {
    g_isrInActive = false;
    g_isrActAcc += t - g_isrActStart;
  }
}

// ---- scan scheduler ------------------------------------------------------------------
struct ScanState {
  bool running, open;
  uint8_t cur;
  uint32_t winStartUs, cycleStartUs;
  uint32_t cycleCount, lastCycleMs;
  uint32_t cycleUs, cycleMinUs, cycleMaxUs;
  float cycleAvgUs;
  uint8_t enabledCount;
  uint32_t lastWinPlannedUs;
};
static ScanState g_scan;

struct BallEst {
  bool valid;             // acquired (hysteresis) and not yet lost
  bool anyDetect;         // at least one trusted channel active in the latest cycle
  bool ambiguous;         // active channels disagree (low resultant length)
  float bearingDeg;       // (-180,180], 0 = forward, + = clockwise. Heuristic, from calibrated bearings
  float conf;             // 0..1 HEURISTIC, see irEstimate()
  float W;                // sum of per-channel hit fractions
  float rInst;            // instantaneous resultant length 0..1 (agreement between channels)
  uint8_t nActive;
  uint16_t actMask;
  uint32_t lastGoodMs, validSinceMs, lastDetectMs, lostAtMs, cycleStamp;
  float fx, fy;
  uint8_t acqCount;
  bool lostEvent;         // set for one poll when the ball is lost (consumed by navigation)
};
static BallEst g_ball;

// operator probes (":learn" coverage mapping, ":verify" controlled receiver test)
struct Probe {
  uint8_t mode;           // 0 none, 1 learn, 2 verify
  uint8_t ch;
  float deg;
  uint32_t endMs;
  uint32_t hits[MUX_CHANNELS], windows[MUX_CHANNELS], maxEdges[MUX_CHANNELS];
};
static Probe g_probe;
struct CovRow { float deg; uint16_t mask; uint8_t pct[MUX_CHANNELS]; };
#define COV_MAX 48
static CovRow g_cov[COV_MAX];
static uint8_t g_covN = 0;

// ---- helpers ---------------------------------------------------------------------------
static void irSelect(uint8_t ch) {
  digitalWrite(PIN_MUX_S0, (ch & 1) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S1, (ch & 2) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S2, (ch & 4) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S3, (ch & 8) ? HIGH : LOW);
}

static uint8_t irCountEnabled(const Config& c) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) if (c.ch[i].enabled) n++;
  return n;
}

// Pure window classifier (unit tested). activeUs: time at active level; startActive/endActive:
// level at window open/close.
static WinState irClassifyWindow(const Config& c, uint16_t edges, uint32_t activeUs, uint32_t windowUs,
                                 bool startActive, bool endActive) {
  if (edges == 0) {
    if (startActive && endActive && activeUs + windowUs / 20 >= windowUs) return WS_STUCK;
    return WS_NONE;
  }
  float windowMs = (float)windowUs / 1000.0f;
  if (edges >= IR_EDGE_CAP || (float)edges > (float)c.irNoisyPerMs * windowMs) return WS_NOISY;
  if (edges >= c.irMinEdges) return WS_DETECT;
  return WS_NONE;   // a single pulse is not enough evidence
}

static ChStatus irChStatus(const Config& c, uint8_t ch) {
  const ChCfg& h = c.ch[ch];
  const IrChan& s = g_ir[ch];
  if (!h.enabled) return CH_DISABLED;
  if (s.stuckFail) return CH_FAIL_STUCK;
  if (h.verifyFails >= 2) return CH_FAIL_TEST;
  if (s.stuckSuspect) return CH_SUSPECT_STUCK;
  if (s.noisy) return CH_SUSPECT_NOISY;
  if (h.verifyFails == 1) return CH_SUSPECT_TEST;
  return h.verified ? CH_VERIFIED : CH_UNVERIFIED;
}

// A channel is used for navigation unless it is disabled/untrusted or *currently* shows a
// condition that positively indicates a fault (stuck output, interference, failed tests).
static bool irTrusted(const Config& c, uint8_t ch) {
  if (!c.ch[ch].enabled || !c.ch[ch].trusted) return false;
  ChStatus st = irChStatus(c, ch);
  return !(st == CH_FAIL_STUCK || st == CH_FAIL_TEST || st == CH_SUSPECT_STUCK || st == CH_SUSPECT_NOISY);
}

// Largest bearing interval not covered by any trusted receiver, using the *configured* bearings
// and the assumed half field-of-view. It is a planning figure from metadata, not a measurement.
static float irCoverageGap(const Config& c, uint8_t* nTrusted, uint8_t* nUncal) {
  float b[MUX_CHANNELS];
  uint8_t n = 0, nu = 0;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) {
    if (!irTrusted(c, i)) continue;
    b[n++] = wrapDeg360(c.ch[i].bearingDeg);
    if (!c.ch[i].bearingCal) nu++;
  }
  if (nTrusted) *nTrusted = n;
  if (nUncal) *nUncal = nu;
  if (n == 0) return 360.0f;
  for (uint8_t i = 1; i < n; i++) {           // insertion sort (n <= 16)
    float v = b[i]; int j = i - 1;
    while (j >= 0 && b[j] > v) { b[j + 1] = b[j]; j--; }
    b[j + 1] = v;
  }
  float worst = 0;
  for (uint8_t i = 0; i < n; i++) {
    float next = (i + 1 < n) ? b[i + 1] : b[0] + 360.0f;
    float gap = (next - b[i]) - 2.0f * c.fovHalfDeg;
    if (n == 1) gap = 360.0f - 2.0f * c.fovHalfDeg;
    if (gap > worst) worst = gap;
  }
  return worst;
}

// ---- window open/close ---------------------------------------------------------------
static void irOpenWindow(const Config& c, uint8_t ch) {
  g_isrGate = false;
  irSelect(ch);
  delayMicroseconds(c.irSettleUs);          // bounded (<=100 us) mux/line settling
  g_isrActiveLevel = c.ch[ch].activeLow ? 0 : 1;
  g_isrEdges = 0; g_isrActAcc = 0; g_isrFirst = 0; g_isrLast = 0;
  uint32_t t = micros();
  g_isrInActive = (digitalRead(PIN_MUX_SIG) == g_isrActiveLevel);
  g_isrActStart = t;
  g_scan.winStartUs = t;
  g_scan.cur = ch;
  g_scan.open = true;
  g_scan.lastWinPlannedUs = c.irWindowUs;
  g_isrGate = true;
}

static void irChannelUpdate(const Config& c, uint8_t ch, WinState ws, uint32_t nowMs) {
  IrChan& s = g_ir[ch];
  s.winState = ws;
  s.windows++;
  s.lastWindowMs = nowMs;
  if (ws == WS_INVALID) { s.overruns++; g_stats.scanOverruns++; return; }   // never interpret a late window
  s.detect = (ws == WS_DETECT);
  s.hist = (uint16_t)((s.hist << 1) | (s.detect ? 1 : 0));
  if (s.detect) { s.detCount++; s.lastDetectMs = nowMs; }
  int hits = popcount16(s.hist & 0xFF);
  if (!s.active && hits >= c.irHistMinOn) s.active = 1;
  else if (s.active && hits == 0) s.active = 0;

  // Stuck at active level: a modulated ball can never hold the output active continuously.
  if (ws == WS_STUCK) {
    if (!s.stuckSinceMs) s.stuckSinceMs = nowMs ? nowMs : 1;
    uint32_t dur = elapsedMs(nowMs, s.stuckSinceMs);
    if (dur >= c.irStuckMs) s.stuckSuspect = 1;
    if (dur >= c.irStuckFailMs) s.stuckFail = 1;
    s.cleanStreak = 0;
  } else {
    s.stuckSinceMs = 0;
    if ((s.stuckSuspect || s.stuckFail) && ws != WS_NOISY) {
      if (s.cleanStreak < 255) s.cleanStreak++;
      if (s.cleanStreak >= 25 && !s.stuckFail) s.stuckSuspect = 0;   // transient recovered
      if (s.cleanStreak >= 25 && s.stuckFail) { s.stuckFail = 0; s.stuckSuspect = 1; s.cleanStreak = 0; }  // needs operator verify to fully clear
    }
  }
  // Interference / noise
  if (ws == WS_NOISY) { if (s.noisyScore < 8) s.noisyScore++; s.cleanStreak = 0; }
  else if (s.noisyScore > 0) s.noisyScore--;
  if (s.noisyScore >= 4) s.noisy = 1;
  else if (s.noisyScore == 0) s.noisy = 0;
}

static void irProbeFinish(Config& c, uint32_t nowMs);

static void irCloseWindow(Config& c, uint32_t nowMs) {
  g_isrGate = false;
  uint32_t t = micros();
  uint32_t actual = t - g_scan.winStartUs;
  uint8_t ch = g_scan.cur;
  bool endActive = (digitalRead(PIN_MUX_SIG) == g_isrActiveLevel);
  bool startActive = false;
  uint32_t acc = g_isrActAcc;
  if (g_isrInActive) acc += t - g_isrActStart;
  uint16_t edges = g_isrEdges;
  IrChan& s = g_ir[ch];
  s.raw = digitalRead(PIN_MUX_SIG) ? 1 : 0;
  s.edges = edges;
  s.activeUs = acc;
  s.periodUs = (edges >= 2) ? (g_isrLast - g_isrFirst) / (uint32_t)(edges - 1) : 0;
  // "started active with no pulse yet" is encoded by edges==0 && acc ~ window; startActive only matters then.
  startActive = (edges == 0) && (acc + c.irWindowUs / 20 >= actual);
  WinState ws;
  if (actual > g_scan.lastWinPlannedUs + g_scan.lastWinPlannedUs / 2 + 500) ws = WS_INVALID;   // loop was late
  else ws = irClassifyWindow(c, edges, acc, actual, startActive, endActive);
  irChannelUpdate(c, ch, ws, nowMs);
  if (g_probe.mode && ws != WS_INVALID) {
    g_probe.windows[ch]++;
    if (ws == WS_DETECT) g_probe.hits[ch]++;
    if (edges > g_probe.maxEdges[ch]) g_probe.maxEdges[ch] = edges;
  }
  g_scan.open = false;
}

static void irEstimate(const Config& c, uint32_t nowMs) {
  float sx = 0, sy = 0, W = 0;
  uint8_t n = 0;
  uint16_t mask = 0;
  bool unverified = false;
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
    if (!irTrusted(c, ch) || !g_ir[ch].active) continue;
    float w = (float)popcount16(g_ir[ch].hist & 0xFF) / 8.0f;
    float b = c.ch[ch].bearingDeg * DEG2RAD;
    sx += w * sinf(b); sy += w * cosf(b); W += w; n++;
    mask |= (uint16_t)(1u << ch);
    if (!c.ch[ch].verified) unverified = true;
  }
  g_ball.nActive = n;
  g_ball.actMask = mask;
  g_ball.W = W;
  g_ball.anyDetect = n > 0;
  g_ball.cycleStamp = g_scan.cycleCount;
  if (n == 0 || W <= 0) { g_ball.acqCount = 0; g_ball.rInst = 0; if (!g_ball.valid) g_ball.conf = 0; return; }
  float ux = sx / W, uy = sy / W;
  g_ball.rInst = clampf(sqrtf(ux * ux + uy * uy), 0, 1);
  // Temporal smoothing of the unit-vector (not the angle), so 359/1 degrees average to 0.
  if (!g_ball.valid && elapsedMs(nowMs, g_ball.lastDetectMs) > c.lostHoldMs) { g_ball.fx = ux; g_ball.fy = uy; }
  else { g_ball.fx += 0.5f * (ux - g_ball.fx); g_ball.fy += 0.5f * (uy - g_ball.fy); }
  float rf = clampf(sqrtf(g_ball.fx * g_ball.fx + g_ball.fy * g_ball.fy), 0, 1);
  g_ball.bearingDeg = wrapDeg180(atan2f(g_ball.fx, g_ball.fy) * RAD2DEG);
  // CONFIDENCE (heuristic, not empirically calibrated):
  //   conf = R_filtered * min(1, W) * (0.85 if any contributing channel is still UNVERIFIED)
  //   R_filtered : agreement between active channels' bearings (1 = all point the same way)
  //   W          : summed hit fractions over the last 8 windows (1 channel with 8/8 hits = 1.0)
  g_ball.conf = rf * (W < 1.0f ? W : 1.0f) * (unverified ? 0.85f : 1.0f);
  g_ball.ambiguous = g_ball.rInst < 0.5f;
  g_ball.lastDetectMs = nowMs;
  if (g_ball.conf >= c.acqConf * 0.6f) g_ball.lastGoodMs = nowMs;
  if (g_ball.conf >= c.acqConf) { if (g_ball.acqCount < 255) g_ball.acqCount++; } else g_ball.acqCount = 0;
  if (!g_ball.valid && g_ball.acqCount >= 2) { g_ball.valid = true; g_ball.validSinceMs = nowMs; }
}

static void irBallTimeout(const Config& c, uint32_t nowMs) {
  if (g_ball.valid && elapsedMs(nowMs, g_ball.lastGoodMs) > c.lostHoldMs) {
    g_ball.valid = false;
    g_ball.lostAtMs = nowMs;
    g_ball.lostEvent = true;
    g_ball.conf = 0;
    g_ball.acqCount = 0;
  }
}

static uint8_t irNextEnabled(const Config& c, uint8_t from, bool* wrapped) {
  for (uint8_t k = 1; k <= MUX_CHANNELS; k++) {
    uint8_t i = (uint8_t)((from + k) % MUX_CHANNELS);
    if (c.ch[i].enabled) { *wrapped = (from + k) >= MUX_CHANNELS || i <= from; return i; }
  }
  *wrapped = true;
  return 0;
}

static void irBegin(const Config& c) {
  pinMode(PIN_MUX_SIG, INPUT);                       // GPIO34: input only, external pull-up recommended
  pinMode(PIN_MUX_S0, OUTPUT); pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT); pinMode(PIN_MUX_S3, OUTPUT);
  irSelect(0);
  memset(g_ir, 0, sizeof(g_ir));
  memset(&g_ball, 0, sizeof(g_ball));
  memset(&g_scan, 0, sizeof(g_scan));
  g_scan.enabledCount = irCountEnabled(c);
  g_scan.lastCycleMs = millis();
  g_scan.cycleMinUs = 0xFFFFFFFFu;
  attachInterrupt(digitalPinToInterrupt(PIN_MUX_SIG), irIsr, CHANGE);
  g_scan.running = true;   // armed even with zero enabled channels, so enabling one later starts scanning
}

// Call every loop iteration. Never blocks longer than the settle delay.
static void irService(Config& c, uint32_t nowMs) {
  if (!g_scan.running) { irBallTimeout(c, nowMs); return; }
  g_scan.enabledCount = irCountEnabled(c);
  if (g_scan.enabledCount == 0) { irBallTimeout(c, nowMs); return; }
  uint32_t nowUs = micros();
  if (!g_scan.open) {
    bool w;
    uint8_t first = c.ch[0].enabled ? 0 : irNextEnabled(c, 0, &w);
    g_scan.cycleStartUs = nowUs;
    irOpenWindow(c, first);
  } else if ((uint32_t)(nowUs - g_scan.winStartUs) >= c.irWindowUs) {
    irCloseWindow(c, nowMs);
    bool wrapped;
    uint8_t next = irNextEnabled(c, g_scan.cur, &wrapped);
    if (wrapped) {
      uint32_t endUs = micros();
      g_scan.cycleUs = endUs - g_scan.cycleStartUs;
      if (g_scan.cycleUs < g_scan.cycleMinUs) g_scan.cycleMinUs = g_scan.cycleUs;
      if (g_scan.cycleUs > g_scan.cycleMaxUs) g_scan.cycleMaxUs = g_scan.cycleUs;
      g_scan.cycleAvgUs = g_scan.cycleCount ? g_scan.cycleAvgUs * 0.9f + (float)g_scan.cycleUs * 0.1f : (float)g_scan.cycleUs;
      g_scan.cycleCount++;
      g_scan.lastCycleMs = nowMs;
      g_scan.cycleStartUs = endUs;
      irEstimate(c, nowMs);
      if (g_probe.mode && deadlineReached(nowMs, g_probe.endMs)) irProbeFinish(c, nowMs);
    }
    irOpenWindow(c, next);
  }
  irBallTimeout(c, nowMs);
}

// ---- operator probes -----------------------------------------------------------------------
static void irProbeStart(uint8_t mode, uint8_t ch, float deg, uint32_t durMs, uint32_t nowMs) {
  memset(&g_probe, 0, sizeof(g_probe));
  g_probe.mode = mode; g_probe.ch = ch; g_probe.deg = deg;
  g_probe.endMs = nowMs + durMs;
}

static void irProbeFinish(Config& c, uint32_t nowMs) {
  (void)nowMs;
  if (g_probe.mode == 1) {                       // learn: which receivers see the ball at this bearing?
    CovRow row;
    memset(&row, 0, sizeof(row));
    row.deg = g_probe.deg;
    for (uint8_t i = 0; i < MUX_CHANNELS; i++) {
      uint32_t w = g_probe.windows[i];
      row.pct[i] = w ? (uint8_t)((g_probe.hits[i] * 100u) / w) : 0;
      if (w && row.pct[i] >= 25) row.mask |= (uint16_t)(1u << i);
    }
    if (g_covN < COV_MAX) g_cov[g_covN++] = row;
    say("TEST", "learn bearing=%.0f deg: seen by mask=0x%04X (>=25%% of windows)", g_probe.deg, (unsigned)row.mask);
    for (uint8_t i = 0; i < MUX_CHANNELS; i++)
      if (g_probe.windows[i])
        say("TEST", "  C%u hits=%lu/%lu (%u%%) maxEdges=%lu", i, UL(g_probe.hits[i]), UL(g_probe.windows[i]),
            (unsigned)row.pct[i], UL(g_probe.maxEdges[i]));
  } else if (g_probe.mode == 2) {                // verify: ball held at this receiver's face
    uint8_t ch = g_probe.ch;
    uint32_t w = g_probe.windows[ch], h = g_probe.hits[ch];
    uint32_t need = w / 4 > 3 ? w / 4 : 3;
    if (w >= 6 && h >= need) {
      c.ch[ch].verified = 1; c.ch[ch].verifyFails = 0;
      g_ir[ch].stuckFail = 0;
      say("TEST", "C%u PASS: detected the ball in %lu/%lu windows (controlled test). Verified flag set (use :save to keep).", ch, UL(h), UL(w));
    } else {
      if (c.ch[ch].verifyFails < 5) c.ch[ch].verifyFails++;
      c.ch[ch].verified = 0;
      g_stats.channelTestFailMask |= (uint16_t)(1u << ch);
      say("TEST", "C%u %s: only %lu/%lu windows detected (needed %lu). Fails=%u. Check the ball is on and at the receiver face before concluding it is faulty.",
          ch, c.ch[ch].verifyFails >= 2 ? "FAIL" : "SUSPECT", UL(h), UL(w), UL(need), (unsigned)c.ch[ch].verifyFails);
    }
  }
  g_probe.mode = 0;
}

// ---- health evaluation (called ~20 Hz) ----------------------------------------------------
static void irHealth(const Config& c, uint32_t nowMs) {
  uint8_t nT, nU;
  float gap = irCoverageGap(c, &nT, &nU);
  char ev[72];
  snprintf(ev, sizeof(ev), "no IR channel is enabled");
  faultUpdate(F_IR_NO_CHANNELS, irCountEnabled(c) == 0, c.recoverVerifyMs, ev);
  snprintf(ev, sizeof(ev), "no completed scan for %lu ms (cycles=%lu)", UL(elapsedMs(nowMs, g_scan.lastCycleMs)), UL(g_scan.cycleCount));
  faultUpdate(F_IR_SCAN_STALL, irCountEnabled(c) > 0 && elapsedMs(nowMs, g_scan.lastCycleMs) > 600, c.recoverVerifyMs, ev);
  snprintf(ev, sizeof(ev), "%u trusted receivers, minimum %u", (unsigned)nT, (unsigned)c.minTrusted);
  faultUpdate(F_IR_TRUSTED_LOW, nT < c.minTrusted, c.recoverVerifyMs, ev);
  snprintf(ev, sizeof(ev), "largest uncovered gap %.0f deg > %.0f deg (config bearings, assumed fov %.0f)", gap, c.maxGapDeg, c.fovHalfDeg);
  faultUpdate(F_IR_GAP, nT >= c.minTrusted && gap > c.maxGapDeg, c.recoverVerifyMs, ev);
  char list[72] = "";
  bool any = false;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) {
    ChStatus st = irChStatus(c, i);
    if (st == CH_SUSPECT_STUCK || st == CH_SUSPECT_NOISY || st == CH_FAIL_STUCK || st == CH_FAIL_TEST) {
      char one[24];
      snprintf(one, sizeof(one), "%sC%u:%s", any ? " " : "", (unsigned)i, CH_NAME[st]);
      if (strlen(list) + strlen(one) < sizeof(list)) strcat(list, one);
      any = true;
    }
  }
  faultUpdate(F_IR_CHANNEL, any, c.recoverVerifyMs, list);
}
