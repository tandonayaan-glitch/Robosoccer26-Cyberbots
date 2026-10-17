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

enum WinState : uint8_t { WS_NONE, WS_DETECT, WS_STUCK, WS_NOISY, WS_INVALID, WS_OFFBAND };
enum ChStatus : uint8_t {
  CH_DISABLED, CH_UNVERIFIED, CH_VERIFIED, CH_SUSPECT_STUCK, CH_SUSPECT_NOISY, CH_SUSPECT_TEST,
  CH_FAIL_STUCK, CH_FAIL_TEST
};
static const char* WS_NAME[] = {"none", "DETECT", "STUCK", "NOISY", "invalid", "offband"};
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
  uint8_t streak;         // consecutive DETECT windows ending with the latest visit (0 after any miss)
  uint32_t windows, detCount, overruns, offband;
  uint32_t lastWindowMs, lastDetectMs;
  uint32_t lastDetectUs;       // micros() at the close of the last DETECT window (latency instrumentation)
  uint32_t stuckSinceMs;
  uint8_t noisyScore, cleanStreak, stuckSuspect, stuckFail, noisy;
  // engine 2 evidence state (see irVisit2)
  float S;                     // leaky evidence counter: +1 per DETECT window, -IR2_MISS_DECAY per miss, 0 when idle too long
  float ha, hb;                // discounted hit / miss counts of the open episode (Beta posterior for the hit probability)
  uint16_t missRun;            // visits without a hit since the last hit
  uint32_t epStartUs;          // micros() of the first hit of the current evidence episode
  uint32_t lastHitUs, lastVisitUs, revisitUs;
};
static IrChan g_ir[MUX_CHANNELS];

// ---- edge counter (ISR) -----------------------------------------------------------
#define IR_EDGE_CAP 250
// The ISR must not call anything that lives in flash (digitalRead()/micros() are in flash on
// Arduino-ESP32: confirmed with nm on the real 2.0.17/3.0.7 builds). A GPIO32-39 input is read
// straight from the GPIO input register and time comes from esp_timer_get_time() (IRAM).
static_assert(PIN_MUX_SIG >= 32 && PIN_MUX_SIG <= 39, "ISR reads GPIO_IN1: SIG must be GPIO32..39");
#ifdef HOST_TEST
  #define IR_SIG_LEVEL() digitalRead(PIN_MUX_SIG)
  #define IR_NOW_US() micros()
#else
  #include <soc/gpio_struct.h>
  #include <esp_timer.h>
  #define IR_SIG_LEVEL() ((uint8_t)((GPIO.in1.val >> (PIN_MUX_SIG - 32)) & 1u))
  #define IR_NOW_US() ((uint32_t)esp_timer_get_time())
#endif
static volatile bool g_isrGate = false;
static volatile uint8_t g_isrActiveLevel = 0;
static volatile bool g_isrInActive = false;
static volatile uint16_t g_isrEdges = 0;
static volatile uint32_t g_isrActAcc = 0, g_isrActStart = 0, g_isrFirst = 0, g_isrLast = 0;

static void IRAM_ATTR irIsr() {
  if (!g_isrGate) return;
  bool isActive = (IR_SIG_LEVEL() == g_isrActiveLevel);
  uint32_t t = IR_NOW_US();
  if (isActive) {
    if (!g_isrInActive) {
      g_isrInActive = true;
      g_isrActStart = t;
      if (g_isrEdges == 0) g_isrFirst = t;
      g_isrLast = t;
      g_isrEdges = (uint16_t)(g_isrEdges + 1);
      if (g_isrEdges >= IR_EDGE_CAP) g_isrGate = false;   // flood guard: bounded interrupt load
    }
  } else if (g_isrInActive) {
    g_isrInActive = false;
    g_isrActAcc = g_isrActAcc + (t - g_isrActStart);
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
  uint32_t winOverrunMaxUs;                    // longest (actual - planned) window length seen: loop lateness
  uint32_t revisitMaxUs[MUX_CHANNELS];         // longest time between two window openings of the same receiver
  uint32_t lastOpenUs[MUX_CHANNELS];           // 0 = never opened since the schedule (re)started
  uint8_t ringPos;                             // last channel opened by the ring sweep (engine 2 interleaves focus windows)
  uint8_t ringRun;                             // ring windows since the last focus window
  uint8_t focusLast;                           // last channel opened as a focus window
  uint32_t focusWindows;                       // statistics: windows spent on focus
  uint32_t lastBlindUs;                        // how late the last window closed if it had to be discarded (0 otherwise)
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
  bool evOpen;                       // an evidence burst is in progress
  uint32_t firstEvidenceUs, lastEvidenceUs, acquiredUs, lostUs;   // micros() timestamps
  float fx, fy;
  float q;                           // engine 1: fraction of scan cycles that delivered track evidence (EWMA); misses are accounted
                                     // retroactively, only when evidence returns, so a current outage cannot lengthen its own grace
  uint32_t freshCycle;               // scan-cycle index of the newest evidence counted in q
  uint32_t lastGoodUs, lastUpdateUs;   // engine 1: newest track-consistent evidence, last estimate update
  float lostBearingDeg;              // engine 2: bearing of the track when it was lost (reference for fast re-acquisition)
  float trackFlick;                  // engine 2: measured miss fraction of that receiver (0 = steady, 1 = never)
  uint8_t trackM;                    // engine 2: misses in a row that the strongest supporting receiver would make less likely than 10^-lossEps
  uint32_t holdUs;                   // engine 2: grace period, fixed at the last good evidence (cannot grow during an outage)
  uint32_t revisitUs;                // engine 2: smoothed revisit interval of the supporting receivers
  uint8_t acqCount;
  bool lostEvent;         // set for one poll when the ball is lost (consumed by navigation)
};
static BallEst g_ball;

// Firmware-internal latency statistics (microseconds). acq = first DETECT window of the evidence burst ->
// ball confirmed. loss = last DETECT window -> ball declared lost. Same code runs on the robot, so the
// same numbers can be read there (excluding the time the ball itself needed to switch on/off).
struct LatStat { uint32_t n, minUs, maxUs, lastUs; uint64_t sumUs; };
static LatStat g_latAcq, g_latLoss;
static void latAdd(LatStat& s, uint32_t us) {
  if (!s.n || us < s.minUs) s.minUs = us;
  if (us > s.maxUs) s.maxUs = us;
  s.lastUs = us; s.sumUs += us; s.n++;
}
static void irNoteEvidence(uint32_t tUs) {
  if (!g_ball.evOpen) { g_ball.evOpen = true; g_ball.firstEvidenceUs = tUs; }
  g_ball.lastEvidenceUs = tUs;
}

// operator probes (":learn" coverage mapping, ":verify" controlled receiver test)
struct Probe {
  uint8_t mode;           // 0 none, 1 learn, 2 verify
  uint8_t ch;
  float deg;
  uint32_t endMs;
  uint32_t hits[MUX_CHANNELS], windows[MUX_CHANNELS], maxEdges[MUX_CHANNELS];
  uint32_t perSum, perN, perMin, perMax;   // pulse periods (us) of windows with >= 3 pulses
};
static Probe g_probe;
struct PeriodStat { uint32_t n, mean, minUs, maxUs; };
static PeriodStat g_lastPeriod;           // result of the last probe, input to ":irgate auto"
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
                                 bool startActive, bool endActive, uint32_t periodUs = 0) {
  if (edges == 0) {
    if (startActive && endActive && activeUs + windowUs / 20 >= windowUs) return WS_STUCK;
    return WS_NONE;
  }
  float windowMs = (float)windowUs / 1000.0f;
  if (edges >= IR_EDGE_CAP || (float)edges > (float)c.irNoisyPerMs * windowMs) return WS_NOISY;
  if (edges >= c.irMinEdges) {
    // Optional pulse-period gate: an unrelated 38-40 kHz source (remote, other robot, lamp driver)
    // produces a different envelope rhythm from the ball. Only usable once the ball's real period
    // has been measured (":verify"/":learn" then ":irgate auto").
    if (c.irPeriodMaxUs && edges >= 2 && (periodUs < c.irPeriodMinUs || periodUs > c.irPeriodMaxUs)) return WS_OFFBAND;
    return WS_DETECT;
  }
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
  if (g_scan.lastOpenUs[ch]) {
    uint32_t rv = t - g_scan.lastOpenUs[ch];
    if (rv > g_scan.revisitMaxUs[ch]) g_scan.revisitMaxUs[ch] = rv;
  }
  g_scan.lastOpenUs[ch] = t ? t : 1u;
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
  if (ws == WS_OFFBAND) s.offband++;
  s.detect = (ws == WS_DETECT);
  s.streak = s.detect ? (uint8_t)(s.streak < 255 ? s.streak + 1 : 255) : 0;
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
static void irVisit2(const Config& c, uint8_t ch, bool detect, uint32_t tUs);

static void irCloseWindow(Config& c, uint32_t nowMs) {
  g_isrGate = false;
  uint32_t t = micros();
  uint32_t actual = t - g_scan.winStartUs;
  uint8_t ch = g_scan.cur;
  if (actual > g_scan.lastWinPlannedUs && actual - g_scan.lastWinPlannedUs > g_scan.winOverrunMaxUs) g_scan.winOverrunMaxUs = actual - g_scan.lastWinPlannedUs;
  g_scan.lastBlindUs = 0;
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
  if (actual > g_scan.lastWinPlannedUs + g_scan.lastWinPlannedUs / 2 + 500) { ws = WS_INVALID; g_scan.lastBlindUs = actual - g_scan.lastWinPlannedUs; }   // loop was late
  else ws = irClassifyWindow(c, edges, acc, actual, startActive, endActive, s.periodUs);
  irChannelUpdate(c, ch, ws, nowMs);
  if (ws == WS_DETECT) { s.lastDetectUs = t; if (irTrusted(c, ch)) irNoteEvidence(t); }
  if (c.engine == 2 && ws != WS_INVALID) irVisit2(c, ch, ws == WS_DETECT && irTrusted(c, ch), t);
  if (g_probe.mode && ws != WS_INVALID) {
    g_probe.windows[ch]++;
    if (ws == WS_DETECT) g_probe.hits[ch]++;
    if (edges > g_probe.maxEdges[ch]) g_probe.maxEdges[ch] = edges;
    if ((ws == WS_DETECT || ws == WS_OFFBAND) && edges >= 3 && s.periodUs) {
      g_probe.perSum += s.periodUs; g_probe.perN++;
      if (!g_probe.perMin || s.periodUs < g_probe.perMin) g_probe.perMin = s.periodUs;
      if (s.periodUs > g_probe.perMax) g_probe.perMax = s.periodUs;
    }
  }
  g_scan.open = false;
}

static void irEstimate(const Config& c, uint32_t nowMs) {
  float sx = 0, sy = 0, W = 0;
  uint8_t n = 0;
  uint16_t mask = 0;
  bool unverified = false;
  bool recentHit = false;     // a contributing channel detected in one of its last 2 windows (~72 ms)
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
    if (!irTrusted(c, ch) || !g_ir[ch].active) continue;
    if (g_ir[ch].hist & 0x3) recentHit = true;
    float w = clampf((float)popcount16(g_ir[ch].hist & 0xFF) / (float)c.irFullHits, 0.0f, 1.0f);
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
  // Freshness comes only from RECENT windows. Otherwise the 8-window history that smooths the estimate
  // would keep refreshing the timestamps for ~0.3 s after the ball has gone and delay the loss decision.
  if (recentHit) {
    g_ball.lastDetectMs = nowMs;
    if (g_ball.conf >= c.acqConf * 0.6f) g_ball.lastGoodMs = nowMs;
  }
  if (g_ball.conf >= c.acqConf) { if (g_ball.acqCount < 255) g_ball.acqCount++; } else g_ball.acqCount = 0;
  if (!g_ball.valid && g_ball.acqCount >= c.acqCycles) {
    g_ball.valid = true; g_ball.validSinceMs = nowMs;
    g_ball.acquiredUs = micros();
    if (g_ball.evOpen) latAdd(g_latAcq, g_ball.acquiredUs - g_ball.firstEvidenceUs);
  }
}


// ---- engine 1: recency / streak evidence ---------------------------------------------------------------
// Evidence unit: a receiver whose latest visits were all hits contributes min(1, streak/acqStreak). The ball is
// acquired when the evidence summed over receivers reaches 1 and the receivers agree on a direction, so two adjacent
// receivers hitting in the same scan confirm it after ONE cycle, while a lone receiver needs acqStreak consecutive
// hits (which is what rejects short single-receiver bursts). After acquisition only evidence consistent with the track
// (within gateDeg of the bearing) can refresh it, so stale or unrelated signals cannot keep the ball alive.
static float g_chSin[MUX_CHANNELS], g_chCos[MUX_CHANNELS], g_chBear[MUX_CHANNELS];
static void irChTrig(const Config& c, uint8_t ch, float* sn, float* cs) {
  if (g_chBear[ch] != c.ch[ch].bearingDeg || (g_chSin[ch] == 0 && g_chCos[ch] == 0)) {
    g_chBear[ch] = c.ch[ch].bearingDeg;
    g_chSin[ch] = sinf(c.ch[ch].bearingDeg * DEG2RAD);
    g_chCos[ch] = cosf(c.ch[ch].bearingDeg * DEG2RAD);
  }
  *sn = g_chSin[ch]; *cs = g_chCos[ch];
}

// Grace period before a ball is declared lost. Engine 1: short for a strong signal (hit on nearly every visit),
// growing to lostHoldMaxMs when the signal is intermittent, so marginal-range balls are not dropped on a run of misses.
static uint32_t irHoldUs2(const Config& c);
static uint32_t irEffectiveHoldMs(const Config& c) {
  if (c.engine == 2) return g_ball.holdUs ? g_ball.holdUs / 1000u : c.lostHoldMs;
  if (!c.engine || c.lostHoldMaxMs <= c.lostHoldMs) return c.lostHoldMs;
  float k = clampf((0.98f - g_ball.q) / 0.2f, 0.0f, 1.0f);
  return (uint32_t)((float)c.lostHoldMs + k * (float)(c.lostHoldMaxMs - c.lostHoldMs));
}

static void irEstimateFast(const Config& c, uint32_t nowUs, uint32_t nowMs) {
  const uint16_t dmask = (uint16_t)((1u << c.dirWindows) - 1u);
  const bool track = g_ball.valid;
  float ax = 0, ay = 0, aW = 0, aE = 0, aEc = 0;       // all live receivers
  float gx = 0, gy = 0, gW = 0, gE = 0, gEc = 0;       // live receivers consistent with the track
  uint8_t aN = 0, gN = 0, aMaxStreak = 0; uint16_t aMask = 0, gMask = 0;
  bool aUnver = false, gUnver = false, gFresh = false;
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
    if (!irTrusted(c, ch)) continue;
    const IrChan& s = g_ir[ch];
    int h = popcount16(s.hist & dmask);
    if (!h) continue;
    float w = (float)h / (float)c.dirWindows, sn, cs;
    irChTrig(c, ch, &sn, &cs);
    float ev = s.streak ? (s.streak >= c.acqStreak ? 1.0f : (float)s.streak / (float)c.acqStreak) : 0.0f;
    float evc = s.streak ? (s.streak >= 4 ? 1.0f : (float)s.streak / 4.0f) : 0.0f;
    if (s.streak > aMaxStreak) aMaxStreak = s.streak;
    ax += w * sn; ay += w * cs; aW += w; aE += ev; aEc += evc; aN++; aMask |= (uint16_t)(1u << ch);
    if (!c.ch[ch].verified) aUnver = true;
    bool inGate = !track || absf(angDiffDeg(c.ch[ch].bearingDeg, g_ball.bearingDeg)) <= (float)c.gateDeg;
    if (inGate) {
      gx += w * sn; gy += w * cs; gW += w; gE += ev; gEc += evc; gN++; gMask |= (uint16_t)(1u << ch);
      if (!c.ch[ch].verified) gUnver = true;
      if (s.streak) gFresh = true;
    }
  }
  g_ball.cycleStamp = g_scan.cycleCount;
  float dtUs = (float)(uint32_t)(nowUs - g_ball.lastUpdateUs);
  g_ball.lastUpdateUs = nowUs;
  // Choose the evidence set: the gated one while a track exists and has support, otherwise everything.
  bool useGate = track && gW > 0;
  float sx = useGate ? gx : ax, sy = useGate ? gy : ay, W = useGate ? gW : aW, Ec = useGate ? gEc : aEc, E = useGate ? gE : aE;
  uint8_t n = useGate ? gN : aN; uint16_t mask = useGate ? gMask : aMask; bool unver = useGate ? gUnver : aUnver;
  g_ball.nActive = n; g_ball.actMask = mask; g_ball.W = W; g_ball.anyDetect = n > 0;
  if (n == 0 || W <= 0) { g_ball.rInst = 0; if (!track) g_ball.conf = 0; return; }
  float ux = sx / W, uy = sy / W;
  float r = clampf(sqrtf(ux * ux + uy * uy), 0, 1);
  g_ball.rInst = r;
  g_ball.ambiguous = r < 0.5f;
  bool retarget = track && !useGate && aE >= 1.0f && !(r < 0.5f);   // track has no support, a new cluster has full evidence
  if (!track || retarget) {
    // provisional (or re-targeted) direction: take the evidence as-is, no smoothing against an obsolete value
    g_ball.fx = ux; g_ball.fy = uy;
  } else {
    float a = clampf(dtUs / ((float)c.dirTauMs * 1000.0f), 0.0f, 1.0f);
    g_ball.fx += a * (ux - g_ball.fx); g_ball.fy += a * (uy - g_ball.fy);
  }
  float rf = clampf(sqrtf(g_ball.fx * g_ball.fx + g_ball.fy * g_ball.fy), 0, 1);
  g_ball.bearingDeg = wrapDeg180(atan2f(g_ball.fx, g_ball.fy) * RAD2DEG);
  g_ball.conf = rf * (Ec < 1.0f ? Ec : 1.0f) * (unver ? 0.85f : 1.0f);   // heuristic, see HEALTH_AND_LOGGING.md
  if (!track) {
    // aMaxStreak >= acqMinStreak: the scan visits receivers in bearing order, so until one receiver has been hit twice in
    // a row the other receivers have not been re-checked since the onset; a broadband onset would look like a ball
    // seen by a few adjacent receivers. After one revisit the whole ring has been sampled and ambiguity is judged on it.
    if (E >= 1.0f && !g_ball.ambiguous && aMaxStreak >= c.acqMinStreak) {
      g_ball.valid = true; g_ball.validSinceMs = nowMs; g_ball.acquiredUs = nowUs; g_ball.q = 1.0f; g_ball.freshCycle = g_scan.cycleCount;
      g_ball.lastGoodUs = nowUs; g_ball.lastDetectMs = nowMs; g_ball.lastGoodMs = nowMs;
      if (g_ball.evOpen) latAdd(g_latAcq, nowUs - g_ball.firstEvidenceUs);
    }
  } else if (gFresh || retarget) {
    g_ball.lastGoodUs = nowUs; g_ball.lastDetectMs = nowMs; g_ball.lastGoodMs = nowMs;
    if (g_scan.cycleCount != g_ball.freshCycle) {         // first evidence in this scan cycle: update the quality estimate
      uint32_t missed = g_scan.cycleCount - g_ball.freshCycle - 1u;
      if (missed > 12u) missed = 12u;
      for (uint32_t m = 0; m < missed; m++) g_ball.q *= (1.0f - 1.0f / 10.0f);
      g_ball.q += (1.0f / 10.0f) * (1.0f - g_ball.q);
      g_ball.freshCycle = g_scan.cycleCount;
    }
  }
}

// ---- engine 2: leaky evidence, focused revisits, visit-statistics loss -----------------------------------------------
// Evidence per receiver is a leaky counter S (+1 per DETECT window, -IR2_MISS_DECAY per miss, cleared after epIdleMs
// without a hit), so an intermittent signal still accumulates while a ball-like burst is bounded by TIME: a ball is
// acquired only when the evidence has lasted acqSpan1Ms (one receiver) / acqSpan2Ms (two or more neighbouring
// receivers) AND the rest of the ring has been looked at since the onset (a broadband onset hits the receivers in scan
// order, so it looks like a local source until the ring has been revisited) AND the receivers agree on a direction.
// While a candidate or a weak track exists the scheduler spends every second window on the receivers that matter
// ("focus"), so a weak signal is sampled several times faster than the ring revisit interval.
// Loss: the grace period is the time in which the supporting receivers are visited often enough that a run of misses
// at their measured hit probability would be rarer than 10^-lossEps; it is fixed at the last good evidence, so an
// outage cannot lengthen its own grace period, and is bounded by [lostHoldMs, lostHoldMaxMs].
static const float IR2_MISS_DECAY = 0.1f;
static const float IR2_DISCOUNT = 0.97f;                 // per-visit forgetting of the hit/miss counts (about 33 visits of memory)
static uint16_t g_focusMask = 0;

static void irVisit2(const Config& c, uint8_t ch, bool detect, uint32_t tUs) {
  IrChan& s = g_ir[ch];
  s.revisitUs = s.lastVisitUs ? (uint32_t)(tUs - s.lastVisitUs) : 0;
  s.lastVisitUs = tUs ? tUs : 1u;
  if (detect) {
    if (s.S < 0.5f) {                                   // no supporting recent evidence: a new episode starts
      s.S = 0.0f; s.epStartUs = tUs; s.ha = 0.0f; s.hb = 0.0f; s.missRun = 0;
    }
    s.ha = s.ha * IR2_DISCOUNT + 1.0f; s.hb *= IR2_DISCOUNT;
    s.missRun = 0;
    s.S = s.S + 1.0f > 8.0f ? 8.0f : s.S + 1.0f;
    s.lastHitUs = tUs;
  } else {
    if (s.missRun < 65535) s.missRun++;
    if (s.S > 0.0f) {
      s.ha *= IR2_DISCOUNT; s.hb = s.hb * IR2_DISCOUNT + 1.0f;
      s.S -= IR2_MISS_DECAY;
      if (s.S < 0.05f || (uint32_t)(tUs - s.lastHitUs) > (uint32_t)c.epIdleMs * 1000u) { s.S = 0.0f; s.ha = 0.0f; s.hb = 0.0f; }
    }
  }
}

// Smallest run of misses whose probability, under the Beta(a+1, b+1) predictive distribution of the hit probability, is
// below 10^-epsExp. The predictive form keeps the uncertainty of a young or small sample (a lucky streak of hits is not
// proof of a strong signal), and it is evaluated only at hits, so an outage cannot raise it.
static uint8_t irMissBudget(float a, float b, uint8_t epsExp) {
  float lim = 1.0f;
  for (uint8_t i = 0; i < epsExp; i++) lim *= 0.1f;
  float p = 1.0f;
  for (uint8_t r = 1; r <= 80; r++) {
    p *= (b + (float)r) / (a + b + 1.0f + (float)r);
    if (p < lim) return r;
  }
  return 80;
}
static inline float irHitProb(const IrChan& s) { return (s.ha + 1.0f) / (s.ha + s.hb + 2.0f); }

static uint32_t irHoldUs2(const Config& c) {
  float rv = (float)g_ball.revisitUs;
  if (rv < 3000.0f) rv = 3000.0f;
  // A steady signal that stops is almost certainly gone: short grace (lostHoldMs). A signal that was already flickering may
  // simply be in a long gap, so its grace grows towards flickerHoldMs as the measured miss fraction goes from 5 % to 25 %.
  float fl = clampf((g_ball.trackFlick - 0.05f) / 0.20f, 0.0f, 1.0f);
  float floorUs = (float)c.lostHoldMs * 1000.0f + fl * (float)(c.flickerHoldMs - c.lostHoldMs) * 1000.0f;
  return (uint32_t)clampf((float)g_ball.trackM * rv, floorUs, (float)c.lostHoldMaxMs * 1000.0f);
}

// Which receivers deserve extra visits right now (bit mask).
static void irFocus2(const Config& c, uint32_t nowUs) {
  g_focusMask = 0;
  if (!c.focusOn) return;
  uint8_t want, picked = 0;
  if (g_ball.valid) {
    // strong track: the ring revisit already gives enough visits inside the minimum grace period
    if ((uint32_t)g_ball.trackM * (uint32_t)g_scan.enabledCount * (uint32_t)(c.irWindowUs + c.irSettleUs) <= (uint32_t)c.lostHoldMs * 1000u) return;
    want = 3;
  } else want = 2;
  for (uint8_t round = 0; round < want; round++) {
    int best = -1; uint32_t bestKey = 0;
    for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
      if ((g_focusMask >> ch) & 1u || !irTrusted(c, ch)) continue;
      const IrChan& s = g_ir[ch];
      if (s.S <= 0.0f) continue;
      uint32_t age = nowUs - s.lastHitUs;
      if (g_ball.valid) {
        if (age > 1000000u || irHitProb(s) < 0.1f) continue;
        if (absf(angDiffDeg(c.ch[ch].bearingDeg, g_ball.bearingDeg)) > (float)c.gateDeg) continue;
      } else if (s.missRun < 1 || s.missRun > 4) continue;   // only a candidate that has just MISSED needs probing: a steady one is confirmed by time alone
      uint32_t key = 0xFFFFFFFFu - age;                  // newest hit first
      if (best < 0 || key > bestKey) { best = ch; bestKey = key; }
    }
    if (best < 0) break;
    g_focusMask |= (uint16_t)(1u << best); picked++;
  }
}

// Miss budget and flicker measure of the strongest receiver among those in `mask` (evaluated only at hits / acquisition).
static void irSupportStats(const Config& c, uint16_t mask, uint8_t* m, float* flick) {
  uint8_t best = 80; float fl = 1.0f;
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
    if (!((mask >> ch) & 1u)) continue;
    const IrChan& s = g_ir[ch];
    uint8_t k = irMissBudget(s.ha, s.hb, c.lossEpsExp);
    if (k < best) { best = k; fl = s.hb / (s.ha + s.hb + 0.001f); }
  }
  *m = best; *flick = fl;
}

static void irEstimate2(const Config& c, uint32_t nowUs, uint32_t nowMs, uint8_t closedCh, bool hit, uint32_t blindUs) {
  const bool track = g_ball.valid;
  // Absence has to be OBSERVED: time in which the loop was stalled (window closed far too late, so it was discarded) says
  // nothing about the ball, so it is not counted against the track. Bounded, so a permanent stall still ends the track.
  if (track && blindUs) {
    uint32_t credit = blindUs > 500000u ? 500000u : blindUs;
    uint32_t room = nowUs - g_ball.lastGoodUs;
    g_ball.lastGoodUs += credit < room ? credit : room;
  }
  const uint32_t ringUs = (uint32_t)g_scan.enabledCount * (uint32_t)(c.irWindowUs + c.irSettleUs);
  const uint32_t dirSpanUs = (uint32_t)c.dirWindows * ringUs;
  float w[MUX_CHANNELS];
  float ax = 0, ay = 0, aW = 0;
  uint8_t aN = 0; int seed = -1; float seedS = 0; uint32_t seedHit = 0;
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
    w[ch] = 0;
    if (!irTrusted(c, ch)) continue;
    const IrChan& s = g_ir[ch];
    if (s.S <= 0.0f) continue;
    uint32_t age = nowUs - s.lastHitUs;
    if (age > dirSpanUs) continue;
    float wt = (1.0f - (float)age / (float)dirSpanUs) * (0.25f + 0.75f * irHitProb(s));
    if (wt <= 0.0f) continue;
    w[ch] = wt;
    float sn, cs; irChTrig(c, ch, &sn, &cs);
    ax += wt * sn; ay += wt * cs; aW += wt; aN++;
    if (s.S > seedS || (s.S == seedS && (int32_t)(s.lastHitUs - seedHit) > 0)) { seed = ch; seedS = s.S; seedHit = s.lastHitUs; }
  }
  if (aN == 0 || (!track && seed < 0)) {
    g_ball.nActive = 0; g_ball.actMask = 0; g_ball.W = 0; g_ball.anyDetect = false; g_ball.rInst = 0;
    if (!track) g_ball.conf = 0;
    g_ball.cycleStamp = g_scan.cycleCount;
    irFocus2(c, nowUs);
    return;
  }
  const float refDeg = track ? g_ball.bearingDeg : c.ch[seed].bearingDeg;
  float gx = 0, gy = 0, gW = 0, Ec = 0;
  uint8_t gN = 0; uint16_t gMask = 0; bool gUnver = false, haveOnset = false; uint32_t onset = 0;
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
    if (w[ch] <= 0.0f) continue;
    if (absf(angDiffDeg(c.ch[ch].bearingDeg, refDeg)) > (float)c.gateDeg) continue;
    const IrChan& s = g_ir[ch];
    float sn, cs; irChTrig(c, ch, &sn, &cs);
    gx += w[ch] * sn; gy += w[ch] * cs; gW += w[ch]; gN++; gMask |= (uint16_t)(1u << ch);
    Ec += s.S >= (float)c.acqStreak ? 1.0f : s.S / (float)c.acqStreak;
    if (!c.ch[ch].verified) gUnver = true;
    if (!haveOnset || (int32_t)(s.epStartUs - onset) < 0) { onset = s.epStartUs; haveOnset = true; }
  }
  const float rAll = aW > 0 ? clampf(sqrtf(ax * ax + ay * ay) / aW, 0.0f, 1.0f) : 0.0f;
  const bool ambiguous = rAll < 0.5f;
  // Acceptance must also see receivers whose evidence is no longer FRESH but whose episode is still open: after a
  // broadband burst the receivers that were hit first age out first, and the last ones visited would otherwise look like a
  // large local source. Open episodes (S > 0, any age up to epIdleMs) are therefore weighed regardless of recency.
  float ex = 0, ey = 0, eW = 0;
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
    if (!irTrusted(c, ch) || g_ir[ch].S <= 0.0f) continue;
    float e = g_ir[ch].S >= (float)c.acqStreak ? 1.0f : g_ir[ch].S / (float)c.acqStreak, sn, cs;
    irChTrig(c, ch, &sn, &cs);
    ex += e * sn; ey += e * cs; eW += e;
  }
  const bool episodeAmbiguous = eW > 0.0f && sqrtf(ex * ex + ey * ey) / eW < 0.5f;
  const bool closedInGate = hit && absf(angDiffDeg(c.ch[closedCh].bearingDeg, refDeg)) <= (float)c.gateDeg;

  // acceptance test shared by acquisition, re-acquisition and re-targeting
  bool accept = false;
  if (gN > 0 && haveOnset) {
    float ux = gx / gW, uy = gy / gW;
    float cbear = wrapDeg180(atan2f(ux, uy) * RAD2DEG);
    bool reacq = !track && c.reacqMs && g_ball.lostUs && (uint32_t)(nowUs - g_ball.lostUs) < (uint32_t)c.reacqMs * 1000u &&
                 absf(angDiffDeg(cbear, g_ball.lostBearingDeg)) <= 45.0f;
    uint32_t need = reacq ? 0u : (uint32_t)(gN >= 2 ? c.acqSpan2Ms : c.acqSpan1Ms) * 1000u;
    bool ringChecked = true;
    for (uint8_t ch = 0; ch < MUX_CHANNELS && ringChecked; ch++)
      if (c.ch[ch].enabled && irTrusted(c, ch) && (int32_t)(g_ir[ch].lastVisitUs - onset) <= 0) ringChecked = false;
    accept = Ec >= (reacq ? 0.5f : 1.0f) && (uint32_t)(nowUs - onset) >= need && ringChecked && !ambiguous && !episodeAmbiguous;
  }

  if (!track) {
    g_ball.nActive = gN; g_ball.actMask = gMask; g_ball.W = gW; g_ball.anyDetect = gN > 0;
    if (gN > 0) {
      float ux = gx / gW, uy = gy / gW;
      g_ball.rInst = rAll;                                // agreement of ALL fresh receivers, not just the cluster
      g_ball.ambiguous = ambiguous || episodeAmbiguous;
      g_ball.fx = ux; g_ball.fy = uy;
      g_ball.bearingDeg = wrapDeg180(atan2f(ux, uy) * RAD2DEG);
      g_ball.conf = g_ball.rInst * (Ec < 1.0f ? Ec : 1.0f) * (gUnver ? 0.85f : 1.0f);
    }
    if (accept) {
      g_ball.valid = true; g_ball.validSinceMs = nowMs; g_ball.acquiredUs = nowUs;
      g_ball.lastGoodUs = nowUs; g_ball.lastDetectMs = nowMs; g_ball.lastGoodMs = nowMs; g_ball.lastUpdateUs = nowUs;
      irSupportStats(c, gMask, &g_ball.trackM, &g_ball.trackFlick); g_ball.revisitUs = ringUs; g_ball.holdUs = irHoldUs2(c);
      if (g_ball.evOpen) latAdd(g_latAcq, nowUs - g_ball.firstEvidenceUs);
    }
    irFocus2(c, nowUs);
    return;
  }

  // ---- a track exists ----
  float dtUs = (float)(uint32_t)(nowUs - g_ball.lastUpdateUs);
  g_ball.lastUpdateUs = nowUs;
  if (gN == 0) {
    // nothing consistent with the track is fresh any more. A different full-strength cluster takes over (re-target);
    // otherwise the track just ages until its grace period ends.
    g_ball.nActive = 0; g_ball.actMask = 0; g_ball.W = 0; g_ball.anyDetect = false; g_ball.rInst = 0;
    if (seed >= 0) {
      // evaluate the strongest foreign cluster around the seed
      float fx2 = 0, fy2 = 0, fW = 0, fEc = 0; uint8_t fN = 0; uint16_t fMask = 0; bool have = false; uint32_t on2 = 0;
      for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
        if (w[ch] <= 0.0f || absf(angDiffDeg(c.ch[ch].bearingDeg, c.ch[seed].bearingDeg)) > (float)c.gateDeg) continue;
        const IrChan& s = g_ir[ch]; float sn, cs; irChTrig(c, ch, &sn, &cs);
        fx2 += w[ch] * sn; fy2 += w[ch] * cs; fW += w[ch]; fN++;
        fEc += s.S >= (float)c.acqStreak ? 1.0f : s.S / (float)c.acqStreak;
        if (!have || (int32_t)(s.epStartUs - on2) < 0) { on2 = s.epStartUs; have = true; }
        fMask |= (uint16_t)(1u << ch);
      }
      bool ringOk = true;
      for (uint8_t ch = 0; ch < MUX_CHANNELS && ringOk; ch++)
        if (c.ch[ch].enabled && irTrusted(c, ch) && (int32_t)(g_ir[ch].lastVisitUs - on2) <= 0) ringOk = false;
      uint32_t need = (uint32_t)(fN >= 2 ? c.acqSpan2Ms : c.acqSpan1Ms) * 1000u;
      if (have && fEc >= 1.0f && (uint32_t)(nowUs - on2) >= need && ringOk && !ambiguous && !episodeAmbiguous) {
        float ux = fx2 / fW, uy = fy2 / fW;
        g_ball.fx = ux; g_ball.fy = uy; g_ball.bearingDeg = wrapDeg180(atan2f(ux, uy) * RAD2DEG);
        g_ball.lastGoodUs = nowUs; g_ball.lastDetectMs = nowMs; g_ball.lastGoodMs = nowMs;
        irSupportStats(c, fMask, &g_ball.trackM, &g_ball.trackFlick); g_ball.holdUs = irHoldUs2(c);
        g_ball.rInst = clampf(sqrtf(ux * ux + uy * uy), 0.0f, 1.0f); g_ball.conf = g_ball.rInst * (fEc < 1.0f ? fEc : 1.0f);
      }
    }
    irFocus2(c, nowUs);
    return;
  }
  g_ball.nActive = gN; g_ball.actMask = gMask; g_ball.W = gW; g_ball.anyDetect = true;
  float ux = gx / gW, uy = gy / gW;
  g_ball.rInst = clampf(sqrtf(ux * ux + uy * uy), 0.0f, 1.0f);
  g_ball.ambiguous = ambiguous;
  float a = clampf(dtUs / ((float)c.dirTauMs * 1000.0f), 0.0f, 1.0f);
  g_ball.fx += a * (ux - g_ball.fx); g_ball.fy += a * (uy - g_ball.fy);
  float rf = clampf(sqrtf(g_ball.fx * g_ball.fx + g_ball.fy * g_ball.fy), 0.0f, 1.0f);
  g_ball.bearingDeg = wrapDeg180(atan2f(g_ball.fx, g_ball.fy) * RAD2DEG);
  g_ball.conf = rf * (Ec < 1.0f ? Ec : 1.0f) * (gUnver ? 0.85f : 1.0f);
  if (closedInGate) {
    g_ball.lastGoodUs = nowUs; g_ball.lastDetectMs = nowMs; g_ball.lastGoodMs = nowMs;
    irSupportStats(c, gMask, &g_ball.trackM, &g_ball.trackFlick);
    uint32_t rv = g_ir[closedCh].revisitUs;
    if (rv) g_ball.revisitUs = g_ball.revisitUs + ((int32_t)(rv - g_ball.revisitUs)) / 4;
    g_ball.holdUs = irHoldUs2(c);
  }
  irFocus2(c, nowUs);
}

static void irBallTimeout(const Config& c, uint32_t nowMs) {
  if (g_ball.evOpen && !g_ball.valid && (uint32_t)(micros() - g_ball.lastEvidenceUs) > 300000u) g_ball.evOpen = false;
  bool expired = c.engine == 2 ? ((uint32_t)(micros() - g_ball.lastGoodUs) > (g_ball.holdUs ? g_ball.holdUs : (uint32_t)c.lostHoldMs * 1000u))
               : c.engine ? ((uint32_t)(micros() - g_ball.lastGoodUs) > irEffectiveHoldMs(c) * 1000u)
                          : (elapsedMs(nowMs, g_ball.lastGoodMs) > c.lostHoldMs);
  if (g_ball.valid && expired) {
    g_ball.lostUs = micros();
    g_ball.lostBearingDeg = g_ball.bearingDeg;
    latAdd(g_latLoss, g_ball.lostUs - (c.engine ? g_ball.lastGoodUs : g_ball.lastEvidenceUs));
    g_ball.evOpen = false;
    g_ball.valid = false;
    g_ball.lostAtMs = nowMs;
    g_ball.lostEvent = true;
    g_ball.conf = 0;
    g_ball.acqCount = 0;
    if (c.engine == 2)                                   // the old evidence must not pre-load the next acquisition
      for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) g_ir[ch].S = 0.0f;
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

// Restart the scan schedule (drops any half-finished window) and clear the cycle statistics. Called when
// boot-time blocking work (report printing, calibration) is over so it cannot pollute steady-state numbers.
static void irRestartSchedule() {
  g_isrGate = false;
  g_scan.open = false;
  g_scan.cycleStartUs = micros();
  g_scan.cycleMinUs = 0xFFFFFFFFu; g_scan.cycleMaxUs = 0; g_scan.cycleAvgUs = 0; g_scan.cycleCount = 0;
  g_scan.winOverrunMaxUs = 0; g_focusMask = 0; g_scan.focusWindows = 0;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) { g_scan.revisitMaxUs[i] = 0; g_scan.lastOpenUs[i] = 0; }
  g_scan.lastCycleMs = millis();
}

// Pick the next window. Ring windows sweep the enabled receivers in order; with engine 2 every second window goes to
// the focus receivers instead (when there are any), so the ring revisit interval at most doubles while a candidate or a
// weak track is being watched. Strong tracks and idle scanning use the plain ring.
static uint8_t irPickNext(const Config& c, bool* ringWrapped, bool* focus) {
  *focus = false; *ringWrapped = false;
  if (c.engine == 2 && g_focusMask && g_scan.ringRun >= 1) {
    for (uint8_t k = 1; k <= MUX_CHANNELS; k++) {
      uint8_t i = (uint8_t)((g_scan.focusLast + k) % MUX_CHANNELS);
      if (((g_focusMask >> i) & 1u) && c.ch[i].enabled) {
        g_scan.focusLast = i; g_scan.ringRun = 0; g_scan.focusWindows++; *focus = true;
        return i;
      }
    }
  }
  uint8_t next = irNextEnabled(c, g_scan.ringPos, ringWrapped);
  g_scan.ringPos = next;
  if (g_scan.ringRun < 255) g_scan.ringRun++;
  return next;
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
    g_scan.ringPos = first; g_scan.ringRun = 0;
    irOpenWindow(c, first);
  } else if ((uint32_t)(nowUs - g_scan.winStartUs) >= c.irWindowUs) {
    uint8_t closed = g_scan.cur;
    irCloseWindow(c, nowMs);
    if (c.engine == 1) irEstimateFast(c, micros(), nowMs);   // engine 1: every closed window updates the estimate
    else if (c.engine == 2) irEstimate2(c, micros(), nowMs, closed, g_ir[closed].winState == WS_DETECT && irTrusted(c, closed), g_scan.lastBlindUs);
    bool wrapped, focus;
    uint8_t next = irPickNext(c, &wrapped, &focus);
    if (wrapped) {                                           // the ring sweep has come round: one cycle
      uint32_t endUs = micros();
      g_scan.cycleUs = endUs - g_scan.cycleStartUs;
      if (g_scan.cycleUs < g_scan.cycleMinUs) g_scan.cycleMinUs = g_scan.cycleUs;
      if (g_scan.cycleUs > g_scan.cycleMaxUs) g_scan.cycleMaxUs = g_scan.cycleUs;
      g_scan.cycleAvgUs = g_scan.cycleCount ? g_scan.cycleAvgUs * 0.9f + (float)g_scan.cycleUs * 0.1f : (float)g_scan.cycleUs;
      g_scan.cycleCount++;
      g_scan.lastCycleMs = nowMs;
      g_scan.cycleStartUs = endUs;
      if (c.engine == 0) irEstimate(c, nowMs);                // engine 0 (legacy): once per scan cycle
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
  g_lastPeriod.n = g_probe.perN;
  g_lastPeriod.mean = g_probe.perN ? g_probe.perSum / g_probe.perN : 0;
  g_lastPeriod.minUs = g_probe.perMin; g_lastPeriod.maxUs = g_probe.perMax;
  if (g_probe.perN) say("TEST", "measured pulse period of this source: mean %lu us (%.0f Hz), min %lu, max %lu over %lu windows (use :irgate auto to gate on it)",
                        UL(g_lastPeriod.mean), 1e6f / (float)g_lastPeriod.mean, UL(g_lastPeriod.minUs), UL(g_lastPeriod.maxUs), UL(g_lastPeriod.n));
  else say("TEST", "no window had >= 3 pulses: pulse period could not be measured");
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
