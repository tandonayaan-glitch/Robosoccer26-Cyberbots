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

// Pulse statistics for ":irchar" (pulse width and gap histograms). Streamed from the ISR only while g_isrChar is set, so
// normal operation pays one flag test per edge.
#define CH_WBINS 24              // pulse width histogram: 50 us bins, last bin = overflow
#define CH_GBINS 32              // gap histogram: 100 us bins, last bin = overflow
static volatile bool g_isrChar = false;
static volatile bool g_chTrunc = false;           // the first pulse of this window began before it opened: its width is unknown
static volatile uint32_t g_chLastEnd = 0;         // when the previous pulse of this window ended (0 = none yet)
static volatile uint32_t g_chWMin, g_chWMax, g_chWSum, g_chWN, g_chGMin, g_chGMax, g_chGSum, g_chGN;
static volatile uint32_t g_chW[CH_WBINS], g_chG[CH_GBINS];
#define CH_STARTS 8
static volatile uint32_t g_chT[CH_STARTS];        // start time of the first CH_STARTS pulses of the window

static void IRAM_ATTR irCharGap(uint32_t g) {
  if (g < g_chGMin) g_chGMin = g;
  if (g > g_chGMax) g_chGMax = g;
  g_chGSum = g_chGSum + g; g_chGN = g_chGN + 1;
  uint32_t b = g / 100u; if (b >= CH_GBINS) b = CH_GBINS - 1;
  g_chG[b] = g_chG[b] + 1;
}
static void IRAM_ATTR irCharWidth(uint32_t w) {
  if (w < g_chWMin) g_chWMin = w;
  if (w > g_chWMax) g_chWMax = w;
  g_chWSum = g_chWSum + w; g_chWN = g_chWN + 1;
  uint32_t b = w / 50u; if (b >= CH_WBINS) b = CH_WBINS - 1;
  g_chW[b] = g_chW[b] + 1;
}

static void IRAM_ATTR irIsr() {
  if (!g_isrGate) return;
  bool isActive = (IR_SIG_LEVEL() == g_isrActiveLevel);
  uint32_t t = IR_NOW_US();
  if (isActive) {
    if (!g_isrInActive) {
      if (g_isrChar) {
        if (g_chLastEnd) irCharGap(t - g_chLastEnd);
        if (g_isrEdges < CH_STARTS) g_chT[g_isrEdges] = t;
      }
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
    if (g_isrChar) {
      if (g_chTrunc) g_chTrunc = false; else irCharWidth(t - g_isrActStart);
      g_chLastEnd = t;
    }
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
  uint32_t lastGoodUs, lastUpdateUs;   // engine 2: newest track-consistent evidence, last estimate update
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

// operator probes (":learn" coverage mapping, ":verify" controlled receiver test, ":irchar" pulse characterisation)
struct Probe {
  uint8_t mode;           // 0 none, 1 learn, 2 verify, 3 irchar
  uint8_t ch;
  float deg;
  uint32_t endMs;
  uint32_t hits[MUX_CHANNELS], windows[MUX_CHANNELS], maxEdges[MUX_CHANNELS];
  uint32_t perSum, perN, perMin, perMax;   // pulse periods (us) of windows with >= 3 pulses
};
static Probe g_probe;
// ":irchar": one receiver watched with back-to-back windows, every window classified exactly as in normal operation.
#define CH_PBINS 32              // window-period histogram: 100 us bins, last bin = overflow
#define CH_IBINS 160             // pulse-interval histogram: 20 us bins up to 3.2 ms
struct IrCharStat {
  uint32_t startUs;
  uint32_t windows, invalid, cls[6];             // cls[] indexed by WinState
  uint32_t edgeSum, edgeHist[6];                 // edges per window: 0..4, 5+
  uint32_t per[CH_PBINS], perN, perSum;          // mean pulse period of windows with >= 2 pulses (the statistic the gate acts on)
  uint32_t run, longestRun, runs[7];             // consecutive windows without a (pre-gate) detection: 1,2,3-4,5-8,9-16,17-32,33+
  bool seenDetect;
  // pulse-to-pulse intervals (start to start) inside a window, and two candidate gates evaluated against a reference period
  uint32_t refUs;                                // reference period the shadow gates are judged against
  bool refGiven;                                 // false = the nominal 833 us of the ball documentation
  uint16_t iv[CH_IBINS];                         // interval histogram, 20 us bins (saturating)
  uint32_t ivN, kN[5], kOut;                     // intervals total; near 1x..4x the reference; elsewhere
  uint32_t shN, shMean[3], shComb[3];            // pre-gate detections with >= 2 pulses; pass counts at +-10, 20, 35 %
};
static IrCharStat g_char;
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
  g_chTrunc = g_isrInActive; g_chLastEnd = 0;
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

// ":irchar" window bookkeeping. Pre-gate detection = DETECT or OFFBAND (OFFBAND = enough pulses but rejected by the period gate).
static const uint8_t CH_TOL[3] = {10, 20, 35};
// Shadow gate 1 = what ":irgate" does today: the MEAN pulse interval of the window within +-tol % of the reference.
static bool irMeanGatePass(uint32_t periodUs, uint32_t refUs, uint32_t tolPct) {
  uint32_t dev = periodUs > refUs ? periodUs - refUs : refUs - periodUs;
  return (uint64_t)dev * 100u <= (uint64_t)tolPct * refUs;
}
// Shadow gate 2 (candidate, not used by the detector): EVERY interval between consecutive pulse starts must sit within +-tol % of the
// reference of a whole multiple (1..4) of it, so a missed burst (interval = 2x) is tolerated and an unrelated rhythm is not.
static bool irCombGatePass(const uint32_t* t, uint8_t n, uint32_t refUs, uint32_t tolPct) {
  if (n < 2 || refUs == 0) return false;
  for (uint8_t i = 1; i < n; i++) {
    uint32_t d = t[i] - t[i - 1];
    uint32_t k = (d + refUs / 2) / refUs;
    if (k < 1 || k > 4) return false;
    uint32_t target = k * refUs, dev = d > target ? d - target : target - d;
    if ((uint64_t)dev * 100u > (uint64_t)tolPct * refUs) return false;
  }
  return true;
}

static void irCharWindow(uint8_t ch, WinState ws, uint16_t edges, uint32_t periodUs, const uint32_t* starts, uint8_t nStarts) {
  if (ws == WS_INVALID) { g_char.invalid++; return; }
  g_char.windows++;
  g_char.cls[ws]++;
  g_char.edgeSum += edges;
  g_char.edgeHist[edges > 5 ? 5 : edges]++;
  if (edges >= 2) { uint32_t b = periodUs / 100u; g_char.per[b >= CH_PBINS ? CH_PBINS - 1 : b]++; g_char.perN++; g_char.perSum += periodUs; }
  bool det = (ws == WS_DETECT || ws == WS_OFFBAND);
  for (uint8_t i = 1; i < nStarts; i++) {          // intervals between consecutive pulse starts
    uint32_t d = starts[i] - starts[i - 1], b = d / 20u;
    if (b >= CH_IBINS) b = CH_IBINS - 1;
    if (g_char.iv[b] < 65535) g_char.iv[b]++;
    g_char.ivN++;
    uint32_t k = (d + g_char.refUs / 2) / g_char.refUs;
    if (k >= 1 && k <= 4 && irMeanGatePass(d, k * g_char.refUs, 35)) g_char.kN[k]++; else g_char.kOut++;
  }
  if (det && edges >= 2 && nStarts >= 2) {
    g_char.shN++;
    for (uint8_t i = 0; i < 3; i++) {
      if (irMeanGatePass(periodUs, g_char.refUs, CH_TOL[i])) g_char.shMean[i]++;
      if (irCombGatePass(starts, nStarts, g_char.refUs, CH_TOL[i])) g_char.shComb[i]++;
    }
  }
  if (det) {
    if (g_char.run) { g_char.runs[g_char.run == 1 ? 0 : g_char.run == 2 ? 1 : g_char.run <= 4 ? 2 : g_char.run <= 8 ? 3 : g_char.run <= 16 ? 4 : g_char.run <= 32 ? 5 : 6]++; }
    g_char.run = 0; g_char.seenDetect = true;
  } else if (g_char.seenDetect) {
    g_char.run++;
    if (g_char.run > g_char.longestRun) g_char.longestRun = g_char.run;
  }
  g_probe.windows[ch]++;
  if (ws == WS_DETECT) g_probe.hits[ch]++;
  if (edges > g_probe.maxEdges[ch]) g_probe.maxEdges[ch] = edges;
  if (det && edges >= 3 && periodUs) {            // same input as ":irgate auto" gets from the other probes
    g_probe.perSum += periodUs; g_probe.perN++;
    if (!g_probe.perMin || periodUs < g_probe.perMin) g_probe.perMin = periodUs;
    if (periodUs > g_probe.perMax) g_probe.perMax = periodUs;
  }
}
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
  if (g_probe.mode == 3) {                       // characterisation: measure only, leave receiver health and the ball track alone
    uint32_t st[CH_STARTS] = {0};
    uint8_t ns = edges < CH_STARTS ? (uint8_t)edges : (uint8_t)CH_STARTS;
    for (uint8_t i = 0; i < ns; i++) st[i] = g_chT[i];
    irCharWindow(ch, ws, edges, s.periodUs, st, ns);
    g_scan.lastCycleMs = nowMs;                  // the scan is deliberately parked on one receiver: not a stalled scan
    g_scan.open = false;
    return;
  }
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


// ---- shared helpers for the evidence engine ------------------------------------------------------------
static float g_chSin[MUX_CHANNELS], g_chCos[MUX_CHANNELS], g_chBear[MUX_CHANNELS];
static void irChTrig(const Config& c, uint8_t ch, float* sn, float* cs) {
  if (g_chBear[ch] != c.ch[ch].bearingDeg || (g_chSin[ch] == 0 && g_chCos[ch] == 0)) {
    g_chBear[ch] = c.ch[ch].bearingDeg;
    g_chSin[ch] = sinf(c.ch[ch].bearingDeg * DEG2RAD);
    g_chCos[ch] = cosf(c.ch[ch].bearingDeg * DEG2RAD);
  }
  *sn = g_chSin[ch]; *cs = g_chCos[ch];
}

// Grace period (ms) currently applied to the track: engine 0 uses the fixed lostHoldMs; engine 2 fixes it at the last good
// evidence (see irHoldUs2).
static uint32_t irEffectiveHoldMs(const Config& c) {
  if (c.engine == 2) return g_ball.holdUs ? g_ball.holdUs / 1000u : c.lostHoldMs;
  return c.lostHoldMs;
}

// ---- engine 2: leaky evidence, focused revisits, visit-statistics loss -----------------------------------------------
// ACQUISITION. Evidence per receiver is a leaky counter S (+1 per DETECT window, -IR2_MISS_DECAY per miss, cleared after
// epIdleMs without a hit), so an intermittent signal still accumulates, while a ball-like burst is bounded by TIME: the
// ball is acquired only when the evidence has lasted acqSpan1Ms (one receiver) / acqSpan2Ms (two or more neighbouring
// receivers), AND every other receiver has been looked at since the onset (a broadband onset hits the receivers in scan
// order, so until the ring has been revisited it looks like a local source), AND neither the fresh receivers nor the
// still-open episodes of the last ring cycle contradict each other (a burst that has just ended leaves an arc of "fresh"
// receivers that must not pass as a ball). For a short time after a loss, evidence near the lost bearing is accepted
// after two hits (reacqMs).
// SCHEDULING. While a candidate that has just missed, or a track whose miss statistics are poor, exists, every second
// window goes to the receivers that matter ("focus"): a weak signal is sampled several times faster than the ring
// revisit interval, the other receivers at most half as fast (never starved).
// LOSS. The grace period is the time in which the supporting receivers are visited often enough that a run of misses at
// their measured hit probability would be rarer than 10^-lossEps (Beta-predictive, so a young track is judged cautiously),
// but at least lostHoldMs (flickerHoldMs if the signal was already flickering) and at most lostHoldMaxMs. It is fixed at the last
// good evidence, so an outage cannot lengthen its own grace period, and time in which the loop itself was stalled is
// not counted as absence.
static const float IR2_MISS_DECAY = 0.1f;
static const float IR2_FOCUS_FLICK = 0.05f;              // measured miss fraction from which a track is watched more closely
static const uint8_t IR2_FOCUS_BUDGET = 6;               // ... or a miss budget above this (young / uncertain track)
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
      const uint32_t idleUs = (uint32_t)c.epIdleMs * 1000u;
      const uint32_t sinceHitUs = (uint32_t)(tUs - s.lastHitUs);
      if (s.S < 0.05f || sinceHitUs > idleUs) { s.S = 0.0f; s.ha = 0.0f; s.hb = 0.0f; }
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
    // Only a track that is young (its miss budget is still large) or has been MISSING visits needs extra windows; a steady one is
    // covered by the ring revisit. (Deciding this from the grace period in milliseconds would tie it to the loop's real speed:
    // a slightly slower ring would switch focus on for good.)
    if (g_ball.trackFlick < IR2_FOCUS_FLICK && g_ball.trackM <= IR2_FOCUS_BUDGET) return;
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
  // Acceptance must also see receivers whose evidence is no longer FRESH: after a broadband burst the receivers that were
  // hit first age out first, and the last ones visited would otherwise look like a large local source. Receivers that were
  // hit within one ring cycle beyond the freshness span are therefore weighed too (a burst reaches every receiver within one
  // ring cycle; a ball that has moved away must not veto a new cluster for longer than that).
  float ex = 0, ey = 0, eW = 0;
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++) {
    if (!irTrusted(c, ch) || g_ir[ch].S <= 0.0f || (uint32_t)(nowUs - g_ir[ch].lastHitUs) > dirSpanUs + ringUs) continue;
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
      irSupportStats(c, gMask, &g_ball.trackM, &g_ball.trackFlick); g_ball.revisitUs = c.focusOn ? ringUs / 4u : ringUs;   // a new track is watched with focus windows: its first grace is judged at that rate
      g_ball.holdUs = irHoldUs2(c);
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
    if (c.engine == 2)                                   // the old track's evidence must not pre-load the next acquisition;
      for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++)      // evidence elsewhere (a new ball already being confirmed) is kept
        if (absf(angDiffDeg(c.ch[ch].bearingDeg, g_ball.lostBearingDeg)) <= (float)c.gateDeg) g_ir[ch].S = 0.0f;
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
  if (g_probe.mode == 3) return g_probe.ch;      // ":irchar" watches one receiver back to back
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
    if (g_probe.mode == 3) {
      if (deadlineReached(nowMs, g_probe.endMs)) irProbeFinish(c, nowMs);   // the next call restarts the ring schedule
      else { bool w, f; irOpenWindow(c, irPickNext(c, &w, &f)); }
      irBallTimeout(c, nowMs);
      return;
    }
    if (c.engine == 2) irEstimate2(c, micros(), nowMs, closed, g_ir[closed].winState == WS_DETECT && irTrusted(c, closed), g_scan.lastBlindUs);
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
static void irProbeStart(uint8_t mode, uint8_t ch, float deg, uint32_t durMs, uint32_t nowMs, uint32_t refUs = 0) {
  memset(&g_probe, 0, sizeof(g_probe));
  g_probe.mode = mode; g_probe.ch = ch; g_probe.deg = deg;
  g_probe.endMs = nowMs + durMs;
  if (mode == 3) {
    g_isrChar = false;
    memset(&g_char, 0, sizeof(g_char));
    g_char.startUs = micros();
    g_char.refUs = refUs ? refUs : 833; g_char.refGiven = refUs != 0;
    g_chWMin = 0xFFFFFFFFu; g_chGMin = 0xFFFFFFFFu;
    g_chWMax = 0; g_chGMax = 0; g_chWSum = 0; g_chWN = 0; g_chGSum = 0; g_chGN = 0;
    for (uint8_t i = 0; i < CH_WBINS; i++) g_chW[i] = 0;
    for (uint8_t i = 0; i < CH_GBINS; i++) g_chG[i] = 0;
    g_isrGate = false; g_scan.open = false;     // the next service call opens the first window on the chosen receiver
    g_isrChar = true;
  }
}

// One line of "lo-hi:count" cells for the non-empty bins of a histogram (the last bin is open-ended).
template <typename T>
static void irCharHistLine(const char* name, const T* h, uint8_t nb, uint32_t binUs) {
  char b[256];
  int n = snprintf(b, sizeof(b), "%s", name);
  for (uint8_t i = 0; i < nb && n > 0 && n < (int)sizeof(b) - 24; i++) {
    if (!h[i]) continue;
    if (i == nb - 1) n += snprintf(b + n, sizeof(b) - (size_t)n, " >=%lu:%lu", UL(i * binUs), UL((uint32_t)h[i]));
    else n += snprintf(b + n, sizeof(b) - (size_t)n, " %lu-%lu:%lu", UL(i * binUs), UL((i + 1) * binUs - 1), UL((uint32_t)h[i]));
  }
  say("TEST", "%s", b);
}

static void irCharReport(const Config& c) {
  uint8_t ch = g_probe.ch;
  uint32_t elapsedUs = micros() - g_char.startUs;
  uint32_t w = g_char.windows;
  uint32_t pre = g_char.cls[WS_DETECT] + g_char.cls[WS_OFFBAND];
  uint32_t cadence = w ? elapsedUs / w : 0;
  say("TEST", "irchar C%u %lu ms: %lu windows of %u us (one per %lu us, %lu invalid) | pre-gate detections %lu (%.1f%%), passed gate %lu, rejected by gate %lu | none %lu, noisy %lu, stuck %lu",
      ch, UL(elapsedUs / 1000), UL(w), (unsigned)c.irWindowUs, UL(cadence), UL(g_char.invalid), UL(pre), w ? 100.0f * (float)pre / (float)w : 0.0f,
      UL(g_char.cls[WS_DETECT]), UL(g_char.cls[WS_OFFBAND]), UL(g_char.cls[WS_NONE]), UL(g_char.cls[WS_NOISY]), UL(g_char.cls[WS_STUCK]));
  if (c.irPeriodMaxUs) say("TEST", "irchar gate: %u..%u us is ON; gate pass rate of pre-gate detections = %.1f%% (%lu of %lu)", (unsigned)c.irPeriodMinUs, (unsigned)c.irPeriodMaxUs,
      pre ? 100.0f * (float)g_char.cls[WS_DETECT] / (float)pre : 0.0f, UL(g_char.cls[WS_DETECT]), UL(pre));
  else say("TEST", "irchar gate: OFF (set one with ':irgate auto' or ':irgate <min> <max>', then capture again to see its pass rate)");
  if (g_chWN) say("TEST", "irchar pulse width us (output at active level): n=%lu min=%lu mean=%lu max=%lu", UL(g_chWN), UL(g_chWMin), UL(g_chWSum / g_chWN), UL(g_chWMax));
  else say("TEST", "irchar pulse width: no complete pulse seen");
  if (g_chGN) say("TEST", "irchar gap us (inactive time between pulses in a window): n=%lu min=%lu mean=%lu max=%lu", UL(g_chGN), UL(g_chGMin), UL(g_chGSum / g_chGN), UL(g_chGMax));
  uint32_t wcopy[CH_WBINS], gcopy[CH_GBINS];
  for (uint8_t i = 0; i < CH_WBINS; i++) wcopy[i] = g_chW[i];
  for (uint8_t i = 0; i < CH_GBINS; i++) gcopy[i] = g_chG[i];
  irCharHistLine("irchar width hist (50 us bins):", wcopy, CH_WBINS, 50);
  irCharHistLine("irchar gap hist (100 us bins):", gcopy, CH_GBINS, 100);
  if (g_char.perN) say("TEST", "irchar window period us (windows with >= 2 pulses, the value the gate tests): n=%lu mean=%lu", UL(g_char.perN), UL(g_char.perSum / g_char.perN));
  irCharHistLine("irchar window period hist (100 us bins):", g_char.per, CH_PBINS, 100);
  say("TEST", "irchar pulses per window: 0:%lu 1:%lu 2:%lu 3:%lu 4:%lu 5+:%lu (mean %.2f)", UL(g_char.edgeHist[0]), UL(g_char.edgeHist[1]), UL(g_char.edgeHist[2]), UL(g_char.edgeHist[3]),
      UL(g_char.edgeHist[4]), UL(g_char.edgeHist[5]), w ? (double)g_char.edgeSum / (double)w : 0.0);
  // pulse-to-pulse rhythm: where the intervals between consecutive pulse starts cluster, and how the two gates would have judged this capture
  if (g_char.ivN) {
    uint8_t mb = 0;
    for (uint8_t i = 1; i < CH_IBINS - 1; i++) if (g_char.iv[i] > g_char.iv[mb]) mb = i;
    uint32_t modeUs = (uint32_t)mb * 20u + 10u, lo = modeUs - modeUs / 10u, hi = modeUs + modeUs / 10u, sum = 0, n = 0;
    for (uint8_t i = 0; i < CH_IBINS - 1; i++) { uint32_t cen = (uint32_t)i * 20u + 10u; if (cen >= lo && cen <= hi) { sum += cen * g_char.iv[i]; n += g_char.iv[i]; } }
    say("TEST", "irchar pulse intervals (start to start inside windows): n=%lu, most common ~%lu us (20 us bins; mean of the +-10%% around it %lu us) -> repeat captures of other sources with ':irchar %u <s> %lu'",
        UL(g_char.ivN), UL(modeUs), UL(n ? sum / n : 0), (unsigned)ch, UL(n ? sum / n : modeUs));
    irCharHistLine("irchar interval hist (20 us bins):", g_char.iv, CH_IBINS, 20);
    uint32_t tot = g_char.kN[1] + 2 * g_char.kN[2] + 3 * g_char.kN[3] + 4 * g_char.kN[4];
    say("TEST", "irchar vs reference %lu us (%s): intervals near 1x:%lu 2x:%lu 3x:%lu 4x:%lu, elsewhere:%lu | pulses delivered %.1f%% of the bursts the intervals span",
        UL(g_char.refUs),  g_char.refGiven ? "given" : "NOMINAL 833 us from the ball documentation, not measured", UL(g_char.kN[1]), UL(g_char.kN[2]), UL(g_char.kN[3]), UL(g_char.kN[4]), UL(g_char.kOut),
        tot ? 100.0f * (float)(g_char.kN[1] + g_char.kN[2] + g_char.kN[3] + g_char.kN[4]) / (float)tot : 0.0f);
  }
  if (g_char.shN) say("TEST", "irchar gate SHADOW (reference %lu us, %lu pre-gate detections with >=2 pulses): mean-interval gate (today's :irgate) passes %lu/%lu/%lu at +-10/20/35%% | every-interval-near-a-multiple gate (candidate) passes %lu/%lu/%lu",
      UL(g_char.refUs), UL(g_char.shN), UL(g_char.shMean[0]), UL(g_char.shMean[1]), UL(g_char.shMean[2]), UL(g_char.shComb[0]), UL(g_char.shComb[1]), UL(g_char.shComb[2]));
  say("TEST", "irchar silent runs between detections (windows): 1:%lu 2:%lu 3-4:%lu 5-8:%lu 9-16:%lu 17-32:%lu 33+:%lu | longest %lu windows = about %lu ms",
      UL(g_char.runs[0]), UL(g_char.runs[1]), UL(g_char.runs[2]), UL(g_char.runs[3]), UL(g_char.runs[4]), UL(g_char.runs[5]), UL(g_char.runs[6]),
      UL(g_char.longestRun), UL((uint32_t)((uint64_t)g_char.longestRun * cadence / 1000u)));
}

static void irProbeFinish(Config& c, uint32_t nowMs) {
  (void)nowMs;
  g_isrChar = false;
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
  else if (g_probe.mode == 3) {                  // irchar: pulse shape of whatever the receiver sees
    irCharReport(c);
    g_probe.mode = 0;
    irRestartSchedule();                         // resume the ring sweep on a clean schedule
    return;
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
