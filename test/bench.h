// bench.h - latency / reliability benchmark of the sensing-to-motor path (SIMULATION, virtual time).
//
//   ./rs26 --bench [suite=all|acq|loss|dir|drop|miss|interf|fault|stall|soak|start|cpu] [seed=N] [n=K] [name=value ...]
//
// name=value pairs set firmware parameters (any ':set' name, e.g. engine=2 losthold=120).
//
// Method: a simulated 1.2 kHz ball (833 us period, 42 % active) is injected into / removed from the simulated
// GPIO34 signal at controlled instants spread uniformly over TWO scan cycles (so scan-phase effects are sampled,
// not averaged away) with random pulse phase per trial. The real firmware code runs in the loop (50 us of virtual
// time per iteration). Every stage timestamp is taken from the firmware's own micros() or from the virtual clock.
// Randomness (pulse phases, per-visit misses, soak timeline) comes from rand() seeded with seed*7919+scenario, so
// every table can be regenerated exactly and a suite can be run alone.
//
// Limits: virtual time only. Real loop speed, flash stalls, receiver AGC/recovery, ISR latency, optical effects
// and the real ball's modulation are NOT modelled. Per-visit misses are modelled as independent; real misses are
// probably correlated. Results are SIMULATED figures, not robot measurements.
#pragma once
#include <chrono>

struct Dist {
  std::vector<double> v;
  int fails = 0, trials = 0;
  void add(double x) { v.push_back(x); trials++; }
  void fail() { fails++; trials++; }
  void addIf(double x) { if (x >= 0) add(x); else fail(); }
  double pct(double p) {
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    double k = p * (double)(v.size() - 1); size_t lo = (size_t)k; size_t hi = lo + 1 < v.size() ? lo + 1 : lo;
    return v[lo] + (v[hi] - v[lo]) * (k - (double)lo);
  }
  void print(const char* name) {
    if (v.empty()) { printf("  %-42s n=%4d  (no successful trial; %d never reached it)\n", name, trials, fails); return; }
    char p99[24];
    if (v.size() >= 100) snprintf(p99, sizeof(p99), "%7.1f", pct(0.99)); else snprintf(p99, sizeof(p99), "      -");
    printf("  %-42s n=%4zu  median=%7.1f  p95=%7.1f  p99=%s  max=%7.1f ms%s\n", name, v.size(), pct(0.5), pct(0.95), p99, pct(1.0),
           fails ? ("  [" + std::to_string(fails) + " never reached it]").c_str() : "");
  }
};

static uint32_t g_benchSeed = 12345;
static int g_benchN = 240;                                 // trials per latency distribution
static void benchReseed(int scenario) { srand(g_benchSeed * 7919u + (unsigned)scenario); }
static double benchRand() { return (double)(rand() % 100000) / 100000.0; }

static double msSince(uint64_t t0) { return (double)(g_mockUs - t0) / 1000.0; }

// ---- world model: per-channel desired signal + per-visit miss probability -------------------------------
static int g_bwMode[16];
static double g_bwMiss[16];
static int g_bwLastCur = -1;
static uint64_t g_benchLoops = 0;
static void bwClear() { for (int i = 0; i < 16; i++) { g_bwMode[i] = SIM_IDLE; g_simMode[i] = SIM_IDLE; g_bwMiss[i] = 0; } g_bwLastCur = -1; }
static void bwSet(int ch, int mode, double missP = 0.0) { g_bwMode[ch] = mode; g_bwMiss[ch] = missP; g_simMode[ch] = mode; }
static int bwBall(float deg, float fov, double missP = 0.0) {         // every receiver within fov of deg sees the ball
  int n = 0;
  for (int i = 0; i < 12; i++) {
    bool see = absf(angDiffDeg(g_cfg.ch[i].bearingDeg, deg)) <= fov;
    if (see) { bwSet(i, SIM_BALL, missP); n++; } else if (g_bwMode[i] == SIM_BALL) bwSet(i, SIM_IDLE);
  }
  return n;
}
static void bwBallMask(int mask, double missP = 0.0) { for (int i = 0; i < 12; i++) { if (mask >> i & 1) bwSet(i, SIM_BALL, missP); else if (g_bwMode[i] == SIM_BALL) bwSet(i, SIM_IDLE); } }
static void bwOff() { for (int i = 0; i < 16; i++) if (g_bwMode[i] == SIM_BALL) bwSet(i, SIM_IDLE); }
static int bwMask() { int m = 0; for (int i = 0; i < 12; i++) if (g_bwMode[i] == SIM_BALL) m |= 1 << i; return m; }

static void benchStep() {
  loop(); g_benchLoops++;
  int cur = g_scan.open ? g_scan.cur : -1;
  if (cur != g_bwLastCur) {                                // a new observation window has just opened
    if (g_bwLastCur >= 0) g_simMode[g_bwLastCur] = g_bwMode[g_bwLastCur];
    if (cur >= 0 && g_bwMode[cur] == SIM_BALL && g_bwMiss[cur] > 0.0 && benchRand() < g_bwMiss[cur]) g_simMode[cur] = SIM_IDLE;
    g_bwLastCur = cur;
  }
  mockAdvance(50);
}
static void benchRun(uint32_t ms) { uint64_t end = g_mockUs + (uint64_t)ms * 1000ULL; while (g_mockUs < end) benchStep(); }
static void benchStall(uint32_t ms) { mockAdvance(ms * 1000u); }          // the main loop does not run; interrupts still do
static void benchRandomisePhases() { for (int i = 0; i < 16; i++) g_simPhase[i] = (uint64_t)(rand() % 833); }

// Translation component actually applied to the wheels (rotation cancels out).
static double benchTranslation() {
  double vy = (g_wheelOut[0] + g_wheelOut[1] + g_wheelOut[2] + g_wheelOut[3]) / 4.0;
  double vx = (g_wheelOut[0] - g_wheelOut[1] - g_wheelOut[2] + g_wheelOut[3]) / 4.0;
  return sqrt(vx * vx + vy * vy);
}

// Return to a clean idle state: autonomy off, motors stopped, no ball, empty histories.
static void benchPrepare() {
  send("s"); bwClear(); benchRandomisePhases();
  benchRun(900);
  for (int guard = 0; guard < 20 && (g_ball.valid || !allWheelsStopped()); guard++) benchRun(100);
  for (int i = 0; i < 16; i++) { g_ir[i].hist = 0; g_ir[i].active = 0; g_ir[i].streak = 0; }
}
static void benchEnableAuto() { send("a"); benchRun(30); }
static void benchWaitUs(uint64_t us) { uint64_t tp = g_mockUs + us; while (g_mockUs < tp) benchStep(); }

// ---- A. acquisition, stage by stage -----------------------------------------------------------------------
struct AcqResult { double sample = -1, evidence = -1, acquired = -1, direction = -1, command = -1, motor = -1, start = -1; };

// mode: 1 = one receiver, 2 = two neighbouring receivers, 3 = three
static AcqResult benchAcq(int mode, uint32_t phaseUs, float truth, double missP = 0.0, bool wholeStallBg = false) {
  benchPrepare();
  benchEnableAuto();
  benchWaitUs(phaseUs);
  uint64_t t0 = g_mockUs; uint32_t t0us = (uint32_t)t0;
  if (mode == 1) bwBall(truth, 10, missP); else if (mode == 2) bwBall(truth - 15, 20, missP); else bwBall(truth, 35, missP);
  int mask = bwMask();
  uint32_t winBase[12]; for (int i = 0; i < 12; i++) winBase[i] = g_ir[i].windows;
  AcqResult r;
  uint64_t nextStall = t0 + 150000;
  while (g_mockUs - t0 < 1500000ULL) {
    benchStep();
    if (wholeStallBg && g_mockUs >= nextStall) { benchStall(20); nextStall = g_mockUs + 130000; }
    if (r.sample < 0) for (int i = 0; i < 12; i++) if ((mask >> i & 1) && g_ir[i].windows != winBase[i]) { r.sample = msSince(t0); break; }
    if (r.evidence < 0 && g_ball.evOpen) r.evidence = (double)(uint32_t)(g_ball.firstEvidenceUs - t0us) / 1000.0;
    if (r.acquired < 0 && g_ball.valid) r.acquired = msSince(t0);
    if (r.direction < 0 && g_ball.valid && fabsf(angDiffDeg(g_ball.bearingDeg, truth)) < 20) r.direction = msSince(t0);
    if (r.command < 0 && g_navState == NAV_APPROACH && (g_reqVx != 0 || g_reqVy != 0)) r.command = msSince(t0);
    if (r.motor < 0 && benchTranslation() > 0.5) r.motor = msSince(t0);
    if (r.start < 0 && benchTranslation() >= g_cfg.minPwm) r.start = msSince(t0);
    if (r.start >= 0) break;
  }
  bwClear();
  return r;
}

// ---- B. loss, stage by stage --------------------------------------------------------------------------------
struct LossResult { double evidenceAge = -1, declared = -1, nav = -1, firstDrop = -1, motor = -1; };

static LossResult benchLoss(int mode, uint32_t phaseUs, float truth, double missP = 0.0) {
  benchPrepare();
  benchEnableAuto();
  if (mode == 1) bwBall(truth, 10, missP); else if (mode == 2) bwBall(truth - 15, 20, missP); else bwBall(truth, 35, missP);
  benchRun(2500);                                          // acquired, approaching, ramps settled
  double steady = benchTranslation();
  benchWaitUs(phaseUs);
  uint64_t t0 = g_mockUs; uint32_t t0us = (uint32_t)t0;
  bwOff();
  LossResult r;
  if (!g_ball.valid) { r.evidenceAge = -2; return r; }       // a weak ball that was not being tracked: nothing to lose
  r.evidenceAge = (double)(int32_t)(t0us - g_ball.lastEvidenceUs) / 1000.0;     // age of the newest evidence at removal
  while (g_mockUs - t0 < 2500000ULL) {
    benchStep();
    if (r.declared < 0 && !g_ball.valid) r.declared = msSince(t0);
    if (r.nav < 0 && g_reqVx == 0 && g_reqVy == 0) r.nav = msSince(t0);
    if (r.firstDrop < 0 && benchTranslation() < 0.9 * steady) r.firstDrop = msSince(t0);
    if (r.motor < 0 && benchTranslation() <= 0.1 * steady) r.motor = msSince(t0);
    if (r.declared >= 0 && r.nav >= 0 && r.motor >= 0) break;
  }
  return r;
}

// ---- C. direction updates -----------------------------------------------------------------------------------
struct DirResult { double est = -1, cmd = -1, excursion = 0; };
static DirResult benchDir(float from, float to, uint32_t phaseUs, double missP = 0.0) {
  benchPrepare();
  benchEnableAuto();
  bwBall(from, 10, missP);
  benchRun(1500);
  benchWaitUs(phaseUs);
  uint64_t t0 = g_mockUs;
  bwBall(to, 10, missP);
  DirResult r;
  float arc = angDiffDeg(to, from);                         // signed shortest arc
  while (g_mockUs - t0 < 1500000ULL) {
    benchStep();
    if (g_ball.valid) {
      float e = g_ball.bearingDeg;
      if (r.est < 0 && fabsf(angDiffDeg(e, to)) < 15) r.est = msSince(t0);
      float d = angDiffDeg(e, from), dev;
      if (arc >= 0) dev = d < 0 ? -d : (d > arc ? d - arc : 0); else dev = d > 0 ? d : (d < arc ? arc - d : 0);
      if (dev > r.excursion) r.excursion = dev;
    }
    if (r.cmd < 0 && g_navState == NAV_APPROACH && (g_reqVx != 0 || g_reqVy != 0)) {
      float ang = atan2f(g_reqVx, g_reqVy) * RAD2DEG;
      if (fabsf(angDiffDeg(ang, to)) < 25) r.cmd = msSince(t0);
    }
    if (r.est >= 0 && r.cmd >= 0) break;
  }
  return r;
}

// ---- D. dropouts of a tracked ball -------------------------------------------------------------------------
// kind: 0 = strong 1 receiver, 1 = strong 2 receivers, 2 = weak 1 rx (30 % misses), 3 = weak 1 rx (50 %), 4 = weak 2 rx (50 %)
struct DropResult { bool lost = false, navLeft = false; double reacq = -1; };
static const char* DROP_KIND[5] = {"strong, 1 receiver", "strong, 2 receivers", "weak 30 % misses, 1 rx", "weak 50 % misses, 1 rx", "weak 50 % misses, 2 rx"};
static DropResult benchDropout(int kind, uint32_t gapMs, uint32_t phaseUs) {
  benchPrepare();
  benchEnableAuto();
  double p = kind == 2 ? 0.3 : (kind >= 3 ? 0.5 : 0.0);
  if (kind == 0 || kind == 2 || kind == 3) bwBall(90, 10, p); else bwBall(75, 20, p);
  benchRun(2500);
  benchWaitUs(phaseUs);
  DropResult r;
  int mask = bwMask();
  bwOff();
  uint64_t t0 = g_mockUs;
  while (g_mockUs - t0 < (uint64_t)gapMs * 1000ULL) { benchStep(); if (!g_ball.valid) r.lost = true; if (g_navState != NAV_APPROACH) r.navLeft = true; }
  bwBallMask(mask, p);
  uint64_t t1 = g_mockUs;
  while (g_mockUs - t1 < 1500000ULL) {
    benchStep();
    if (g_navState != NAV_APPROACH && g_navState != NAV_SEEK) r.navLeft = true;
    if (g_ball.valid) { if (r.lost && r.reacq < 0) r.reacq = msSince(t1); if (!r.lost || r.reacq >= 0) break; }
  }
  bwClear();
  return r;
}

// ---- E. random per-visit misses ----------------------------------------------------------------------------
struct MissResult { int losses = 0; double validFrac = 0; };
static MissResult benchMiss(int nrx, double p, uint32_t seconds) {
  benchPrepare();
  benchEnableAuto();
  if (nrx == 1) bwBall(90, 10, p); else if (nrx == 2) bwBall(75, 20, p); else bwBall(90, 35, p);
  benchRun(1500);
  MissResult r; bool was = g_ball.valid; uint64_t validSteps = 0, steps = 0; uint64_t end = g_mockUs + (uint64_t)seconds * 1000000ULL;
  while (g_mockUs < end) {
    benchStep();
    if (was && !g_ball.valid) r.losses++;
    was = g_ball.valid; steps++; if (g_ball.valid) validSteps++;
  }
  r.validFrac = steps ? (double)validSteps / (double)steps : 0;
  bwClear();
  return r;
}

// ---- F. interference ---------------------------------------------------------------------------------------
// shape: 0 one receiver, 1 two neighbours, 2 three neighbours, 3 whole ring coherent, 4 whole ring random phase
static const char* BURST_SHAPE[5] = {"1 receiver", "2 neighbours", "3 neighbours", "ring coherent", "ring random"};
static int burstMask(int shape, int base) {
  if (shape == 0) return 1 << (base % 12);
  if (shape == 1) return (1 << (base % 12)) | (1 << ((base + 1) % 12));
  if (shape == 2) return (1 << (base % 12)) | (1 << ((base + 1) % 12)) | (1 << ((base + 2) % 12));
  return 0xFFF;
}
static void burstOn(int shape, int mask) {
  for (int i = 0; i < 12; i++) if (mask >> i & 1) { bwSet(i, SIM_BALL); g_simPhase[i] = (shape == 3) ? 0 : (uint64_t)(rand() % 833); }
}
static void burstOff(int mask) { for (int i = 0; i < 12; i++) if (mask >> i & 1) bwSet(i, SIM_IDLE); }

static bool benchBurstAcquiresShape(int shape, uint32_t durMs, uint32_t phaseUs, int base) {
  benchPrepare();
  benchWaitUs(phaseUs);
  bool got = false;
  int mask = burstMask(shape, base);
  burstOn(shape, mask);
  uint64_t t0 = g_mockUs;
  while (g_mockUs - t0 < (uint64_t)durMs * 1000ULL) { benchStep(); if (g_ball.valid) got = true; }
  burstOff(mask);
  benchRun(500);
  if (g_ball.valid) got = true;
  return got;
}
// kept for the unit tests
static bool benchBurstAcquires(uint32_t durMs, uint32_t phaseUs, int ch) { return benchBurstAcquiresShape(0, durMs, phaseUs, ch); }
static bool benchBroadbandAcquires(uint32_t durMs, uint32_t phaseUs, bool coherent) { return benchBurstAcquiresShape(coherent ? 3 : 4, durMs, phaseUs, 0); }

// A burst hits while a ball is being tracked at 90 deg. where: 0 = opposite receiver, 1 = receiver next to the ball, 2 = ring.
struct InterfTrack { double maxErr = 0; bool lost = false; bool retarget = false; };
static InterfTrack benchInterfTrack(int where, uint32_t durMs, uint32_t phaseUs) {
  benchPrepare();
  benchEnableAuto();
  bwBall(75, 20);
  benchRun(1500);
  benchWaitUs(phaseUs);
  int ballMask = bwMask();
  int mask = where == 0 ? (1 << 9) : (where == 1 ? (1 << 4) : 0xFFF & ~ballMask);
  InterfTrack r;
  for (int i = 0; i < 12; i++) if (mask >> i & 1) { bwSet(i, SIM_BALL); g_simPhase[i] = (uint64_t)(rand() % 833); }
  uint64_t t0 = g_mockUs;
  while (g_mockUs - t0 < (uint64_t)(durMs + 500) * 1000ULL) {
    benchStep();
    if (g_mockUs - t0 >= (uint64_t)durMs * 1000ULL) burstOff(mask);
    if (!g_ball.valid) r.lost = true;
    else { float e = fabsf(angDiffDeg(g_ball.bearingDeg, 75)); if (e > r.maxErr) r.maxErr = e; if (e > 40) r.retarget = true; }
  }
  bwClear();
  return r;
}

// The ball disappears while an unrelated source keeps shining on another receiver. where: 0 = opposite, 1 = neighbour of the ball
static double benchDistractorLoss(int where, uint32_t phaseUs) {
  benchPrepare();
  benchEnableAuto();
  bwBall(75, 20);
  benchRun(1500);
  int dis = where == 0 ? 9 : 4;
  bwSet(dis, SIM_BALL); g_simPhase[dis] = (uint64_t)(rand() % 833);
  benchWaitUs(phaseUs);
  uint64_t t0 = g_mockUs;
  for (int i = 0; i < 12; i++) if (i != dis && g_bwMode[i] == SIM_BALL) bwSet(i, SIM_IDLE);
  double lost = -1;
  // "abandoned": the old track is no longer reported (lost, or retargeted more than 30 deg away onto the distractor)
  while (g_mockUs - t0 < 2000000ULL) { benchStep(); if (!g_ball.valid || fabsf(angDiffDeg(g_ball.bearingDeg, 75)) > 30) { lost = msSince(t0); break; } }
  bwClear();
  return lost;
}

// ---- G. channel faults ---------------------------------------------------------------------------------------
// 0 healthy, 1 ball receiver disconnected (never active), 2 ball receiver stuck active, 3 neighbour stuck active, 4 far receiver noisy,
// 5 far receiver stuck active, 6 neighbour disabled in the configuration
static const char* FAULT_NAME[7] = {"none", "ball receiver dead (stuck inactive)", "ball receiver stuck active", "neighbour stuck active", "far receiver noisy", "far receiver stuck active", "neighbour disabled"};
static void faultApply(int f) {
  bwClear();
  for (int i = 0; i < 12; i++) g_cfg.ch[i].enabled = 1;
  if (f == 3) bwSet(5, SIM_STUCK_ACTIVE);
  if (f == 4) bwSet(10, SIM_NOISE);
  if (f == 5) bwSet(10, SIM_STUCK_ACTIVE);
  if (f == 6) g_cfg.ch[5].enabled = 0;
}
// ball at 90 deg on receivers 2,3,4 (60,90,120), the fault is injected on a receiver of that group where relevant
static void faultBall(int f, double p) {
  bwBall(90, 35, p);
  if (f == 1) bwSet(3, SIM_IDLE);
  if (f == 2) bwSet(3, SIM_STUCK_ACTIVE);
}
static AcqResult benchFaultAcq(int f, uint32_t phaseUs, uint32_t settleMs) {
  benchPrepare(); faultApply(f); benchEnableAuto(); benchRun(settleMs);
  benchWaitUs(phaseUs);
  uint64_t t0 = g_mockUs;
  faultBall(f, 0.0);
  AcqResult r;
  while (g_mockUs - t0 < 1500000ULL) { benchStep(); if (r.acquired < 0 && g_ball.valid) { r.acquired = msSince(t0); break; } }
  bwClear(); for (int i = 0; i < 12; i++) g_cfg.ch[i].enabled = 1;
  return r;
}
static double benchFaultLoss(int f, uint32_t phaseUs, uint32_t settleMs) {
  benchPrepare(); faultApply(f); benchEnableAuto(); benchRun(settleMs);
  faultBall(f, 0.0); benchRun(2000);
  benchWaitUs(phaseUs);
  uint64_t t0 = g_mockUs;
  for (int i = 0; i < 12; i++) if (g_bwMode[i] == SIM_BALL || (f == 2 && i == 3)) { if (!(f == 3 && i == 5)) bwSet(i, SIM_IDLE); }
  if (f == 3) bwSet(5, SIM_STUCK_ACTIVE);
  if (f == 4) bwSet(10, SIM_NOISE);
  if (f == 5) bwSet(10, SIM_STUCK_ACTIVE);
  double lost = -1;
  while (g_mockUs - t0 < 2500000ULL) { benchStep(); if (!g_ball.valid) { lost = msSince(t0); break; } }
  bwClear(); for (int i = 0; i < 12; i++) g_cfg.ch[i].enabled = 1;
  return lost;
}
// false acquisitions with no ball at all, and false losses with a 30 %-miss ball, in `seconds`
static int benchFaultIdleAcq(int f, uint32_t seconds) {
  benchPrepare(); faultApply(f); benchEnableAuto();
  int acq = 0; bool was = false; uint64_t end = g_mockUs + (uint64_t)seconds * 1000000ULL;
  while (g_mockUs < end) { benchStep(); if (!was && g_ball.valid) acq++; was = g_ball.valid; }
  bwClear(); for (int i = 0; i < 12; i++) g_cfg.ch[i].enabled = 1;
  return acq;
}
static int benchFaultMiss(int f, double p, uint32_t seconds) {
  benchPrepare(); faultApply(f); benchEnableAuto(); benchRun(3000);
  faultBall(f, p); benchRun(1500);
  int losses = 0; bool was = g_ball.valid; uint64_t end = g_mockUs + (uint64_t)seconds * 1000000ULL;
  while (g_mockUs < end) { benchStep(); if (was && !g_ball.valid) losses++; was = g_ball.valid; }
  bwClear(); for (int i = 0; i < 12; i++) g_cfg.ch[i].enabled = 1;
  return losses;
}

// ---- H. scheduler stalls --------------------------------------------------------------------------------------
struct StallResult { int losses = 0; uint32_t invalid = 0; uint32_t maxRevisitMs = 0; uint32_t maxOverrunUs = 0; };
static StallResult benchStalls(uint32_t stallMs, uint32_t everyMs, uint32_t seconds, int nrx, double p) {
  benchPrepare(); benchEnableAuto();
  if (nrx == 1) bwBall(90, 10, p); else bwBall(75, 20, p);
  benchRun(1500);
  irRestartSchedule();
  uint32_t inv0 = g_stats.scanOverruns;
  StallResult r; bool was = g_ball.valid; uint64_t end = g_mockUs + (uint64_t)seconds * 1000000ULL;
  uint64_t next = g_mockUs + (uint64_t)everyMs * 1000ULL;
  while (g_mockUs < end) {
    benchStep();
    if (g_mockUs >= next) { benchStall(stallMs); next = g_mockUs + (uint64_t)(everyMs / 2 + (uint32_t)(rand() % (int)everyMs)) * 1000ULL; }
    if (was && !g_ball.valid) r.losses++;
    was = g_ball.valid;
  }
  r.invalid = g_stats.scanOverruns - inv0;
  for (int i = 0; i < 12; i++) if (g_scan.revisitMaxUs[i] / 1000 > r.maxRevisitMs) r.maxRevisitMs = g_scan.revisitMaxUs[i] / 1000;
  r.maxOverrunUs = g_scan.winOverrunMaxUs;
  bwClear();
  return r;
}

// ---- I. mixed-scenario soak with ground truth ---------------------------------------------------------------
struct SoakResult {
  int ballSegments = 0, missedSegments = 0, falseAcq = 0, falseLoss = 0, dropoutLoss = 0, burstsInjected = 0;
  double seconds = 0, validWhileBall = 0, ballSeconds = 0;
  Dist acq, loss;
};
// Timeline of random segments. Ball segments: random position/width/miss rate, with occasional dropouts and jumps.
// Idle segments: nothing, or interference bursts of random shape/duration. All truths are known.
static SoakResult benchSoak(uint32_t totalSeconds, uint32_t seed) {
  srand(seed * 7919u + 77u);
  benchPrepare(); benchEnableAuto();
  SoakResult R; R.seconds = totalSeconds;
  uint64_t end = g_mockUs + (uint64_t)totalSeconds * 1000000ULL;
  bool ballTruth = false; uint64_t truthEnd = 0, graceUntil = 0, dropUntil = 0;
  bool wasValid = g_ball.valid; bool segAcquired = false; uint64_t segStart = 0; int segMask = 0; double segMiss = 0;
  uint64_t nextEvent = g_mockUs + 200000; int phase = 0;           // 0 idle, 1 ball
  uint64_t burstEnd = 0; int burstMask_ = 0; bool inDrop = false;
  uint64_t nextBurst = g_mockUs + 500000;
  while (g_mockUs < end) {
    uint64_t now = g_mockUs;
    if (now >= nextEvent) {
      if (phase == 0) {                                                // start a ball segment
        phase = 1; ballTruth = true; segStart = now; segAcquired = false; R.ballSegments++;
        int nrx = 1 + rand() % 3; float deg = (float)(rand() % 360);
        static const double MISS[4] = {0.0, 0.1, 0.3, 0.5};
        segMiss = MISS[rand() % 4];
        bwBall(deg, nrx == 1 ? 10.0f : (nrx == 2 ? 20.0f : 35.0f), segMiss);
        if (nrx == 2) bwBall(deg - 15, 20.0f, segMiss);
        segMask = bwMask(); if (!segMask) bwBall(deg, 20, segMiss), segMask = bwMask();
        nextEvent = now + (uint64_t)(600 + rand() % 5400) * 1000ULL;
      } else {                                                         // end of segment: ball vanishes for 0.4 - 3 s
        phase = 0; ballTruth = false; truthEnd = now; bwOff(); segMask = 0;
        if (!segAcquired && now - segStart >= 900000ULL) R.missedSegments++;
        nextEvent = now + (uint64_t)(400 + rand() % 2600) * 1000ULL;
        graceUntil = now + (uint64_t)g_cfg.lostHoldMaxMs * 1000ULL + 100000ULL;
      }
    }
    if (phase == 1 && !inDrop && (rand() % 4000) == 0) {               // dropout of 20-300 ms
      inDrop = true; dropUntil = now + (uint64_t)(20 + rand() % 281) * 1000ULL; bwOff();
    }
    if (inDrop && now >= dropUntil) { inDrop = false; bwBallMask(segMask, segMiss); }
    if (burstEnd && now >= burstEnd) { burstOff(burstMask_); burstEnd = 0; if (phase == 1 && !inDrop) bwBallMask(segMask, segMiss); }
    if (!burstEnd && now >= nextBurst) {                                // interference burst: 10 - 250 ms, random shape
      int shape = rand() % 5; int base = rand() % 12; burstMask_ = burstMask(shape, base);
      if (phase == 1) burstMask_ &= ~segMask;                           // do not cover the ball's own receivers
      if (burstMask_) { burstOn(shape, burstMask_); burstEnd = now + (uint64_t)(10 + rand() % 241) * 1000ULL; R.burstsInjected++; }
      nextBurst = now + (uint64_t)(300 + rand() % 2200) * 1000ULL;
    }
    benchStep();
    bool v = g_ball.valid;
    if (!wasValid && v) {
      if (ballTruth) { segAcquired = true; R.acq.add(msSince(segStart)); }
      else if (now > graceUntil || true) {
        // acquisition while no ball is present: false unless it is the re-acquisition right after a genuine dropout
        R.falseAcq++;
      }
    }
    if (wasValid && !v) {
      if (ballTruth && !inDrop) {
        if (dropUntil && now - dropUntil < 400000ULL) R.dropoutLoss++; else R.falseLoss++;
      } else if (ballTruth && inDrop) R.dropoutLoss++;
      else if (!ballTruth) R.loss.add(msSince(truthEnd));
    }
    if (ballTruth) { R.ballSeconds += 5e-5; if (v) R.validWhileBall += 5e-5; }
    wasValid = v;
  }
  bwClear();
  return R;
}

// ---- J. start-up and CPU -----------------------------------------------------------------------------------------
static int benchApplyArgs(int argc, char** argv, std::string* suite) {
  for (int i = 2; i < argc; i++) {
    char* eq = strchr(argv[i], '=');
    if (!eq) continue;
    *eq = 0;
    if (!strcmp(argv[i], "suite")) { *suite = eq + 1; *eq = '='; continue; }
    if (!strcmp(argv[i], "seed")) { g_benchSeed = (uint32_t)atoi(eq + 1); *eq = '='; continue; }
    if (!strcmp(argv[i], "n")) { g_benchN = atoi(eq + 1); *eq = '='; continue; }
    if (!strcmp(argv[i], "verified")) { *eq = '='; continue; }
    const ParamDef* p = paramFind(argv[i]);
    if (!p) { fprintf(stderr, "unknown parameter %s\n", argv[i]); return 1; }
    paramSet(g_cfg, *p, (float)atof(eq + 1));
    *eq = '=';
  }
  return 0;
}

static void benchEstimateOnce() { irEstimateFast(g_cfg, micros(), millis()); }

static void benchStartup() {
  // setup() ends with irRestartSchedule(): statistics cover only the cycles after that point
  printf("START-UP (post-setup scheduling): %lu cycles since setup() finished, cycle min %.2f max %.2f ms, window overrun max %lu us\n",
         (unsigned long)g_scan.cycleCount, g_scan.cycleMinUs == 0xFFFFFFFFu ? 0.0 : g_scan.cycleMinUs / 1000.0, g_scan.cycleMaxUs / 1000.0, (unsigned long)g_scan.winOverrunMaxUs);
}

static void benchScanMetrics() {
  uint32_t mx = 0, mn = 0xFFFFFFFFu;
  for (int i = 0; i < 12; i++) { if (g_scan.revisitMaxUs[i] > mx) mx = g_scan.revisitMaxUs[i]; if (g_scan.revisitMaxUs[i] && g_scan.revisitMaxUs[i] < mn) mn = g_scan.revisitMaxUs[i]; }
  printf("SCAN: ring cycle (time to open each enabled window once) = %.2f ms nominal [min %.2f max %.2f]; per-receiver revisit interval max seen: %.2f ms (best receiver %.2f ms); window overrun max %lu us\n",
         g_scan.cycleUs / 1000.0, g_scan.cycleMinUs / 1000.0, g_scan.cycleMaxUs / 1000.0, mx / 1000.0, mn == 0xFFFFFFFFu ? 0.0 : mn / 1000.0, (unsigned long)g_scan.winOverrunMaxUs);
}

static int benchMain(int argc, char** argv) {
  std::string suite = "all";
  int verifiedFlag = 0;
  setup(); calibrateAll();
  for (int i = 2; i < argc; i++) if (!strcmp(argv[i], "verified=1")) verifiedFlag = 1;
  if (benchApplyArgs(argc, argv, &suite)) return 1;
  for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = (uint8_t)verifiedFlag;
  auto want = [&](const char* s) { return suite == "all" || suite == s; };
  benchRun(1500);
  benchStartup();
  auto cpu0 = std::chrono::steady_clock::now();
  printf("CONFIG: seed=%u n=%d suite=%s |", g_benchSeed, g_benchN, suite.c_str());
  for (int i = 2; i < argc; i++) printf(" %s", argv[i]);
  printf("\n");
  printf("ENGINE=%d window=%u us settle=%u us hold=%u..%u ms acqstreak=%u acqminstreak=%u\n", (int)g_cfg.engine, (unsigned)g_cfg.irWindowUs, (unsigned)g_cfg.irSettleUs,
         (unsigned)g_cfg.lostHoldMs, (unsigned)g_cfg.lostHoldMaxMs, (unsigned)g_cfg.acqStreak, (unsigned)g_cfg.acqMinStreak);
  benchScanMetrics();
  uint32_t span = 2 * g_scan.cycleUs;
  const int N = g_benchN;

  if (want("acq")) {
    benchReseed(1);
    const char* names[3] = {"1 receiver", "2 receivers", "3 receivers"};
    for (int mode = 1; mode <= 3; mode++) {
      Dist sa, ev, acq, dir, cmd, mot, st;
      for (int k = 0; k < N; k++) {
        AcqResult r = benchAcq(mode, (uint32_t)((uint64_t)span * k / N), 90.0f);
        sa.addIf(r.sample); ev.addIf(r.evidence); acq.addIf(r.acquired); dir.addIf(r.direction); cmd.addIf(r.command); mot.addIf(r.motor); st.addIf(r.start);
      }
      printf("ACQUISITION, ball appears, %s (onset -> stage; onset phase spread over 2 scan cycles):\n", names[mode - 1]);
      sa.print("first window closed on a ball receiver"); ev.print("first candidate evidence (DETECT window)"); acq.print("ball confirmed (valid)");
      dir.print("direction within 20 deg"); cmd.print("navigation command (translation)"); mot.print("first motor output (PWM > 0)"); st.print("wheels reach start PWM (minpwm)");
    }
    benchReseed(2);
    for (double pm : {0.3, 0.5}) {
      Dist acq;
      for (int k = 0; k < N; k++) acq.addIf(benchAcq(1, (uint32_t)((uint64_t)span * k / N), 90.0f, pm).acquired);
      char nm[64]; snprintf(nm, sizeof(nm), "1 receiver, %.0f %% misses: confirmed", pm * 100); acq.print(nm);
      Dist acq2;
      for (int k = 0; k < N; k++) acq2.addIf(benchAcq(2, (uint32_t)((uint64_t)span * k / N), 90.0f, pm).acquired);
      snprintf(nm, sizeof(nm), "2 receivers, %.0f %% misses: confirmed", pm * 100); acq2.print(nm);
    }
    benchReseed(3);
    { Dist a; for (int k = 0; k < N; k++) a.addIf(benchAcq(2, (uint32_t)((uint64_t)span * k / N), 90.0f, 0.0, true).acquired); a.print("2 receivers, 20 ms loop stall every ~150 ms"); }
    // onset relative to the multiplexer channel change: fine grid over one full ring cycle at 100 us steps
    { Dist a; double worst = 0; uint32_t worstPh = 0;
      for (uint32_t ph = 0; ph < g_scan.cycleUs; ph += 100) { double v = benchAcq(1, ph, 90.0f).acquired; a.addIf(v); if (v > worst) { worst = v; worstPh = ph; } }
      a.print("1 receiver, onset on a 100 us grid over one ring cycle"); printf("      worst onset phase %u us -> %.1f ms\n", (unsigned)worstPh, worst); }
  }
  if (want("loss")) {
    benchReseed(4);
    const char* names[3] = {"1 receiver", "2 receivers", "3 receivers"};
    for (int mode = 1; mode <= 3; mode++) {
      Dist age, dec, nav, drop, mot;
      for (int k = 0; k < N; k++) {
        LossResult r = benchLoss(mode, (uint32_t)((uint64_t)span * k / N), 90.0f);
        age.addIf(r.evidenceAge); dec.addIf(r.declared); nav.addIf(r.nav); drop.addIf(r.firstDrop); mot.addIf(r.motor);
      }
      printf("LOSS after a STRONG track, ball disappears, %s (removal -> stage):\n", names[mode - 1]);
      age.print("age of newest evidence at removal"); dec.print("ball declared lost"); nav.print("translation request cleared");
      drop.print("translation PWM first < 90 % of steady"); mot.print("translation PWM <= 10 % of steady");
    }
    benchReseed(5);
    for (double pm : {0.3, 0.5, 0.7}) {
      for (int mode = 1; mode <= 2; mode++) {
        Dist dec, mot; int untracked = 0;
        for (int k = 0; k < N / 2; k++) {
          LossResult r = benchLoss(mode, (uint32_t)((uint64_t)span * k / (N / 2)), 90.0f, pm);
          if (r.evidenceAge == -2) { untracked++; continue; }
          dec.addIf(r.declared); mot.addIf(r.motor);
        }
        char nm[64]; snprintf(nm, sizeof(nm), "WEAK track %.0f %% misses, %d rx: declared lost", pm * 100, mode); dec.print(nm);
        if (untracked) printf("      (%d of %d trials: ball not being tracked at removal, excluded)\n", untracked, N / 2);
        snprintf(nm, sizeof(nm), "WEAK track %.0f %% misses, %d rx: PWM <= 10 %%", pm * 100, mode); mot.print(nm);
      }
    }
  }
  if (want("dir")) {
    benchReseed(6);
    struct Tr { const char* name; float a, b; } trs[] = {
      {"adjacent sector 90 -> 120", 90, 120}, {"adjacent sector 90 -> 60", 90, 60}, {"opposite sector 90 -> 270", 90, 270},
      {"front seam 330 -> 0", 330, 0}, {"front seam 0 -> 330", 0, 330}, {"front seam 330 -> 30", 330, 30},
      {"rear seam 150 -> 180", 150, 180}, {"rear seam 180 -> 210", 180, 210}, {"rear seam 150 -> 210", 150, 210}};
    printf("DIRECTION UPDATE, ball jumps between sectors (jump -> stage):\n");
    for (auto& t : trs) {
      Dist est, cmd; double worstExc = 0;
      int M = N / 2;
      for (int k = 0; k < M; k++) {
        DirResult r = benchDir(t.a, t.b, (uint32_t)((uint64_t)span * k / M));
        est.addIf(r.est); cmd.addIf(r.cmd); if (r.excursion > worstExc) worstExc = r.excursion;
      }
      char nm[64];
      snprintf(nm, sizeof(nm), "%s estimate", t.name); est.print(nm);
      snprintf(nm, sizeof(nm), "%s nav command", t.name); cmd.print(nm);
      printf("      worst excursion from the short arc: %.0f deg\n", worstExc);
    }
    Dist e; for (int k = 0; k < N / 2; k++) e.addIf(benchDir(90, 120, (uint32_t)((uint64_t)span * k / (N / 2)), 0.3).est);
    e.print("adjacent 90 -> 120 with 30 % misses");
  }
  if (want("drop")) {
    benchReseed(7);
    printf("DROPOUT of a tracked ball (ball vanishes for gap, then returns; 48 phases each). Cell = declared lost / left APPROACH / median re-acquire ms:\n");
    for (int kind = 0; kind < 5; kind++) {
      printf("  %-26s", DROP_KIND[kind]);
      for (uint32_t g : {20u, 40u, 60u, 80u, 100u, 120u, 150u, 200u, 250u, 300u}) {
        int lost = 0, left = 0; Dist re;
        for (int k = 0; k < 48; k++) { DropResult r = benchDropout(kind, g, (uint32_t)((uint64_t)span * k / 48)); lost += r.lost; left += r.navLeft; if (r.reacq >= 0) re.add(r.reacq); }
        printf(" | %3u:%2d/%2d/%s", (unsigned)g, lost, left, re.v.empty() ? "-" : (std::to_string((int)re.pct(0.5))).c_str());
      }
      printf("\n");
    }
  }
  if (want("miss")) {
    benchReseed(8);
    printf("FALSE LOSS under independent per-visit misses (120 s each; false losses per minute | fraction of ball time reported valid):\n");
    for (int nrx = 1; nrx <= 3; nrx++) {
      printf("  %d receiver%s:", nrx, nrx > 1 ? "s" : " ");
      for (double pm : {0.1, 0.3, 0.5, 0.7}) { MissResult r = benchMiss(nrx, pm, 120); printf("  miss %2.0f%%: %5.2f/min %.3f |", pm * 100, r.losses / 2.0, r.validFrac); }
      printf("\n");
    }
  }
  if (want("interf")) {
    benchReseed(9);
    printf("FALSE ACQUISITION by ball-like bursts with NO ball present (48 phases each; count acquired):\n");
    for (int shape = 0; shape < 5; shape++) {
      printf("  %-14s", BURST_SHAPE[shape]);
      for (uint32_t d : {20u, 40u, 60u, 80u, 100u, 120u, 150u, 250u, 500u}) {
        int got = 0;
        for (int k = 0; k < 48; k++) got += benchBurstAcquiresShape(shape, d, (uint32_t)((uint64_t)span * k / 48), 3 + (k % 7)) ? 1 : 0;
        printf(" | %3u ms:%2d/48", (unsigned)d, got);
      }
      printf("\n");
    }
    printf("INTERFERENCE WHILE TRACKING a 2-receiver ball at 75 deg (48 phases each): worst bearing error p95 / max, false losses, retargets (>40 deg):\n");
    static const char* WH[3] = {"opposite receiver", "receiver next to ball", "rest of ring"};
    for (int where = 0; where < 3; where++) {
      printf("  %-22s", WH[where]);
      for (uint32_t d : {40u, 100u, 250u}) {
        Dist err; int lost = 0, ret = 0;
        for (int k = 0; k < 48; k++) { InterfTrack r = benchInterfTrack(where, d, (uint32_t)((uint64_t)span * k / 48)); err.add(r.maxErr); lost += r.lost; ret += r.retarget; }
        printf(" | %3u ms: err %5.1f/%5.1f lost %2d ret %2d", (unsigned)d, err.pct(0.95), err.pct(1.0), lost, ret);
      }
      printf("\n");
    }
    for (int where = 0; where < 2; where++) {
      Dist d; for (int k = 0; k < 48; k++) d.addIf(benchDistractorLoss(where, (uint32_t)((uint64_t)span * k / 48)));
      d.print(where == 0 ? "ball gone, steady source OPPOSITE: old track abandoned" : "ball gone, steady source NEIGHBOUR: old track abandoned");
    }
  }
  if (want("fault")) {
    benchReseed(10);
    printf("CHANNEL FAULTS (ball at 90 deg on receivers 2,3,4). acquire = ball confirmed, loss = removal -> declared. 'early' = fault not yet diagnosed, 'late' = after 3 s:\n");
    for (int f = 0; f < 7; f++) {
      Dist a0, a1, l0, l1;
      for (int k = 0; k < 24; k++) {
        uint32_t ph = (uint32_t)((uint64_t)span * k / 24);
        a0.addIf(benchFaultAcq(f, ph, 100).acquired); a1.addIf(benchFaultAcq(f, ph, 3000).acquired);
        l0.addIf(benchFaultLoss(f, ph, 100)); l1.addIf(benchFaultLoss(f, ph, 3000));
      }
      printf(" fault: %s\n", FAULT_NAME[f]);
      a0.print("  acquire (early)"); a1.print("  acquire (late)"); l0.print("  loss (early)"); l1.print("  loss (late)");
      printf("      false acquisitions with no ball in 60 s: %d   false losses/min for a 30 %%-miss ball: %d\n", benchFaultIdleAcq(f, 60), benchFaultMiss(f, 0.3, 60));
    }
  }
  if (want("stall")) {
    benchReseed(11);
    printf("SCHEDULER STALLS (loop does not run for the stall time; 60 s each; strong 2-receiver ball | weak 30%% 1-receiver ball): false losses / invalid windows / max revisit ms / max window overrun us\n");
    for (uint32_t s : {10u, 20u, 50u, 100u, 200u}) {
      StallResult a = benchStalls(s, 700, 60, 2, 0.0), b = benchStalls(s, 700, 60, 1, 0.3);
      printf("  stall %3u ms every ~0.7 s:  strong: %d/%u/%u/%u   weak: %d/%u/%u/%u\n", (unsigned)s, a.losses, (unsigned)a.invalid, (unsigned)a.maxRevisitMs, (unsigned)a.maxOverrunUs,
             b.losses, (unsigned)b.invalid, (unsigned)b.maxRevisitMs, (unsigned)b.maxOverrunUs);
    }
  }
  if (want("soak")) {
    printf("MIXED SOAK with ground truth (random ball segments with 0-50 %% misses, dropouts 20-300 ms, bursts 10-250 ms of random shape):\n");
    for (uint32_t sd = 0; sd < 3; sd++) {
      SoakResult r = benchSoak(600, g_benchSeed + sd);
      printf("  seed %u: %d ball segments, %d missed (never acquired in >=0.9 s), false acquisitions %d (%.2f/min), false losses %d (%.2f/min), losses during dropouts %d, bursts %d, valid while ball %.1f %%\n",
             (unsigned)(g_benchSeed + sd), r.ballSegments, r.missedSegments, r.falseAcq, r.falseAcq / (r.seconds / 60.0), r.falseLoss, r.falseLoss / (r.seconds / 60.0), r.dropoutLoss, r.burstsInjected,
             r.ballSeconds > 0 ? 100.0 * r.validWhileBall / r.ballSeconds : 0.0);
      r.acq.print("  ball-segment start -> first valid"); r.loss.print("  segment end -> declared lost");
    }
  }
  if (want("cpu")) {
    benchPrepare(); benchEnableAuto();
    auto tc0 = std::chrono::steady_clock::now(); uint64_t l0 = g_benchLoops;
    benchRun(5000);
    auto tc1 = std::chrono::steady_clock::now(); uint64_t l1 = g_benchLoops;
    bwBall(75, 20); benchRun(1000);
    auto tc2 = std::chrono::steady_clock::now(); uint64_t l2 = g_benchLoops;
    benchRun(5000);
    auto tc3 = std::chrono::steady_clock::now(); uint64_t l3 = g_benchLoops;
    double idle = std::chrono::duration<double, std::nano>(tc1 - tc0).count() / (double)(l1 - l0);
    double trk = std::chrono::duration<double, std::nano>(tc3 - tc2).count() / (double)(l3 - l2);
    (void)l2;
    printf("HOST CPU (x86 wall clock per loop() call incl. mock signal generation; NOT ESP32 time): idle %.0f ns, tracking %.0f ns (host-relative comparison between engines only)\n", idle, trk);
    bwBall(75, 20); benchRun(500);
    const int REP = 400000;
    auto te0 = std::chrono::steady_clock::now();
    for (int i = 0; i < REP; i++) { if (g_cfg.engine) benchEstimateOnce(); else irEstimate(g_cfg, millis()); }
    auto te1 = std::chrono::steady_clock::now();
    printf("HOST CPU estimator only: %.0f ns per call (engine %d), called once per 3 ms window (engine 0: once per 36 ms cycle)\n", std::chrono::duration<double, std::nano>(te1 - te0).count() / REP, (int)g_cfg.engine);
    bwClear();
  }
  auto cpu1 = std::chrono::steady_clock::now();
  printf("TOTAL host time %.1f s for %llu loop iterations\n", std::chrono::duration<double>(cpu1 - cpu0).count(), (unsigned long long)g_benchLoops);
  return 0;
}
