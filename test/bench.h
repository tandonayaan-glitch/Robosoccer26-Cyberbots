// bench.h - latency / reliability benchmark of the sensing-to-motor path (SIMULATION, virtual time).
//
//   ./rs26 --bench [name=value ...]        (name = any ':set' parameter, e.g. irwindow=2400 acqcycles=1)
//
// Method: a simulated 1.2 kHz ball (833 us period, 42 % active) is injected into / removed from the
// simulated GPIO34 signal at controlled instants spread uniformly over TWO scan cycles (so scan-phase
// effects are sampled, not hidden in an average) with random pulse phase per trial. The real firmware
// code runs in the loop (50 us per iteration of virtual time). Every stage timestamp is taken from the
// firmware's own micros() or from the virtual clock.
//
// Limits: virtual time only. Real loop speed, real flash stalls, real receiver behaviour, ISR latency and
// optical effects are NOT modelled. Results are SIMULATED latencies, not robot measurements.
#pragma once

struct Dist {
  std::vector<double> v;
  int fails = 0, trials = 0;
  void add(double x) { v.push_back(x); trials++; }
  void fail() { fails++; trials++; }
  double pct(double p) {
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    double k = p * (double)(v.size() - 1); size_t lo = (size_t)k; size_t hi = lo + 1 < v.size() ? lo + 1 : lo;
    return v[lo] + (v[hi] - v[lo]) * (k - (double)lo);
  }
  void print(const char* name) {
    if (v.empty()) { printf("  %-34s n=%3d  (no successful trial; %d failures)\n", name, trials, fails); return; }
    printf("  %-34s n=%3zu  median=%7.1f  p95=%7.1f  max=%7.1f ms%s\n", name, v.size(), pct(0.5), pct(0.95), pct(1.0),
           fails ? ("  [" + std::to_string(fails) + " trials never reached it]").c_str() : "");
  }
};

static double msSince(uint64_t t0) { return (double)(g_mockUs - t0) / 1000.0; }
static void benchStep() { loop(); mockAdvance(50); }
static void benchRun(uint32_t ms) { uint64_t end = g_mockUs + (uint64_t)ms * 1000ULL; while (g_mockUs < end) benchStep(); }

static void benchRandomisePhases() { for (int i = 0; i < 16; i++) g_simPhase[i] = (uint64_t)(rand() % 833); }

// Translation component actually applied to the wheels (rotation and nothing else cancels out).
static double benchTranslation() {
  double vy = (g_wheelOut[0] + g_wheelOut[1] + g_wheelOut[2] + g_wheelOut[3]) / 4.0;
  double vx = (g_wheelOut[0] - g_wheelOut[1] - g_wheelOut[2] + g_wheelOut[3]) / 4.0;
  return sqrt(vx * vx + vy * vy);
}

// Return to a clean idle state: autonomy off, motors stopped, no ball, empty histories.
static void benchPrepare() {
  send("s"); clearBall(); benchRandomisePhases();
  benchRun(900);
  for (int guard = 0; guard < 20 && (g_ball.valid || !allWheelsStopped()); guard++) benchRun(100);
  for (int i = 0; i < 16; i++) { g_ir[i].hist = 0; g_ir[i].active = 0; }
}
static void benchEnableAuto() { send("a"); benchRun(30); }

struct AcqResult { double evidence = -1, acquired = -1, direction = -1, command = -1, motor = -1, start = -1; };

// Ball appears at an instant `phaseUs` after the trial start. mode: 1 = one receiver, 2 = two, 3 = three
static AcqResult benchAcq(int mode, uint32_t phaseUs, float truth) {
  benchPrepare();
  benchEnableAuto();
  uint64_t tp = g_mockUs + phaseUs;
  while (g_mockUs < tp) benchStep();
  uint64_t t0 = g_mockUs; uint32_t t0us = (uint32_t)t0;
  if (mode == 1) ballAt(truth, 10); else if (mode == 2) ballAt(truth - 15, 20); else ballAt(truth, 35);
  AcqResult r;
  while (g_mockUs - t0 < 1500000ULL) {
    benchStep();
    if (r.acquired < 0 && g_ball.valid) {
      r.acquired = msSince(t0);
      if (g_ball.evOpen) r.evidence = (double)(uint32_t)(g_ball.firstEvidenceUs - t0us) / 1000.0;
    }
    if (r.direction < 0 && g_ball.valid && fabsf(angDiffDeg(g_ball.bearingDeg, truth)) < 20) r.direction = msSince(t0);
    if (r.command < 0 && g_navState == NAV_APPROACH && (g_reqVx != 0 || g_reqVy != 0)) r.command = msSince(t0);
    if (r.motor < 0 && benchTranslation() > 0.5) r.motor = msSince(t0);
    if (r.start < 0 && benchTranslation() >= g_cfg.minPwm) r.start = msSince(t0);
    if (r.start >= 0) break;
  }
  return r;
}

struct LossResult { double evidenceAge = -1, declared = -1, nav = -1, motor = -1; };

static LossResult benchLoss(int mode, uint32_t phaseUs, float truth) {
  benchPrepare();
  benchEnableAuto();
  if (mode == 1) ballAt(truth, 10); else if (mode == 2) ballAt(truth - 15, 20); else ballAt(truth, 35);
  benchRun(1800);                                          // acquired, approaching, ramps settled
  double steady = benchTranslation();
  uint64_t tp = g_mockUs + phaseUs;
  while (g_mockUs < tp) benchStep();
  uint64_t t0 = g_mockUs; uint32_t t0us = (uint32_t)t0;
  clearBall();
  LossResult r;
  uint32_t lastEv = g_ball.lastEvidenceUs;
  r.evidenceAge = (double)(int32_t)(t0us - lastEv) / 1000.0;   // how old the newest evidence already was at removal
  while (g_mockUs - t0 < 2500000ULL) {
    benchStep();
    if (r.declared < 0 && !g_ball.valid) r.declared = msSince(t0);
    if (r.nav < 0 && g_reqVx == 0 && g_reqVy == 0) r.nav = msSince(t0);
    if (r.motor < 0 && benchTranslation() <= 0.1 * steady) r.motor = msSince(t0);
    if (r.declared >= 0 && r.nav >= 0 && r.motor >= 0) break;
  }
  return r;
}

// Ball jumps from one sector to another (adjacent / opposite / across the front or rear seam).
struct DirResult { double est = -1, cmd = -1, excursion = 0; };
static DirResult benchDir(float from, float to, uint32_t phaseUs) {
  benchPrepare();
  benchEnableAuto();
  ballAt(from, 10);
  benchRun(1500);
  uint64_t tp = g_mockUs + phaseUs;
  while (g_mockUs < tp) benchStep();
  uint64_t t0 = g_mockUs;
  ballAt(to, 10);
  DirResult r;
  float arc = angDiffDeg(to, from);                         // signed shortest arc
  while (g_mockUs - t0 < 1500000ULL) {
    benchStep();
    if (g_ball.valid) {
      float e = g_ball.bearingDeg;
      if (r.est < 0 && fabsf(angDiffDeg(e, to)) < 15) r.est = msSince(t0);
      // distance of the estimate from the shortest arc between the two truths
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

// Ball present on two receivers, vanishes for gapMs, returns. true = the ball was (wrongly) declared lost.
static bool benchGapLoses(uint32_t gapMs, uint32_t phaseUs) {
  benchPrepare();
  benchEnableAuto();
  ballAt(75, 20);
  benchRun(1500);
  uint64_t tp = g_mockUs + phaseUs;
  while (g_mockUs < tp) benchStep();
  bool lost = false;
  clearBall();
  uint64_t t0 = g_mockUs;
  while (g_mockUs - t0 < (uint64_t)gapMs * 1000ULL) { benchStep(); if (!g_ball.valid) lost = true; }
  ballAt(75, 20);
  benchRun(300);
  return lost;
}

// A burst of interference that looks like the ball on ONE receiver. true = it was (wrongly) acquired.
static bool benchBurstAcquires(uint32_t durMs, uint32_t phaseUs, int ch) {
  benchPrepare();
  uint64_t tp = g_mockUs + phaseUs;
  while (g_mockUs < tp) benchStep();
  bool got = false;
  g_simMode[ch] = SIM_BALL;
  uint64_t t0 = g_mockUs;
  while (g_mockUs - t0 < (uint64_t)durMs * 1000ULL) { benchStep(); if (g_ball.valid) got = true; }
  g_simMode[ch] = SIM_IDLE;
  benchRun(500);
  if (g_ball.valid) got = true;
  return got;
}

// Same, on ALL receivers at once (broadband EMI / camera flash style).
static bool benchBroadbandAcquires(uint32_t durMs, uint32_t phaseUs, bool coherent) {
  benchPrepare();
  uint64_t tp = g_mockUs + phaseUs;
  while (g_mockUs < tp) benchStep();
  bool got = false;
  for (int i = 0; i < 12; i++) { g_simMode[i] = SIM_BALL; g_simPhase[i] = coherent ? 0 : (uint64_t)(rand() % 833); }
  uint64_t t0 = g_mockUs;
  while (g_mockUs - t0 < (uint64_t)durMs * 1000ULL) { benchStep(); if (g_ball.valid) got = true; }
  clearBall();
  benchRun(500);
  if (g_ball.valid) got = true;
  return got;
}

// Per-visit miss model: every time a window opens on a channel that currently carries the ball, the pulses are
// suppressed for that window with probability p (receiver/AGC/phase misses). Returns false losses in `seconds`.
static int benchMissLosses(int mode, double p, uint32_t seconds) {
  benchPrepare();
  benchEnableAuto();
  int mask = 0;
  if (mode == 1) ballAt(90, 10); else ballAt(75, 20);
  for (int i = 0; i < 12; i++) if (g_simMode[i] == SIM_BALL) mask |= 1 << i;
  benchRun(1500);
  int losses = 0; bool wasValid = g_ball.valid; int lastCur = -1; uint64_t end = g_mockUs + (uint64_t)seconds * 1000000ULL;
  while (g_mockUs < end) {
    benchStep();
    int cur = g_scan.open ? g_scan.cur : -1;
    if (cur != lastCur) {
      if (lastCur >= 0 && (mask >> lastCur & 1)) g_simMode[lastCur] = SIM_BALL;       // restore after the window
      if (cur >= 0 && (mask >> cur & 1) && (rand() % 1000) < (int)(p * 1000.0)) g_simMode[cur] = SIM_IDLE;
      lastCur = cur;
    }
    if (wasValid && !g_ball.valid) losses++;
    wasValid = g_ball.valid;
  }
  clearBall();
  return losses;
}

static int benchApplyArgs(int argc, char** argv) {
  for (int i = 2; i < argc; i++) {
    char* eq = strchr(argv[i], '=');
    if (!eq) continue;
    *eq = 0;
    const ParamDef* p = paramFind(argv[i]);
    if (!p) { fprintf(stderr, "unknown parameter %s\n", argv[i]); return 1; }
    paramSet(g_cfg, *p, (float)atof(eq + 1));
    *eq = '=';
  }
  return 0;
}

static int benchMain(int argc, char** argv) {
  int verifiedFlag = 0;
  setup(); calibrateAll();
  for (int i = 2; i < argc; i++) if (!strcmp(argv[i], "verified=1")) verifiedFlag = 1;
  if (benchApplyArgs(argc, argv)) return 1;
  for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = (uint8_t)verifiedFlag;
  benchRun(1500);
  srand(12345);
  int N = 96;                                              // trials per distribution
  printf("CONFIG:");
  for (int i = 2; i < argc; i++) printf(" %s", argv[i]);
  printf("\n");
  printf("scan: cycle=%.2f ms (min %.2f max %.2f) | revisit interval per receiver = cycle | window=%u us settle=%u us (settle overhead %.2f %% of cycle)\n",
         g_scan.cycleUs / 1000.0, g_scan.cycleMinUs / 1000.0, g_scan.cycleMaxUs / 1000.0, (unsigned)g_cfg.irWindowUs, (unsigned)g_cfg.irSettleUs,
         100.0 * 12.0 * g_cfg.irSettleUs / (double)g_scan.cycleUs);
  uint32_t span = 2 * g_scan.cycleUs;

  const char* names[3] = {"1 receiver", "2 receivers", "3 receivers"};
  for (int mode = 1; mode <= 3; mode++) {
    Dist ev, acq, dir, cmd, mot, st;
    for (int k = 0; k < N; k++) {
      AcqResult r = benchAcq(mode, (uint32_t)((uint64_t)span * k / N), 90.0f);
      if (r.evidence >= 0) ev.add(r.evidence); else ev.fail();
      if (r.acquired >= 0) acq.add(r.acquired); else acq.fail();
      if (r.direction >= 0) dir.add(r.direction); else dir.fail();
      if (r.command >= 0) cmd.add(r.command); else cmd.fail();
      if (r.motor >= 0) mot.add(r.motor); else mot.fail();
      if (r.start >= 0) st.add(r.start); else st.fail();
    }
    printf("ACQUISITION, ball appears, %s (injection -> stage):\n", names[mode - 1]);
    ev.print("first evidence (DETECT window)"); acq.print("ball confirmed (valid)"); dir.print("direction within 20 deg");
    cmd.print("navigation command (translation)"); mot.print("first motor output (PWM > 0)"); st.print("wheels reach start PWM (minpwm)");
  }
  for (int mode = 1; mode <= 3; mode += 1) {
    Dist age, dec, nav, mot;
    for (int k = 0; k < N; k++) {
      LossResult r = benchLoss(mode, (uint32_t)((uint64_t)span * k / N), 90.0f);
      if (r.evidenceAge >= 0) age.add(r.evidenceAge); else age.fail();
      if (r.declared >= 0) dec.add(r.declared); else dec.fail();
      if (r.nav >= 0) nav.add(r.nav); else nav.fail();
      if (r.motor >= 0) mot.add(r.motor); else mot.fail();
    }
    printf("LOSS, ball disappears, %s (removal -> stage):\n", names[mode - 1]);
    age.print("age of newest evidence at removal"); dec.print("ball declared lost");
    nav.print("translation request cleared"); mot.print("translation PWM <= 10 % of steady");
  }
  struct Tr { const char* name; float a, b; } trs[] = {
    {"adjacent sector 90 -> 120", 90, 120}, {"adjacent sector 90 -> 60", 90, 60}, {"opposite sector 90 -> 270", 90, 270},
    {"front seam 330 -> 0", 330, 0}, {"front seam 0 -> 330", 0, 330}, {"front seam 330 -> 30", 330, 30},
    {"rear seam 150 -> 180", 150, 180}, {"rear seam 180 -> 210", 180, 210}, {"rear seam 150 -> 210", 150, 210}};
  printf("DIRECTION UPDATE, ball jumps between sectors (jump -> stage):\n");
  for (auto& t : trs) {
    Dist est, cmd; double worstExc = 0;
    for (int k = 0; k < 48; k++) {
      DirResult r = benchDir(t.a, t.b, (uint32_t)((uint64_t)span * k / 48));
      if (r.est >= 0) est.add(r.est); else est.fail();
      if (r.cmd >= 0) cmd.add(r.cmd); else cmd.fail();
      if (r.excursion > worstExc) worstExc = r.excursion;
    }
    char nm[64];
    snprintf(nm, sizeof(nm), "%s estimate", t.name); est.print(nm);
    snprintf(nm, sizeof(nm), "%s nav command", t.name); cmd.print(nm);
    printf("      worst excursion from the short arc: %.0f deg\n", worstExc);
  }
  printf("FALSE LOSS on a brief dropout of a 2-receiver ball (trials %d each):\n  ", 24);
  for (uint32_t g : {20u, 40u, 60u, 80u, 100u, 150u, 250u}) {
    int lost = 0;
    for (int k = 0; k < 24; k++) lost += benchGapLoses(g, (uint32_t)((uint64_t)span * k / 24)) ? 1 : 0;
    printf("gap %3u ms: %2d/24 lost | ", g, lost);
  }
  printf("\nFALSE ACQUISITION by a ball-like burst on ONE receiver (trials 24 each):\n  ");
  for (uint32_t d : {20u, 40u, 60u, 80u, 100u, 150u, 250u}) {
    int got = 0;
    for (int k = 0; k < 24; k++) got += benchBurstAcquires(d, (uint32_t)((uint64_t)span * k / 24), 5 + (k % 3)) ? 1 : 0;
    printf("burst %3u ms: %2d/24 | ", d, got);
  }
  printf("\nFALSE ACQUISITION by a ball-like burst on ALL receivers (coherent / random phase, 24 each):\n  ");
  for (uint32_t d : {40u, 100u, 250u}) {
    int c = 0, r = 0;
    for (int k = 0; k < 24; k++) { c += benchBroadbandAcquires(d, (uint32_t)((uint64_t)span * k / 24), true) ? 1 : 0; r += benchBroadbandAcquires(d, (uint32_t)((uint64_t)span * k / 24), false) ? 1 : 0; }
    printf("burst %3u ms: %2d/24 , %2d/24 | ", d, c, r);
  }
  printf("\n");
  printf("FALSE LOSS under random per-visit misses (60 s each; 1-receiver ball | 2-receiver ball):\n  ");
  for (double pm : {0.1, 0.3, 0.5}) printf("miss %.0f%%: %2d | %2d    ", pm * 100.0, benchMissLosses(1, pm, 60), benchMissLosses(2, pm, 60));
  printf("\n");
  int fa = evalFalseAcquisitions(2.0, 120);
  printf("SOAK: random single-receiver bursts (10-120 ms, 2 per second, 120 s): false acquisitions = %d\n", fa);
  return 0;
}
