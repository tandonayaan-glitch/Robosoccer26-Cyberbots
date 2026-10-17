// Host-side verification of the firmware logic. Each test case runs in its own process
// (./run_tests.sh), so all firmware globals start fresh. "Reboot" tests re-launch the binary
// and share the on-disk mock LittleFS directory.
//
// Everything here is SOFTWARE verification against a simulated ball, IMU and filesystem.
// It says nothing about the physical robot.
#define HOST_TEST 1
#include "../Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino"

#include <functional>
#include <map>

static int g_fail = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)
#define CHECK_NEAR(a, b, tol) do { g_checks++; double _a = (a), _b = (b); if (!(fabs(_a - _b) <= (tol))) { g_fail++; printf("  FAIL line %d: %s (=%g) !~ %s (=%g) tol %g\n", __LINE__, #a, _a, #b, _b, (double)(tol)); } } while (0)

// ---- harness helpers ---------------------------------------------------------------------------
static void simRun(uint32_t ms, uint32_t loopCostUs = 50) {
  uint64_t end = g_mockUs + (uint64_t)ms * 1000ULL;
  while (g_mockUs < end) { loop(); mockAdvance(loopCostUs); }
}
static void send(const char* s) { Serial.feed(s); }
static bool outHas(const char* s) { return Serial.out.find(s) != std::string::npos; }
static int outCount(const char* s) { int n = 0; size_t p = 0; while ((p = Serial.out.find(s, p)) != std::string::npos) { n++; p += strlen(s); } return n; }
static void freshFs(const char* name) {
  g_fsDir = std::string("/tmp/mockfs_") + name;
  std::string cmd = "rm -rf " + g_fsDir;
  if (system(cmd.c_str()) != 0) {}
}
static int wheelDir(int i) {
  int a = g_pinLevel[WHEEL_PINS[i].in1], b = g_pinLevel[WHEEL_PINS[i].in2];
  return a && !b ? 1 : (!a && b ? -1 : (!a && !b ? 0 : 99));
}
static bool allWheelsStopped() {
  for (int i = 0; i < 4; i++) if (wheelDir(i) != 0 || g_pwmDuty[WHEEL_PINS[i].en] != 0) return false;
  return true;
}
static bool anyWheelEverMoved = false;
// Bearing-based ball model: every channel whose configured bearing is within `fov` deg sees the ball.
static void clearBall() { for (int i = 0; i < 16; i++) g_simMode[i] = SIM_IDLE; }
static void ballAt(float deg, float fov = 20.0f) {
  clearBall();
  for (int i = 0; i < 12; i++) if (absf(angDiffDeg(g_cfg.ch[i].bearingDeg, deg)) <= fov) g_simMode[i] = SIM_BALL;
}
static void calibrateAll() { for (int i = 0; i < 12; i++) { g_cfg.ch[i].bearingCal = 1; g_cfg.ch[i].bearingDeg = (float)i * 30.0f; } }
static void bootCal() { setup(); calibrateAll(); }
static bool wheelsMatchStrafeRight() { return wheelDir(W_FL) > 0 && wheelDir(W_FR) < 0 && wheelDir(W_BL) < 0 && wheelDir(W_BR) > 0; }
static void trackWheelMotion() { if (!allWheelsStopped()) anyWheelEverMoved = true; }
static void simRunTracking(uint32_t ms) {
  uint64_t end = g_mockUs + (uint64_t)ms * 1000ULL;
  while (g_mockUs < end) { loop(); trackWheelMotion(); mockAdvance(50); }
}
static void startAuto() {
  send("a");
  simRun(50);
}

// ================================================================================================
// pure logic
static void t_angles() {
  CHECK_NEAR(angDiffDeg(359, 1), -2, 1e-4);
  CHECK_NEAR(angDiffDeg(1, 359), 2, 1e-4);
  CHECK_NEAR(angDiffDeg(180, -180), 0, 1e-4);
  CHECK_NEAR(wrapDeg180(540), 180, 1e-3);
  CHECK_NEAR(wrapDeg180(-190), 170, 1e-3);
  CHECK_NEAR(wrapDeg360(-1), 359, 1e-3);
  CHECK_NEAR(wrapDeg360(720.5), 0.5, 1e-3);
  CHECK_NEAR(wrapDeg180(NAN), 0, 0);
  CHECK_NEAR(wrapDeg180(INFINITY), 0, 0);
  CHECK_NEAR(clampf(NAN, -1, 1), 0, 0);
  CHECK(clampf(5, -1, 1) == 1 && clampf(-5, -1, 1) == -1);
}

static void t_timer_rollover() {
  CHECK(elapsedMs(5, 0xFFFFFFF0u) == 21);
  CHECK(deadlineReached(10, 0xFFFFFFF0u));
  CHECK(!deadlineReached(0xFFFFFFF0u, 10));
  // virtual clock crosses the 32-bit millis() rollover during a full tracking run
  g_mockUs = 4294960000ULL * 1000ULL;   // 7 s before millis() wraps
  uint32_t startMs = millis();
  bootCal();
  startAuto();
  ballAt(0);
  simRun(5000);
  CHECK(g_navState == NAV_APPROACH);
  simRun(12000);                         // millis() wraps during this run
  CHECK(millis() < startMs);             // proves the 32-bit wrap happened
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_cond == COND_NORMAL);
  CHECK(g_ball.valid);
  clearBall();
  simRun(1500);
  CHECK(g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE);
}

static void t_mecanum() {
  float o[4];
  mecanumMix(0, 1, 0, false, o);   CHECK(o[0] == 1 && o[1] == 1 && o[2] == 1 && o[3] == 1);        // forward
  mecanumMix(0, -1, 0, false, o);  CHECK(o[0] == -1 && o[1] == -1 && o[2] == -1 && o[3] == -1);    // backward
  mecanumMix(1, 0, 0, false, o);   CHECK(o[0] == 1 && o[1] == -1 && o[2] == -1 && o[3] == 1);      // strafe right
  mecanumMix(-1, 0, 0, false, o);  CHECK(o[0] == -1 && o[1] == 1 && o[2] == 1 && o[3] == -1);      // strafe left
  mecanumMix(0, 0, 1, false, o);   CHECK(o[0] == 1 && o[1] == -1 && o[2] == 1 && o[3] == -1);      // clockwise
  mecanumMix(0, 0, -1, false, o);  CHECK(o[0] == -1 && o[1] == 1 && o[2] == -1 && o[3] == 1);      // counter-clockwise
  mecanumMix(1, 0, 0, true, o);    CHECK(o[0] == -1 && o[1] == 1 && o[2] == 1 && o[3] == -1);      // "O" layout mirrors strafe
  // normalisation keeps ratios: forward+strafe+rotate would exceed 1 on one wheel
  mecanumMix(0.8f, 0.8f, 0.8f, false, o);
  float m = 0; for (int i = 0; i < 4; i++) m = fmaxf(m, fabsf(o[i]));
  CHECK_NEAR(m, 1.0, 1e-5);
  CHECK_NEAR(o[2] / o[3], (0.8f - 0.8f + 0.8f) / (0.8f + 0.8f - 0.8f), 1e-4);      // BL/BR ratio unchanged by scaling
  CHECK_NEAR(o[0] / o[1], (0.8f + 0.8f + 0.8f) / (0.8f - 0.8f - 0.8f), 1e-4);      // FL/FR ratio unchanged by scaling
  mecanumMix(0.5f, 0.25f, 0.0f, false, o);   // no scaling needed
  CHECK_NEAR(o[0], 0.75, 1e-6); CHECK_NEAR(o[1], -0.25, 1e-6); CHECK_NEAR(o[2], -0.25, 1e-6); CHECK_NEAR(o[3], 0.75, 1e-6);
  mecanumMix(9, 9, 9, false, o);             // out-of-range inputs are clamped, never blow up
  for (int i = 0; i < 4; i++) CHECK(fabsf(o[i]) <= 1.0f);
  mecanumMix(NAN, INFINITY, -INFINITY, false, o);
  for (int i = 0; i < 4; i++) CHECK(isfinite(o[i]) && fabsf(o[i]) <= 1.0f);
  // PWM mapping
  CHECK(wheelPwmFromUnit(0, 40, 170) == 0);
  CHECK(wheelPwmFromUnit(0.01f, 40, 170) == 0);
  CHECK_NEAR(wheelPwmFromUnit(1, 40, 170), 170, 1e-4);
  CHECK_NEAR(wheelPwmFromUnit(-1, 40, 170), -170, 1e-4);
  CHECK(wheelPwmFromUnit(0.05f, 40, 170) >= 40);
  CHECK(fabsf(wheelPwmFromUnit(7, 40, 170)) <= 170);
}

static void t_pinmap() {
  // Independent copy of the specified wiring (not taken from pins.h).
  struct Spec { int in1, in2, en; } spec[4] = {{18, 19, 23}, {25, 26, 33}, {21, 22, 5}, {27, 14, 32}};  // FL FR BL BR
  for (int i = 0; i < 4; i++) {
    CHECK(WHEEL_PINS[i].in1 == spec[i].in1 && WHEEL_PINS[i].in2 == spec[i].in2 && WHEEL_PINS[i].en == spec[i].en);
  }
  CHECK(PIN_MUX_SIG == 34 && PIN_MUX_S0 == 4 && PIN_MUX_S1 == 13 && PIN_MUX_S2 == 15 && PIN_MUX_S3 == 2);
  CHECK(PIN_I2C_SDA == 16 && PIN_I2C_SCL == 17);
  setup();
  for (int w = 0; w < 4; w++) {
    for (int rev = 0; rev < 2; rev++) {
      g_cfg.reverse[w] = (uint8_t)rev;
      motorsStopNow();
      for (int i = 0; i < 4; i++) motorsSetWheelTarget((uint8_t)i, 0);
      motorsSetWheelTarget((uint8_t)w, 100);
      for (int k = 0; k < 100; k++) motorsUpdate(g_cfg, millis() + (uint32_t)k * 10), mockAdvance(10000);
      for (int i = 0; i < 4; i++) {
        if (i == w) {
          CHECK(wheelDir(i) == (rev ? -1 : 1));
          CHECK(g_pwmDuty[spec[i].en] == 100);
          CHECK(g_pinLevel[spec[i].in1] == (rev ? 0 : 1) && g_pinLevel[spec[i].in2] == (rev ? 1 : 0));
        } else {
          CHECK(wheelDir(i) == 0 && g_pwmDuty[spec[i].en] == 0);
        }
      }
      motorsSetWheelTarget((uint8_t)w, -100);
      for (int k = 0; k < 200; k++) motorsUpdate(g_cfg, millis() + (uint32_t)k * 10), mockAdvance(10000);
      CHECK(wheelDir(w) == (rev ? 1 : -1));
    }
    g_cfg.reverse[w] = 0;
  }
}

static void t_boot_safe() {
  // pins are LOW before anything else runs
  for (int i = 0; i < 64; i++) g_pinLevel[i] = 1;     // pretend the pins floated high
  motorsPinsSafe();
  for (int i = 0; i < 4; i++) CHECK(g_pinLevel[WHEEL_PINS[i].in1] == 0 && g_pinLevel[WHEEL_PINS[i].in2] == 0 && g_pinLevel[WHEEL_PINS[i].en] == 0);
  CHECK(!g_motorsEnabled);
  setup();
  CHECK(!g_autoEnabled);
  CHECK(g_navState == NAV_IDLE);
  CHECK(allWheelsStopped());
  anyWheelEverMoved = false;
  simRunTracking(8000);      // long idle after boot, even with a ball in view, nothing may move by itself
  ballAt(0);
  simRunTracking(5000);
  CHECK(!anyWheelEverMoved);
  CHECK(!g_autoEnabled);
  CHECK(outHas("autonomous mode: DISABLED"));
  CHECK(outHas("reset reason: POWERON"));
}

static void t_ramp() {
  setup();
  uint32_t t = millis();
  g_motorsLastUpdateMs = t;
  motorsSetWheelTarget(0, 150);
  float prev = 0;
  for (int k = 1; k <= 400; k++) {
    t += 10; mockAdvance(10000);
    motorsUpdate(g_cfg, t);
    float step = g_wheelCur[0] - prev;
    CHECK(step <= g_cfg.accelPwmPerS * 0.010f + 1e-3f);
    CHECK(step >= 0);
    prev = g_wheelCur[0];
  }
  CHECK_NEAR(g_wheelCur[0], 150, 1e-3);
  // direction reversal: must pass through zero and dwell
  motorsSetWheelTarget(0, -150);
  bool sawZero = false; int zeroTicks = 0; float last = g_wheelCur[0];
  for (int k = 1; k <= 600; k++) {
    t += 10; mockAdvance(10000);
    motorsUpdate(g_cfg, t);
    if (g_wheelCur[0] == 0) { sawZero = true; zeroTicks++; }
    CHECK(!(last > 0 && g_wheelCur[0] < 0));   // never flips sign in a single step
    last = g_wheelCur[0];
  }
  CHECK(sawZero);
  CHECK(zeroTicks >= (int)(g_cfg.reverseDeadMs / 10));
  CHECK_NEAR(g_wheelCur[0], -150, 1e-3);
  // zero command stops reliably (ramped), and stop-now is immediate
  motorsSetWheelTarget(0, 0);
  for (int k = 0; k < 400; k++) { t += 10; mockAdvance(10000); motorsUpdate(g_cfg, t); }
  CHECK(g_wheelCur[0] == 0 && wheelDir(0) == 0 && g_pwmDuty[23] == 0);
  motorsSetWheelTarget(1, 200);
  for (int k = 0; k < 100; k++) { t += 10; mockAdvance(10000); motorsUpdate(g_cfg, t); }
  CHECK(wheelDir(1) != 0);
  motorsStopNow();
  CHECK(allWheelsStopped());
  // autonomous speed limit is honoured
  motorsDrive(g_cfg, 0, 1, 0, g_cfg.maxAuto);
  for (int i = 0; i < 4; i++) CHECK(g_wheelTgt[i] <= g_cfg.maxAuto);
  motorsDrive(g_cfg, 0, 1, 0, 255);
  for (int i = 0; i < 4; i++) CHECK(g_wheelTgt[i] <= 255);
  // a long stall must not produce a giant ramp step
  motorsStopNow();
  motorsSetWheelTarget(2, 200);
  g_motorsLastUpdateMs = t; t += 5000;
  motorsUpdate(g_cfg, t);
  CHECK(g_wheelCur[2] <= g_cfg.accelPwmPerS * 0.1f + 1e-3f);
}

static void t_config() {
  Config c; cfgDefaults(c);
  char rep[96];
  CHECK(cfgValidate(c, rep, sizeof(rep)) == 0);
  // NaN / out of range / bad enumerations are replaced with safe defaults and reported
  Config d; cfgDefaults(d);
  c.acqConf = NAN;                 CHECK(cfgValidate(c, rep, sizeof(rep)) == 1 && c.acqConf == d.acqConf && strstr(rep, "acqconf"));
  c.maxAuto = 0;                   CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.maxAuto == d.maxAuto);
  c.pwmFreq = 60000;               CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.pwmFreq == d.pwmFreq);
  c.reverse[2] = 7;                CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.reverse[2] == 0);
  c.ch[3].bearingDeg = 720;        CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.ch[3].bearingDeg == d.ch[3].bearingDeg && !c.ch[3].bearingCal);
  c.ch[4].enabled = 9;             CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.ch[4].enabled == d.ch[4].enabled);
  c.minPwm = 200; c.maxManual = 100; CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.minPwm < c.maxManual);
  c.imuYawSign = 3;                CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.imuYawSign == -1);
  c.imuAddrPref = 0x30;            CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.imuAddrPref == 0);
  CHECK(cfgValidate(c, rep, sizeof(rep)) == 0);
  // every default is inside its own declared range
  for (size_t i = 0; i < PARAM_COUNT; i++) CHECK(paramInRange(PARAMS[i], paramGet(d, PARAMS[i])));
  // CRC / persistence
  freshFs("config");
  CHECK(fsInit());
  Config s; cfgDefaults(s); s.maxAuto = 123; s.ch[5].bearingDeg = 150; s.ch[5].bearingCal = 1;
  CHECK(cfgSave(s));
  Config l; CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_OK);
  CHECK(l.maxAuto == 123 && l.ch[5].bearingCal == 1);
  // corrupt one byte in the middle -> CRC failure -> defaults + reported
  { std::string p = g_fsDir + "/config.bin"; FILE* f = fopen(p.c_str(), "r+b"); fseek(f, 40, SEEK_SET); fputc(0xAB, f); fclose(f); }
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_BADCRC && l.maxAuto == d.maxAuto);
  // wrong version
  { Config v; cfgDefaults(v); v.version = 99; cfgSeal(v); std::string p = g_fsDir + "/config.bin"; FILE* f = fopen(p.c_str(), "wb"); fwrite(&v, sizeof(v), 1, f); fclose(f); }
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_BADVERSION);
  { Config v; cfgDefaults(v); v.magic = 1; cfgSeal(v); std::string p = g_fsDir + "/config.bin"; FILE* f = fopen(p.c_str(), "wb"); fwrite(&v, sizeof(v), 1, f); fclose(f); }
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_BADMAGIC);
  { std::string p = g_fsDir + "/config.bin"; FILE* f = fopen(p.c_str(), "wb"); fwrite("short", 1, 5, f); fclose(f); }
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_BADSIZE && l.maxAuto == d.maxAuto);
  // valid CRC but invalid field -> repaired, others kept
  { Config v; cfgDefaults(v); v.maxManual = 111; v.acqConf = 7.0f; cfgSeal(v); std::string p = g_fsDir + "/config.bin"; FILE* f = fopen(p.c_str(), "wb"); fwrite(&v, sizeof(v), 1, f); fclose(f); }
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_REPAIRED && l.maxManual == 111 && l.acqConf == d.acqConf && strstr(rep, "acqconf"));
  // never persist invalid settings
  Config bad; cfgDefaults(bad); bad.acqConf = NAN;
  CHECK(!cfgSave(bad));
  // pins are not part of the configuration: no parameter name can touch them
  for (size_t i = 0; i < PARAM_COUNT; i++) CHECK(strstr(PARAMS[i].name, "gpio") == nullptr && strncmp(PARAMS[i].name, "pin", 3) != 0);
  // and no ParamDef offset can reach outside the Config struct
  for (size_t i = 0; i < PARAM_COUNT; i++) CHECK(PARAMS[i].off + 1 <= offsetof(Config, ch));
}

static void t_classify() {
  Config c; cfgDefaults(c);
  CHECK(irClassifyWindow(c, 0, 0, 3000, false, false) == WS_NONE);
  CHECK(irClassifyWindow(c, 1, 300, 3000, false, false) == WS_NONE);        // one pulse is not enough
  CHECK(irClassifyWindow(c, 2, 700, 3000, false, false) == WS_DETECT);
  CHECK(irClassifyWindow(c, 4, 1400, 3000, false, false) == WS_DETECT);     // ~1.2 kHz ball
  CHECK(irClassifyWindow(c, 40, 1500, 3000, false, false) == WS_NOISY);     // 13 pulses/ms
  CHECK(irClassifyWindow(c, 0, 3000, 3000, true, true) == WS_STUCK);
  CHECK(irClassifyWindow(c, 0, 1000, 3000, true, false) == WS_NONE);        // went inactive: not stuck
  CHECK(irClassifyWindow(c, 250, 0, 3000, false, false) == WS_NOISY);       // flood cap
}

// ================================================================================================
// IR
static void t_ir_detect() {
  bootCal();
  simRun(1500);
  CHECK(!g_ball.valid);
  CHECK(g_cond == COND_NORMAL);                      // no ball is a navigation condition, not a failure
  for (int i = 0; i < 12; i++) CHECK(irChStatus(g_cfg, i) == CH_UNVERIFIED);
  CHECK(!faultActive(F_IR_CHANNEL));
  // scan timing: 12 * (window + settle + loop quantisation)
  CHECK(g_scan.cycleCount > 5);
  CHECK(g_scan.cycleUs >= 12 * 3000 && g_scan.cycleUs < 12 * 3200);
  CHECK(g_scan.enabledCount == 12);
  for (int i = 12; i < 16; i++) CHECK(g_ir[i].windows == 0);   // C12..C15 never scanned
  ballAt(90, 10);                                    // only channel 3 sees it
  simRun(1200);
  CHECK(g_ir[3].active && g_ir[3].detCount > 3);
  CHECK(g_ir[3].edges >= 2 && g_ir[3].edges <= 5);
  CHECK_NEAR(g_ir[3].periodUs, 833, 120);            // measured pulse period of the simulated ball
  int others = 0; for (int i = 0; i < 12; i++) if (i != 3 && g_ir[i].active) others++;
  CHECK(others == 0);
  CHECK(g_ball.valid);
  CHECK_NEAR(g_ball.bearingDeg, 90, 1.0);
  CHECK(g_ball.conf >= g_cfg.acqConf);
  CHECK(g_ball.conf <= 0.86);                        // channel is UNVERIFIED -> 0.85 factor
  g_cfg.ch[3].verified = 1;
  simRun(500);
  CHECK(g_ball.conf > 0.95);
  // ball taken away: held for lostHold then declared lost, with no sensor fault
  clearBall();
  simRun(g_cfg.lostHoldMs + 600);
  CHECK(!g_ball.valid);
  CHECK(!faultActive(F_IR_CHANNEL) && g_cond == COND_NORMAL);
  CHECK(g_stats.channelTestFailMask == 0);
}

static void t_ir_wrap() {
  bootCal();
  clearBall();
  g_simMode[11] = SIM_BALL; g_simMode[0] = SIM_BALL;       // bearings 330 and 0 -> ball near 345 (= -15)
  simRun(1500);
  CHECK(g_ball.valid);
  CHECK_NEAR(g_ball.bearingDeg, -15, 2.0);                 // not 165
  CHECK(g_ball.rInst > 0.95);
  clearBall(); simRun(1500);
  g_simMode[5] = SIM_BALL; g_simMode[6] = SIM_BALL; g_simMode[7] = SIM_BALL;   // 150,180,210 -> 180
  simRun(1500);
  CHECK(fabsf(g_ball.bearingDeg) > 175);
}

static void t_ir_ambiguous() {
  bootCal();
  g_simMode[0] = SIM_BALL; g_simMode[6] = SIM_BALL;        // opposite sides: contradictory
  simRun(2000);
  CHECK(g_ball.anyDetect);
  CHECK(g_ball.rInst < 0.5);
  CHECK(g_ball.ambiguous);
  CHECK(g_ball.conf < g_cfg.acqConf);
  CHECK(!g_ball.valid);                                    // never acts on contradictory data
}

static void t_ir_stuck_and_recover() {
  bootCal();
  g_simMode[5] = SIM_STUCK_ACTIVE;
  simRun(1500);
  CHECK(irChStatus(g_cfg, 5) == CH_UNVERIFIED);            // not yet long enough to suspect
  simRun(1500);                                            // > 2 s
  CHECK(irChStatus(g_cfg, 5) == CH_SUSPECT_STUCK);
  CHECK(!irTrusted(g_cfg, 5));
  CHECK(faultActive(F_IR_CHANNEL) && g_cond == COND_DEGRADED);
  CHECK(outHas("IR_CHANNEL"));
  simRun(14000);
  CHECK(irChStatus(g_cfg, 5) == CH_FAIL_STUCK);
  uint8_t nT, nU; float gap = irCoverageGap(g_cfg, &nT, &nU);
  CHECK(nT == 11);                                         // recalculated usable channel count
  CHECK(gap == 0);                                         // 30 deg spacing with a 35 deg fov: still covered
  // remaining channels still find the ball
  g_simMode[2] = SIM_BALL; simRun(1500);
  CHECK(g_ball.valid && fabsf(g_ball.bearingDeg - 60) < 2);
  // the transient clears: suspect first, verification period, then fault cleared
  g_simMode[5] = SIM_IDLE; g_simMode[2] = SIM_IDLE;
  simRun(2500);
  CHECK(irChStatus(g_cfg, 5) != CH_FAIL_STUCK);
  simRun(g_cfg.recoverVerifyMs + 2500);
  CHECK(irChStatus(g_cfg, 5) == CH_UNVERIFIED || irChStatus(g_cfg, 5) == CH_SUSPECT_STUCK);
}

static void t_ir_noise() {
  bootCal();
  g_simMode[2] = SIM_NOISE;
  simRun(3000);
  CHECK(g_ir[2].noisy);
  CHECK(irChStatus(g_cfg, 2) == CH_SUSPECT_NOISY && !irTrusted(g_cfg, 2));
  CHECK(!g_ball.valid);                                    // noise never becomes a ball
  g_simMode[2] = SIM_IDLE;
  simRun(3000);
  CHECK(!g_ir[2].noisy);
  CHECK(irTrusted(g_cfg, 2));
}

static void t_ir_polarity() {
  bootCal();
  send(":ch 4 pol high\n");
  simRun(50);
  CHECK(g_cfg.ch[4].activeLow == 0);
  g_simActiveHigh[4] = true;                               // module idles LOW, pulses HIGH
  g_simMode[4] = SIM_BALL;
  simRun(1500);
  CHECK(g_ir[4].active);
  g_simMode[4] = SIM_IDLE;
  simRun(1500);
  CHECK(!g_ir[4].active && irChStatus(g_cfg, 4) == CH_UNVERIFIED);   // idle-LOW is not "stuck" for an active-high channel
}

static void t_ir_system_loss() {
  bootCal();
  simRun(500);
  startAuto();
  CHECK(g_autoEnabled);
  for (int i = 0; i < 12; i++) { char b[40]; snprintf(b, sizeof(b), ":ch %d en 0\n", i); send(b); }
  simRun(300);
  CHECK(faultActive(F_IR_NO_CHANNELS));
  CHECK(g_cond == COND_SAFE_STOP);
  CHECK(!g_autoEnabled);
  CHECK(g_navState == NAV_SAFE_STOP);
  CHECK(allWheelsStopped());
  Serial.out.clear();
  send("a"); simRun(100);
  CHECK(!g_autoEnabled && outHas("REFUSED"));
  // diagnostics stay alive in SAFE_STOP
  Serial.out.clear(); send("h"); simRun(300);
  CHECK(outHas("HARDWARE HEALTH REPORT") && outHas("IR_NO_CHANNELS"));
  // re-enable channels: fault clears only after the verification period; autonomy stays OFF
  for (int i = 0; i < 12; i++) { char b[40]; snprintf(b, sizeof(b), ":ch %d en 1\n", i); send(b); }
  simRun(500);
  CHECK(g_cond == COND_SAFE_STOP);                         // not yet verified
  simRun(g_cfg.recoverVerifyMs + 1500);
  CHECK(g_cond != COND_SAFE_STOP);
  CHECK(!g_autoEnabled);
  CHECK(g_navState == NAV_IDLE);
}

static void t_ir_trusted_low() {
  bootCal();
  for (int i = 3; i < 12; i++) { char b[40]; snprintf(b, sizeof(b), ":ch %d trust 0\n", i); send(b); }   // 3 trusted left (< 4)
  simRun(500);
  CHECK(faultActive(F_IR_TRUSTED_LOW) && g_cond == COND_SAFE_STOP);
  startAuto();
  CHECK(!g_autoEnabled);
}

static void t_ir_scan_stall() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0);
  simRun(1500);
  CHECK(g_navState == NAV_APPROACH && !allWheelsStopped());
  g_scan.running = false;                                  // scheduler dies
  simRun(1500);
  CHECK(faultActive(F_IR_SCAN_STALL));
  CHECK(g_cond == COND_SAFE_STOP && !g_autoEnabled && allWheelsStopped());
  g_scan.running = true; g_scan.open = false;
  simRun(500);
  CHECK(g_cond == COND_SAFE_STOP);                         // recovery must be verified, not instant
  simRun(g_cfg.recoverVerifyMs + 1000);
  CHECK(g_cond == COND_NORMAL);
  CHECK(!g_autoEnabled && allWheelsStopped());             // never auto-resumes
}

static void t_ir_overrun() {
  bootCal();
  simRun(500);
  uint32_t ov0 = g_stats.scanOverruns;
  g_simMode[3] = SIM_BALL;
  simRun(1000, 20000);                                     // control loop only runs every 20 ms: windows overrun
  CHECK(g_stats.scanOverruns > ov0);
  CHECK(irChStatus(g_cfg, 3) != CH_SUSPECT_STUCK && irChStatus(g_cfg, 3) != CH_FAIL_STUCK);
  CHECK(!faultActive(F_IR_CHANNEL));                       // late windows are discarded, not misread
}

static void t_ir_coverage() {
  bootCal();
  for (int i = 2; i <= 7; i++) { char b[40]; snprintf(b, sizeof(b), ":ch %d trust 0\n", i); send(b); }   // leaves 0,1,8..11 = 6 trusted
  simRun(600);
  uint8_t nT, nU; float gap = irCoverageGap(g_cfg, &nT, &nU);
  CHECK(nT == 6);
  CHECK(gap > g_cfg.maxGapDeg);
  CHECK(faultActive(F_IR_GAP) && g_cond == COND_DEGRADED);
  CHECK(!faultActive(F_IR_TRUSTED_LOW));
  startAuto();
  CHECK(g_autoEnabled);                                    // degraded but allowed
  CHECK(outHas("DEGRADED"));
  CHECK_NEAR(navSensorScale(g_cfg), g_cfg.degradedScale, 1e-6);
}

static void t_ir_probe() {
  bootCal();
  simRun(600);
  // controlled test with the ball on channel 7: PASS and persistent flag
  g_simMode[7] = SIM_BALL;
  send(":verify 7\n");
  simRun(5000);
  CHECK(outHas("C7 PASS"));
  CHECK(g_cfg.ch[7].verified == 1 && irChStatus(g_cfg, 7) == CH_VERIFIED);
  // channel 8: ball NOT present -> suspect first, then confirmed fail on a second failed test
  Serial.out.clear();
  send(":verify 8\n"); simRun(5000);
  CHECK(outHas("C8 SUSPECT") && irChStatus(g_cfg, 8) == CH_SUSPECT_TEST && irTrusted(g_cfg, 8));
  send(":verify 8\n"); simRun(5000);
  CHECK(outHas("C8 FAIL") && irChStatus(g_cfg, 8) == CH_FAIL_TEST && !irTrusted(g_cfg, 8));
  CHECK((g_stats.channelTestFailMask & (1u << 8)) != 0);
  CHECK(faultActive(F_IR_CHANNEL));
  send(":chreset 8\n"); simRun(100);
  CHECK(irChStatus(g_cfg, 8) == CH_UNVERIFIED);
  // coverage learning at a known bearing
  clearBall(); g_simMode[2] = SIM_BALL; g_simMode[3] = SIM_BALL;
  Serial.out.clear();
  send(":learn 75\n"); simRun(4000);
  CHECK(g_covN == 1 && g_cov[0].mask == ((1u << 2) | (1u << 3)) && g_cov[0].deg == 75);
  send(":cov\n"); simRun(200);
  CHECK(outHas("COV,75,000C"));
}

// ================================================================================================
// IMU
static void t_imu_integrate() {
  setup();
  CHECK(g_imu.present && g_imu.ok && g_imu.calibrated && g_imu.yawValid);
  CHECK_NEAR(g_imu.bias, 0, 0.2);
  CHECK(g_imu.addr == 0x68);
  g_imu.yaw = 0;
  g_mpu.gz = 30.0f;                                         // 30 deg/s counter-clockwise (right-hand rule, Z up)
  simRun(2000);
  CHECK_NEAR(g_imu.yaw, -60, 2.5);                          // clockwise-positive yaw decreases
  g_mpu.gz = -90.0f;
  simRun(3000);                                             // net +210 clockwise -> wraps to -150
  CHECK_NEAR(g_imu.yaw, wrapDeg180(-60 + 270), 4.0);
  CHECK(g_imu.yaw > -180 && g_imu.yaw <= 180);
  g_mpu.gz = 0;
  float y0 = g_imu.yaw; simRun(3000);
  CHECK_NEAR(g_imu.yaw, y0, 0.5);
}

static void t_imu_bias() {
  g_mpu.gz = 1.5f;                                          // constant gyro offset while stationary
  setup();
  CHECK_NEAR(g_imu.bias, 1.5, 0.2);
  g_imu.yaw = 0; simRun(5000);
  CHECK_NEAR(g_imu.yaw, 0, 1.0);                            // bias compensated: no drift despite the offset
  // calibration refuses a moving robot
  g_mpu.gz = 0;
  send(":gcal\n"); simRun(100);
  g_mpu.gz = 40;
  for (int i = 0; i < 200; i++) { g_mpu.gz = (i & 1) ? 40.0f : -40.0f; simRun(10); }
  CHECK(g_imu.calState == 3);
  CHECK(outHas("calibration rejected"));
}

static void t_imu_missing() {
  g_mpu.present = false;
  bootCal();
  CHECK(!g_imu.present);
  CHECK(faultActive(F_IMU_MISSING) && g_cond == COND_DEGRADED);
  CHECK(outHas("NOT FOUND"));
  // ball seeking remains available with a robot-relative fallback
  simRun(500);
  startAuto();
  CHECK(g_autoEnabled);
  CHECK(outHas("turn-toward-ball"));
  ballAt(60);
  simRun(2000);
  CHECK(g_navState == NAV_APPROACH);
  CHECK(isnan(g_hdgErr));
  CHECK(g_reqW > 0);                                        // turns toward a ball on the right
  CHECK(fabsf(g_reqW) <= g_cfg.hdgMaxRot + 1e-6);
  CHECK(!allWheelsStopped());
  // slower than the healthy case because the condition is DEGRADED
  CHECK(sqrtf(g_reqVx * g_reqVx + g_reqVy * g_reqVy) <= g_cfg.degradedScale * g_cfg.attackSpeed + 1e-4);
}

static void t_imu_dropout() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0);
  simRun(2500);
  CHECK(g_navState == NAV_APPROACH);
  CHECK(!isnan(g_hdgErr));                                  // heading hold active
  g_mpu.mode = 1;                                           // I2C NACK: sensor disconnected
  simRun(800);
  CHECK(!g_imu.ok && !g_imu.yawValid);
  CHECK(faultActive(F_IMU_TIMEOUT) && g_cond == COND_DEGRADED);
  CHECK(g_stats.imuTimeouts == 1);
  CHECK(g_navState == NAV_APPROACH && !allWheelsStopped());   // ball seeking continues without heading
  CHECK(isnan(g_hdgErr));
  CHECK(outHas("IMU_TIMEOUT"));
  g_mpu.mode = 0;                                           // reconnect
  simRun(3500);
  CHECK(g_imu.ok && g_imu.yawValid);
  CHECK(g_imu.recoveries >= 1);
  CHECK(faultActive(F_IMU_TIMEOUT));                        // verified-recovery period still running
  simRun(g_cfg.recoverVerifyMs + 500);
  CHECK(!faultActive(F_IMU_TIMEOUT) && g_cond == COND_NORMAL);
  CHECK(outHas("IMU_TIMEOUT cleared"));
  CHECK_NEAR(g_imu.yaw, 0, 3.0);                            // reference re-zeroed after the outage
}

static void t_imu_invalid() {
  bootCal();
  simRun(500);
  g_imu.yaw = 0;
  g_mpu.mode = 4;                                           // gyro pinned at full scale (-32768)
  simRun(1500);
  CHECK_NEAR(g_imu.yaw, 0, 0.01);                           // invalid frames are never integrated
  CHECK(g_imu.invalidCount > 50 && !g_imu.ok);
  CHECK(faultActive(F_IMU_INVALID));
  g_mpu.mode = 3;                                           // all zeros (dead bus pattern)
  simRun(500);
  CHECK_NEAR(g_imu.yaw, 0, 0.01);
  g_mpu.mode = 2;                                           // frozen sensor
  g_mpu.gz = 0;
  simRun(2500);
  CHECK(!g_imu.ok);
  g_mpu.mode = 0;
  simRun(g_cfg.recoverVerifyMs + 2000);
  CHECK(g_imu.ok && !faultActive(F_IMU_INVALID));
}

static void t_heading_hold() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0);
  simRun(1500);
  CHECK(!isnan(g_hdgErr));
  // disturbance: robot rotated 20 deg counter-clockwise -> must command clockwise rotation, bounded
  g_imu.yaw = -20.0f;
  simRun(40);
  CHECK(g_hdgErr > 15 && g_reqW > 0 && g_reqW <= g_cfg.hdgMaxRot + 1e-6);
  g_imu.yaw = 170.0f;                                       // reference 0: shortest way is anticlockwise
  simRun(40);
  CHECK(g_hdgErr < 0 && g_reqW < 0);
  g_imu.yaw = 179.0f; g_refYaw = -179.0f;                   // across the +-180 seam: error is +2, not -358
  simRun(40);
  CHECK_NEAR(g_hdgErr, 2.0, 0.6);
  g_refYaw = g_imu.yaw;                                     // within deadband
  simRun(40);
  CHECK(fabsf(g_reqW) < 1e-6);
}

// ================================================================================================
// navigation
static void t_nav_approach() {
  bootCal();
  simRun(500);
  startAuto();
  CHECK(g_autoEnabled && g_navState == NAV_SEEK);
  ballAt(90, 10);                                           // ball exactly to the right
  simRun(2500);
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_reqVx > 0.3 && fabsf(g_reqVy) < 0.2);             // strafe right
  CHECK(wheelsMatchStrafeRight());
  ballAt(0, 10);                                            // dead ahead
  simRun(2500);
  CHECK(g_reqVy > 0.5 && fabsf(g_reqVx) < 0.15);
  CHECK(wheelDir(W_FL) > 0 && wheelDir(W_FR) > 0 && wheelDir(W_BL) > 0 && wheelDir(W_BR) > 0);
  ballAt(270, 10);                                          // left
  simRun(2500);
  CHECK(g_reqVx < -0.3);
  // aggressive only with confidence + healthy sensing
  for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = 1;
  ballAt(0, 40);
  simRun(2500);
  CHECK(g_ball.conf > g_cfg.aggressiveConf);
  CHECK(sqrtf(g_reqVx * g_reqVx + g_reqVy * g_reqVy) > g_cfg.approachSpeed + 0.05f);
  CHECK(g_cond == COND_NORMAL);
}

static void t_nav_unreliable_slow() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(2500);
  float normal = sqrtf(g_reqVx * g_reqVx + g_reqVy * g_reqVy);
  g_simMode[8] = SIM_NOISE;                                 // interference on another channel
  simRun(4000);
  CHECK(g_ir[8].noisy);
  float reduced = sqrtf(g_reqVx * g_reqVx + g_reqVy * g_reqVy);
  CHECK(reduced < normal);
  CHECK(g_cond == COND_DEGRADED);
}

static void t_nav_lost_search() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(2000);
  CHECK(g_navState == NAV_APPROACH);
  clearBall();
  simRun(g_cfg.lostHoldMs + 400);
  CHECK(g_navState == NAV_SEARCH);
  CHECK(g_stats.ballLossEvents == 1);
  simRun(500);
  CHECK(fabsf(g_reqW) > 0.1 && g_reqVx == 0 && g_reqVy == 0);   // slow rotation in place
  CHECK(fabsf(g_reqW) <= g_cfg.searchSpin + 1e-6);
  CHECK(!faultActive(F_IR_CHANNEL) && g_cond == COND_NORMAL);   // losing the ball is not a fault
  // reacquire during the search
  ballAt(180, 15);
  simRun(2500);
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_stats.reacquisitions == 1 && g_stats.reacquireTotalMs > 0);
  // bounded search: exhaust attempts and end up stopped, never spinning forever
  clearBall();
  simRun(g_cfg.lostHoldMs + 200);
  uint32_t perAttempt = g_cfg.searchTimeoutMs + g_cfg.searchPauseMs + 500;
  simRun(perAttempt * (g_cfg.searchAttempts + 1));
  CHECK(g_navState == NAV_WAIT);
  CHECK(allWheelsStopped());
  CHECK(g_autoEnabled);
  ballAt(30, 15);
  simRun(2500);
  CHECK(g_navState == NAV_APPROACH);                         // watching while stopped, resumes on its own
  clearBall();
  simRun(g_cfg.lostHoldMs + 200 + perAttempt * (g_cfg.searchAttempts + 1) + 61000);
  CHECK(g_navState == NAV_IDLE && !g_autoEnabled);           // gives up after a long wait
}

static void t_nav_hysteresis() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 40);
  simRun(2500);
  // a ball hovering around the centre boundary must not flip the centred flag every cycle
  int flips = 0; bool last = g_centered;
  for (int i = 0; i < 40; i++) {
    ballAt((i & 1) ? 17.0f : 23.0f, 12);
    simRun(150);
    if (g_centered != last) { flips++; last = g_centered; }
  }
  CHECK(flips <= 4);
}

static void t_nav_isolated_samples() {
  bootCal();
  simRun(500);
  startAuto();
  clearBall();
  // single isolated detections (one window) on random channels never start the approach
  for (int k = 0; k < 20; k++) {
    int ch = (k * 5) % 12;
    g_simMode[ch] = SIM_BALL; simRun(36); g_simMode[ch] = SIM_IDLE; simRun(500);
  }
  CHECK(g_stats.ballLossEvents == 0);
  CHECK(g_navState == NAV_SEARCH || g_navState == NAV_SEEK || g_navState == NAV_SEARCH_PAUSE || g_navState == NAV_WAIT);
  CHECK(g_navState != NAV_APPROACH);
}

static void t_safe_stop_moving() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(2500);
  CHECK(!allWheelsStopped());
  for (int i = 0; i < 12; i++) g_simMode[i] = SIM_STUCK_ACTIVE;   // every receiver wedged: sensing lost
  uint64_t t0 = g_mockUs;
  while (g_cond != COND_SAFE_STOP && g_mockUs - t0 < 30000000ULL) { loop(); mockAdvance(50); }
  CHECK(g_cond == COND_SAFE_STOP);
  loop();
  CHECK(allWheelsStopped());                                // commanded motion stopped at once, no ramp
  CHECK(!g_autoEnabled);
  CHECK(g_stats.safetyStops >= 1);
  CHECK(outHas("SAFE_STOP entered"));
}

static void t_loop_stall() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(2500);
  CHECK(!allWheelsStopped());
  mockAdvance(1500000);                                     // loop blocked for 1.5 s
  loop();
  CHECK(allWheelsStopped());
  CHECK(faultActive(F_LOOP_STALL) && g_cond == COND_SAFE_STOP && !g_autoEnabled);
  simRun(5000);
  CHECK(g_cond == COND_SAFE_STOP);                          // latched: does not self-clear
  send("f"); simRun(100);
  CHECK(allWheelsStopped());                                // manual motion refused too
  send(":recover\n"); simRun(1500);
  CHECK(!faultActive(F_LOOP_STALL) && g_cond != COND_SAFE_STOP);
  CHECK(!g_autoEnabled);                                    // explicit 'a' still required
}

static void t_resets() {
  g_mockReset = ESP_RST_BROWNOUT;
  setup();
  CHECK(faultActive(F_BROWNOUT_RESET));
  CHECK(!g_autoEnabled && g_navState == NAV_IDLE && allWheelsStopped());
  CHECK(outHas("BROWNOUT"));
  CHECK(g_cond == COND_NORMAL);                             // a warning, not a degradation
  simRun(5000);
  CHECK(!g_autoEnabled && allWheelsStopped());
  send(":recover\n"); simRun(100);
  CHECK(!faultActive(F_BROWNOUT_RESET));
}

static void t_reset_wdt() {
  g_mockReset = ESP_RST_TASK_WDT;
  setup();
  CHECK(faultActive(F_ABNORMAL_RESET) && !g_autoEnabled && allWheelsStopped());
  CHECK(outHas("TASK_WDT"));
}

// ================================================================================================
// serial interface
static void t_cli_manual() {
  bootCal();
  simRun(300);
  startAuto();
  CHECK(g_autoEnabled);
  send("f"); simRun(150);
  CHECK(!g_autoEnabled && g_navState == NAV_MANUAL);        // manual disables autonomy
  CHECK(wheelDir(0) > 0 && wheelDir(1) > 0 && wheelDir(2) > 0 && wheelDir(3) > 0);
  simRun(g_cfg.manualHoldMs + 1500);                        // stops itself; does NOT resume autonomy
  CHECK(allWheelsStopped() && !g_autoEnabled && g_navState == NAV_IDLE);
  send("b"); simRun(400);  CHECK(wheelDir(0) < 0 && wheelDir(1) < 0 && wheelDir(2) < 0 && wheelDir(3) < 0);
  send("r"); simRun(400);  CHECK(wheelsMatchStrafeRight());
  send("l"); simRun(400); CHECK(wheelDir(0) < 0 && wheelDir(1) > 0 && wheelDir(2) > 0 && wheelDir(3) < 0);
  send("e"); simRun(400); CHECK(wheelDir(0) > 0 && wheelDir(1) < 0 && wheelDir(2) > 0 && wheelDir(3) < 0);
  send("q"); simRun(400); CHECK(wheelDir(0) < 0 && wheelDir(1) > 0 && wheelDir(2) < 0 && wheelDir(3) > 0);
  send("f"); simRun(100);
  send("s"); simRun(10);
  CHECK(allWheelsStopped() && !g_autoEnabled && g_navState == NAV_IDLE);
  send("a"); simRun(100);                                   // 'a' only works with calibrated bearings and a healthy robot
  CHECK(g_autoEnabled);
  send("s"); simRun(10);
  CHECK(!g_autoEnabled && allWheelsStopped());
  // '!' emergency stop
  send("f"); simRun(100); send("!"); simRun(10);
  CHECK(allWheelsStopped());
  // 'a' refused with uncalibrated bearings
  for (int i = 0; i < 12; i++) g_cfg.ch[i].bearingCal = 0;
  Serial.out.clear(); send("a"); simRun(100);
  CHECK(!g_autoEnabled && outHas("REFUSED") && outHas("calibrated bearing"));
}

static void t_cli_robust() {
  bootCal();
  simRun(300);
  anyWheelEverMoved = false;
  // CR/LF, tabs, spaces, NULs, high bytes, unknown letters: nothing may move
  send("\r\n\r\n \t\n");
  std::string junk;
  for (int i = 0; i < 256; i++) { char ch = (char)i; if (strchr("fblrqea!:\r\n", ch)) continue; junk.push_back(ch); }
  send(junk.c_str());
  std::string withNul(junk); withNul.insert(10, 1, '\0');
  for (unsigned char ch : withNul) Serial.in.push_back(ch);
  simRunTracking(800);
  CHECK(!anyWheelEverMoved);
  CHECK(outHas("ignored unknown input"));
  CHECK(outCount("ignored unknown input") <= 2);           // rate limited, not a flood
  // overlong ':' line is rejected, not executed, not overflowing
  std::string longLine = ":set maxauto ";
  longLine.append(300, '9');
  longLine += "\r\n";
  Serial.out.clear();
  send(longLine.c_str()); simRun(100);
  CHECK(outHas("too long"));
  CHECK(g_cfg.maxAuto == 170);
  // control characters inside a line -> rejected
  Serial.out.clear();
  Serial.in.push_back(':'); for (char ch : std::string("set maxauto 150")) Serial.in.push_back((uint8_t)ch);
  Serial.in.push_back(0x07); Serial.in.push_back('\n'); simRun(100);
  CHECK(outHas("invalid characters") && g_cfg.maxAuto == 170);
  // motion letters inside a ':' line are data, not commands
  anyWheelEverMoved = false;
  send(":set bogus fff\n"); simRunTracking(300);
  CHECK(!anyWheelEverMoved);
  // an unterminated ':' must not swallow a later 's' forever
  send(":"); simRun(4500);
  CHECK(outHas("abandoned"));
  send("f"); simRun(100); send("s"); simRun(20);
  CHECK(allWheelsStopped());
  // '!' works even inside a line
  send("f"); simRun(100); send(":se"); send("!"); simRun(20);
  CHECK(allWheelsStopped());
  // fuzz with a safe alphabet (no motion/auto letters): never moves, never crashes
  srand(7);
  anyWheelEverMoved = false;
  const char alphabet[] = "ghimtxdIs:?\r\n 0123456789abcdefghijklmnopqrstuvwxyz";
  for (int k = 0; k < 4000; k++) {
    char ch = alphabet[rand() % (sizeof(alphabet) - 1)];
    if (strchr("fblrqea", ch)) continue;
    Serial.in.push_back((uint8_t)ch);
    if (k % 50 == 0) simRunTracking(5);
  }
  simRunTracking(500);
  CHECK(!anyWheelEverMoved);
}

static void t_diag_no_motion() {
  bootCal();
  simRun(300);
  anyWheelEverMoved = false;
  const char* cmds[] = {"i", "m", "h", "d", "g", "?", "I", "t", "x", ":get\n", ":cov\n", ":xsel ring\n", ":help\n"};
  for (const char* c : cmds) { send(c); simRunTracking(200); }
  ballAt(0, 30);
  simRunTracking(1500);
  CHECK(!anyWheelEverMoved);
  CHECK(outHas("IR snapshot") && outHas("MPU6050"));
  CHECK(outHas("HARDWARE HEALTH REPORT"));
  CHECK(outHas("state=IDLE"));
  CHECK(outHas("persist=ON"));
  CHECK(outHas("TEST MODE"));
  CHECK(outHas("commands (115200 baud)"));
  CHECK(outHas("no distance or signal strength"));
  CHECK(!outHas("distance:"));                              // no fabricated distance anywhere
  CHECK(g_streamIr);                                        // 'I' toggled the live view on
  send("I"); simRun(100);
  CHECK(!g_streamIr);
}

static void t_live_ir_rate() {
  bootCal();
  simRun(300);
  Serial.out.clear();
  send("I"); simRun(5000);
  int lines = outCount("cyc=");
  CHECK(lines >= 8 && lines <= 11);                         // ~streamMs=500 -> 10 lines in 5 s, not unlimited
  send("I"); simRun(100);
}

static void t_motor_test() {
  bootCal();
  simRun(300);
  anyWheelEverMoved = false;
  send(":mtest FL\n"); simRunTracking(1000);
  CHECK(g_mtest.state == 1 && !anyWheelEverMoved);          // armed only
  CHECK(outHas("LIFT THE WHEELS"));
  send(":confirm\n"); simRun(500);
  CHECK(wheelDir(W_FL) > 0 && wheelDir(W_FR) == 0 && wheelDir(W_BL) == 0 && wheelDir(W_BR) == 0);
  CHECK(g_pwmDuty[23] > 0 && g_pwmDuty[23] <= g_cfg.testPwm);
  simRun(1300);                                             // pause then reverse
  simRun(800);
  CHECK(wheelDir(W_FL) < 0 && wheelDir(W_FR) == 0);
  simRun(4000);
  CHECK(allWheelsStopped() && g_mtest.state == 0 && g_navState == NAV_IDLE);
  CHECK(outHas("Did the wheel turn FORWARD"));
  send(":mrev FL\n"); simRun(10);
  CHECK(g_cfg.reverse[0] == 1 && !(g_cfg.motorVerified & 1));
  send(":mok FL\n"); simRun(10);
  CHECK(g_cfg.motorVerified & 1);
  // confirm without arming does nothing
  anyWheelEverMoved = false;
  send(":confirm\n"); simRunTracking(500);
  CHECK(!anyWheelEverMoved);
  // arming times out without confirmation
  send(":mtest BR\n"); simRunTracking(16000);
  CHECK(g_mtest.state == 0 && !anyWheelEverMoved);
  // abort during the run
  send(":mtest BR\n"); simRun(100); send(":confirm\n"); simRun(300);
  CHECK(wheelDir(W_BR) != 0);
  send("s"); simRun(20);
  CHECK(allWheelsStopped() && g_mtest.state == 0);
  // motor test while in SAFE_STOP due to a sensing fault is allowed on the bench
  for (int i = 0; i < 12; i++) { char b[40]; snprintf(b, sizeof(b), ":ch %d en 0\n", i); send(b); }
  simRun(300);
  CHECK(g_cond == COND_SAFE_STOP);
  send(":mtest FR\n"); simRun(100); send(":confirm\n"); simRun(400);
  CHECK(wheelDir(W_FR) > 0);
  send("s"); simRun(20);
  CHECK(allWheelsStopped());
}

static void t_cfg_set() {
  bootCal();
  simRun(200);
  send(":set maxauto 150\n"); simRun(50);
  CHECK(g_cfg.maxAuto == 150);
  send(":set maxauto 5\n"); simRun(50);                     // out of range
  CHECK(g_cfg.maxAuto == 150 && outHas("rejected"));
  send(":set minpwm 119\n"); simRun(50);                    // would exceed the speed limits: cross-field
  CHECK(g_cfg.minPwm == 40);
  send(":set acqconf nan\n"); simRun(50);
  CHECK(g_cfg.acqConf == 0.45f);
  send(":set nonsense 3\n"); simRun(50);
  send(":rev FR 1\n"); simRun(50);
  CHECK(g_cfg.reverse[1] == 1);
  send(":roller O\n"); simRun(50);
  CHECK(g_cfg.rollerO == 1);
  send(":ch 3 bear 400\n"); simRun(50);
  CHECK(g_cfg.ch[3].bearingDeg == 90);                      // rejected
  send(":ch 3 bear 85\n"); simRun(50);
  CHECK(g_cfg.ch[3].bearingDeg == 85 && g_cfg.ch[3].bearingCal);
  send(":save\n"); simRun(100);
  CHECK(outHas("configuration saved"));
  send(":defaults\n"); simRun(50);
  CHECK(g_cfg.maxAuto == 170 && g_cfg.reverse[1] == 0);
  send(":load\n"); simRun(50);
  CHECK(g_cfg.maxAuto == 150 && g_cfg.reverse[1] == 1 && g_cfg.rollerO == 1 && g_cfg.ch[3].bearingDeg == 85);
}

// ================================================================================================
// storage / logging
static void t_fs_missing() {
  freshFs("fsmissing");
  mkdir(g_fsDir.c_str(), 0755);
  { std::string p = g_fsDir + "/precious.txt"; FILE* f = fopen(p.c_str(), "w"); fputs("keep me", f); fclose(f); }
  g_fsMountOk = false;
  bootCal();
  CHECK(!g_fsMounted && !g_logPersist);
  CHECK(faultActive(F_FS_UNAVAILABLE) && g_cond == COND_DEGRADED);
  CHECK(outHas("persistent logging OFF"));
  { std::string p = g_fsDir + "/precious.txt"; FILE* f = fopen(p.c_str(), "r"); CHECK(f != nullptr); if (f) fclose(f); }   // never formatted
  // everything else still works
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(2500);
  CHECK(g_navState == NAV_APPROACH && !allWheelsStopped());
  CHECK(g_frCount > 10);                                    // RAM recorder still running
  send("g"); simRun(300);
  CHECK(outHas("persist=OFF"));
  send(":xsel ring\n"); send("x"); simRun(3000);
  CHECK(outHas("#BEGIN-EXPORT name=ring") && outHas("#END-EXPORT"));
  g_fsMountOk = true;
}

static void t_fs_write_failure() {
  freshFs("fsfail");
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(3000);
  CHECK(g_logPersist && g_bytesWritten > 0);
  g_fsFailWrites = true;
  simRun(40000);                                            // keep logging into a dead filesystem
  CHECK(g_fsFailTotal >= 1);
  CHECK(faultActive(F_FS_WRITE) && g_cond == COND_DEGRADED);
  CHECK(!g_logPersist);                                     // gave up after repeated failures
  CHECK(g_stats.storageFailures >= 5);
  CHECK(g_navState == NAV_APPROACH && !allWheelsStopped());   // robot unaffected
  CHECK(g_cond != COND_SAFE_STOP);
  CHECK(g_frCount > 10);
  CHECK(outHas("FS_WRITE"));
  g_fsFailWrites = false;
}

static void t_fs_full() {
  freshFs("fsfull");
  g_fsTotal = 60000;                                        // tiny partition: below reserve + record size quickly
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(30000);
  CHECK(g_cond != COND_SAFE_STOP);
  CHECK(g_fsFailTotal >= 1 || !g_logPersist);
  CHECK(g_navState == NAV_APPROACH);
  g_fsTotal = 1500000;
}

static void t_log_rotation() {
  freshFs("rotate");
  bootCal();
  g_cfg.fileMaxBytes = 4096; g_cfg.maxFiles = 3; g_cfg.snapshotMs = 50; g_cfg.flushMs = 500; g_cfg.fsReserveBytes = 16384;
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(60000);
  CHECK(g_logPersist && g_filesCreated >= 5);
  CHECK(g_filesDeleted >= 2);
  LogFile lf[LOG_LIST_MAX];
  uint8_t n = logList(lf);
  CHECK(n <= g_cfg.maxFiles);
  CHECK(n >= 2);
  for (uint8_t i = 0; i < n; i++) CHECK(lf[i].size <= g_cfg.fileMaxBytes + 300);
  // every file: header, column row, constant column count
  int expectCols = 0; for (const char* p = LOG_COLUMNS; *p; p++) if (*p == ',') expectCols++;
  for (uint8_t i = 0; i < n; i++) {
    std::string path = g_fsDir + lf[i].name;
    FILE* f = fopen(path.c_str(), "r");
    CHECK(f != nullptr);
    if (!f) continue;
    char line[400]; int ln = 0; bool ok = true;
    while (fgets(line, sizeof(line), f)) {
      ln++;
      if (ln == 1) { ok = ok && strncmp(line, "#schema=1", 9) == 0; continue; }
      if (ln == 2) { ok = ok && strncmp(line, "t_ms,sess", 9) == 0; continue; }
      int cols = 0; for (char* p = line; *p; p++) if (*p == ',') cols++;
      if (cols != expectCols) { ok = false; printf("  bad cols %d in %s: %s", cols, lf[i].name, line); }
    }
    fclose(f);
    CHECK(ok);
  }
  // only our own log files are ever deleted
}

static void t_log_queue_overflow() {
  bootCal();
  uint32_t d0 = g_logDropped;
  for (int i = 0; i < 2000; i++) logRow('S', "flood");     // 2000 rows into a 6 KB queue with no writer running
  CHECK(g_logDropped > d0);
  CHECK(logqUsed() < LOGQ_SIZE);
  // serial queue flood is bounded and counted too
  uint32_t o0 = g_outDropped;
  g_bootPhase = false;
  for (int i = 0; i < 2000; i++) say("INFO", "flooding the serial queue with a moderately long line number %d", i);
  CHECK(g_outDropped > o0);
  CHECK(outUsed() < OUTQ_SIZE);
}

static void t_flight_recorder() {
  freshFs("fr");
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(6000);                                             // fill the ring with normal history
  uint16_t pre = g_frCount;
  CHECK(pre > 40);
  Serial.out.clear();
  g_mpu.mode = 1;                                           // IMU disconnect => trigger
  simRun(500);
  CHECK(g_frCapturing);
  CHECK(faultActive(F_IMU_TIMEOUT));
  simRun(g_cfg.frPostMs + 500);
  CHECK(!g_frCapturing && g_frCaptures == 1);
  CHECK(g_frFrozenCount >= pre);
  CHECK(outHas("flight recorder captured"));
  // frozen window holds records from BEFORE the event and AFTER it
  uint32_t trig = 0;
  for (int i = 0; i < g_frFrozenCount; i++) if (g_frFrozen[i].type == 'E' && strstr(g_frFrozen[i].note, "IMU")) { trig = g_frFrozen[i].t; break; }
  CHECK(trig != 0);
  int before = 0, after = 0;
  for (int i = 0; i < g_frFrozenCount; i++) { if (g_frFrozen[i].t < trig) before++; else if (g_frFrozen[i].t > trig) after++; }
  CHECK(before > 20 && after > 10);
  // chronological order
  for (int i = 1; i < g_frFrozenCount; i++) CHECK((int32_t)(g_frFrozen[i].t - g_frFrozen[i - 1].t) >= 0);
  // ring is bounded
  simRun(20000);
  CHECK(g_frCount <= FR_N);
  // dump reached the CSV queue / file as type B rows
  simRun(5000);
  bool foundB = false;
  { LogFile lf[LOG_LIST_MAX]; uint8_t n = logList(lf);
    for (uint8_t i = 0; i < n; i++) { std::string p = g_fsDir + lf[i].name; FILE* f = fopen(p.c_str(), "r"); char line[400]; while (f && fgets(line, sizeof(line), f)) { const char* c = line; int commas = 0; while (*c && commas < 3) { if (*c == ',') commas++; c++; } if (commas == 3 && *c == 'B') foundB = true; } if (f) fclose(f); } }
  CHECK(foundB);
  // a second trigger during capture is merged, not duplicated
  uint32_t cap0 = g_frCaptures;
  frTrigger("one"); frTrigger("two");
  CHECK(g_frIgnored >= 1);
  simRun(g_cfg.frPostMs + 200);
  CHECK(g_frCaptures == cap0 + 1);
}

static uint32_t parseExportAndCrc(const std::string& out, const char* beginTag, bool* ok, int* lines) {
  *ok = false; *lines = 0;
  size_t b = out.find(beginTag);
  if (b == std::string::npos) return 0;
  size_t s = out.find('\n', b) + 1;
  size_t e = out.find("#END-EXPORT", s);
  if (e == std::string::npos) return 0;
  uint32_t crc = 0; int n = 0;
  size_t p = s;
  while (p < e) {
    size_t nl = out.find('\n', p);
    std::string line = out.substr(p, nl - p + 1);
    if (line[0] != '[') { crc = crc32Update(crc, (const uint8_t*)line.data(), line.size()); n++; }
    p = nl + 1;
  }
  unsigned want = 0; int wl = 0;
  sscanf(out.c_str() + e, "#END-EXPORT lines=%d crc32=%x", &wl, &want);
  *ok = (want == crc) && wl == n;
  *lines = n;
  return crc;
}

static void t_export() {
  freshFs("export");
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  g_cfg.flushMs = 500; g_cfg.snapshotMs = 100;
  simRun(8000);
  send("g"); simRun(500);
  CHECK(outHas("[0] /L"));
  send(":xsel 0\n"); simRun(50);
  Serial.out.clear();
  send("x"); simRun(20000);
  bool ok; int lines;
  parseExportAndCrc(Serial.out, "#BEGIN-EXPORT", &ok, &lines);
  CHECK(ok && lines > 20);
  CHECK(outHas("#BEGIN-EXPORT name=/L") && outHas("schema=1"));
  // ring export matches the in-RAM data and its CRC verifies
  Serial.out.clear();
  send(":xsel ring\n"); send("x"); simRun(20000);
  parseExportAndCrc(Serial.out, "#BEGIN-EXPORT name=ring", &ok, &lines);
  CHECK(ok && lines > 20);
  // export of a missing file is refused politely
  Serial.out.clear();
  send(":xsel 9\n"); send("x"); simRun(300);
  CHECK(outHas("export unavailable"));
}

static void t_nonblocking() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(2000);
  g_bootPhase = false;
  // flood diagnostics while a slow UART drains: every loop() call must stay short in (virtual) time
  uint64_t worst = 0;
  const char* floods[] = {"h", "i", "h", "d", "m", "g", "h"};
  for (int rep = 0; rep < 20; rep++) {
    for (const char* f : floods) send(f);
    for (int k = 0; k < 200; k++) {
      uint64_t t0 = g_mockUs;
      loop();
      uint64_t dt = g_mockUs - t0;
      if (dt > worst) worst = dt;
      mockAdvance(50);
    }
  }
  CHECK(worst < 300);                                       // only the 10 us mux settle delays, never a wait on Serial
  CHECK(g_outDropped > 0 || outUsed() > 0);                 // back-pressure shows up as drops/queueing, not blocking
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_scan.cycleCount > 20);
  // the writer can be arbitrarily slow without delaying the control loop: it lives outside loop()
  freshFs("nb");
  g_hostWriterInline = false;
  g_cfg.snapshotMs = 50;
  simRun(5000);
  CHECK(logqUsed() > 0);                                    // data accumulates for the (separate) writer
  CHECK(g_navState == NAV_APPROACH);
  logPump(g_cfg, millis());                                 // the writer task would do this off-core
  g_hostWriterInline = true;
}

// ================================================================================================
// persistence across "reboots" (re-run as two processes)
static void t_reboot_phase1() {
  freshFs("reboot");
  bootCal();
  simRun(300);
  for (int i = 0; i < 12; i++) { char b[48]; snprintf(b, sizeof(b), ":ch %d bear %d\n", i, i * 30); send(b); }
  send(":set maxauto 133\n"); send(":rev BL 1\n"); send(":mok FL\n"); send(":save\n");
  simRun(300);
  startAuto();
  CHECK(g_autoEnabled);
  ballAt(0, 10);
  g_cfg.flushMs = 500;
  simRun(5000);
  CHECK(!allWheelsStopped());
  CHECK(g_session == 1);
  CHECK(g_bytesWritten > 0);
  // the process now "loses power" while driving autonomously
}

static void t_reboot_phase2() {
  g_fsDir = "/tmp/mockfs_reboot";
  setup();
  CHECK(!g_autoEnabled);                                    // reboot never resumes aggressive movement
  CHECK(g_navState == NAV_IDLE && allWheelsStopped());
  CHECK(g_cfg.maxAuto == 133 && g_cfg.reverse[2] == 1 && (g_cfg.motorVerified & 1));
  CHECK(g_cfg.ch[3].bearingCal == 1 && g_cfg.ch[3].bearingDeg == 90);
  CHECK(g_session == 2);
  LogFile lf[LOG_LIST_MAX]; uint8_t n = logList(lf);
  bool hasSession1 = false;
  for (uint8_t i = 0; i < n; i++) if (strncmp(lf[i].name, "/L00001_", 8) == 0) hasSession1 = true;
  CHECK(hasSession1);                                       // previous session's log was not erased
  ballAt(0, 20);
  anyWheelEverMoved = false;
  simRunTracking(4000);
  CHECK(!anyWheelEverMoved);                                // sees the ball, still does not move
  send("a"); simRun(50);
  CHECK(g_autoEnabled);                                     // only an explicit 'a'
}

static void t_cal_startup_defaults() {
  setup();
  CHECK(navCalibratedTrusted(g_cfg) == 0);
  Serial.out.clear();
  send("a"); simRun(50);
  CHECK(!g_autoEnabled);                                    // placeholder bearings are never trusted for autonomy
  CHECK(outHas("calibrated bearing"));
}

static void t_health_report_text() {
  bootCal();
  simRun(500);
  Serial.out.clear();
  send("h"); simRun(1500);
  CHECK(outHas("NOT MEASURABLE (no voltage monitor installed)"));
  CHECK(outHas("NOT MEASURABLE (no encoders, no current sensing)"));
  CHECK(outHas("NOT TESTED"));
  CHECK(outHas("TSOP1738 vs 40 kHz ball compatibility: NOT TESTED"));
  CHECK(outHas("reserved for future receivers: disabled"));
  CHECK(outHas("(UNCALIBRATED)") == false);                // bootCal calibrated them
  CHECK(outHas("MPU6050 @0x68 who=0x68: PASS"));
  CHECK(outHas("PWM shown is COMMANDED"));
  CHECK(outHas("reset reason"));
  CHECK(!outHas("PASS (operator-confirmed") );             // nothing is claimed verified
}

// ================================================================================================
struct TestCase { const char* name; void (*fn)(); };
#define T(n) {#n, n}
static const TestCase TESTS[] = {
  T(t_angles), T(t_timer_rollover), T(t_mecanum), T(t_pinmap), T(t_boot_safe), T(t_ramp), T(t_config), T(t_classify),
  T(t_ir_detect), T(t_ir_wrap), T(t_ir_ambiguous), T(t_ir_stuck_and_recover), T(t_ir_noise), T(t_ir_polarity),
  T(t_ir_system_loss), T(t_ir_trusted_low), T(t_ir_scan_stall), T(t_ir_overrun), T(t_ir_coverage), T(t_ir_probe),
  T(t_imu_integrate), T(t_imu_bias), T(t_imu_missing), T(t_imu_dropout), T(t_imu_invalid), T(t_heading_hold),
  T(t_nav_approach), T(t_nav_unreliable_slow), T(t_nav_lost_search), T(t_nav_hysteresis), T(t_nav_isolated_samples),
  T(t_safe_stop_moving), T(t_loop_stall), T(t_resets), T(t_reset_wdt),
  T(t_cli_manual), T(t_cli_robust), T(t_diag_no_motion), T(t_live_ir_rate), T(t_motor_test), T(t_cfg_set),
  T(t_fs_missing), T(t_fs_write_failure), T(t_fs_full), T(t_log_rotation), T(t_log_queue_overflow),
  T(t_flight_recorder), T(t_export), T(t_nonblocking), T(t_reboot_phase1), T(t_reboot_phase2),
  T(t_cal_startup_defaults), T(t_health_report_text),
};

int main(int argc, char** argv) {
  if (argc >= 2 && strcmp(argv[1], "--list") == 0) { for (auto& t : TESTS) printf("%s\n", t.name); return 0; }
  if (argc < 2) { fprintf(stderr, "usage: %s <test>|--list\n", argv[0]); return 2; }
  for (auto& t : TESTS) {
    if (strcmp(t.name, argv[1]) != 0) continue;
    if (strncmp(t.name, "t_reboot", 8) != 0 && strcmp(t.name, "t_config") != 0 && strncmp(t.name, "t_fs_", 5) != 0 &&
        strcmp(t.name, "t_log_rotation") != 0 && strcmp(t.name, "t_flight_recorder") != 0 && strcmp(t.name, "t_export") != 0)
      freshFs(t.name);
    t.fn();
    if (getenv("SHOW_OUT")) fwrite(Serial.out.data(), 1, Serial.out.size(), stdout);
    printf("%s %s (%d checks, %d failed)\n", g_fail ? "FAIL" : "PASS", t.name, g_checks, g_fail);
    return g_fail ? 1 : 0;
  }
  fprintf(stderr, "no such test %s\n", argv[1]);
  return 2;
}
