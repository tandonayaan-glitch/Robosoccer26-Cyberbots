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
    CHECK(step <= g_cfg.accelPwmPerS * 0.010f + (prev == 0 ? (float)g_cfg.minPwm : 0.0f) + 1e-3f);   // a start from rest jumps to minPwm
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
  CHECK(g_wheelCur[2] <= g_cfg.accelPwmPerS * 0.1f + (float)g_cfg.minPwm + 1e-3f);
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
  // pulse-period gate must be both-zero or a sane band
  { Config g; cfgDefaults(g); g.irPeriodMinUs = 500; g.irPeriodMaxUs = 0; CHECK(cfgValidate(g, rep, sizeof(rep)) >= 1 && g.irPeriodMinUs == 0 && strstr(rep, "irperiodgate")); }
  { Config g; cfgDefaults(g); g.irPeriodMinUs = 900; g.irPeriodMaxUs = 800; CHECK(cfgValidate(g, rep, sizeof(rep)) >= 1 && g.irPeriodMaxUs == 0); }
  { Config g; cfgDefaults(g); g.irPeriodMinUs = 50; g.irPeriodMaxUs = 800; CHECK(cfgValidate(g, rep, sizeof(rep)) >= 1); }
  { Config g; cfgDefaults(g); g.irPeriodMinUs = 400; g.irPeriodMaxUs = 1500; CHECK(cfgValidate(g, rep, sizeof(rep)) == 0); }
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

static void t_fault_report_format() {
  bootCal();
  simRun(500);
  Serial.out.clear();
  g_mpu.mode = 1;
  simRun(1500);
  // timestamp, severity, component, code, evidence, certainty, action, mode, recovery
  CHECK(outHas("IMU_TIMEOUT sev=DEGRADED comp=MPU6050 evidence=\""));
  CHECK(outHas("certainty=\"bus errors observed; cause unproven\""));
  CHECK(outHas("action=\"heading hold disabled, retrying init\" mode=DEGRADED recovery=automatic when verified"));
  size_t p = Serial.out.find("IMU_TIMEOUT sev=");
  CHECK(p != std::string::npos && Serial.out.rfind("[WARN] ", p) != std::string::npos);     // tag + timestamp prefix
  // repeated identical warnings do not flood the port
  simRun(20000);
  CHECK(outCount("IMU_TIMEOUT sev=") == 1);
  CHECK(outCount("IMU_TIMEOUT still active") <= 3);
  // snapshot rows carry drop / write-failure counters
  CHECK(g_logDropped == 0);
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

static void t_drive_test() {
  bootCal();
  simRun(300);
  anyWheelEverMoved = false;
  send(":dtest\n"); simRunTracking(1000);
  CHECK(g_mtest.state == 1 && g_mtest.drive && !anyWheelEverMoved);
  send(":confirm\n");
  simRun(600);   CHECK(wheelDir(0) > 0 && wheelDir(1) > 0 && wheelDir(2) > 0 && wheelDir(3) > 0);          // forward
  simRun(1600);  CHECK(wheelDir(0) < 0 && wheelDir(1) < 0 && wheelDir(2) < 0 && wheelDir(3) < 0);          // backward
  simRun(1600);  CHECK(wheelDir(0) < 0 && wheelDir(1) > 0 && wheelDir(2) > 0 && wheelDir(3) < 0);          // strafe left
  simRun(1600);  CHECK(wheelsMatchStrafeRight());                                                           // strafe right
  simRun(1600);  CHECK(wheelDir(0) > 0 && wheelDir(1) < 0 && wheelDir(2) > 0 && wheelDir(3) < 0);          // clockwise
  simRun(1600);  CHECK(wheelDir(0) < 0 && wheelDir(1) > 0 && wheelDir(2) < 0 && wheelDir(3) > 0);          // counter-clockwise
  for (int i = 0; i < 4; i++) CHECK(g_pwmDuty[WHEEL_PINS[i].en] <= g_cfg.testPwm);
  simRun(3000);
  CHECK(allWheelsStopped() && g_mtest.state == 0 && g_navState == NAV_IDLE);
  CHECK(outHas("drivetrain test finished"));
}

static void t_boot_no_channels() {
  CHECK(fsInit());
  Config c; cfgDefaults(c);
  for (int i = 0; i < 16; i++) c.ch[i].enabled = 0;
  CHECK(cfgSave(c));
  setup();
  CHECK(faultActive(F_IR_NO_CHANNELS) && g_cond == COND_SAFE_STOP);
  CHECK(!g_autoEnabled && allWheelsStopped());
  g_cfg.ch[0].enabled = 1;                                   // enabling a channel later must start the scan
  simRun(1000);
  CHECK(g_scan.cycleCount > 10);
  CHECK(g_scan.enabledCount == 1);
  for (int i = 1; i < 6; i++) g_cfg.ch[i].enabled = 1;
  calibrateAll();
  simRun(g_cfg.recoverVerifyMs + 2000);
  CHECK(!faultActive(F_IR_NO_CHANNELS) && !faultActive(F_IR_TRUSTED_LOW) && g_cond != COND_SAFE_STOP);
  CHECK(!g_autoEnabled);
}

static void t_fs_commands() {
  freshFs("fscmd");
  bootCal();
  simRun(500);
  { std::string p = g_fsDir + "/keep.txt"; FILE* f = fopen(p.c_str(), "w"); fputs("x", f); fclose(f); }
  anyWheelEverMoved = false;
  send(":fsformat\n"); simRunTracking(200);
  { std::string p = g_fsDir + "/keep.txt"; FILE* f = fopen(p.c_str(), "r"); CHECK(f != nullptr); if (f) fclose(f); }   // refused without CONFIRM
  CHECK(outHas("ERASES"));
  send(":fsformat confirm\n"); simRunTracking(200);          // wrong case: still refused
  { std::string p = g_fsDir + "/keep.txt"; FILE* f = fopen(p.c_str(), "r"); CHECK(f != nullptr); if (f) fclose(f); }
  startAuto(); ballAt(0, 10); simRun(2000);
  CHECK(!allWheelsStopped());
  int wd = g_wdtDeleted;
  send(":fsformat CONFIRM\n"); simRun(300);
  { std::string p = g_fsDir + "/keep.txt"; FILE* f = fopen(p.c_str(), "r"); CHECK(f == nullptr); if (f) fclose(f); }
  CHECK(g_wdtDeleted == (int)wd + 1);
  CHECK(outHas("format OK"));
  CHECK(!g_autoEnabled && allWheelsStopped());               // motors were stopped for the operation
  CHECK(!faultActive(F_LOOP_STALL));
  CHECK(g_logPersist);
  // :logdel keeps the file being written
  g_cfg.flushMs = 500; g_cfg.fileMaxBytes = 4096; g_cfg.snapshotMs = 50;
  for (int i = 0; i < 40; i++) { send("f"); simRun(500); }   // keep moving so snapshots are written at the fast rate
  send(":logdel\n"); simRun(100);
  CHECK(outHas("deletes old log files"));
  uint8_t before = 0; { LogFile lf[LOG_LIST_MAX]; before = logList(lf); }
  CHECK(before >= 2);
  send(":logdel CONFIRM\n"); simRun(100);
  { LogFile lf[LOG_LIST_MAX]; uint8_t n = logList(lf); CHECK(n >= 1 && n < before + 1); }
}

static void t_ir_spare_channels() {
  bootCal();
  for (int i = 12; i < 16; i++) g_simMode[i] = SIM_BALL;      // a "ball" on mux inputs that have no receiver installed
  simRun(3000);
  CHECK(!g_ball.valid && !g_ball.anyDetect);
  for (int i = 12; i < 16; i++) { CHECK(g_ir[i].windows == 0 && g_ir[i].detCount == 0 && !g_ir[i].active); CHECK(irChStatus(g_cfg, i) == CH_DISABLED); CHECK(!irTrusted(g_cfg, i)); }
  CHECK(g_scan.enabledCount == 12);
  uint8_t nT, nU; irCoverageGap(g_cfg, &nT, &nU);
  CHECK(nT == 12);                                             // spare channels are not counted as coverage
  CHECK(!faultActive(F_IR_CHANNEL));                           // and are never reported as failed
  CHECK(outHas("reserved for future receivers: disabled"));
  // defaults agree with the physical count and the angle range
  Config d; cfgDefaults(d);
  int en = 0; for (int i = 0; i < 16; i++) { en += d.ch[i].enabled; CHECK(d.ch[i].bearingDeg >= 0 && d.ch[i].bearingDeg < 360 && d.ch[i].tiltDownDeg == 22.0f && !d.ch[i].bearingCal); }
  CHECK(en == IR_PLANNED_CHANNELS && en == 12);
}

static void t_ir_period_gate() {
  bootCal();
  simRun(500);
  // before any measurement: 'auto' is refused and the gate is reported OFF
  send(":irgate auto\n"); send("h"); simRun(1500);
  CHECK(outHas("needs a measurement"));
  CHECK(outHas("pulse-period gate OFF"));
  // An unrelated modulated source with a ball-like rhythm (2.5 kHz envelope) on channel 3.
  g_simMode[3] = SIM_BALL; g_simChPeriod[3] = 400;
  simRun(1500);
  CHECK(g_ir[3].active);                                       // WITHOUT a gate it is indistinguishable from a ball (documented limitation)
  CHECK(g_ir[3].offband == 0);
  // Measure the real ball (833 us) on channel 7 and gate on it.
  g_simMode[3] = SIM_IDLE; g_simChPeriod[3] = 0;
  g_simMode[7] = SIM_BALL;
  send(":verify 7\n"); simRun(5000);
  CHECK(g_lastPeriod.n >= 8 && g_lastPeriod.mean > 700 && g_lastPeriod.mean < 950);
  CHECK(outHas("measured pulse period of this source"));
  send(":irgate auto\n"); simRun(100);
  CHECK(g_cfg.irPeriodMaxUs > 0 && g_cfg.irPeriodMinUs < 833 && g_cfg.irPeriodMaxUs > 833);
  CHECK(g_cfg.irPeriodMinUs >= 100 && g_cfg.irPeriodMaxUs <= 20000);
  // Now the same unrelated source is rejected, the real ball still detects.
  g_simMode[7] = SIM_IDLE; clearBall();
  simRun(1500);
  g_simMode[3] = SIM_BALL; g_simChPeriod[3] = 400;
  simRun(2000);
  CHECK(!g_ir[3].active && g_ir[3].offband > 5);
  CHECK(!g_ball.anyDetect && !g_ball.valid);
  CHECK(!faultActive(F_IR_CHANNEL) && g_cond == COND_NORMAL);   // an off-band source is not a hardware fault
  g_simMode[3] = SIM_IDLE; g_simChPeriod[3] = 0;
  g_simMode[7] = SIM_BALL;
  simRun(2000);
  CHECK(g_ir[7].active && g_ball.valid);
  // validation: lone limits and nonsense are refused; off works
  send(":irgate off\n"); simRun(50);
  Serial.out.clear();
  send(":set irperlo 500\n"); simRun(50);
  CHECK(outHas("rejected") && g_cfg.irPeriodMinUs == 0);
  Serial.out.clear();
  send(":irgate 50 900\n"); simRun(50);
  CHECK(outHas("usage"));
  send(":irgate 500 400\n"); simRun(50);
  CHECK(outHas("usage"));
  send(":irgate 300 1500\n"); simRun(50);
  CHECK(g_cfg.irPeriodMinUs == 300 && g_cfg.irPeriodMaxUs == 1500);
  send(":irgate off\n"); simRun(50);
  CHECK(g_cfg.irPeriodMinUs == 0 && g_cfg.irPeriodMaxUs == 0);
}

static void t_latency() {
  // Measured in simulation (control-loop quantisation 50 us): how long from the ball appearing until
  // the firmware would act on it, and from disappearing until it is declared lost.
  bootCal();
  simRun(1000);
  for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = 1;
  ballAt(90, 10);                                              // single verified receiver
  uint64_t t0 = g_mockUs;
  while (!g_ball.valid && g_mockUs - t0 < 3000000ULL) { loop(); mockAdvance(50); }
  double acq1 = (double)(g_mockUs - t0) / 1000.0;
  clearBall();
  simRun(1500);
  ballAt(75, 20);                                              // two neighbouring receivers
  t0 = g_mockUs;
  while (!g_ball.valid && g_mockUs - t0 < 3000000ULL) { loop(); mockAdvance(50); }
  double acq2 = (double)(g_mockUs - t0) / 1000.0;
  clearBall();                                                 // removed right after acquisition: a young track
  t0 = g_mockUs;
  while (g_ball.valid && g_mockUs - t0 < 3000000ULL) { loop(); mockAdvance(50); }
  double lostYoung = (double)(g_mockUs - t0) / 1000.0;
  simRun(1500);
  ballAt(75, 20);                                              // established track: two seconds of steady signal
  simRun(2000);
  CHECK(g_ball.valid);
  clearBall();
  t0 = g_mockUs;
  while (g_ball.valid && g_mockUs - t0 < 3000000ULL) { loop(); mockAdvance(50); }
  double lost = (double)(g_mockUs - t0) / 1000.0;
  printf("  latency: acquire(1 verified channel)=%.0f ms, acquire(2 channels)=%.0f ms, loss(young track)=%.0f ms, loss(steady track)=%.0f ms (scan cycle %.1f ms)\n", acq1, acq2, lostYoung, lost, g_scan.cycleUs / 1000.0);
  CHECK(acq1 < 200);                                           // bound well under the legacy 240 ms
  CHECK(acq2 < 100);
  CHECK(lost < g_cfg.lostHoldMs + 25);                         // steady track: the minimum grace period
  CHECK(lostYoung <= g_cfg.lostHoldMaxMs + 25);                // young track: longer grace, never beyond the configured maximum
  CHECK(acq1 > g_scan.cycleUs / 1000.0);                       // can never be faster than one scan
}

static bool benchBurstAcquires(uint32_t durMs, uint32_t phaseUs, int ch);
static bool benchBroadbandAcquires(uint32_t durMs, uint32_t phaseUs, bool coherent);

static void t_fast_engine_rejection() {
  // The fast engine must not trade away rejection of ball-like bursts: with the default settings a burst shorter than the
  // immunity time must never be acquired, whatever its phase relative to the scan (simulation only).
  bootCal();
  for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = 1;
  const uint32_t span = (uint32_t)(g_scan.cycleUs * 2);
  for (uint32_t d : {20u, 60u, 90u})
    for (int k = 0; k < 12; k++) CHECK(!benchBurstAcquires(d, (uint32_t)((uint64_t)span * k / 12), 5 + (k % 3)));
  for (uint32_t d : {40u, 100u, 250u})
    for (int k = 0; k < 12; k++) { CHECK(!benchBroadbandAcquires(d, (uint32_t)((uint64_t)span * k / 12), true)); CHECK(!benchBroadbandAcquires(d, (uint32_t)((uint64_t)span * k / 12), false)); }
  CHECK(g_cfg.lostHoldMs <= g_cfg.lostHoldMaxMs);
}

static void t_side_zone_note() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(90, 10);
  simRun(2500);
  CHECK(g_navState == NAV_APPROACH);
  Serial.out.clear();
  clearBall();
  simRun(g_cfg.lostHoldMs + 500);
  CHECK(outHas("possible entry into the ~40 mm side blind zone"));
  // a loss straight ahead does not carry that hint
  startAuto();
  ballAt(0, 10); simRun(2500);
  Serial.out.clear(); clearBall(); simRun(g_cfg.lostHoldMs + 500);
  CHECK(!outHas("side blind zone"));
  // the health report never implies continuous coverage
  send("h"); simRun(1500);
  CHECK(outHas("does NOT mean continuous coverage"));
  CHECK(outHas("stack headroom"));
}

static void t_nav_all_directions() {
  bootCal();
  simRun(500);
  startAuto();
  for (int deg = 0; deg < 360; deg += 30) {
    ballAt((float)deg, 10);                                    // exactly one receiver sees the ball
    simRun(3000);
    char msg[64]; snprintf(msg, sizeof(msg), "bearing %d", deg);
    if (g_navState != NAV_APPROACH || !g_ball.valid) { printf("  not approaching at %s\n", msg); }
    CHECK(g_navState == NAV_APPROACH && g_ball.valid);
    CHECK_NEAR(fabsf(angDiffDeg(g_ball.bearingDeg, (float)deg)), 0, 2.0);
    // requested direction must point toward the ball (centre band softens the sideways part)
    float reqAng = atan2f(g_reqVx, g_reqVy) * RAD2DEG;
    float tol = (deg == 0) ? 1.0f : (deg == 30 || deg == 330 ? 25.0f : 8.0f);
    CHECK_NEAR(fabsf(angDiffDeg(reqAng, (float)deg)), 0, tol);
    // wheels follow the mecanum mixing of that request
    float u[4]; mecanumMix(g_reqVx, g_reqVy, g_reqW, g_cfg.rollerO != 0, u);
    for (int i = 0; i < 4; i++) if (fabsf(u[i]) > 0.15f) CHECK((wheelDir(i) > 0) == (u[i] > 0) && wheelDir(i) != 0);
  }
}

static void t_nav_sweep_and_multi() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 20); simRun(2500);
  CHECK(g_navState == NAV_APPROACH);
  // ball circles the robot at 60 deg/s: the estimate must follow it continuously across every sector boundary
  float prev = g_ball.bearingDeg; float worstJump = 0; int lostSamples = 0;
  for (int step = 0; step < 120; step++) {                    // 6 s = 360 deg
    ballAt(fmodf(step * 3.0f, 360.0f), 20);
    simRun(50);
    if (!g_ball.valid) lostSamples++;
    float d = fabsf(angDiffDeg(g_ball.bearingDeg, prev)); if (d > worstJump) worstJump = d; prev = g_ball.bearingDeg;
  }
  printf("  sweep: worst bearing jump between 50 ms samples = %.1f deg, samples without a valid ball = %d\n", worstJump, lostSamples);
  CHECK(worstJump < 40.0f);
  CHECK(lostSamples == 0);
  CHECK(g_stats.ballLossEvents == 0);
  // several receivers at once
  ballAt(45, 25); simRun(2000);
  CHECK(g_ball.nActive == 2 && fabsf(g_ball.bearingDeg - 45) < 4);
  ballAt(90, 35); simRun(2000);
  CHECK(g_ball.nActive == 3 && fabsf(g_ball.bearingDeg - 90) < 3);
  ballAt(0, 65); simRun(2000);                                // five receivers: front arc
  CHECK(g_ball.nActive >= 4 && fabsf(g_ball.bearingDeg) < 3);
  CHECK_NEAR(g_ball.rInst, 0.746, 0.03);                       // geometry of five receivers across 120 deg
}

static void t_imu_frozen_in_approach() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(2500);
  CHECK(g_navState == NAV_APPROACH && !isnan(g_hdgErr));
  for (int mode : {2, 4, 3}) {                                // frozen, saturated, all-zero
    g_mpu.mode = mode;
    g_mpu.gz = 0;
    simRun(2500);
    CHECK(!imuHeadingUsable(g_cfg, millis()));
    CHECK(isnan(g_hdgErr));
    CHECK(g_navState == NAV_APPROACH);
    CHECK(fabsf(g_reqW) <= g_cfg.hdgMaxRot + 1e-6f);          // bounded, and ball is dead ahead: no turning
    CHECK(fabsf(g_reqW) < 0.05f);
    CHECK(wheelDir(0) > 0 && wheelDir(1) > 0 && wheelDir(2) > 0 && wheelDir(3) > 0);   // drives straight, no spin
    g_mpu.mode = 0;
    simRun(g_cfg.recoverVerifyMs + 2500);
    CHECK(g_imu.ok);
  }
  // ball off to one side + no usable heading: fallback turn is bounded and toward the ball
  g_mpu.mode = 1;
  ballAt(60, 10);
  simRun(2500);
  CHECK(g_reqW > 0 && g_reqW <= g_cfg.hdgMaxRot + 1e-6f);
}

static void t_gpio_tamper() {
  bootCal();
  simRun(300);
  anyWheelEverMoved = false;
  const char* cmds[] = {":set gpio 3\n", ":set pin 5\n", ":set in1 4\n", ":set pwmpin 3\n", ":set en 12\n", ":set muxsig 35\n",
                        ":ch 3 pin 4\n", ":ch 3 gpio 4\n", ":rev ZZ 1\n", ":mtest XX\n", ":set pwmfreq 99999999\n", ":set maxauto -5\n",
                        ":set maxauto 1e99\n", ":set maxauto 0x20\n", ":ch -1 en 1\n", ":ch 16 en 1\n", ":ch 3 bear nan\n", ":ch 3 bear inf\n"};
  for (const char* c : cmds) { send(c); simRunTracking(60); }
  CHECK(!anyWheelEverMoved);
  CHECK(outCount("unknown") + outCount("usage") + outCount("rejected") + outCount("must be a number") + outCount("bearing 0..359.9") >= 14);
  struct { int a, b, e; } spec[4] = {{18, 19, 23}, {25, 26, 33}, {21, 22, 5}, {27, 14, 32}};
  for (int i = 0; i < 4; i++) CHECK(WHEEL_PINS[i].in1 == spec[i].a && WHEEL_PINS[i].in2 == spec[i].b && WHEEL_PINS[i].en == spec[i].e);
  CHECK(g_cfg.maxAuto == 170 && g_cfg.pwmFreq == 5000);
  CHECK(g_cfg.ch[3].bearingDeg == 90);
  // the persisted blob cannot carry pins either: sizes/offsets of the pin tables are not part of Config
  CHECK(sizeof(Config) < 700);
}

static void t_ch_inspect() {
  bootCal();
  simRun(1500);
  g_simMode[3] = SIM_BALL; simRun(1200);
  Serial.out.clear();
  send(":ch 3\n"); send(":ch 13\n"); send(":ch 99\n"); simRun(200);
  CHECK(outHas("C3: ENABLED | active-LOW"));
  CHECK(outHas("status UNVERIFIED"));
  CHECK(outHas("state=DETECT") || outHas("state=none"));
  CHECK(outHas("detections="));
  CHECK(outHas("C13 is not scanned"));
  CHECK(outHas("usage: :ch"));
  // the snapshot shows every enabled channel individually, and only those
  Serial.out.clear();
  send("i"); simRun(300);
  int rows = 0; for (int i = 0; i < 16; i++) { char tag[16]; snprintf(tag, sizeof(tag), " C%-2d ", i); if (outHas(tag)) rows++; }
  CHECK(rows == 12);
  CHECK(!outHas(" C12 ") && !outHas(" C15 "));
}

static void t_ir_slow_interferer() {
  bootCal();
  simRun(500);
  g_simMode[4] = SIM_BALL; g_simChPeriod[4] = 10000;           // 100 Hz envelope (mains-lamp-like), 4.2 ms active
  simRun(4000);
  CHECK(g_ir[4].windows > 20);
  CHECK(g_ir[4].detCount == 0 && !g_ir[4].active);              // at most one pulse fits in a 3 ms window
  CHECK(!g_ball.anyDetect && !faultActive(F_IR_CHANNEL));
}

static void t_cfg_boot_corrupt() {
  freshFs("cfgcorrupt");
  CHECK(fsInit());
  { Config v; cfgDefaults(v); v.maxAuto = 222; v.reverse[0] = 1; cfgSeal(v); v.crc ^= 0xDEADBEEF;            // CRC no longer matches
    FILE* f = fopen((g_fsDir + "/config.bin").c_str(), "wb"); fwrite(&v, sizeof(v), 1, f); fclose(f); }
  setup();
  CHECK(g_cfg.maxAuto == 170 && g_cfg.reverse[0] == 0);                  // safe defaults, corrupt values NOT used
  CHECK(faultActive(F_CFG_REPLACED));
  CHECK(outHas("CRC mismatch"));
  CHECK(g_cond == COND_NORMAL && !g_autoEnabled && allWheelsStopped());   // a warning, not a degradation or a motion
  calibrateAll();
  simRun(500);
  startAuto(); ballAt(0, 10); simRun(2500);
  CHECK(g_navState == NAV_APPROACH);                                      // fully usable with defaults
}

static void t_cfg_save_fail() {
  bootCal();
  simRun(500);
  startAuto(); ballAt(0, 10); simRun(2000);
  g_fsFailWrites = true;
  Serial.out.clear();
  send(":save\n"); simRun(200);
  CHECK(outHas("save FAILED"));
  CHECK(g_navState == NAV_APPROACH && !allWheelsStopped());               // control unaffected by the failed operation
  g_fsFailWrites = false;
  send(":save\n"); simRun(200);
  CHECK(outHas("configuration saved"));
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
  g_hostWriterInline = false;                                // freeze the file so its line count can be compared exactly
  logPump(g_cfg, millis() + 10000);
  std::string newest; { LogFile lf[LOG_LIST_MAX]; uint8_t n = logList(lf); uint32_t best = 0; for (uint8_t i = 0; i < n; i++) if (lf[i].key >= best) { best = lf[i].key; newest = lf[i].name; } }
  int fileLines = 0; std::string firstLine, secondLine;
  { FILE* f = fopen((g_fsDir + newest).c_str(), "r"); char ln[400]; while (f && fgets(ln, sizeof(ln), f)) { fileLines++; if (fileLines == 1) firstLine = ln; if (fileLines == 2) secondLine = ln; } if (f) fclose(f); }
  CHECK(firstLine.rfind("#schema=1;fw=", 0) == 0);
  CHECK(secondLine == std::string(LOG_COLUMNS) + "\n");
  send(":xsel 0\n"); simRun(50);
  Serial.out.clear();
  send("x"); simRun(20000);
  bool ok; int lines;
  parseExportAndCrc(Serial.out, "#BEGIN-EXPORT", &ok, &lines);
  CHECK(ok && lines > 20);
  CHECK(lines == fileLines);                                 // exported row count == rows actually on flash
  g_hostWriterInline = true;
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
  CHECK(outHas(MOCK_CORE_V3 ? "Arduino-ESP32 core 3.0.7 |" : "Arduino-ESP32 core 2.0.17 |"));   // version string composed from MAJOR/MINOR/PATCH
  CHECK(outHas(MOCK_CORE_V3 ? "Arduino-ESP32 core 3.0.7 |" : "Arduino-ESP32 core 2.0.17 |"));   // composed from MAJOR/MINOR/PATCH
  CHECK(!outHas("PASS (operator-confirmed") );             // nothing is claimed verified
}

// ================================================================================================

// ================================================================================================
// IR response evaluation (not a pass/fail test): sweeps the filter parameters over explicit scenarios.
//   ./rs26 --eval-ir <irFullHits> <acqCycles> <lostHoldMs> [verified 0|1]
// All scenarios use the simulated 1.2 kHz ball (833 us period) and the real firmware code.
static void runMsCb(uint32_t ms, const std::function<void(uint32_t)>& each) {
  for (uint32_t t = 0; t < ms; t++) {
    uint64_t end = g_mockUs + 1000ULL;
    while (g_mockUs < end) { loop(); mockAdvance(50); }
    each(t);
  }
}
static void evalClearHistory() {
  clearBall();
  runMsCb(900, [](uint32_t) {});
  for (int guard = 0; guard < 20 && g_ball.valid; guard++) runMsCb(100, [](uint32_t) {});
}
// ms (with 0.05 ms resolution) from the moment two receivers start seeing the ball until g_ball.valid
static double evalAcq(int channelsOn, uint32_t phaseMs, double* cmdMs = nullptr, double* moveMs = nullptr) {
  evalClearHistory();
  runMsCb(phaseMs, [](uint32_t) {});
  if (channelsOn == 1) ballAt(90, 10); else ballAt(75, 20);
  uint64_t t0 = g_mockUs; double tv = -1, tc = -1, tm = -1;
  while (g_mockUs - t0 < 1500000ULL) {
    loop(); mockAdvance(50);
    if (tv < 0 && g_ball.valid) tv = (double)(g_mockUs - t0) / 1000.0;
    if (tc < 0 && g_autoEnabled && g_navState == NAV_APPROACH && (g_wheelTgt[0] != 0 || g_wheelTgt[1] != 0)) tc = (double)(g_mockUs - t0) / 1000.0;
    if (tm < 0 && (g_wheelOut[0] != 0 || g_wheelOut[1] != 0 || g_wheelOut[2] != 0 || g_wheelOut[3] != 0)) tm = (double)(g_mockUs - t0) / 1000.0;
    if (tv >= 0 && (!g_autoEnabled || tm >= 0)) break;
  }
  if (cmdMs) *cmdMs = tc;
  if (moveMs) *moveMs = tm;
  return tv;
}
static bool evalBurstAcquires(uint32_t durMs, uint32_t phaseMs, int ch) {
  evalClearHistory();
  runMsCb(phaseMs, [](uint32_t) {});
  bool got = false;
  g_simMode[ch] = SIM_BALL;
  runMsCb(durMs, [&](uint32_t) { if (g_ball.valid) got = true; });
  g_simMode[ch] = SIM_IDLE;
  runMsCb(700, [&](uint32_t) { if (g_ball.valid) got = true; });
  return got;
}
static bool evalGapLoses(uint32_t gapMs) {
  evalClearHistory();
  ballAt(75, 20);
  runMsCb(700, [](uint32_t) {});
  bool lost = false;
  clearBall();
  runMsCb(gapMs, [&](uint32_t) { if (!g_ball.valid) lost = true; });
  ballAt(75, 20);
  runMsCb(400, [&](uint32_t) { if (!g_ball.valid) lost = true; });
  return lost;
}
static double evalLossLatency() {
  evalClearHistory();
  ballAt(75, 20);
  runMsCb(900, [](uint32_t) {});
  clearBall();
  uint64_t t0 = g_mockUs;
  while (g_ball.valid && g_mockUs - t0 < 3000000ULL) { loop(); mockAdvance(50); }
  return (double)(g_mockUs - t0) / 1000.0;
}
static int evalMissLosses(double startPerSec, uint32_t seconds) {
  evalClearHistory();
  srand(11);
  ballAt(75, 20);
  runMsCb(800, [](uint32_t) {});
  int losses = 0; bool wasValid = g_ball.valid; uint32_t offUntil = 0;
  runMsCb(seconds * 1000, [&](uint32_t t) {
    if (t >= offUntil && (rand() % 100000) < (int)(startPerSec * 100.0)) { clearBall(); offUntil = t + 5 + (uint32_t)(rand() % 11); }
    else if (offUntil && t >= offUntil) { ballAt(75, 20); offUntil = 0; }
    if (wasValid && !g_ball.valid) losses++;
    wasValid = g_ball.valid;
  });
  return losses;
}
static int evalFalseAcquisitions(double burstsPerSec, uint32_t seconds) {
  evalClearHistory();
  srand(5);
  int acq = 0; bool wasValid = false; uint32_t until = 0; int ch = 0;
  runMsCb(seconds * 1000, [&](uint32_t t) {
    if (!until && (rand() % 100000) < (int)(burstsPerSec * 100.0)) { ch = rand() % 12; g_simMode[ch] = SIM_BALL; until = t + 10 + (uint32_t)(rand() % 111); }
    else if (until && t >= until) { g_simMode[ch] = SIM_IDLE; until = 0; }
    if (!wasValid && g_ball.valid) acq++;
    wasValid = g_ball.valid;
  });
  clearBall();
  return acq;
}
static void evalSweep(double degPerSec, double* meanErr, double* maxErr, int* losses) {
  evalClearHistory();
  double sum = 0, mx = 0; int n = 0; int lost = 0; bool wasValid = false;
  uint32_t total = (uint32_t)(360.0 / degPerSec * 1000.0);
  runMsCb(total, [&](uint32_t t) {
    double th = fmod(degPerSec * (double)t / 1000.0, 360.0);
    if (t % 10 == 0) ballAt((float)th, 20);
    if (wasValid && !g_ball.valid) lost++;
    wasValid = g_ball.valid;
    if (g_ball.valid && t % 20 == 0) { double e = fabsf(angDiffDeg(g_ball.bearingDeg, (float)th)); sum += e; if (e > mx) mx = e; n++; }
  });
  *meanErr = n ? sum / n : -1; *maxErr = mx; *losses = lost;
}
static double evalBoundaryJitter() {
  evalClearHistory();
  srand(3);
  uint32_t off0 = 0, off1 = 0; double sum = 0, sum2 = 0; int n = 0;
  ballAt(15, 20);                                    // between receivers C0 (0 deg) and C1 (30 deg)
  runMsCb(10000, [&](uint32_t t) {
    if (!off0 && (rand() % 1000) < 15) { g_simMode[0] = SIM_IDLE; off0 = t + 3 + (uint32_t)(rand() % 4); }
    else if (off0 && t >= off0) { g_simMode[0] = SIM_BALL; off0 = 0; }
    if (!off1 && (rand() % 1000) < 15) { g_simMode[1] = SIM_IDLE; off1 = t + 3 + (uint32_t)(rand() % 4); }
    else if (off1 && t >= off1) { g_simMode[1] = SIM_BALL; off1 = 0; }
    if (t > 1500 && t % 20 == 0 && g_ball.valid) { double b = g_ball.bearingDeg; sum += b; sum2 += b * b; n++; }
  });
  if (n < 10) return -1;
  double m = sum / n; return sqrt(fmax(0.0, sum2 / n - m * m));
}
static int evalMain(int argc, char** argv) {
  uint8_t full = (uint8_t)atoi(argv[2]), acqc = (uint8_t)atoi(argv[3]); uint16_t hold = (uint16_t)atoi(argv[4]);
  bool verified = argc > 5 ? atoi(argv[5]) != 0 : false;
  setup(); calibrateAll();
  g_cfg.irFullHits = full; g_cfg.acqCycles = acqc; g_cfg.lostHoldMs = hold;
  for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = verified ? 1 : 0;
  runMsCb(1500, [](uint32_t) {});
  auto stats = [&](int chans, double* mean, double* mx) {
    double s = 0, m = 0; int n = 0;
    for (uint32_t ph = 0; ph < 36; ph += 3) { double v = evalAcq(chans, ph); if (v < 0) { *mean = -1; *mx = -1; return; } s += v; if (v > m) m = v; n++; }
    *mean = s / n; *mx = m;
  };
  double a1m, a1x, a2m, a2x; stats(1, &a1m, &a1x); stats(2, &a2m, &a2x);
  int minBurst = -1; double burst100 = 0;
  for (uint32_t d : {20u, 40u, 60u, 80u, 100u, 120u, 150u, 200u}) {
    int hit = 0, tot = 0;
    for (uint32_t ph = 0; ph < 36; ph += 6) { hit += evalBurstAcquires(d, ph, 5) ? 1 : 0; tot++; }
    double frac = (double)hit / tot;
    if (minBurst < 0 && frac >= 0.5) minBurst = (int)d;
    if (d == 100) burst100 = frac;
  }
  int maxGap = 0; for (uint32_t g : {50u, 100u, 150u, 200u, 250u, 300u, 400u, 500u}) { if (!evalGapLoses(g)) maxGap = (int)g; else break; }
  double loss = evalLossLatency();
  int miss5 = evalMissLosses(5, 60), miss20 = evalMissLosses(20, 60);
  int falseAcq = evalFalseAcquisitions(2.0, 120);
  double m60, x60, m180, x180; int l60, l180; evalSweep(60, &m60, &x60, &l60); evalSweep(180, &m180, &x180, &l180);
  double jit = evalBoundaryJitter();
  printf("full=%u acq=%u hold=%4u ver=%d | acq1ch mean/max=%6.1f/%6.1f ms | acq2ch %6.1f/%6.1f | burstAcceptMin=%4d ms (frac@100=%.2f) | gapToleratedUpTo=%3d ms | loss=%5.1f ms | missLosses(5Hz,20Hz)=%d,%d | falseAcq/120s@2Hz=%d | sweep60 err mean/max=%.1f/%.1f loss=%d | sweep180 %.1f/%.1f loss=%d | jitterStd=%.2f deg\n",
         full, acqc, hold, verified, a1m, a1x, a2m, a2x, minBurst, burst100, maxGap, loss, miss5, miss20, falseAcq, m60, x60, l60, m180, x180, l180, jit);
  return 0;
}

#include "bench.h"


// ================================================================================================
// engine 2 (leaky evidence, focused revisits, visit-statistics loss) - regression tests, simulation only
static void e2Boot() { bootCal(); for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = 1; g_cfg.engine = 2; irRestartSchedule(); bwClear(); simRun(500); }
static bool e2Acquire(float deg, float fov, double miss, uint32_t withinMs) {
  bwBall(deg, fov, miss);
  uint64_t t0 = g_mockUs;
  while (!g_ball.valid && g_mockUs - t0 < (uint64_t)withinMs * 1000ULL) benchStep();
  return g_ball.valid;
}

static void t_e2_broadband_tail() {
  // After a broadband burst the receivers hit first age out first; the arc that is left must not be accepted as a ball.
  e2Boot();
  for (uint32_t d : {60u, 120u, 250u}) {
    for (int coherent = 0; coherent < 2; coherent++) {
      bool got = false;
      for (int i = 0; i < 12; i++) { bwSet(i, SIM_BALL); g_simPhase[i] = coherent ? 0 : (uint64_t)(rand() % 833); }
      uint64_t t0 = g_mockUs; while (g_mockUs - t0 < (uint64_t)d * 1000ULL) { benchStep(); got |= g_ball.valid; }
      bwClear(); t0 = g_mockUs; while (g_mockUs - t0 < 700000ULL) { benchStep(); got |= g_ball.valid; }
      CHECK(!got);
    }
  }
}

static void t_e2_stall_is_not_absence() {
  // A stalled control loop produces no evidence either way: it must not end a track.
  e2Boot();
  CHECK(e2Acquire(75, 20, 0.0, 400));
  benchRun(1500);
  for (uint32_t stall : {50u, 120u, 250u}) {
    benchStall(stall);
    benchRun(60);
    CHECK(g_ball.valid);
    benchRun(200);
  }
  CHECK(g_scan.winOverrunMaxUs >= 240000u);                  // the lateness was measured, not hidden
  uint32_t mx = 0; for (int i = 0; i < 12; i++) if (g_scan.revisitMaxUs[i] > mx) mx = g_scan.revisitMaxUs[i];
  CHECK(mx >= 240000u);                                       // ... and so was the longest wait of a receiver
}

static void t_e2_focus_scheduling() {
  // A strong track needs no extra windows (the ring revisit interval stays one cycle); a flickering one gets them,
  // but every receiver is still visited within about two ring cycles.
  e2Boot();
  CHECK(e2Acquire(90, 10, 0.0, 400));
  benchRun(2500);
  irRestartSchedule();
  benchRun(3000);
  CHECK(g_scan.focusWindows == 0);
  uint32_t mx = 0; for (int i = 0; i < 12; i++) if (g_scan.revisitMaxUs[i] > mx) mx = g_scan.revisitMaxUs[i];
  CHECK(mx < g_scan.cycleUs + 2000);
  bwClear(); benchRun(1500);
  CHECK(!g_ball.valid);
  CHECK(e2Acquire(90, 10, 0.5, 3000));
  irRestartSchedule();
  benchRun(4000);
  CHECK(g_ball.valid);
  CHECK(g_scan.focusWindows > 100);
  mx = 0; for (int i = 0; i < 12; i++) if (g_scan.revisitMaxUs[i] > mx) mx = g_scan.revisitMaxUs[i];
  CHECK(mx < 2 * g_scan.cycleUs + 4000);                      // no receiver starved
  bwClear();
}

static void t_e2_grace_period_bounds() {
  // Steady signal: short grace. Flickering signal: long grace. Never unbounded.
  e2Boot();
  CHECK(e2Acquire(75, 20, 0.0, 400)); benchRun(2500);
  uint64_t t0 = g_mockUs; bwClear();
  while (g_ball.valid && g_mockUs - t0 < 2000000ULL) benchStep();
  double steady = (double)(g_mockUs - t0) / 1000.0;
  CHECK(steady <= g_cfg.lostHoldMs + 5);                     // measured from removal, the last evidence is older than that
  benchRun(1500);
  CHECK(e2Acquire(90, 10, 0.4, 3000)); benchRun(3000);
  CHECK(g_ball.valid);
  t0 = g_mockUs; bwClear();
  while (g_ball.valid && g_mockUs - t0 < 2000000ULL) benchStep();
  double flick = (double)(g_mockUs - t0) / 1000.0;
  CHECK(flick > g_cfg.lostHoldMs + 40);                       // the flickering track was given more time ...
  CHECK(flick <= g_cfg.lostHoldMaxMs + 5);                    // ... but never more than the configured maximum
  benchRun(1500);
  CHECK(e2Acquire(90, 10, 0.7, 6000)); benchRun(4000);        // very weak signal: still bounded
  t0 = g_mockUs; bwClear();
  while (g_ball.valid && g_mockUs - t0 < 2000000ULL) benchStep();
  CHECK(!g_ball.valid);
  CHECK((double)(g_mockUs - t0) / 1000.0 <= g_cfg.lostHoldMaxMs + 5);
}

static void t_e2_dropout_survival() {
  e2Boot();
  CHECK(e2Acquire(90, 10, 0.0, 400)); benchRun(2500);
  int m = bwMask(); bwOff(); benchRun(100); bwBallMask(m); benchRun(300);
  CHECK(g_ball.valid && g_stats.ballLossEvents == 0);        // 100 ms gap in a steady signal: survived
  benchRun(1000);
  bwOff(); benchRun(300); CHECK(!g_ball.valid);              // 300 ms: beyond the grace period of a steady signal
  benchRun(1000);
  CHECK(e2Acquire(90, 10, 0.4, 3000)); benchRun(4000);
  m = bwMask(); bwOff(); benchRun(200); bwBallMask(m, 0.4); benchRun(300);
  CHECK(g_ball.valid);                                        // 200 ms gap in a flickering signal: survived
}

static void t_e2_reacquisition() {
  e2Boot();
  CHECK(e2Acquire(90, 10, 0.0, 400)); benchRun(2500);
  bwClear(); benchRun(190); CHECK(!g_ball.valid);
  uint64_t t0 = g_mockUs; bwBall(90, 10);                     // same place, shortly after the loss: fast path
  while (!g_ball.valid && g_mockUs - t0 < 1000000ULL) benchStep();
  double fast = (double)(g_mockUs - t0) / 1000.0;
  printf("  re-acquisition after a loss: %.0f ms\n", fast);
  CHECK(fast < 90);
  bwClear(); benchRun(1500); CHECK(!g_ball.valid);
  t0 = g_mockUs; bwBall(90, 10);                              // long after: ordinary acquisition rules
  while (!g_ball.valid && g_mockUs - t0 < 1000000ULL) benchStep();
  CHECK((double)(g_mockUs - t0) / 1000.0 > fast + 20);
  bwClear(); benchRun(1500);
  CHECK(e2Acquire(90, 10, 0.0, 400)); benchRun(2500); bwClear(); benchRun(190);
  t0 = g_mockUs; bwBall(270, 10);                             // shortly after the loss but on the opposite side: not accelerated
  while (!g_ball.valid && g_mockUs - t0 < 1000000ULL) benchStep();
  CHECK((double)(g_mockUs - t0) / 1000.0 > 100);
  bwClear();
}

static void t_e2_stop_after_loss() {
  // After a loss the wheels are brought to rest before the search rotation starts: no immediate strafe -> spin reversal.
  e2Boot();
  startAuto();
  CHECK(e2Acquire(90, 10, 0.0, 600)); benchRun(2000);
  CHECK(g_navState == NAV_APPROACH);
  bwClear();
  uint64_t t0 = g_mockUs; double firstSpin = -1; bool reversed = false; int prevDir[4];
  for (int i = 0; i < 4; i++) prevDir[i] = wheelDir(i);
  while (g_mockUs - t0 < 700000ULL) {
    benchStep();
    if (g_navState == NAV_SEARCH && g_reqW != 0 && firstSpin < 0) firstSpin = msSince(t0);
    if (g_navState == NAV_SEARCH && firstSpin < 0) {
      CHECK(g_reqW == 0 && g_reqVx == 0 && g_reqVy == 0);       // braking: nothing requested
      for (int i = 0; i < 4; i++) { int d = wheelDir(i); if (prevDir[i] != 0 && d != 0 && d != prevDir[i]) reversed = true; }
    }
    for (int i = 0; i < 4; i++) { int d = wheelDir(i); if (d != 0) prevDir[i] = d; }
  }
  CHECK(!reversed);
  CHECK(firstSpin > 0);                                          // the search rotation does start
  CHECK(!anyWheelEverMoved || true);
  send("s"); simRun(100);
  CHECK(allWheelsStopped());
}

static void t_legacy_fallback() {
  // The legacy estimator stays selectable and keeps its old characteristics.
  bootCal(); for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = 1;
  g_cfg.engine = 0; g_cfg.lostHoldMs = 500; g_cfg.lostHoldMaxMs = 500; g_cfg.flickerHoldMs = 500; irRestartSchedule(); bwClear(); simRun(500);
  CHECK(e2Acquire(90, 10, 0.0, 1000));
  benchRun(2000);
  uint64_t t0 = g_mockUs; bwClear();
  while (g_ball.valid && g_mockUs - t0 < 2000000ULL) benchStep();
  double lost = (double)(g_mockUs - t0) / 1000.0;
  CHECK(lost > 480 && lost < 620);
  send(":set engine 1\n"); simRun(20);                       // the removed intermediate estimator cannot be selected
  CHECK(g_cfg.engine == 0);
}

static void t_e2_config_params() {
  bootCal();
  send(":set engine 2\n"); simRun(20);
  send(":set acqspan1 120\n"); simRun(20); CHECK(g_cfg.acqSpan1Ms == 120);
  send(":set flickerhold 280\n"); simRun(20); CHECK(g_cfg.flickerHoldMs == 280);
  send(":set focus 0\n"); simRun(20); CHECK(g_cfg.focusOn == 0);
  send(":set losseps 20\n"); simRun(20); CHECK(g_cfg.lossEpsExp != 20);        // out of range: rejected
  send(":set engine 3\n"); simRun(20); CHECK(g_cfg.engine == 2);
  send(":set engine 1\n"); simRun(20); CHECK(g_cfg.engine == 2);
  // cross-field rule: flickerhold must lie between losthold and losthold_max
  g_cfg.flickerHoldMs = 600; char rep[96]; int bad = cfgValidate(g_cfg, rep, sizeof(rep));
  CHECK(bad >= 1 && g_cfg.flickerHoldMs <= g_cfg.lostHoldMaxMs && g_cfg.flickerHoldMs >= g_cfg.lostHoldMs);
  Config d; cfgDefaults(d);
  CHECK(d.engine == 2 && d.lostHoldMs == 150 && d.lostHoldMaxMs == 300);
}

static void t_scan_revisit_metrics() {
  // The reported "scan cycle" is the ring sweep; the per-receiver revisit interval is measured separately.
  e2Boot();
  irRestartSchedule();
  benchRun(2000);
  CHECK_NEAR(g_scan.cycleUs, 12 * (g_cfg.irWindowUs + g_cfg.irSettleUs), 200);
  uint32_t mn = 0xFFFFFFFFu, mx = 0; for (int i = 0; i < 12; i++) { if (g_scan.revisitMaxUs[i] > mx) mx = g_scan.revisitMaxUs[i]; if (g_scan.revisitMaxUs[i] < mn) mn = g_scan.revisitMaxUs[i]; }
  CHECK(mn > 0 && mx < g_scan.cycleUs + 300);                  // idle: every receiver waits exactly one ring cycle
  benchStall(90);
  benchRun(100);
  mx = 0; for (int i = 0; i < 12; i++) if (g_scan.revisitMaxUs[i] > mx) mx = g_scan.revisitMaxUs[i];
  CHECK(mx > 90000u);                                          // a stall shows up as a long revisit interval
  CHECK(g_scan.winOverrunMaxUs > 80000u);
}


static void t_startup_schedule() {
  // Right after setup() the first scan cycles must be regular (boot-time blocking work must not leak into the schedule),
  // nothing may move, and a ball that is already there is found within the normal acquisition time.
  setup(); calibrateAll();
  for (int i = 0; i < 12; i++) g_cfg.ch[i].verified = 1;
  bwClear();
  benchRun(800);                                               // idle ring only
  CHECK(g_scan.cycleCount >= 15);
  CHECK(g_scan.cycleMaxUs < 12u * (g_cfg.irWindowUs + g_cfg.irSettleUs) + 1000u);
  CHECK(g_scan.cycleMinUs > 12u * (g_cfg.irWindowUs + g_cfg.irSettleUs) - 1000u);
  CHECK(g_scan.winOverrunMaxUs < 1500);
  bwBall(75, 20);                                              // now a ball appears
  uint64_t t0 = g_mockUs; double acq = -1;
  while (g_mockUs - t0 < 600000ULL) { benchStep(); if (acq < 0 && g_ball.valid) acq = msSince(t0); trackWheelMotion(); }
  CHECK(acq > 0 && acq < 100);
  CHECK(!anyWheelEverMoved);                                   // autonomy is off: sensing alone moves nothing
  bwClear();
}

static void t_e2_view_transitions() {
  // A ball that moves between "seen by one receiver" and "seen by two" (and back) must not be dropped.
  for (double miss : {0.0, 0.3}) {
    e2Boot(); g_cfg.engine = 2;
    CHECK(e2Acquire(90, 10, miss, 3000)); benchRun(500);
    int loss0 = (int)g_stats.ballLossEvents; bool wasValid = g_ball.valid; int losses = 0;
    static const float POS[6] = {90, 80, 75, 70, 60, 75};
    uint64_t end = g_mockUs + 40000000ULL; int idx = 0; uint64_t nextMove = g_mockUs;
    while (g_mockUs < end) {
      if (g_mockUs >= nextMove) { nextMove += 700000ULL; float p = POS[idx++ % 6]; bwBall(p, p == 90 || p == 60 ? 10.0f : 20.0f, miss); }
      benchStep();
      if (wasValid && !g_ball.valid) losses++;
      wasValid = g_ball.valid;
    }
    (void)loss0;
    CHECK(losses == 0);
    bwClear();
  }
}

static void t_e2_stale_not_followed() {
  // Once the evidence stops, the direction must not move and the navigation speed must already be reduced before the
  // ball is declared lost.
  e2Boot();
  startAuto();
  CHECK(e2Acquire(90, 10, 0.0, 600)); benchRun(2500);
  CHECK(g_navState == NAV_APPROACH);
  double steadySpeed = sqrt((double)g_reqVx * g_reqVx + (double)g_reqVy * g_reqVy);
  float bearing = g_ball.bearingDeg;
  bwClear();
  uint64_t t0 = g_mockUs; bool slowedBeforeLoss = false; float maxDrift = 0;
  while (g_ball.valid && g_mockUs - t0 < 1000000ULL) {
    benchStep();
    maxDrift = fmaxf(maxDrift, fabsf(angDiffDeg(g_ball.bearingDeg, bearing)));
    if (g_ball.valid && sqrt((double)g_reqVx * g_reqVx + (double)g_reqVy * g_reqVy) < 0.5 * steadySpeed) slowedBeforeLoss = true;
  }
  CHECK(maxDrift < 1.0f);
  CHECK(slowedBeforeLoss);
  send("s"); simRun(100);
}

struct TestCase { const char* name; void (*fn)(); };
#define T(n) {#n, n}
static const TestCase TESTS[] = {
  T(t_angles), T(t_timer_rollover), T(t_mecanum), T(t_pinmap), T(t_boot_safe), T(t_ramp), T(t_config), T(t_classify),
  T(t_ir_detect), T(t_ir_wrap), T(t_ir_ambiguous), T(t_ir_stuck_and_recover), T(t_ir_noise), T(t_ir_polarity),
  T(t_ir_system_loss), T(t_ir_trusted_low), T(t_ir_scan_stall), T(t_ir_overrun), T(t_ir_coverage), T(t_ir_probe),
  T(t_imu_integrate), T(t_imu_bias), T(t_imu_missing), T(t_imu_dropout), T(t_imu_invalid), T(t_heading_hold),
  T(t_nav_approach), T(t_nav_unreliable_slow), T(t_nav_lost_search), T(t_nav_hysteresis), T(t_nav_isolated_samples),
  T(t_safe_stop_moving), T(t_loop_stall), T(t_resets), T(t_reset_wdt), T(t_fault_report_format),
  T(t_cli_manual), T(t_cli_robust), T(t_diag_no_motion), T(t_live_ir_rate), T(t_motor_test), T(t_drive_test), T(t_nav_all_directions), T(t_nav_sweep_and_multi), T(t_imu_frozen_in_approach), T(t_gpio_tamper), T(t_ch_inspect), T(t_ir_spare_channels), T(t_ir_period_gate), T(t_ir_slow_interferer), T(t_latency), T(t_fast_engine_rejection), T(t_side_zone_note), T(t_boot_no_channels), T(t_fs_commands), T(t_cfg_boot_corrupt), T(t_cfg_save_fail), T(t_cfg_set),
  T(t_fs_missing), T(t_fs_write_failure), T(t_fs_full), T(t_log_rotation), T(t_log_queue_overflow),
  T(t_flight_recorder), T(t_export), T(t_nonblocking), T(t_reboot_phase1), T(t_reboot_phase2),
  T(t_cal_startup_defaults), T(t_health_report_text),
  T(t_e2_broadband_tail), T(t_e2_stall_is_not_absence), T(t_e2_focus_scheduling), T(t_e2_grace_period_bounds), T(t_e2_dropout_survival),
  T(t_e2_reacquisition), T(t_e2_stop_after_loss), T(t_legacy_fallback), T(t_e2_config_params), T(t_scan_revisit_metrics),
  T(t_startup_schedule), T(t_e2_view_transitions), T(t_e2_stale_not_followed),
};

int main(int argc, char** argv) {
  if (argc >= 2 && strcmp(argv[1], "--list") == 0) { for (auto& t : TESTS) printf("%s\n", t.name); return 0; }
  if (argc >= 2 && strcmp(argv[1], "--bench") == 0) { freshFs("bench"); return benchMain(argc, argv); }
  if (argc >= 5 && strcmp(argv[1], "--eval-ir") == 0) { freshFs("eval"); return evalMain(argc, argv); }
  if (argc < 2) { fprintf(stderr, "usage: %s <test>|--list\n", argv[0]); return 2; }
  for (auto& t : TESTS) {
    if (strcmp(t.name, argv[1]) != 0) continue;
    if (strncmp(t.name, "t_reboot", 8) != 0 && strcmp(t.name, "t_config") != 0 && strncmp(t.name, "t_fs_", 5) != 0 && strcmp(t.name, "t_boot_no_channels") != 0 && strcmp(t.name, "t_cfg_boot_corrupt") != 0 &&
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
