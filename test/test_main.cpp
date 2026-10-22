// Host-side verification of the firmware logic. Each test case runs in its own process
// (./run_tests.sh), so all firmware globals start fresh. "Reboot" tests re-launch the binary
// and share the on-disk mock LittleFS directory.
//
// Everything here is SOFTWARE verification against simulated sensor values, a simulated IMU and a simulated filesystem.
// It says nothing about the physical robot: the real IR sensors' response to the real ball is NOT TESTED.
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
static int maxWheelDuty() { int m = 0; for (int i = 0; i < 4; i++) m = std::max(m, (int)g_pwmDuty[WHEEL_PINS[i].en]); return m; }
static bool anyWheelEverMoved = false;

// Sensor model of the tests: an ANALOG sensor that reads CLR with nothing in front and NEAR with the ball in the shooting position
// (rising polarity). The simulated "strength" is the fraction of that span. These numbers are test inputs, not measurements.
static const uint16_t CLR = 500, NEAR = 3000;
static void sensors(float s0, float s1) {
  g_mockAnalog[PIN_IR[0]] = (uint16_t)(CLR + s0 * (NEAR - CLR) + 0.5f);
  g_mockAnalog[PIN_IR[1]] = (uint16_t)(CLR + s1 * (NEAR - CLR) + 0.5f);
}
static void ballGone() { sensors(0.0f, 0.0f); }
static void ballFar() { sensors(0.35f, 0.35f); }          // both sensors see the ball, not near
static void ballHeld() { sensors(1.0f, 0.9f); }           // sensor 0 near, sensor 1 sees it, centred: the shooting position
// Compatibility with the older tests: a ball roughly ahead is seen by both sensors, anything else is not seen.
static void ballAt(float deg, float = 20.0f) { if (absf(deg) <= 45.0f) ballFar(); else ballGone(); }
static void calibrateAll() {
  g_cfg.irClear0 = CLR; g_cfg.irNear0 = NEAR; g_cfg.irClear1 = CLR; g_cfg.irNear1 = NEAR;
  g_cfg.irOffsetSide = 1;                                   // sensor 1 mounted to the right of sensor 0
}
static void bootCal() { ballGone(); setup(); calibrateAll(); }
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
  ballGone();
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
  CHECK(PIN_IR[0] == 35 && PIN_IR[1] == 36);
  CHECK(PIN_IR_EMIT[0] == 4 && PIN_IR_EMIT[1] == 13);          // PROPOSED emitter pins (docs/IR_SENSORS.md): distinct from every motor / I2C / receiver pin
  for (int e = 0; e < 2; e++) { for (int i = 0; i < 4; i++) CHECK(PIN_IR_EMIT[e] != WHEEL_PINS[i].in1 && PIN_IR_EMIT[e] != WHEEL_PINS[i].in2 && PIN_IR_EMIT[e] != WHEEL_PINS[i].en); CHECK(PIN_IR_EMIT[e] != PIN_I2C_SDA && PIN_IR_EMIT[e] != PIN_I2C_SCL && PIN_IR_EMIT[e] != PIN_IR[0] && PIN_IR_EMIT[e] != PIN_IR[1]); }
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
  c.irSeenPct = 200;               CHECK(cfgValidate(c, rep, sizeof(rep)) == 1 && c.irSeenPct == d.irSeenPct && strstr(rep, "irseen"));
  c.maxAuto = 0;                   CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.maxAuto == d.maxAuto);
  c.pwmFreq = 60000;               CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.pwmFreq == d.pwmFreq);
  c.reverse[2] = 7;                CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.reverse[2] == 0);
  c.irOffsetSide = 5;              CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.irOffsetSide == d.irOffsetSide);
  c.irClear0 = 1000; c.irNear0 = 1020; CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.irClear0 == 0 && c.irNear0 == 0);   // no usable span: stored as "not calibrated"
  c.minPwm = 200; c.maxManual = 100; CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.minPwm < c.maxManual);
  c.imuYawSign = 3;                CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.imuYawSign == -1);
  c.imuAddrPref = 0x30;            CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.imuAddrPref == 0);
  CHECK(cfgValidate(c, rep, sizeof(rep)) == 0);
  // every default is inside its own declared range
  for (size_t i = 0; i < PARAM_COUNT; i++) CHECK(paramInRange(PARAMS[i], paramGet(d, PARAMS[i])));
  // CRC / persistence
  freshFs("config");
  CHECK(fsInit());
  Config s; cfgDefaults(s); s.maxAuto = 123; s.irClear0 = 800; s.irNear0 = 3200;
  CHECK(cfgSave(s));
  Config l; CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_OK);
  CHECK(l.maxAuto == 123 && l.irClear0 == 800 && l.irNear0 == 3200);
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
  { Config v; cfgDefaults(v); v.maxManual = 111; v.irSeenPct = 200; cfgSeal(v); std::string p = g_fsDir + "/config.bin"; FILE* f = fopen(p.c_str(), "wb"); fwrite(&v, sizeof(v), 1, f); fclose(f); }
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_REPAIRED && l.maxManual == 111 && l.irSeenPct == d.irSeenPct && strstr(rep, "irseen"));
  // never persist invalid settings
  Config bad; cfgDefaults(bad); bad.irSeenPct = 200;
  CHECK(!cfgSave(bad));
  // pins are not part of the configuration: no parameter name can touch them
  for (size_t i = 0; i < PARAM_COUNT; i++) CHECK(strstr(PARAMS[i].name, "gpio") == nullptr && strncmp(PARAMS[i].name, "pin", 3) != 0);
  // and no ParamDef offset can reach outside the Config struct
  for (size_t i = 0; i < PARAM_COUNT; i++) CHECK(PARAMS[i].off + 1 <= offsetof(Config, crc));
}

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
  // ball seeking remains available without heading hold
  simRun(500);
  startAuto();
  CHECK(g_autoEnabled);
  CHECK(outHas("heading hold unavailable"));
  ballAt(0);
  simRun(2000);
  CHECK(g_navState == NAV_APPROACH);
  CHECK(isnan(g_hdgErr));
  CHECK(g_reqW == 0.0f);                                    // no heading, no bearing: the robot never turns toward the ball
  CHECK(fabsf(g_reqW) <= g_cfg.hdgMaxRot + 1e-6);
  CHECK(!allWheelsStopped());
  // slower than the healthy case because the condition is DEGRADED
  CHECK(sqrtf(g_reqVx * g_reqVx + g_reqVy * g_reqVy) <= g_cfg.degradedScale * g_cfg.approachSpeed + 1e-4);
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

static void t_safe_stop_moving() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0, 10);
  simRun(2500);
  CHECK(!allWheelsStopped());
  g_mockAnalog[PIN_IR[0]] = 0;                              // front sensor wiring fault: sensing lost
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
  // 'a' refused with uncalibrated sensors
  g_cfg.irClear0 = g_cfg.irNear0 = 0;
  Serial.out.clear(); send("a"); simRun(100);
  CHECK(!g_autoEnabled && outHas("REFUSED") && outHas("not calibrated"));
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
  CHECK(outHas("IR pairs (2") && outHas("MPU6050"));
  CHECK(outHas("HARDWARE HEALTH REPORT"));
  CHECK(outHas("state=IDLE"));
  CHECK(outHas("persist=ON"));
  CHECK(outHas("TEST MODE"));
  CHECK(outHas("commands (115200 baud)"));
  CHECK(outHas("signal STRENGTH only"));
  CHECK(!outHas("distance:"));                              // no fabricated distance anywhere
  CHECK(g_streamIr);                                        // 'I' toggled the live view on
  send("I"); simRun(100);
  CHECK(!g_streamIr);
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
  g_mockAnalog[PIN_IR[0]] = 0;                              // front sensor wiring fault
  simRun(g_cfg.irRailMs + 600);
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

static void t_gpio_tamper() {
  bootCal();
  simRun(300);
  anyWheelEverMoved = false;
  const char* cmds[] = {":set gpio 3\n", ":set pin 5\n", ":set in1 4\n", ":set pwmpin 3\n", ":set en 12\n", ":set muxsig 35\n",
                        ":rev ZZ 1\n", ":mtest XX\n", ":set pwmfreq 99999999\n", ":set maxauto -5\n",
                        ":set maxauto 1e99\n", ":set maxauto 0x20\n", ":ir cal clear 9\n", ":ir mode 0 nan\n", ":set irclear0 nan\n", ":set irclear0 inf\n"};
  for (const char* c : cmds) { send(c); simRunTracking(60); }
  CHECK(!anyWheelEverMoved);
  CHECK(outCount("unknown") + outCount("usage") + outCount("rejected") + outCount("must be a number") >= 12);
  struct { int a, b, e; } spec[4] = {{18, 19, 23}, {25, 26, 33}, {21, 22, 5}, {27, 14, 32}};
  for (int i = 0; i < 4; i++) CHECK(WHEEL_PINS[i].in1 == spec[i].a && WHEEL_PINS[i].in2 == spec[i].b && WHEEL_PINS[i].en == spec[i].e);
  CHECK(g_cfg.maxAuto == 170 && g_cfg.pwmFreq == 5000);
  CHECK(g_cfg.irClear0 == CLR);
  // the persisted blob cannot carry pins either: sizes/offsets of the pin tables are not part of Config
  CHECK(sizeof(Config) < 700);
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
  send(":set irseen nan\n"); simRun(50);
  CHECK(g_cfg.irSeenPct == 15);
  send(":set nonsense 3\n"); simRun(50);
  send(":rev FR 1\n"); simRun(50);
  CHECK(g_cfg.reverse[1] == 1);
  send(":roller O\n"); simRun(50);
  CHECK(g_cfg.rollerO == 1);
  send(":set irclear0 5000\n"); simRun(50);
  CHECK(g_cfg.irClear0 == CLR);                             // rejected (range 0..4095)
  send(":set irclear0 700\n"); simRun(50);
  CHECK(g_cfg.irClear0 == 700);
  send(":save\n"); simRun(100);
  CHECK(outHas("configuration saved"));
  send(":defaults\n"); simRun(50);
  CHECK(g_cfg.maxAuto == 170 && g_cfg.reverse[1] == 0);
  send(":load\n"); simRun(50);
  CHECK(g_cfg.maxAuto == 150 && g_cfg.reverse[1] == 1 && g_cfg.rollerO == 1 && g_cfg.irClear0 == 700);
}

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
  CHECK(worst < 300);                                       // never a wait on Serial
  CHECK(g_outDropped > 0 || outUsed() > 0);                 // back-pressure shows up as drops/queueing, not blocking
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_irs[0].samples > 20);
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

static void t_reboot_phase1() {
  freshFs("reboot");
  bootCal();
  simRun(300);
  send(":set irclear0 600\n");
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
  ballGone();
  setup();
  CHECK(!g_autoEnabled);                                    // reboot never resumes aggressive movement
  CHECK(g_navState == NAV_IDLE && allWheelsStopped());
  CHECK(g_cfg.maxAuto == 133 && g_cfg.reverse[2] == 1 && (g_cfg.motorVerified & 1));
  CHECK(g_cfg.irClear0 == 600 && g_cfg.irNear0 == NEAR);                // the sensor calibration survived the reboot
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
  ballGone(); setup();
  CHECK(!irCalibrated(g_cfg, 0) && !irCalibrated(g_cfg, 1) && g_cfg.irOffsetSide == 0);
  Serial.out.clear();
  send("a"); simRun(50);
  CHECK(!g_autoEnabled);                                    // an uncalibrated sensor is never trusted for autonomy
  CHECK(outHas("not calibrated"));
}

static void t_health_report_text() {
  bootCal();
  simRun(500);
  Serial.out.clear();
  send("h"); simRun(1500);
  CHECK(outHas("NOT MEASURABLE (no voltage monitor installed)"));
  CHECK(outHas("NOT MEASURABLE (no encoders, no current sensing)"));
  CHECK(outHas("NOT TESTED"));
  CHECK(outHas("[IR] sensor 0 GPIO35") && outHas("[IR] sensor 1 GPIO36"));
  CHECK(outHas("physical operation with the real ball"));
  CHECK(outHas("NOT TESTED (not calibrated)") == false);    // bootCal calibrated them
  CHECK(outHas("MPU6050 @0x68 who=0x68: PASS"));
  CHECK(outHas("PWM shown is COMMANDED"));
  CHECK(outHas("reset reason"));
  CHECK(outHas(MOCK_CORE_V3 ? "Arduino-ESP32 core 3.0.7 |" : "Arduino-ESP32 core 2.0.17 |"));   // version string composed from MAJOR/MINOR/PATCH
  CHECK(outHas(MOCK_CORE_V3 ? "Arduino-ESP32 core 3.0.7 |" : "Arduino-ESP32 core 2.0.17 |"));   // composed from MAJOR/MINOR/PATCH
  CHECK(!outHas("PASS (operator-confirmed") );             // nothing is claimed verified
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


// ================================================================================================
// IR sensors (two, front)
static void t_ir_pure_rules() {
  // strength: 0 at clear, 1 at near, either polarity
  CHECK_NEAR(irStrength(500, 500, 3000), 0.0, 1e-4);
  CHECK_NEAR(irStrength(3000, 500, 3000), 1.0, 1e-4);
  CHECK_NEAR(irStrength(1750, 500, 3000), 0.5, 1e-4);
  CHECK_NEAR(irStrength(3000, 3500, 500), 0.1667, 1e-3);    // falling polarity: near = LOW raw
  CHECK_NEAR(irStrength(500, 3500, 500), 1.0, 1e-4);
  CHECK_NEAR(irStrength(900, 100, 130), 0.0, 1e-4);         // span below 50: not calibrated, strength 0
  // hysteresis: on at the threshold, off only below 60 % of it
  CHECK(!irHyst(false, 0.14f, 15, 0.6f) && irHyst(false, 0.15f, 15, 0.6f));
  CHECK(irHyst(true, 0.10f, 15, 0.6f) && !irHyst(true, 0.08f, 15, 0.6f));
  // sideways position, sensor 1 on the right (+1)
  float o = 9;
  CHECK(!irOffsetFrom(false, false, 0, 0, 1, &o));                         // nothing seen
  CHECK(!irOffsetFrom(true, true, 0.5f, 0.5f, 0, &o));                     // side not set
  CHECK(irOffsetFrom(true, true, 0.5f, 0.5f, 1, &o) && absf(o) < 1e-4);    // equal: centred
  CHECK(irOffsetFrom(true, true, 0.2f, 0.6f, 1, &o) && o > 0.4f);          // sensor 1 (right) stronger: ball to the right
  CHECK(irOffsetFrom(true, true, 0.6f, 0.2f, 1, &o) && o < -0.4f);
  CHECK(irOffsetFrom(false, true, 0, 0.5f, 1, &o) && o == 1.0f);           // only sensor 1: its side
  CHECK(irOffsetFrom(true, false, 0.5f, 0, 1, &o) && o == -1.0f);          // only sensor 0: the other side
  CHECK(irOffsetFrom(false, true, 0, 0.5f, -1, &o) && o == -1.0f);         // sensor 1 mounted left: mirrored
  // configuration: a calibration with no usable span is stored as "not calibrated"
  Config c; cfgDefaults(c);
  c.irClear0 = 1000; c.irNear0 = 1020;
  char rep[64]; CHECK(cfgValidate(c, rep, sizeof(rep)) > 0 && c.irClear0 == 0 && c.irNear0 == 0);
  cfgDefaults(c); c.irSeenPct = 80; c.irNearPct = 50;
  CHECK(cfgValidate(c, rep, sizeof(rep)) > 0 && c.irSeenPct == 15 && c.irNearPct == 70);
}

static void t_ir_states_and_debounce() {
  bootCal();
  simRun(200);
  CHECK(!g_irs[0].seen && !g_ball.valid);
  sensors(0.5f, 0.0f); simRun(100);
  CHECK(g_irs[0].seen && !g_irs[0].near && !g_irs[1].seen);
  CHECK(g_ball.valid && g_ball.seenMask == 1);
  sensors(0.9f, 0.0f); simRun(100);
  CHECK(g_irs[0].near && g_ball.nearMask == 1);
  sensors(0.0f, 0.0f); simRun(100);
  CHECK(!g_irs[0].seen && !g_irs[0].near);
  // a 4 ms glitch (shorter than the 3-sample debounce) is ignored
  sensors(0.9f, 0.0f); simRun(4); sensors(0.0f, 0.0f); simRun(100);
  CHECK(!g_irs[0].seen);
  // the ball stays "valid" for lostHoldMs after it was last seen, then a loss event is raised
  sensors(0.5f, 0.0f); simRun(100); sensors(0.0f, 0.0f); simRun(100);
  CHECK(g_ball.valid);
  simRun(g_cfg.lostHoldMs + 100);
  CHECK(!g_ball.valid);
}

static void t_ir_polarity_and_digital() {
  bootCal();
  // falling polarity: the ball LOWERS the reading (clear 3000, near 500)
  g_cfg.irClear0 = 3000; g_cfg.irNear0 = 500;
  g_mockAnalog[PIN_IR[0]] = 3000; simRun(100);
  CHECK(!g_irs[0].seen);
  g_mockAnalog[PIN_IR[0]] = 1200; simRun(100);                                // (3000-1200)/2500 = 0.72: near
  CHECK(g_irs[0].seen && g_irs[0].near);
  // a digital sensor: nothing assumed about the level. Calibrate it with the ball absent / present.
  g_mockAnalog[PIN_IR[0]] = 0;                                                // idles LOW
  send(":ir mode 0 digital\n"); simRun(50);
  CHECK(g_cfg.irMode0 == 1 && g_cfg.irClear0 == 0 && g_cfg.irNear0 == 0);    // calibration cleared by the mode change
  simRun(100);
  send(":ir cal clear 0\n"); simRun(20);
  g_mockAnalog[PIN_IR[0]] = 4000; simRun(100);                                // ball present: output goes HIGH
  send(":ir cal near 0\n"); simRun(20);
  CHECK(irCalibrated(g_cfg, 0) && g_cfg.irClear0 == 0 && g_cfg.irNear0 == 4095);
  g_mockAnalog[PIN_IR[0]] = 0; simRun(100);
  CHECK(!g_irs[0].seen);
  g_mockAnalog[PIN_IR[0]] = 4000; simRun(100);
  CHECK(g_irs[0].seen && g_irs[0].near);                                      // a digital sensor: seen and near coincide
  // the opposite digital polarity (active LOW) works the same way after its own calibration
  g_cfg.irClear0 = 4095; g_cfg.irNear0 = 0;
  g_mockAnalog[PIN_IR[0]] = 4000; simRun(100); CHECK(!g_irs[0].seen);
  g_mockAnalog[PIN_IR[0]] = 0; simRun(100); CHECK(g_irs[0].seen && g_irs[0].near);
}

static void t_ir_rail_fault_and_uncalibrated() {
  bootCal();
  simRun(300);
  CHECK(!faultActive(F_IR_FRONT) && !faultActive(F_IR_ALIGN));
  // sensor 1 shorted to ground: DEGRADED, sensor 1 excluded, robot keeps working
  g_mockAnalog[PIN_IR[1]] = 0;
  simRun(g_cfg.irRailMs + 600);
  CHECK(faultActive(F_IR_ALIGN) && g_cond == COND_DEGRADED && !irUsable(g_cfg, 1) && irUsable(g_cfg, 0));
  g_mockAnalog[PIN_IR[1]] = (uint16_t)(CLR + 500); simRun(g_cfg.recoverVerifyMs + 1500);
  CHECK(!faultActive(F_IR_ALIGN) && g_cond == COND_NORMAL);
  // the end stop on the "near" side is legitimate (a ball touching the sensor can saturate it): no fault, however long
  g_mockAnalog[PIN_IR[0]] = 4095;
  simRun(g_cfg.irRailMs + 600);
  CHECK(!faultActive(F_IR_FRONT) && g_irs[0].near);          // held at the saturated end: still a ball, not a fault
  // sensor 0 at the bottom stop (the clear side): CRITICAL, motors stop
  g_mockAnalog[PIN_IR[0]] = 0;
  simRun(g_cfg.irRailMs + 600);
  CHECK(faultActive(F_IR_FRONT) && g_cond == COND_SAFE_STOP);
  // an uncalibrated or disabled sensor is "not tested", never a fault
  ballGone(); simRun(g_cfg.recoverVerifyMs + 1500);
  CHECK(!faultActive(F_IR_FRONT));
  g_cfg.irClear1 = g_cfg.irNear1 = 0; g_mockAnalog[PIN_IR[1]] = 0; simRun(g_cfg.irRailMs + 600);
  CHECK(!faultActive(F_IR_ALIGN));
  // a falling sensor (the ball lowers the reading): stuck at 4095 is the fault, 0 is legitimate; and a calibration at the end stop switches the check off
  g_cfg.irEnable = 3; g_cfg.irClear1 = 3000; g_cfg.irNear1 = 800; g_mockAnalog[PIN_IR[1]] = 0; simRun(g_cfg.irRailMs + 600);
  CHECK(!faultActive(F_IR_ALIGN));
  g_mockAnalog[PIN_IR[1]] = 4095; simRun(g_cfg.irRailMs + 600);
  CHECK(faultActive(F_IR_ALIGN));
  g_cfg.irClear1 = 4090; g_cfg.irNear1 = 1000; simRun(g_cfg.irRailMs + 600 + g_cfg.recoverVerifyMs + 1500);
  CHECK(!faultActive(F_IR_ALIGN));
}

static void t_ir_uncalibrated_autonomy_refused() {
  ballGone(); setup();                                      // factory configuration: no calibration at all
  simRun(300);
  CHECK(!irCalibrated(g_cfg, 0) && !irCalibrated(g_cfg, 1));
  Serial.out.clear(); send("a"); simRun(100);
  CHECK(!g_autoEnabled && outHas("REFUSED") && outHas("not calibrated"));
  Serial.out.clear(); send("h"); simRun(300);
  CHECK(outHas("NOT TESTED (not calibrated)") && outHas("[IR] sensor 0 GPIO35") && outHas("[IR] sensor 1 GPIO36"));
  CHECK(!outHas("multiplexer") && !outHas("TSOP") && !outHas("C12"));
  CHECK(g_cond == COND_NORMAL);                             // not calibrated is not a fault
  calibrateAll(); g_cfg.irClear1 = g_cfg.irNear1 = 0;       // only sensor 0 calibrated: allowed, with a warning
  Serial.out.clear(); send("a"); simRun(100);
  CHECK(g_autoEnabled && outHas("sensor 1 is not calibrated"));
}

static void t_ir_cli() {
  bootCal();
  g_cfg.irClear0 = g_cfg.irNear0 = g_cfg.irClear1 = g_cfg.irNear1 = 0;
  simRun(200);
  Serial.out.clear(); send(":ir\n"); simRun(20);
  CHECK(outHas("NOT CALIBRATED"));
  ballGone(); simRun(100);
  send(":ir cal clear 0\n"); simRun(20); CHECK(g_cfg.irClear0 == CLR);
  sensors(1.0f, 0.0f); simRun(100);
  send(":ir cal near 0\n"); simRun(20); CHECK(g_cfg.irNear0 == NEAR && outHas("a HIGHER raw means a closer ball"));
  ballGone(); simRun(100);
  send(":ir cal clear 1\n"); simRun(20);
  sensors(0.0f, 1.0f); simRun(100);
  send(":ir cal near 1\n"); simRun(20);
  CHECK(irCalibrated(g_cfg, 0) && irCalibrated(g_cfg, 1));
  send(":ir side left\n"); simRun(20); CHECK(g_cfg.irOffsetSide == -1);
  send(":ir side right\n"); simRun(20); CHECK(g_cfg.irOffsetSide == 1);
  send(":ir side none\n"); simRun(20); CHECK(g_cfg.irOffsetSide == 0);
  send(":ir off 1\n"); simRun(20); CHECK(!irEnabled(g_cfg, 1));
  send(":ir on 1\n"); simRun(20); CHECK(irEnabled(g_cfg, 1));
  // identical readings (no usable difference) are reported, not accepted
  g_cfg.irClear1 = g_cfg.irNear1 = 0; ballGone(); simRun(100);
  send(":ir cal clear 1\n"); simRun(20); send(":ir cal near 1\n"); simRun(20);
  CHECK(outHas("differ by less than 50"));
  send(":ir cal near 7\n"); simRun(20); send(":ir mode 0 banana\n"); simRun(20); send(":ir bogus\n"); simRun(20);
  CHECK(outCount("usage: :ir") >= 3);
}

static void t_ball_offset_and_shooting_position() {
  bootCal();
  simRun(200);
  // sensor 1 is to the right (+1)
  sensors(0.3f, 0.6f); simRun(150);
  CHECK(g_ball.offsetKnown && g_ball.offset > 0.2f && !g_ball.held);
  sensors(0.6f, 0.3f); simRun(300);
  CHECK(g_ball.offset < -0.2f);
  sensors(0.5f, 0.5f); simRun(300);
  CHECK(absf(g_ball.offset) < 0.1f && g_ball.centered);
  sensors(0.0f, 0.5f); simRun(100);                         // only sensor 1: the ball is on its side (right)
  CHECK(g_ball.seenMask == 2 && g_ball.offset > 0.5f);
  sensors(0.5f, 0.0f); simRun(300);
  CHECK(g_ball.seenMask == 1 && g_ball.offset < -0.5f);
  // shooting position: sensor 0 near, and (irNeedBoth) sensor 1 sees the ball, and centred
  sensors(1.0f, 0.0f); simRun(300);
  CHECK(g_irs[0].near && !g_ball.held && !g_ball.ready);    // sensor 1 does not see it: not the shooting position
  sensors(1.0f, 0.9f); simRun(300);
  CHECK(g_ball.held && g_ball.centered && g_ball.ready);
  sensors(1.0f, 0.3f); simRun(300);
  CHECK(g_ball.held && !g_ball.centered && !g_ball.ready);  // held, but off to the left
  g_cfg.irNeedBoth = 0; sensors(1.0f, 0.0f); simRun(300);
  CHECK(g_ball.held);
  // without a mounting side nothing is claimed about left/right
  g_cfg.irOffsetSide = 0; ballGone(); simRun(500); sensors(0.5f, 0.5f); simRun(100);
  CHECK(!g_ball.offsetKnown && g_ball.centered);
  // sensor 1 unusable: sensor 0 alone decides
  g_cfg.irOffsetSide = 1; g_cfg.irNeedBoth = 1; g_cfg.irEnable = 1; ballGone(); simRun(100);
  sensors(1.0f, 0.0f); simRun(300);
  CHECK(g_ball.held && !g_ball.offsetKnown);
}


// ================================================================================================
// emitter pulsing (reflected light, ambient subtraction)
static void t_ir_emit_pins_untouched_by_default() {
  bootCal();
  simRun(2000);
  CHECK(g_cfg.irEmit == 0 && !g_emitDriven);
  CHECK(g_pinMode[PIN_IR_EMIT[0]] != OUTPUT && g_pinMode[PIN_IR_EMIT[1]] != OUTPUT);    // the proposed emitter pins are never configured unless enabled
  CHECK(g_mockEmitWrites[PIN_IR_EMIT[0]] == 0 && g_mockEmitWrites[PIN_IR_EMIT[1]] == 0);
  send(":ir emit on\n"); simRun(100);
  CHECK(g_cfg.irEmit == 1 && g_emitDriven && g_pinMode[PIN_IR_EMIT[0]] == OUTPUT && g_pinMode[PIN_IR_EMIT[1]] == OUTPUT);
  CHECK(outHas("through YOUR transistor driver") && !irCalibrated(g_cfg, 0));            // calibrations are cleared: the signal changed meaning
  CHECK(g_pinLevel[PIN_IR_EMIT[0]] == 0 && g_pinLevel[PIN_IR_EMIT[1]] == 0);            // LOW (off) between pulses
  CHECK(g_mockEmitWrites[PIN_IR_EMIT[0]] > 10);
  send(":ir emit off\n"); simRun(100);
  CHECK(!g_emitDriven && g_pinMode[PIN_IR_EMIT[0]] == INPUT && g_pinLevel[PIN_IR_EMIT[0]] == 0 && g_pinLevel[PIN_IR_EMIT[1]] == 0);
}

static void t_ir_emit_ambient_rejection() {
  g_mockEmitModel = true;
  g_mockAmbient[PIN_IR[0]] = g_mockAmbient[PIN_IR[1]] = 1500;                            // lit room: both receivers see 1500 with the emitters off
  bootCal();
  simRun(300);
  send(":ir emit on\n"); simRun(100);
  send(":ir cal clear 0\n"); send(":ir cal clear 1\n"); simRun(50);
  g_mockReflect[PIN_IR[0]] = g_mockReflect[PIN_IR[1]] = 1500;                            // the ball reflects 1500 counts of OUR emitter (shooting position)
  simRun(100);
  send(":ir cal near 0\n"); send(":ir cal near 1\n"); simRun(50);
  CHECK(irCalibrated(g_cfg, 0) && irCalibrated(g_cfg, 1) && absf((float)g_cfg.irClear0 - 2048.0f) < 3.0f && g_cfg.irNear0 > g_cfg.irClear0);
  g_cfg.irOffsetSide = 1;
  // ambient light swings from dark to bright sunlight: nothing changes (it is in both the dark and the lit reading)
  g_mockReflect[PIN_IR[0]] = g_mockReflect[PIN_IR[1]] = 0;
  for (uint16_t amb : {0, 800, 2500, 3300}) {
    g_mockAmbient[PIN_IR[0]] = g_mockAmbient[PIN_IR[1]] = amb; simRun(150);
    CHECK(!g_irs[0].seen && !g_irs[1].seen && absf(g_irs[0].s) < 0.1f);
  }
  // the reflection is what counts, at any ambient level
  g_mockAmbient[PIN_IR[0]] = g_mockAmbient[PIN_IR[1]] = 2500;
  g_mockReflect[PIN_IR[0]] = 700; g_mockReflect[PIN_IR[1]] = 0; simRun(150);
  CHECK(g_irs[0].seen && !g_irs[0].near && !g_irs[1].seen && g_ball.seenMask == 1);     // only pair 0 sees a reflection
  g_mockReflect[PIN_IR[0]] = 1500; g_mockReflect[PIN_IR[1]] = 1400; simRun(150);
  CHECK(g_irs[0].near && g_irs[1].seen && g_ball.held && g_ball.ready);
  CHECK(g_irs[0].lit > g_irs[0].dark);
  // a receiver that does not react to its emitter (nothing reflects) shows strength about 0
  g_mockReflect[PIN_IR[0]] = g_mockReflect[PIN_IR[1]] = 0; simRun(150);
  CHECK(absf(g_irs[0].s) < 0.1f);
  simRun(500);
  CHECK(!g_ball.valid);
  // the same room WITHOUT emitter pulsing (emitters hard-wired on): ambient light alone looks like a ball
  send(":ir emit off\n"); simRun(100);
  g_mockAmbient[PIN_IR[0]] = g_mockAmbient[PIN_IR[1]] = 0; g_mockReflect[PIN_IR[0]] = g_mockReflect[PIN_IR[1]] = 0; simRun(100);
  send(":ir cal clear 0\n"); send(":ir cal clear 1\n"); simRun(50);
  g_mockReflect[PIN_IR[0]] = g_mockReflect[PIN_IR[1]] = 1500; simRun(100);
  send(":ir cal near 0\n"); send(":ir cal near 1\n"); simRun(50);
  g_mockReflect[PIN_IR[0]] = g_mockReflect[PIN_IR[1]] = 0; simRun(100);
  CHECK(!g_irs[0].seen);
  g_mockAmbient[PIN_IR[0]] = 1000; simRun(150);
  CHECK(g_irs[0].seen);                                                                  // sunlight on the receiver: a false "ball"
}

static void t_ir_emit_pair_independence_and_digital() {
  g_mockEmitModel = true;
  bootCal();
  simRun(300);
  send(":ir emit on\n"); simRun(100);
  send(":ir cal clear 0\n"); send(":ir cal clear 1\n"); simRun(50);
  g_mockReflect[PIN_IR[0]] = 2000; g_mockReflect[PIN_IR[1]] = 2000; simRun(100);
  send(":ir cal near 0\n"); send(":ir cal near 1\n"); simRun(50);
  g_cfg.irOffsetSide = 1;
  // pair 1's emitter lights only receiver 1: receiver 0's lit reading never contains pair 1's reflection
  g_mockReflect[PIN_IR[0]] = 0; g_mockReflect[PIN_IR[1]] = 2000; simRun(150);
  CHECK(g_irs[1].near && !g_irs[0].seen && g_ball.seenMask == 2 && g_ball.offset > 0.5f);
  CHECK(g_irs[0].lit == g_irs[0].dark);
  // a digital receiver (comparator output: HIGH when it sees the reflection) works the same way, with half-scale difference
  send(":ir mode 0 digital\n"); simRun(50);
  CHECK(g_cfg.irMode0 == 1 && !irCalibrated(g_cfg, 0));
  g_mockAmbient[PIN_IR[0]] = 0; g_mockReflect[PIN_IR[0]] = 0; simRun(100);
  send(":ir cal clear 0\n"); simRun(20);
  g_mockReflect[PIN_IR[0]] = 3000; simRun(100);
  send(":ir cal near 0\n"); simRun(20);
  CHECK(irCalibrated(g_cfg, 0));
  g_mockReflect[PIN_IR[0]] = 0; simRun(100); CHECK(!g_irs[0].seen);
  g_mockReflect[PIN_IR[0]] = 3000; simRun(100); CHECK(g_irs[0].seen && g_irs[0].near);
  // the rail check does not apply while the emitters are pulsed
  g_mockReflect[PIN_IR[1]] = 0; g_mockAmbient[PIN_IR[1]] = 0; simRun(g_cfg.irRailMs + 600);
  CHECK(!faultActive(F_IR_ALIGN) && !faultActive(F_IR_FRONT));
}

// ================================================================================================
// navigation with two sensors
static void t_nav_approach_and_stop() {
  bootCal();
  simRun(300);
  startAuto();
  CHECK(g_autoEnabled && g_navState == NAV_SEEK);
  ballFar(); simRun(300);
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_reqVy > 0.2f && absf(g_reqVx) < 0.05f);           // centred, far: forward, no sideways motion
  float vyFar = g_reqVy;
  sensors(0.6f, 0.6f); simRun(300);
  CHECK(g_reqVy < vyFar);                                   // slows as the signal strengthens
  ballHeld(); simRun(400);
  CHECK(g_ball.ready && g_reqVy == 0.0f && g_navState == NAV_APPROACH);   // shooting position: no forward motion
  ballFar(); simRun(300);
  CHECK(g_reqVy > 0.1f);                                    // ball moved away: approaches again
}

static void t_nav_single_sensor_alignment() {
  bootCal();
  simRun(300);
  startAuto();
  sensors(0.0f, 0.4f); simRun(300);                         // only sensor 1 (right) sees it: ball is to the right
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_reqVx > 0.05f && g_reqVx <= g_cfg.alignSpeed + 1e-3f);     // cautious sideways move to the right
  CHECK(g_reqVy <= g_cfg.slowSpeed + 1e-3f);                         // and slow forward
  sensors(0.4f, 0.0f); simRun(400);                         // only sensor 0: the ball is to the left
  CHECK(g_reqVx < -0.05f && g_reqVy <= g_cfg.slowSpeed + 1e-3f);
  // without a mounting side the robot makes no sideways guess
  g_cfg.irOffsetSide = 0; ballGone(); simRun(500); startAuto(); sensors(0.0f, 0.4f); simRun(400);
  CHECK(absf(g_reqVx) < 1e-4f);
}

static void t_nav_lost_search_reacquire() {
  bootCal();
  simRun(300);
  startAuto();
  sensors(0.0f, 0.5f); simRun(400);                         // seen on the right
  CHECK(g_navState == NAV_APPROACH);
  ballGone();
  simRun(g_cfg.lostHoldMs + 400);
  CHECK(g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE);
  CHECK(g_stats.ballLossEvents == 1);
  simRun(800);
  CHECK(g_navState == NAV_SEARCH && g_reqW > 0.1f);         // the ball was last seen on the right: turn clockwise first
  ballFar(); simRun(400);
  CHECK(g_navState == NAV_APPROACH && g_stats.reacquisitions == 1);
  // nothing ever found: searches, pauses, waits, never spins forever
  ballGone();
  simRun(g_cfg.lostHoldMs + 200);
  g_cfg.searchTimeoutMs = 2000; g_cfg.searchAttempts = 2; g_cfg.searchPauseMs = 500;
  simRun(15000);
  CHECK(g_navState == NAV_WAIT && allWheelsStopped());
}

static void t_nav_sensor_fault_safe() {
  bootCal();
  simRun(300);
  startAuto();
  ballFar(); simRun(500);
  CHECK(!allWheelsStopped());
  g_mockAnalog[PIN_IR[0]] = 0;                              // front sensor wiring fault
  simRun(g_cfg.irRailMs + 800);
  CHECK(faultActive(F_IR_FRONT) && !g_autoEnabled && allWheelsStopped() && g_cond == COND_SAFE_STOP);
}

// ================================================================================================
// goal heading and shot
static void shotSetup() {
  bootCal();
  g_cfg.shootEnable = 1; g_cfg.shootBackMs = 250; g_cfg.shootDriveMs = 400; g_cfg.shootHoldMs = 200; g_cfg.shootParallel = 0;
  simRun(300);
}
// Emulates the robot turning toward the goal heading when the aim phase commands it (the mock has no physics).
static void emulateAimRotation() {
  if (g_navState == NAV_SHOOT && g_shootPhase == 0) { float err = angDiffDeg(g_goalYaw, g_imu.yaw); g_mpu.gz = absf(err) < 0.5f ? 0.0f : (err < 0 ? 30.0f : -30.0f); }
  else g_mpu.gz = 0.0f;
}
// Rotation model for the timing comparison: ASSUMED 180 deg/s at full rotation command (the real value is unknown and must be measured).
static void emulateRotationFromCommand() {
  if (g_navState == NAV_SHOOT) g_mpu.gz = -180.0f * g_reqW; else g_mpu.gz = 0.0f;
}

static void t_goal_register_and_aim_window() {
  bootCal();
  simRun(200);
  CHECK(goalValid());                                       // boot takes longer than goalregms: registered as soon as the gyro was usable
  Serial.out.clear(); send(":goal\n"); simRun(20);
  CHECK(outHas("goal heading") && outHas("aim window"));
  send(":goal clear\n"); simRun(20); CHECK(!g_goalRefSet);
  send(":goal set\n"); simRun(20); CHECK(g_goalRefSet);
  send(":goal bogus\n"); simRun(20); CHECK(outHas("usage: :goal"));
  Config c; cfgDefaults(c);                                 // 450 mm goal, 43 mm ball, 1000 mm away
  CHECK_NEAR(shootAimTolDeg(c), 11.5, 0.2);
  c.goalWidthMm = 600; CHECK_NEAR(shootAimTolDeg(c), 15.5, 0.2);
  c.goalDistMm = 3000; CHECK_NEAR(shootAimTolDeg(c), 5.3, 0.2);
  c.goalDistMm = 200; CHECK_NEAR(shootAimTolDeg(c), 20.0, 1e-4);   // clamped
}

static void t_shoot_sequence() {
  shotSetup();
  ballFar();
  startAuto(); simRun(1500);
  CHECK(goalValid() && g_stats.shots == 0);                 // ball not in the shooting position: no shot
  g_mpu.gz = -40.0f; simRun(1000); g_mpu.gz = 0.0f; simRun(300);   // knock the robot 40 degrees off the goal heading (clockwise)
  CHECK(absf(angDiffDeg(g_goalYaw, g_imu.yaw)) > 25.0f);
  ballHeld();
  bool aimed = false, sawBack = false, sawDrive = false; int rotA = 0, backDir = 0, driveDir = 0, maxDrive = 0;
  uint64_t end = g_mockUs + 6000000ULL;
  while (g_mockUs < end && g_stats.shots == 0) {
    emulateAimRotation();
    loop(); mockAdvance(50);
    if (g_navState == NAV_SHOOT) {
      if (g_shootPhase == 0 && g_pwmDuty[WHEEL_PINS[0].en] > 0) { aimed = true; rotA = wheelDir(W_FL) * wheelDir(W_FR); }
      if (g_shootPhase == 1 && maxWheelDuty() > 0) { sawBack = true; backDir = wheelDir(W_FL) + wheelDir(W_FR) + wheelDir(W_BL) + wheelDir(W_BR); }
      if (g_shootPhase == 2) { sawDrive = true; driveDir = wheelDir(W_FL) + wheelDir(W_FR) + wheelDir(W_BL) + wheelDir(W_BR); maxDrive = std::max(maxDrive, maxWheelDuty()); }
    }
  }
  g_mpu.gz = 0.0f;
  CHECK(aimed && rotA < 0);                                 // aimed by rotating in place (FL and FR turn opposite ways)
  CHECK(sawBack && sawDrive);
  CHECK(backDir == -driveDir && driveDir != 0);             // reverse, then the opposite direction
  CHECK(maxDrive >= (int)(g_cfg.maxAuto * 0.9f));           // the drive phase reaches (nearly) full speed
  CHECK(g_stats.shots == 1 && g_navState != NAV_SHOOT);
  CHECK(absf(angDiffDeg(g_goalYaw, g_imu.yaw)) <= 0.5f * shootAimTolDeg(g_cfg) + 1.0f);   // fired inside the aim window
  simRun(1500);
  CHECK(g_stats.shots == 1);                                // cooldown / same ball: no second shot
}

static void t_shoot_preconditions() {
  shotSetup();
  g_cfg.shootEnable = 0;
  ballHeld(); simRun(300);
  startAuto();
  simRun(1500);
  CHECK(g_stats.shots == 0 && g_navState != NAV_SHOOT);     // shot switched off: holding the ball alone never shoots
  g_cfg.shootEnable = 1; send(":goal clear\n"); simRun(1500);
  CHECK(g_stats.shots == 0 && g_navState != NAV_SHOOT);     // no registered goal heading: no shot
  sensors(1.0f, 0.3f); simRun(100);                         // (let the sensor states settle first)
  send(":goal set\n"); simRun(1500);
  CHECK(g_stats.shots == 0 && g_navState != NAV_SHOOT);     // held but off-centre: the robot aligns, it does not shoot
  sensors(1.0f, 0.0f); simRun(1500);
  CHECK(g_stats.shots == 0 && g_navState != NAV_SHOOT);     // sensor 1 does not see the ball: not the shooting position
  send("s"); simRun(100);
  CHECK(!g_autoEnabled);
}

static void t_shoot_aim_timeout_and_ball_lost() {
  shotSetup();
  ballFar();
  startAuto(); simRun(1200);
  g_mpu.gz = -40.0f; simRun(1000); g_mpu.gz = 0.0f; simRun(300);
  ballHeld();                                               // held, but the emulated robot never turns: the aim must time out
  uint64_t end = g_mockUs + 6000000ULL; bool entered = false;
  while (g_mockUs < end && g_stats.shots == 0) { loop(); mockAdvance(50); if (g_navState == NAV_SHOOT) entered = true; if (entered && g_navState != NAV_SHOOT) break; }
  CHECK(entered && g_navState != NAV_SHOOT && g_stats.shots == 0);
  g_cfg.shootCooldownMs = 500; simRun(700);
  ballGone(); simRun(400);
  ballHeld(); simRun(400);                                  // held again, aim starts
  ballGone();                                               // ball taken away during the aim: cancelled well before the timeout
  uint64_t t0 = g_mockUs; bool inShot = false;
  while (g_mockUs < t0 + 2000000ULL) { loop(); mockAdvance(50); if (g_navState == NAV_SHOOT) inShot = true; if (inShot && g_navState != NAV_SHOOT) break; }
  CHECK(g_navState != NAV_SHOOT);
}

static void t_goal_invalid_after_gyro_reset() {
  shotSetup();
  ballFar();
  startAuto(); simRun(800);
  CHECK(goalValid());
  ballHeld(); simRun(100);
  send("s"); simRun(100);
  send(":zero\n"); simRun(100);                             // the gyro reference is reset: the registered heading no longer means anything
  CHECK(g_goalRefSet && !goalValid());
  Serial.out.clear(); simRun(200); send(":goal\n"); simRun(20);
  CHECK(outHas("INVALID"));
  startAuto(); simRun(1500);
  CHECK(g_stats.shots == 0 && g_navState != NAV_SHOOT);     // no shot on a stale heading
  send(":goal set\n"); simRun(300);
  CHECK(goalValid());
  uint32_t ep = g_imu.yawEpoch;                             // an IMU outage (the yaw is reset to 0 on recovery) invalidates it again
  g_mpu.mode = 1; simRun(800); g_mpu.mode = 0; simRun(3500);
  CHECK(g_imu.yawEpoch > ep && !goalValid());
}

static void t_shoot_launch_clock_and_one_shot_per_ball() {
  shotSetup();
  g_cfg.shootCooldownMs = 500;
  ballFar();
  startAuto(); simRun(1200);
  ballHeld();
  uint64_t tLaunch = 0, tFull = 0, tEnd = 0; int shots0 = 0;
  uint64_t end = g_mockUs + 5000000ULL;
  while (g_mockUs < end && g_stats.shots == 0) {
    emulateAimRotation(); loop(); mockAdvance(50);
    if (g_navState == NAV_SHOOT && g_shootPhase == 2 && g_shootLaunched && !tLaunch) tLaunch = g_mockUs;
    if (tLaunch && !tFull && maxWheelDuty() >= (int)(g_cfg.maxAuto * 0.9f)) tFull = g_mockUs;
  }
  tEnd = g_mockUs;
  CHECK(g_stats.shots == 1 && tLaunch && tFull);
  printf("  launch->full speed %.0f ms\n", (double)(tFull - tLaunch) / 1000.0);
  CHECK(tFull - tLaunch < 120000ULL);                       // the ramp was lifted: (nearly) full speed within ~100 ms of the wheels turning forward
  CHECK_NEAR((double)(tEnd - tLaunch) / 1000.0, (double)g_cfg.shootDriveMs, 80.0);   // the drive lasts shootdrivems from the launch, not from the dwell
  CHECK(!g_rampBoost);
  shots0 = (int)g_stats.shots; simRun(3000);                // the ball is still there: no second shot however long we wait
  CHECK((int)g_stats.shots == shots0);
  ballGone(); simRun(400); ballHeld();                      // the ball leaves and a new one is taken: the next shot is allowed
  end = g_mockUs + 5000000ULL;
  while (g_mockUs < end && (int)g_stats.shots == shots0) { emulateAimRotation(); loop(); mockAdvance(50); }
  CHECK((int)g_stats.shots == shots0 + 1);
}

struct ShotTiming { double toLaunchMs, totalMs; bool ok; };
// Several scenarios in one process: setup() does not clear these (a real reboot does).
static void resetShotState() {
  g_stats.shots = 0; g_goalRefSet = false; g_goalAutoOff = false; g_goalWaitNoted = false; g_goalLostNoted = false;
  g_shotDone = false; g_needRelease = false; g_shootLaunched = false; g_rampBoost = false; g_shootPhase = 0; g_mpu.gz = 0.0f;
}
static ShotTiming shotTiming(float offsetDeg, bool fast) {
  resetShotState();
  shotSetup();
  if (!fast) { g_cfg.shootParallel = 0; g_cfg.shootHoldMs = 300; g_cfg.shootBackMs = 250; g_cfg.shootBackSpeed = 0.5f; g_cfg.aimSpeed = 0.3f; g_cfg.shootPwm = g_cfg.maxAuto; }
  else { g_cfg.shootParallel = 1; g_cfg.shootHoldMs = 100; g_cfg.shootBackMs = 150; g_cfg.shootBackSpeed = 0.7f; g_cfg.aimSpeed = 0.5f; g_cfg.shootPwm = 255; }
  ballFar();
  startAuto(); simRun(1200);
  if (offsetDeg != 0.0f) { g_mpu.gz = -offsetDeg; simRun(1000); g_mpu.gz = 0.0f; simRun(300); }
  uint32_t t0 = millis();
  ballHeld();
  uint64_t end = g_mockUs + 8000000ULL;
  while (g_mockUs < end && g_stats.shots == 0) { emulateRotationFromCommand(); loop(); mockAdvance(50); }
  g_mpu.gz = 0.0f;
  ShotTiming r{0, 0, g_stats.shots == 1};
  r.toLaunchMs = (double)(uint32_t)(g_shotLaunchMs - t0);
  r.totalMs = (double)(uint32_t)(millis() - t0);
  return r;
}
static void t_shot_speed_old_vs_new() {
  ShotTiming o0 = shotTiming(0.0f, false), n0 = shotTiming(0.0f, true), o40 = shotTiming(40.0f, false), n40 = shotTiming(40.0f, true);
  CHECK(o0.ok && n0.ok && o40.ok && n40.ok);
  printf("  shot: sequential settings launch after %.0f ms (total %.0f) at 0 deg off, %.0f (%.0f) at 40 deg off; defaults %.0f (%.0f) and %.0f (%.0f) [simulation, assumed 180 deg/s at full rotation command]\n",
         o0.toLaunchMs, o0.totalMs, o40.toLaunchMs, o40.totalMs, n0.toLaunchMs, n0.totalMs, n40.toLaunchMs, n40.totalMs);
  CHECK(n0.toLaunchMs < o0.toLaunchMs && n40.toLaunchMs < o40.toLaunchMs);
  CHECK(n0.totalMs < o0.totalMs && n40.totalMs < o40.totalMs);
}
static void t_shot_parallel_aims_while_reversing() {
  shotSetup();
  g_cfg.shootParallel = 1; g_cfg.shootBackMs = 150; g_cfg.shootBackSpeed = 0.7f; g_cfg.aimSpeed = 0.5f; g_cfg.shootPwm = 255;
  ballFar();
  startAuto(); simRun(1200);
  g_mpu.gz = -40.0f; simRun(1000); g_mpu.gz = 0.0f; simRun(300);
  ballHeld();
  bool both = false, seqPhase1 = false; int maxDrive = 0;
  uint64_t end = g_mockUs + 6000000ULL;
  while (g_mockUs < end && g_stats.shots == 0) {
    emulateRotationFromCommand(); loop(); mockAdvance(50);
    if (g_navState == NAV_SHOOT) {
      if (g_shootPhase == 0 && g_reqVy < -0.1f && absf(g_reqW) > 0.05f) both = true;     // reversing and turning at the same time
      if (g_shootPhase == 1) seqPhase1 = true;
      if (g_shootPhase == 2) maxDrive = std::max(maxDrive, maxWheelDuty());
    }
  }
  g_mpu.gz = 0.0f;
  CHECK(both && !seqPhase1);
  CHECK(g_stats.shots == 1);
  CHECK(maxDrive >= 250);                                   // shootpwm 255 overrides the autonomous limit (maxauto = 170)
  CHECK(absf(angDiffDeg(g_goalYaw, g_imu.yaw)) <= 0.5f * shootAimTolDeg(g_cfg) + 1.5f);
}
static void t_shoot_pwm_limit_and_no_reverse() {
  shotSetup();
  g_cfg.shootPwm = 200; g_cfg.shootBackMs = 0; g_cfg.shootParallel = 1;
  ballFar();
  startAuto(); simRun(1200);
  ballHeld();
  int maxDrive = 0; bool reversed = false;
  uint64_t end = g_mockUs + 4000000ULL;
  while (g_mockUs < end && g_stats.shots == 0) {
    loop(); mockAdvance(50);
    if (g_navState == NAV_SHOOT) { if (g_reqVy < -0.05f) reversed = true; if (g_shootPhase == 2) maxDrive = std::max(maxDrive, maxWheelDuty()); }
  }
  CHECK(g_stats.shots == 1 && !reversed);                   // shootbackms 0: no reverse phase at all
  CHECK(maxDrive >= 195 && maxDrive <= 200);                // limited by shootpwm, not by maxauto
}

// ================================================================================================
// the old hardware is gone
static void t_old_hardware_removed() {
  bootCal();
  simRun(200);
  Serial.out.clear();
  const char* cmds[] = {":ch 3 en 1\n", ":learn 90\n", ":verify 2\n", ":irgate auto\n", ":irprofile fast\n", ":cov\n", ":irchar 3\n", ":aux on\n", ":avoid\n", ":hold\n", ":chreset 1\n"};
  for (const char* c : cmds) { send(c); simRun(30); }
  CHECK(outCount("unknown word command") == 11);           // none of the multiplexer-era commands exists any more
  Serial.out.clear(); send("?"); simRun(50); send("h"); simRun(300); send("i"); simRun(50); send("d"); simRun(50);
  CHECK(!outHas("mux") && !outHas("Mux") && !outHas("multiplexer") && !outHas("TSOP") && !outHas("C0 ") && !outHas("C11"));
  CHECK(outHas("IR sensor"));
  // only the specified pins exist: 12 motor + 2 I2C outputs, 2 input-only sensor pins
  CHECK(OUTPUT_PIN_MASK != 0 && __builtin_popcountll(OUTPUT_PIN_MASK) == 14 && __builtin_popcountll(IR_PIN_MASK) == 2);
  CHECK(strstr(LOG_COLUMNS, "irmask,ball,offset,s0,s1") != nullptr && strstr(LOG_COLUMNS, "actmask") == nullptr && strstr(LOG_COLUMNS, "bear_deg") == nullptr);
}

struct TestCase { const char* name; void (*fn)(); };
#define T(n) {#n, n}
static const TestCase TESTS[] = {
  T(t_ir_emit_pins_untouched_by_default), T(t_ir_emit_ambient_rejection), T(t_ir_emit_pair_independence_and_digital),
  T(t_angles),
  T(t_timer_rollover),
  T(t_mecanum),
  T(t_pinmap),
  T(t_boot_safe),
  T(t_ramp),
  T(t_config),
  T(t_imu_integrate),
  T(t_imu_bias),
  T(t_imu_missing),
  T(t_imu_dropout),
  T(t_imu_invalid),
  T(t_heading_hold),
  T(t_safe_stop_moving),
  T(t_loop_stall),
  T(t_resets),
  T(t_fault_report_format),
  T(t_reset_wdt),
  T(t_cli_manual),
  T(t_cli_robust),
  T(t_diag_no_motion),
  T(t_motor_test),
  T(t_drive_test),
  T(t_fs_commands),
  T(t_gpio_tamper),
  T(t_cfg_boot_corrupt),
  T(t_cfg_save_fail),
  T(t_cfg_set),
  T(t_fs_missing),
  T(t_fs_write_failure),
  T(t_fs_full),
  T(t_log_rotation),
  T(t_log_queue_overflow),
  T(t_flight_recorder),
  T(t_export),
  T(t_nonblocking),
  T(t_reboot_phase1),
  T(t_reboot_phase2),
  T(t_cal_startup_defaults),
  T(t_health_report_text),
  T(t_imu_frozen_in_approach),
  T(t_ir_pure_rules),
  T(t_ir_states_and_debounce),
  T(t_ir_polarity_and_digital),
  T(t_ir_rail_fault_and_uncalibrated),
  T(t_ir_uncalibrated_autonomy_refused),
  T(t_ir_cli),
  T(t_ball_offset_and_shooting_position),
  T(t_nav_approach_and_stop),
  T(t_nav_single_sensor_alignment),
  T(t_nav_lost_search_reacquire),
  T(t_nav_sensor_fault_safe),
  T(t_goal_register_and_aim_window),
  T(t_shoot_sequence),
  T(t_shoot_preconditions),
  T(t_shoot_aim_timeout_and_ball_lost),
  T(t_goal_invalid_after_gyro_reset),
  T(t_shoot_launch_clock_and_one_shot_per_ball),
  T(t_shot_speed_old_vs_new),
  T(t_shot_parallel_aims_while_reversing),
  T(t_shoot_pwm_limit_and_no_reverse),
  T(t_old_hardware_removed),
};

int main(int argc, char** argv) {
  if (argc >= 2 && strcmp(argv[1], "--list") == 0) { for (auto& t : TESTS) printf("%s\n", t.name); return 0; }
  if (argc < 2) { fprintf(stderr, "usage: %s <test>|--list\n", argv[0]); return 2; }
  for (auto& t : TESTS) {
    if (strcmp(t.name, argv[1]) != 0) continue;
    if (strncmp(t.name, "t_reboot", 8) != 0 && strcmp(t.name, "t_config") != 0 && strncmp(t.name, "t_fs_", 5) != 0 && strcmp(t.name, "t_cfg_boot_corrupt") != 0 &&
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
