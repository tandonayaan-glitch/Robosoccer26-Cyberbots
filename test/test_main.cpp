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
#include <vector>
#include <string>
#include "telemetry_check.h"
#include "test_remote.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)
#define CHECK_NEAR(a, b, tol) do { g_checks++; double _a = (a), _b = (b); if (!(fabs(_a - _b) <= (tol))) { g_fail++; printf("  FAIL line %d: %s (=%g) !~ %s (=%g) tol %g\n", __LINE__, #a, _a, #b, _b, (double)(tol)); } } while (0)

// ---- harness helpers ---------------------------------------------------------------------------
static void simRun(uint32_t ms, uint32_t loopCostUs = 50) {
  uint64_t end = g_mockUs + (uint64_t)ms * 1000ULL;
  while (g_mockUs < end) { loop(); mockAdvance(loopCostUs); }
}
static void send(const char* s) { Serial.feed(s); }
static int outCountOf(const std::string& hay, const char* s) { int n = 0; size_t p = 0; while ((p = hay.find(s, p)) != std::string::npos) { n++; p += strlen(s); } return n; }
static bool outHas(const char* s) { return Serial.out.find(s) != std::string::npos; }
static int outCount(const char* s) { int n = 0; size_t p = 0; while ((p = Serial.out.find(s, p)) != std::string::npos) { n++; p += strlen(s); } return n; }
static void freshEeprom(const char* name) {          // a private EEPROM image per test case; erased (0xFF) unless the file already exists
  const char* tempDir = getenv("TMPDIR");
  if (!tempDir || !*tempDir) tempDir = getenv("TEMP");
  if (!tempDir || !*tempDir) tempDir = "/tmp";
  g_eepromPath = std::string(tempDir) + "/mockeeprom_" + name;
  ::remove(g_eepromPath.c_str());
  EEPROM.reset();
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

// Sensor model of the tests: the ring channel at (or near) the ball's bearing sees a burst swing proportional to `strength` * ringnear, falling off
// with the angle between the channel's configured bearing and the ball (cosine, zero beyond the assumed half field of view). Every channel also
// has a DC level (steady light), which the swing measurement must ignore. These numbers are test inputs, not measurements.
static float g_tBrg = 0.0f;
static void ballAt(float deg, float strength = 0.4f) {
  g_tBrg = deg;
  for (int ch = 0; ch < 8; ch++) {
    float d = absf(angDiffDeg((float)g_cfg.ringBearing[ch], deg));
    float f = d < (float)g_cfg.ringFovHalf ? cosf(d / (float)g_cfg.ringFovHalf * 1.5707963f) : 0.0f;
    g_mockRingBase[ch] = 300;
    g_mockRingAmp[ch] = (uint16_t)(strength * (float)g_cfg.ringNear * f + 0.5f);
  }
}
static void ballGone() { ballAt(0.0f, 0.0f); g_mockModDetect[0] = false; g_mockUnderDetect = false; }
static void ballFar() { ballAt(0.0f, 0.35f); g_mockModDetect[0] = false; g_mockUnderDetect = false; }          // ahead, not near
static void ballHeld() { ballAt(0.0f, 1.0f); g_mockModDetect[0] = true; g_mockUnderDetect = true; }           // ahead, front module detecting: the shooting position
static void calibrateAll() { g_cfg.ringCalDone = 1; }
// The default configuration has only A0 and A1 (both "straight ahead"). Most tests want 8 receivers 45 deg apart: A0 = 0 deg, clockwise.
static void layout8() {
  g_cfg.bearingsSet = 1;                         // the test layout stands in for operator-entered mounting angles
  g_cfg.ringEnable = 0x00FF;
  for (int i = 0; i < 8; i++) g_cfg.ringBearing[i] = (int16_t)(i <= 4 ? i * 45 : i * 45 - 360);
}
static void bootCal() { ballGone(); setup(); g_autoStartPending = false; layout8(); calibrateAll(); }       // (tests start idle; t_autostart covers the real boot)
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


// The firmware's own printf (compat.h) replaces avr-libc's, which has no floating point: it must agree with the C library on everything the firmware prints.
static void fmtCheck(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void fmtCheck(const char* fmt, ...) {
  char a[96], b[96];
  va_list ap, ap2;
  va_start(ap, fmt); va_copy(ap2, ap);
  vsnprintf(a, sizeof(a), fmt, ap);
  rsFormat(b, sizeof(b), fmt, false, ap2);
  va_end(ap); va_end(ap2);
  g_checks++;
  if (strcmp(a, b) != 0) { g_fail++; printf("  FAIL fmt '%s': libc '%s' vs rsFormat '%s'\n", fmt, a, b); }
}
static void t_minifmt() {
  fmtCheck("%d|%i|%u|%x|%X|%04X|%02x|%c|%%", -42, 7, 65535u, 0xBEEF, 0xBEEF, 0x2F, 0xA, 'Z');
  fmtCheck("%ld|%lu|%lx|%08lX", -123456L, 4000000000UL, 0xDEADBEEFUL, 0xABCUL);
  fmtCheck("[%5d][%-5d][%05d][%+d][% d]", 42, 42, 42, 42, 42);
  fmtCheck("[%5u][%-5u][%3s][%-6s][%.2s]", 7u, 7u, "ab", "ab", "abcdef");
  fmtCheck("%.0f|%.1f|%.2f|%.3f|%5.1f|%-8.2f|%08.3f", 2.6, 2.54, 2.555, -0.0004, 3.14159, 3.14159, -3.14159);
  fmtCheck("%.1f|%.2f|%.0f", 0.05, 99.995, 0.4);
  fmtCheck("%f|%6.2f|%.2f", 1.5, 100.0, 1234567.891);
  fmtCheck("%g|%g|%g|%g", 0.5, 12.0, 0.1, 300.0);
  fmtCheck("%.1f %.1f", 360.0, -180.0);
  fmtCheck("%s|%s", "", "x");
  fmtCheck("%-2u|%3.0f|%4.0f", 3u, 12.6, 7.0);
  char buf[8];
  rsSnprintfP(buf, sizeof(buf), "%s", "0123456789");                    // truncation is safe and terminated
  CHECK(strlen(buf) == 7 && buf[7] == 0);
  CHECK(rsSnprintfP(buf, 0, "x") == 0);
}

// A long report is longer than the 512-byte output queue: it must come out complete, in order, once, and the main loop must not stall while it prints.
static void t_report_paced() {
  bootCal();
  simRun(300);
  Serial.out.clear();
  send("h");
  uint64_t worst = 0;
  for (int k = 0; k < 40000 && Serial.out.size() < 20000; k++) {
    uint64_t t0 = g_mockUs; loop(); uint64_t dt = g_mockUs - t0; if (dt > worst) worst = dt; mockAdvance(50);
    if (outHas("limits: no wheel feedback") && outHas("========================================================")) break;
  }
  simRun(300);
  CHECK(outCount("HARDWARE HEALTH REPORT") == 1);                        // once, not repeated by the paging
  CHECK(outCount("========================================================") == 1);   // the closing line, once
  CHECK(outHas("[Ring] receivers on A0..A7") && outHas("stats: shots=") && outHas("limits: no wheel feedback"));
  CHECK(worst < 900);                                                    // never a wait on Serial while it printed
  CHECK(g_outDropped == 0);
  // every line is complete: each '[' line start is preceded by a newline
  size_t pos = 0; bool ok = true;
  while ((pos = Serial.out.find("[INFO] ", pos)) != std::string::npos) { if (pos != 0 && Serial.out[pos - 1] != '\n') ok = false; pos += 7; }
  CHECK(ok);
  // the other long reports
  Serial.out.clear(); send("?"); simRun(1500);
  CHECK(outHas("commands (115200 baud") && outHas("Safety: first motor tests"));
  Serial.out.clear(); send("i"); simRun(1500);
  CHECK(outHas("ball receivers: 8 of 8") && outHas("IR module 1 D35") && outHas("a module reacts to ANY reflective object"));
}

// ---- event records in the EEPROM ------------------------------------------------------------------------------
static std::vector<std::string> outLinesWith(const char* marker) {      // all output lines containing `marker`
  std::vector<std::string> v; size_t p = 0;
  while (p < Serial.out.size()) { size_t e = Serial.out.find('\n', p); if (e == std::string::npos) e = Serial.out.size(); std::string l = Serial.out.substr(p, e - p); if (l.find(marker) != std::string::npos) v.push_back(l); p = e + 1; }
  return v;
}
static void t_records() {
  bootCal();
  simRun(500);
  CHECK(g_recCount >= 1);                                                // the BOOT record of this start
  uint16_t base = g_recCount;
  recAdd(REC_NOTE, 77);
  startAuto(); ballFar(); simRun(500);                                   // SEEK -> APPROACH transitions are recorded
  ballGone(); simRun(g_cfg.lostHoldMs + 600);                            // ball lost, search
  CHECK(g_recCount > base + 2);
  send(":save\n"); simRun(200);
  // dump: complete, in order, once, from the first marker to the end marker
  Serial.out.clear();
  send(":log\n"); simRun(3000);
  CHECK(outCount("#RECORDS") == 1 && outCount("#END-RECORDS") == 1);
  CHECK(outHas(",BOOT,0") && outHas(",NOTE,77") && outHas(",NAV,IDLE>SEEK") && outHas(",NAV,SEEK>APPROACH") && outHas(",BALL_LOST,last_bearing_deg=") && outHas(",CFG_SAVED,0"));
  // power cycle: the records survive, the sequence numbers continue, a new BOOT record is added
  uint16_t before = g_recCount, seqBefore = g_recSeq;
  EEPROM.reset(); g_autoEnabled = false; g_navState = NAV_IDLE; ballGone(); setup();
  CHECK(g_recCount == before + 1 && g_recSeq == (uint16_t)(seqBefore + 1));
  Serial.out.clear(); send(":log\n"); simRun(3000);
  CHECK(outCount(",BOOT,0") == 2 && outHas(",NOTE,77"));
  // the user's marker and clear
  send(":log note 5\n"); simRun(50);
  send(":log clear\n"); simRun(50);
  CHECK(g_recCount == 0);
  Serial.out.clear(); send(":log\n"); simRun(1000);
  CHECK(outHas("#RECORDS 0 stored") && outHas("#END-RECORDS"));
}

// More events than slots: the ring overwrites the oldest, keeps the order, and the dump lists exactly REC_SLOTS records oldest-first.
static void t_records_wrap() {
  bootCal();
  simRun(300);
  recClear();
  for (int i = 0; i < REC_SLOTS * 2 + 17; i++) recAdd(REC_FAULT, (uint16_t)(i % F_COUNT));
  CHECK(g_recCount == REC_SLOTS);
  Serial.out.clear();
  send(":log\n");
  for (int k = 0; k < 40000 && !outHas("#END-RECORDS"); k++) { loop(); mockAdvance(100); }
  std::vector<std::string> lines = outLinesWith(",FAULT,");
  CHECK((int)lines.size() == REC_SLOTS);
  long prev = -1; bool inOrder = true;
  for (auto& l : lines) { size_t c = l.find(']'); long seq = atol(l.c_str() + (c == std::string::npos ? 0 : c + 1)); (void)seq; }
  for (auto& l : lines) { long seq = atol(l.c_str()); if (seq <= prev) inOrder = false; prev = seq; }
  CHECK(inOrder);
  CHECK(prev == REC_SLOTS * 2 + 17);                                     // the newest record is the last one written
  CHECK(outCount("#END-RECORDS") == 1);
  // a routine-record flood is limited (6 per second) and counted
  recClear(); g_recDropped = 0;
  uint32_t t0 = g_recCount;
  for (int i = 0; i < 50; i++) recAdd(REC_NOTE, (uint16_t)i);
  CHECK(g_recCount - t0 <= 6 && g_recDropped >= 44);
  // important records are never dropped by the flood guard
  recAdd(REC_FAULT, 1); recAdd(REC_SAFE_STOP, 0);
  CHECK(g_recCount - t0 <= 8);
}

// Parts that are missing: the robot still starts, reports it, and can be driven by hand; nothing blocks.
static void t_missing_hardware() {
  g_mpu.present = false;                                                 // no MPU6050 on the bus
  ballGone(); setup(); layout8(); calibrateAll();
  CHECK(g_navState == NAV_IDLE && g_cond == COND_DEGRADED && !g_imu.present);   // it booted; the IMU fault only degrades
  simRun(300);
  send(":status\n"); simRun(800);
  CHECK(outHas("IMU MISSING") && outHas("condition DEGRADED"));
  Serial.out.clear(); send(":raw\n"); simRun(800);
  CHECK(outHas("IMU: NOT FOUND") && outHas("raw analog"));
  send("f"); simRun(200);
  CHECK(wheelDir(W_FL) > 0 && wheelDir(W_FR) > 0);                       // manual driving works without the IMU
  send("s"); simRun(100);
  startAuto(); ballFar(); simRun(600);
  CHECK(g_autoEnabled && g_navState == NAV_APPROACH && g_reqVy > 0.1f);  // autonomy works without heading hold
  CHECK(isnan(g_hdgErr) && !imuHeadingUsable(g_cfg, millis()));
  send("s"); simRun(100);
  // no receivers enabled at all: SAFE_STOP (sensing), autonomy refused, manual driving still allowed
  g_cfg.ringEnable = 0; simRun(g_cfg.ringRailMs + 1500);
  CHECK(faultActive(F_RING_NONE) && g_cond == COND_SAFE_STOP);
  Serial.out.clear(); startAuto();
  CHECK(!g_autoEnabled && outHas("REFUSED"));
  send("f"); simRun(200);
  CHECK(wheelDir(W_FL) > 0);
  send("!"); simRun(50);
  CHECK(allWheelsStopped());
  // modules: an active-LOW module gets the internal pull-up (a missing wire reads "nothing detected"), an active-HIGH one does not
  g_mpu.present = true;
  bootCal(); g_cfg.modEnable = 3; simRun(100);
  CHECK(g_pinMode[PIN_MOD[0]] == INPUT_PULLUP && g_pinMode[PIN_MOD[1]] == INPUT_PULLUP);
  CHECK(!g_mod[0].active && !g_mod[1].active);
  send(":mod level 0 high\n"); simRun(50);
  CHECK(g_pinMode[PIN_MOD[0]] == INPUT && g_pinMode[PIN_MOD[1]] == INPUT_PULLUP);
}

// Every command answers; the status and diagnosis commands show what is there.
static void t_commands() {
  bootCal();
  simRun(500);
  Serial.out.clear(); send(":status\n"); simRun(800);
  CHECK(outHas("uptime") && outHas("receivers: 8 on") && outHas("IMU present") && outHas("records"));
  Serial.out.clear(); send(":pins\n"); simRun(800);
  CHECK(outHas("FR: IN1 D22 IN2 D23 ENA D5") && outHas("BL: IN3 D28 IN4 D29 ENB D10") && outHas("front D34, second D35") && outHas("SDA D20 SCL D21"));
  Serial.out.clear(); send(":raw\n"); simRun(800);
  CHECK(outHas("raw analog") && outHas("A0=") && outHas("A7=") && outHas("raw module 0 (D34)") && outHas("raw module 1 (D35)") && outHas("IMU: present"));
  Serial.out.clear(); send(":ver\n"); simRun(100);
  CHECK(outHas("Arduino Mega 2560"));
  send(":auto on\n"); simRun(100);
  CHECK(g_autoEnabled);
  send(":auto off\n"); simRun(100);
  CHECK(!g_autoEnabled && allWheelsStopped());
  send(":auto on\n"); simRun(100); send(":stop\n"); simRun(100);
  CHECK(!g_autoEnabled);
  send(":auto on\n"); simRun(100); send(":estop\n"); simRun(100);
  CHECK(!g_autoEnabled && allWheelsStopped());
  send(":recover\n"); simRun(50);                                       // :estop latched the emergency stop; :recover clears the latch
  send(":manual l\n"); simRun(200);
  CHECK(wheelsMatchStrafeRight() == false && wheelDir(W_FL) < 0);        // strafe left
  send(":manual x\n"); simRun(50);
  CHECK(outHas("usage: :manual"));
  send(":log live on\n"); simRun(50);
  Serial.out.clear(); recAdd(REC_SAFE_STOP, 9); simRun(100);
  CHECK(outHas(",SAFE_STOP,9"));                                              // printed the moment it was recorded
  send(":log live off\n"); simRun(50);
  Serial.out.clear(); send("g"); simRun(100);
  CHECK(outHas("records in the EEPROM"));
  Serial.out.clear(); send(":help\n"); simRun(2500);
  for (const char* w : {":log", ":status", ":raw", ":pins", ":auto on|off", ":manual", ":set", ":ring", ":mod", ":save", ":goal", ":mtest", ":recover", ":diag"}) CHECK(outHas(w));
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
  struct Spec { int in1, in2, en; } spec[4] = {{26, 27, 9}, {22, 23, 5}, {28, 29, 10}, {24, 25, 6}};  // FL FR BL BR (Mega D numbers)
  for (int i = 0; i < 4; i++) {
    CHECK(WHEEL_PINS[i].in1 == spec[i].in1 && WHEEL_PINS[i].in2 == spec[i].in2 && WHEEL_PINS[i].en == spec[i].en);
  }
  // receivers on A0..A7 (pins 54..61), each its own analog input; modules D34 / D35; MPU6050 on the Mega hardware I2C pins D20 / D21
  for (int k = 0; k < 8; k++) CHECK(PIN_RECV[k] == A0 + k && PIN_RECV[k] >= 54 && PIN_RECV[k] <= 61);
  CHECK(PIN_RECV[0] == A0 && PIN_RECV[1] == A0 + 1 && A0 == 54);
  CHECK(PIN_MOD[0] == 34 && PIN_MOD[1] == 35);
  for (int i = 0; i < 4; i++) CHECK(digitalPinToTimer(WHEEL_PINS[i].en));                  // D5, D6, D9, D10 are PWM-capable
  for (int k = 0; k < 2; k++) CHECK(PIN_MOD[k] != PIN_I2C_SDA && PIN_MOD[k] != PIN_I2C_SCL && PIN_MOD[k] != 0 && PIN_MOD[k] != 1);
  for (int k = 0; k < 2; k++) for (int i = 0; i < 4; i++) CHECK(PIN_MOD[k] != WHEEL_PINS[i].in1 && PIN_MOD[k] != WHEEL_PINS[i].in2 && PIN_MOD[k] != WHEEL_PINS[i].en);
  for (int k = 30; k <= 33; k++) { bool used = false; for (int i = 0; i < 4; i++) used |= k == WHEEL_PINS[i].in1 || k == WHEEL_PINS[i].in2 || k == WHEEL_PINS[i].en; CHECK(!used); }   // D30..D33 are free (no multiplexer)
  CHECK(PIN_I2C_SDA == 20 && PIN_I2C_SCL == 21);
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
  for (int i = 0; i < 70; i++) g_pinLevel[i] = 1;     // pretend the pins floated high
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
  CHECK(outHas("autostart"));
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
  c.ringSeen = 5000;               CHECK(cfgValidate(c, rep, sizeof(rep)) == 1 && c.ringSeen == d.ringSeen && strstr(rep, "ringseen"));
  c.maxAuto = 0;                   CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.maxAuto == d.maxAuto);
  c.ringRailHi = 100;               CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.ringRailHi == d.ringRailHi);   // below the allowed range
  c.reverse[2] = 7;                CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.reverse[2] == 0);
  c.ringBearing[4] = 400;          CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.ringBearing[4] == d.ringBearing[4]);
  c.ringSeen = 400; c.ringNear = 300; CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.ringSeen < c.ringNear);   // "seen" above "very close" is impossible
  c.minPwm = 200; c.maxManual = 100; CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.minPwm < c.maxManual);
  c.imuYawSign = 3;                CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.imuYawSign == -1);
  c.imuAddrPref = 0x30;            CHECK(cfgValidate(c, rep, sizeof(rep)) >= 1 && c.imuAddrPref == 0);
  CHECK(cfgValidate(c, rep, sizeof(rep)) == 0);
  // every default is inside its own declared range
  for (size_t i = 0; i < PARAM_COUNT; i++) { ParamDef p; paramAt(i, &p); CHECK(paramInRange(p, paramGet(d, p))); }
  // CRC / persistence in the EEPROM
  freshEeprom("config");
  Config l;
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_NOFILE && l.maxAuto == d.maxAuto);            // erased EEPROM (0xFF) = no saved config
  Config s; cfgDefaults(s); s.maxAuto = 123; s.ringNear = 300; s.ringBearing[2] = 75;
  CHECK(cfgSave(s));
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_OK);
  CHECK(l.maxAuto == 123 && l.ringNear == 300 && l.ringBearing[2] == 75);
  EEPROM.mem[40] ^= 0xAB; EEPROM.flush();                                                // corrupt one byte -> CRC failure -> defaults
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_BADCRC && l.maxAuto == d.maxAuto);
  { Config v; cfgDefaults(v); v.version = 99; cfgSeal(v); EEPROM.put(0, v); }
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_BADVERSION);
  { Config v; cfgDefaults(v); v.magic = 1; cfgSeal(v); EEPROM.put(0, v); }
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_BADMAGIC);
  { Config v; cfgDefaults(v); v.maxManual = 111; v.ringSeen = 5000; cfgSeal(v); EEPROM.put(0, v); }   // valid CRC but an invalid field -> repaired, others kept
  CHECK(cfgLoad(l, rep, sizeof(rep)) == CL_REPAIRED && l.maxManual == 111 && l.ringSeen == d.ringSeen && strstr(rep, "ringseen"));
  // never persist invalid settings
  Config bad; cfgDefaults(bad); bad.ringSeen = 5000;
  CHECK(!cfgSave(bad));
  // pins are not part of the configuration: no parameter name can touch them
  for (size_t i = 0; i < PARAM_COUNT; i++) { ParamDef p; paramAt(i, &p); CHECK(strstr(p.name, "gpio") == nullptr && strncmp(p.name, "pin", 3) != 0); }
  // and no ParamDef offset can reach outside the Config struct (and every name fits the 16-byte field)
  for (size_t i = 0; i < PARAM_COUNT; i++) { ParamDef p; paramAt(i, &p); CHECK(p.off + 1u <= offsetof(Config, crc) && strlen(p.name) < sizeof(p.name)); }
  CHECK(sizeof(Config) < 460);                                   // the EEPROM image and the RAM copy stay small
}

// A "reboot" (power cycle): the EEPROM image survives, everything else restarts from setup(). Settings, calibrations and mounting angles come back;
// autonomy and the motors are off again.
static void t_config_survives_reboot() {
  bootCal();
  simRun(300);
  send(":set maxauto 133\n"); send(":rev BL 1\n"); send(":mok FL\n"); send(":ring bearing 1 25\n"); send(":ring on 4\n"); send(":mod level 1 high\n"); send(":set ringnear 300\n");
  send(":save\n");
  simRun(300);
  startAuto();
  CHECK(g_autoEnabled);
  EEPROM.reset();                                           // power cycle: RAM is gone, the EEPROM file stays
  g_autoEnabled = false; g_navState = NAV_IDLE;             // (the host process keeps its statics: emulate the RAM reset of the autonomy flag)
  ballGone(); setup();
  CHECK(!g_autoEnabled && g_navState == NAV_IDLE && allWheelsStopped());
  CHECK(g_cfg.maxAuto == 133 && g_cfg.reverse[2] == 1 && (g_cfg.motorVerified & 1));
  CHECK(g_cfg.ringBearing[1] == 25 && ringEnabled(g_cfg, 4) && g_cfg.ringNear == 300 && g_cfg.ringCalDone == 1 && (g_cfg.modActiveLow & 2) == 0);
  CHECK(outHas("config: loaded OK"));
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
  ballAt(0);
  simRun(2500);
  CHECK(!allWheelsStopped());
  for (int i = 0; i < 8; i++) g_mockRingBase[i] = 1023;    // whole ring at the top of the range: sensing lost
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
  ballAt(0);
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
  Serial.out.clear(); send("f"); simRun(100);
  CHECK(allWheelsStopped() && outHas("LATCHED"));                        // the emergency stop is latched: no movement from any source
  send(":recover\n"); simRun(50);
  // 'a' refused with an uncalibrated ring
  g_cfg.ringCalDone = 0;
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
  CHECK(outHas("input line too long") || outHas("unknown input"));
  CHECK(outCount("unknown input") + outCount("too long") <= 3);           // not a flood
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
  ballAt(0);
  simRunTracking(1500);
  CHECK(!anyWheelEverMoved);
  CHECK(outHas("ball receivers: ") && outHas("MPU6050"));
  CHECK(outHas("HARDWARE HEALTH REPORT"));
  CHECK(outHas("state=IDLE"));
  CHECK(outHas("TEST MODE"));
  CHECK(outHas("commands (115200 baud"));
  CHECK(outHas("signal STRENGTH"));
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
  CHECK(g_pwmDuty[WHEEL_PINS[W_FL].en] > 0 && g_pwmDuty[WHEEL_PINS[W_FL].en] <= g_cfg.testPwm);
  simRun(1300);                                             // pause then reverse
  simRun(800);
  CHECK(wheelDir(W_FL) < 0 && wheelDir(W_FR) == 0);
  simRun(4000);
  CHECK(allWheelsStopped() && g_mtest.state == 0 && g_navState == NAV_IDLE);
  CHECK(outHas("Did it turn FORWARD"));
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
  for (int i = 0; i < 8; i++) g_mockRingBase[i] = 1023;    // ring wiring fault
  simRun(g_cfg.ringRailMs + 600);
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





static void t_cfg_set() {
  bootCal();
  simRun(200);
  send(":set maxauto 150\n"); simRun(50);
  CHECK(g_cfg.maxAuto == 150);
  send(":set maxauto 5\n"); simRun(50);                     // out of range
  CHECK(g_cfg.maxAuto == 150 && outHas("rejected"));
  send(":set minpwm 119\n"); simRun(50);                    // would exceed the speed limits: cross-field
  CHECK(g_cfg.minPwm == 40);
  send(":set ringseen nan\n"); simRun(50);
  CHECK(g_cfg.ringSeen == 20);
  send(":set nonsense 3\n"); simRun(50);
  send(":rev FR 1\n"); simRun(50);
  CHECK(g_cfg.reverse[1] == 1);
  send(":roller O\n"); simRun(50);
  CHECK(g_cfg.rollerO == 1);
  send(":set ringnear 5000\n"); simRun(50);
  CHECK(g_cfg.ringNear == 375);                             // rejected (range 20..1023)
  send(":set ringnear 700\n"); simRun(50);
  CHECK(g_cfg.ringNear == 700);
  send(":save\n"); simRun(100);
  CHECK(outHas("configuration saved"));
  send(":defaults\n"); simRun(50);
  CHECK(g_cfg.maxAuto == 170 && g_cfg.reverse[1] == 0);
  send(":load\n"); simRun(50);
  CHECK(g_cfg.maxAuto == 150 && g_cfg.reverse[1] == 1 && g_cfg.rollerO == 1 && g_cfg.ringNear == 700);
}








static void t_nonblocking() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0);
  simRun(2000);
  g_bootPhase = false;
  // flood diagnostics while a slow UART drains: every loop() call must stay short in (virtual) time
  uint64_t worst = 0;
  const char* floods[] = {"h", "i", "h", "d", "m", "g", "h"};
  for (int rep = 0; rep < 20; rep++) {
    for (const char* f : floods) { send(f); send("\n"); }
    for (int k = 0; k < 200; k++) {
      uint64_t t0 = g_mockUs;
      loop();
      uint64_t dt = g_mockUs - t0;
      if (dt > worst) worst = dt;
      mockAdvance(50);
    }
  }
  CHECK(worst < 700);                                       // never a wait on Serial (one analogRead of about 112 us per iteration)
  CHECK(g_outDropped > 0 || outUsed() > 0);                 // back-pressure shows up as drops/queueing, not blocking
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_ring[0].scans > 20);
}




static void t_health_report_text() {
  bootCal();
  simRun(500);
  Serial.out.clear();
  send("h"); simRun(1500);
  CHECK(outHas("NOT MEASURABLE (no voltage monitor installed)"));
  CHECK(outHas("NOT MEASURABLE (no encoders, no current sensing)"));
  CHECK(outHas("NOT TESTED"));
  CHECK(outHas("[Ring] receivers on A0..A7") && outHas("[Module 0] D34") && outHas("[Module 1] D35") && !outHas("CD74"));
  CHECK(outHas("physical response to the real ball"));
  CHECK(outHas("NOT TESTED (not calibrated)") == false);    // bootCal calibrated them
  CHECK(outHas("MPU6050 @0x68 who=0x68: PASS") && outHas("Arduino Mega 2560"));
  CHECK(outHas("PWM shown is COMMANDED"));
  CHECK(!outHas("PASS (operator-confirmed") );             // nothing is claimed verified
}

static void t_imu_frozen_in_approach() {
  bootCal();
  simRun(500);
  startAuto();
  ballAt(0);
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
// ================================================================================================
// ball ring (CD74HC4067 + phototransistors), IR modules, ring-based navigation

static void t_ring_pure_rules() {
  bootCal();
  CHECK_NEAR(ringThreshold(g_cfg, 0), 20.0, 1e-3);                    // no noise floor: the fixed minimum
  g_cfg.ringNoise[0] = 100;
  CHECK_NEAR(ringThreshold(g_cfg, 0), 300.0, 1e-3);                   // noise 100 x 3.0
  CHECK(!ringHyst(false, 19, 20) && ringHyst(false, 20, 20));
  CHECK(ringHyst(true, 13, 20) && !ringHyst(true, 11, 20));           // off below 0.6 of the on level
  // bearing from the seen receivers (test layout: A0 = 0 deg, 45 deg apart, clockwise)
  float amp[16] = {0}; bool seen[16] = {false};
  float b; uint8_t best;
  CHECK(!ringBearingFrom(amp, seen, g_cfg.ringBearing, 16, 65, &b, &best));
  amp[0] = 100; seen[0] = true;
  CHECK(ringBearingFrom(amp, seen, g_cfg.ringBearing, 16, 65, &b, &best) && best == 0 && absf(b) < 0.01f);
  amp[1] = 100; seen[1] = true;                                        // A1 = 45 deg, same strength: halfway
  CHECK(ringBearingFrom(amp, seen, g_cfg.ringBearing, 16, 65, &b, &best) && absf(b - 22.5f) < 0.5f);
  amp[1] = 50;
  CHECK(ringBearingFrom(amp, seen, g_cfg.ringBearing, 16, 65, &b, &best) && best == 0 && b > 8.0f && b < 18.0f);   // pulled towards the weaker one
  amp[4] = 90; seen[4] = true;                                         // far side (180 deg) is outside the blend: ignored
  CHECK(ringBearingFrom(amp, seen, g_cfg.ringBearing, 16, 65, &b, &best) && b > 8.0f && b < 18.0f);
  memset(amp, 0, sizeof(amp)); memset(seen, 0, sizeof(seen));
  amp[7] = 100; seen[7] = true; amp[0] = 100; seen[0] = true;          // A7 = -45 deg and A0: wraps correctly to -22.5
  CHECK(ringBearingFrom(amp, seen, g_cfg.ringBearing, 16, 65, &b, &best) && absf(b + 22.5f) < 0.5f);
  // coverage
  uint8_t n;
  CHECK_NEAR(ringCoverageGap(g_cfg, &n), 0.0, 1e-3);
  CHECK(n == 8);
  g_cfg.ringEnable = 0x00FF & ~((1u << 2) | (1u << 3));               // 90 and 135 deg off: A1 (45) and A4 (180) remain around the hole
  CHECK_NEAR(ringCoverageGap(g_cfg, &n), 45.0, 1e-3);
  CHECK(ringInBlindSpot(g_cfg, 112.0f) && !ringInBlindSpot(g_cfg, 0.0f));
  g_cfg.ringEnable = 0;
  CHECK_NEAR(ringCoverageGap(g_cfg, &n), 360.0, 1e-3);
}

static void t_ring_pins_and_isolation() {
  ballGone(); setup();                                                   // default configuration: A0..A5 (six receivers)
  simRun(300);
  for (int ch = 0; ch < 6; ch++) CHECK(g_ring[ch].scans > 0);
  for (int ch = 6; ch < 8; ch++) CHECK(g_ring[ch].scans == 0);          // A6, A7 are not scanned until switched on
  for (int k = 0; k < 8; k++) CHECK(g_pinMode[PIN_RECV[k]] == INPUT);   // analog inputs, no pull-up
  CHECK(g_pinMode[PIN_MOD[0]] == INPUT_PULLUP && g_pinMode[PIN_MOD[1]] == INPUT);   // active-LOW front module: internal pull-up; the disabled second module is left alone
  CHECK(allWheelsStopped());
  layout8(); calibrateAll();
  simRun(300);
  for (int ch = 0; ch < 8; ch++) CHECK(g_ring[ch].scans > 0);           // every enabled receiver is visited
  // only A3 carries a signal: only A3 may see it, and the bearing is A3's
  memset(g_mockRingAmp, 0, sizeof(g_mockRingAmp));
  g_mockRingAmp[3] = 250;
  simRun(300);
  CHECK(g_ball.seenMask == (1u << 3) && g_ball.bestCh == 3 && g_ball.bearingKnown && absf(g_ball.bearing - 135.0f) < 0.5f);
  // each receiver has its own input: a signal on A5 must not appear on A4 or A6
  memset(g_mockRingAmp, 0, sizeof(g_mockRingAmp));
  g_mockRingAmp[5] = 200;
  simRun(300);
  CHECK(g_ball.seenMask == (1u << 5));
}

// Start-up order on the Mega: the motor pins are made safe FIRST; afterwards only assigned pins are ever touched (the motor pins, the module pins and the
// receiver analog inputs); D0/D1 (serial), D20/D21 (I2C, owned by Wire) and every other pin are left alone.
static void t_ring_boot_sequence() {
  memset(g_pinFirstTouchMs, 0, sizeof(g_pinFirstTouchMs));
  g_mockUs = 0;                                                          // reset: the clock starts at zero
  ballGone(); setup();
  uint32_t motorFirst = 0xFFFFFFFFu;
  for (int i = 0; i < 4; i++) {
    CHECK(g_pinFirstTouchMs[WHEEL_PINS[i].in1] != 0 && g_pinFirstTouchMs[WHEEL_PINS[i].en] != 0);
    motorFirst = std::min(motorFirst, std::min(g_pinFirstTouchMs[WHEEL_PINS[i].in1], g_pinFirstTouchMs[WHEEL_PINS[i].en]));
  }
  for (int k = 0; k < 8; k++) CHECK(g_pinFirstTouchMs[PIN_RECV[k]] >= motorFirst);     // after the motor pins were made safe
  for (int p = 0; p < 70; p++) {
    bool assigned = p == PIN_MOD[0] || p == PIN_MOD[1] || p == P_UNDER || p == PIN_I2C_SDA || p == PIN_I2C_SCL;   // the bus-recovery clocking touches SDA / SCL before Wire.begin()
    for (int i = 0; i < 4; i++) assigned |= p == WHEEL_PINS[i].in1 || p == WHEEL_PINS[i].in2 || p == WHEEL_PINS[i].en;
    for (int k = 0; k < 8; k++) assigned |= p == PIN_RECV[k];
    if (!assigned) CHECK(g_pinFirstTouchMs[p] == 0);                     // no unassigned pin is touched (D30..D33 in particular: no multiplexer)
  }
  CHECK(g_pinFirstTouchMs[0] == 0 && g_pinFirstTouchMs[1] == 0 && g_pinFirstTouchMs[30] == 0 && g_pinFirstTouchMs[33] == 0);
  CHECK(allWheelsStopped());
  CHECK(Wire.begun && Wire.clock == 400000 && Wire.timeoutUs > 0);       // hardware I2C (Wire) started at 400 kHz with a bounded timeout
}

static void t_ring_detect_and_bearing() {
  bootCal();
  simRun(200);
  const float degs[] = {0, 30, -30, 90, -90, 180, 45, -135, 10};
  for (float d : degs) {
    ballAt(d, 0.5f);
    simRun(250);
    CHECK(g_ball.valid && g_ball.bearingKnown);
    CHECK(absf(angDiffDeg(g_ball.bearing, d)) < 8.0f);
    if (absf(angDiffDeg(g_ball.bearing, d)) >= 8.0f) printf("  bearing for %g deg -> %g\n", d, g_ball.bearing);
  }
  ballAt(0, 0.5f); simRun(250);
  CHECK_NEAR(g_ball.strength, 0.5, 0.02);
  ballAt(0, 0.25f); simRun(250);
  CHECK_NEAR(g_ball.strength, 0.25, 0.02);                              // strength follows the swing
  // steady light (DC) on one channel is not a ball
  ballGone(); g_mockRingBase[3] = 900; simRun(400);
  CHECK(!g_ball.valid && g_ball.seenMask == 0 && !g_ring[3].railFault);
  // lost: valid for lostHoldMs, then gone; the last bearing is remembered
  g_mockRingBase[3] = 300;
  ballAt(70, 0.5f); simRun(250);
  ballGone();
  simRun(g_cfg.lostHoldMs / 2);
  CHECK(g_ball.valid && !g_ball.bearingKnown);
  simRun(g_cfg.lostHoldMs + 100);
  CHECK(!g_ball.valid && absf(g_lastBearing - 70.0f) < 8.0f);
}

// The scan window must span the gap between the ball's bursts (833 us period, 192 us burst): count how many scans of C0 see the swing.
static void burstRun(uint16_t winUs, int* hits, int* total) {
  g_cfg.ringWinUs = winUs;
  *hits = 0; *total = 0;
  uint32_t last = g_ring[0].scans;
  for (int k = 0; k < 40000 && *total < 120; k++) {
    loop(); mockAdvance((uint32_t)(20 + (k * 37) % 61));                  // irregular loop timing: a different phase against the burst period every time
    if (g_ring[0].scans != last) { last = g_ring[0].scans; (*total)++; if (g_ring[0].amp >= 110) (*hits)++; }
  }
}
static void t_ring_burst_window() {
  bootCal();
  g_cfg.ringEnable = 1;                                                  // only C0
  g_mockRingAmp[0] = 125; g_mockRingBase[0] = 75;
  int hitsLong, totalLong, hitsShort, totalShort;
  burstRun(900, &hitsLong, &totalLong);
  CHECK(totalLong >= 100 && hitsLong == totalLong);                      // a window of at least one burst period never misses a burst
  burstRun(250, &hitsShort, &totalShort);
  CHECK(totalShort >= 100 && hitsShort > 0 && hitsShort < totalShort);   // a window shorter than the gap between bursts does
  g_cfg.ringWinUs = 900;
  ringResetStats();                                                      // (setup() itself blocks for the gyro calibration)
  simRun(400);
  CHECK(g_ringScanMaxUs < 1400);                                         // one channel takes about the window plus a conversion
}

static void t_ring_noise_and_threshold() {
  bootCal();
  g_mockRingNoise = 3;                                                   // +-3 counts of jitter on every conversion
  simRun(1200);
  CHECK(g_ball.seenMask == 0 && !g_ball.valid);                          // jitter alone is no ball
  ballAt(0, 10.0f / 375.0f); simRun(600);                                // a 10-count swing: below the 20-count minimum
  CHECK(g_ball.seenMask == 0);
  ballAt(0, 50.0f / 375.0f); simRun(600);                                // 50 counts: clearly a ball
  CHECK((g_ball.seenMask & 1u) && g_ball.valid);
  ballAt(0, 15.0f / 375.0f); simRun(600);                                // weaker, but above 0.6 x threshold: hysteresis keeps it
  CHECK(g_ball.seenMask & 1u);
  ballAt(0, 3.0f / 375.0f); simRun(600);
  CHECK(g_ball.seenMask == 0);
  // noise calibration: measured with the ball off, stored per channel, and it raises the threshold of a noisy channel
  ballGone();
  Serial.out.clear();
  send(":ring cal noise\n"); simRun(100);
  CHECK(g_ringCalActive && outHas("BALL MUST BE OFF"));
  simRun(2300);
  CHECK(!g_ringCalActive && g_cfg.ringCalDone == 1 && outHas("noise floor stored"));
  for (int ch = 0; ch < 8; ch++) CHECK(g_cfg.ringNoise[ch] >= 1 && g_cfg.ringNoise[ch] <= 7);
  g_cfg.ringNoise[0] = 30;
  ballAt(0, 50.0f / 375.0f); simRun(600);                                // C0 swing 50 < 30 x 3.0: C0 ignores it (its neighbours may see part of it)
  CHECK(!(g_ball.seenMask & 1u));
}

static void t_ring_saturation_and_faults() {
  bootCal();
  g_cfg.ringFovHalf = 60;                                                // wider assumed view so that the neighbours of A2 also see a ball at 90 deg
  ballAt(90, 0.6f);
  simRun(300);
  CHECK(g_ball.bearingKnown && absf(g_ball.bearing - 90.0f) < 8.0f);
  g_mockRingBase[2] = 1023;                                              // A2 (90 deg) stuck at the top of the ADC range
  simRun(g_cfg.ringRailMs + 600);
  CHECK(g_ring[2].railFault && faultActive(F_RING_CHANNEL) && g_cond == COND_DEGRADED);
  CHECK(!(g_ball.seenMask & (1u << 2)));                                 // the excluded channel is not used,
  CHECK(g_ball.bearingKnown && absf(g_ball.bearing - 90.0f) < 25.0f);    // the neighbours still place the ball
  g_mockRingBase[2] = 300;
  simRun(g_cfg.recoverVerifyMs + 1500);
  CHECK(!g_ring[2].railFault && !faultActive(F_RING_CHANNEL) && g_cond == COND_NORMAL);
  // a channel that is switched off is "not tested", never "failed"
  g_cfg.ringEnable &= ~(1u << 2); g_mockRingBase[2] = 1023; simRun(g_cfg.ringRailMs + 600);
  CHECK(!faultActive(F_RING_CHANNEL));
  g_cfg.ringEnable |= (1u << 2); g_mockRingBase[2] = 300; simRun(1500);
  // every channel saturated: no usable ring -> critical
  for (int i = 0; i < 8; i++) g_mockRingBase[i] = 1023;
  simRun(g_cfg.ringRailMs + 800);
  CHECK(faultActive(F_RING_NONE) && g_cond == COND_SAFE_STOP);
  // a stalled scan is critical too
  for (int i = 0; i < 8; i++) g_mockRingBase[i] = 300;
  simRun(g_cfg.recoverVerifyMs + 1500);
  CHECK(!faultActive(F_RING_NONE) && g_cond == COND_NORMAL);
  ringHealth(g_cfg, g_ringLastScanMs + 900);
  CHECK(faultActive(F_RING_STALL) && g_cond == COND_SAFE_STOP);
}

static void t_ring_coverage_gap() {
  bootCal();
  g_cfg.ringMaxGap = 90;                                                 // the default (360) never raises RING_GAP: only two receivers are fitted
  simRun(300);
  CHECK(!faultActive(F_RING_GAP));
  send(":ring off 2\n"); send(":ring off 3\n"); simRun(300);            // 90 and 135 deg off: A1 (45) .. A4 (180)
  uint8_t n;
  CHECK_NEAR(ringCoverageGap(g_cfg, &n), 45.0, 1e-3);
  CHECK(!faultActive(F_RING_GAP));                                       // slight blind spot: tolerated (up to ringmaxgap = 90)
  send(":ring off 4\n"); send(":ring off 5\n"); simRun(300);            // A1 (45) .. A6 (270)
  CHECK_NEAR(ringCoverageGap(g_cfg, &n), 135.0, 1e-3);
  CHECK(faultActive(F_RING_GAP) && g_cond == COND_DEGRADED);
  send(":ring on 4\n"); send(":ring on 5\n"); send(":ring on 2\n"); send(":ring on 3\n"); simRun(g_cfg.recoverVerifyMs + 1000);
  CHECK(!faultActive(F_RING_GAP));
  // the default configuration (two receivers, ringmaxgap 360) never raises it
  CHECK(g_cond != COND_SAFE_STOP);
}

static void t_ring_cli() {
  bootCal();
  simRun(300);
  Serial.out.clear();
  send(":ring\n"); simRun(600);
  CHECK(outHas("ball receivers: 8 of 8") && outHas(" A0 ") && outHas(" A7 ") && !outHas("CD74") && !outHas("multiplex"));
  CHECK(outHas("coverage: largest arc"));
  send(":ring off 7\n"); simRun(120);
  CHECK(!ringEnabled(g_cfg, 7));
  Serial.out.clear(); send(":ring\n"); simRun(600);
  CHECK(!outHas(" A7 "));                                                // a switched-off optional receiver is hidden
  send(":ring on 7\n"); simRun(120);
  CHECK(ringEnabled(g_cfg, 7) && g_ring[7].scans > 0);
  send(":ring bearing 3 45\n"); simRun(20);
  CHECK(g_cfg.ringBearing[3] == 45);
  send(":ring bearing 3 200\n"); simRun(20);
  CHECK(g_cfg.ringBearing[3] == 45);                                     // out of range: unchanged
  send(":ring bearing 8 0\n"); send(":ring on 8\n"); send(":ring foo\n"); simRun(600);
  CHECK(outCount("usage: :ring") >= 3);
  send(":set ringseen 2\n"); simRun(20);
  CHECK(g_cfg.ringSeen == 20);                                           // below the allowed range
  send(":set ringseen 30\n"); simRun(20);
  CHECK(g_cfg.ringSeen == 30);
  g_cfg.ringSeen = 20;
  // near calibration needs a ball in view
  Serial.out.clear(); send(":ring cal near\n"); simRun(30);
  CHECK(outHas("no channel sees the ball") && g_cfg.ringNear == 375);
  ballAt(0, 0.6f); simRun(300);
  send(":ring cal near\n"); simRun(30);
  CHECK(g_cfg.ringNear >= 220 && g_cfg.ringNear <= 230);
  send(":ring reset\n"); simRun(30);
  CHECK(g_ring[0].scans < 30);
  // the saved configuration carries the ring settings
  send(":ring bearing 7 -170\n"); simRun(20);
  send(":save\n"); simRun(200);
  Config chk; char rep[40];
  CfgLoadResult r = cfgLoad(chk, rep, sizeof(rep));
  CHECK(r == CL_OK && chk.ringBearing[7] == -170 && chk.ringNear == g_cfg.ringNear);
}

static void t_modules() {
  bootCal();
  g_cfg.underEnable = 0;                                                 // the modules alone here; the underside sensor has its own test
  simRun(100);
  CHECK(!g_mod[0].active && !g_ball.held);                               // a clear module (output HIGH) is not "detecting"
  bool ever = false;
  g_mockModDetect[0] = true;
  for (int k = 0; k < 3; k++) { simRun(1); ever |= g_mod[0].active; }
  g_mockModDetect[0] = false;
  for (int k = 0; k < 60; k++) { simRun(1); ever |= g_mod[0].active; }
  CHECK(!ever);                                                          // a 3 ms blip is debounced away
  g_mockModDetect[0] = true; simRun(80);
  CHECK(g_mod[0].active && g_ball.held && !g_ball.ready);                // module alone: a wall or a robot also trips it -> not ready without the ring
  ballAt(0, 0.5f); simRun(300);
  CHECK(g_ball.held && g_ball.ready && g_ball.readySinceMs != 0);
  ballAt(60, 0.5f); simRun(300);
  CHECK(g_ball.held && !g_ball.ready);                                   // module detects but the ring says the ball is 60 deg off
  ballGone(); g_mockModDetect[0] = true; g_cfg.modNeedRing = 0; simRun(120);
  CHECK(g_ball.ready);                                                   // operator chose "module only"
  g_cfg.modNeedRing = 1; g_mockModDetect[0] = false; simRun(80);
  CHECK(!g_ball.held && !g_ball.ready);
  // module 1 is read and logged, never used for the shooting position
  g_cfg.modEnable = 3; g_mockModDetect[1] = true; simRun(80);
  CHECK(g_mod[1].active && !g_ball.held);
  g_cfg.modEnable = 1; simRun(40);
  CHECK(!g_mod[1].active);                                               // switched off: never active
  // polarity is a setting, not an assumption
  g_cfg.modActiveLow = 2; g_mockModActiveHigh[0] = true; g_mockModDetect[0] = true; simRun(80);       // module 0 active-high, module 1 still active-low
  CHECK(g_mod[0].active);
  g_mockModDetect[0] = false; simRun(80);
  CHECK(!g_mod[0].active);
  Serial.out.clear(); send(":mod\n"); send(":mod level 0 low\n"); send(":mod level 1 high\n"); send(":mod off 0\n"); send(":mod on 0\n"); send(":mod foo\n"); simRun(60);
  CHECK(outHas("IR module 0 D34") && outHas("IR module 1 D35") && g_cfg.modActiveLow == 1 && modEnabled(g_cfg, 0) && outHas("usage: :mod"));
  g_mockModActiveHigh[0] = false;
}

static void t_ring_autonomy_refused() {
  ballGone(); setup();                                                   // defaults: noise floor not calibrated
  simRun(200);
  Serial.out.clear(); startAuto();
  CHECK(!g_autoEnabled && outHas("noise floor not calibrated"));
  calibrateAll();
  g_cfg.modEnable = 0; Serial.out.clear(); startAuto();
  CHECK(!g_autoEnabled && outHas("IR module 0"));
  g_cfg.modEnable = 1;
  g_cfg.ringEnable = 0; Serial.out.clear(); startAuto();
  CHECK(!g_autoEnabled && outHas("no usable ball-ring channel"));
  g_cfg.ringEnable = 0x00FF; simRun(g_cfg.recoverVerifyMs + 1500);      // the "no usable channel" fault clears once the ring is back
  Serial.out.clear(); startAuto();
  CHECK(g_autoEnabled && g_navState == NAV_SEEK);
}

static void t_nav_approach_and_stop() {
  bootCal();
  simRun(300);
  startAuto();
  CHECK(g_autoEnabled && g_navState == NAV_SEEK);
  ballFar(); simRun(300);
  CHECK(g_navState == NAV_APPROACH);
  CHECK(g_reqVy > 0.2f && absf(g_reqVx) < 0.05f && absf(g_reqW) < 0.05f);   // ahead, far: straight forward
  float vyFar = g_reqVy;
  ballAt(0, 0.7f); simRun(300);
  CHECK(g_reqVy < vyFar);                                                  // slows as the signal strengthens
  ballHeld(); simRun(400);
  CHECK(g_ball.ready && g_reqVy == 0.0f && g_navState == NAV_APPROACH);   // shooting position: no forward motion
  ballFar(); simRun(300);
  CHECK(g_reqVy > 0.1f);                                                   // ball moved away: approaches again
}

static void t_nav_turn_toward_ball() {
  bootCal();
  simRun(300);
  startAuto();
  ballAt(60, 0.4f); simRun(300);                                           // well to the right: turn clockwise first
  CHECK(g_navState == NAV_APPROACH && g_reqW >= 0.12f && g_reqVy <= 0.5f * g_cfg.slowSpeed + 1e-3f);
  ballAt(-100, 0.4f); simRun(300);                                         // to the left and slightly behind: turn anticlockwise, no forward motion
  CHECK(g_reqW <= -0.12f && g_reqVy == 0.0f);
  ballAt(180, 0.4f); simRun(300);                                          // straight behind: still turns (the ring sees the back)
  CHECK(absf(g_reqW) >= 0.12f && g_reqVy == 0.0f);
  ballAt(12, 0.4f); simRun(300);                                           // slightly right: drive and steer gently
  CHECK(g_reqVy > 0.2f && g_reqW > 0.0f && g_reqW < 0.2f);
  ballAt(0, 0.4f); simRun(300);
  CHECK(g_reqVy > 0.2f && absf(g_reqW) < 0.05f);
  ballAt(20, 0.8f); simRun(300);                                           // close and 20 deg off: keeps its heading and strafes to the right
  CHECK(g_reqVx > 0.02f && absf(g_reqW) < 0.05f);
}

static void t_nav_lost_search_reacquire() {
  bootCal();
  simRun(300);
  startAuto();
  ballAt(60, 0.5f); simRun(400);                                           // seen on the right
  CHECK(g_navState == NAV_APPROACH);
  ballGone();
  simRun(g_cfg.lostHoldMs + 400);
  CHECK(g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE);
  CHECK(g_stats.ballLossEvents == 1);
  simRun(800);
  CHECK(g_navState == NAV_SEARCH && g_reqW > 0.1f);                        // last seen on the right: turn clockwise first
  ballFar(); simRun(400);
  CHECK(g_navState == NAV_APPROACH && g_stats.reacquisitions == 1);
  // nothing ever found: searches, pauses, waits, never spins forever
  ballGone();
  simRun(g_cfg.lostHoldMs + 200);
  g_cfg.searchTimeoutMs = 2000; g_cfg.searchAttempts = 2; g_cfg.searchPauseMs = 500;
  simRun(15000);
  CHECK(g_navState == NAV_WAIT && allWheelsStopped());
}

// A ball that disappears is looked for where it can have gone: the way it was moving, else the nearest uncovered arc, else its last side.
static void t_nav_blind_spot_search() {
  bootCal();
  // Receivers at front (A0), right (A2 = 90 deg) and back (A4): the only uncovered arc is on the LEFT. A ball last seen by the front receiver alone
  // (which cannot say which side it is on) can only have slipped into that arc: search anticlockwise.
  g_cfg.ringEnable = (1u << 0) | (1u << 2) | (1u << 4);
  simRun(300);
  startAuto();
  CHECK(g_autoEnabled);
  ballAt(0, 0.6f); simRun(300);
  CHECK(g_ball.bearingKnown && g_ball.seenMask == 1u);
  CHECK(ringInBlindSpot(g_cfg, -90.0f) && !ringInBlindSpot(g_cfg, 90.0f));
  ballGone(); simRun(g_cfg.lostHoldMs + 500);
  CHECK((g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE) && g_searchDir == -1);
  simRun(1500);
  CHECK(g_navState == NAV_SEARCH && g_reqW < -0.1f);                       // and it really rotates that way
  // mirror image: the uncovered arc is on the RIGHT
  g_cfg.ringEnable = (1u << 0) | (1u << 6) | (1u << 4);
  ballGone(); simRun(2000);
  startAuto();
  ballAt(0, 0.6f); simRun(300);
  ballGone(); simRun(g_cfg.lostHoldMs + 500);
  CHECK((g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE) && g_searchDir == 1);
  // full ring: a ball that was moving quickly to the LEFT while its last bearing is still slightly right left through the left
  g_cfg.ringEnable = 0x00FF;
  ballGone(); simRun(2000);
  startAuto(); ballAt(40, 0.5f); simRun(300);
  for (int k = 0; k <= 20; k++) { ballAt(40.0f - 1.75f * (float)k, 0.5f); simRun(20); }   // 40 -> 5 deg in 400 ms (about -88 deg/s)
  CHECK(g_bearingRate < -30.0f && g_ball.bearing > 0.0f);
  ballGone(); simRun(g_cfg.lostHoldMs + 500);
  CHECK((g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE) && g_searchDir == -1);
}

static void t_nav_ring_fault_safe() {
  bootCal();
  simRun(300);
  startAuto();
  ballFar(); simRun(500);
  CHECK(!allWheelsStopped());
  for (int i = 0; i < 8; i++) g_mockRingBase[i] = 1023;                   // the whole ring reads the top of the range (SIG shorted to 3.3 V)
  simRun(g_cfg.ringRailMs + 800);
  CHECK(faultActive(F_RING_NONE) && !g_autoEnabled && allWheelsStopped() && g_cond == COND_SAFE_STOP);
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
  ballAt(35, 1.0f); g_mockModDetect[0] = true; simRun(100);   // (let the sensor states settle first)
  send(":goal set\n"); simRun(1500);
  CHECK(g_stats.shots == 0 && g_navState != NAV_SHOOT);     // module detects but the ring puts the ball 35 deg off: the robot aligns, it does not shoot
  ballGone(); g_mockModDetect[0] = true; simRun(1500);
  CHECK(g_stats.shots == 0 && g_navState != NAV_SHOOT);     // the module alone (a wall?) is not the shooting position
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
  g_cfg.reMax = 0;                                          // (no reverse-and-reattack here: this test is about one shot per ball)
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
  const char* cmds[] = {":ch 3 en 1\n", ":learn 90\n", ":irgate auto\n", ":ir cal clear 0\n", ":aux on\n", ":avoid\n"};
  for (const char* c : cmds) { send(c); simRun(30); }
  CHECK(outCount("unknown word command") == 6);            // none of the earlier-era commands exists any more
  Serial.out.clear(); send("?"); simRun(50); send("h"); simRun(300); send("i"); simRun(50); send("d"); simRun(50);
  CHECK(!outHas("TSOP") && !outHas("emitter"));            // no emitter / digital-receiver hardware is described
  CHECK(outHas("ball receivers") || outHas("[Ring]"));
  // pins: 8 direction + 4 PWM + 2 modules + 2 I2C + 2 serial + 2 Bluetooth + 1 underside sensor, all distinct (the compile-time asserts in pins.h); the receivers on A0..A7
  CHECK(__builtin_popcountll(ALL_DIGITAL_MASK) == 21 && __builtin_popcountll(RECV_MASK) == 8 && (MOTOR_PWM_MASK & ~MEGA_PWM_MASK) == 0 && __builtin_popcountll(MEGA_PWM_MASK) == 15);
  CHECK(outHas("ball receivers: ") && !outHas("GPIO") && !outHas("CD74") && !outHas("multiplex"));                    // no ESP32 pin names are left in the output
}

// ================================================================================================
// Bluetooth remote (HC-05 on Serial1), emergency-stop latch, mode switching

static void bt(const char* s) { Serial1.feed(s); if (strlen(s) == 1) Serial1.feed("\n"); }    // a single command is one line
static bool btHas(const char* s) { return Serial1.out.find(s) != std::string::npos; }
static void forwardNow() { CHECK(wheelDir(W_FL) > 0 && wheelDir(W_FR) > 0 && wheelDir(W_BL) > 0 && wheelDir(W_BR) > 0); }

static void t_bt_protocol() {
  bootCal();
  simRun(300);
  CHECK(Serial1.begun && g_cfg.btEnable == 1);
  CHECK(!g_autoEnabled && g_navState == NAV_IDLE && allWheelsStopped());       // boots idle: nothing moves by itself
  bt("W"); simRun(60);                                                          // movement before choosing manual mode is refused
  CHECK(btHas("ERR:NOT_MANUAL") && allWheelsStopped());
  Serial1.out.clear(); bt("M"); simRun(60);
  CHECK(btHas("OK:M") && g_remoteManual && !g_autoEnabled && allWheelsStopped());
  // the four-wheel directions of every drive command (X-layout mecanum)
  Serial1.out.clear(); bt("W"); simRun(120);
  CHECK(btHas("OK:W")); forwardNow();
  bt("S"); simRun(400); CHECK(wheelDir(W_FL) < 0 && wheelDir(W_FR) < 0 && wheelDir(W_BL) < 0 && wheelDir(W_BR) < 0);
  bt("A"); simRun(400); CHECK(wheelDir(W_FL) < 0 && wheelDir(W_BL) < 0 && wheelDir(W_FR) > 0 && wheelDir(W_BR) > 0);      // turn left = rotate counter-clockwise
  bt("D"); simRun(400); CHECK(wheelDir(W_FL) > 0 && wheelDir(W_BL) > 0 && wheelDir(W_FR) < 0 && wheelDir(W_BR) < 0);      // turn right
  bt("Z"); simRun(400); CHECK(wheelDir(W_FL) < 0 && wheelDir(W_FR) > 0 && wheelDir(W_BL) > 0 && wheelDir(W_BR) < 0);      // strafe left
  bt("C"); simRun(400); CHECK(wheelsMatchStrafeRight());
  bt("w"); simRun(400); forwardNow();                                           // lower case is accepted
  // a movement command is a lease: it ends by itself
  simRun(g_cfg.btLeaseMs + 400);
  CHECK(allWheelsStopped() && g_remoteManual && g_navState == NAV_IDLE);
  // a held key repeats the command: the robot keeps driving
  for (int k = 0; k < 15; k++) { bt("W"); simRun(100); }
  forwardNow();
  bt("Q"); simRun(30);                                                          // normal stop: stops at once, stays in manual mode, does not touch the latch
  CHECK(allWheelsStopped() && g_remoteManual && !g_estop);
  // unknown bytes never move anything
  Serial1.out.clear(); bt("k#$1\r\n  "); simRun(100);
  CHECK(allWheelsStopped() && g_btUnknown >= 1 && outCountOf(Serial1.out, "ERR:UNKNOWN") <= 1);
}

static void t_bt_modes_and_estop() {
  bootCal();
  simRun(300);
  // autonomous mode: 'U' runs the same checks as 'a'
  bt("U"); simRun(80);
  CHECK(g_autoEnabled && g_navState == NAV_SEEK);
  ballFar(); simRun(400);
  CHECK(g_navState == NAV_APPROACH && g_reqVy > 0.1f);
  Serial1.out.clear(); bt("W"); simRun(100);                                    // manual drive is refused while autonomous: the autonomous controller keeps the motors
  CHECK(btHas("ERR:NOT_MANUAL") && g_autoEnabled && g_navState == NAV_APPROACH && g_reqVy > 0.1f);
  bt("M"); simRun(60);                                                          // explicit switch to manual: autonomy ends, nothing keeps driving
  CHECK(!g_autoEnabled && g_remoteManual && allWheelsStopped());
  bt("W"); simRun(100); forwardNow();
  bt("U"); simRun(80);                                                          // back to autonomous: the manual move is cleared
  CHECK(g_autoEnabled && g_navState != NAV_MANUAL);
  // Q in autonomous mode ends autonomy and keeps manual mode
  bt("Q"); simRun(50);
  CHECK(!g_autoEnabled && allWheelsStopped() && g_remoteManual);
  // emergency stop: immediate, latched, outranks everything
  bt("M"); simRun(40);
  for (int k = 0; k < 6; k++) { bt("W"); simRun(50); }
  CHECK(!allWheelsStopped());
  Serial1.out.clear(); bt("X");
  loop();                                                                       // ONE pass of the main loop is enough
  CHECK(allWheelsStopped() && g_estop && g_navState == NAV_IDLE && !g_remoteManual);
  simRun(50);
  CHECK(btHas("OK:X"));
  Serial1.out.clear(); bt("W"); bt("M"); bt("U"); simRun(100);
  CHECK(outCountOf(Serial1.out, "ERR:ESTOP") == 3 && allWheelsStopped() && !g_autoEnabled);
  send("a"); simRun(50); send("f"); simRun(100);                                // the USB console cannot start anything either
  CHECK(!g_autoEnabled && allWheelsStopped());
  bt("Q"); simRun(30);
  CHECK(g_estop);                                                               // Q is a normal stop: it does not clear the latch
  Serial1.out.clear(); bt("R"); simRun(60);
  CHECK(btHas("OK:R") && !g_estop && !g_remoteManual && !g_autoEnabled && allWheelsStopped());   // cleared, and the robot stays idle
  bt("W"); simRun(60);
  CHECK(allWheelsStopped());                                                    // still needs an explicit M or U
  bt("M"); bt("W"); simRun(120); forwardNow();
  // emergency stop during autonomous attack/search states, and with a backlog of stale commands
  bt("U"); simRun(80); ballFar(); simRun(300);
  bt("WWWWWWWWWWWWWWWWWWWWX"); simRun(5);
  CHECK(allWheelsStopped() && g_estop);                                         // the X anywhere in the waiting bytes wins, the rest is dropped
  bt("R"); simRun(40);
  // a fault that is not the latch: R refuses while the drivetrain fault is active
  bt("X"); simRun(20); faultRaise(F_MOTOR_INIT, "test fault"); simRun(20);
  Serial1.out.clear(); bt("R"); simRun(40);
  CHECK(btHas("ERR:FAULT") && g_estop);
  // CLI side of the latch
  send(":recover\n"); simRun(50);
  CHECK(!g_estop);
  send("!"); simRun(20);
  CHECK(g_estop);
  send(":recover\n"); simRun(50);
  CHECK(!g_estop);
  // status / ping
  Serial1.out.clear(); bt("P"); bt("?"); simRun(60);
  CHECK(btHas("PONG:") && btHas("ST:"));
}

// Controller disconnected, stale bytes, and the switch off.
static void t_bt_link_loss() {
  bootCal();
  g_cfg.btLeaseMs = 3000; g_cfg.btLinkMs = 500;                                 // a long lease: only the link supervision can stop the robot in time
  simRun(300);
  bt("M"); simRun(40);
  bt("W"); simRun(200);
  forwardNow();
  simRun(200);                                                                  // still within the link timeout
  CHECK(!allWheelsStopped() && g_btLink);
  simRun(800);                                                                  // the remote went silent
  CHECK(allWheelsStopped() && !g_btLink && g_navState == NAV_IDLE);
  CHECK(outHas("Bluetooth link LOST"));
  bt("W"); simRun(100);                                                         // the link comes back and the remote can drive again
  CHECK(g_btLink); forwardNow();
  g_cfg.btLeaseMs = 400;
  // many stale movement bytes then silence: the robot stops within the lease, it does not keep driving
  bt("Q"); simRun(30);
  for (int k = 0; k < 60; k++) bt("W");
  simRun(200);
  CHECK(!allWheelsStopped());
  simRun(g_cfg.btLeaseMs + 500);
  CHECK(allWheelsStopped());
  // switched off in the configuration: bytes are ignored
  g_cfg.btEnable = 0; bt("X"); simRun(50);
  CHECK(!g_estop);
}

// ================================================================================================
// underside ball-position sensor (D40)

static void t_under_sensor() {
  bootCal();
  simRun(200);
  CHECK(g_cfg.underEnable == 1 && g_pinMode[P_UNDER] == INPUT_PULLUP);          // active-LOW default: pull-up on, so a missing sensor reads "no ball"
  CHECK(!g_under.active && g_ball.positioned == false);
  // debounce
  bool ever = false;
  g_mockUnderDetect = true; for (int k = 0; k < 3; k++) { simRun(1); ever |= g_under.active; }
  g_mockUnderDetect = false; for (int k = 0; k < 60; k++) { simRun(1); ever |= g_under.active; }
  CHECK(!ever);
  // it is a confirmation, not a replacement: module + receivers alone are not enough while the underside sensor says "no ball"
  ballAt(0, 0.5f); g_mockModDetect[0] = true; simRun(500);
  CHECK(g_ball.held && !g_ball.ready && underUsable(g_cfg));
  g_mockUnderDetect = true; simRun(100);
  CHECK(g_ball.ready && g_ball.positioned);                                     // all three agree
  // the receivers alone with only the underside sensor: not ready
  ballAt(0, 0.5f); g_mockModDetect[0] = false; simRun(200);
  CHECK(!g_ball.ready);
  // no answer: everything else says "ball in place" but the sensor never reports it (disconnected, wrong polarity): it is ignored after understuck
  g_mockUnderDetect = false; g_mockModDetect[0] = true; simRun(200);
  CHECK(!g_ball.ready && !g_under.suspectMiss);
  simRun(g_cfg.underStuckMs + 600);
  CHECK(g_under.suspectMiss && !underUsable(g_cfg) && g_ball.ready);
  g_mockUnderDetect = true; simRun(100);                                        // it answers: trusted again
  CHECK(!g_under.suspectMiss && underUsable(g_cfg));
  // stuck active while the receivers see no ball at all: ignored (and does not fake a position)
  ballGone(); g_mockUnderDetect = true; simRun(g_cfg.underStuckMs + 600);
  CHECK(g_under.suspectStuck && !underUsable(g_cfg) && !g_ball.valid);
  g_mockUnderDetect = false; simRun(100);
  CHECK(!g_under.suspectStuck);
  // wrong polarity guess is survivable: the sensor really is active-HIGH but the setting says LOW
  g_mockUnderActiveHigh = true; g_mockUnderDetect = false; simRun(g_cfg.underStuckMs + 800);
  CHECK(g_under.active && g_under.suspectStuck && !g_ball.valid);               // read as "ball" forever with no receiver ball: ignored
  send(":set underlow 0\n"); simRun(200);
  CHECK(!g_under.active && g_pinMode[P_UNDER] == INPUT);
  g_mockUnderDetect = true; simRun(100);
  CHECK(g_under.active);
  // switched off: it does not vote
  send(":set underenable 0\n"); simRun(100);
  CHECK(!g_under.active && g_ball.positioned);
  Serial.out.clear(); send(":raw\n"); simRun(800);
  CHECK(outHas("raw underside sensor (D40)"));
}

// ================================================================================================
// reverse and re-attack, search layers, no-goal attack, wheel scale

static void t_reattack() {
  shotSetup();
  g_cfg.shootCooldownMs = 500; g_cfg.shootHoldMs = 100;
  ballFar();
  startAuto(); simRun(1200);
  ballHeld();                                                                   // the ball sits in front of the robot all the time: the push never works
  uint64_t end = g_mockUs + 60000000ULL;
  bool reversed = false; uint8_t maxCount = 0; uint64_t waitSince = 0;
  while (g_mockUs < end) {
    emulateAimRotation(); loop(); mockAdvance(50);
    if (g_navState == NAV_REVERSE && g_rePhase == 1 && g_reqVy < -0.3f) reversed = true;
    if (g_reCount > maxCount) maxCount = g_reCount;
    if (g_navState == NAV_WAIT) { waitSince = g_mockUs; break; }
  }
  CHECK(waitSince != 0 && reversed);                                            // it did reverse, and in the end it stopped trying instead of looping forever
  CHECK(maxCount == g_cfg.reMax);                                               // exactly reMax reverse tries
  CHECK((int)g_stats.shots == g_cfg.reMax + 1);                                 // the first attack plus one per try
  CHECK(allWheelsStopped());
  uint32_t shots = g_stats.shots;
  simRun(g_cfg.reCooldownMs - 400);                                             // while the pause lasts: no further attack
  CHECK(g_stats.shots == shots && g_navState == NAV_WAIT);
  // after the pause it tries again (the count starts over): bounded, not blind
  simRun(2500);
  CHECK(g_navState != NAV_WAIT || g_stats.shots > shots);
}

static void t_reattack_phases() {
  shotSetup();
  g_cfg.reJudgeMs = 250; g_cfg.reBackMs = 350; g_cfg.reSettleMs = 200; g_cfg.reMax = 2;
  ballHeld(); startAuto(); simRun(300);
  // phase machinery on its own: force the state after a "shot"
  auto enter = [&]() { g_rePhase = 0; g_reSince = millis(); navSetState(NAV_REVERSE, PSTR("test")); };
  // 1. the ball left during the judging window: success, count reset
  g_reCount = 1; enter(); ballGone(); simRun(150);
  CHECK(g_navState != NAV_REVERSE && g_reCount == 0);
  // 2. the push failed: judge, reverse (bounded), settle, back to tracking
  ballHeld(); simRun(100); g_reCount = 0; enter();
  simRun(g_cfg.reJudgeMs - 100);
  CHECK(g_navState == NAV_REVERSE && g_rePhase == 0 && allWheelsStopped());
  simRun(150);
  CHECK(g_rePhase == 1 && g_reCount == 1);
  simRun(100);
  CHECK(g_reqVy < -0.3f && wheelDir(W_FL) < 0 && wheelDir(W_FR) < 0);          // reversing with all four wheels
  simRun(g_cfg.reBackMs - 100 + 60);
  CHECK(g_rePhase == 2);
  simRun(40);
  CHECK(g_reqVy == 0.0f);                                  // the reverse is bounded by reBackMs
  simRun(g_cfg.reSettleMs + 100);
  CHECK(g_navState != NAV_REVERSE);
  // 3. the ball is lost while reversing: stop at once and search
  ballHeld(); simRun(100); g_reCount = 0; enter(); simRun(g_cfg.reJudgeMs + 100);
  CHECK(g_rePhase == 1);
  ballGone();
  for (int k = 0; k < 80 && g_navState != NAV_SEARCH && g_navState != NAV_SEARCH_PAUSE; k++) simRun(50);
  CHECK(g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE);
  CHECK(g_reqVy >= -0.01f);                                                     // not still driving backwards
  // 4. the ball moves sideways during the reverse: reposition at once
  ballHeld(); simRun(100); startAuto(); g_reCount = 0; enter(); simRun(g_cfg.reJudgeMs + 100);
  CHECK(g_rePhase == 1);
  ballAt(80, 0.5f); g_mockModDetect[0] = false; g_mockUnderDetect = false; simRun(150);
  CHECK(g_rePhase == 2 || g_navState != NAV_REVERSE);
  // 5. an emergency stop in the middle of the reverse
  ballHeld(); simRun(100); startAuto(); g_reCount = 0; enter(); simRun(g_cfg.reJudgeMs + 100);
  CHECK(g_rePhase == 1 && !allWheelsStopped());
  bt("X"); loop();
  CHECK(allWheelsStopped() && g_estop && g_navState == NAV_IDLE);
  // 6. the tries used up: pause, then a new count
  bt("R"); simRun(30); send("a"); simRun(50);
  ballHeld(); simRun(100); g_reCount = g_cfg.reMax; enter(); simRun(g_cfg.reJudgeMs + 100);
  CHECK(g_navState == NAV_WAIT && g_reCount == 0 && allWheelsStopped());
}

static void t_search_layers() {
  bootCal();
  simRun(300);
  startAuto();
  // false detection: the ball is "seen" but never reached (the front module never reports it)
  g_cfg.approachTimeoutMs = 2000;
  ballFar(); simRun(1500);
  CHECK(g_navState == NAV_APPROACH);
  simRun(1200);
  CHECK(g_stats.ballLossEvents >= 1 && g_appTimeouts >= 1);                    // timed out and searched again
  // three timeouts in a row: stop and pause instead of chasing it forever
  simRun(12000);
  CHECK(g_navState == NAV_WAIT || g_appTimeouts > 0 || g_stats.ballLossEvents >= 3);
  uint32_t losses = g_stats.ballLossEvents;
  uint64_t end = g_mockUs + 20000000ULL; bool waited = false;
  while (g_mockUs < end) { loop(); mockAdvance(50); if (g_navState == NAV_WAIT) { waited = true; simRun(400); CHECK(allWheelsStopped()); break; } }
  CHECK(waited || losses >= 3);
  // reaching the ball resets the false-detection count
  ballGone(); simRun(500);
  startAuto(); ballFar(); simRun(500);
  ballHeld(); simRun(500);
  CHECK(g_appTimeouts == 0);
  // WAIT is not forever: a fresh search starts after researchms; and with no ball at all autonomy gives up by itself
  send("s"); ballGone(); simRun(1000);
  g_cfg.searchTimeoutMs = 2000; g_cfg.searchAttempts = 1; g_cfg.searchPauseMs = 500; g_cfg.researchMs = 1500; g_cfg.giveUpMs = 12000;
  startAuto();
  bool sawWait = false, sawResearch = false;
  for (int k = 0; k < 200; k++) {                                               // 10 s: search, wait, then a fresh search by itself
    simRun(50);
    if (g_navState == NAV_WAIT) sawWait = true;
    else if (sawWait && g_navState == NAV_SEARCH) sawResearch = true;
  }
  CHECK(sawWait && sawResearch);
  simRun(20000);
  CHECK(!g_autoEnabled && allWheelsStopped());                                  // and after giveupms without any ball it stops for good
  // the ball appears during a search sweep: it is picked up at once
  startAuto(); simRun(1500);
  CHECK(g_navState == NAV_SEARCH || g_navState == NAV_SEARCH_PAUSE || g_navState == NAV_SEEK);
  ballFar(); simRun(300);
  CHECK(g_navState == NAV_APPROACH);
}

static void t_shoot_no_goal_and_wheel_scale() {
  // no goal heading registered: the default is no shot (it needs the goal heading) ...
  bootCal();
  g_cfg.shootEnable = 1; g_cfg.shootHoldMs = 100; g_cfg.reMax = 0; g_cfg.shootBackMs = 150;
  simRun(300); ballFar(); startAuto(); simRun(300);
  send(":zero\n"); simRun(100);                                                // the gyro reference was reset: the registered goal heading is no longer valid
  CHECK(!goalValid());
  ballHeld(); simRun(2500);
  CHECK(g_stats.shots == 0 && g_navState != NAV_SHOOT);
  // ... and with shootneedgoal 0 a straight drive burst is allowed
  send("s"); ballGone(); simRun(400);
  g_cfg.shootNeedGoal = 0;
  ballFar(); startAuto(); simRun(300);
  send(":zero\n"); simRun(100);
  CHECK(!goalValid());
  ballHeld();
  uint64_t end = g_mockUs + 8000000ULL;
  while (g_mockUs < end && g_stats.shots == 0) { loop(); mockAdvance(50); }
  CHECK(g_stats.shots == 1);
  // per-wheel open-loop scale: 50 % on the front-left wheel halves its PWM, the others are unchanged, stop is still zero everywhere
  bootCal();
  simRun(200);
  send(":set wscalefl 50\n"); simRun(20);
  CHECK(g_cfg.wheelScale[W_FL] == 50);
  send(":set wscalefr 49\n"); simRun(20);
  CHECK(g_cfg.wheelScale[W_FR] == 100);                                         // out of range (50..150): rejected
  send("m"); simRun(30); bt("M"); simRun(30);
  for (int k = 0; k < 30; k++) { bt("W"); simRun(100); }
  uint32_t fl = g_pwmDuty[WHEEL_PINS[W_FL].en], fr = g_pwmDuty[WHEEL_PINS[W_FR].en], bl = g_pwmDuty[WHEEL_PINS[W_BL].en];
  CHECK(fr > 0 && fr == bl);
  CHECK(fl * 2 >= fr - 2 && fl * 2 <= fr + 2);                                   // 50 % trim on the front-left wheel only
  bt("Q"); simRun(400);
  CHECK(allWheelsStopped());
  send(":set wscalebr 150\n"); simRun(20);
  for (int k = 0; k < 30; k++) { bt("W"); simRun(100); }
  CHECK(g_pwmDuty[WHEEL_PINS[W_BR].en] > g_pwmDuty[WHEEL_PINS[W_BL].en]);
  bt("Q"); simRun(400);
  CHECK(allWheelsStopped());
}

static void t_autostart() {
  // real boot sequence with the default settings: no command needed
  ballGone(); setup(); layout8(); calibrateAll();
  CHECK(g_cfg.autoStart == 1 && g_autoStartPending && !g_autoEnabled && allWheelsStopped());   // motors stay stopped through initialisation
  simRun(g_cfg.autoStartDelayMs / 2);
  CHECK(!g_autoEnabled && allWheelsStopped());
  simRun(g_cfg.autoStartDelayMs);
  CHECK(g_autoEnabled && g_navState == NAV_SEEK);
  // Bluetooth is not needed: no link was ever made
  CHECK(!g_btLink);
  // X still stops everything at once, then the latch holds
  bt("X"); loop();
  CHECK(allWheelsStopped() && g_estop && !g_autoEnabled);
  // manual mode still works after an automatic start
  bt("R"); simRun(30); bt("M"); bt("W"); simRun(100);
  CHECK(g_remoteManual && !allWheelsStopped());
  // an operator action during the delay cancels the automatic start
  ballGone(); setup(); layout8(); calibrateAll();
  simRun(300); bt("M"); simRun(g_cfg.autoStartDelayMs + 500);
  CHECK(!g_autoEnabled && g_remoteManual && !g_autoStartPending);
  ballGone(); setup(); layout8(); calibrateAll();
  simRun(300); bt("X"); simRun(g_cfg.autoStartDelayMs + 500);
  CHECK(!g_autoEnabled && g_estop && allWheelsStopped());
  // a failed safety check keeps the motors off and says why
  ballGone(); setup(); layout8();                                                // ring NOT calibrated
  Serial.out.clear(); simRun(g_cfg.autoStartDelayMs + 500);
  CHECK(!g_autoEnabled && allWheelsStopped() && outHas("autostart"));
  // disabled by setting
  ballGone(); setup(); layout8(); calibrateAll(); g_cfg.autoStart = 0; g_autoStartPending = false;
  simRun(g_cfg.autoStartDelayMs + 500);
  CHECK(!g_autoEnabled);
}

// ================================================================================================
// Defending mode, telemetry, gyro diagnostics, motion diagnostics, parser (2026 intelligence update)

static void lineBt(const char* s) { Serial1.feed(s); Serial1.feed("\n"); }                 // a word / line from the Bluetooth remote
static void word(const char* s) { send(s); send("\n"); }                                    // a word typed in the USB Serial Monitor
static bool modeIs(const char* m) { return strcmp(modeName(), m) == 0; }
static std::vector<std::string> btFrames() {
  std::vector<std::string> v; size_t p = 0; const std::string& o = Serial1.out;
  while (p < o.size()) { size_t e = o.find('\n', p); if (e == std::string::npos) break; if (o[p] == '{') v.push_back(o.substr(p, e - p)); p = e + 1; }
  return v;
}
static uint32_t g_visited = 0;
static void runWatch(uint32_t ms) { uint32_t n = ms / 20; for (uint32_t k = 0; k < n; k++) { simRun(20); g_visited |= 1u << g_defState; } }
static bool visited(DefState s) { return (g_visited >> s) & 1u; }
static void defendStart() { bootCal(); simRun(300); word("defend"); simRun(300); }

static void t_defend_enter_exit() {
  bootCal(); simRun(300);
  g_cfg.bearingsSet = 0;                                                   // mounting angles still the default guess: no directional behaviour
  word("defend"); simRun(100);
  CHECK(!g_defendEnabled && outHas("mounting angles") && allWheelsStopped());
  g_cfg.bearingsSet = 1; g_cfg.ringCalDone = 0;
  Serial.out.clear(); word("defend"); simRun(100);
  CHECK(!g_defendEnabled && outHas("not calibrated"));
  g_cfg.ringCalDone = 1;
  word("defend"); simRun(100);
  CHECK(g_defendEnabled && g_navState == NAV_DEFEND && allWheelsStopped() && modeIs("DEFEND"));
  simRun(800);
  CHECK(outHas("POSITION -> SEARCH") && g_defState == DEF_SEARCH);         // the state change and its reason are reported
  // telemetry says DEFEND
  Serial1.out.clear(); simRun(1500);
  bool sawMode = false; for (auto& f : btFrames()) if (f.find("\"mode\":\"DEFEND\"") != std::string::npos && f.find("\"sub\":\"SEARCH\"") != std::string::npos) sawMode = true;
  CHECK(sawMode);
  // switching between modes cancels the previous one and leaves no motion behind
  word("attack"); simRun(100);
  CHECK(g_autoEnabled && !g_defendEnabled && modeIs("ATTACK"));
  word("defend"); simRun(100);
  CHECK(g_defendEnabled && !g_autoEnabled && allWheelsStopped());
  word("manual"); simRun(100);
  CHECK(!g_defendEnabled && g_remoteManual && modeIs("MANUAL") && allWheelsStopped());
  word("defend"); simRun(100); word("stop"); simRun(100);
  CHECK(!g_defendEnabled && modeIs("IDLE") && allWheelsStopped());
  // the same words through Bluetooth, through the same parser
  Serial1.out.clear(); lineBt("defend"); simRun(100);
  CHECK(btHas("OK:DEFEND") && g_defendEnabled);
  lineBt("attack"); simRun(100);
  CHECK(btHas("OK:ATTACK") && g_autoEnabled && !g_defendEnabled);
  lineBt("manual"); simRun(100);
  CHECK(btHas("OK:MANUAL") && g_remoteManual && !g_autoEnabled);
  lineBt("stop"); simRun(100);
  CHECK(btHas("OK:STOP"));
  // refused: drivetrain fault, emergency stop
  faultRaise(F_MOTOR_INIT, "test fault"); simRun(60);
  Serial1.out.clear(); lineBt("defend"); simRun(100);
  CHECK(!g_defendEnabled && btHas("ERR:REFUSED"));
  g_fault[F_MOTOR_INIT].active = false; condEvaluate();
  bt("X"); simRun(20);
  Serial1.out.clear(); lineBt("defend"); simRun(60);
  CHECK(!g_defendEnabled && btHas("ERR:ESTOP"));
  word("defend"); simRun(60);
  CHECK(!g_defendEnabled && g_estop);
}

static void t_defend_behaviour() {
  defendStart(); simRun(800);
  CHECK(g_defState == DEF_SEARCH && allWheelsStopped());
  g_visited = 0;
  ballAt(30, 0.3f); runWatch(800);                                         // a distant, steady ball: confirm, track, but do not run after it
  CHECK(g_defState == DEF_TRACK && visited(DEF_TRACK) && !visited(DEF_INTERCEPT));
  CHECK(absf(g_reqVx) < 0.01f && absf(g_reqW) < 0.05f);
  ballAt(40, 0.55f); runWatch(1000);                                       // closer and to the right: intercept = strafe right, no spinning
  CHECK(visited(DEF_INTERCEPT));
  CHECK(g_reqVx > 0.1f && absf(g_reqW) < 0.05f && absf(g_reqVy) < 0.01f);
  CHECK(wheelsMatchStrafeRight() || g_defState != DEF_INTERCEPT);
  ballAt(0, 0.9f); runWatch(1200);                                         // in line with the robot and strong: block (stand still)
  CHECK(visited(DEF_BLOCK) && g_defState == DEF_BLOCK);
  simRun(400); CHECK(absf(g_reqVx) < 0.05f && allWheelsStopped());
  ballGone(); runWatch(800);                                               // gone after the block attempt
  CHECK(visited(DEF_LOST));
  runWatch(g_cfg.defLostMs + 3000);
  CHECK(g_defState == DEF_SEARCH || g_defState == DEF_POSITION);           // back to the defensive posture
  CHECK(allWheelsStopped());
  // ball on the left: strafe left
  ballAt(-45, 0.6f); runWatch(1200);
  CHECK(g_reqVx < -0.1f || g_defState == DEF_TRACK);
  // ball behind the robot: cannot be blocked from here, no strafing
  ballAt(180, 0.6f); ballAt(180, 0.6f); runWatch(1500);
  CHECK(g_defState == DEF_RECOVER || g_defState == DEF_POSITION || g_defState == DEF_SEARCH || g_defState == DEF_TRACK);
  CHECK(absf(g_reqVx) < 0.01f);
}

static void t_defend_prediction() {
  defendStart(); simRun(600);
  // approaching: the strength RISES over time-stamped observations
  g_visited = 0;
  for (int k = 0; k < 60; k++) { ballAt(35, 0.15f + 0.5f * (float)k / 60.0f); runWatch(20); }
  CHECK(visited(DEF_PREDICT) && visited(DEF_INTERCEPT));
  CHECK(g_defPredOk && g_defApproaching);
  CHECK(g_defStrSlope > g_cfg.defSlope);
  // steady far ball: a trend IS available (flat), it is NOT approaching, and there is no prediction state
  ballGone(); runWatch(3500);
  word("stop"); simRun(50); word("defend"); simRun(1200);
  g_visited = 0; ballAt(35, 0.3f); runWatch(3000);
  CHECK(!visited(DEF_PREDICT) && !visited(DEF_INTERCEPT) && !g_defApproaching);
  // receding
  ballGone(); word("stop"); simRun(50); word("defend"); simRun(1200);
  g_visited = 0;
  for (int k = 0; k < 60; k++) { ballAt(35, 0.7f - 0.5f * (float)k / 60.0f); runWatch(20); }
  CHECK(!visited(DEF_PREDICT));
  // too few observations: no trend is claimed at all
  ballGone(); word("stop"); simRun(50); word("defend"); simRun(1200);
  ballAt(30, 0.5f); runWatch(240);
  CHECK(!g_defPredOk && g_defLeadDeg == 0.0f && g_defStrSlope == 0.0f);
}

static void t_defend_hysteresis_and_limits() {
  defendStart(); simRun(600);
  g_cfg.defConfOn = 50; g_cfg.defConfOff = 35; g_cfg.defSlope = 2.0f;      // (no 'approaching' verdict in this test)
  ballAt(0, 0.6f); runWatch(1000);                                         // one receiver sees it: confidence about 56
  CHECK(g_defState == DEF_TRACK);
  uint32_t tr = g_defTransitions;
  for (int k = 0; k < 100; k++) { ballAt(0, (k & 1) ? 0.4f : 0.3f); runWatch(20); }       // confidence flips 44 <-> 38: between the two thresholds
  CHECK(g_defTransitions == tr && g_defState == DEF_TRACK);                // no flicker
  ballAt(0, 0.1f); runWatch(1500);                                         // really weak: below the off threshold
  CHECK(visited(DEF_LOST) || g_defState == DEF_LOST || g_defState == DEF_POSITION || g_defState == DEF_SEARCH);
  // interception limits: a ball held at 70 deg to the right never makes the robot wander off
  word("stop"); simRun(50); g_cfg.defConfOn = 40; g_cfg.defConfOff = 25; word("defend"); simRun(800);
  float maxLat = 0; uint32_t strafeRun = 0, longestRun = 0; bool paused = false; float maxW = 0;
  ballAt(70, 0.6f);
  for (int k = 0; k < 400; k++) {
    simRun(50);
    if (absf(g_defLat) > maxLat) maxLat = absf(g_defLat);
    if (absf(g_reqVx) > 0.05f) { strafeRun += 50; if (strafeRun > longestRun) longestRun = strafeRun; } else { if (strafeRun) paused = true; strafeRun = 0; }
    if (absf(g_reqW) > maxW) maxW = absf(g_reqW);
  }
  CHECK(maxLat <= (float)g_cfg.defLatMs + 120.0f);                         // the open-loop sideways budget holds
  CHECK(longestRun <= (uint32_t)g_cfg.defStrafeMs + 400u);                 // no unbounded strafe
  CHECK(paused);                                                           // it did pause between strafes
  CHECK(maxW < 0.1f);                                                      // no unnecessary spinning
  CHECK(maxLat > 200.0f);                                                  // it did move
}

static void t_defend_fault_stop() {
  defendStart(); simRun(600);
  ballAt(70, 0.6f); simRun(700);
  CHECK(!allWheelsStopped() || g_defState != DEF_INTERCEPT);
  for (int i = 0; i < 8; i++) g_mockRingBase[i] = 1023;                    // receiver wiring fault: every input pinned high
  simRun(g_cfg.ringRailMs + 800);
  CHECK(g_defState == DEF_FAULT && allWheelsStopped() && modeIs("FAULT"));
  CHECK(outHas("sensing or drivetrain fault"));
  Serial.out.clear(); word("defend"); simRun(100);
  CHECK(outHas("REFUSED"));
  word("stop"); simRun(60);
  CHECK(!g_defendEnabled && allWheelsStopped());
}

static void t_estop_every_mode() {
  for (int mode = 0; mode < 5; mode++) {
    g_estop = false; g_remoteManual = false; g_defendEnabled = false; g_autoEnabled = false;      // (a reboot between the modes)
    bootCal(); simRun(300); ballGone();
    if (mode == 0) { bt("M"); for (int k = 0; k < 4; k++) { bt("W"); simRun(60); } }
    else if (mode == 1) { ballFar(); word("attack"); simRun(800); }
    else if (mode == 2) { word("defend"); simRun(600); ballAt(70, 0.6f); simRun(800); }
    else if (mode == 3) { word("calibrate"); simRun(100); CHECK(modeIs("CALIBRATION")); }
    else { simRun(50); }
    bt("X"); loop();                                                       // ONE pass of the main loop
    CHECK(allWheelsStopped() && g_estop && !g_autoEnabled && !g_defendEnabled && g_navState == NAV_IDLE);
    CHECK(modeIs("FAULT"));
    simRun(60);
    Serial.out.clear(); word("defend"); word("attack"); word("manual"); lineBt("U"); simRun(100);
    CHECK(allWheelsStopped() && !g_defendEnabled && !g_autoEnabled);       // nothing starts while the latch is set
    send("!"); simRun(20); CHECK(allWheelsStopped());                      // USB emergency stop too
  }
}

static void t_cmd_parser_limits() {
  bootCal(); simRun(300);
  word("DeFeNd"); simRun(100);
  CHECK(g_defendEnabled);                                                  // case-insensitive
  word("stop"); simRun(100);
  anyWheelEverMoved = false;
  Serial.out.clear();
  word("defendx"); word("launch"); word("ff"); word("fbl"); simRunTracking(200);   // unknown words and multi-key bursts: nothing is executed
  CHECK(!g_defendEnabled && !g_autoEnabled && !anyWheelEverMoved && outHas("NOTHING was executed"));
  std::string big(100, 'a'); Serial.out.clear();
  word(big.c_str()); simRun(100);
  CHECK(outHas("too long") && !g_autoEnabled);                             // oversized input is rejected, not executed as 'a'
  word("def\x01nd"); simRun(50);                                           // control byte inside a word: dropped
  Serial.out.clear();
  send("gyro"); simRun(30);                                                // no line ending: 8 ms of silence completes it
  CHECK(outHas("MPU6050") || outHas("GYRO"));
  Serial.out.clear(); send("  status  \r\n"); simRun(100);
  CHECK(outHas("MODE "));
  // Bluetooth side
  Serial1.out.clear();
  lineBt("123456789012345678901234567890"); simRun(100);                   // 30 characters
  CHECK(btHas("ERR:TOO_LONG"));
  lineBt("fly"); simRun(100);
  CHECK(btHas("ERR:UNKNOWN") && allWheelsStopped() && !g_autoEnabled);
  Serial1.feed("stop"); simRun(60);                                        // unterminated word is completed by silence
  CHECK(btHas("OK:STOP"));
  lineBt("  DEFEND \r"); simRun(100);
  CHECK(g_defendEnabled);
  lineBt("stop"); simRun(60);
  Serial1.feed("dexfend\n"); simRun(60);                                   // an X inside the text is an emergency stop (fail-safe direction)
  CHECK(g_estop);
  // a Bluetooth word cannot name a pin, a register or a setting: only the fixed word table is reachable
  bt("R"); simRun(50);
  Serial1.out.clear(); lineBt("set maxauto 255"); lineBt("digitalwrite 22 1"); simRun(100);
  CHECK(g_cfg.maxAuto == 170 && allWheelsStopped());
}

static void t_telemetry_frames() {
  bootCal(); g_cfg.modEnable = 3; g_cfg.teleDetailEvery = 2; simRun(300);
  ballAt(30, 0.5f); g_mockModDetect[1] = true; simRun(500);
  Serial1.out.clear(); simRun(6000);
  auto fr = btFrames();
  TeleSnapshot sn; teleInit(&sn);
  bool k[4] = {false, false, false, false}; size_t maxLen = 0; int bad = 0;
  for (auto& f : fr) {
    maxLen = std::max(maxLen, f.size() + 1);
    char kind = teleParseLine(&sn, f.c_str(), 1000);
    if (!kind) bad++; else k[kind == 'c' ? 0 : (kind == 'r' ? 1 : (kind == 'i' ? 2 : 3))] = true;
    CHECK(f.find("current") == std::string::npos && f.find("dist") == std::string::npos);      // nothing the hardware cannot measure
  }
  CHECK(bad == 0 && k[0] && k[1] && k[2] && k[3]);
  CHECK(sn.lost == 0 && sn.bad == 0);                                      // sequence numbers contiguous
  CHECK(maxLen <= 383);
  CHECK(fr.size() >= 8 && fr.size() <= 30);                                // limited by the 9600 baud link, well under the UART capacity
  // content
  CHECK(strcmp(sn.mode, "IDLE") == 0 && strcmp(sn.safety, "OK") == 0 && sn.ball);
  CHECK(sn.bearingValid && absf(sn.bearing - 30.0f) < 8.0f && sn.conf > 20);
  CHECK(sn.obst[1] == 1.0f && sn.obst[0] == 0.0f);
  CHECK(sn.swing[0] >= 0 && sn.thresh[0] > 0 && sn.rawLevel[0] > 0);
  CHECK(strcmp(sn.bearingMap, "operator") == 0);
  CHECK(sn.imuOk && !isnan(sn.acc[2]) && absf(sn.acc[2] - 1.0f) < 0.05f);
  // a missing ball is null, not zero
  ballGone(); simRun(900); Serial1.out.clear(); simRun(2500);
  TeleSnapshot s2; teleInit(&s2);
  for (auto& f : btFrames()) teleParseLine(&s2, f.c_str(), 1000);
  CHECK(!s2.ball && !s2.bearingValid && isnan(s2.bearing));
  // faster link: more frames
  g_cfg.btBaud100 = 1152; g_cfg.teleHz = 20; Serial1.out.clear(); simRun(2000);
  CHECK(btFrames().size() >= 25);
  // the HC-05 does not drain (no link): the control loop never waits and the queue does not overrun
  Serial1.blocked = true; Serial1.out.clear();
  uint64_t worst = 0; send("f"); send("\n");
  for (int i = 0; i < 400; i++) { uint64_t t0 = g_mockUs; loop(); uint64_t dt = g_mockUs - t0; if (dt > worst) worst = dt; mockAdvance(50); }
  CHECK(worst < 700);
  CHECK(btTxUsed() < BT_TXQ_SIZE && g_navState == NAV_MANUAL);
  uint32_t sent = g_tele.sent; simRun(2000);
  CHECK(g_tele.sent <= sent + 1);                                          // no frames pile up behind a stuck UART
  send("!"); simRun(30);
  CHECK(allWheelsStopped());                                               // emergency stop is independent of the Bluetooth link
}

static void t_telemetry_imu_large_values() {
  bootCal();
  g_imu.present = g_imu.ok = g_imu.calibrated = g_imu.tiltValid = g_imu.yawValid = true;
  g_imu.calState = 2;
  g_imu.ax = -3.999f; g_imu.ay = 3.999f; g_imu.az = -3.999f;
  g_imu.gx = -499.99f; g_imu.gy = 499.99f; g_imu.gz = -499.99f;
  g_imu.roll = -177.3f; g_imu.pitch = -179.9f; g_imu.yaw = 179.9f;
  g_imu.bias = -12.345f; g_imu.driftState = 2;
  g_imu.driftRateDegMin = -999.99f; g_imu.driftMeanDps = -12.345f;
  g_tele.seq = 65535;
  teleBuildImu(g_cfg, 4294967295u);
  CHECK(g_teleN > 300 && g_teleN < TELE_BUF - 1);
  TeleSnapshot sn; teleInit(&sn);
  CHECK(teleParseLine(&sn, g_teleBuf, 1000) == 'i');
  CHECK(sn.imuOk && sn.bad == 0);
}

static void t_gyro_diagnostics() {
  // (a) MPU6050 missing at startup
  g_mpu.present = false; bootCal(); g_cfg.teleDetailEvery = 1; simRun(600);
  CHECK(!g_imu.present && !g_imu.calibrated);
  Serial.out.clear(); word("gyro"); simRun(300); CHECK(outHas("MISSING"));
  word("gyro drift"); simRun(100); CHECK(outHas("refused"));
  Serial1.out.clear(); simRun(3000);
  { TeleSnapshot s; teleInit(&s); for (auto& f : btFrames()) teleParseLine(&s, f.c_str(), 1000); CHECK(!s.imuOk && isnan(s.acc[0]) && isnan(s.yaw) && isnan(s.roll)); }
  g_mpu.present = true;
  // (b) bias measured at startup while still
  g_mpu.gz = 1.2f; bootCal(); simRun(500);
  CHECK(g_imu.calibrated && absf(g_imu.bias - 1.2f) < 0.05f && absf(g_imu.calMean - 1.2f) < 0.05f && g_imu.calAtMs != 0);
  // (c) rejected while moving, and (d) a bias beyond the limit is rejected
  g_mpu.gz = 0; bootCal(); simRun(300);
  word("gyro cal"); for (int i = 0; i < 300; i++) { g_mpu.gz = (i & 1) ? 40.0f : -40.0f; simRun(10); }
  CHECK(g_imu.calState == 3);
  g_mpu.gz = 15.0f; word("gyro cal"); simRun(3500);
  CHECK(g_imu.calState == 3 && absf(g_imu.bias - 15.0f) > 1.0f);
  // (e) drift test: the real bias has crept 0.05 dps away from the calibrated value
  g_mpu.gz = 1.2f; bootCal(); simRun(500);
  g_mpu.gz = 1.25f;
  word("gyro drift 10"); simRun(200);
  CHECK(g_imu.driftState == 1 && modeIs("CALIBRATION"));
  simRun(10500);
  CHECK(g_imu.driftState == 2);
  CHECK(absf(g_imu.driftMeanDps - 1.25f) < 0.02f);                         // measured gyro bias
  CHECK(absf(g_imu.driftRateDegMin) > 2.0f && absf(g_imu.driftRateDegMin) < 4.0f);   // heading drift: 0.05 dps = 3 deg/min
  Serial.out.clear(); word("gyro"); simRun(300);
  CHECK(outHas("measured BIAS ERROR") && outHas("measured HEADING DRIFT") && outHas("RELATIVE"));
  // (f) motion during the test fails it; (g) shaking fails it
  word("gyro drift 10"); simRun(300); send("f"); simRun(300);
  CHECK(g_imu.driftState == 3 && g_imu.driftWhy == 1);
  send("s"); simRun(1500);
  word("gyro drift 10"); simRun(300); g_mpu.azg = 1.4f; simRun(100); g_mpu.azg = 1.0f; simRun(10500);
  CHECK(g_imu.driftState == 3 && g_imu.driftWhy == 4);
  // not allowed while a motor command is active
  send("f"); simRun(100); Serial.out.clear(); word("gyro drift"); simRun(100);
  CHECK(g_imu.driftState != 1 && outHas("refused"));
  send("s"); simRun(300);
  // (h) tilt from the accelerometer, (i) uncertainty estimate grows with time
  g_mpu.ayg = 0.5f; g_mpu.azg = 0.866f; simRun(3000);
  CHECK(absf(g_imu.roll - 30.0f) < 2.0f && g_imu.tiltValid);
  g_mpu.ayg = 0; g_mpu.axg = -0.5f; simRun(3000);
  CHECK(absf(g_imu.pitch - 30.0f) < 2.0f);
  float u1 = imuYawUncertDeg(millis()); simRun(5000); float u2 = imuYawUncertDeg(millis());
  CHECK(u2 > u1 && u1 >= 0);
}

static void t_motion_diagnostics() {
  bootCal(); simRun(300);
  send("f"); simRun(300);
  Serial.out.clear(); word("motors"); simRun(300);
  CHECK(outHas("FL: command FORWARD") && outHas("no encoder or current sensor"));
  send("s"); simRun(300);
  Serial.out.clear(); word("motors"); simRun(300);
  CHECK(outHas("FL: command STOPPED") && !outHas("FORWARD"));
  // continuous-motion watchdog
  g_cfg.maxMoveSec = 3; bt("M"); simRun(50);
  bool tripped = false;
  for (int k = 0; k < 70 && !tripped; k++) { bt("W"); simRun(100); tripped = g_motion.wdTrips > 0; }
  CHECK(tripped && allWheelsStopped() && g_motion.wdStopped && strcmp(safetyText(), "MOTION_WATCHDOG") == 0);
  CHECK(outHas("motion watchdog"));
  // a turn without translation is measured separately
  g_cfg.maxMoveSec = 0; send("q"); simRun(350);
  CHECK(g_motion.turnSinceMs != 0 && g_reqW != 0.0f);
  send("s"); simRun(300);
  // a motor command left behind by a mode that should not move anything is cleared and counted
  bt("Q"); simRun(100);
  // a mode transition that forgets to stop: attack ends without navStop, the approach keeps commanding motion
  ballFar(); word("attack"); simRun(1500);
  CHECK(!allWheelsStopped());
  Serial.out.clear();
  g_autoEnabled = false; simRun(200);
  CHECK(allWheelsStopped() && g_motion.staleEvents >= 1 && outHas("stale motor command"));
  // reset/boot: every motor pin low
  setup();
  for (int i = 0; i < 4; i++) CHECK(g_pinLevel[WHEEL_PINS[i].in1] == 0 && g_pinLevel[WHEEL_PINS[i].in2] == 0 && g_pwmDuty[WHEEL_PINS[i].en] == 0);
}

static void t_attack_side_ball() {
  bootCal(); simRun(300); startAuto();
  ballAt(90, 0.4f); simRun(600);
  CHECK(g_navState == NAV_APPROACH && absf(g_reqVy) < 0.01f && absf(g_reqW) > 0.05f);       // turns first, does not charge forward at a ball on its side
  ballAt(10, 0.4f); simRun(600);
  CHECK(g_reqVy > 0.1f);
  // sensor failure returns to a safe state
  for (int i = 0; i < 8; i++) g_mockRingBase[i] = 1023;
  simRun(g_cfg.ringRailMs + 800);
  CHECK(allWheelsStopped() && !g_autoEnabled);
}

static void t_ring_polarity_and_calibration() {
  // the swing measurement ignores the DC level: low or high resting level, same detection (no polarity assumption)
  for (int dc = 0; dc < 2; dc++) {
    bootCal(); simRun(300);
    for (int i = 0; i < 8; i++) { g_mockRingBase[i] = dc ? 900 : 40; g_mockRingAmp[i] = 0; }
    g_mockRingAmp[0] = 200; simRun(500);
    CHECK(g_ball.valid && g_ball.bestCh == 0);
  }
  // calibration with the ball absent stores the floor; a weak signal then stays below the threshold
  bootCal(); ballGone(); g_mockRingNoise = 6; g_cfg.ringCalDone = 0; simRun(300);
  Serial.out.clear(); word("calibrate"); simRun(400);
  CHECK(modeIs("CALIBRATION"));
  simRun(2500);
  CHECK(g_cfg.ringCalDone == 1 && !modeIs("CALIBRATION"));
  float thr = ringThreshold(g_cfg, 0);
  CHECK(thr >= (float)g_cfg.ringSeen);
  g_mockRingAmp[0] = (uint16_t)(0.5f * thr); simRun(400);
  CHECK(!g_ball.valid);                                                    // below threshold: no ball
  g_mockRingAmp[0] = (uint16_t)(3.0f * thr); simRun(400);
  CHECK(g_ball.valid);
  // refused while a ball signal is present, and while moving
  Serial.out.clear(); word("calibrate"); simRun(100);
  CHECK(outHas("ball signal is present") && !g_ringCalActive);
  g_mockRingAmp[0] = 0; simRun(1500); send("f"); simRun(100); Serial.out.clear(); word("calibrate"); simRun(100);
  CHECK(outHas("stop first") && !g_ringCalActive);
}

static void t_direction_confidence() {
  bootCal(); simRun(300);
  ballAt(60, 0.5f); simRun(500);
  CHECK(g_ball.valid && !g_ball.ambiguous && absf(g_ball.bearing - 60.0f) < 12.0f && g_ball.confidence >= 50);
  uint8_t good = g_ball.confidence;
  ballAt(60, 0.02f); simRun(500);                                          // too weak: below the detection threshold
  CHECK(!g_ball.bearingKnown);
  ballAt(60, 0.12f); simRun(500);                                          // barely above: low confidence
  if (g_ball.bearingKnown) CHECK(g_ball.confidence < good);
  // conflicting pattern: two strong receivers looking in opposite directions and nothing in between
  ballGone();
  for (int i = 0; i < 8; i++) g_mockRingAmp[i] = 0;
  g_mockRingAmp[0] = 200; g_mockRingAmp[4] = 190; simRun(600);
  CHECK(g_ball.ambiguous && g_ball.confidence <= 30);
  Serial1.out.clear(); simRun(3000);
  TeleSnapshot s; teleInit(&s); for (auto& f : btFrames()) teleParseLine(&s, f.c_str(), 1000);
  CHECK(s.ambiguous && !s.bearingValid);                                   // no confident direction is reported
  word("defend"); simRun(1500);
  CHECK(g_defState != DEF_TRACK && g_defState != DEF_INTERCEPT);           // and defending does not act on it
}


// ================================================================================================
// Bluetooth authentication (Mega: btauth.h). HOST SIMULATION ONLY: a test-only C++ remote (test_remote.h) talks to the real firmware code; there is no radio and no HC-05.
// The independent check is the Python client suite (laptop/tests/).

static const char* TKEY = "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f";     // synthetic TEST key (not a credential of the robot)
static const char* TKEY_WRONG = "ffeeddccbbaa99887766554433221100ffeeddccbbaa99887766554433221100";
static size_t g_btCursor = 0;
static std::vector<std::string> btNewLines() {
  std::vector<std::string> v;
  const std::string& o = Serial1.out;
  if (g_btCursor > o.size()) g_btCursor = 0;
  size_t p = g_btCursor;
  while (p < o.size()) { size_t e = o.find('\n', p); if (e == std::string::npos) break; v.push_back(o.substr(p, e - p)); p = e + 1; }
  g_btCursor = p;
  return v;
}
static std::string lastBt(const char* prefix) { std::string r; for (auto& l : btNewLines()) if (l.rfind(prefix, 0) == 0) r = l; return r; }
static void rndTest(uint8_t* o, size_t n) { static uint8_t c = 1; for (size_t i = 0; i < n; i++) { o[i] = c; c = (uint8_t)(c * 5u + 3u); } }
static ClientAuth g_cl;
static void authBoot(bool provision = true) {
  g_mockAuthEnforced = true;
  ballGone(); setup(); g_autoStartPending = false; layout8(); calibrateAll();
  Serial1.out.clear(); g_btCursor = 0;
  if (provision) { send(":authkey "); send(TKEY); send("\n"); simRun(60); }
  clientInit(&g_cl, TKEY);
}
// Runs the client side against the real Mega code for `ms` of simulated time.
static void pumpClient(bool want, uint32_t ms) {
  static uint32_t lastPingMs = 0;
  for (uint32_t t = 0; t < ms; t += 20) {
    if (clientAuthed(&g_cl) && millis() - lastPingMs >= 500) { lastPingMs = millis(); bt("P"); }      // the sketch pings every 500 ms while authenticated
    char buf[100]; size_t n = clientWantLine(&g_cl, want, millis(), rndTest, buf, sizeof(buf));
    if (n) { buf[n] = 0; lineBt(buf); }
    simRun(20);
    for (auto& l : btNewLines()) clientOnLine(&g_cl, l.c_str(), millis());
  }
}
static void waitPing(uint32_t ms) { for (uint32_t k = 0; k < ms; k += 500) { bt("P"); simRun(500); } }   // silence longer than btlink would end the link (and cancel the handshake): keep it alive
static std::string respLine(const char* keyHex, const std::string& chLine, uint8_t cnSeed, bool corrupt = false) {
  uint8_t key[32], sn[16], cn[16], tag[16];
  hexToBytes(keyHex, key, 32); hexToBytes(chLine.c_str() + 8, sn, 16);
  for (int i = 0; i < 16; i++) cn[i] = (uint8_t)(cnSeed + i);
  authTag(key, "E2M", sn, cn, tag);
  if (corrupt) tag[3] ^= 1;
  char a[33], b[33]; bytesToHex(cn, 16, a); bytesToHex(tag, 16, b);
  return std::string("AUTH RESP ") + a + " " + b;
}
static bool noSecretOnAnyWire() { return Serial.out.find(TKEY) == std::string::npos && Serial1.out.find(TKEY) == std::string::npos; }

static void t_sha256_hmac_vectors() {
  uint8_t o[32]; char h[65];
  sha256((const uint8_t*)"", 0, o); bytesToHex(o, 32, h); CHECK(strcmp(h, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855") == 0);
  sha256((const uint8_t*)"abc", 3, o); bytesToHex(o, 32, h); CHECK(strcmp(h, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0);
  const char* m2 = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  sha256((const uint8_t*)m2, strlen(m2), o); bytesToHex(o, 32, h); CHECK(strcmp(h, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1") == 0);
  { Sha256 s; sha256_init(&s); uint8_t blk[1000]; memset(blk, 'a', sizeof(blk)); for (int i = 0; i < 1000; i++) sha256_update(&s, blk, sizeof(blk)); sha256_final(&s, o);
    bytesToHex(o, 32, h); CHECK(strcmp(h, "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0") == 0); }       // one million 'a'
  uint8_t k20[20]; memset(k20, 0x0b, 20);
  hmac_sha256(k20, 20, (const uint8_t*)"Hi There", 8, nullptr, 0, nullptr, 0, o); bytesToHex(o, 32, h); CHECK(strcmp(h, "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7") == 0);   // RFC 4231 case 1
  hmac_sha256((const uint8_t*)"Jefe", 4, (const uint8_t*)"what do ya want ", 16, (const uint8_t*)"for nothing?", 12, nullptr, 0, o); bytesToHex(o, 32, h);
  CHECK(strcmp(h, "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843") == 0);                                                                                            // case 2 (split message)
  uint8_t kaa[131], d50[50]; memset(kaa, 0xaa, sizeof(kaa)); memset(d50, 0xdd, sizeof(d50));
  hmac_sha256(kaa, 20, d50, 50, nullptr, 0, nullptr, 0, o); bytesToHex(o, 32, h); CHECK(strcmp(h, "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe") == 0);                    // case 3
  const char* big = "Test Using Larger Than Block-Size Key - Hash Key First";
  hmac_sha256(kaa, 131, (const uint8_t*)big, strlen(big), nullptr, 0, nullptr, 0, o); bytesToHex(o, 32, h); CHECK(strcmp(h, "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54") == 0);       // case 6
  // the protocol tags against an independent implementation (Python hashlib / hmac)
  uint8_t key[32], sn[16], cn[16], tag[16];
  hexToBytes(TKEY, key, 32); for (int i = 0; i < 16; i++) { sn[i] = (uint8_t)(16 + i); cn[i] = (uint8_t)(100 + i); }
  authTag(key, "E2M", sn, cn, tag); bytesToHex(tag, 16, h); CHECK(strcmp(h, "d54502d4b71c0790adc761e607ec55e1") == 0);
  authTag(key, "M2E", cn, sn, tag); bytesToHex(tag, 16, h); CHECK(strcmp(h, "86309f2d6e8b732daa735aaf71dc4b7e") == 0);
  uint8_t t1[16], t2[16]; authTag(key, "E2M", sn, cn, t1); authTag(key, "M2E", sn, cn, t2); CHECK(!ct_equal(t1, t2, 16));       // direction labels differ: no reflection
  // hex parsing is strict
  uint8_t b[4];
  CHECK(hexToBytes("0a0B0c0d", b, 4) && b[1] == 0x0b);
  CHECK(!hexToBytes("0a0b0c0", b, 4) && !hexToBytes("0a0b0c0dd", b, 4) && !hexToBytes("0a0b0c0g", b, 4) && !hexToBytes("", b, 4));
  // the two copies of the header are identical (also checked by run_tests.sh)
  CHECK(true);
}

static void t_auth_enforced_basics() {
  authBoot();
  CHECK(g_auth.enabled && g_auth.keySet && !g_auth.authed && !btAuthOk());
  char fp[9]; authFingerprint(fp); CHECK(strcmp(fp, "17a56f78") == 0);
  // nothing that controls the robot is accepted from an unauthenticated peer
  Serial1.out.clear(); g_btCursor = 0;
  bt("M"); bt("U"); bt("W"); bt("R"); bt("Q"); lineBt("manual"); lineBt("defend"); lineBt("attack"); lineBt("calibrate"); lineBt("stop"); lineBt("motors"); lineBt("set maxauto 255"); simRun(200);
  CHECK(!g_remoteManual && !g_defendEnabled && !g_autoEnabled && allWheelsStopped() && g_cfg.maxAuto == 170);
  CHECK(btHas("ERR:AUTH") && g_auth.refusedCount >= 8);
  CHECK(outCountOf(Serial1.out, "ERR:AUTH") <= 3);                                  // the refusal replies are rate limited
  // no telemetry to a peer that has not authenticated
  Serial1.out.clear(); simRun(3000);
  CHECK(btFrames().empty());
  // the ping and the emergency stop are accepted from anyone
  Serial1.out.clear(); bt("P"); simRun(30);
  CHECK(btHas("PONG:AUTH"));
  bt("X"); loop();
  CHECK(g_estop && allWheelsStopped());
  CHECK(noSecretOnAnyWire());
  // the real handshake
  g_estop = false;
  pumpClient(true, 600);
  CHECK(clientAuthed(&g_cl) && g_auth.authed && btAuthOk() && g_auth.okCount == 1);
  bt("M"); simRun(60);
  CHECK(g_remoteManual);
  lineBt("defend"); simRun(100);
  CHECK(g_defendEnabled);
  Serial1.out.clear(); g_btCursor = 0; simRun(3000);
  CHECK(!btFrames().empty());                                                       // telemetry flows once authenticated
  CHECK(noSecretOnAnyWire());                                                       // the key and the passcode never appear on either serial line
  // a bad length / bad hex key is refused and does not replace the good one
  send(":authkey 00\n"); simRun(60); CHECK(outHas("key rejected"));
  char fp2[9]; authFingerprint(fp2); CHECK(strcmp(fp2, "17a56f78") == 0);
}

static void t_auth_invalid() {
  authBoot();
  // wrong key on the client side
  clientInit(&g_cl, TKEY_WRONG);
  pumpClient(true, 1500);
  CHECK(!clientAuthed(&g_cl) && !g_auth.authed && g_auth.failCount >= 1);
  Serial1.out.clear(); g_btCursor = 0; bt("M"); simRun(60);
  CHECK(!g_remoteManual && btHas("ERR:AUTH"));
  // hand-made attempts: 3 failures in a row start the lockout; the right proof is then refused until it ends
  authBoot();
  std::string ch;
  for (int i = 0; i < 3; i++) {
    simRun(400); lineBt("AUTH"); simRun(60); ch = lastBt("AUTH CH ");
    CHECK(ch.size() == 40);
    lineBt(respLine(TKEY_WRONG, ch, 10).c_str()); simRun(60);
  }
  CHECK(g_auth.fails == 3 && g_auth.lockUntilMs != 0 && !g_auth.authed);
  simRun(400); Serial1.out.clear(); g_btCursor = 0; lineBt("AUTH"); simRun(60);
  CHECK(btHas("ERR:AUTH_LOCKOUT") && lastBt("AUTH CH ").empty());
  simRun(4200);
  lineBt("AUTH"); simRun(60); ch = lastBt("AUTH CH ");
  CHECK(ch.size() == 40);
  lineBt(respLine(TKEY, ch, 20).c_str()); simRun(60);
  CHECK(g_auth.authed && g_auth.fails == 0);                                        // the correct proof works once the lockout is over, and the counter resets
  // malformed proofs count as failures and never authenticate
  authBoot();
  const char* bad[] = {"AUTH RESP zz", "AUTH RESP", "AUTH RESP 00112233445566778899aabbccddeeff", "AUTH RESP 00112233445566778899aabbccddeeff 00112233445566778899aabbccddeef",
                       "AUTH RESP 00112233445566778899aabbccddeeff 00112233445566778899aabbccddeeffaa", "AUTH RESP gg112233445566778899aabbccddeeff 00112233445566778899aabbccddeeff", "AUTH BOGUS"};
  for (const char* b : bad) { simRun(400); lineBt("AUTH"); simRun(60); lineBt(b); simRun(60); CHECK(!g_auth.authed); }
  // a proof with one flipped bit
  authBoot(); lineBt("AUTH"); simRun(60); ch = lastBt("AUTH CH ");
  lineBt(respLine(TKEY, ch, 30, true).c_str()); simRun(60);
  CHECK(!g_auth.authed && btHas("ERR:AUTH_FAIL"));
  // a proof without any challenge outstanding
  authBoot(); lineBt(respLine(TKEY, std::string("AUTH CH 000102030405060708090a0b0c0d0e0f"), 1).c_str()); simRun(60);
  CHECK(!g_auth.authed && btHas("ERR:AUTH_NOCHAL"));
}

static void t_auth_replay_expired() {
  authBoot();
  lineBt("AUTH"); simRun(60);
  std::string ch1 = lastBt("AUTH CH ");
  std::string resp1 = respLine(TKEY, ch1, 40);
  lineBt(resp1.c_str()); simRun(60);
  CHECK(g_auth.authed);
  // the SAME proof again: the challenge was consumed
  Serial1.out.clear(); g_btCursor = 0; lineBt(resp1.c_str()); simRun(60);
  CHECK(btHas("ERR:AUTH_NOCHAL"));
  // end the session, then replay the recorded proof: refused, still locked
  lineBt("AUTH OUT"); simRun(60);
  CHECK(!g_auth.authed);
  Serial1.out.clear(); g_btCursor = 0; lineBt(resp1.c_str()); simRun(60);
  CHECK(!g_auth.authed && btHas("ERR:AUTH_NOCHAL"));
  // a new challenge is different, and the old proof does not answer it
  simRun(400); lineBt("AUTH"); simRun(60);
  std::string ch2 = lastBt("AUTH CH ");
  CHECK(ch2 != ch1 && ch2.size() == 40);
  lineBt(resp1.c_str()); simRun(60);
  CHECK(!g_auth.authed);
  // a proof that arrives after the challenge expired
  simRun(400); lineBt("AUTH"); simRun(60);
  std::string ch3 = lastBt("AUTH CH ");
  waitPing(AUTH_CH_MS + 500);
  Serial1.out.clear(); g_btCursor = 0;
  lineBt(respLine(TKEY, ch3, 50).c_str()); simRun(60);
  CHECK(!g_auth.authed && btHas("ERR:AUTH_EXPIRED") && g_auth.expiredCount >= 1);
  // ... and one that arrives just in time
  simRun(400); lineBt("AUTH"); simRun(60);
  std::string ch4 = lastBt("AUTH CH ");
  waitPing(AUTH_CH_MS - 2000);
  lineBt(respLine(TKEY, ch4, 60).c_str()); simRun(60);
  CHECK(g_auth.authed);
  // challenges are never repeated, not even after a reboot (persistent boot counter)
  uint32_t bc = g_auth.bootCtr;
  lineBt("AUTH OUT"); simRun(60);
  simRun(400); lineBt("AUTH"); simRun(60);
  std::string first = lastBt("AUTH CH ");
  g_mockAuthEnforced = true; setup(); g_autoStartPending = false; layout8(); calibrateAll(); Serial1.out.clear(); g_btCursor = 0;
  CHECK(g_auth.bootCtr == bc + 1u && g_auth.keySet && !g_auth.authed);
  lineBt("AUTH"); simRun(60);
  std::string second = lastBt("AUTH CH ");
  CHECK(second.size() == 40);
  // (chCtr restarts at 1 after the reboot, as it did for 'first' only if first was also challenge 1; compare against a same-counter challenge)
  CHECK(second != first);
  CHECK(!g_auth.authed);                                                            // a reset ends every session
}

static void t_auth_session_and_reconnect() {
  authBoot();
  pumpClient(true, 600);
  CHECK(g_auth.authed);
  // a held key keeps the session; the move stops with the lease, the session with 2.5 s of silence
  bt("M"); for (int k = 0; k < 20; k++) { bt("W"); simRun(100); }
  CHECK(g_auth.authed && !allWheelsStopped());
  simRun(g_cfg.btLeaseMs + 400);
  CHECK(allWheelsStopped() && g_auth.authed);
  simRun(AUTH_IDLE_MS);
  CHECK(!g_auth.authed && !btAuthOk() && outHas("Bluetooth session ended"));
  Serial1.out.clear(); g_btCursor = 0; bt("W"); simRun(100);
  CHECK(allWheelsStopped() && btHas("ERR:AUTH"));
  // the remote reconnects: it must authenticate again before anything works
  clientReset(&g_cl); g_cl.nextTryMs = 0;
  pumpClient(true, 800);
  CHECK(clientAuthed(&g_cl) && g_auth.authed && g_auth.okCount == 2);
  // pings keep the session alive without any other traffic
  uint32_t ok = g_auth.okCount;
  for (int k = 0; k < 40; k++) { bt("P"); simRun(500); }
  CHECK(g_auth.authed && g_auth.okCount == ok);
  // the client notices a robot that lost the session (PONG:AUTH / ERR:AUTH) and re-handshakes by itself
  g_auth.authed = false;
  for (int k = 0; k < 20; k++) { char b[100]; size_t n = clientWantLine(&g_cl, true, millis(), rndTest, b, sizeof(b)); if (n) { b[n] = 0; lineBt(b); } bt("P"); simRun(100); for (auto& l : btNewLines()) clientOnLine(&g_cl, l.c_str(), millis()); }
  CHECK(g_auth.authed && clientAuthed(&g_cl));
  // a new AUTH request ends the current session immediately
  simRun(400); lineBt("AUTH"); simRun(60);
  CHECK(!g_auth.authed);
  // link-loss timeout (btlink) also ends it, and attack / defend keep running without the link
  authBoot(); pumpClient(true, 600); ballFar(); lineBt("attack"); simRun(1000);
  CHECK(g_autoEnabled && g_auth.authed);
  simRun(g_cfg.btLinkMs + 500);
  CHECK(!g_auth.authed && g_autoEnabled);                                           // no link needed for autonomous play; nothing started or stopped it
  // a different client later: needs the key
  clientInit(&g_cl, TKEY_WRONG); pumpClient(true, 1500);
  CHECK(!g_auth.authed);
}

static void t_auth_estop_latch() {
  authBoot();
  // unauthenticated emergency stop from a running autonomous mode
  pumpClient(true, 600); ballFar(); lineBt("attack"); simRun(1500);
  CHECK(g_autoEnabled && !allWheelsStopped());
  simRun(AUTH_IDLE_MS + 300);
  CHECK(!g_auth.authed && g_autoEnabled);
  bt("X"); loop();
  CHECK(g_estop && allWheelsStopped() && !g_autoEnabled);
  // unauthenticated: the latch cannot be cleared and nothing can start
  Serial1.out.clear(); g_btCursor = 0; bt("R"); lineBt("attack"); lineBt("defend"); bt("M"); simRun(100);
  CHECK(g_estop && allWheelsStopped() && btHas("ERR:AUTH"));
  // authenticating does NOT clear it, and authentication failures do not move anything
  clientInit(&g_cl, TKEY_WRONG); pumpClient(true, 600);
  CHECK(g_estop && allWheelsStopped());
  clientInit(&g_cl, TKEY); g_cl.nextTryMs = 0; pumpClient(true, 3000);
  CHECK(g_auth.authed && g_estop && allWheelsStopped());
  // an authenticated session still gets ERR:ESTOP for movement, and only an explicit R clears the latch
  Serial1.out.clear(); g_btCursor = 0; bt("M"); bt("W"); simRun(100);
  CHECK(btHas("ERR:ESTOP") && allWheelsStopped());
  bt("R"); simRun(60);
  CHECK(!g_estop && allWheelsStopped() && !g_autoEnabled && !g_remoteManual);        // cleared, and the robot stays idle until told otherwise
  // X works whatever the session state, from every mode (one pass of the loop each)
  for (int mode = 0; mode < 3; mode++) {
    if (mode == 0) { bt("M"); for (int k = 0; k < 4; k++) { bt("W"); simRun(60); } }
    else if (mode == 1) { ballFar(); lineBt("attack"); simRun(800); }
    else { lineBt("defend"); simRun(600); ballAt(70, 0.6f); simRun(600); }
    if (mode == 2) { g_auth.authed = false; }                                        // and also with the session gone
    bt("X"); loop();
    CHECK(allWheelsStopped() && g_estop && !g_defendEnabled && !g_autoEnabled);
    g_estop = false; g_auth.authed = true; g_auth.lastOkMs = millis(); ballGone(); simRun(100);
  }
}

static void t_auth_nokey_and_open() {
  authBoot(false);
  send(":authkey clear\n"); simRun(60);
  CHECK(!g_auth.keySet && EEPROM.mem[EEPROM_AUTH_ADDR] == 0xFF);
  Serial1.out.clear(); g_btCursor = 0;
  bt("M"); lineBt("attack"); lineBt("AUTH"); simRun(100);
  CHECK(!g_remoteManual && !g_autoEnabled && btHas("ERR:NO_KEY") && !btAuthOk());   // fail closed: no key, no control
  bt("X"); loop(); CHECK(g_estop);                                                  // but the emergency stop still works
  g_estop = false;
  // bad keys are refused
  const char* badKeys[] = {"1234", "zz0102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", "0000000000000000000000000000000000000000000000000000000000000000",
                           "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f00", "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1"};
  for (const char* k : badKeys) { Serial.out.clear(); send(":authkey "); send(k); send("\n"); simRun(60); CHECK(outHas("key rejected") && !g_auth.keySet); }
  send(":authkey "); send(TKEY); send("\n"); simRun(60);
  CHECK(g_auth.keySet && outHas("fingerprint 17a56f78"));
  CHECK(Serial.out.find(TKEY) == std::string::npos);                                // the key is never echoed
  Serial.out.clear(); send(":authkey\n"); simRun(60);
  CHECK(outHas("fingerprint 17a56f78") && Serial.out.find(TKEY) == std::string::npos);
  Serial.out.clear(); send(":bt\n"); simRun(60);
  CHECK(Serial.out.find(TKEY) == std::string::npos && outHas("ENFORCED"));
  // the key survives a reboot, the session does not
  setup(); g_autoStartPending = false;
  CHECK(g_auth.keySet && !g_auth.authed && g_auth.enabled);
  // bench mode: enforcement off until the next reset; the handshake says so and the client never treats it as authenticated
  send(":auth off\n"); simRun(60);
  CHECK(!g_auth.enabled && btAuthOk());
  Serial1.out.clear(); g_btCursor = 0; bt("M"); lineBt("AUTH"); simRun(100);
  CHECK(g_remoteManual && btHas("AUTH OPEN"));
  clientInit(&g_cl, TKEY); g_cl.st = CL_WAIT_CH; clientOnLine(&g_cl, "AUTH OPEN", millis());
  CHECK(g_cl.robotOpen && !clientAuthed(&g_cl));
  send(":auth on\n"); simRun(60);
  Serial1.out.clear(); g_btCursor = 0; bt("W"); simRun(60);
  CHECK(btHas("ERR:AUTH"));
  setup(); g_autoStartPending = false;
  CHECK(g_auth.enabled);                                                            // after a reset enforcement is back on, whatever was set before (host harness aside)
  CHECK(EEPROM.mem[EEPROM_AUTH_ADDR] != 0xFF);
}





struct TestCase { const char* name; void (*fn)(); };
#define T(n) {#n, n}
static const TestCase TESTS[] = {
  T(t_minifmt),
  T(t_report_paced),
  T(t_records),
  T(t_records_wrap),
  T(t_missing_hardware),
  T(t_commands),
  T(t_angles),
  T(t_timer_rollover),
  T(t_mecanum),
  T(t_pinmap),
  T(t_boot_safe),
  T(t_ramp),
  T(t_config),
  T(t_config_survives_reboot),
  T(t_imu_integrate),
  T(t_imu_bias),
  T(t_imu_missing),
  T(t_imu_dropout),
  T(t_imu_invalid),
  T(t_heading_hold),
  T(t_safe_stop_moving),
  T(t_loop_stall),
  T(t_cli_manual),
  T(t_cli_robust),
  T(t_diag_no_motion),
  T(t_motor_test),
  T(t_drive_test),
  T(t_cfg_set),
  T(t_nonblocking),
  T(t_health_report_text),
  T(t_imu_frozen_in_approach),
  T(t_ring_pure_rules),
  T(t_ring_pins_and_isolation),
  T(t_ring_boot_sequence),
  T(t_ring_detect_and_bearing),
  T(t_ring_burst_window),
  T(t_ring_noise_and_threshold),
  T(t_ring_saturation_and_faults),
  T(t_ring_coverage_gap),
  T(t_ring_cli),
  T(t_modules),
  T(t_ring_autonomy_refused),
  T(t_nav_approach_and_stop),
  T(t_nav_turn_toward_ball),
  T(t_nav_lost_search_reacquire),
  T(t_nav_blind_spot_search),
  T(t_nav_ring_fault_safe),
  T(t_goal_register_and_aim_window),
  T(t_shoot_sequence),
  T(t_shoot_preconditions),
  T(t_shoot_aim_timeout_and_ball_lost),
  T(t_goal_invalid_after_gyro_reset),
  T(t_shoot_launch_clock_and_one_shot_per_ball),
  T(t_shot_speed_old_vs_new),
  T(t_shot_parallel_aims_while_reversing),
  T(t_shoot_pwm_limit_and_no_reverse),
  T(t_autostart),
  T(t_defend_enter_exit),
  T(t_defend_behaviour),
  T(t_defend_prediction),
  T(t_defend_hysteresis_and_limits),
  T(t_defend_fault_stop),
  T(t_estop_every_mode),
  T(t_cmd_parser_limits),
  T(t_telemetry_frames),
  T(t_telemetry_imu_large_values),
  T(t_gyro_diagnostics),
  T(t_motion_diagnostics),
  T(t_attack_side_ball),
  T(t_ring_polarity_and_calibration),
  T(t_direction_confidence),
  T(t_sha256_hmac_vectors),
  T(t_auth_enforced_basics),
  T(t_auth_invalid),
  T(t_auth_replay_expired),
  T(t_auth_session_and_reconnect),
  T(t_auth_estop_latch),
  T(t_auth_nokey_and_open),
  T(t_bt_protocol),
  T(t_bt_modes_and_estop),
  T(t_bt_link_loss),
  T(t_under_sensor),
  T(t_reattack),
  T(t_reattack_phases),
  T(t_search_layers),
  T(t_shoot_no_goal_and_wheel_scale),
  T(t_old_hardware_removed),
};

int main(int argc, char** argv) {
  if (argc >= 2 && strcmp(argv[1], "--list") == 0) { for (auto& t : TESTS) printf("%s\n", t.name); return 0; }
  if (argc < 2) { fprintf(stderr, "usage: %s <test>|--list\n", argv[0]); return 2; }
  for (auto& t : TESTS) {
    if (strcmp(t.name, argv[1]) != 0) continue;
    freshEeprom(t.name);
    t.fn();
    if (getenv("SHOW_OUT")) fwrite(Serial.out.data(), 1, Serial.out.size(), stdout);
    printf("%s %s (%d checks, %d failed)\n", g_fail ? "FAIL" : "PASS", t.name, g_checks, g_fail);
    return g_fail ? 1 : 0;
  }
  fprintf(stderr, "no such test %s\n", argv[1]);
  return 2;
}
