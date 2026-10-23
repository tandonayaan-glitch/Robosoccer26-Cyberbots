// cli.h - Serial command interface (115200 baud).
//   Single keys act immediately:  f b l r q e s i m t a h I d g x ?   and '!' (emergency stop).
//   Words start with ':' and end at CR/LF, e.g. ":set maxauto 150".
// Input is bounded (64 chars), non-printables are ignored, an unterminated ':' line is abandoned
// after 4 s so a stray ':' can never swallow a later 's'. Diagnostic commands never move motors.
#pragma once
#include "report.h"
#include "bt.h"

static uint32_t g_lastLoopUs = 0;      // loop() timestamp; cli.h may reset it after an intentional long operation

static bool g_streamIr = false;
static uint32_t g_streamLastMs = 0;
static bool g_testMode = false;
static bool g_lineMode = false, g_lineBad = false, g_lineOvf = false;
static char g_line[64];
static uint8_t g_lineLen = 0;
static uint32_t g_lineStartMs = 0;
static char g_word[40];
static uint8_t g_wordLen = 0;
static bool g_wordOvf = false;
static uint32_t g_wordLastMs = 0;

static void printHelp() {
  say("INFO", "--- commands (115200 baud, line ending: Newline or Both NL & CR). A single key sent alone acts at once; a WORD ends with Enter; ':' commands take arguments ---");
  say("INFO", "WORDS (same on USB and Bluetooth): defend | attack | manual | stop | status | sensors | gyro [drift [s]|cal|stop] | calibrate | motors | tele | help   ('!' or X = emergency stop)");
  say("INFO", "f b l r : manual forward/back/strafe-left/strafe-right   q e : rotate CCW/CW   (each key moves ~%u ms, repeat to continue)", (unsigned)g_cfg.manualHoldMs);
  say("INFO", "s : stop + disable autonomy    ! : emergency stop    a : ENABLE autonomous ball seeking (refused if unsafe)");
  say("INFO", "i : receivers + modules   I : live receiver view   m : IMU   d : nav/ball/PWM   h : health report   g : diagnostics   t : test mode help   ? : this help");
  say("INFO", "--- records (events kept in the EEPROM) ---");
  say("INFO", ":log = show all records | :log clear | :log live on|off (print each new record at once) | :log note <n> (add a marker record)");
  say("INFO", "PC: tools/get_records.py saves the records to records.txt (see README)");
  say("INFO", "--- status / diagnosis (use these when something may be missing) ---");
  say("INFO", ":status = one-screen summary | :raw = raw A0..A7, module pins, IMU | :pins = pin map | :ver | :diag on|off = CSV rows over Serial");
  say("INFO", "--- control ---");
  say("INFO", ":auto on|off (= a / s) | :stop | :estop (latched) | :manual f|b|l|r|q|e | :goal [set|clear] | :zero | :gcal | :recover (also clears the e-stop latch)");
  say("INFO", ":bt = Bluetooth remote status (commands M U W S A D Z C Q X R P ?; docs/BLUETOOTH.md)");
  say("INFO", ":mtest <wheel> | :dtest (whole drivetrain) then :confirm (wheels LIFTED!) | :mok <wheel> | :mrev <wheel>");
  say("INFO", "--- settings (EEPROM) ---");
  say("INFO", ":set <name> <v> | :get [name] | :save | :load | :defaults | :rev <FL|FR|BL|BR> <0|1> | :roller <X|O>");
  say("INFO", ":ring [on|off <0-7> | bearing <0-7> <deg> | cal noise | cal near | reset] | :mod [on|off <0|1> | level <0|1> low|high]");
  say("INFO", ":tele [on|off|usb on|usb off|now] = telemetry to the ESP32 | :gyro drift [s] = stationary drift test | :motors = commanded motor state | :defend / :attack / :manual = modes (docs/DEFENDING.md)");
  say("INFO", "Safety: first motor tests with wheels off the ground and the motor battery switch within reach.");
}

static bool parseF(const char* s, float* out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  float v = (float)strtod(s, &end);
  if (end == s || *end != 0 || !isfinite(v)) return false;
  *out = v;
  return true;
}

#include "ringcli.h"

// 'calibrate': gyro bias + receiver noise floor in one safe step. Refused unless everything is stopped and no ball signal is present.
static void cmdCalibrate(uint32_t now) {
  (void)now;
  const Config& c = g_cfg;
  if (g_estop) { say("SAFETY", "calibrate refused: emergency stop is latched"); return; }
  if (g_autoEnabled || g_defendEnabled || g_navState == NAV_MANUAL || g_navState == NAV_TEST || !motorsAllZero()) { say("SAFETY", "calibrate refused: stop first ('stop') and wait for the wheels to rest"); return; }
  if (g_ball.valid) { say("WARN", "calibrate refused: a ball signal is present (confidence %u). Remove or switch off the ball and any other infrared source, then repeat.", (unsigned)g_ball.confidence); return; }
  if (ringCountEnabled(c) == 0) { say("WARN", "calibrate refused: no receiver channel is enabled"); return; }
  if (g_imu.present && g_imu.ok) { imuStartCalibration(); say("INFO", "gyro bias calibration started (~%u samples): do not touch the robot", (unsigned)c.imuCalSamples); }
  else say("WARN", "gyro part skipped: the MPU6050 is not delivering data");
  memset(g_ringCalMax, 0, sizeof(g_ringCalMax));
  g_ringCalActive = true; g_ringCalEndMs = millis() + 2000u;
  say("INFO", "receiver noise floor calibration: measuring for 2 s with the ball OFF. Mode shows CALIBRATION meanwhile; ':save' keeps the result.");
}

static void cmdTestModeText() {
  say("TEST", "TEST MODE: motors are OFF and nothing moves by itself. Bench checklist:");
  say("TEST", " 1) wheels off the ground, robot secured, motor power switch reachable");
  say("TEST", " 2) per wheel: :mtest FL  then :confirm -> watch rotation -> :mok FL (right) or :mrev FL (wrong, repeat)");
  say("TEST", " 3) :dtest then :confirm runs forward/back/strafe/rotate; or f b l r q e in short bursts. ':roller O' if strafe is mirrored");
  say("TEST", " 4) ring: 'i' shows the channels and modules. Ball OFF -> ':ring cal noise'; ball in the shooting position -> ':ring cal near'");
  say("TEST", "    ':ring bearing <ch> <deg>' = where each receiver looks; ':mod' = the IR modules (check the active level with a hand in front)");
  say("TEST", " 5) :save to persist. 'h' shows what is verified and what is NOT TESTED.");
}

static void cmdTestMode() {
  navStop(PSTR("test mode"));
  g_testMode = true;
  reportRequest(REP_TEST);
}

static void cmdParamSet(const char* name, const char* val) {
  ParamDef p;
  float v;
  if (!paramFind(name, &p) || !parseF(val, &v)) { say("WARN", "usage: :set <name> <number>   (':get' lists names)"); return; }
  if (!paramInRange(p, v)) { say("WARN", "%s=%g rejected: allowed %g..%g", p.name, (double)v, (double)p.lo, (double)p.hi); return; }
  float old = paramGet(g_cfg, p);
  paramSet(g_cfg, p, v);
  Config chk = g_cfg;                            // validated on a copy (cross-field limits); the live value is restored if it conflicts
  char rep[40];
  if (cfgValidate(chk, rep, sizeof(rep)) != 0) { paramSet(g_cfg, p, old); say("WARN", "%s=%g rejected: conflicts with related limits (%s)", p.name, (double)v, rep); return; }
  say("INFO", "%s = %g", p.name, (double)paramGet(g_cfg, p));
}

static void cmdParamGet(const char* name) {
  ParamDef p;
  if (name) {
    if (!paramFind(name, &p)) { say("WARN", "unknown parameter '%s'", name); return; }
    say("INFO", "%s = %g (range %g..%g)", p.name, (double)paramGet(g_cfg, p), (double)p.lo, (double)p.hi);
    return;
  }
  char line[120]; line[0] = 0;
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    paramAt(i, &p);
    char one[32];
    snprintf(one, sizeof(one), "%s=%g ", p.name, (double)paramGet(g_cfg, p));
    if (strlen(line) + strlen(one) >= sizeof(line) - 1) { say("INFO", "%s", line); line[0] = 0; }
    strcat(line, one);
  }
  if (line[0]) say("INFO", "%s", line);
  say("INFO", "rev=%d%d%d%d roller=%c motorverified=0x%X ringenable=0x%X", g_cfg.reverse[0], g_cfg.reverse[1], g_cfg.reverse[2], g_cfg.reverse[3], g_cfg.rollerO ? 'O' : 'X', (unsigned)g_cfg.motorVerified, (unsigned)g_cfg.ringEnable);
}

static void cmdRecover(const Config& c, uint32_t now) {
  (void)now;
  int cleared = 0;
  if (g_estop) { g_estop = false; g_remoteManual = false; recAdd(REC_ESTOP, 0); cleared++; }          // the emergency-stop latch; the robot stays idle
  if (g_fault[F_LOOP_STALL].active) { g_fault[F_LOOP_STALL].active = false; cleared++; }
  if (g_fault[F_MOTOR_INIT].active) {
    g_fault[F_MOTOR_INIT].active = false;
    if (motorsAttachPwm(c)) cleared++;     // re-raises itself if still failing
  }
  condEvaluate();
  say("SAFETY", "recover: %d latched fault(s) cleared; condition now %S. Autonomous stays DISABLED; verify with 'h' then send 'a'.", cleared, condName(g_cond));
}

static void cmdSave() {
  if (cfgSave(g_cfg)) { say("INFO", "configuration saved to EEPROM (CRC protected)"); recAdd(REC_CFG_SAVED, 0); }
  else say("INFO", "save FAILED (invalid settings or EEPROM verify error)");
}

static void cliExecLine(char* line, uint32_t now) {
  char* save = nullptr;
  char* t[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  int n = 0;
  for (char* tok = strtok_r(line, " \t", &save); tok && n < 5; tok = strtok_r(nullptr, " \t", &save)) t[n++] = tok;
  if (n == 0) return;
  const char* cmd = t[0];
  Config& c = g_cfg;
  if (STREQ(cmd, "help")) reportRequest(REP_HELP);
  else if (STREQ(cmd, "defend")) { defendEnable(c, now); }
  else if (STREQ(cmd, "attack")) { if (navEnableAuto(c, now)) say("NAV", "ATTACK mode (autonomous ball seeking) ENABLED. 'stop' ends it."); }
  else if (STREQ(cmd, "idle")) { navStop(PSTR("operator: idle")); g_remoteManual = false; say("SAFETY", "idle: stopped, no mode active"); }
  else if (STREQ(cmd, "sensors")) reportRequest(REP_SENSORS);
  else if (STREQ(cmd, "motors")) reportRequest(REP_MOTORS);
  else if (STREQ(cmd, "calibrate")) cmdCalibrate(now);
  else if (STREQ(cmd, "gyro")) {
    if (!t[1]) reportRequest(REP_GYRO);
    else if (STREQ(t[1], "cal")) { if (!motorsAllZero()) say("WARN", "gyro calibration refused: motors are not stopped"); else if (!g_imu.present) say("WARN", "gyro calibration impossible: IMU not present"); else { imuStartCalibration(); say("INFO", "gyro bias calibration started - keep the robot completely still"); } }
    else if (STREQ(t[1], "stop")) { imuDriftFail(5); say("INFO", "drift test stopped"); }
    else if (STREQ(t[1], "drift")) {
      float sec = (float)c.driftSec;
      if (t[2] && (!parseF(t[2], &sec) || sec < 5 || sec > 300)) { say("WARN", "usage: gyro drift [5..300 seconds]"); }
      else if (imuDriftStart(c, now, (uint32_t)(sec * 1000.0f), motorsAllZero() && elapsedMs(now, g_motorsStoppedSinceMs) > 1000 && !g_autoEnabled && !g_defendEnabled && g_navState != NAV_MANUAL)) say("INFO", "gyro drift test running for %u s: do not touch or move the robot; 'gyro' shows the result", (unsigned)sec);
      else say("WARN", "gyro drift test refused: needs a calibrated, working MPU6050 and all motors stopped for at least 1 s (try 'calibrate' / 'gyro cal' first)");
    } else say("WARN", "usage: gyro | gyro drift [s] | gyro cal | gyro stop");
  }
  else if (STREQ(cmd, "tele")) {
    if (!t[1]) printTele(c, now);
    else if (STREQ(t[1], "on")) { c.teleEnable = 1; say("INFO", "telemetry ON"); }
    else if (STREQ(t[1], "off")) { c.teleEnable = 0; say("INFO", "telemetry OFF"); }
    else if (STREQ(t[1], "now")) { teleSendNow(c, now, 'c'); teleSendNow(c, now, 'r'); }
    else if (STREQ(t[1], "usb")) { g_tele.usb = t[2] && STREQ(t[2], "on"); say("INFO", "telemetry echo on USB %S", g_tele.usb ? PSTR("ON (frames also print here)") : PSTR("OFF")); }
    else say("WARN", "usage: :tele [on|off|now|usb on|usb off]");
  }
  else if (STREQ(cmd, "set")) cmdParamSet(t[1] ? t[1] : "", t[2]);
  else if (STREQ(cmd, "get")) cmdParamGet(t[1]);
  else if (STREQ(cmd, "save")) cmdSave();
  else if (STREQ(cmd, "load")) {
    char rep[40];
    Config tmp;
    CfgLoadResult r = cfgLoad(tmp, rep, sizeof(rep));
    c = tmp; say("INFO", "config %S%s%s", cfgLoadText(r), rep[0] ? ": " : "", rep);
  }
  else if (STREQ(cmd, "defaults")) { cfgDefaults(c); say("INFO", "defaults loaded (not saved; :save to keep). Bearings are placeholders - recalibrate."); }
  else if (STREQ(cmd, "rev")) {
    int w = t[1] ? wheelByName(t[1]) : -1; float v;
    if (w < 0 || !parseF(t[2], &v) || (v != 0 && v != 1)) say("WARN", "usage: :rev <FL|FR|BL|BR> <0|1>");
    else { c.reverse[w] = (uint8_t)v; c.motorVerified &= (uint8_t)~(1u << w); say("INFO", "%s reversal=%d (direction verification cleared)", WHEEL_PINS[w].name, c.reverse[w]); }
  }
  else if (STREQ(cmd, "roller")) {
    if (t[1] && (t[1][0] == 'X' || t[1][0] == 'x' || t[1][0] == 'O' || t[1][0] == 'o')) { c.rollerO = (t[1][0] == 'O' || t[1][0] == 'o'); c.motorVerified = 0; say("INFO", "roller layout %c (all wheel verifications cleared)", c.rollerO ? 'O' : 'X'); }
    else say("WARN", "usage: :roller <X|O>");
  }
  else if (STREQ(cmd, "ring")) cmdRing(c, t[1], t[2], t[3]);
  else if (STREQ(cmd, "mod")) cmdMod(c, t[1], t[2], t[3]);
  else if (STREQ(cmd, "goal")) {
    if (t[1] && STREQ(t[1], "set")) {
      if (navGoalRegister(c, now)) say("INFO", "goal heading registered NOW at %.1f deg of gyro yaw (the robot must be pointing at the goal, standing still)", (double)g_goalYaw);
      else say("WARN", "goal heading not registered: the gyro is not calibrated/usable");
    } else if (t[1] && STREQ(t[1], "clear")) { g_goalRefSet = false; g_goalWaitNoted = false; g_goalAutoOff = true; say("INFO", "goal heading cleared; it stays unregistered until ':goal set' (no shot without it)"); }
    else if (t[1]) say("WARN", "usage: :goal [set|clear]");
    else if (!g_goalRefSet) say("INFO", "goal heading not registered (registers %u ms after power-on; ':goal set' does it now)", (unsigned)c.goalRegisterMs);
    else if (!goalValid()) say("WARN", "goal heading INVALID: the gyro reference was reset (':zero', recalibration or an IMU outage). Point at the goal, ':goal set'");
    else say("INFO", "goal heading %.1f deg, yaw %.1f, error %.1f deg; aim window +-%.1f deg (goal %u mm, %u mm away, ball %u mm); fires at half",
             (double)g_goalYaw, (double)g_imu.yaw, (double)angDiffDeg(g_goalYaw, g_imu.yaw), (double)shootAimTolDeg(c), (unsigned)c.goalWidthMm, (unsigned)c.goalDistMm, (unsigned)c.ballDiaMm);
  }
  else if (STREQ(cmd, "zero")) { imuZeroYaw(); g_refYaw = 0; say("INFO", "heading reference zeroed"); }
  else if (STREQ(cmd, "gcal")) {
    if (!motorsAllZero()) say("WARN", "gyro calibration refused: motors are not stopped");
    else if (!g_imu.present) say("WARN", "gyro calibration impossible: IMU not present");
    else { imuStartCalibration(); say("INFO", "gyro bias calibration started - keep the robot completely still (~2 s)"); }
  }
  else if (STREQ(cmd, "mtest")) { int w = t[1] ? wheelByName(t[1]) : -1; if (!navMotorTestArm(c, w, now)) say("WARN", "usage: :mtest <FL|FR|BL|BR>"); }
  else if (STREQ(cmd, "dtest")) navMotorTestArm(c, 0, now, true);
  else if (STREQ(cmd, "confirm")) navMotorTestConfirm(now);
  else if (STREQ(cmd, "mok")) { int w = t[1] ? wheelByName(t[1]) : -1; if (w < 0) say("WARN", "usage: :mok <wheel>"); else { c.motorVerified |= (uint8_t)(1u << w); say("INFO", "%s direction recorded as operator-confirmed (':save' to keep). Rotation was never measured by the robot.", WHEEL_PINS[w].name); } }
  else if (STREQ(cmd, "mrev")) { int w = t[1] ? wheelByName(t[1]) : -1; if (w < 0) say("WARN", "usage: :mrev <wheel>"); else { c.reverse[w] ^= 1; c.motorVerified &= (uint8_t)~(1u << w); say("INFO", "%s reversal now %d - repeat :mtest %s to check", WHEEL_PINS[w].name, c.reverse[w], WHEEL_PINS[w].name); } }
  else if (STREQ(cmd, "recover")) cmdRecover(c, now);
  else if (STREQ(cmd, "diag")) {
    g_csvOn = t[1] && STREQ(t[1], "on");
    say("INFO", "CSV rows over Serial %S (every %u ms)", g_csvOn ? PSTR("ON") : PSTR("OFF"), (unsigned)c.snapshotMs);
    if (g_csvOn) logHeader();
  }
  else if (STREQ(cmd, "log")) {
    if (!t[1]) reportRequest(REP_LOG);
    else if (STREQ(t[1], "clear")) { recClear(); say("INFO", "records cleared"); }
    else if (STREQ(t[1], "live")) { g_recLive = t[2] && STREQ(t[2], "on"); say("INFO", "live records %S", g_recLive ? PSTR("ON: each new record is printed as it happens") : PSTR("OFF")); }
    else if (STREQ(t[1], "note")) { float v = 0; parseF(t[2], &v); recAdd(REC_NOTE, (uint16_t)v); say("INFO", "marker record %u added", (unsigned)v); }
    else say("WARN", "usage: :log | :log clear | :log live on|off | :log note <n>");
  }
  else if (STREQ(cmd, "raw")) reportRequest(REP_RAW);
  else if (STREQ(cmd, "pins")) reportRequest(REP_PINS);
  else if (STREQ(cmd, "status")) reportRequest(REP_STATUS);
  else if (STREQ(cmd, "ver")) say("INFO", "Robosoccer26_Cyberbots %s, Arduino Mega 2560 (ATmega2560), built %s %s", FW_VERSION, __DATE__, __TIME__);
  else if (STREQ(cmd, "auto")) {
    if (t[1] && STREQ(t[1], "on")) { if (navEnableAuto(c, now)) say("NAV", "autonomous ball seeking ENABLED. 's' or any manual key stops it."); }
    else if (t[1] && STREQ(t[1], "off")) { navStop(PSTR("operator :auto off")); say("SAFETY", "stopped; autonomy disabled"); }
    else say("WARN", "usage: :auto on|off");
  }
  else if (STREQ(cmd, "stop")) { navStop(PSTR("operator stop")); say("SAFETY", "stopped; autonomy disabled"); }
  else if (STREQ(cmd, "estop")) { navEstop(PSTR("EMERGENCY STOP")); say("SAFETY", "EMERGENCY STOP: latched (':recover' or remote 'R' clears it)"); }
  else if (STREQ(cmd, "bt")) printBt(c, now);
  else if (STREQ(cmd, "manual") && !t[1]) {
    if (!navEstopRefuses(PSTR("manual"))) { navStop(PSTR("operator: manual mode")); g_remoteManual = true; recAdd(REC_MODE, 1); say("NAV", "MANUAL mode: keys f b l r q e on USB, W S A D Z C on Bluetooth. 'stop' leaves it; 'attack' / 'defend' switch."); }
  }
  else if (STREQ(cmd, "manual")) {
    char k1 = t[1] ? t[1][0] : 0;
    if (t[1] && !t[1][1] && (k1 == 'f' || k1 == 'b' || k1 == 'l' || k1 == 'r' || k1 == 'q' || k1 == 'e')) navManual(c, k1, now);
    else say("WARN", "usage: :manual f|b|l|r|q|e");
  }
  else if (STREQ(cmd, "xsel") || STREQ(cmd, "fsformat") || STREQ(cmd, "logdel"))
    say("WARN", "':%s' is not available on the Arduino Mega: no flash filesystem (use ':diag on' to stream CSV rows over Serial)", cmd);
  else say("WARN", "unknown word command '%s' (':help')", cmd);
}

static void cliSingle(char ch, uint32_t now) {
  Config& c = g_cfg;
  switch (ch) {
    case 'f': case 'b': case 'l': case 'r': case 'q': case 'e': navManual(c, ch, now); break;
    case 's': navStop(PSTR("operator stop 's'")); say("SAFETY", "stopped; autonomy disabled"); break;
    case '!': navEstop(PSTR("EMERGENCY STOP")); say("SAFETY", "EMERGENCY STOP: latched (':recover' or remote 'R' clears it)"); break;
    case 'i': reportRequest(REP_RING); break;
    case 'm': reportRequest(REP_IMU); break;
    case 't': cmdTestMode(); break;
    case 'a': if (navEnableAuto(c, now)) say("NAV", "autonomous ball seeking ENABLED. 's' or any manual key stops it."); break;
    case 'h': reportRequest(REP_HEALTH); break;
    case 'I': g_streamIr = !g_streamIr; g_streamLastMs = now; say("INFO", "live ring view %S (every %u ms)", g_streamIr ? PSTR("ON") : PSTR("OFF"), (unsigned)c.streamMs); break;
    case 'd': reportRequest(REP_NAV); break;
    case 'g': say("INFO", "diagnostics: CSV over Serial %S; %u records in the EEPROM (':log'); %lu output lines dropped; free RAM %d B", g_csvOn ? PSTR("ON") : PSTR("OFF (':diag on')"), (unsigned)g_recCount, UL(g_outDropped), freeRam()); break;
    case 'x': say("WARN", "'x' (log export) is not available on the Mega: no flash filesystem"); break;
    case '?': reportRequest(REP_HELP); break;
    default: sayRL(2, 2000, "WARN", "ignored unknown input 0x%02X ('?' for help)", (unsigned)(uint8_t)ch); break;
  }
}

static void cliFinishWord(uint32_t now) {
  uint8_t n = g_wordLen;
  bool ovf = g_wordOvf;
  g_wordLen = 0; g_wordOvf = false;
  if (ovf) { say("WARN", "input line too long: ignored, nothing executed"); return; }
  uint8_t s0 = 0;
  while (s0 < n && (g_word[s0] == ' ' || g_word[s0] == '\t')) s0++;
  while (n > s0 && (g_word[n - 1] == ' ' || g_word[n - 1] == '\t')) n--;
  if (s0 >= n) return;
  if (n - s0 == 1) { cliSingle(g_word[s0], now); return; }          // one key alone = the classic single-key command
  char w[40]; uint8_t k = 0;
  for (uint8_t i = s0; i < n; i++) w[k++] = (g_word[i] >= 'A' && g_word[i] <= 'Z') ? (char)(g_word[i] - 'A' + 'a') : g_word[i];
  w[k] = 0;
  char first[16]; uint8_t f = 0;
  while (f < sizeof(first) - 1 && w[f] && w[f] != ' ' && w[f] != '\t') { first[f] = w[f]; f++; }
  first[f] = 0;
  if (!cliIsWord(first)) { say("WARN", "unknown input '%s': NOTHING was executed (single keys only act when sent alone; words: 'help')", first); return; }
  cliExecLine(w, now);
}

static void cliPoll(uint32_t now) {
  if (g_lineMode && elapsedMs(now, g_lineStartMs) > 4000) { g_lineMode = false; say("WARN", "unterminated ':' line abandoned"); }
  if (g_wordLen && elapsedMs(now, g_wordLastMs) >= 8) cliFinishWord(now);       // no line ending: 8 ms of silence completes the input
  int budget = 64;
  while (Serial.available() > 0 && budget-- > 0) {
    int v = Serial.read();
    if (v < 0) break;
    uint8_t ch = (uint8_t)v;
    if (ch == '!') { cliSingle('!', now); g_lineMode = false; g_wordLen = 0; g_wordOvf = false; continue; }   // works even inside a line or a word
    if (ch == 0x03 || ch == 0x1B) { g_lineMode = false; g_wordLen = 0; g_wordOvf = false; continue; }          // Ctrl-C / ESC cancels the line
    if (ch == '\r' || ch == '\n') {
      if (g_lineMode) {
        g_line[g_lineLen] = 0;
        if (g_lineBad || g_lineOvf) say("WARN", "line rejected (%S)", g_lineOvf ? PSTR("too long") : PSTR("invalid characters"));
        else cliExecLine(g_line, now);
        g_lineMode = false;
      } else if (g_wordLen || g_wordOvf) cliFinishWord(now);
      continue;
    }
    if (g_lineMode) {
      if (ch < 0x20 || ch > 0x7E) { g_lineBad = true; continue; }
      if (g_lineLen < sizeof(g_line) - 1) g_line[g_lineLen++] = (char)ch; else g_lineOvf = true;
      continue;
    }
    if (ch == ':' && g_wordLen == 0) { g_lineMode = true; g_lineBad = g_lineOvf = false; g_lineLen = 0; g_lineStartMs = now; continue; }
    if (ch < 0x20 || ch > 0x7E) continue;
    if (g_wordLen < sizeof(g_word) - 1) g_word[g_wordLen++] = (char)ch; else g_wordOvf = true;
    g_wordLastMs = now;
  }
}

static void streamService(const Config& c, uint32_t now) {
  if (g_streamIr && elapsedMs(now, g_streamLastMs) >= c.streamMs) { g_streamLastMs = now; printRingStream(c); }
}
