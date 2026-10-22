// cli.h - Serial command interface (115200 baud).
//   Single keys act immediately:  f b l r q e s i m t a h I d g x ?   and '!' (emergency stop).
//   Words start with ':' and end at CR/LF, e.g. ":set maxauto 150".
// Input is bounded (64 chars), non-printables are ignored, an unterminated ':' line is abandoned
// after 4 s so a stray ':' can never swallow a later 's'. Diagnostic commands never move motors.
#pragma once
#include "report.h"

static uint32_t g_lastLoopUs = 0;      // loop() timestamp; cli.h may reset it after an intentional long operation

static bool g_streamIr = false;
static uint32_t g_streamLastMs = 0;
static bool g_testMode = false;
static bool g_lineMode = false, g_lineBad = false, g_lineOvf = false;
static char g_line[64];
static uint8_t g_lineLen = 0;
static uint32_t g_lineStartMs = 0;

static void printHelp() {
  say("INFO", "--- commands (115200 baud). Single keys act at once; words start with ':' and end with Enter ---");
  say("INFO", "f b l r : manual forward/back/strafe-left/strafe-right   q e : rotate CCW/CW   (each key moves ~%u ms, repeat to continue)", (unsigned)g_cfg.manualHoldMs);
  say("INFO", "s : stop + disable autonomy    ! : emergency stop    a : ENABLE autonomous ball seeking (refused if unsafe)");
  say("INFO", "i : IR sensor snapshot   I : toggle live IR view   m : IMU/heading   d : nav/ball/PWM state   h : health report");
  say("INFO", "g : logger status   x : export selected log (:xsel ring|<n>)   t : test mode help   ? : this help");
  say("INFO", ":set <name> <v> | :get [name] | :save | :load | :defaults | :rev <FL|FR|BL|BR> <0|1> | :roller <X|O>");
  say("INFO", ":ir [cal clear|near <0|1> | mode <0|1> analog|digital | on|off <0|1> | side right|left|none | reset] | :goal [set|clear] | :zero | :gcal | :diag <on|off>");
  say("INFO", ":mtest <wheel> | :dtest (whole drivetrain) then :confirm (wheels LIFTED!) | :mok <wheel> | :mrev <wheel> | :recover | :xsel | :fsformat CONFIRM | :logdel CONFIRM");
  say("INFO", "Safety: first motor tests with wheels off the ground and the motor battery switch within reach. Software is not a substitute for hardware protection.");
}

static bool parseF(const char* s, float* out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  float v = strtof(s, &end);
  if (end == s || *end != 0 || !isfinite(v)) return false;
  *out = v;
  return true;
}

#include "ircli.h"

static void cmdTestMode() {
  navStop("test mode");
  g_testMode = true;
  say("TEST", "TEST MODE: motors are OFF and nothing moves by itself. Bench checklist:");
  say("TEST", " 1) wheels off the ground, robot secured, motor power switch reachable");
  say("TEST", " 2) per wheel: :mtest FL  then :confirm  -> watch rotation -> :mok FL (correct) or :mrev FL (wrong, then repeat)");
  say("TEST", " 3) :dtest then :confirm runs forward/back/strafe/rotate; or use f b l r q e in short bursts. ':roller O' if strafe is mirrored");
  say("TEST", " 4) IR: 'i' shows both sensors. Nothing in front -> ':ir cal clear 0'; ball in the shooting position -> ':ir cal near 0'; same for sensor 1; ':set iroffset 1|-1' = side of sensor 1");
  say("TEST", " 5) :save to persist. 'h' shows what is verified and what is NOT TESTED.");
}

static void cmdParamSet(const char* name, const char* val) {
  const ParamDef* p = paramFind(name);
  float v;
  if (!p || !parseF(val, &v)) { say("WARN", "usage: :set <name> <number>   (':get' lists names)"); return; }
  if (!paramInRange(*p, v)) { say("WARN", "%s=%g rejected: allowed %g..%g", name, v, p->lo, p->hi); return; }
  static Config t, chk;                          // 0.5 KB each: not on the task stack
  t = g_cfg;
  paramSet(t, *p, v);
  chk = t;
  char rep[64];
  if (cfgValidate(chk, rep, sizeof(rep)) != 0) { say("WARN", "%s=%g rejected: conflicts with related limits (%s)", name, v, rep); return; }
  g_cfg = t;
  say("INFO", "%s = %g%s", name, paramGet(g_cfg, *p), (strcmp(name, "pwmfreq") == 0) ? " (applies after reboot)" : "");
}

static void cmdParamGet(const char* name) {
  if (name) {
    const ParamDef* p = paramFind(name);
    if (!p) { say("WARN", "unknown parameter '%s'", name); return; }
    say("INFO", "%s = %g (range %g..%g)", p->name, paramGet(g_cfg, *p), p->lo, p->hi);
    return;
  }
  char line[160]; int n = 0; line[0] = 0;
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    char one[40];
    snprintf(one, sizeof(one), "%s=%g ", PARAMS[i].name, paramGet(g_cfg, PARAMS[i]));
    if (n + (int)strlen(one) >= (int)sizeof(line) - 1) { say("INFO", "%s", line); n = 0; line[0] = 0; }
    strcat(line, one); n += (int)strlen(one);
  }
  if (n) say("INFO", "%s", line);
  say("INFO", "rev=%d%d%d%d roller=%c motorverified=0x%X", g_cfg.reverse[0], g_cfg.reverse[1], g_cfg.reverse[2], g_cfg.reverse[3], g_cfg.rollerO ? 'O' : 'X', (unsigned)g_cfg.motorVerified);
}

static void cmdRecover(const Config& c, uint32_t now) {
  (void)now;
  int cleared = 0;
  if (g_fault[F_LOOP_STALL].active) { g_fault[F_LOOP_STALL].active = false; cleared++; }
  if (g_fault[F_BROWNOUT_RESET].active) { g_fault[F_BROWNOUT_RESET].active = false; cleared++; }   // acknowledged by the operator
  if (g_fault[F_ABNORMAL_RESET].active) { g_fault[F_ABNORMAL_RESET].active = false; cleared++; }
  if (g_fault[F_MOTOR_INIT].active) {
    g_fault[F_MOTOR_INIT].active = false;
    if (motorsAttachPwm(c)) cleared++;     // re-raises itself if still failing
  }
  condEvaluate();
  say("SAFETY", "recover: %d latched fault(s) cleared; condition now %s. Autonomous stays DISABLED; verify with 'h' then send 'a'.", cleared, condName(g_cond));
}

static void cmdSave() { say("INFO", cfgSave(g_cfg) ? "configuration saved to /config.bin (CRC protected)" : "save FAILED (no filesystem or invalid settings)"); }

static void cliExecLine(char* line, uint32_t now) {
  char* save = nullptr;
  char* t[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  int n = 0;
  for (char* tok = strtok_r(line, " \t", &save); tok && n < 5; tok = strtok_r(nullptr, " \t", &save)) t[n++] = tok;
  if (n == 0) return;
  const char* cmd = t[0];
  Config& c = g_cfg;
  if (!strcmp(cmd, "help")) printHelp();
  else if (!strcmp(cmd, "set")) cmdParamSet(t[1] ? t[1] : "", t[2]);
  else if (!strcmp(cmd, "get")) cmdParamGet(t[1]);
  else if (!strcmp(cmd, "save")) cmdSave();
  else if (!strcmp(cmd, "load")) {
    char rep[64]; static Config tmp;
    CfgLoadResult r = cfgLoad(tmp, rep, sizeof(rep));
    c = tmp; say("INFO", "config %s%s%s", cfgLoadText(r), rep[0] ? ": " : "", rep);
  }
  else if (!strcmp(cmd, "defaults")) { cfgDefaults(c); say("INFO", "defaults loaded (not saved; :save to keep). Bearings are placeholders - recalibrate."); }
  else if (!strcmp(cmd, "rev")) {
    int w = t[1] ? wheelByName(t[1]) : -1; float v;
    if (w < 0 || !parseF(t[2], &v) || (v != 0 && v != 1)) say("WARN", "usage: :rev <FL|FR|BL|BR> <0|1>");
    else { c.reverse[w] = (uint8_t)v; c.motorVerified &= (uint8_t)~(1u << w); say("INFO", "%s reversal=%d (direction verification cleared)", WHEEL_PINS[w].name, c.reverse[w]); }
  }
  else if (!strcmp(cmd, "roller")) {
    if (t[1] && (t[1][0] == 'X' || t[1][0] == 'x' || t[1][0] == 'O' || t[1][0] == 'o')) { c.rollerO = (t[1][0] == 'O' || t[1][0] == 'o'); c.motorVerified = 0; say("INFO", "roller layout %c (all wheel verifications cleared)", c.rollerO ? 'O' : 'X'); }
    else say("WARN", "usage: :roller <X|O>");
  }
  else if (!strcmp(cmd, "ir")) cmdIr(c, t[1], t[2], t[3]);
  else if (!strcmp(cmd, "goal")) {
    if (t[1] && !strcmp(t[1], "set")) {
      if (navGoalRegister(c, now)) say("INFO", "goal heading registered NOW at %.1f deg of gyro yaw (the robot must be pointing at the goal, standing still)", (double)g_goalYaw);
      else say("WARN", "goal heading not registered: the gyro is not calibrated/usable");
    } else if (t[1] && !strcmp(t[1], "clear")) { g_goalRefSet = false; g_goalWaitNoted = false; g_goalAutoOff = true; say("INFO", "goal heading cleared; it stays unregistered until ':goal set' (no shot without it)"); }
    else if (t[1]) say("WARN", "usage: :goal [set|clear]");
    else if (!g_goalRefSet) say("INFO", "goal heading not registered (registers %u ms after power-on; ':goal set' does it now)", (unsigned)c.goalRegisterMs);
    else if (!goalValid()) say("WARN", "goal heading registered but INVALID: the gyro reference was reset (':zero', recalibration or an IMU outage). Point the robot at the goal and use ':goal set'");
    else say("INFO", "goal heading %.1f deg (gyro yaw), current yaw %.1f, error %.1f deg; aim window +-%.1f deg (goal %u mm wide, %u mm away, ball %u mm); fires at half of it",
             (double)g_goalYaw, (double)g_imu.yaw, (double)angDiffDeg(g_goalYaw, g_imu.yaw), (double)shootAimTolDeg(c), (unsigned)c.goalWidthMm, (unsigned)c.goalDistMm, (unsigned)c.ballDiaMm);
  }
  else if (!strcmp(cmd, "zero")) { imuZeroYaw(); g_refYaw = 0; say("INFO", "heading reference zeroed"); }
  else if (!strcmp(cmd, "gcal")) {
    if (!motorsAllZero()) say("WARN", "gyro calibration refused: motors are not stopped");
    else if (!g_imu.present) say("WARN", "gyro calibration impossible: IMU not present");
    else { imuStartCalibration(); say("INFO", "gyro bias calibration started - keep the robot completely still (~2 s)"); }
  }
  else if (!strcmp(cmd, "mtest")) { int w = t[1] ? wheelByName(t[1]) : -1; if (!navMotorTestArm(c, w, now)) say("WARN", "usage: :mtest <FL|FR|BL|BR>"); }
  else if (!strcmp(cmd, "dtest")) navMotorTestArm(c, 0, now, true);
  else if (!strcmp(cmd, "confirm")) navMotorTestConfirm(now);
  else if (!strcmp(cmd, "mok")) { int w = t[1] ? wheelByName(t[1]) : -1; if (w < 0) say("WARN", "usage: :mok <wheel>"); else { c.motorVerified |= (uint8_t)(1u << w); say("INFO", "%s direction recorded as operator-confirmed (':save' to keep). Rotation was never measured by the robot.", WHEEL_PINS[w].name); } }
  else if (!strcmp(cmd, "mrev")) { int w = t[1] ? wheelByName(t[1]) : -1; if (w < 0) say("WARN", "usage: :mrev <wheel>"); else { c.reverse[w] ^= 1; c.motorVerified &= (uint8_t)~(1u << w); say("INFO", "%s reversal now %d - repeat :mtest %s to check", WHEEL_PINS[w].name, c.reverse[w], WHEEL_PINS[w].name); } }
  else if (!strcmp(cmd, "recover")) cmdRecover(c, now);
  else if (!strcmp(cmd, "diag")) { c.detailMs = (t[1] && !strcmp(t[1], "on")) ? 100 : 0; say("INFO", "detailed per-sensor log rows %s (%u ms)", c.detailMs ? "ON" : "OFF", (unsigned)c.detailMs); }
  else if (!strcmp(cmd, "xsel")) {
    if (t[1] && !strcmp(t[1], "ring")) { g_exportSel = -1; say("LOG", "export source: flight-recorder ring"); }
    else { float v; if (parseF(t[1], &v) && v >= 0 && v < 20) { g_exportSel = (int8_t)v; say("LOG", "export source: file #%d (see 'g' for the list, 0 = newest)", (int)v); } else say("WARN", "usage: :xsel <ring|file#>"); }
  }
  else if (!strcmp(cmd, "fsformat")) {
    if (t[1] && !strcmp(t[1], "CONFIRM")) {
      navStop("filesystem format");
      say("LOG", "formatting LittleFS (explicit operator request; motors stopped; may take several seconds)...");
      outFlushBlocking(500);
      esp_task_wdt_delete(NULL);                     // a long erase must not trip the watchdog
      bool ok = fsFormatExplicit();
      esp_task_wdt_add(NULL);
      g_lastLoopUs = micros();                       // the stall was intentional: not a LOOP_STALL fault
      if (ok) logInit(c);
      say("LOG", "format %s", ok ? "OK, logging re-enabled" : "FAILED");
    }
    else say("WARN", "this ERASES the log partition. Send ':fsformat CONFIRM' to proceed");
  }
  else if (!strcmp(cmd, "logdel")) {
    if (t[1] && !strcmp(t[1], "CONFIRM") && g_fsRwOk && fsLock(100)) {
      static LogFile lf[LOG_LIST_MAX]; uint8_t k = logList(lf);
      for (uint8_t i = 0; i < k; i++) if (strcmp(lf[i].name, g_curPath) != 0) LittleFS.remove(lf[i].name);
      fsUnlock(); say("LOG", "deleted %u old log file(s) (current file kept)", (unsigned)k);
    } else say("WARN", "this deletes old log files. Send ':logdel CONFIRM'");
  }
  else say("WARN", "unknown word command '%s' (':help')", cmd);
}

static void cliSingle(char ch, uint32_t now) {
  Config& c = g_cfg;
  switch (ch) {
    case 'f': case 'b': case 'l': case 'r': case 'q': case 'e': navManual(c, ch, now); break;
    case 's': navStop("operator stop 's'"); say("SAFETY", "stopped; autonomy disabled"); break;
    case '!': navStop("EMERGENCY STOP"); say("SAFETY", "EMERGENCY STOP"); break;
    case 'i': printIrSnapshot(c); break;
    case 'm': printImu(c, now); break;
    case 't': cmdTestMode(); break;
    case 'a': if (navEnableAuto(c, now)) say("NAV", "autonomous ball seeking ENABLED. 's' or any manual key stops it."); break;
    case 'h': printHealth(c, now, "INFO"); break;
    case 'I': g_streamIr = !g_streamIr; g_streamLastMs = now; say("INFO", "live IR view %s (every %u ms)", g_streamIr ? "ON" : "OFF", (unsigned)c.streamMs); break;
    case 'd': printNavStatus(c, now); break;
    case 'g': logStatusPrint(c); break;
    case 'x': if (!exportStart(g_exportSel)) say("WARN", "export unavailable (busy, no such file, or no filesystem); ':xsel ring' always works"); break;
    case '?': printHelp(); break;
    default: sayRL(2, 2000, "WARN", "ignored unknown input 0x%02X ('?' for help)", (unsigned)(uint8_t)ch); break;
  }
}

static void cliPoll(uint32_t now) {
  if (g_lineMode && elapsedMs(now, g_lineStartMs) > 4000) { g_lineMode = false; say("WARN", "unterminated ':' line abandoned"); }
  int budget = 64;
  while (Serial.available() > 0 && budget-- > 0) {
    int v = Serial.read();
    if (v < 0) break;
    uint8_t ch = (uint8_t)v;
    if (ch == '!') { cliSingle('!', now); g_lineMode = false; continue; }   // works even inside a ':' line
    if (ch == 0x03 || ch == 0x1B) { g_lineMode = false; continue; }          // Ctrl-C / ESC cancels the line
    if (ch == '\r' || ch == '\n') {
      if (g_lineMode) {
        g_line[g_lineLen] = 0;
        if (g_lineBad || g_lineOvf) say("WARN", "line rejected (%s)", g_lineOvf ? "too long" : "invalid characters");
        else cliExecLine(g_line, now);
        g_lineMode = false;
      }
      continue;
    }
    if (g_lineMode) {
      if (ch < 0x20 || ch > 0x7E) { g_lineBad = true; continue; }
      if (g_lineLen < sizeof(g_line) - 1) g_line[g_lineLen++] = (char)ch; else g_lineOvf = true;
      continue;
    }
    if (ch == ':') { g_lineMode = true; g_lineBad = g_lineOvf = false; g_lineLen = 0; g_lineStartMs = now; continue; }
    if (ch == ' ' || ch == '\t') continue;
    cliSingle((char)ch, now);
  }
}

static void streamService(const Config& c, uint32_t now) {
  if (g_streamIr && elapsedMs(now, g_streamLastMs) >= c.streamMs) { g_streamLastMs = now; printIrStream(c); }
}
