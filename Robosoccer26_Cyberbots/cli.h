// cli.h - Serial command interface (115200 baud).
//   Single keys act immediately:  f b l r q e s i m t a h I d g x ?   and '!' (emergency stop).
//   Words start with ':' and end at CR/LF, e.g. ":set maxauto 150".
// Input is bounded (64 chars), non-printables are ignored, an unterminated ':' line is abandoned
// after 4 s so a stray ':' can never swallow a later 's'. Diagnostic commands never move motors.
#pragma once
#include "report.h"
#include "auxin.h"

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
  say("INFO", "i : IR snapshot   I : toggle live IR view   m : IMU/heading   d : nav/ball/PWM state   h : health report");
  say("INFO", "g : logger status   x : export selected log (:xsel ring|<n>)   t : test mode help   ? : this help");
  say("INFO", ":set <name> <v> | :get [name] | :save | :load | :defaults | :rev <FL|FR|BL|BR> <0|1> | :roller <X|O>");
  say("INFO", ":ch <n> [<en|pol|trust|bear|tilt> <v>] | :chreset <n> | :learn <deg> | :verify <n> | :irchar <n> [s] [period_us] | :aux [on|off|reset] | :cov | :irgate <off|auto|min max> | :irprofile <fast|balanced|safe> | :zero | :gcal | :diag <on|off>");
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

static void cmdTestMode() {
  navStop("test mode");
  g_testMode = true;
  say("TEST", "TEST MODE: motors are OFF and nothing moves by itself. Bench checklist:");
  say("TEST", " 1) wheels off the ground, robot secured, motor power switch reachable");
  say("TEST", " 2) per wheel: :mtest FL  then :confirm  -> watch rotation -> :mok FL (correct) or :mrev FL (wrong, then repeat)");
  say("TEST", " 3) :dtest then :confirm runs forward/back/strafe/rotate; or use f b l r q e in short bursts. ':roller O' if strafe is mirrored");
  say("TEST", " 4) IR: hold the ball at a receiver -> :verify <ch>; sweep the ball around -> :learn <deg>; ':ch <n> bear <deg>' to record bearings");
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

static void cmdCh(char* a, char* b, char* v) {
  float n, x;
  if (!parseF(a, &n) || n < 0 || n > 15) { say("WARN", "usage: :ch <0-15> [<en|pol|trust|bear|tilt> <value>]"); return; }
  ChCfg& h = g_cfg.ch[(int)n];
  if (!b) {                                   // inspection of one channel
    const IrChan& s = g_ir[(int)n];
    say("INFO", "C%d: %s | active-%s | trusted=%d | bearing %.0f%s tilt %.0f | status %s%s", (int)n, h.enabled ? "ENABLED" : "disabled",
        h.activeLow ? "LOW" : "HIGH", h.trusted, h.bearingDeg, h.bearingCal ? "" : " (UNCALIBRATED)", h.tiltDownDeg,
        CH_NAME[irChStatus(g_cfg, (uint8_t)n)], irTrusted(g_cfg, (uint8_t)n) ? "" : " [excluded from navigation]");
    if (h.enabled)
      say("INFO", "C%d: last window raw=%u state=%s edges=%u active_us=%lu period_us=%lu | smoothed active=%u hist8=%u/8 | windows=%lu detections=%lu offband=%lu overruns=%lu | verified=%d failedTests=%d",
          (int)n, (unsigned)s.raw, WS_NAME[s.winState], (unsigned)s.edges, UL(s.activeUs), UL(s.periodUs), (unsigned)s.active, popcount16(s.hist & 0xFF),
          UL(s.windows), UL(s.detCount), UL(s.offband), UL(s.overruns), h.verified, h.verifyFails);
    else say("INFO", "C%d is not scanned (a disabled channel is never reported as failed)", (int)n);
    return;
  }
  if (strcmp(b, "pol") == 0 && v && (strcmp(v, "low") == 0 || strcmp(v, "high") == 0)) { h.activeLow = (strcmp(v, "low") == 0); say("INFO", "C%d output polarity: active-%s", (int)n, h.activeLow ? "LOW" : "HIGH"); return; }
  if (!parseF(v, &x)) { say("WARN", "value must be a number"); return; }
  if (strcmp(b, "en") == 0) { h.enabled = x != 0; if (!h.enabled) { g_ir[(int)n].active = 0; g_ir[(int)n].detect = 0; } }
  else if (strcmp(b, "pol") == 0) h.activeLow = x != 0;
  else if (strcmp(b, "trust") == 0) h.trusted = x != 0;
  else if (strcmp(b, "bear") == 0) { if (x < 0 || x >= 360) { say("WARN", "bearing 0..359.9 (0=forward, + clockwise)"); return; } h.bearingDeg = x; h.bearingCal = 1; }
  else if (strcmp(b, "tilt") == 0) { if (x < 0 || x > 90) { say("WARN", "tilt 0..90"); return; } h.tiltDownDeg = x; }
  else { say("WARN", "unknown field '%s'", b); return; }
  say("INFO", "C%d en=%d pol=active-%s trust=%d bearing=%.0f%s tilt=%.0f", (int)n, h.enabled, h.activeLow ? "LOW" : "HIGH", h.trusted, h.bearingDeg, h.bearingCal ? "" : "(uncal)", h.tiltDownDeg);
}

static void cmdCov() {
  say("INFO", "coverage map (CSV): bearing_deg,seen_mask_hex,pct_C0..pct_C15   rows=%u", (unsigned)g_covN);
  for (uint8_t i = 0; i < g_covN; i++) {
    char l[140]; int p = snprintf(l, sizeof(l), "COV,%.0f,%04X", g_cov[i].deg, (unsigned)g_cov[i].mask);
    for (uint8_t k = 0; k < MUX_CHANNELS; k++) p += snprintf(l + p, sizeof(l) - (size_t)p, ",%u", (unsigned)g_cov[i].pct[k]);
    say("INFO", "%s", l);
  }
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
  else if (!strcmp(cmd, "ch")) cmdCh(t[1], t[2], t[3]);
  else if (!strcmp(cmd, "chreset")) {
    float n2;
    if (!parseF(t[1], &n2) || n2 < 0 || n2 > 15) say("WARN", "usage: :chreset <0-15>");
    else { int k = (int)n2; c.ch[k].verified = 0; c.ch[k].verifyFails = 0; memset(&g_ir[k], 0, sizeof(g_ir[k])); say("INFO", "C%d diagnostics + verification cleared", k); }
  }
  else if (!strcmp(cmd, "learn")) {
    float d;
    if (!parseF(t[1], &d)) say("WARN", "usage: :learn <ball bearing deg, 0=front, + clockwise> (hold the ball still for 2.5 s)");
    else { irProbeStart(1, 0, wrapDeg360(d), 2500, now); say("TEST", "learning at %.0f deg for 2.5 s - hold the ball still, motors stopped", wrapDeg360(d)); }
  }
  else if (!strcmp(cmd, "verify")) {
    float n2;
    if (!parseF(t[1], &n2) || n2 < 0 || n2 > 15 || !c.ch[(int)n2].enabled) say("WARN", "usage: :verify <enabled channel>  (touch the ball to that receiver's face)");
    else { irProbeStart(2, (uint8_t)n2, 0, 3000, now); say("TEST", "verifying C%d for 3 s - hold the ball against that receiver", (int)n2); }
  }
  else if (!strcmp(cmd, "irchar")) {
    float chn, sec = 3, ref = 0;
    if (!parseF(t[1], &chn) || chn < 0 || chn > 15 || !c.ch[(int)chn].enabled || (t[2] && (!parseF(t[2], &sec) || sec < 1 || sec > 20)) || (t[3] && (!parseF(t[3], &ref) || ref < 200 || ref > 3000)))
      say("WARN", "usage: :irchar <enabled channel> [seconds 1..20, default 3] [reference pulse period us 200..3000]");
    else if (g_autoEnabled || g_probe.mode) say("WARN", "irchar: autonomy must be off and no other probe running");
    else {
      irProbeStart(3, (uint8_t)chn, 0, (uint32_t)(sec * 1000.0f), now, (uint32_t)ref);
      say("TEST", "irchar C%d for %.0f s, reference period %s: robot stationary, source fixed in front of that receiver; the other receivers are not scanned meanwhile", (int)chn, (double)sec, ref > 0 ? "given" : "nominal 833 us");
    }
  }
  else if (!strcmp(cmd, "irgate")) {
    float lo, hi;
    static Config gate;
    gate = c;
    if (t[1] && !strcmp(t[1], "off")) { gate.irPeriodMinUs = gate.irPeriodMaxUs = 0; }
    else if (t[1] && !strcmp(t[1], "auto")) {
      if (g_lastPeriod.n < 8) { say("WARN", "irgate auto needs a measurement: run :verify <ch> or :learn <deg> with the ball first (last probe had %lu usable windows, need 8)", UL(g_lastPeriod.n)); return; }
      if (g_lastPeriod.mean < 625 || g_lastPeriod.mean > 1041)
        say("WARN", "irgate auto: learned period %lu us is more than 25%% from the ball documentation's 833 us - was only the ball in front of the receiver? (the ball's period is software-timed, so some spread is normal)", UL(g_lastPeriod.mean));
      if (g_lastPeriod.maxUs * 2 > g_lastPeriod.mean * 3)
        say("WARN", "irgate auto: the longest window period (%lu us) is over 1.5x the mean (%lu us): missed bursts are in the sample, and the +-35%% gate will reject windows that miss a burst. Check with ':irchar <ch> 5 <period>' (shadow gate lines) before relying on it.", UL(g_lastPeriod.maxUs), UL(g_lastPeriod.mean));
      uint32_t lo2 = (uint32_t)(g_lastPeriod.mean * 0.65f), hi2 = (uint32_t)(g_lastPeriod.mean * 1.35f);
      gate.irPeriodMinUs = (uint16_t)(lo2 < 100 ? 100 : lo2); gate.irPeriodMaxUs = (uint16_t)(hi2 > 20000 ? 20000 : hi2);
    }
    else if (parseF(t[1], &lo) && parseF(t[2], &hi) && lo >= 100 && hi > lo && hi <= 20000) { gate.irPeriodMinUs = (uint16_t)lo; gate.irPeriodMaxUs = (uint16_t)hi; }
    else { say("WARN", "usage: :irgate off | auto | <min_us 100..> <max_us ..20000>"); return; }
    char rep[64];
    static Config chk;
    chk = gate;
    if (cfgValidate(chk, rep, sizeof(rep)) != 0) { say("WARN", "gate rejected (%s)", rep); return; }
    c = gate;
    if (c.irPeriodMaxUs) say("INFO", "pulse-period gate ON: %u..%u us (:save to keep). Check with 'i' that the real ball still detects.", (unsigned)c.irPeriodMinUs, (unsigned)c.irPeriodMaxUs);
    else say("INFO", "pulse-period gate OFF");
  }
  else if (!strcmp(cmd, "irprofile")) {
    // Presets for the speed / false-acceptance trade-off of engine 2 (all simulated figures: docs/LATENCY_OPTIMIZATION.md).
    static const struct { const char* name; uint16_t span1, span2; const char* note; } PROF[] = {
      {"fast", 96, 30, "meets the latency targets; accepts shorter ball-like bursts than the legacy engine"},
      {"balanced", 140, 70, "default: burst rejection as good as the legacy engine (simulated), acquisition 30-45 % faster"},
      {"safe", 180, 100, "longer evidence: fewer accepted bursts, slower acquisition"}};
    int pick = -1;
    if (t[1]) for (int i = 0; i < 3; i++) if (!strcmp(t[1], PROF[i].name)) pick = i;
    if (pick >= 0) {
      c.acqSpan1Ms = PROF[pick].span1; c.acqSpan2Ms = PROF[pick].span2;
      say("INFO", "profile %s: acqspan1=%u acqspan2=%u ms (%s). ':save' to keep.", PROF[pick].name, (unsigned)c.acqSpan1Ms, (unsigned)c.acqSpan2Ms, PROF[pick].note);
    } else {
      say("INFO", "acqspan1=%u acqspan2=%u ms. Usage: :irprofile <fast|balanced|safe>", (unsigned)c.acqSpan1Ms, (unsigned)c.acqSpan2Ms);
      for (int i = 0; i < 3; i++) say("INFO", "  %-8s %3u / %3u ms  %s", PROF[i].name, (unsigned)PROF[i].span1, (unsigned)PROF[i].span2, PROF[i].note);
    }
  }
  else if (!strcmp(cmd, "cov")) cmdCov();
  else if (!strcmp(cmd, "aux")) cmdAux(t[1], now);
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
  else if (!strcmp(cmd, "diag")) { c.detailMs = (t[1] && !strcmp(t[1], "on")) ? 100 : 0; say("INFO", "detailed per-channel log rows %s (%u ms)", c.detailMs ? "ON" : "OFF", (unsigned)c.detailMs); }
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
