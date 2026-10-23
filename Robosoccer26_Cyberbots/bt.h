// bt.h - remote control and telemetry over the HC-05 Bluetooth Classic module on the Mega's Serial1 (TX1 = D18, RX1 = D19; see pins.h and docs/BLUETOOTH.md).
//
// FRAMING. The link is line oriented (the laptop remote sends "<command>\n"; a terminal set to "Both NL & CR" or "Newline" does the same):
//   * a line of ONE character is a single-key command (case-insensitive):
//       M manual mode   U autonomous (attack) mode   Q stop (manual stays)   W S A D Z C drive (manual only)   X EMERGENCY STOP   R clear the e-stop latch   P ping   ? status line
//   * a line of several characters is a WORD command that goes through the same parser as the USB console (cliExecLine):
//       defend attack manual stop status sensors gyro calibrate motors help ...   (docs/BLUETOOTH.md)
//   * a line with no terminator is completed after 25 ms of silence, so "W" without a newline still works.
//   * lines longer than 24 characters are rejected (ERR:TOO_LONG); unknown lines are rejected (ERR:UNKNOWN) and never move anything.
//   * an 'X' anywhere in the bytes received is an emergency stop at once; everything else waiting is discarded.
// Replies: "OK:<cmd>" / "ERR:<reason>" lines. Telemetry frames (telemetry.h) are JSON lines starting with '{'.
//
// Every movement command is a LEASE: the robot keeps driving for btlease ms (400) and then stops by itself, so a held key (the remote repeats it every ~100 ms)
// drives continuously and a lost link stops the robot within btlease. Nothing here blocks: output goes through a small queue and is dropped when it is full.
#pragma once
#include "nav.h"
#include "diag.h"

static void cliExecLine(char* line, uint32_t now);
static void teleSendNow(const Config& c, uint32_t now, char kind);

static bool g_btLink = false;
static uint32_t g_btLastByteMs = 0, g_btBytes = 0, g_btCmds = 0, g_btUnknown = 0, g_btRefused = 0, g_btReplyDrops = 0, g_btUnkReplyMs = 0;
static char g_btLast = 0;

static PGM_P btModeName() {
  return g_estop ? PSTR("ESTOP") : (g_defendEnabled ? PSTR("DEFEND") : (g_autoEnabled ? PSTR("AUTO") : (g_remoteManual ? PSTR("MANUAL") : PSTR("IDLE"))));
}

// Transmit queue for the HC-05. Serial1's own buffer is only 64 bytes, so replies and telemetry frames wait here and are fed to the UART as it drains.
// Nothing ever waits for the UART: a put that does not fit completely is refused (and counted), a half frame is never sent.
#define BT_TXQ_SIZE 400
static char g_btTx[BT_TXQ_SIZE];
static uint16_t g_btTxHead = 0, g_btTxTail = 0;
static inline uint16_t btTxUsed() { return (uint16_t)((g_btTxHead + BT_TXQ_SIZE - g_btTxTail) % BT_TXQ_SIZE); }
static inline uint16_t btTxFree() { return (uint16_t)(BT_TXQ_SIZE - 1 - btTxUsed()); }
static bool btTxPut(const char* s, size_t n, bool newline) {
  if (n + (newline ? 1u : 0u) > btTxFree()) return false;
  for (size_t i = 0; i < n; i++) { g_btTx[g_btTxHead] = s[i]; g_btTxHead = (uint16_t)((g_btTxHead + 1) % BT_TXQ_SIZE); }
  if (newline) { g_btTx[g_btTxHead] = '\n'; g_btTxHead = (uint16_t)((g_btTxHead + 1) % BT_TXQ_SIZE); }
  return true;
}
static void btTxPump() {
  int room = Serial1.availableForWrite();
  while (room-- > 0 && g_btTxTail != g_btTxHead) { Serial1.write((uint8_t)g_btTx[g_btTxTail]); g_btTxTail = (uint16_t)((g_btTxTail + 1) % BT_TXQ_SIZE); }
}
static void btReplyRam(const char* s) { if (!btTxPut(s, strlen(s), true)) g_btReplyDrops++; }
static void btReply(char tag1, const char* a, char c) {          // "OK:W" / "ERR:NOT_MANUAL"
  char b[40];
  snprintf(b, sizeof(b), "%s:%c", a, c);
  (void)tag1;
  btReplyRam(b);
}
static void btReplyErr(const char* why) { char b[40]; snprintf(b, sizeof(b), "ERR:%s", why); btReplyRam(b); }

#include "btauth.h"          // challenge-response authentication of the remote (every control command below is gated by btAuthOk())

static uint32_t btBaudNow(const Config& c) { return (uint32_t)c.btBaud100 * 100u; }
static void btBegin(const Config& c) {
  Serial1.begin(btBaudNow(c));
}

static void btHandle(const Config& c, char ch, uint32_t now) {
  if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
  if (ch == 0 || ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') return;   // line endings and padding are not commands
  if (ch != 'X' && ch != 'P') {                                            // everything except the emergency stop and the ping needs an authenticated session
    if (!btAuthOk()) { btAuthRefuse(now); g_btRefused++; return; }
    authTouch(now);
  }
  switch (ch) {
    case 'X':
      navEstop(PSTR("EMERGENCY STOP (remote)"));
      say("SAFETY", "REMOTE EMERGENCY STOP: latched. 'R' (remote) or ':recover' clears it.");
      btReply(0, "OK", 'X');
      break;
    case 'R':
      if (!g_estop) { btReply(0, "OK", 'R'); break; }
      if (drivetrainFault()) { btReplyErr("FAULT"); g_btRefused++; break; }
      g_estop = false; g_remoteManual = false;
      recAdd(REC_ESTOP, 0);
      say("SAFETY", "emergency-stop latch CLEARED by the remote. The robot is idle: send M (manual) or U (autonomous).");
      btReply(0, "OK", 'R');
      break;
    case 'M':
      if (g_estop) { btReplyErr("ESTOP"); g_btRefused++; break; }
      navStop(PSTR("remote: manual mode"));
      g_remoteManual = true;
      recAdd(REC_MODE, 1);
      btReply(0, "OK", 'M');
      break;
    case 'U':
      if (g_estop) { btReplyErr("ESTOP"); g_btRefused++; break; }
      g_remoteManual = false;
      if (navEnableAuto(c, now)) { recAdd(REC_MODE, 2); btReply(0, "OK", 'U'); }
      else { btReplyErr("REFUSED"); g_btRefused++; }
      break;
    case 'Q': {
      bool was = g_autoEnabled || g_remoteManual;
      navStop(PSTR("remote: stop (Q)"));
      g_remoteManual = was && !g_estop;                   // a stop ends autonomy but keeps manual mode; it never clears the emergency-stop latch
      btReply(0, "OK", 'Q');
      break;
    }
    case 'W': case 'S': case 'A': case 'D': case 'Z': case 'C': {
      if (g_estop) { btReplyErr("ESTOP"); g_btRefused++; break; }
      if (!g_remoteManual || g_autoEnabled) { btReplyErr("NOT_MANUAL"); g_btRefused++; break; }     // autonomy and manual driving never share the motors
      char key = ch == 'W' ? 'f' : ch == 'S' ? 'b' : ch == 'A' ? 'q' : ch == 'D' ? 'e' : ch == 'Z' ? 'l' : 'r';
      navManual(c, key, now, c.btLeaseMs);
      btReply(0, "OK", ch);
      break;
    }
    case 'P': { char b[24]; if (btAuthOk()) { authTouch(now); snprintf(b, sizeof(b), "PONG:%S", btModeName()); } else snprintf(b, sizeof(b), "PONG:AUTH"); btReplyRam(b); break; }
    case '?': {
      char b[48];
      snprintf(b, sizeof(b), "ST:%S,ESTOP=%u,BALL=%u,COND=%u", btModeName(), g_estop ? 1u : 0u, g_ball.valid ? 1u : 0u, (unsigned)g_cond);
      btReplyRam(b);
      break;
    }
    default:
      g_btUnknown++;
      if (elapsedMs(now, g_btUnkReplyMs) >= 1000) { g_btUnkReplyMs = now; btReplyErr("UNKNOWN"); }
      return;
  }
  g_btCmds++;
  g_btLast = ch;
}

// ---- line assembly ----
static char g_btLine[84];           // a normal command line is <= 24 characters; the AUTH RESP line (75) is the only long one
static uint8_t g_btLineLen = 0;
static bool g_btLineOvf = false;
static uint32_t g_btLineLastMs = 0, g_btLines = 0, g_btTooLong = 0;

static bool cliIsWord(const char* w) {
  return STREQ(w, "defend") || STREQ(w, "attack") || STREQ(w, "manual") || STREQ(w, "stop") || STREQ(w, "status") || STREQ(w, "sensors") || STREQ(w, "gyro")
      || STREQ(w, "calibrate") || STREQ(w, "help") || STREQ(w, "motors") || STREQ(w, "tele") || STREQ(w, "idle");
}

static void btFinishLine(const Config& c, uint32_t now) {
  uint8_t n = g_btLineLen;
  bool ovf = g_btLineOvf;
  g_btLineLen = 0; g_btLineOvf = false;
  if (ovf) { g_btTooLong++; g_btRefused++; btReplyErr("TOO_LONG"); return; }
  while (n && (g_btLine[n - 1] == ' ' || g_btLine[n - 1] == '\t')) n--;
  uint8_t s0 = 0;
  while (s0 < n && (g_btLine[s0] == ' ' || g_btLine[s0] == '\t')) s0++;
  if (s0 >= n) return;
  g_btLines++;
  if (n - s0 == 1) { btHandle(c, g_btLine[s0], now); return; }
  bool isAuth = n - s0 >= 4 && (g_btLine[s0] | 0x20) == 'a' && (g_btLine[s0 + 1] | 0x20) == 'u' && (g_btLine[s0 + 2] | 0x20) == 't' && (g_btLine[s0 + 3] | 0x20) == 'h' && (n - s0 == 4 || g_btLine[s0 + 4] == ' ');
  if (isAuth) {                                                            // never reaches the command table, never executed as keys
    char a[84]; uint8_t ka = 0;
    memset(a, 0, sizeof(a));
    for (uint8_t i = (uint8_t)(s0 + 4); i < n && ka < sizeof(a) - 1; i++) a[ka++] = (g_btLine[i] >= 'A' && g_btLine[i] <= 'Z') ? (char)(g_btLine[i] - 'A' + 'a') : g_btLine[i];
    a[ka] = 0;
    authLine(a, now);
    memset(a, 0, sizeof(a));
    return;
  }
  if (n - s0 > 24) { g_btTooLong++; g_btRefused++; btReplyErr("TOO_LONG"); return; }
  char w[26]; uint8_t k = 0;
  for (uint8_t i = s0; i < n && k < sizeof(w) - 1; i++) w[k++] = (g_btLine[i] >= 'A' && g_btLine[i] <= 'Z') ? (char)(g_btLine[i] - 'A' + 'a') : g_btLine[i];
  w[k] = 0;
  char first[16]; uint8_t f = 0;
  while (f < sizeof(first) - 1 && w[f] && w[f] != ' ') { first[f] = w[f]; f++; }
  first[f] = 0;
  if (!cliIsWord(first)) {
    g_btUnknown++;
    if (elapsedMs(now, g_btUnkReplyMs) >= 1000) { g_btUnkReplyMs = now; btReplyErr("UNKNOWN"); }
    return;
  }
  if (!btAuthOk()) { btAuthRefuse(now); g_btRefused++; return; }          // words (modes, calibration, settings, ...) need an authenticated session
  authTouch(now);
  if (g_estop && !(STREQ(first, "status") || STREQ(first, "sensors") || STREQ(first, "gyro") || STREQ(first, "stop") || STREQ(first, "help") || STREQ(first, "motors") || STREQ(first, "tele"))) {
    btReplyErr("ESTOP"); g_btRefused++; return;                       // the latch blocks every movement-capable word
  }
  cliExecLine(w, now);                                                // the SAME parser as the USB console
  char b[32];
  bool ok = true;
  if (STREQ(first, "defend")) ok = g_defendEnabled;
  else if (STREQ(first, "attack")) ok = g_autoEnabled;
  else if (STREQ(first, "manual")) ok = g_remoteManual;
  if (!ok) { btReplyErr("REFUSED"); g_btRefused++; return; }
  if (STREQ(first, "status")) teleSendNow(c, now, 'c');
  else if (STREQ(first, "sensors")) { teleSendNow(c, now, 'r'); teleSendNow(c, now, 'c'); }
  else if (STREQ(first, "gyro")) teleSendNow(c, now, 'i');
  else if (STREQ(first, "motors")) teleSendNow(c, now, 'd');
  snprintf(b, sizeof(b), "OK:%s", first);
  for (uint8_t i = 3; b[i]; i++) if (b[i] >= 'a' && b[i] <= 'z') b[i] = (char)(b[i] - 'a' + 'A');
  btReplyRam(b);
  g_btCmds++;
}

// Called first in every loop pass. Reads at most 32 bytes per pass.
static void btPoll(const Config& c, uint32_t now) {
  btTxPump();
  if (!c.btEnable) return;
  uint8_t n = 0;
  while (n < 32 && Serial1.available() > 0) {
    int v = Serial1.read();
    if (v < 0) break;
    n++;
    char ch = (char)v;
    if (ch == 'X' || ch == 'x') {                                       // emergency stop first; whatever else was waiting is stale
      g_btLineLen = 0; g_btLineOvf = false;
      while (Serial1.available() > 0) { Serial1.read(); n = (uint8_t)(n < 255 ? n + 1 : n); }     // discard the backlog
      g_btBytes += n; g_btLastByteMs = now ? now : 1u; g_btLink = true;
      btHandle(c, 'X', now);
      return;
    }
    g_btBytes++;
    g_btLastByteMs = now ? now : 1u;
    g_btLineLastMs = now;
    if (!g_btLink) { g_btLink = true; say("INFO", "Bluetooth link: bytes arriving from the remote"); }
    if (ch == '\r' || ch == '\n') { if (g_btLineLen || g_btLineOvf) btFinishLine(c, now); continue; }
    if (ch < 0x20 || ch > 0x7E) { g_btUnknown++; continue; }          // control bytes are ignored and counted
    if (g_btLineLen < 82) g_btLine[g_btLineLen++] = ch; else g_btLineOvf = true;
  }
  if (g_btLineLen && elapsedMs(now, g_btLineLastMs) >= 25) btFinishLine(c, now);     // no terminator: a short silence completes the line
  authTick(now);
  // Link supervision: the movement lease already stops the motors; this reports it and makes sure a manual move is over.
  if (g_btLink && elapsedMs(now, g_btLastByteMs) > c.btLinkMs) {
    g_btLink = false;
    say("WARN", "Bluetooth link LOST (nothing received for %u ms)%S", (unsigned)c.btLinkMs, g_navState == NAV_MANUAL ? PSTR(": stopping the manual move") : PSTR(""));
    if (g_navState == NAV_MANUAL) navStop(PSTR("remote link lost"));
    authDeauth(PSTR("Bluetooth link lost"));
  }
}

static void printBt(const Config& c, uint32_t now) {
  say("INFO", "Bluetooth: %S on Serial1 (TX1 D%u, RX1 D%u) at %lu baud | link %S | mode %S | e-stop latch %S",
      c.btEnable ? PSTR("enabled") : PSTR("DISABLED"), (unsigned)P_BT_TX, (unsigned)P_BT_RX, UL(btBaudNow(c)), g_btLink ? PSTR("UP") : PSTR("down (nothing received recently)"),
      btModeName(), g_estop ? PSTR("LATCHED") : PSTR("clear"));
  say("INFO", "  bytes %lu, commands %lu (last '%c'), refused %lu, unknown bytes %lu, replies dropped %lu, last byte %lu ms ago | movement lease %u ms, link timeout %u ms",
      UL(g_btBytes), UL(g_btCmds), g_btLast ? g_btLast : '-', UL(g_btRefused), UL(g_btUnknown), UL(g_btReplyDrops), UL(g_btLastByteMs ? elapsedMs(now, g_btLastByteMs) : 0), (unsigned)c.btLeaseMs, (unsigned)c.btLinkMs);
  { char fp[9] = "--------"; if (g_auth.keySet) authFingerprint(fp);
    say("INFO", "  authentication: %S | key %S (fingerprint %s) | session %S | ok %lu, failed %lu (%u in a row), expired %lu, refused commands %lu%S",
        g_auth.enabled ? PSTR("ENFORCED") : PSTR("OFF (bench, until reset)"), g_auth.keySet ? PSTR("set") : PSTR("NOT SET"), fp, btAuthOk() ? PSTR("AUTHENTICATED") : PSTR("locked"),
        UL(g_auth.okCount), UL(g_auth.failCount), (unsigned)g_auth.fails, UL(g_auth.expiredCount), UL(g_auth.refusedCount), g_auth.lockUntilMs && !deadlineReached(now, g_auth.lockUntilMs) ? PSTR(" | THROTTLED") : PSTR("")); }
  say("INFO", "  commands: M manual, U autonomous, W S A D Z C drive (manual only), Q stop, X EMERGENCY STOP (latched), R clear the latch, P ping, ? status");
}

#include "telemetry.h"
