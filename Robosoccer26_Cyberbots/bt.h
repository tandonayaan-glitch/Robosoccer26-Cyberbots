// bt.h - remote control over the HC-05 Bluetooth Classic module on the Mega's Serial1 (TX1 = D18, RX1 = D19; see pins.h and docs/BLUETOOTH.md).
//
// The ESP32 remote connects to the HC-05 and sends single-character commands (case-insensitive). The robot answers each with one short line ("OK:W", "ERR:NOT_MANUAL", ...).
//
//   M  manual mode          U  autonomous mode (same checks as the 'a' key)     Q  stop (manual mode stays, autonomy ends)
//   W  forward   S  backward   A  turn left   D  turn right   Z  strafe left   C  strafe right     (manual mode only)
//   X  EMERGENCY STOP: everything stops at once and a LATCH blocks all movement and autonomy      R  clear the latch (the robot stays idle: send M or U next)
//   P  ping (answers PONG:<mode>)      ?  status line
//
// Every movement command is a LEASE: the robot keeps driving for btlease ms (400) and then stops by itself, so a held key (the remote repeats it every ~100 ms)
// drives continuously and a lost link stops the robot within btlease. Unknown bytes are ignored and counted; they never map to a motor action.
// The emergency stop has priority: if an 'X' is anywhere in the bytes waiting, it is handled first and the rest of them are discarded.
#pragma once
#include "nav.h"

static bool g_btLink = false;
static uint32_t g_btLastByteMs = 0, g_btBytes = 0, g_btCmds = 0, g_btUnknown = 0, g_btRefused = 0, g_btReplyDrops = 0, g_btUnkReplyMs = 0;
static char g_btLast = 0;

static PGM_P btModeName() {
  return g_estop ? PSTR("ESTOP") : (g_autoEnabled ? PSTR("AUTO") : (g_remoteManual ? PSTR("MANUAL") : PSTR("IDLE")));
}

// Never blocks: if the HC-05 transmit buffer is full the reply is dropped.
static void btReplyRam(const char* s) {
  size_t n = strlen(s);
  if (Serial1.availableForWrite() < (int)(n + 1)) { g_btReplyDrops++; return; }
  for (size_t i = 0; i < n; i++) Serial1.write((uint8_t)s[i]);
  Serial1.write((uint8_t)'\n');
}
static void btReply(char tag1, const char* a, char c) {          // "OK:W" / "ERR:NOT_MANUAL"
  char b[40];
  snprintf(b, sizeof(b), "%s:%c", a, c);
  (void)tag1;
  btReplyRam(b);
}
static void btReplyErr(const char* why) { char b[40]; snprintf(b, sizeof(b), "ERR:%s", why); btReplyRam(b); }

static void btBegin() {
  Serial1.begin(BT_BAUD);
}

static void btHandle(const Config& c, char ch, uint32_t now) {
  if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
  switch (ch) {
    case 0: case '\r': case '\n': case ' ': case '\t': return;               // line endings and padding are not commands
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
    case 'P': { char b[24]; snprintf(b, sizeof(b), "PONG:%S", btModeName()); btReplyRam(b); break; }
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

// Called first in every loop pass. Reads at most 32 bytes per pass.
static void btPoll(const Config& c, uint32_t now) {
  if (!c.btEnable) return;
  char buf[32];
  uint8_t n = 0;
  while (n < sizeof(buf) && Serial1.available() > 0) { int v = Serial1.read(); if (v < 0) break; buf[n++] = (char)v; }
  if (n) {
    g_btBytes += n;
    g_btLastByteMs = now ? now : 1u;
    if (!g_btLink) { g_btLink = true; say("INFO", "Bluetooth link: bytes arriving from the remote"); }
    bool hasX = false;
    for (uint8_t i = 0; i < n; i++) if (buf[i] == 'X' || buf[i] == 'x') hasX = true;
    if (hasX) btHandle(c, 'X', now);                       // emergency stop first; whatever else was waiting is stale
    else for (uint8_t i = 0; i < n; i++) btHandle(c, buf[i], now);
  }
  // Link supervision: the movement lease already stops the motors; this reports it and makes sure a manual move is over.
  if (g_btLink && elapsedMs(now, g_btLastByteMs) > c.btLinkMs) {
    g_btLink = false;
    say("WARN", "Bluetooth link LOST (nothing received for %u ms)%S", (unsigned)c.btLinkMs, g_navState == NAV_MANUAL ? PSTR(": stopping the manual move") : PSTR(""));
    if (g_navState == NAV_MANUAL) navStop(PSTR("remote link lost"));
  }
}

static void printBt(const Config& c, uint32_t now) {
  say("INFO", "Bluetooth: %S on Serial1 (TX1 D%u, RX1 D%u) at %lu baud | link %S | mode %S | e-stop latch %S",
      c.btEnable ? PSTR("enabled") : PSTR("DISABLED"), (unsigned)P_BT_TX, (unsigned)P_BT_RX, UL(BT_BAUD), g_btLink ? PSTR("UP") : PSTR("down (nothing received recently)"),
      btModeName(), g_estop ? PSTR("LATCHED") : PSTR("clear"));
  say("INFO", "  bytes %lu, commands %lu (last '%c'), refused %lu, unknown bytes %lu, replies dropped %lu, last byte %lu ms ago | movement lease %u ms, link timeout %u ms",
      UL(g_btBytes), UL(g_btCmds), g_btLast ? g_btLast : '-', UL(g_btRefused), UL(g_btUnknown), UL(g_btReplyDrops), UL(g_btLastByteMs ? elapsedMs(now, g_btLastByteMs) : 0), (unsigned)c.btLeaseMs, (unsigned)c.btLinkMs);
  say("INFO", "  commands: M manual, U autonomous, W S A D Z C drive (manual only), Q stop, X EMERGENCY STOP (latched), R clear the latch, P ping, ? status");
}
