// auth_client.h - portable (no Arduino dependency, unit-tested on the host against the real Mega firmware code) logic of the ESP32 remote's two locks:
//
//  1. ConsoleAuth  - the USB console lock. The laptop must type `AUTH <passcode>` before any command is forwarded to the robot. Locked at every boot, `LOCK` re-locks,
//                    failed attempts are throttled, the emergency stop (`X` or `!` alone on a line) always works, the passcode is never echoed, forwarded or logged.
//  2. ClientAuth   - the ESP32 <-> Mega handshake (HMAC-SHA-256 challenge-response with the 32-byte shared key; protocol in Robosoccer26_Cyberbots/btauth.h).
//
// The passcode only proves that whoever is typing KNOWS it. It does not identify a laptop: anyone who learns the passcode can unlock the console from any computer.
#pragma once
#include "sha256_hmac.h"

// ============================================================ USB console lock ============================================================
enum ConAct : uint8_t {
  CA_NONE,            // blank line
  CA_FORWARD,         // unlocked and not a local command: forward `line` to the robot
  CA_ESTOP,           // send the emergency stop (works while locked, throttled and during the passcode prompt)
  CA_UNLOCKED,        // `esp32connect` accepted: connect and authenticate to the robot now
  CA_DISCONNECT,      // `esp32disconnect` / `LOCK`: end the robot session and lock
  CA_PROMPT,          // `esp32connect` without a passcode: the NEXT line is the passcode
  CA_REJECT_LOCKED,   // locked: nothing is forwarded
  CA_BAD_PASSCODE,    // wrong passcode (throttle started)
  CA_THROTTLED,       // too many failures: attempt ignored without checking
  CA_NOT_PROVISIONED, // no passcode was provisioned in secrets.h: the console can never be unlocked
  CA_HELP,            // `help [topic]` (also while locked); topic in ConsoleAuth.arg
  CA_VIEW,            // a local display command: switch to ConsoleAuth.view
  CA_LOCALSTATUS      // `esp32status`
};

struct ConsoleAuth {
  bool unlocked, provisioned, requirePass, awaitingPass;
  uint8_t fails, view;
  uint32_t blockedUntil, lastActivityMs, idleLockMs, failCount, awaitSinceMs;
  uint8_t passHash[32];   // SHA-256 of the passcode: the plain text is not kept in RAM after init
  char arg[24];           // argument of `help <topic>` (never a passcode)
};

static inline char authUp(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c; }
static bool authWordIs(const char* s, size_t n, const char* word) {       // case-insensitive, whole token
  size_t w = strlen(word);
  if (n != w) return false;
  for (size_t i = 0; i < n; i++) if (authUp(s[i]) != word[i]) return false;
  return true;
}

// requirePass = 0 is the convenience mode (secrets.h CONSOLE_REQUIRE_PASSCODE 0): `esp32connect` alone unlocks. Anyone at the USB port can then drive the robot (the
// robot-side key still stops anyone who is merely in Bluetooth range).
static void consoleInit(ConsoleAuth* c, const char* passcode, uint32_t idleLockMs, bool requirePass = true) {
  memset(c, 0, sizeof(*c));
  c->idleLockMs = idleLockMs;
  c->requirePass = requirePass;
  bool strong = passcode && passcode[0] && strncmp(passcode, "REPLACE", 7) != 0 && strlen(passcode) >= 8;
  c->provisioned = strong || !requirePass;
  if (strong) sha256((const uint8_t*)passcode, strlen(passcode), c->passHash);
}
static uint32_t consoleBackoffMs(uint8_t fails) {            // 1st and 2nd failure: 1 s; then 5, 15, 30, 60 s
  static const uint16_t t[6] = {1000, 1000, 5000, 15000, 30000, 60000};
  return t[fails > 6 ? 5 : (fails ? fails - 1 : 0)];
}
static bool consoleLockedOut(const ConsoleAuth* c, uint32_t now) { return c->blockedUntil && (int32_t)(now - c->blockedUntil) < 0; }

static ConAct consoleTryPass(ConsoleAuth* c, const char* p, size_t n, uint32_t now) {
  c->awaitingPass = false;
  if (!c->provisioned) return CA_NOT_PROVISIONED;
  if (consoleLockedOut(c, now)) return CA_THROTTLED;
  bool ok;
  if (!c->requirePass) ok = true;
  else { uint8_t h[32]; sha256((const uint8_t*)p, n, h); ok = ct_equal(h, c->passHash, 32); memset(h, 0, sizeof(h)); }
  if (ok) { c->unlocked = true; c->fails = 0; c->blockedUntil = 0; c->lastActivityMs = now ? now : 1u; return CA_UNLOCKED; }
  c->unlocked = false;                                                                          // a wrong attempt also re-locks
  if (c->fails < 250) c->fails++;
  c->failCount++;
  c->blockedUntil = now + consoleBackoffMs(c->fails);
  return CA_BAD_PASSCODE;
}

// Local display words: exactly one token, no arguments. They never reach the robot (the sketch asks the robot for the data they need).
static int consoleViewWord(const char* s, size_t n) {
  struct W { const char* w; uint8_t v; };
  static const W t[] = {{"ALL", 0}, {"BACK", 0}, {"IR", 1}, {"SENSORS", 1}, {"IMU", 2}, {"GYRO", 2}, {"MOTORS", 3}, {"MOTOR", 3}, {"BALL", 4}, {"SAFETY", 5}, {"STATUS", 5}, {"RAW", 6}};
  for (const W& e : t) if (authWordIs(s, n, e.w)) return e.v;
  return -1;
}

// One complete input line (no newline). Never touches the robot itself: the caller acts on the returned value.
static ConAct consoleLine(ConsoleAuth* c, const char* line, uint32_t now) {
  while (*line == ' ' || *line == '\t') line++;
  size_t len = strlen(line);
  while (len && (line[len - 1] == ' ' || line[len - 1] == '\t' || line[len - 1] == '\r')) len--;
  if (!len) return CA_NONE;
  if (len == 1 && (line[0] == '!' || authUp(line[0]) == 'X')) return CA_ESTOP;                 // the one thing that is always allowed (also during the passcode prompt)
  if (c->awaitingPass) {                                                                        // the previous line was `esp32connect`: this whole line is the passcode
    if (now - c->awaitSinceMs <= 30000) return consoleTryPass(c, line, len, now);              // (never interpreted as a command, never forwarded)
    c->awaitingPass = false;
  }
  size_t t1 = 0;
  while (t1 < len && line[t1] != ' ' && line[t1] != '\t') t1++;
  const char* rest = line + t1;
  while (rest < line + len && (*rest == ' ' || *rest == '\t')) rest++;
  size_t restLen = (size_t)(line + len - rest);
  if (authWordIs(line, t1, "ESP32CONNECT") || authWordIs(line, t1, "AUTH")) {                  // consumed here, whatever follows it
    if (!c->provisioned) return CA_NOT_PROVISIONED;
    if (consoleLockedOut(c, now)) return CA_THROTTLED;
    if (!c->requirePass) return consoleTryPass(c, rest, restLen, now);
    if (restLen == 0) { c->awaitingPass = true; c->awaitSinceMs = now ? now : 1u; return CA_PROMPT; }
    return consoleTryPass(c, rest, restLen, now);
  }
  if (authWordIs(line, len, "ESP32DISCONNECT") || authWordIs(line, len, "LOCK")) { c->unlocked = false; c->awaitingPass = false; return CA_DISCONNECT; }
  if (authWordIs(line, t1, "HELP")) {
    size_t n = restLen < sizeof(c->arg) - 1 ? restLen : sizeof(c->arg) - 1;
    memcpy(c->arg, rest, n); c->arg[n] = 0;
    return CA_HELP;
  }
  if (authWordIs(line, len, "ESP32STATUS")) return CA_LOCALSTATUS;
  if (!c->unlocked) return CA_REJECT_LOCKED;
  c->lastActivityMs = now ? now : 1u;
  int v = consoleViewWord(line, len);
  if (v >= 0) { c->view = (uint8_t)v; return CA_VIEW; }
  return CA_FORWARD;
}
// Call regularly. Returns true on the call that auto-locks the console after idleLockMs without input or button use (0 = never).
static bool consoleTick(ConsoleAuth* c, uint32_t now) {
  if (c->awaitingPass && now - c->awaitSinceMs > 30000) c->awaitingPass = false;
  if (c->unlocked && c->idleLockMs && now - c->lastActivityMs > c->idleLockMs) { c->unlocked = false; return true; }
  return false;
}
// Buttons: the emergency-stop button is never gated (caller); every other button goes through this.
static bool consoleButtonAllowed(ConsoleAuth* c, uint32_t now) {
  if (!c->unlocked) return false;
  c->lastActivityMs = now ? now : 1u;
  return true;
}

// ============================================================ ESP32 <-> Mega handshake ============================================================
typedef void (*AuthRandFn)(uint8_t* out, size_t n);       // ESP32: esp_fill_random(); tests: a deterministic generator

enum ClState : uint8_t { CL_IDLE, CL_WAIT_CH, CL_SEND_RESP, CL_WAIT_OK, CL_AUTHED };
struct ClientAuth {
  uint8_t key[32];
  bool keyOk, robotOpen;
  ClState st;
  uint8_t cn[16], sn[16];
  uint32_t stSinceMs, nextTryMs, lastRxMs, okCount, failCount;
  uint8_t tries;
};

static void clientInit(ClientAuth* a, const char* keyHex) {
  memset(a, 0, sizeof(*a));
  a->keyOk = keyHex && hexToBytes(keyHex, a->key, 32);
  if (a->keyOk) { uint8_t same = 1; for (uint8_t i = 1; i < 32; i++) if (a->key[i] != a->key[0]) same = 0; if (same) { a->keyOk = false; memset(a->key, 0, 32); } }
}
static void clientReset(ClientAuth* a) { a->st = CL_IDLE; a->stSinceMs = 0; a->nextTryMs = 0; a->tries = 0; a->robotOpen = false; memset(a->cn, 0, 16); memset(a->sn, 0, 16); }
static bool clientAuthed(const ClientAuth* a) { return a->st == CL_AUTHED; }

static void clientBackoff(ClientAuth* a, uint32_t now, uint32_t ms) { a->st = CL_IDLE; a->nextTryMs = now + ms; if (a->tries < 250) a->tries++; }

// Returns the length of the next line to send to the robot ("AUTH" or "AUTH RESP ...", without newline) or 0. `want` = the console is unlocked and the link is up.
static size_t clientWantLine(ClientAuth* a, bool want, uint32_t now, AuthRandFn rnd, char* out, size_t outN) {
  if (!want || !a->keyOk) { if (a->st != CL_IDLE) clientReset(a); return 0; }
  if (a->st == CL_IDLE) {
    if (a->nextTryMs && (int32_t)(now - a->nextTryMs) < 0) return 0;
    if (outN < 8) return 0;
    strcpy(out, "AUTH"); a->st = CL_WAIT_CH; a->stSinceMs = now ? now : 1u;
    return 4;
  }
  if (a->st == CL_SEND_RESP) {
    if (outN < 96) return 0;
    uint8_t tag[16];
    rnd(a->cn, 16);
    authTag(a->key, "E2M", a->sn, a->cn, tag);
    strcpy(out, "AUTH RESP ");
    bytesToHex(a->cn, 16, out + 10);
    out[42] = ' ';
    bytesToHex(tag, 16, out + 43);
    a->st = CL_WAIT_OK; a->stSinceMs = now ? now : 1u;
    return 75;
  }
  if ((a->st == CL_WAIT_CH || a->st == CL_WAIT_OK) && now - a->stSinceMs > 3000) { a->failCount++; clientBackoff(a, now, 2000); }   // no answer: try again later
  if (a->st == CL_AUTHED && a->lastRxMs && now - a->lastRxMs > 3000) { clientReset(a); a->nextTryMs = now; }                           // the robot went silent: its session is over
  return 0;
}

// Feed every line received from the robot. Returns true if it was an authentication line (the caller then does not treat it as telemetry or text).
static bool clientOnLine(ClientAuth* a, const char* line, uint32_t now) {
  if (strncmp(line, "AUTH ", 5) == 0) {
    a->lastRxMs = now ? now : 1u;
    if (strncmp(line, "AUTH CH ", 8) == 0) {
      if (a->st == CL_WAIT_CH && hexToBytes(line + 8, a->sn, 16)) a->st = CL_SEND_RESP;
      return true;
    }
    if (strncmp(line, "AUTH OK ", 8) == 0) {
      uint8_t tag[16], want[16];
      if (a->st == CL_WAIT_OK && hexToBytes(line + 8, tag, 16)) {
        authTag(a->key, "M2E", a->cn, a->sn, want);
        if (ct_equal(tag, want, 16)) { a->st = CL_AUTHED; a->okCount++; a->tries = 0; a->robotOpen = false; }
        else { a->failCount++; clientBackoff(a, now, 10000); }              // the robot did NOT prove it knows the key: do not trust it
      }
      return true;
    }
    if (strcmp(line, "AUTH OPEN") == 0) { a->robotOpen = true; clientBackoff(a, now, 5000); return true; }   // the robot is not enforcing authentication: never "authenticated"
    if (strcmp(line, "AUTH OUT") == 0) { a->st = CL_IDLE; return true; }
    return true;
  }
  if (strncmp(line, "ERR:AUTH", 8) == 0 || strncmp(line, "ERR:NO_KEY", 10) == 0) {
    a->lastRxMs = now ? now : 1u;
    if (strcmp(line, "ERR:AUTH") == 0) {                                      // plain refusal = "you have no session": stale during a handshake, session loss once authenticated
      if (a->st == CL_AUTHED) { a->st = CL_IDLE; a->nextTryMs = now; }
      return true;
    }
    if (strncmp(line, "ERR:AUTH_LOCKOUT", 16) == 0) clientBackoff(a, now, 8000);
    else if (strncmp(line, "ERR:AUTH_BUSY", 13) == 0) clientBackoff(a, now, 500);
    else if (strncmp(line, "ERR:NO_KEY", 10) == 0) clientBackoff(a, now, 15000);
    else { a->failCount++; clientBackoff(a, now, 3000); }                      // AUTH_FAIL / AUTH_EXPIRED / AUTH_NOCHAL / AUTH_SYNTAX
    return true;
  }
  if (a->st == CL_AUTHED && strncmp(line, "PONG:AUTH", 9) == 0) { clientReset(a); a->nextTryMs = now; return true; }   // the robot says it no longer has our session
  if (a->st == CL_AUTHED) a->lastRxMs = now ? now : 1u;       // any reply (PONG, OK:, telemetry) proves the robot still has our session
  return false;
}
