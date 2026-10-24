// btauth.h - authentication of the Bluetooth remote (the Windows laptop, directly through the HC-05) to the robot: mutual challenge-response with HMAC-SHA-256 and a shared 32-byte key.
//
//   laptop -> Mega  AUTH                        request (starts a handshake and ENDS any current session)
//   Mega -> laptop  AUTH CH <sn: 32 hex>        the Mega's challenge: 16 bytes, unique, single use, valid AUTH_CH_MS
//   laptop -> Mega  AUTH RESP <cn: 32 hex> <tag: 32 hex>      cn = the laptop's own 16-byte challenge, tag = HMAC(key, "E2M"|sn|cn)[0..16]
//   Mega -> laptop  AUTH OK <tag2: 32 hex>      tag2 = HMAC(key, "M2E"|cn|sn)[0..16]  (the Mega proves it knows the key too)
//   laptop -> Mega  AUTH OUT                    end the session (also ended by 2.5 s without a valid command, by the link timeout, by a new AUTH, or a reset)
//
// WHAT IT GIVES: nobody who merely pairs with the HC-05 (or knows its name / PIN) can drive the robot, change modes, calibrate, write settings or clear the emergency-stop
// latch; a recorded handshake cannot be replayed (the challenge is single use, time limited, and the proofs are bound to both nonces and to a direction label); the key
// never crosses the air; failed attempts are throttled.
// WHAT IT DOES NOT GIVE (docs/BLUETOOTH.md): the commands AFTER the handshake carry no per-message MAC and are not encrypted by this layer (only Bluetooth's own link
// encryption protects them); anyone holding the key can authenticate; the USB cable to the Mega is trusted (physical access); the key sits unencrypted in the Mega's EEPROM;
// the emergency stop `X` and the harmless ping `P` are deliberately accepted from anyone (a stop is always safe).
//
// The Mega has no hardware random number generator. Its challenge is HMAC(key, "NON" | boot counter | challenge counter | micros | millis) truncated to 16 bytes:
// unpredictable to anyone without the key, and NEVER repeated because the boot counter lives in the EEPROM and increases at every power-up (a repeated challenge is what
// would make a replay possible). The laptop client uses the operating system's secure random generator (Python os.urandom) for its own challenge.
#pragma once
#include "sha256_hmac.h"

static const uint32_t AUTH_CH_MS = 10000;            // a challenge is valid this long, once
static const uint32_t AUTH_IDLE_MS = 2500;           // the session ends after this long without a valid command (the remote pings every 500 ms)
static const uint32_t AUTH_MIN_GAP_MS = 300;         // at most one challenge per this time (each handshake costs ~30 ms of CPU on the ATmega)
static const int EEPROM_AUTH_ADDR = 4032;            // 64 bytes at the top of the EEPROM (the event records stop at 4032)
static const uint32_t AUTH_MAGIC = 0x48545541u;      // "AUTH"

struct AuthBlock { uint32_t magic; uint8_t key[32]; uint32_t bootCtr; uint32_t crc; };

struct AuthState {
  bool keySet;
  bool enabled;                    // runtime only, true at every boot. ':auth off' (USB) lets the bench work without a key until the next reset
  bool authed, chValid, chExpired;
  uint8_t key[32], sn[16];
  uint32_t bootCtr, chCtr, chIssuedMs, lastChMs, lastOkMs, lockUntilMs, authedAtMs, lastRefuseMs;
  uint8_t fails;
  uint32_t okCount, failCount, expiredCount, refusedCount, noChallengeCount;
};
static AuthState g_auth;

static uint32_t authBlockCrc(const AuthBlock& b) { return crc32Update(0, (const uint8_t*)&b, offsetof(AuthBlock, crc)); }
static void authStore(uint32_t bootCtr) {
  AuthBlock b;
  memset(&b, 0, sizeof(b));
  b.magic = AUTH_MAGIC; memcpy(b.key, g_auth.key, 32); b.bootCtr = bootCtr; b.crc = authBlockCrc(b);
  EEPROM.put(EEPROM_AUTH_ADDR, b);
  g_auth.bootCtr = bootCtr;
}
static void authWipeRam() { memset(g_auth.key, 0, 32); memset(g_auth.sn, 0, 16); g_auth.keySet = false; g_auth.authed = false; g_auth.chValid = false; }

// At boot: load the key, advance the persistent boot counter (so no challenge value is ever used twice), start unauthenticated and enforcing.
static void authBegin() {
  memset(&g_auth, 0, sizeof(g_auth));
#ifdef HOST_TEST
  g_auth.enabled = g_mockAuthEnforced;     // the host tests of unrelated features run with enforcement off; the authentication tests turn it on
#else
  g_auth.enabled = true;
#endif
  AuthBlock b;
  EEPROM.get(EEPROM_AUTH_ADDR, b);
  if (b.magic == AUTH_MAGIC && b.crc == authBlockCrc(b)) {
    memcpy(g_auth.key, b.key, 32);
    g_auth.keySet = true;
    authStore(b.bootCtr + 1u);
  }
  memset(&b, 0, sizeof(b));
}
static bool authKeyWeak(const uint8_t* k) {         // refuse an obviously unusable key (all one byte value)
  uint8_t same = 1;
  for (uint8_t i = 1; i < 32; i++) if (k[i] != k[0]) same = 0;
  return same != 0;
}
static bool authSetKey(const char* hex) {
  uint8_t k[32];
  if (!hexToBytes(hex, k, 32) || authKeyWeak(k)) { memset(k, 0, sizeof(k)); return false; }
  memcpy(g_auth.key, k, 32); memset(k, 0, sizeof(k));
  g_auth.keySet = true; g_auth.authed = false; g_auth.chValid = false; g_auth.fails = 0; g_auth.lockUntilMs = 0;
  authStore(g_auth.bootCtr + 1u);
  return true;
}
static void authClearKey() {
  AuthBlock b;
  memset(&b, 0xFF, sizeof(b));
  EEPROM.put(EEPROM_AUTH_ADDR, b);
  g_auth.enabled = true;
  authWipeRam();
}
// Short fingerprint to compare keys on the two boards without showing the key: first 4 bytes of HMAC(key, "FPR").
static void authFingerprint(char out[9]) {
  uint8_t full[32];
  hmac_sha256(g_auth.key, 32, (const uint8_t*)"FPR", 3, nullptr, 0, nullptr, 0, full);
  bytesToHex(full, 4, out);
  memset(full, 0, sizeof(full));
}

// May a control command be accepted now? (true when authenticated, or when enforcement was switched off on the bench). Without a key it is false: fail closed.
static bool btAuthOk() { return !g_auth.enabled || (g_auth.keySet && g_auth.authed); }

static void authDeauth(PGM_P why) {
  bool was = g_auth.authed;
  g_auth.authed = false; g_auth.chValid = false;
  if (was) {
    say("SAFETY", "Bluetooth session ended: %S. Control commands are refused until the remote authenticates again.", why);
    if (g_navState == NAV_MANUAL) navStop(PSTR("remote session ended"));      // a manual move stops; attack / defend carry on (they do not depend on the link)
  }
}
static void authTouch(uint32_t now) { if (g_auth.authed) g_auth.lastOkMs = now ? now : 1u; }

// Reply used for every refused control command; rate limited so a flood from an unauthenticated peer cannot make the robot flood the air.
static void btAuthRefuse(uint32_t now) {
  g_auth.refusedCount++;
  if (elapsedMs(now, g_auth.lastRefuseMs) >= 1000) { g_auth.lastRefuseMs = now ? now : 1u; btReplyErr(g_auth.keySet ? "AUTH" : "NO_KEY"); }
}

static void authTick(uint32_t now) {
  if (g_auth.chValid && elapsedMs(now, g_auth.chIssuedMs) > AUTH_CH_MS) { g_auth.chValid = false; g_auth.chExpired = true; g_auth.expiredCount++; }
  if (g_auth.authed && elapsedMs(now, g_auth.lastOkMs) > AUTH_IDLE_MS) authDeauth(PSTR("no valid command for 2.5 s"));
}

static void authMakeChallenge(uint32_t now) {
  uint8_t m[12], extra[4], full[32];
  g_auth.chCtr++;
  uint32_t us = micros();
  for (uint8_t i = 0; i < 4; i++) { m[i] = (uint8_t)(g_auth.bootCtr >> (8 * i)); m[4 + i] = (uint8_t)(g_auth.chCtr >> (8 * i)); m[8 + i] = (uint8_t)(us >> (8 * i)); }
  uint32_t ms = millis();
  for (uint8_t i = 0; i < 4; i++) extra[i] = (uint8_t)(ms >> (8 * i));                 // extra variation only: uniqueness comes from the persistent boot counter
  hmac_sha256(g_auth.key, 32, (const uint8_t*)"NON", 3, m, 12, extra, 4, full);
  memcpy(g_auth.sn, full, 16);
  g_auth.chValid = true; g_auth.chIssuedMs = now ? now : 1u; g_auth.lastChMs = g_auth.chIssuedMs;
  memset(full, 0, sizeof(full));
}

static void authFailed(uint32_t now) {
  g_auth.failCount++;
  if (g_auth.fails < 250) g_auth.fails++;
  if (g_auth.fails >= 3) {                                               // 3rd failure: 4 s; then 8, 16, 32, 60 s
    uint32_t wait = 1000u << (g_auth.fails - 1 > 6 ? 6 : g_auth.fails - 1);
    if (wait > 60000u) wait = 60000u;
    g_auth.lockUntilMs = now + wait;
  }
  btReplyErr("AUTH_FAIL");
  say("SAFETY", "Bluetooth authentication FAILED (%u in a row)%S", (unsigned)g_auth.fails, g_auth.lockUntilMs ? PSTR(": further attempts throttled") : PSTR(""));
}

// Handles one line that starts with AUTH (already upper/lower-cased by the caller; hex digits are accepted in either case). `rest` points after "AUTH".
static void authLine(const char* rest, uint32_t now) {
  while (*rest == ' ') rest++;
  if (!g_auth.enabled) { btReplyRam("AUTH OPEN"); return; }                // enforcement is off (bench): say so, never pretend to have authenticated
  if (!g_auth.keySet) { btReplyErr("NO_KEY"); return; }
  if (*rest == 0 || strncmp(rest, "req", 3) == 0 || strncmp(rest, "REQ", 3) == 0) {
    if (g_auth.lockUntilMs && !deadlineReached(now, g_auth.lockUntilMs)) { btReplyErr("AUTH_LOCKOUT"); return; }
    if (g_auth.lastChMs && elapsedMs(now, g_auth.lastChMs) < AUTH_MIN_GAP_MS) { btReplyErr("AUTH_BUSY"); return; }
    g_auth.authed = false;                                                   // a new handshake ends the old session first
    g_auth.chExpired = false;
    authMakeChallenge(now);
    char b[44]; strcpy(b, "AUTH CH ");
    bytesToHex(g_auth.sn, 16, b + 8);
    btReplyRam(b);
    return;
  }
  if (strncmp(rest, "out", 3) == 0 || strncmp(rest, "OUT", 3) == 0) { authDeauth(PSTR("ended by the remote")); btReplyRam("AUTH OUT"); return; }
  if (strncmp(rest, "resp ", 5) == 0 || strncmp(rest, "RESP ", 5) == 0) {
    rest += 5;
    if (g_auth.lockUntilMs && !deadlineReached(now, g_auth.lockUntilMs)) { g_auth.chValid = false; btReplyErr("AUTH_LOCKOUT"); return; }
    if (!g_auth.chValid) {                                                    // no outstanding challenge: an expired one, a replay or an out-of-order proof
      if (g_auth.chExpired) { g_auth.chExpired = false; btReplyErr("AUTH_EXPIRED"); } else { g_auth.noChallengeCount++; btReplyErr("AUTH_NOCHAL"); }
      return;
    }
    bool fresh = elapsedMs(now, g_auth.chIssuedMs) <= AUTH_CH_MS;
    g_auth.chValid = false;                                                  // SINGLE USE: consumed here whatever the outcome
    if (!fresh) { g_auth.expiredCount++; btReplyErr("AUTH_EXPIRED"); return; }
    uint8_t cn[16], tag[16], want[16];
    if (strlen(rest) != 65 || rest[32] != ' ' || !hexToBytes(rest + 33, tag, 16)) { authFailed(now); return; }
    char cnHex[33]; memcpy(cnHex, rest, 32); cnHex[32] = 0;
    if (!hexToBytes(cnHex, cn, 16)) { authFailed(now); return; }
    authTag(g_auth.key, "E2M", g_auth.sn, cn, want);
    bool ok = ct_equal(want, tag, 16);
    memset(want, 0, sizeof(want));
    if (!ok) { authFailed(now); return; }
    g_auth.authed = true; g_auth.authedAtMs = now ? now : 1u; g_auth.lastOkMs = g_auth.authedAtMs; g_auth.fails = 0; g_auth.lockUntilMs = 0; g_auth.okCount++;
    uint8_t back[16];
    authTag(g_auth.key, "M2E", cn, g_auth.sn, back);
    char b[44]; strcpy(b, "AUTH OK ");
    bytesToHex(back, 16, b + 8);
    btReplyRam(b);
    say("SAFETY", "Bluetooth remote authenticated. (This does NOT clear an emergency-stop latch.)");
    return;
  }
  btReplyErr("AUTH_SYNTAX");
}
