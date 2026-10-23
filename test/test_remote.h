// test_remote.h - TEST-ONLY C++ stand-in for a remote, used by the firmware simulation tests to drive the Mega's authentication (btauth.h) with a client state machine.
// It shares sha256_hmac.h with the firmware, so it only proves the firmware's own consistency; the independent check is the Python client (laptop/robot_protocol.py,
// standard-library hmac) run against the same firmware code by laptop/tests/.
#pragma once
#include "../Robosoccer26_Cyberbots/sha256_hmac.h"

// ============================================================ remote <-> Mega handshake ============================================================
typedef void (*AuthRandFn)(uint8_t* out, size_t n);       // real client: os.urandom; tests: a deterministic generator

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
