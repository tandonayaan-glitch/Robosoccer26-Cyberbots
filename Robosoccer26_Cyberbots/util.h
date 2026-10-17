// util.h - math helpers, timing helpers, CRC, and the non-blocking Serial output queue.
#pragma once
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define UL(x) ((unsigned long)(x))

static const float RAD2DEG = 57.29577951f;
static const float DEG2RAD = 0.017453292f;

// ---- math ------------------------------------------------------------------------
// NaN-safe clamp: NaN is treated as 0 (then clamped into range).
static inline float clampf(float v, float lo, float hi) {
  if (isnan(v)) v = 0.0f;
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline float absf(float v) { return v < 0 ? -v : v; }

// Wrap to (-180, 180]. Non-finite input returns 0 so it can never produce a huge command.
static inline float wrapDeg180(float a) {
  if (!isfinite(a)) return 0.0f;
  a = fmodf(a, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  else if (a <= -180.0f) a += 360.0f;
  return a;
}
// Wrap to [0, 360).
static inline float wrapDeg360(float a) {
  if (!isfinite(a)) return 0.0f;
  a = fmodf(a, 360.0f);
  if (a < 0.0f) a += 360.0f;
  if (a >= 360.0f) a = 0.0f;
  return a;
}
// Smallest signed difference a-b on the circle, (-180, 180]. 359 vs 1 -> -2, not 358.
static inline float angDiffDeg(float a, float b) { return wrapDeg180(a - b); }

static inline int popcount16(uint16_t v) {
  int n = 0;
  while (v) { n += v & 1; v >>= 1; }
  return n;
}

// ---- timing (rollover safe: always subtract unsigned) ------------------------------
static inline uint32_t elapsedMs(uint32_t now, uint32_t then) { return (uint32_t)(now - then); }
static inline bool deadlineReached(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

// ---- CRC32 (IEEE, bitwise; used for config + export integrity) ---------------------
static inline uint32_t crc32Update(uint32_t crc, const uint8_t* d, size_t n) {
  crc = ~crc;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
  }
  return ~crc;
}

// ---- Non-blocking Serial output queue --------------------------------------------
// All human-readable output goes through this queue and is drained by outPump() only as
// fast as the UART accepts bytes, so printing can never stall the control loop.
// If the queue is full, whole lines are dropped and counted (never partial lines).
#define OUTQ_SIZE 8192
static char g_outq[OUTQ_SIZE];
static volatile uint16_t g_outHead = 0;   // write index
static volatile uint16_t g_outTail = 0;   // read index
static uint32_t g_outDropped = 0;
static bool g_bootPhase = true;           // during setup(), motors are off: allow short blocking drain

static inline uint16_t outFree() {
  return (uint16_t)((g_outTail + OUTQ_SIZE - g_outHead - 1) % OUTQ_SIZE);
}
static inline uint16_t outUsed() { return (uint16_t)(OUTQ_SIZE - 1 - outFree()); }

static void outPump() {
  int room = Serial.availableForWrite();
  while (room > 0 && g_outTail != g_outHead) {
    uint16_t head = g_outHead, tail = g_outTail;
    uint16_t contiguous = (head >= tail) ? (uint16_t)(head - tail) : (uint16_t)(OUTQ_SIZE - tail);
    uint16_t n = contiguous < (uint16_t)room ? contiguous : (uint16_t)room;
    Serial.write((const uint8_t*)&g_outq[tail], n);
    g_outTail = (uint16_t)((tail + n) % OUTQ_SIZE);
    room -= n;
  }
}

// Enqueue one complete line (caller supplies the trailing '\n').
static void outPutLine(const char* s) {
  size_t n = strlen(s);
  if (n == 0 || n >= OUTQ_SIZE / 2) { g_outDropped++; return; }
  if (g_bootPhase) {
    // Boot only: motors are disabled, so a bounded wait for the UART is harmless.
    for (int guard = 0; guard < 3000 && outFree() < n; guard++) { outPump(); delay(1); }
  }
  if (outFree() < n) { g_outDropped++; return; }
  uint16_t h = g_outHead;
  for (size_t i = 0; i < n; i++) { g_outq[h] = s[i]; h = (uint16_t)((h + 1) % OUTQ_SIZE); }
  g_outHead = h;
}

static void outFlushBlocking(uint32_t maxMs) {
  uint32_t t0 = millis();
  while (g_outTail != g_outHead && elapsedMs(millis(), t0) < maxMs) { outPump(); delay(1); }
}

// "[TAG] sss.mmm message"
static void sayv(const char* tag, const char* fmt, va_list ap) {
  char b[288];
  uint32_t ms = millis();
  int p = snprintf(b, sizeof(b), "[%s] %lu.%03lu ", tag, UL(ms / 1000), UL(ms % 1000));
  if (p < 0 || p >= (int)sizeof(b) - 2) return;
  vsnprintf(b + p, sizeof(b) - (size_t)p - 2, fmt, ap);
  size_t l = strlen(b);
  b[l] = '\n';
  b[l + 1] = 0;
  outPutLine(b);
}
static void say(const char* tag, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
static void say(const char* tag, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  sayv(tag, fmt, ap);
  va_end(ap);
}

// Rate-limited variant: at most one line per `minGapMs` for a given key (0..31).
// Suppressed repeats are counted and reported with the next line that does get through.
static uint32_t g_rlLast[32];
static uint16_t g_rlSuppressed[32];
static bool g_rlUsed[32];
static void sayRL(uint8_t key, uint32_t minGapMs, const char* tag, const char* fmt, ...)
    __attribute__((format(printf, 4, 5)));
static void sayRL(uint8_t key, uint32_t minGapMs, const char* tag, const char* fmt, ...) {
  key &= 31;
  uint32_t now = millis();
  if (g_rlUsed[key] && elapsedMs(now, g_rlLast[key]) < minGapMs) {
    if (g_rlSuppressed[key] < 65535) g_rlSuppressed[key]++;
    return;
  }
  char msg[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  if (g_rlSuppressed[key]) say(tag, "%s (+%u repeats suppressed)", msg, (unsigned)g_rlSuppressed[key]);
  else say(tag, "%s", msg);
  g_rlUsed[key] = true;
  g_rlLast[key] = now;
  g_rlSuppressed[key] = 0;
}
