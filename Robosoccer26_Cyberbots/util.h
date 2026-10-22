// util.h - math helpers, timing helpers, CRC, and the non-blocking Serial output queue.
#pragma once
#include "compat.h"

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

// ---- CRC32 (IEEE, bitwise; used for the EEPROM configuration) ---------------------
static inline uint32_t crc32Update(uint32_t crc, const uint8_t* d, size_t n) {
  crc = ~crc;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
  }
  return ~crc;
}

// ---- Non-blocking Serial output queue --------------------------------------------
// All human-readable output goes through this small ring buffer and is drained by outPump() only as fast as
// the UART accepts bytes, so printing can never stall the control loop (the AVR's own 64-byte TX buffer would block
// when full). If the queue is full, whole lines are dropped and counted (never partial lines).
#define OUTQ_SIZE 512
#define OUT_LINE_MAX 200
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
    Serial.write((uint8_t)g_outq[g_outTail]);
    g_outTail = (uint16_t)((g_outTail + 1) % OUTQ_SIZE);
    room--;
  }
}

// Paced reports (see report.h): a long report (health, help, snapshots) is longer than the 512-byte queue. While a report runs, lines that were already
// printed by an earlier pass are skipped and the first line that does not fit stops the output; reportService() runs the report again once the queue has drained.
static bool g_repActive = false, g_repOverflow = false;
static uint16_t g_repLine = 0, g_repSkip = 0;

// Enqueue one complete line (caller supplies the trailing '\n').
static void outPutLine(const char* s) {
  size_t n = strlen(s);
  if (g_repActive) {
    if (g_repLine++ < g_repSkip) return;                // printed by an earlier pass
    if (g_repOverflow) return;
    if (!g_bootPhase && outFree() < n) { g_repOverflow = true; return; }
    g_repSkip++;
  }
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

// "[TAG] sss.mmm message"  - tag and format are in flash (see compat.h); %S prints a flash string, %s a RAM string.
static void sayvP(PGM_P tag, PGM_P fmt, va_list ap) {
  char b[OUT_LINE_MAX];
  uint32_t ms = millis();
  int p = rsSnprintfP(b, sizeof(b), PSTR("[%S] %lu.%03lu "), tag, (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
  if (p < 0 || p >= (int)sizeof(b) - 2) return;
  rsFormat(b + p, sizeof(b) - (size_t)p - 2, fmt, RS_HAS_FLASH != 0, ap);
  size_t l = strlen(b);
  b[l] = '\n';
  b[l + 1] = 0;
  outPutLine(b);
}
static void sayP(PGM_P tag, PGM_P fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  sayvP(tag, fmt, ap);
  va_end(ap);
}
#define say(tag, fmt, ...) sayP(PSTR(tag), PSTR(fmt), ##__VA_ARGS__)
#define sayT(tagp, fmt, ...) sayP((tagp), PSTR(fmt), ##__VA_ARGS__)   // tag already a flash string

// Rate-limited variant: at most one line per `minGapMs` for a given key (0..7).
// Suppressed repeats are counted and reported with the next line that does get through.
static uint32_t g_rlLast[8];
static uint16_t g_rlSuppressed[8];
static bool g_rlUsed[8];
static bool sayRLgate(uint8_t key, uint32_t minGapMs, uint16_t* suppressed) {
  key &= 7;
  uint32_t now = millis();
  if (g_rlUsed[key] && elapsedMs(now, g_rlLast[key]) < minGapMs) {
    if (g_rlSuppressed[key] < 65535) g_rlSuppressed[key]++;
    return false;
  }
  *suppressed = g_rlSuppressed[key];
  g_rlUsed[key] = true;
  g_rlLast[key] = now;
  g_rlSuppressed[key] = 0;
  return true;
}
#define sayRL(key, gap, tag, fmt, ...) do { uint16_t _sup; if (sayRLgate((key), (gap), &_sup)) { say(tag, fmt, ##__VA_ARGS__); if (_sup) say("INFO", "(+%u repeats suppressed)", (unsigned)_sup); } } while (0)
