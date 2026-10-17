// logger.h - CSV logging, RAM flight recorder ("black box"), and Serial export.
//
// Data path:  producers (control loop) -> RAM line queue -> logPump() -> LittleFS file.
// logPump() runs in a low-priority writer task on the other core (on the host test build it is
// called explicitly), so a slow flash erase/GC can never be waited on by the motor loop. Records
// are batched (flushMs), files are size-capped and rotated within a fixed file budget, and a full
// or failing filesystem only disables persistence - it never stops the robot or the RAM recorder.
// Nothing here formats or erases the filesystem; only rotation deletes the OLDEST log of this
// firmware's own naming scheme (L<session>_<part>.csv) when the configured budget is reached.
#pragma once
#include "config.h"
#include "health.h"
#include "storage.h"
#include "motors.h"
#include "irscan.h"
#include "imu.h"

#define FW_VERSION "1.0.0"
#define LOG_SCHEMA 1
static const char* LOG_COLUMNS =
  "t_ms,sess,seq,type,cond,nav,irmask,actmask,ball,bear_deg,conf,imu,yaw_deg,herr_deg,gz_dps,"
  "vx,vy,w,pwm_fl,pwm_fr,pwm_bl,pwm_br,faults,scan_us,loop_us,note";

enum NavState : uint8_t {
  NAV_IDLE, NAV_MANUAL, NAV_SEEK, NAV_APPROACH, NAV_SEARCH, NAV_SEARCH_PAUSE, NAV_WAIT, NAV_TEST, NAV_SAFE_STOP
};
static const char* NAV_NAME[] = {"IDLE", "MANUAL", "SEEK", "APPROACH", "SEARCH", "SEARCH_PAUSE", "WAIT", "TEST", "SAFE_STOP"};
static NavState g_navState = NAV_IDLE;
static float g_hdgErr = NAN;               // degrees; NaN when heading hold is not active
static uint32_t g_loopMaxUs = 0;           // longest loop() iteration since the last snapshot
static uint32_t g_loopMaxEverUs = 0;

// ---- RAM line queue (single producer, single consumer) -----------------------------------
#define LOGQ_SIZE 6144
static char g_logq[LOGQ_SIZE];
static volatile uint16_t g_logHead = 0, g_logTail = 0;
static volatile uint32_t g_logDropped = 0;
static volatile uint32_t g_logDiscarded = 0;   // lines discarded because persistence is off (not an error)
static inline uint16_t logqUsed() { return (uint16_t)((g_logHead + LOGQ_SIZE - g_logTail) % LOGQ_SIZE); }
static inline uint16_t logqFree() { return (uint16_t)(LOGQ_SIZE - 1 - logqUsed()); }

static void logPutLine(const char* line) {     // line includes '\n'
  size_t n = strlen(line);
  if (n == 0 || n > 300 || logqFree() < n) { g_logDropped = g_logDropped + 1; g_stats.logDropped++; return; }
  uint16_t h = g_logHead;
  for (size_t i = 0; i < n; i++) { g_logq[h] = line[i]; h = (uint16_t)((h + 1) % LOGQ_SIZE); }
  __sync_synchronize();
  g_logHead = h;
}

// ---- writer state --------------------------------------------------------------------
static TaskHandle_t g_logTaskHandle = nullptr;   // set by setup() so the stack watermark can be reported
static uint32_t g_session = 0, g_seq = 0;
static uint16_t g_part = 0;
static uint32_t g_curBytes = 0;
static char g_curPath[28] = "";
static volatile bool g_logPersist = false;      // persistent logging currently enabled
static volatile uint8_t g_wrFailStreak = 0;
static volatile uint32_t g_fsFailTotal = 0;
static volatile bool g_fsWriteFault = false;
static char g_fsFaultWhy[40] = "";
static uint32_t g_wrLastFlushMs = 0, g_wrBackoffUntil = 0;
static uint32_t g_bytesWritten = 0;
static uint32_t g_filesCreated = 0, g_filesDeleted = 0;

static void logPathFor(uint32_t sess, uint16_t part, char* out, size_t n) {
  snprintf(out, n, "/L%05lu_%02u.csv", UL(sess % 100000), (unsigned)(part % 100));
}
static bool parseLogName(const char* nm, uint32_t* sess, uint16_t* part) {
  if (nm[0] == '/') nm++;
  if (nm[0] != 'L' || strlen(nm) < 12) return false;
  unsigned s = 0, p = 0;
  if (sscanf(nm, "L%5u_%2u.csv", &s, &p) != 2) return false;
  *sess = s; *part = (uint16_t)p;
  return true;
}

struct LogFile { uint32_t key; char name[24]; uint32_t size; };
#define LOG_LIST_MAX 24
static uint8_t logList(LogFile* out) {            // unsorted listing of our own log files
  uint8_t n = 0;
  File root = LittleFS.open("/");
  if (!root) return 0;
  File f = root.openNextFile();
  while (f && n < LOG_LIST_MAX) {
    uint32_t s; uint16_t p;
    const char* nm = f.name();
    if (parseLogName(nm, &s, &p)) {
      out[n].key = s * 100u + p;
      snprintf(out[n].name, sizeof(out[n].name), "/%s", nm[0] == '/' ? nm + 1 : nm);
      out[n].size = (uint32_t)f.size();
      n++;
    }
    f = root.openNextFile();
  }
  return n;
}

// Keep at most (maxFiles-1) old files so that creating one more stays inside the budget.
static void logEnforceBudget(const Config& c, const char* keepPath) {
  static LogFile lf[LOG_LIST_MAX];            // only the writer context calls this: keep it off the task stack
  uint8_t n = logList(lf);
  while (n >= c.maxFiles) {
    int oldest = -1;
    for (uint8_t i = 0; i < n; i++) {
      if (keepPath && strcmp(lf[i].name, keepPath) == 0) continue;
      if (oldest < 0 || lf[i].key < lf[oldest].key) oldest = i;
    }
    if (oldest < 0) break;
    LittleFS.remove(lf[oldest].name);
    g_filesDeleted++;
    lf[oldest] = lf[n - 1];
    n--;
  }
}

static void headerFor(char* out, size_t n) {
  snprintf(out, n, "#schema=%d;fw=%s;session=%lu;part=%u\n%s\n", LOG_SCHEMA, FW_VERSION, UL(g_session),
           (unsigned)g_part, LOG_COLUMNS);
}

static void writerFail(const char* why) {
  g_wrFailStreak = (uint8_t)(g_wrFailStreak + 1);
  g_fsFailTotal = g_fsFailTotal + 1;
  g_stats.storageFailures++;
  snprintf(g_fsFaultWhy, sizeof(g_fsFaultWhy), "%s", why);
  g_fsWriteFault = true;
  uint32_t back = 1000u << (g_wrFailStreak < 5 ? g_wrFailStreak : 5);
  g_wrBackoffUntil = millis() + back;
  if (g_wrFailStreak >= 5) g_logPersist = false;   // give up on persistence; RAM recorder + Serial continue
}

// Writer: gather whole lines (<=1 KB), append them, consume only after a verified write.
static void logPump(const Config& c, uint32_t nowMs) {
  if (!g_logPersist) {                              // persistence off: drain so the queue cannot jam
    while (g_logTail != g_logHead) {
      while (g_logTail != g_logHead && g_logq[g_logTail] != '\n') g_logTail = (uint16_t)((g_logTail + 1) % LOGQ_SIZE);
      if (g_logTail != g_logHead) g_logTail = (uint16_t)((g_logTail + 1) % LOGQ_SIZE);
      g_logDiscarded = g_logDiscarded + 1;
    }
    return;
  }
  uint16_t used = logqUsed();
  if (used == 0) return;
  bool due = elapsedMs(nowMs, g_wrLastFlushMs) >= c.flushMs || used > LOGQ_SIZE / 2;
  if (!due || (g_wrFailStreak && !deadlineReached(nowMs, g_wrBackoffUntil))) return;

  static char buf[1100];
  uint16_t n = 0, t = g_logTail;
  __sync_synchronize();
  uint16_t lineStart = 0, scanned = 0;
  while (t != g_logHead && scanned < used && n < 1000) {
    char ch = g_logq[t];
    buf[n++] = ch;
    t = (uint16_t)((t + 1) % LOGQ_SIZE);
    scanned++;
    if (ch == '\n') lineStart = n;
  }
  n = lineStart;                                    // only whole lines
  if (n == 0) return;
  g_wrLastFlushMs = nowMs;

  if (!fsLock(20)) return;
  bool ok = false;
  do {
    uint32_t freeB = fsFreeBytes();
    if (freeB < c.fsReserveBytes + n) { writerFail("filesystem full (reserve reached)"); break; }
    if (!g_curPath[0] || g_curBytes + n > c.fileMaxBytes) {          // open a new rotated part
      if (g_curPath[0]) g_part++;
      char np[28];
      logPathFor(g_session, g_part, np, sizeof(np));
      logEnforceBudget(c, nullptr);
      char hdr[240];
      headerFor(hdr, sizeof(hdr));
      File nf = LittleFS.open(np, "w");
      if (!nf) { writerFail("cannot create log file"); break; }
      size_t hw = nf.write((const uint8_t*)hdr, strlen(hdr));
      nf.close();
      if (hw != strlen(hdr)) { writerFail("header write short"); break; }
      snprintf(g_curPath, sizeof(g_curPath), "%s", np);
      g_curBytes = (uint32_t)hw;
      g_filesCreated++;
    }
    File f = LittleFS.open(g_curPath, "a");
    if (!f) { writerFail("cannot open log for append"); break; }
    size_t w = f.write((const uint8_t*)buf, n);
    f.close();
    if (w != n) { g_curBytes += (uint32_t)w; writerFail("short write"); break; }
    ok = true;
  } while (0);
  fsUnlock();
  if (!ok) return;
  g_curBytes += n;
  g_bytesWritten += n;
  g_wrFailStreak = 0;
  __sync_synchronize();
  g_logTail = (uint16_t)((g_logTail + n) % LOGQ_SIZE);
}

static void logInit(const Config& c) {
  (void)c;
  g_session = esp_random() % 100000u;               // used only if the filesystem is unusable
  if (g_fsRwOk && fsLock(100)) {
    uint32_t prev = 0;
    g_session = 1;                                  // first session on this filesystem
    File f = LittleFS.open("/session.cnt", "r");
    if (f) { char b[16] = {0}; size_t n = f.readBytes(b, sizeof(b) - 1); b[n] = 0; prev = (uint32_t)strtoul(b, nullptr, 10); f.close(); g_session = prev + 1; }
    File w = LittleFS.open("/session.cnt", "w");
    if (w) { char b[16]; snprintf(b, sizeof(b), "%lu", UL(g_session)); w.write((const uint8_t*)b, strlen(b)); w.close(); }
    fsUnlock();
  }
  g_part = 0;
  g_curPath[0] = 0;
  g_logPersist = g_fsRwOk;
}

// ---- flight recorder + row formatting -----------------------------------------------------
struct FrRec {
  uint32_t t;
  char type;
  uint8_t nav, cond;
  uint16_t irmask, actmask, faults;
  int16_t bear10, yaw10, herr10, gz10;   // 0x7FFF = not available
  uint8_t conf100, ball, imu;            // imu: 0 absent, 1 present-not-ok, 2 ok
  int8_t vx100, vy100, w100;
  int16_t pwm[4];
  uint16_t scan100us;
  uint16_t loop100us;
  char note[24];
};
#define FR_N 160
#define FR_NA 0x7FFF
static FrRec g_fr[FR_N];
static FrRec g_frFrozen[FR_N];
static uint16_t g_frHead = 0, g_frCount = 0, g_frFrozenCount = 0;
static bool g_frCapturing = false, g_frDumpPending = false;
static uint32_t g_frCapEndMs = 0, g_frCaptures = 0, g_frIgnored = 0, g_frLastPushMs = 0;
static uint16_t g_frDumpIdx = 0;
static char g_frReason[24] = "";

static int16_t sc10(float v) { return (int16_t)(v * 10.0f + (v >= 0 ? 0.5f : -0.5f)); }

static void frCapture(FrRec& r, char type, const char* note) {
  memset(&r, 0, sizeof(r));
  r.t = millis();
  r.type = type;
  r.nav = g_navState;
  r.cond = g_cond;
  uint16_t im = 0;
  for (uint8_t i = 0; i < MUX_CHANNELS; i++) if (g_ir[i].detect) im |= (uint16_t)(1u << i);
  r.irmask = im;
  r.actmask = g_ball.actMask;
  r.faults = faultMask();
  r.ball = g_ball.valid ? 1 : 0;
  r.bear10 = (g_ball.valid || g_ball.anyDetect) ? sc10(g_ball.bearingDeg) : FR_NA;
  r.conf100 = (uint8_t)(clampf(g_ball.conf, 0, 1) * 100.0f + 0.5f);
  r.imu = g_imu.present ? (g_imu.ok ? 2 : 1) : 0;
  r.yaw10 = (g_imu.present && g_imu.yawValid) ? sc10(g_imu.yaw) : FR_NA;
  r.herr10 = isnan(g_hdgErr) ? FR_NA : sc10(g_hdgErr);
  r.gz10 = (g_imu.present && g_imu.ok) ? sc10(g_imu.gz) : FR_NA;
  r.vx100 = (int8_t)(g_reqVx * 100.0f); r.vy100 = (int8_t)(g_reqVy * 100.0f); r.w100 = (int8_t)(g_reqW * 100.0f);
  for (int i = 0; i < 4; i++) r.pwm[i] = g_wheelOut[i];
  r.scan100us = (uint16_t)(g_scan.cycleUs / 100u > 65535u ? 65535u : g_scan.cycleUs / 100u);
  r.loop100us = (uint16_t)(g_loopMaxUs / 100u > 65535u ? 65535u : g_loopMaxUs / 100u);
  if (note) { snprintf(r.note, sizeof(r.note), "%s", note); for (char* p = r.note; *p; p++) if (*p == ',' || *p == '\n' || *p == '\r') *p = ';'; }
}

static void frFormat(const FrRec& r, uint32_t seq, char* out, size_t n) {
  char bear[12] = "", yaw[12] = "", herr[12] = "", gz[12] = "";
  if (r.bear10 != FR_NA) snprintf(bear, sizeof(bear), "%.0f", r.bear10 / 10.0f);
  if (r.yaw10 != FR_NA) snprintf(yaw, sizeof(yaw), "%.1f", r.yaw10 / 10.0f);
  if (r.herr10 != FR_NA) snprintf(herr, sizeof(herr), "%.1f", r.herr10 / 10.0f);
  if (r.gz10 != FR_NA) snprintf(gz, sizeof(gz), "%.1f", r.gz10 / 10.0f);
  snprintf(out, n, "%lu,%lu,%lu,%c,%u,%u,%04X,%04X,%u,%s,%.2f,%u,%s,%s,%s,%.2f,%.2f,%.2f,%d,%d,%d,%d,%04X,%lu,%lu,%s\n",
           UL(r.t), UL(g_session), UL(seq), r.type, (unsigned)r.cond, (unsigned)r.nav, (unsigned)r.irmask,
           (unsigned)r.actmask, (unsigned)r.ball, bear, r.conf100 / 100.0f, (unsigned)r.imu, yaw, herr, gz,
           r.vx100 / 100.0f, r.vy100 / 100.0f, r.w100 / 100.0f, r.pwm[0], r.pwm[1], r.pwm[2], r.pwm[3],
           (unsigned)r.faults, UL((uint32_t)r.scan100us * 100u), UL((uint32_t)r.loop100us * 100u), r.note);
}

static void frPush(const FrRec& r) {
  g_fr[g_frHead] = r;
  g_frHead = (uint16_t)((g_frHead + 1) % FR_N);
  if (g_frCount < FR_N) g_frCount++;
}

static void logRow(char type, const char* note) {          // one CSV row to the persistent queue
  FrRec r;
  frCapture(r, type, note);
  char line[260];
  frFormat(r, g_seq++, line, sizeof(line));
  logPutLine(line);
}

// Event: CSV row + flight-recorder entry (always, even if persistence is off).
static void logEvent(const char* type, const char* note) {
  FrRec r;
  frCapture(r, type[0], note);
  frPush(r);
  char line[260];
  frFormat(r, g_seq++, line, sizeof(line));
  logPutLine(line);
}

static void frTrigger(const char* why) {
  if (g_frCapturing) { g_frIgnored++; return; }
  g_frCapturing = true;
  g_frCapEndMs = millis() + g_cfg.frPostMs;
  FrRec r;
  frCapture(r, 'E', why);
  frPush(r);
  snprintf(g_frReason, sizeof(g_frReason), "%s", why);
}

// Pre-event history is whatever is already in the ring; post-event records keep accumulating until
// frPostMs elapses. Then the whole ring is frozen (kept in RAM for export) and queued to the log.
static void frService(const Config& c, uint32_t nowMs) {
  if (elapsedMs(nowMs, g_frLastPushMs) >= c.frPeriodMs) {
    g_frLastPushMs = nowMs;
    FrRec r;
    frCapture(r, 'F', nullptr);
    frPush(r);
  }
  if (g_frCapturing && deadlineReached(nowMs, g_frCapEndMs)) {
    g_frCapturing = false;
    g_frFrozenCount = g_frCount;
    uint16_t start = (uint16_t)((g_frHead + FR_N - g_frCount) % FR_N);
    for (uint16_t i = 0; i < g_frCount; i++) g_frFrozen[i] = g_fr[(start + i) % FR_N];
    g_frDumpPending = true;
    g_frDumpIdx = 0;
    g_frCaptures++;
    say("LOG", "flight recorder captured %u records around '%s' (export with :xsel ring then x)", (unsigned)g_frFrozenCount, g_frReason);
  }
  // Paced dump of the frozen ring into the CSV queue as type 'B' rows (3 per call).
  for (int k = 0; k < 3 && g_frDumpPending; k++) {
    if (logqFree() < 400) break;
    if (g_frDumpIdx >= g_frFrozenCount) { g_frDumpPending = false; break; }
    FrRec r = g_frFrozen[g_frDumpIdx++];
    r.type = 'B';
    char line[260];
    frFormat(r, g_seq++, line, sizeof(line));
    logPutLine(line);
  }
}

// ---- periodic snapshots ----------------------------------------------------------------------
static uint32_t g_lastSnapMs = 0, g_lastDetailMs = 0;
static void logService(const Config& c, uint32_t nowMs) {
  bool quiet = (g_navState == NAV_IDLE || g_navState == NAV_SAFE_STOP) && motorsAllZero();
  uint32_t period = quiet ? 5000u : c.snapshotMs;       // idle: slow, to spare flash
  if (elapsedMs(nowMs, g_lastSnapMs) >= period) {
    g_lastSnapMs = nowMs;
    char note[24];
    snprintf(note, sizeof(note), "drop=%lu;wf=%lu", UL(g_logDropped % 100000), UL(g_fsFailTotal % 100000));   // dropped records / write failures
    logRow('S', note);
    g_loopMaxUs = 0;
  }
  if (c.detailMs && elapsedMs(nowMs, g_lastDetailMs) >= c.detailMs) {
    g_lastDetailMs = nowMs;
    for (uint8_t i = 0; i < MUX_CHANNELS; i++) {
      if (!c.ch[i].enabled) continue;
      char note[24];
      snprintf(note, sizeof(note), "c%u r%u e%u %s", (unsigned)i, (unsigned)g_ir[i].raw, (unsigned)g_ir[i].edges, WS_NAME[g_ir[i].winState]);
      logRow('D', note);
    }
  }
  static uint32_t lastDrops = 0;
  if (g_logDropped != lastDrops) {
    faultRaise(F_LOG_DROPS, "%lu log records dropped (queue full)", UL(g_logDropped));
    lastDrops = g_logDropped;
  }
  if (g_fsWriteFault) {
    g_fsWriteFault = false;
    faultRaise(F_FS_WRITE, "%s (failures=%lu)", g_fsFaultWhy, UL(g_fsFailTotal));
  }
  if (g_fsRwOk && g_logPersist && g_wrFailStreak == 0 && g_fault[F_FS_WRITE].active)
    faultUpdate(F_FS_WRITE, false, c.recoverVerifyMs, "");
}

// ---- export over Serial -----------------------------------------------------------------------
// Format:
//   #BEGIN-EXPORT name=<file|ring> schema=1
//   <the file's lines verbatim: '#' header lines, column row, CSV rows>
//   #END-EXPORT lines=<n> crc32=<8 hex digits over every exported line including '\n'>
// Every other message on the port starts with '[', so a parser can ignore those lines.
struct ExportState {
  bool active, isRing;
  File file;
  uint16_t ringIdx;
  uint32_t lines, crc;
  char name[28];
};
static ExportState g_exp;
static int8_t g_exportSel = -1;      // -1 = ring, >=0 = file index in newest-first order

static bool exportStart(int8_t sel) {
  static LogFile lf[LOG_LIST_MAX];            // loop-task only
  if (g_exp.active) return false;
  g_exp.active = false; g_exp.isRing = false; g_exp.ringIdx = 0; g_exp.lines = 0; g_exp.crc = 0;   // File member: no memset
  char b[96];
  if (sel < 0) {
    g_exp.isRing = true;
    snprintf(g_exp.name, sizeof(g_exp.name), "ring");
  } else {
    if (!g_fsRwOk) return false;
    if (!fsLock(50)) return false;
    uint8_t n = logList(lf);
    fsUnlock();
    // sort newest first
    for (uint8_t i = 0; i < n; i++) for (uint8_t j = i + 1; j < n; j++) if (lf[j].key > lf[i].key) { LogFile t = lf[i]; lf[i] = lf[j]; lf[j] = t; }
    if (sel >= n) return false;
    snprintf(g_exp.name, sizeof(g_exp.name), "%s", lf[sel].name);
    if (!fsLock(50)) return false;
    g_exp.file = LittleFS.open(g_exp.name, "r");
    fsUnlock();
    if (!g_exp.file) return false;
  }
  snprintf(b, sizeof(b), "#BEGIN-EXPORT name=%s schema=%d\n", g_exp.name, LOG_SCHEMA);
  outPutLine(b);
  g_exp.active = true;
  g_exp.crc = 0;
  return true;
}

static void exportEmit(const char* line) {          // line includes '\n'
  outPutLine(line);
  g_exp.crc = crc32Update(g_exp.crc, (const uint8_t*)line, strlen(line));
  for (const char* q = line; *q; q++) if (*q == '\n') g_exp.lines++;   // header buffers hold two lines
}

static void exportService() {
  if (!g_exp.active) return;
  for (int k = 0; k < 2 && outFree() > 700; k++) {
    char line[260];
    if (g_exp.isRing) {
      if (k == 0 && g_exp.ringIdx == 0) {
        char hdr[256];
        snprintf(hdr, sizeof(hdr), "#schema=%d;fw=%s;session=%lu;source=ring\n%s\n", LOG_SCHEMA, FW_VERSION, UL(g_session), LOG_COLUMNS);
        exportEmit(hdr);
      }
      const FrRec* src; uint16_t total;
      if (g_frFrozenCount) { src = g_frFrozen; total = g_frFrozenCount; }
      else { src = g_fr; total = g_frCount; }
      if (g_exp.ringIdx >= total) break;
      uint16_t start = (g_frFrozenCount ? 0 : (uint16_t)((g_frHead + FR_N - g_frCount) % FR_N));
      const FrRec& r = g_frFrozenCount ? src[g_exp.ringIdx] : src[(start + g_exp.ringIdx) % FR_N];
      frFormat(r, g_exp.ringIdx, line, sizeof(line));
      g_exp.ringIdx++;
      exportEmit(line);
    } else {
      size_t n = 0;
      bool got = false;
      if (fsLock(0)) {
        if (g_exp.file.available()) { n = g_exp.file.readBytesUntil('\n', line, sizeof(line) - 2); got = true; }
        fsUnlock();
      }
      if (!got) break;
      line[n] = '\n'; line[n + 1] = 0;
      exportEmit(line);
    }
  }
  bool finished = false;
  if (g_exp.isRing) {
    uint16_t total = g_frFrozenCount ? g_frFrozenCount : g_frCount;
    finished = g_exp.ringIdx >= total && g_exp.lines > 0;
  } else {
    finished = !g_exp.file.available();
  }
  if (finished && outFree() > 120) {
    char b[80];
    snprintf(b, sizeof(b), "#END-EXPORT lines=%lu crc32=%08lX\n", UL(g_exp.lines), UL(g_exp.crc));
    outPutLine(b);
    if (!g_exp.isRing) g_exp.file.close();
    g_exp.active = false;
  }
}

static void logStatusPrint(const Config& c) {
  static LogFile lf[LOG_LIST_MAX];            // loop-task only
  say("LOG", "session=%lu schema=%d persist=%s fs=%s", UL(g_session), LOG_SCHEMA, g_logPersist ? "ON" : "OFF", g_fsDetail);
  say("LOG", "file=%s bytes=%lu/%lu part=%u budget=%u files x %lu B, reserve=%lu B", g_curPath[0] ? g_curPath : "(none yet)",
      UL(g_curBytes), UL(c.fileMaxBytes), (unsigned)g_part, (unsigned)c.maxFiles, UL(c.fileMaxBytes), UL(c.fsReserveBytes));
  say("LOG", "storage free=%lu B written=%lu B created=%lu deleted(rotation)=%lu write-failures=%lu", UL(fsFreeBytes()),
      UL(g_bytesWritten), UL(g_filesCreated), UL(g_filesDeleted), UL(g_fsFailTotal));
  say("LOG", "RAM log queue %u/%u B, records dropped=%lu, discarded(persist off)=%lu", (unsigned)logqUsed(), LOGQ_SIZE - 1,
      UL(g_logDropped), UL(g_logDiscarded));
  say("LOG", "flight recorder %u/%u records, captures=%lu (frozen=%u), serial lines dropped=%lu, export-select=%s",
      (unsigned)g_frCount, FR_N, UL(g_frCaptures), (unsigned)g_frFrozenCount, UL(g_outDropped),
      g_exportSel < 0 ? "ring" : "file");
  if (g_fsRwOk) {
    if (fsLock(50)) {
      uint8_t n = logList(lf);
      fsUnlock();
      for (uint8_t i = 0; i < n; i++) for (uint8_t j = i + 1; j < n; j++) if (lf[j].key > lf[i].key) { LogFile t = lf[i]; lf[i] = lf[j]; lf[j] = t; }
      for (uint8_t i = 0; i < n; i++) say("LOG", "  [%u] %s %lu B", (unsigned)i, lf[i].name, UL(lf[i].size));
    }
  }
}
