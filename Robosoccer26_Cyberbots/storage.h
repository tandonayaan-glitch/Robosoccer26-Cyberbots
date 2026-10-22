// storage.h - LittleFS mount/self-test and configuration persistence.
// The filesystem is NEVER formatted automatically. A failed mount only disables persistence.
#pragma once
#include "config.h"

static bool g_fsMounted = false;     // LittleFS.begin(false) succeeded
static bool g_fsRwOk = false;        // a real write+read-back test passed
static char g_fsDetail[64] = "not initialised";
static SemaphoreHandle_t g_fsMutex = nullptr;   // serialises the log-writer task, export and config I/O

static inline bool fsLock(uint32_t waitMs) {
  if (!g_fsMutex) return true;
  return xSemaphoreTake(g_fsMutex, pdMS_TO_TICKS(waitMs)) == pdTRUE;
}
static inline void fsUnlock() { if (g_fsMutex) xSemaphoreGive(g_fsMutex); }

static uint32_t fsFreeBytes() {
  if (!g_fsMounted) return 0;
  size_t t = LittleFS.totalBytes(), u = LittleFS.usedBytes();
  return t > u ? (uint32_t)(t - u) : 0;
}

static bool fsReadWriteTest() {
  static const uint8_t pat[8] = {0xA5, 0x5A, 0x01, 0xFE, 0x42, 0x24, 0x7E, 0xE7};
  File f = LittleFS.open("/fstest.tmp", "w");
  if (!f) { snprintf(g_fsDetail, sizeof(g_fsDetail), "mounted, open-for-write failed"); return false; }
  size_t w = f.write(pat, sizeof(pat));
  f.close();
  if (w != sizeof(pat)) { snprintf(g_fsDetail, sizeof(g_fsDetail), "mounted, short write %u/8", (unsigned)w); LittleFS.remove("/fstest.tmp"); return false; }
  uint8_t back[8] = {0};
  File r = LittleFS.open("/fstest.tmp", "r");
  if (!r) { snprintf(g_fsDetail, sizeof(g_fsDetail), "mounted, read-back open failed"); return false; }
  size_t n = r.read(back, sizeof(back));
  r.close();
  LittleFS.remove("/fstest.tmp");
  if (n != sizeof(back) || memcmp(back, pat, sizeof(pat)) != 0) { snprintf(g_fsDetail, sizeof(g_fsDetail), "mounted, read-back MISMATCH"); return false; }
  return true;
}

static bool fsInit() {
  if (!g_fsMutex) g_fsMutex = xSemaphoreCreateMutex();
  g_fsMounted = LittleFS.begin(false);   // formatOnFail=false: never erase existing data
  g_fsRwOk = false;
  if (!g_fsMounted) {
    snprintf(g_fsDetail, sizeof(g_fsDetail), "mount failed (no LittleFS partition/format?)");
    return false;
  }
  g_fsRwOk = fsReadWriteTest();
  if (g_fsRwOk) snprintf(g_fsDetail, sizeof(g_fsDetail), "mounted, write+read-back OK");
  return g_fsRwOk;
}

// Explicit operator action only (":fsformat CONFIRM").
static bool fsFormatExplicit() {
  if (!fsLock(500)) return false;
  LittleFS.end();
  bool ok = LittleFS.format();
  fsUnlock();
  return ok && fsInit();
}

enum CfgLoadResult : uint8_t { CL_OK, CL_NOFS, CL_NOFILE, CL_BADSIZE, CL_BADMAGIC, CL_BADVERSION, CL_BADCRC, CL_REPAIRED };
static const char* cfgLoadText(CfgLoadResult r) {
  switch (r) {
    case CL_OK: return "loaded OK";
    case CL_NOFS: return "no filesystem - using defaults";
    case CL_NOFILE: return "no saved config - using defaults";
    case CL_BADSIZE: return "saved config size mismatch - defaults loaded";
    case CL_BADMAGIC: return "saved config magic invalid - defaults loaded";
    case CL_BADVERSION: return "saved config version incompatible - defaults loaded";
    case CL_BADCRC: return "saved config CRC mismatch (corrupt) - defaults loaded";
    case CL_REPAIRED: return "saved config had invalid fields - those fields reset to defaults";
  }
  return "?";
}

static CfgLoadResult cfgLoad(Config& out, char* repaired, size_t repN) {
  cfgDefaults(out);
  if (repN) repaired[0] = 0;
  if (!g_fsRwOk) return CL_NOFS;
  if (!fsLock(200)) return CL_NOFS;
  CfgLoadResult res = CL_OK;
  Config tmp;
  memset(&tmp, 0, sizeof(tmp));
  File f = LittleFS.open("/config.bin", "r");
  if (!f) { fsUnlock(); return CL_NOFILE; }
  size_t fsz = f.size();
  size_t n = (fsz == sizeof(Config)) ? f.read((uint8_t*)&tmp, sizeof(Config)) : 0;
  f.close();
  fsUnlock();
  if (fsz != sizeof(Config) || n != sizeof(Config)) return CL_BADSIZE;
  if (tmp.magic != CFG_MAGIC) return CL_BADMAGIC;
  if (tmp.version != CFG_VERSION || tmp.size != sizeof(Config)) return CL_BADVERSION;
  if (!cfgCrcOk(tmp)) return CL_BADCRC;
  int bad = cfgValidate(tmp, repaired, repN);
  if (bad) res = CL_REPAIRED;
  out = tmp;
  return res;
}

static bool cfgSave(Config& c) {
  char rep[64];
  if (cfgValidate(c, rep, sizeof(rep)) != 0) return false;   // never persist invalid settings
  cfgSeal(c);
  if (!g_fsRwOk || !fsLock(300)) return false;
  File f = LittleFS.open("/config.tmp", "w");
  bool ok = false;
  if (f) {
    ok = f.write((const uint8_t*)&c, sizeof(Config)) == sizeof(Config);
    f.close();
  }
  if (ok) {
    LittleFS.remove("/config.bin");
    ok = LittleFS.rename("/config.tmp", "/config.bin");
  }
  fsUnlock();
  return ok;
}
