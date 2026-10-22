// storage.h - the configuration lives in the ATmega2560's 4 KB EEPROM (CRC protected, versioned). There is no filesystem and no CSV logging to flash:
// the Mega has 8 KB of RAM and 256 KB of flash. Diagnostic rows are streamed over Serial instead (':diag on').
#pragma once
#ifndef HOST_TEST
#include <EEPROM.h>
#endif
#include "config.h"

enum CfgLoadResult : uint8_t { CL_OK, CL_NOFILE, CL_BADSIZE, CL_BADMAGIC, CL_BADVERSION, CL_BADCRC, CL_REPAIRED };
static PGM_P cfgLoadText(CfgLoadResult r) {
  switch (r) {
    case CL_OK: return PSTR("loaded OK");
    case CL_NOFILE: return PSTR("no saved config (EEPROM blank) - using defaults");
    case CL_BADSIZE: return PSTR("saved config size mismatch - defaults loaded");
    case CL_BADMAGIC: return PSTR("saved config magic invalid - defaults loaded");
    case CL_BADVERSION: return PSTR("saved config version incompatible - defaults loaded");
    case CL_BADCRC: return PSTR("saved config CRC mismatch (corrupt) - defaults loaded");
    case CL_REPAIRED: return PSTR("saved config had invalid fields - those fields reset to defaults");
  }
  return PSTR("?");
}

static const int EEPROM_CFG_ADDR = 0;

static CfgLoadResult cfgLoad(Config& out, char* repaired, size_t repN) {
  cfgDefaults(out);
  if (repN) repaired[0] = 0;
  Config tmp;
  EEPROM.get(EEPROM_CFG_ADDR, tmp);
  if (tmp.magic == 0xFFFFFFFFu) return CL_NOFILE;
  if (tmp.magic != CFG_MAGIC) return CL_BADMAGIC;
  if (tmp.version != CFG_VERSION || tmp.size != sizeof(Config)) return CL_BADVERSION;
  if (!cfgCrcOk(tmp)) return CL_BADCRC;
  int bad = cfgValidate(tmp, repaired, repN);
  out = tmp;
  return bad ? CL_REPAIRED : CL_OK;
}

// Writes only the bytes that changed (EEPROM.put), then reads back and checks the CRC.
static bool cfgSave(Config& c) {
  char rep[48];
  if (cfgValidate(c, rep, sizeof(rep)) != 0) return false;   // never persist invalid settings
  cfgSeal(c);
  EEPROM.put(EEPROM_CFG_ADDR, c);
  Config chk;
  EEPROM.get(EEPROM_CFG_ADDR, chk);
  return cfgCrcOk(chk) && memcmp(&chk, &c, sizeof(Config)) == 0;
}
