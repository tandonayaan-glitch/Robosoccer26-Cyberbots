// ircli.h - ':ir' command: status, per-sensor calibration, read mode, mounting side.
#pragma once

static bool g_irCalSet[2][2];                       // [sensor][0 = clear, 1 = near] captured since boot

static bool irSensorArg(const char* a, uint8_t* i) {
  if (!a || (a[0] != '0' && a[0] != '1') || a[1]) return false;
  *i = (uint8_t)(a[0] - '0');
  return true;
}

// ':ir' | ':ir cal clear|near <0|1>' | ':ir mode <0|1> analog|digital' | ':ir on|off <0|1>' | ':ir side right|left|none' | ':ir reset'
static void cmdIr(Config& c, const char* a1, const char* a2, const char* a3) {
  if (!a1) { printIrSnapshot(c); return; }
  uint8_t i;
  if (!strcmp(a1, "reset")) { irResetStats(); say("INFO", "IR min/max cleared"); return; }
  if (!strcmp(a1, "side") && a2) {
    if (!strcmp(a2, "right")) c.irOffsetSide = 1;
    else if (!strcmp(a2, "left")) c.irOffsetSide = -1;
    else if (!strcmp(a2, "none")) c.irOffsetSide = 0;
    else { say("WARN", "usage: :ir side right|left|none  (where sensor 1 sits relative to sensor 0, seen from behind the robot)"); return; }
    say("INFO", "sensor 1 mounting side: %s (':save' keeps it)", c.irOffsetSide > 0 ? "right of sensor 0" : (c.irOffsetSide < 0 ? "left of sensor 0" : "not set: no sideways guidance"));
    return;
  }
  if ((!strcmp(a1, "on") || !strcmp(a1, "off")) && irSensorArg(a2, &i)) {
    if (!strcmp(a1, "on")) c.irEnable |= (uint8_t)(1u << i); else c.irEnable &= (uint8_t)~(1u << i);
    say("INFO", "IR sensor %u %s", (unsigned)i, irEnabled(c, i) ? "ON" : "OFF");
    return;
  }
  if (!strcmp(a1, "mode") && irSensorArg(a2, &i) && a3 && (!strcmp(a3, "analog") || !strcmp(a3, "digital"))) {
    uint8_t m = !strcmp(a3, "digital") ? 1 : 0;
    if (i == 0) { c.irMode0 = m; c.irClear0 = c.irNear0 = 0; } else { c.irMode1 = m; c.irClear1 = c.irNear1 = 0; }
    g_irs[i].init = false; g_irCalSet[i][0] = g_irCalSet[i][1] = false;
    say("INFO", "sensor %u now read as %s; its calibration was cleared (the raw values mean something else now): ':ir cal clear %u' and ':ir cal near %u'", (unsigned)i, m ? "digital" : "analog", (unsigned)i, (unsigned)i);
    return;
  }
  if (!strcmp(a1, "cal") && a2 && a3) {
    bool clr = !strcmp(a2, "clear");
    if ((!clr && strcmp(a2, "near")) || !irSensorArg(a3, &i)) { say("WARN", "usage: :ir cal clear <0|1>  |  :ir cal near <0|1>"); return; }
    if (!irEnabled(c, i) || g_irs[i].samples < 10) { say("WARN", "ir cal: sensor %u is off or has not been sampled yet", (unsigned)i); return; }
    uint16_t v = (uint16_t)(g_irs[i].avg + 0.5f);
    if (i == 0) (clr ? c.irClear0 : c.irNear0) = v; else (clr ? c.irClear1 : c.irNear1) = v;
    say("INFO", "sensor %u %s = raw %u (smoothed reading now)%s", (unsigned)i, clr ? "CLEAR" : "NEAR", (unsigned)v, clr ? ": nothing may be in front of the sensor" : ": the ball must be in the shooting position");
    g_irCalSet[i][clr ? 0 : 1] = true;
    uint16_t a = irClearOf(c, i), b = irNearOf(c, i);
    if (g_irCalSet[i][0] && g_irCalSet[i][1]) {
      if ((a > b ? a - b : b - a) < 50) say("WARN", "sensor %u: clear (%u) and near (%u) differ by less than 50: the sensor shows no usable difference yet (not calibrated)", (unsigned)i, (unsigned)a, (unsigned)b);
      else say("INFO", "sensor %u calibrated: %s raw means a closer ball. ':save' keeps it.", (unsigned)i, b > a ? "a HIGHER" : "a LOWER");
    }
    return;
  }
  say("WARN", "usage: :ir | :ir cal clear|near <0|1> | :ir mode <0|1> analog|digital | :ir on|off <0|1> | :ir side right|left|none | :ir reset");
}
