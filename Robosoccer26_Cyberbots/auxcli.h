// auxcli.h - ':aux' and ':avoid' commands (needs parseF from cli.h's helpers).
#pragma once

static void cmdAux(const char* arg, uint32_t nowMs) {
  if (arg && !strcmp(arg, "on")) { auxEnable(true, nowMs); g_aux.manualOff = false; say("INFO", "aux inputs ON (GPIO%u analog distance, GPIO%u %s); ':aux off' disables them, also avoidance", (unsigned)PIN_AUX_DIST, (unsigned)PIN_AUX_RX, (g_cfg.avoidMask & 2) ? "analog distance" : "IR receiver"); return; }
  if (arg && !strcmp(arg, "off")) { auxEnable(false, nowMs); g_aux.manualOff = true; say("INFO", "aux inputs OFF%s", g_cfg.avoidMask ? " (obstacle avoidance is therefore inactive until ':aux on')" : ""); return; }
  if (arg && !strcmp(arg, "reset")) { auxReset(); say("INFO", "aux statistics cleared"); return; }
  if (arg) { say("WARN", "usage: :aux [on|off|reset]"); return; }
  if (!g_aux.on) { say("INFO", "aux inputs are OFF (':aux on'). GPIO%u: analog IR distance sensor, GPIO%u: IR receiver (active LOW) or second distance sensor (:avoid)", (unsigned)PIN_AUX_DIST, (unsigned)PIN_AUX_RX); return; }
  say("INFO", "aux distance GPIO%u: raw now %lu, smoothed %.0f, min %lu, max %lu over %lu samples (12-bit ADC; no unit conversion: characterise your sensor)",
      (unsigned)PIN_AUX_DIST, UL(g_aux.dRaw[0]), (double)g_aux.dAvg[0], UL(g_aux.dSamples ? g_aux.dMin[0] : 0), UL(g_aux.dMax[0]), UL(g_aux.dSamples));
  if (g_cfg.avoidMask & 2) {
    say("INFO", "aux distance GPIO%u: raw now %lu, smoothed %.0f, min %lu, max %lu", (unsigned)PIN_AUX_RX, UL(g_aux.dRaw[1]), (double)g_aux.dAvg[1], UL(g_aux.dSamples ? g_aux.dMin[1] : 0), UL(g_aux.dMax[1]));
    return;
  }
  say("INFO", "aux receiver GPIO%u: %lu windows of %lu ms, active in %lu (%.1f%%), last window %u pulses, mean pulses/window %.1f, mean pulse spacing in active windows %lu us, flooded windows %lu",
      (unsigned)PIN_AUX_RX, UL(g_aux.windows), UL(AUX_WINDOW_MS), UL(g_aux.activeWindows), g_aux.windows ? 100.0 * g_aux.activeWindows / g_aux.windows : 0.0,
      (unsigned)g_aux.lastEdges, g_aux.windows ? (double)g_aux.pulseSum / g_aux.windows : 0.0, UL(g_aux.perN ? g_aux.perSum / g_aux.perN : 0), UL(g_aux.floodWindows));
}

// ':avoid' status | cal <0|1> slow|stop | on <0|1> | off <0|1>
static void cmdAvoid(Config& c, const char* a1, const char* a2, const char* a3, uint32_t nowMs) {
  if (!a1) {
    say("INFO", "obstacle avoidance %s (mask %u). Near: toward-obstacle speed <= %u %%; stop zone: none, or a %u %% creep when the ball is within +-%u deg of the sensor direction",
        c.avoidMask ? "ON" : "OFF", (unsigned)c.avoidMask, (unsigned)c.avoidSlowPct, (unsigned)c.avoidCautionPct, (unsigned)c.avoidSectorDeg);
    for (uint8_t i = 0; i < 2; i++) {
      uint16_t sl = i == 0 ? c.avoid0Slow : c.avoid1Slow, st = i == 0 ? c.avoid0Stop : c.avoid1Stop;
      say("INFO", " sensor %u (GPIO%u) %s facing %.0f deg: slow=%u stop=%u raw=%.0f level=%u", (unsigned)i, (unsigned)(i == 0 ? PIN_AUX_DIST : PIN_AUX_RX), ((c.avoidMask >> i) & 1u) ? "IN USE" : "unused",
          (double)(i == 0 ? c.avoid0Deg : c.avoid1Deg), (unsigned)sl, (unsigned)st, (double)g_aux.dAvg[i], (unsigned)g_aux.level[i]);
    }
    return;
  }
  float n;
  if (!strcmp(a1, "cal") && a2 && parseF(a2, &n) && (n == 0 || n == 1) && a3 && (!strcmp(a3, "slow") || !strcmp(a3, "stop"))) {
    if (!g_aux.on || g_aux.dSamples == 0) { say("WARN", "avoid cal: switch the inputs on first (':aux on') and hold the obstacle still"); return; }
    uint16_t v = (uint16_t)(g_aux.dAvg[(int)n] + 0.5f);
    bool slow = !strcmp(a3, "slow");
    if ((int)n == 0) (slow ? c.avoid0Slow : c.avoid0Stop) = v; else (slow ? c.avoid1Slow : c.avoid1Stop) = v;
    say("INFO", "sensor %d %s threshold = raw %u (smoothed value now). Set both, then ':avoid on %d' and ':save'", (int)n, a3, (unsigned)v, (int)n);
    return;
  }
  if ((!strcmp(a1, "on") || !strcmp(a1, "off")) && a2 && parseF(a2, &n) && (n == 0 || n == 1)) {
    uint8_t bit = (uint8_t)(1u << (int)n);
    if (!strcmp(a1, "on")) {
      uint16_t sl = n == 0 ? c.avoid0Slow : c.avoid1Slow, st = n == 0 ? c.avoid0Stop : c.avoid1Stop;
      if ((sl > st ? sl - st : st - sl) < 50) { say("WARN", "avoid on: sensor %d is not calibrated (slow and stop raw values must differ by >= 50): use ':avoid cal %d slow' and ':avoid cal %d stop'", (int)n, (int)n, (int)n); return; }
      c.avoidMask |= bit;
      auxEnable(true, nowMs);
      say("INFO", "avoidance ON for sensor %d%s. Sensors must sit ABOVE the ball's height (else the ball itself reads as an obstacle). ':save' keeps it.", (int)n, n == 1 ? " (GPIO36 now an analog input: the receiver diagnostic is off)" : "");
    } else { c.avoidMask &= (uint8_t)~bit; say("INFO", "avoidance OFF for sensor %d", (int)n); }
    return;
  }
  say("WARN", "usage: :avoid | :avoid cal <0|1> <slow|stop> | :avoid on <0|1> | :avoid off <0|1>");
}
