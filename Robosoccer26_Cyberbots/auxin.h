// auxin.h - two extra IR inputs, diagnostics only (nothing in navigation uses them yet).
//   GPIO35: analog IR distance sensor (raw 12-bit ADC value, higher/lower depends on the sensor: characterise before use)
//   GPIO36: digital IR receiver (TSOP style, active LOW): pulses per 20 ms window and their mean spacing
// Everything is OFF after every reset (RAM-only switch ':aux on'), so the ball detector and the saved configuration are unchanged.
#pragma once
#ifdef HOST_TEST
  #define AUX_RX_LEVEL() digitalRead(PIN_AUX_RX)
  #define AUX_NOW_US() micros()
#else
  #include <soc/gpio_struct.h>
  #include <esp_timer.h>
  #define AUX_RX_LEVEL() ((uint8_t)((GPIO.in1.val >> (PIN_AUX_RX - 32)) & 1u))
  #define AUX_NOW_US() ((uint32_t)esp_timer_get_time())
#endif

static volatile uint16_t g_auxEdges = 0;                // active-level pulse starts since the last snapshot
static volatile uint32_t g_auxFirst = 0, g_auxLast = 0;
static volatile bool g_auxLevelActive = false;

static void IRAM_ATTR auxIsr() {
  bool active = (AUX_RX_LEVEL() == 0);
  if (active && !g_auxLevelActive) {
    uint32_t t = AUX_NOW_US();
    if (g_auxEdges == 0) g_auxFirst = t;
    g_auxLast = t;
    if (g_auxEdges < 500) g_auxEdges = (uint16_t)(g_auxEdges + 1);   // bounded
  }
  g_auxLevelActive = active;
}

static const uint32_t AUX_WINDOW_MS = 20;
struct AuxState {
  bool on;
  uint32_t lastMs, windows, activeWindows, pulseSum, perSum, perN, floodWindows;
  uint16_t lastEdges; uint32_t lastPeriodUs;
  uint32_t dSamples, dRaw, dMin, dMax; float dAvg;
};
static AuxState g_aux;

static void auxReset() {
  bool on = g_aux.on; uint32_t t = g_aux.lastMs;
  memset(&g_aux, 0, sizeof(g_aux));
  g_aux.on = on; g_aux.lastMs = t; g_aux.dMin = 0xFFFFFFFFu;
}

// Pure window rule (unit tested): >= 3 pulses in a window counts as the receiver seeing a modulated source.
static bool auxWindowActive(uint16_t edges) { return edges >= 3; }

static void auxEnable(bool on, uint32_t nowMs) {
  if (on == g_aux.on) return;
  if (on) {
    pinMode(PIN_AUX_DIST, INPUT); pinMode(PIN_AUX_RX, INPUT);
    g_auxLevelActive = (digitalRead(PIN_AUX_RX) == 0);
    g_auxEdges = 0;
    auxReset(); g_aux.on = true; g_aux.lastMs = nowMs;
    attachInterrupt(digitalPinToInterrupt(PIN_AUX_RX), auxIsr, CHANGE);
  } else {
    detachInterrupt(digitalPinToInterrupt(PIN_AUX_RX));
    g_aux.on = false;
  }
}

// Every loop; does nothing while off, one 20 ms bookkeeping step while on.
static void auxService(uint32_t nowMs) {
  if (!g_aux.on || elapsedMs(nowMs, g_aux.lastMs) < AUX_WINDOW_MS) return;
  g_aux.lastMs = nowMs;
  uint32_t raw = (uint32_t)analogRead(PIN_AUX_DIST);
  g_aux.dRaw = raw; g_aux.dSamples++;
  if (raw < g_aux.dMin) g_aux.dMin = raw;
  if (raw > g_aux.dMax) g_aux.dMax = raw;
  g_aux.dAvg = g_aux.dSamples == 1 ? (float)raw : g_aux.dAvg * 0.8f + (float)raw * 0.2f;
  g_auxLevelActive = (AUX_RX_LEVEL() == 0);     // resynchronise (an edge could have been missed while the pin was disabled)
  noInterrupts();
  uint16_t n = g_auxEdges; uint32_t f = g_auxFirst, l = g_auxLast;
  g_auxEdges = 0;
  interrupts();
  g_aux.windows++; g_aux.pulseSum += n; g_aux.lastEdges = n;
  if (n >= 500) g_aux.floodWindows++;
  if (auxWindowActive(n)) { g_aux.activeWindows++; g_aux.lastPeriodUs = (l - f) / (uint32_t)(n - 1); g_aux.perSum += g_aux.lastPeriodUs; g_aux.perN++; }
}

static void cmdAux(const char* arg, uint32_t nowMs) {
  if (arg && !strcmp(arg, "on")) { auxEnable(true, nowMs); say("INFO", "aux inputs ON (GPIO%u analog distance, GPIO%u IR receiver); off again after a reset", (unsigned)PIN_AUX_DIST, (unsigned)PIN_AUX_RX); return; }
  if (arg && !strcmp(arg, "off")) { auxEnable(false, nowMs); say("INFO", "aux inputs OFF"); return; }
  if (arg && !strcmp(arg, "reset")) { auxReset(); say("INFO", "aux statistics cleared"); return; }
  if (arg) { say("WARN", "usage: :aux [on|off|reset]"); return; }
  if (!g_aux.on) { say("INFO", "aux inputs are OFF (':aux on'). GPIO%u: analog IR distance sensor, GPIO%u: IR receiver (active LOW)", (unsigned)PIN_AUX_DIST, (unsigned)PIN_AUX_RX); return; }
  say("INFO", "aux distance GPIO%u: raw now %lu, smoothed %.0f, min %lu, max %lu over %lu samples (12-bit ADC; no unit conversion: characterise your sensor)",
      (unsigned)PIN_AUX_DIST, UL(g_aux.dRaw), (double)g_aux.dAvg, UL(g_aux.dSamples ? g_aux.dMin : 0), UL(g_aux.dMax), UL(g_aux.dSamples));
  say("INFO", "aux receiver GPIO%u: %lu windows of %lu ms, active in %lu (%.1f%%), last window %u pulses, mean pulses/window %.1f, mean pulse spacing in active windows %lu us, flooded windows %lu",
      (unsigned)PIN_AUX_RX, UL(g_aux.windows), UL(AUX_WINDOW_MS), UL(g_aux.activeWindows), g_aux.windows ? 100.0 * g_aux.activeWindows / g_aux.windows : 0.0,
      (unsigned)g_aux.lastEdges, g_aux.windows ? (double)g_aux.pulseSum / g_aux.windows : 0.0, UL(g_aux.perN ? g_aux.perSum / g_aux.perN : 0), UL(g_aux.floodWindows));
}
