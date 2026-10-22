// auxin.h - two extra IR inputs and the obstacle avoidance built on them.
//   GPIO35: analog IR distance sensor 0 (raw 12-bit ADC value; whether larger means closer depends on the sensor: ':avoid cal' decides)
//   GPIO36: either a digital IR receiver (TSOP style, active LOW; pulses per 20 ms window) or, with avoidMask bit1, distance sensor 1
// The inputs are read only after ':aux on' or when avoidMask is non-zero. Avoidance is OFF by default (avoidMask = 0); the ball detector is unaffected.
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
  bool on, manualOff;
  uint32_t lastMs, windows, activeWindows, pulseSum, perSum, perN, floodWindows;
  uint16_t lastEdges; uint32_t lastPeriodUs;
  uint32_t dSamples, dRaw[2], dMin[2], dMax[2]; float dAvg[2];
  uint8_t level[2];                                    // 0 clear, 1 near (slow down), 2 stop zone
  bool held; uint8_t heldCount;                        // ball held at the front sensor (debounced)
};
static AuxState g_aux;

static void auxReset() {
  bool on = g_aux.on, mo = g_aux.manualOff; uint32_t t = g_aux.lastMs;
  memset(&g_aux, 0, sizeof(g_aux));
  g_aux.on = on; g_aux.manualOff = mo; g_aux.lastMs = t; g_aux.dMin[0] = g_aux.dMin[1] = 0xFFFFFFFFu;
}

// Pure window rule (unit tested): >= 3 pulses in a window counts as the receiver seeing a modulated source.
static bool auxWindowActive(uint16_t edges) { return edges >= 3; }

// Pure level rule (unit tested): 0 clear, 1 near, 2 stop. "Closer" is the direction from slow towards stop. Leaving a level needs
// 10 % of the slow-to-stop span of margin, so a reading on a threshold does not chatter.
static uint8_t avoidLevel(float raw, uint16_t slow, uint16_t stop, uint8_t prev) {
  float span = (float)stop - (float)slow;
  if (span > -50.0f && span < 50.0f) return 0;           // not calibrated
  float dir = span > 0 ? 1.0f : -1.0f, m = 0.1f * (span < 0 ? -span : span);
  float x = (raw - (float)slow) * dir, xs = (raw - (float)stop) * dir;
  uint8_t lv = xs >= 0 ? 2 : (x >= 0 ? 1 : 0);
  if (lv < prev) {                                       // falling: apply the margin
    if (prev == 2 && xs > -m) lv = 2;
    else if (prev >= 1 && lv == 0 && x > -m) lv = 1;
  }
  return lv;
}

// Pure speed rule (unit tested). (vx, vy) is the commanded translation (vx right, vy forward, 0..1 of full speed). For each sensor in use
// the part of the motion that points towards that sensor is limited; the sideways part is left alone, so the robot slides along an
// obstacle instead of stopping dead. Near: at most avoidSlowPct. Stop zone: none - unless the ball is within avoidSectorDeg of
// that direction, then a cautious creep of avoidCautionPct, so the robot can still reach a ball that sits next to an obstacle.
static void avoidApply(const Config& c, uint8_t mask, const uint8_t level[2], float ballDeg, float* vx, float* vy) {
  const float dirs[2] = {c.avoid0Deg, c.avoid1Deg};
  for (uint8_t i = 0; i < 2; i++) {
    if (!((mask >> i) & 1u) || level[i] == 0) continue;
    float s = sinf(dirs[i] * DEG2RAD), co = cosf(dirs[i] * DEG2RAD);
    float along = *vx * s + *vy * co;
    if (along <= 0.0f) continue;
    bool ballThere = isfinite(ballDeg) && absf(angDiffDeg(ballDeg, dirs[i])) <= (float)c.avoidSectorDeg;   // NaN = no ball (e.g. during a shot)
    float cap = level[i] == 1 ? (float)c.avoidSlowPct / 100.0f : (ballThere ? (float)c.avoidCautionPct / 100.0f : 0.0f);
    if (along > cap) { *vx -= (along - cap) * s; *vy -= (along - cap) * co; }
  }
}

// Pure rule (unit tested): is the ball held? The midpoint between the "no ball" and "ball held" raw values decides, with 10 % of the span
// as hysteresis. Either polarity works (the sign of held - clear).
static bool holdState(float raw, uint16_t clearRaw, uint16_t heldRaw, bool prev) {
  float span = (float)heldRaw - (float)clearRaw;
  if (span > -50.0f && span < 50.0f) return false;
  float dir = span > 0 ? 1.0f : -1.0f, mid = 0.5f * ((float)clearRaw + (float)heldRaw), m = 0.1f * (span < 0 ? -span : span);
  float x = (raw - mid) * dir;
  return prev ? (x > -m) : (x > m);
}
// Pin 36 carries an analog sensor (avoidance sensor 1 or the ball-held sensor): no edge interrupts there.
static bool auxPin36Analog(const Config& c) { return (c.avoidMask & 2) || (c.holdEnable && c.holdSensor == 1); }

static void auxAttachRx(bool want) {
  static bool attached = false;
  if (want && !attached) { g_auxLevelActive = (digitalRead(PIN_AUX_RX) == 0); g_auxEdges = 0; attachInterrupt(digitalPinToInterrupt(PIN_AUX_RX), auxIsr, CHANGE); attached = true; }
  else if (!want && attached) { detachInterrupt(digitalPinToInterrupt(PIN_AUX_RX)); attached = false; }
}

static void auxEnable(bool on, uint32_t nowMs) {
  if (on == g_aux.on) return;
  if (on) {
    pinMode(PIN_AUX_DIST, INPUT); pinMode(PIN_AUX_RX, INPUT);
    auxReset(); g_aux.on = true; g_aux.manualOff = false; g_aux.lastMs = nowMs;
    auxAttachRx(!auxPin36Analog(g_cfg));                    // GPIO36 carries an analog sensor: no edge interrupts there
  } else {
    auxAttachRx(false);
    g_aux.on = false;
  }
}

// Every loop; does nothing while off, one 20 ms bookkeeping step while on.
static void auxService(uint32_t nowMs) {
  if (!g_aux.on && (g_cfg.avoidMask || g_cfg.holdEnable) && !g_aux.manualOff) auxEnable(true, nowMs);
  if (g_aux.on) auxAttachRx(!auxPin36Analog(g_cfg));       // follows a changed configuration
  if (!g_aux.on || elapsedMs(nowMs, g_aux.lastMs) < AUX_WINDOW_MS) return;
  g_aux.lastMs = nowMs;
  const bool dist1 = auxPin36Analog(g_cfg);
  for (uint8_t i = 0; i < (dist1 ? 2 : 1); i++) {
    uint32_t raw = (uint32_t)analogRead(i == 0 ? PIN_AUX_DIST : PIN_AUX_RX);
    g_aux.dRaw[i] = raw;
    if (raw < g_aux.dMin[i]) g_aux.dMin[i] = raw;
    if (raw > g_aux.dMax[i]) g_aux.dMax[i] = raw;
    g_aux.dAvg[i] = g_aux.dSamples == 0 ? (float)raw : g_aux.dAvg[i] * 0.6f + (float)raw * 0.4f;   // about 2-3 samples = 40-60 ms
    uint16_t slow = i == 0 ? g_cfg.avoid0Slow : g_cfg.avoid1Slow, stop = i == 0 ? g_cfg.avoid0Stop : g_cfg.avoid1Stop;
    g_aux.level[i] = ((g_cfg.avoidMask >> i) & 1u) ? avoidLevel(g_aux.dAvg[i], slow, stop, g_aux.level[i]) : 0;
  }
  if (g_cfg.holdEnable) {                                // ball held at the front sensor, debounced over 3 samples (60 ms)
    bool h = holdState(g_aux.dAvg[g_cfg.holdSensor & 1], g_cfg.holdClearRaw, g_cfg.holdHeldRaw, g_aux.held);
    if (h == g_aux.held) g_aux.heldCount = 0;
    else if (++g_aux.heldCount >= 3) { g_aux.held = h; g_aux.heldCount = 0; }
  } else { g_aux.held = false; g_aux.heldCount = 0; }
  g_aux.dSamples++;
  if (dist1) { g_auxEdges = 0; return; }
  g_auxLevelActive = (AUX_RX_LEVEL() == 0);     // resynchronise (an edge could have been missed while the pin was disabled)
  noInterrupts();
  uint16_t n = g_auxEdges; uint32_t f = g_auxFirst, l = g_auxLast;
  g_auxEdges = 0;
  interrupts();
  g_aux.windows++; g_aux.pulseSum += n; g_aux.lastEdges = n;
  if (n >= 500) g_aux.floodWindows++;
  if (auxWindowActive(n)) { g_aux.activeWindows++; g_aux.lastPeriodUs = (l - f) / (uint32_t)(n - 1); g_aux.perSum += g_aux.lastPeriodUs; g_aux.perN++; }
}

// Called from the approach: limits the commanded translation near obstacles. No-op unless avoidance is on. Stale sensor data
// (loop stalled, aux switched off by a reset race) is treated as "near", never as "clear".
static void avoidForNav(const Config& c, uint32_t nowMs, float ballDeg, float* vx, float* vy) {
  uint8_t mask = c.avoidMask;                             // the ball-held sensor must not count as an obstacle (it sees the ball itself)
  if (c.holdEnable) mask &= (uint8_t)~(1u << (c.holdSensor & 1));
  if (!mask || g_aux.manualOff) return;
  uint8_t lv[2] = {g_aux.level[0], g_aux.level[1]};
  if (!g_aux.on || elapsedMs(nowMs, g_aux.lastMs) > 100) { lv[0] = (mask & 1) ? 1 : 0; lv[1] = (mask & 2) ? 1 : 0; }
  avoidApply(c, mask, lv, ballDeg, vx, vy);
}

