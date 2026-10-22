// motors.h - the single motor-control layer: direction pins, PWM, mecanum mixing, ramping.
// Everything that moves a wheel goes through motorsDrive()/motorsSetWheelTarget().
#pragma once
#include "config.h"
#include "health.h"

// ---- LEDC wrapper: Arduino-ESP32 2.x (channel based) vs 3.x (pin based) ----------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LEDC_API_V3 1
#else
  #define LEDC_API_V3 0
#endif
static const uint8_t PWM_BITS = 8;
static const uint16_t PWM_MAX = 255;

static bool g_pwmAttached[WHEEL_COUNT];
static bool g_motorsEnabled = false;      // false = every output forced LOW/0
static float g_wheelCur[WHEEL_COUNT];     // ramped, signed PWM counts (logical direction)
static float g_wheelTgt[WHEEL_COUNT];     // requested, signed PWM counts
static bool g_rampBoost = false;          // the shot's drive phase: acceleration limit lifted (the ramp would eat most of the 400 ms)
static int16_t g_wheelOut[WHEEL_COUNT];   // last value written (signed, logical direction)
static uint32_t g_wheelHoldUntil[WHEEL_COUNT];
static bool g_wheelHolding[WHEEL_COUNT];
static float g_reqVx, g_reqVy, g_reqW;    // last normalised body request (for diagnostics)
static uint32_t g_motorsLastUpdateMs;
static uint32_t g_motorsStoppedSinceMs;

static void pwmWrite(uint8_t wheel, uint32_t duty) {
  if (!g_pwmAttached[wheel]) return;
#if LEDC_API_V3
  ledcWrite(WHEEL_PINS[wheel].en, duty);
#else
  ledcWrite(wheel, duty);   // 2.x: LEDC channel number == wheel index
#endif
}

// Called first thing in setup(): every motor pin becomes a LOW output (coasting driver).
// External pull-downs are still required for the window before this runs (see docs).
static void motorsPinsSafe() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(WHEEL_PINS[i].in1, OUTPUT); digitalWrite(WHEEL_PINS[i].in1, LOW);
    pinMode(WHEEL_PINS[i].in2, OUTPUT); digitalWrite(WHEEL_PINS[i].in2, LOW);
    pinMode(WHEEL_PINS[i].en, OUTPUT);  digitalWrite(WHEEL_PINS[i].en, LOW);
    g_pwmAttached[i] = false;
    g_wheelCur[i] = g_wheelTgt[i] = 0;
    g_wheelOut[i] = 0;
  }
  g_motorsEnabled = false;
}

// Attach PWM to the enable pins (duty 0). Returns false and raises F_MOTOR_INIT on failure.
static bool motorsAttachPwm(const Config& c) {
  bool ok = true;
  for (int i = 0; i < WHEEL_COUNT; i++) {
#if LEDC_API_V3
    bool a = ledcAttach(WHEEL_PINS[i].en, c.pwmFreq, PWM_BITS);
    if (!a) ok = false;
    g_pwmAttached[i] = a;
#else
    uint32_t f = ledcSetup(i, c.pwmFreq, PWM_BITS);   // returns the achieved frequency, 0 on failure
    if (f == 0) { ok = false; g_pwmAttached[i] = false; continue; }
    ledcAttachPin(WHEEL_PINS[i].en, i);
    g_pwmAttached[i] = true;
#endif
    pwmWrite((uint8_t)i, 0);
  }
  if (!ok) faultRaise(F_MOTOR_INIT, "LEDC attach failed on at least one enable pin");
  return ok;
}

// Immediate stop: no ramp. Used for 's', '!', SAFE_STOP and faults.
static void motorsStopNow() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    g_wheelCur[i] = g_wheelTgt[i] = 0;
    g_wheelOut[i] = 0;
    g_wheelHolding[i] = false;
    pwmWrite((uint8_t)i, 0);
    digitalWrite(WHEEL_PINS[i].in1, LOW);
    digitalWrite(WHEEL_PINS[i].in2, LOW);
  }
  g_reqVx = g_reqVy = g_reqW = 0;
  g_motorsStoppedSinceMs = millis();
}

static void motorsSetEnabled(bool en) {
  if (!en) motorsStopNow();
  g_motorsEnabled = en;
}

// Apply one signed value (logical direction) to the pins, honouring software reversal.
static void wheelApply(const Config& c, uint8_t i, int16_t v) {
  if (!g_motorsEnabled || !g_pwmAttached[i]) v = 0;
  bool rev = c.reverse[i] != 0;
  int16_t phys = rev ? (int16_t)-v : v;
  uint16_t mag = (uint16_t)(phys < 0 ? -phys : phys);
  if (mag > PWM_MAX) mag = PWM_MAX;
  if (phys > 0)      { digitalWrite(WHEEL_PINS[i].in1, HIGH); digitalWrite(WHEEL_PINS[i].in2, LOW); }
  else if (phys < 0) { digitalWrite(WHEEL_PINS[i].in1, LOW);  digitalWrite(WHEEL_PINS[i].in2, HIGH); }
  else               { digitalWrite(WHEEL_PINS[i].in1, LOW);  digitalWrite(WHEEL_PINS[i].in2, LOW); }
  pwmWrite(i, mag);
  g_wheelOut[i] = v;
}

// ---- mecanum kinematics (pure functions; unit tested) ---------------------------------
// Body request: vx = strafe right (+), vy = forward (+), w = rotation clockwise (+), each -1..1.
// "X" roller layout (rollers form an X when viewed from above):
//   FL = vy + vx + w     FR = vy - vx - w
//   BL = vy - vx + w     BR = vy + vx - w
// "O" layout flips the sign of the strafe term. Per-motor mounting direction is handled by the
// per-wheel software reversal, never here.
static void mecanumMix(float vx, float vy, float w, bool layoutO, float out[WHEEL_COUNT]) {
  vx = clampf(vx, -1.0f, 1.0f);
  vy = clampf(vy, -1.0f, 1.0f);
  w = clampf(w, -1.0f, 1.0f);
  if (layoutO) vx = -vx;
  out[W_FL] = vy + vx + w;
  out[W_FR] = vy - vx - w;
  out[W_BL] = vy - vx + w;
  out[W_BR] = vy + vx - w;
  // Normalise: if any wheel exceeds 1, scale all by the same factor so ratios are preserved.
  float m = 0;
  for (int i = 0; i < WHEEL_COUNT; i++) if (absf(out[i]) > m) m = absf(out[i]);
  if (m > 1.0f) for (int i = 0; i < WHEEL_COUNT; i++) out[i] /= m;
}

// Map a normalised wheel command u (-1..1) to a signed PWM target. Non-zero commands start at
// minPwm so the wheel actually turns; tiny commands become a true zero.
static float wheelPwmFromUnit(float u, uint8_t minPwm, uint8_t limitPwm) {
  u = clampf(u, -1.0f, 1.0f);
  float a = absf(u);
  if (a < 0.02f) return 0.0f;
  float lim = (float)limitPwm;
  float lo = (float)minPwm < lim ? (float)minPwm : lim;
  float mag = lo + a * (lim - lo);
  return u < 0 ? -mag : mag;
}

static void motorsSetWheelTarget(uint8_t i, float signedPwm) {
  g_wheelTgt[i] = clampf(signedPwm, -(float)PWM_MAX, (float)PWM_MAX);
}

// High-level drive request. limitPwm caps the strongest wheel (manual vs autonomous limit).
static void motorsDrive(const Config& c, float vx, float vy, float w, uint8_t limitPwm) {
  float u[WHEEL_COUNT];
  mecanumMix(vx, vy, w, c.rollerO != 0, u);
  g_reqVx = clampf(vx, -1, 1); g_reqVy = clampf(vy, -1, 1); g_reqW = clampf(w, -1, 1);
  for (int i = 0; i < WHEEL_COUNT; i++) motorsSetWheelTarget((uint8_t)i, wheelPwmFromUnit(u[i], c.minPwm, limitPwm));
}

static bool motorsAllZero() {
  for (int i = 0; i < WHEEL_COUNT; i++) if (g_wheelCur[i] != 0 || g_wheelTgt[i] != 0) return false;
  return true;
}

// Advance the ramps by the measured elapsed time. Reversals pass through zero and pause there.
static void motorsUpdate(const Config& c, uint32_t nowMs) {
  uint32_t dtMs = elapsedMs(nowMs, g_motorsLastUpdateMs);
  g_motorsLastUpdateMs = nowMs;
  if (dtMs > 100) dtMs = 100;                 // after a stall never take one giant step
  float dt = (float)dtMs / 1000.0f;
  for (int i = 0; i < WHEEL_COUNT; i++) {
    float cur = g_wheelCur[i], tgt = g_wheelTgt[i];
    if (cur != 0 && tgt != 0 && ((cur > 0) != (tgt > 0))) tgt = 0;   // reversing: go through zero first
    if (g_wheelHolding[i]) {
      if (!deadlineReached(nowMs, g_wheelHoldUntil[i])) tgt = 0; else g_wheelHolding[i] = false;
    }
    bool speedingUp = absf(tgt) > absf(cur) && (cur == 0 || ((cur > 0) == (tgt > 0)));
    if (speedingUp && cur == 0 && !g_wheelHolding[i]) {   // below minPwm a wheel does not turn: do not ramp through the dead zone
      float lim = (float)c.minPwm < absf(tgt) ? (float)c.minPwm : absf(tgt);
      cur = tgt > 0 ? lim : -lim;
    }
    float step = (speedingUp ? (g_rampBoost ? 5000.0f : (float)c.accelPwmPerS) : (float)c.decelPwmPerS) * dt;
    if (tgt > cur) cur = (cur + step > tgt) ? tgt : cur + step;
    else if (tgt < cur) cur = (cur - step < tgt) ? tgt : cur - step;
    if (cur == 0 && g_wheelCur[i] != 0 && g_wheelTgt[i] != 0 && ((g_wheelTgt[i] > 0) != (g_wheelCur[i] > 0))) {
      g_wheelHolding[i] = true;               // just hit zero while reversing: dwell
      g_wheelHoldUntil[i] = nowMs + c.reverseDeadMs;
    }
    g_wheelCur[i] = cur;
    int16_t out = (int16_t)(cur >= 0 ? cur + 0.5f : cur - 0.5f);
    wheelApply(c, (uint8_t)i, out);
  }
  if (!motorsAllZero()) g_motorsStoppedSinceMs = nowMs;
}
