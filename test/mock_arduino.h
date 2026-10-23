// mock_arduino.h - host-side stand-in for the Arduino AVR core (Mega 2560), used ONLY by the test harness.
// Time is virtual. The ring / module pins read values the tests set. API shapes follow the Arduino AVR core 1.8.x headers
// (HardwareSerial, TwoWire, EEPROMClass, analogWrite/analogRead). Nothing here models the electrical behaviour of the real board.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Mega 2560: A0 = pin 54; 16 analog inputs
static const uint8_t A8 = 62, A0 = 54, A1 = 55, A2 = 56, A3 = 57, A4 = 58, A5 = 59, A6 = 60, A7 = 61;
#define NOT_ON_TIMER 0
#define digitalPinToTimer(p) ((((p) >= 2 && (p) <= 13) || (p) == 44 || (p) == 45 || (p) == 46) ? 1 : 0)

#include "../Robosoccer26_Cyberbots/compat.h"

// ---- virtual time + signal model ------------------------------------------------------
static uint64_t g_mockUs = 5000000ULL;   // start at 5 s so nothing depends on t=0

// Receivers: analog input An (pin 54+n) reads a DC level plus, while the simulated ball burst is on
// (192 us of every 833 us, the RCJ ball's pattern as read from the ball firmware), that receiver's swing. Values are 10-bit (0..1023).
// Modules: D34 / D35 digital, LOW when g_mockModDetect[] is set and the module is active-low (the default model).
static uint16_t g_mockAnalog[70];
static uint8_t g_pinLevel[70];
static uint8_t g_pinMode[70];
static uint16_t g_mockRingAmp[16], g_mockRingBase[16];
static uint16_t g_mockRingNoise = 0;                 // +- counts of deterministic jitter added to every reading
static bool g_mockModDetect[2];
static bool g_mockModActiveHigh[2];
static bool g_mockUnderDetect = false, g_mockUnderActiveHigh = false;   // underside sensor on D40
static uint32_t g_mockAdcReads = 0;
static uint32_t g_mockRng = 12345;

// Serial TX model: 115200 8N1 ~ 11520 B/s into the AVR's 64-byte transmit buffer
static double g_serialRoom = 63;
static bool g_serialUnlimited = false;
static void mockAdvance(uint32_t us) {
  uint32_t left = us;
  while (left) {
    uint32_t step = left > 10 ? 10 : left;
    left -= step;
    g_mockUs += step;
    g_serialRoom = std::min(63.0, g_serialRoom + step * 0.01152);
  }
}
static inline uint32_t millis() { return (uint32_t)(g_mockUs / 1000ULL); }
static inline uint32_t micros() { return (uint32_t)g_mockUs; }
static inline void delay(uint32_t ms) { mockAdvance(ms * 1000u); }
static inline void delayMicroseconds(uint32_t us) { mockAdvance(us); }
static inline void yield() {}

// ---- GPIO / ADC / PWM ----------------------------------------------------------------------
static uint32_t g_pinFirstTouchMs[70];               // millis()+1 of the first pinMode / digitalWrite on each pin (0 = never touched)
static uint32_t g_pwmDuty[70];
static inline void mockTouch(uint8_t p) { if (!g_pinFirstTouchMs[p]) g_pinFirstTouchMs[p] = (uint32_t)(g_mockUs / 1000ULL) + 1u; }
static inline void pinMode(uint8_t p, uint8_t m) { mockTouch(p); g_pinMode[p] = m; }
static inline void digitalWrite(uint8_t p, uint8_t v) { mockTouch(p); g_pinLevel[p] = v ? 1 : 0; }
static inline void analogWrite(uint8_t p, int duty) { mockTouch(p); g_pwmDuty[p] = (uint32_t)(duty < 0 ? 0 : (duty > 255 ? 255 : duty)); }
static inline int digitalRead(uint8_t p) {
  if (p == 34 || p == 35) { bool d = g_mockModDetect[p - 34]; return (g_mockModActiveHigh[p - 34] ? d : !d) ? 1 : 0; }
  if (p == 40) return (g_mockUnderActiveHigh ? g_mockUnderDetect : !g_mockUnderDetect) ? 1 : 0;
  if (p == 20 || p == 21) return 1;
  return g_pinLevel[p];
}
static inline int analogRead(uint8_t p) {
  if (p < A0 || p > A0 + 7) return g_mockAnalog[p];
  g_mockAdcReads++;
  mockAdvance(112);                                  // one conversion takes about 112 us at the default ADC clock (prescaler 128)
  int ch = p - A0;
  bool burst = (uint32_t)(g_mockUs % 833ULL) < 192u;
  int v = (int)g_mockRingBase[ch] + (burst ? (int)g_mockRingAmp[ch] : 0);
  if (g_mockRingNoise) { g_mockRng = g_mockRng * 1664525u + 1013904223u; v += (int)((g_mockRng >> 16) % (2u * g_mockRingNoise + 1u)) - (int)g_mockRingNoise; }
  return v < 0 ? 0 : (v > 1023 ? 1023 : v);
}

// ---- Serial -----------------------------------------------------------------------------------
struct MockSerial {
  bool usb = true;                                  // the USB serial shares the 64-byte transmit model; Serial1 (the HC-05) has its own unlimited room
  std::string out;
  std::deque<uint8_t> in;
  bool begun = false;
  bool blocked = false;                             // Serial1 only: the transmit buffer never drains (no HC-05 listening / link down)
  unsigned long baud = 0;
  void begin(unsigned long b) { begun = true; baud = b; }
  int availableForWrite() { if (blocked) return 0; return (!usb || g_serialUnlimited) ? 4096 : (int)g_serialRoom; }
  size_t write(uint8_t b) {
    out.push_back((char)b);
    if (usb && !g_serialUnlimited) g_serialRoom = std::max(0.0, g_serialRoom - 1.0);
    return 1;
  }
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; int v = in.front(); in.pop_front(); return v; }
  void feed(const std::string& s) { for (unsigned char c : s) in.push_back(c); }
};
static MockSerial Serial;
static MockSerial Serial1 = [] { MockSerial s; s.usb = false; return s; }();

// ---- MPU6050 model on I2C ------------------------------------------------------------------------
struct MockMpu {
  bool present = true;
  uint8_t addr = 0x68;
  uint8_t whoami = 0x68;
  int mode = 0;            // 0 normal, 1 NACK, 2 frozen frame, 3 all-zero frame, 4 gyro saturated
  float gz = 0, gx = 0, gy = 0, azg = 1.0f, axg = 0, ayg = 0;
  uint8_t regs[128];
  uint32_t reads = 0, initWrites = 0;
  int16_t frozen[7];
  bool haveFrozen = false;
};
static MockMpu g_mpu;
struct MockWire {
  uint8_t cur = 0;
  std::vector<uint8_t> tx, rx;
  size_t rxPos = 0;
  uint8_t regPtr = 0;
  bool begun = false;
  int beginCalls = 0;
  uint32_t clock = 0, timeoutUs = 0;
  void begin() { begun = true; beginCalls++; }
  void end() { begun = false; }
  void setClock(uint32_t f) { clock = f; }
  void setWireTimeout(uint32_t t = 25000, bool = false) { timeoutUs = t; }
  void beginTransmission(uint8_t a) { cur = a; tx.clear(); }
  size_t write(uint8_t b) { tx.push_back(b); return 1; }
  uint8_t endTransmission(uint8_t stop = 1) {
    if (!g_mpu.present || cur != g_mpu.addr || g_mpu.mode == 1) return 2;
    if (tx.size() >= 2) { g_mpu.regs[tx[0] & 127] = tx[1]; g_mpu.initWrites++; }
    else if (tx.size() == 1) regPtr = tx[0];
    (void)stop;
    return 0;
  }
  uint8_t requestFrom(uint8_t a, uint8_t n, uint8_t) {
    rx.clear(); rxPos = 0;
    if (!g_mpu.present || a != g_mpu.addr || g_mpu.mode == 1) return 0;
    if (regPtr == 0x75) { rx.push_back(g_mpu.whoami); }
    else if (regPtr == 0x3B) {
      g_mpu.reads++;
      int16_t r[7];
      auto cl = [](float v) { return (int16_t)std::max(-32767.0f, std::min(32767.0f, v)); };
      r[0] = cl(g_mpu.axg * 8192.0f); r[1] = cl(g_mpu.ayg * 8192.0f); r[2] = cl(g_mpu.azg * 8192.0f); r[3] = 0x0800;
      r[4] = cl(g_mpu.gx * 65.5f); r[5] = cl(g_mpu.gy * 65.5f); r[6] = cl(g_mpu.gz * 65.5f);
      static uint32_t lcg = 1234567u;                       // sensor noise: a real MPU6050 never repeats a frame exactly
      for (int i = 0; i < 7; i++) { lcg = lcg * 1664525u + 1013904223u; if (i != 3) r[i] = (int16_t)(r[i] + (int)((lcg >> 24) % 7u) - 3); }
      if (g_mpu.mode == 3) memset(r, 0, sizeof(r));
      if (g_mpu.mode == 4) r[6] = -32768;
      if (g_mpu.mode == 2) { if (!g_mpu.haveFrozen) { memcpy(g_mpu.frozen, r, sizeof(r)); g_mpu.haveFrozen = true; } memcpy(r, g_mpu.frozen, sizeof(r)); }
      else g_mpu.haveFrozen = false;
      for (int i = 0; i < 7; i++) { rx.push_back((uint8_t)(r[i] >> 8)); rx.push_back((uint8_t)(r[i] & 0xFF)); }
    }
    while (rx.size() < n) rx.push_back(0);
    rx.resize(n);
    return n;
  }
  int available() { return (int)(rx.size() - rxPos); }
  int read() { return rxPos < rx.size() ? rx[rxPos++] : -1; }
};
static MockWire Wire;

// ---- EEPROM (4 KB, optionally persisted to a host file so a "reboot" = re-running the process keeps the settings) ----
static std::string g_eepromPath;
struct MockEEPROM {
  uint8_t mem[4096];
  bool loaded = false;
  void ensure() {
    if (loaded) return;
    loaded = true;
    memset(mem, 0xFF, sizeof(mem));                       // erased EEPROM reads 0xFF
    if (!g_eepromPath.empty()) { FILE* f = fopen(g_eepromPath.c_str(), "rb"); if (f) { if (fread(mem, 1, sizeof(mem), f) == 0) {} fclose(f); } }
  }
  void flush() { if (g_eepromPath.empty()) return; FILE* f = fopen(g_eepromPath.c_str(), "wb"); if (f) { fwrite(mem, 1, sizeof(mem), f); fclose(f); } }
  void reset() { loaded = false; }
  uint16_t length() { return 4096; }
  template <typename T> T& get(int addr, T& t) { ensure(); memcpy(&t, mem + addr, sizeof(T)); return t; }
  template <typename T> const T& put(int addr, const T& t) { ensure(); memcpy(mem + addr, &t, sizeof(T)); flush(); return t; }
  uint8_t read(int a) { ensure(); return mem[a]; }
  void update(int a, uint8_t v) { ensure(); mem[a] = v; flush(); }
};
static MockEEPROM EEPROM;
static bool g_mockAuthEnforced = false;          // read by authBegin() in the host build only
