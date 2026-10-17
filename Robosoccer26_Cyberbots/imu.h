// imu.h - MPU6050 over I2C (raw registers, no external library): bounded, validated, monitored.
// Heading is RELATIVE: integrated gyro Z since the reference was set. A gyro alone drifts, so the
// heading is only used to hold orientation while driving; it is never treated as absolute.
#pragma once
#include "config.h"
#include "health.h"

static const uint8_t MPU_REG_SMPLRT = 0x19, MPU_REG_CONFIG = 0x1A, MPU_REG_GYRO_CFG = 0x1B,
                     MPU_REG_ACCEL_CFG = 0x1C, MPU_REG_ACCEL_XOUT = 0x3B, MPU_REG_PWR1 = 0x6B,
                     MPU_REG_WHOAMI = 0x75;
static const float GYRO_LSB_PER_DPS = 65.5f;    // +/-500 dps range
static const float ACCEL_LSB_PER_G = 8192.0f;   // +/-4 g range
static const uint32_t IMU_READ_PERIOD_US = 5000;
static const uint32_t IMU_RETRY_MS = 2000;

struct ImuState {
  bool present;          // a device answered with a plausible WHO_AM_I
  bool ok;               // currently delivering fresh, valid data
  bool calibrated;       // gyro bias measured while stationary
  bool yawValid;
  uint8_t addr, whoami, consecErr, consecInvalid, goodStreak, calState;  // calState 0 idle,1 running,2 done,3 failed
  float gx, gy, gz, ax, ay, az;     // dps, g
  float yaw;                        // degrees, clockwise positive, relative, (-180,180]
  float bias;                       // dps
  float calSum, calSumSq;
  uint16_t calN;
  uint32_t lastGoodMs, lastUs, lastReadUs, lastTryMs;
  uint32_t samples, errCount, timeoutCount, invalidCount, recoveries, gaps, sameFrames, initAttempts;
  int16_t lastRaw[7];
  bool hadFault;
};
static ImuState g_imu;
static bool g_imuWireStarted = false;

static bool i2cWriteReg(uint8_t a, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(a);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}
static bool i2cReadRegs(uint8_t a, uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(a);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom(a, n, true);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

// If a slave holds SDA low (interrupted transfer), clock SCL up to 9 times to release it.
static void i2cBusRecover() {
  pinMode(PIN_I2C_SCL, OUTPUT);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(5);
  for (int i = 0; i < 9 && digitalRead(PIN_I2C_SDA) == LOW; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_I2C_SDA, OUTPUT);       // generate a STOP
  digitalWrite(PIN_I2C_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(5);
}

static bool whoAmIPlausible(uint8_t w) {
  // 0x68 = genuine MPU6050. 0x70/0x71/0x72/0x98 are common clone/related IDs with the same register map.
  return w == 0x68 || w == 0x70 || w == 0x71 || w == 0x72 || w == 0x98;
}

// Probe + configure. `full` adds a device reset (boot only: it waits 100 ms).
static bool imuInit(const Config& c, bool full) {
  g_imu.lastTryMs = millis();
  g_imu.initAttempts++;
  if (!full) logEvent("R", "imu_reinit_attempt");
  if (g_imuWireStarted) Wire.end();
  i2cBusRecover();
  Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL, 400000u);
  Wire.setTimeOut(20);                // ms; keeps a dead bus from stalling the loop
  g_imuWireStarted = true;
  uint8_t cand[2] = {0x68, 0x69};
  uint8_t nCand = 2;
  if (c.imuAddrPref) { cand[0] = c.imuAddrPref; nCand = 1; }
  bool found = false;
  g_imu.addr = 0;
  g_imu.whoami = 0;
  for (uint8_t i = 0; i < nCand && !found; i++) {
    uint8_t w = 0;
    if (i2cReadRegs(cand[i], MPU_REG_WHOAMI, &w, 1)) { g_imu.addr = cand[i]; g_imu.whoami = w; found = true; }
  }
  if (!found || !whoAmIPlausible(g_imu.whoami)) { g_imu.present = false; return false; }
  bool ok = true;
  if (full) { ok &= i2cWriteReg(g_imu.addr, MPU_REG_PWR1, 0x80); delay(100); }
  ok &= i2cWriteReg(g_imu.addr, MPU_REG_PWR1, 0x01);       // wake, PLL on gyro X
  ok &= i2cWriteReg(g_imu.addr, MPU_REG_SMPLRT, 4);        // 1 kHz / 5 = 200 Hz
  ok &= i2cWriteReg(g_imu.addr, MPU_REG_CONFIG, 0x03);     // DLPF ~42 Hz
  ok &= i2cWriteReg(g_imu.addr, MPU_REG_GYRO_CFG, 0x08);   // +/-500 dps
  ok &= i2cWriteReg(g_imu.addr, MPU_REG_ACCEL_CFG, 0x08);  // +/-4 g
  g_imu.present = ok;
  g_imu.ok = false;                                         // must prove itself with fresh valid samples
  g_imu.goodStreak = 0;
  g_imu.consecErr = g_imu.consecInvalid = 0;
  g_imu.lastReadUs = micros();
  g_imu.lastUs = 0;
  return ok;
}

static void imuStartCalibration() {
  g_imu.calState = 1; g_imu.calN = 0; g_imu.calSum = 0; g_imu.calSumSq = 0;
}

// Plausibility of one raw frame (7 x int16: ax ay az temp gx gy gz).
static bool imuFrameValid(const int16_t r[7]) {
  bool allZero = true, allFF = true;
  for (int i = 0; i < 7; i++) { if (r[i] != 0) allZero = false; if (r[i] != -1) allFF = false; }
  if (allZero || allFF) return false;                        // typical dead-bus patterns
  if (r[4] == -32768 || r[5] == -32768 || r[6] == -32768) return false;
  float gz = (float)r[6] / GYRO_LSB_PER_DPS;
  float gx = (float)r[4] / GYRO_LSB_PER_DPS, gy = (float)r[5] / GYRO_LSB_PER_DPS;
  if (absf(gz) > 490.0f || absf(gx) > 490.0f || absf(gy) > 490.0f) return false;   // at/over full scale
  float ax = (float)r[0] / ACCEL_LSB_PER_G, ay = (float)r[1] / ACCEL_LSB_PER_G, az = (float)r[2] / ACCEL_LSB_PER_G;
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  if (mag < 0.3f || mag > 3.9f) return false;                // implausible specific force
  return true;
}

static bool imuFresh(const Config& c, uint32_t nowMs) {
  return g_imu.ok && elapsedMs(nowMs, g_imu.lastGoodMs) <= c.imuStaleMs;
}
static bool imuHeadingUsable(const Config& c, uint32_t nowMs) {
  return c.hdgEnable && g_imu.present && g_imu.calibrated && g_imu.yawValid && imuFresh(c, nowMs);
}
static void imuZeroYaw() { g_imu.yaw = 0; g_imu.yawValid = g_imu.calibrated; }

// Call every loop. Reads at most every 5 ms; I2C timeouts are bounded by Wire.setTimeOut().
static void imuUpdate(const Config& c, uint32_t nowMs, bool stationary) {
  uint32_t nowUs = micros();
  if (!g_imu.present) {
    if (elapsedMs(nowMs, g_imu.lastTryMs) >= (g_imu.hadFault ? IMU_RETRY_MS : IMU_RETRY_MS * 2)) {   // lost: retry every 2 s, never found: every 4 s
      if (imuInit(c, false)) { g_imu.recoveries++; g_stats.imuRecoveries++; if (g_imu.calibrated == false) imuStartCalibration(); }
    }
    return;
  }
  if ((uint32_t)(nowUs - g_imu.lastReadUs) < IMU_READ_PERIOD_US) return;
  g_imu.lastReadUs = nowUs;

  uint8_t b[14];
  if (!i2cReadRegs(g_imu.addr, MPU_REG_ACCEL_XOUT, b, 14)) {
    g_imu.errCount++;
    if (g_imu.consecErr < 255) g_imu.consecErr++;
    g_imu.goodStreak = 0;
    if (g_imu.consecErr >= 3 && (g_imu.ok || !g_imu.hadFault)) {
      if (g_imu.ok) { g_imu.timeoutCount++; g_stats.imuTimeouts++; }
      g_imu.ok = false; g_imu.yawValid = false; g_imu.hadFault = true;
    }
    if (!g_imu.ok && elapsedMs(nowMs, g_imu.lastTryMs) >= IMU_RETRY_MS) imuInit(c, false);   // re-init attempt
    return;
  }
  g_imu.consecErr = 0;
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (int16_t)(((uint16_t)b[2 * i] << 8) | b[2 * i + 1]);
  bool same = memcmp(r, g_imu.lastRaw, sizeof(r)) == 0;
  memcpy(g_imu.lastRaw, r, sizeof(r));
  if (same) { if (g_imu.sameFrames < 100000) g_imu.sameFrames++; } else g_imu.sameFrames = 0;
  bool bad = !imuFrameValid(r) || g_imu.sameFrames >= 60;   // 60 identical 200 Hz frames = frozen sensor
  if (bad) {
    g_imu.invalidCount++;
    if (g_imu.consecInvalid < 255) g_imu.consecInvalid++;
    g_imu.goodStreak = 0;
    if (g_imu.consecInvalid >= 5 && g_imu.ok) { g_imu.ok = false; g_imu.yawValid = false; g_imu.hadFault = true; }
    return;                                               // an invalid sample is never integrated
  }
  g_imu.consecInvalid = 0;
  g_imu.ax = (float)r[0] / ACCEL_LSB_PER_G; g_imu.ay = (float)r[1] / ACCEL_LSB_PER_G; g_imu.az = (float)r[2] / ACCEL_LSB_PER_G;
  g_imu.gx = (float)r[4] / GYRO_LSB_PER_DPS; g_imu.gy = (float)r[5] / GYRO_LSB_PER_DPS; g_imu.gz = (float)r[6] / GYRO_LSB_PER_DPS;
  g_imu.samples++;
  g_imu.lastGoodMs = nowMs;
  if (g_imu.goodStreak < 255) g_imu.goodStreak++;

  float dt = g_imu.lastUs ? (float)(uint32_t)(nowUs - g_imu.lastUs) * 1e-6f : 0.0f;
  g_imu.lastUs = nowUs;

  if (!g_imu.ok && g_imu.goodStreak >= 20) {                // verified recovery: 20 consecutive good frames
    g_imu.ok = true;
    if (g_imu.hadFault) { g_imu.recoveries++; g_stats.imuRecoveries++; }
    g_imu.hadFault = false;
    g_imu.yaw = 0;                                          // reference lost during the outage
    g_imu.yawValid = g_imu.calibrated;
    return;
  }
  if (g_imu.calState == 1) {                                // stationary gyro bias measurement
    g_imu.calSum += g_imu.gz; g_imu.calSumSq += g_imu.gz * g_imu.gz; g_imu.calN++;
    if (g_imu.calN >= c.imuCalSamples) {
      float mean = g_imu.calSum / (float)g_imu.calN;
      float var = g_imu.calSumSq / (float)g_imu.calN - mean * mean;
      float sd = sqrtf(var > 0 ? var : 0);
      if (sd <= c.imuStillDps && absf(mean) < 20.0f) {
        g_imu.bias = mean; g_imu.calibrated = true; g_imu.calState = 2; g_imu.yaw = 0; g_imu.yawValid = true;
        say("INFO", "IMU gyro bias %.3f dps (sd %.3f) measured; heading reference zeroed", mean, sd);
      } else {
        g_imu.calState = 3;
        say("WARN", "IMU calibration rejected: robot not stationary (sd %.2f dps, mean %.2f). Heading hold stays off; run :gcal at rest.", sd, mean);
      }
    }
    return;
  }
  if (!g_imu.ok || !g_imu.calibrated) return;
  float rate = g_imu.gz - g_imu.bias;
  if (stationary && absf(rate) < 1.5f) g_imu.bias += 0.0005f * rate;   // slow drift tracking, only at rest
  if (dt > 0.0f && dt <= 0.05f) {
    g_imu.yaw = wrapDeg180(g_imu.yaw + (float)c.imuYawSign * rate * dt);
    g_imu.yawValid = true;
  } else if (dt > 0.05f) {
    g_imu.gaps++;                                           // long gap: do not integrate across it
  }
}

static void imuHealth(const Config& c, uint32_t nowMs) {
  (void)nowMs;
  faultUpdate(F_IMU_MISSING, !g_imu.present && !g_imu.hadFault, c.recoverVerifyMs,
              "no MPU6050 answered at 0x68/0x69 (check wiring, power, AD0)");
  char ev[72];
  snprintf(ev, sizeof(ev), "IMU not delivering data (errors=%lu timeouts=%lu)", UL(g_imu.errCount), UL(g_imu.timeoutCount));
  faultUpdate(F_IMU_TIMEOUT, g_imu.hadFault && !g_imu.ok && g_imu.consecInvalid < 5, c.recoverVerifyMs, ev);
  snprintf(ev, sizeof(ev), "IMU frames rejected as invalid/frozen (count=%lu)", UL(g_imu.invalidCount));
  faultUpdate(F_IMU_INVALID, g_imu.consecInvalid >= 5, c.recoverVerifyMs, ev);
}
