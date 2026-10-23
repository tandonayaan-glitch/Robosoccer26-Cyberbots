// mpu calibration - standalone MPU6050 calibration (Arduino Mega, I2C on D20/D21, no external library).
// Serial 115200. Place the robot FLAT (chip Z up) and perfectly STILL, then:
//   c = calibrate (gyro bias x/y/z, accel offsets, noise)   d = 30 s yaw drift test with that bias
//   Runs 'c' automatically at boot.
#include <Wire.h>

static const uint8_t REG_SMPLRT = 0x19, REG_CONFIG = 0x1A, REG_GYRO_CFG = 0x1B, REG_ACCEL_CFG = 0x1C,
                     REG_ACCEL_XOUT = 0x3B, REG_PWR1 = 0x6B, REG_WHOAMI = 0x75;
static const float GYRO_LSB_PER_DPS = 65.5f;   // +/-500 dps
static const float ACCEL_LSB_PER_G = 8192.0f;  // +/-4 g
static const uint16_t CAL_SAMPLES = 1000;
static const float STILL_MAX_DPS = 3.0f;       // reject calibration if gyro noise (std dev) is above this

static uint8_t addr = 0;
static float gBias[3], aOff[3];
static bool calibrated = false;

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n, (uint8_t)true) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool readRaw(int16_t* r) {  // ax ay az temp gx gy gz
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT, b, 14)) return false;
  for (uint8_t i = 0; i < 7; i++) r[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  return true;
}

static bool probe() {
  static const uint8_t cands[2] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++) {
    addr = cands[i];
    uint8_t who;
    if (readRegs(REG_WHOAMI, &who, 1)) {
      Serial.print(F("Device at 0x")); Serial.print(addr, HEX);
      Serial.print(F("  WHO_AM_I=0x")); Serial.println(who, HEX);
      return true;
    }
  }
  return false;
}

static void printVal(const __FlashStringHelper* name, float v, uint8_t dec) {
  Serial.print(name); Serial.println(v, dec);
}

static void calibrate() {
  Serial.println(F("Calibrating - keep flat and still..."));
  double sum[6] = {0}, sq[3] = {0};
  uint16_t n = 0, errs = 0;
  while (n < CAL_SAMPLES && errs < 200) {
    int16_t r[7];
    if (!readRaw(r)) { errs++; continue; }
    float a[3] = {r[0] / ACCEL_LSB_PER_G, r[1] / ACCEL_LSB_PER_G, r[2] / ACCEL_LSB_PER_G};
    float g[3] = {r[4] / GYRO_LSB_PER_DPS, r[5] / GYRO_LSB_PER_DPS, r[6] / GYRO_LSB_PER_DPS};
    for (uint8_t i = 0; i < 3; i++) { sum[i] += a[i]; sum[3 + i] += g[i]; sq[i] += (double)g[i] * g[i]; }
    n++;
    delay(5);
  }
  if (n < CAL_SAMPLES) { Serial.println(F("FAIL: too many I2C read errors")); return; }

  float noise[3];
  for (uint8_t i = 0; i < 3; i++) {
    float mean = sum[3 + i] / n;
    gBias[i] = mean;
    noise[i] = sqrt(max(0.0, sq[i] / n - (double)mean * mean));
  }
  float am[3] = {(float)(sum[0] / n), (float)(sum[1] / n), (float)(sum[2] / n)};
  aOff[0] = am[0];
  aOff[1] = am[1];
  aOff[2] = am[2] - 1.0f;  // Z up: expect +1 g

  float worst = max(noise[0], max(noise[1], noise[2]));
  Serial.println(F("--- result ---"));
  printVal(F("gyro bias X (dps): "), gBias[0], 3);
  printVal(F("gyro bias Y (dps): "), gBias[1], 3);
  printVal(F("gyro bias Z (dps): "), gBias[2], 3);
  printVal(F("gyro noise  Z (dps, std): "), noise[2], 3);
  printVal(F("accel offset X (g): "), aOff[0], 4);
  printVal(F("accel offset Y (g): "), aOff[1], 4);
  printVal(F("accel offset Z (g): "), aOff[2], 4);
  float mag = sqrt(am[0] * am[0] + am[1] * am[1] + am[2] * am[2]);
  printVal(F("accel magnitude (g, expect ~1.00): "), mag, 3);

  calibrated = worst <= STILL_MAX_DPS;
  if (!calibrated) Serial.println(F("REJECTED: robot was moving (gyro noise too high). Hold still and send 'c'."));
  else if (fabs(am[0]) > 0.15f || fabs(am[1]) > 0.15f || am[2] < 0.85f)
    Serial.println(F("WARN: chip is not flat / Z not up - accel offsets are not valid for a tilted chip."));
  else if (fabs(mag - 1.0f) > 0.1f) Serial.println(F("WARN: accel magnitude far from 1 g - check range/sensor."));
  if (calibrated) Serial.println(F("OK. Copy gyro bias Z into your code, or send 'd' to test drift."));
}

static void driftTest() {
  if (!calibrated) { Serial.println(F("Calibrate first ('c').")); return; }
  Serial.println(F("Drift test 30 s - do not move..."));
  float yaw = 0;
  uint32_t t0 = millis(), last = micros();
  while (millis() - t0 < 30000UL) {
    int16_t r[7];
    if (!readRaw(r)) continue;
    uint32_t now = micros();
    yaw -= (r[6] / GYRO_LSB_PER_DPS - gBias[2]) * (now - last) * 1e-6f;
    last = now;
  }
  printVal(F("yaw after 30 s (deg): "), yaw, 2);
  printVal(F("drift (deg/min): "), yaw * 2.0f, 2);
  Serial.println(fabs(yaw) < 1.0f ? F("GOOD") : F("HIGH drift - recalibrate warm/still, or check mounting/vibration"));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("=== MPU6050 calibration ==="));
  if (!probe()) {
    Serial.println(F("FAIL: no device at 0x68/0x69. Check VCC, GND, SDA=D20, SCL=D21."));
    while (true) delay(1000);
  }
  writeReg(REG_PWR1, 0x00); delay(100);
  writeReg(REG_PWR1, 0x01); delay(10);   // PLL with gyro X
  writeReg(REG_SMPLRT, 4);               // 200 Hz
  writeReg(REG_CONFIG, 3);               // ~44 Hz low-pass
  writeReg(REG_GYRO_CFG, 0x08);          // +/-500 dps
  writeReg(REG_ACCEL_CFG, 0x08);         // +/-4 g
  delay(500);                            // let the sensor settle
  calibrate();
  Serial.println(F("Commands: c = recalibrate, d = 30 s drift test"));
}

void loop() {
  if (!Serial.available()) return;
  char ch = Serial.read();
  if (ch == 'c') calibrate();
  else if (ch == 'd') driftTest();
}
