// pins.h - fixed hardware map for the ARDUINO MEGA 2560 (ATmega2560). Pin numbers are the Arduino "D" numbers (D22 = 22, A0 = analog 0).
// They are compile-time constants: nothing in the runtime configuration (Serial commands, EEPROM settings) can change a pin.
// (The ESP32-S version of this project is kept in git: commit 1963a3f.)
#pragma once
#include <stdint.h>

// ---- Motors (two L298N drivers) -------------------------------------------------
// Wheel index order used everywhere in the firmware.
enum WheelId { W_FL = 0, W_FR = 1, W_BL = 2, W_BR = 3, WHEEL_COUNT = 4 };

struct WheelPins {
  uint8_t in1;        // direction input A
  uint8_t in2;        // direction input B
  uint8_t en;         // PWM (L298N ENA/ENB) - must be a Mega PWM pin
  const char* name;
};

// Right L298N: IN1=D22 IN2=D23 (front-right, OUT1/OUT2, ENA=D5), IN3=D24 IN4=D25 (back-right, OUT3/OUT4, ENB=D6)
// Left  L298N: IN1=D26 IN2=D27 (front-left,  OUT1/OUT2, ENA=D9), IN3=D28 IN4=D29 (back-left,  OUT3/OUT4, ENB=D10)
#define P_FR_IN1 22
#define P_FR_IN2 23
#define P_FR_EN 5
#define P_BR_IN1 24
#define P_BR_IN2 25
#define P_BR_EN 6
#define P_FL_IN1 26
#define P_FL_IN2 27
#define P_FL_EN 9
#define P_BL_IN1 28
#define P_BL_IN2 29
#define P_BL_EN 10
static const WheelPins WHEEL_PINS[WHEEL_COUNT] = {
  /* FL */ {P_FL_IN1, P_FL_IN2, P_FL_EN, "FL"},
  /* FR */ {P_FR_IN1, P_FR_IN2, P_FR_EN, "FR"},
  /* BL */ {P_BL_IN1, P_BL_IN2, P_BL_EN, "BL"},
  /* BR */ {P_BR_IN1, P_BR_IN2, P_BR_EN, "BR"},
};

// ---- Ball receivers: 5 mm phototransistors, each on its OWN analog input ---------------------------------------
// Circuit per receiver: collector -> regulated 5 V, emitter -> signal junction, 10 kOhm from the junction to GND, junction -> the analog input.
// Receiver 0 = A0 (first receiver), receiver 1 = A1 (second receiver). A2..A7 are reserved for more receivers (off until fitted and switched on with ':ring on <n>').
// The ATmega2560 ADC is 10-bit (readings 0..1023, 5 V reference). Every receiver has its own wire and its own load resistor: the signals are never joined.
#define P_RECV0 A0
#define P_RECV1 A1
#define P_RECV2 A2
#define P_RECV3 A3
#define P_RECV4 A4
#define P_RECV5 A5
#define P_RECV6 A6
#define P_RECV7 A7
static const uint8_t PIN_RECV[8] = {P_RECV0, P_RECV1, P_RECV2, P_RECV3, P_RECV4, P_RECV5, P_RECV6, P_RECV7};
static const uint8_t RING_CH_MAX = 8;              // analog inputs A0..A7 are supported
static const uint8_t RING_CH_FIXED = 2;            // A0 and A1 are fitted; A2..A7 are optional

// ---- Two IR obstacle modules (3-pin: VCC / GND / OUT, own IR LED + photodiode + LM393, digital output) ---------
// Module 0 sits at the front and reports "the ball is in the right position to shoot" (trimmer sets the distance). Module 1 is the second module
// (position undecided): it is read and logged only. The active level (HIGH or LOW when something is detected) is a SETTING per module (':mod level'),
// it is not assumed.
#define P_MOD0 34                                  // front module OUT
#define P_MOD1 35                                  // second module OUT
static const uint8_t PIN_MOD[2] = {P_MOD0, P_MOD1};
static const uint8_t MOD_COUNT = 2;

// ---- MPU6050 (hardware I2C, Wire library) -----------------------------------------
#define P_SDA 20
#define P_SCL 21
static const uint8_t PIN_I2C_SDA = P_SDA;
static const uint8_t PIN_I2C_SCL = P_SCL;

// Serial (USB, 115200 baud) uses D0/D1 (hardware UART0): nothing else may use them.
static const uint32_t SERIAL_BAUD = 115200;

// ---- compile-time pin sanity ----------------------------------------------------------
#define PIN_BIT(p) (1ULL << (p))
#define MOTOR_DIR_MASK                                                                                \
  (PIN_BIT(P_FL_IN1) | PIN_BIT(P_FL_IN2) | PIN_BIT(P_FR_IN1) | PIN_BIT(P_FR_IN2) |                   \
   PIN_BIT(P_BL_IN1) | PIN_BIT(P_BL_IN2) | PIN_BIT(P_BR_IN1) | PIN_BIT(P_BR_IN2))
#define MOTOR_PWM_MASK (PIN_BIT(P_FL_EN) | PIN_BIT(P_FR_EN) | PIN_BIT(P_BL_EN) | PIN_BIT(P_BR_EN))
#define RECV_MASK (PIN_BIT(P_RECV0) | PIN_BIT(P_RECV1) | PIN_BIT(P_RECV2) | PIN_BIT(P_RECV3) | PIN_BIT(P_RECV4) | PIN_BIT(P_RECV5) | PIN_BIT(P_RECV6) | PIN_BIT(P_RECV7))
#define MOD_MASK (PIN_BIT(P_MOD0) | PIN_BIT(P_MOD1))
#define I2C_MASK (PIN_BIT(P_SDA) | PIN_BIT(P_SCL))
#define SERIAL_MASK (PIN_BIT(0) | PIN_BIT(1))
// Arduino Mega 2560 PWM pins: D2..D13, D44, D45, D46
#define MEGA_PWM_MASK (((1ULL << 14) - (1ULL << 2)) | PIN_BIT(44) | PIN_BIT(45) | PIN_BIT(46))
// All digital pins used, counted once each: 8 direction + 4 PWM + 2 modules + 2 I2C + D0/D1 serial = 18 distinct pins; the receivers use the analog inputs A0..A7
#define ALL_DIGITAL_MASK (MOTOR_DIR_MASK | MOTOR_PWM_MASK | MOD_MASK | I2C_MASK | SERIAL_MASK)
static_assert(__builtin_popcountll(MOTOR_DIR_MASK) == 8, "the 8 motor direction pins must be distinct");
static_assert(__builtin_popcountll(MOTOR_PWM_MASK) == 4, "the 4 motor PWM pins must be distinct");
static_assert(__builtin_popcountll(RECV_MASK) == 8, "the 8 receiver analog inputs must be distinct");
static_assert((RECV_MASK & ALL_DIGITAL_MASK) == 0, "a receiver analog input collides with a digital function");
static_assert(__builtin_popcountll(MOD_MASK) == 2, "the 2 module pins must be distinct");
static_assert(__builtin_popcountll(ALL_DIGITAL_MASK) == 8 + 4 + 2 + 2 + 2, "a pin is assigned to two functions");
static_assert((MOTOR_PWM_MASK & ~MEGA_PWM_MASK) == 0, "every motor enable pin must be a PWM-capable Mega pin (D2..D13, D44..D46)");
static_assert((I2C_MASK == (PIN_BIT(20) | PIN_BIT(21))), "the Mega hardware I2C pins are D20 (SDA) and D21 (SCL)");
static_assert(P_RECV0 == 54 && P_RECV7 == 61, "the receivers must be on A0..A7");
