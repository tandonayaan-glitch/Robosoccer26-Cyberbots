// pins.h - fixed hardware map. These are the verified wiring assignments from the robot
// specification. They are deliberately compile-time constants: nothing in the runtime
// configuration (Serial commands, persisted settings) can change a pin.
#pragma once
#include <stdint.h>

// ---- Motors (two L298N drivers) -------------------------------------------------
// Wheel index order used everywhere in the firmware.
enum WheelId { W_FL = 0, W_FR = 1, W_BL = 2, W_BR = 3, WHEEL_COUNT = 4 };

struct WheelPins {
  uint8_t in1;        // direction input A
  uint8_t in2;        // direction input B
  uint8_t en;         // PWM (L298N ENA/ENB)
  const char* name;
};

// Right L298N: IN1=25 IN2=26 (front-right, OUT1/2, ENA=33), IN3=27 IN4=14 (back-right, OUT3/4, ENB=32)
// Left  L298N: IN1=18 IN2=19 (front-left,  OUT1/2, ENA=23), IN3=21 IN4=22 (back-left,  OUT3/4, ENB=5)
static const WheelPins WHEEL_PINS[WHEEL_COUNT] = {
  /* FL */ {18, 19, 23, "FL"},
  /* FR */ {25, 26, 33, "FR"},
  /* BL */ {21, 22, 5,  "BL"},
  /* BR */ {27, 14, 32, "BR"},
};

// ---- CD74HC4067 16-channel multiplexer ------------------------------------------
static const uint8_t PIN_MUX_SIG = 34;   // input-only GPIO, no internal pull-up/down
static const uint8_t PIN_MUX_S0 = 4;
static const uint8_t PIN_MUX_S1 = 13;
static const uint8_t PIN_MUX_S2 = 15;    // strapping pin (MTDO): boot-log silencing
static const uint8_t PIN_MUX_S3 = 2;     // strapping pin: must not be pulled high during download-boot
static const uint8_t MUX_CHANNELS = 16;  // C0..C15
static const uint8_t IR_PLANNED_CHANNELS = 12;  // C0..C11 are the planned receivers

// ---- MPU6050 (I2C) ---------------------------------------------------------------
static const uint8_t PIN_I2C_SDA = 16;
static const uint8_t PIN_I2C_SCL = 17;

// Serial
static const uint32_t SERIAL_BAUD = 115200;
