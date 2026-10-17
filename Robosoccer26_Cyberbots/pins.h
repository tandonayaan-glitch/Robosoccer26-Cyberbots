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
#define P_FL_IN1 18
#define P_FL_IN2 19
#define P_FL_EN 23
#define P_FR_IN1 25
#define P_FR_IN2 26
#define P_FR_EN 33
#define P_BL_IN1 21
#define P_BL_IN2 22
#define P_BL_EN 5
#define P_BR_IN1 27
#define P_BR_IN2 14
#define P_BR_EN 32
static const WheelPins WHEEL_PINS[WHEEL_COUNT] = {
  /* FL */ {P_FL_IN1, P_FL_IN2, P_FL_EN, "FL"},
  /* FR */ {P_FR_IN1, P_FR_IN2, P_FR_EN, "FR"},
  /* BL */ {P_BL_IN1, P_BL_IN2, P_BL_EN, "BL"},
  /* BR */ {P_BR_IN1, P_BR_IN2, P_BR_EN, "BR"},
};

// ---- CD74HC4067 16-channel multiplexer ------------------------------------------
#define P_MUX_SIG 34
static const uint8_t PIN_MUX_SIG = P_MUX_SIG;   // input-only GPIO, no internal pull-up/down
static const uint8_t PIN_MUX_S0 = 4;
static const uint8_t PIN_MUX_S1 = 13;
static const uint8_t PIN_MUX_S2 = 15;    // strapping pin (MTDO): boot-log silencing
static const uint8_t PIN_MUX_S3 = 2;     // strapping pin: must not be pulled high during download-boot
static const uint8_t MUX_CHANNELS = 16;  // C0..C15
static const uint8_t IR_PLANNED_CHANNELS = 12;  // C0..C11 are the planned receivers

// ---- MPU6050 (I2C) ---------------------------------------------------------------
#define P_SDA 16
#define P_SCL 17
static const uint8_t PIN_I2C_SDA = P_SDA;
static const uint8_t PIN_I2C_SCL = P_SCL;

// Serial
static const uint32_t SERIAL_BAUD = 115200;

// ---- compile-time pin sanity (evaluated by the real toolchain against the real SoC headers) -----
#define PIN_BIT(p) (1ULL << (p))
#define OUTPUT_PIN_MASK                                                                              \
  (PIN_BIT(P_FL_IN1) | PIN_BIT(P_FL_IN2) | PIN_BIT(P_FL_EN) | PIN_BIT(P_FR_IN1) | PIN_BIT(P_FR_IN2) | \
   PIN_BIT(P_FR_EN) | PIN_BIT(P_BL_IN1) | PIN_BIT(P_BL_IN2) | PIN_BIT(P_BL_EN) | PIN_BIT(P_BR_IN1) |  \
   PIN_BIT(P_BR_IN2) | PIN_BIT(P_BR_EN) | PIN_BIT(4) | PIN_BIT(13) | PIN_BIT(15) | PIN_BIT(2) |        \
   PIN_BIT(P_SDA) | PIN_BIT(P_SCL))
static_assert(__builtin_popcountll(OUTPUT_PIN_MASK) == 18, "a GPIO is assigned twice (12 motor + 4 mux select + 2 I2C)");
static_assert((PIN_BIT(P_MUX_SIG) & OUTPUT_PIN_MASK) == 0, "mux SIG must not double as an output pin");
static_assert((OUTPUT_PIN_MASK & (0x3FULL << 6)) == 0, "GPIO6..11 are the SPI flash pins");
static_assert((OUTPUT_PIN_MASK & PIN_BIT(12)) == 0 && (OUTPUT_PIN_MASK & PIN_BIT(0)) == 0, "GPIO12 (flash voltage) and GPIO0 (boot mode) are unused on purpose");
#ifdef SOC_GPIO_VALID_OUTPUT_GPIO_MASK   // only visible on the real toolchain
static_assert((OUTPUT_PIN_MASK & ~SOC_GPIO_VALID_OUTPUT_GPIO_MASK) == 0, "an output pin is not output-capable on this SoC");
static_assert((PIN_BIT(P_MUX_SIG) & SOC_GPIO_VALID_GPIO_MASK) != 0 && (PIN_BIT(P_MUX_SIG) & SOC_GPIO_VALID_OUTPUT_GPIO_MASK) == 0,
              "mux SIG (GPIO34) must be a valid INPUT-ONLY pin");
#endif
