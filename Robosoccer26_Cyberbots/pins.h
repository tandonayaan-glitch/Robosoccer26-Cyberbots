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

// ---- Ball ring: CD74HC4067 16-channel analog multiplexer, one analog line (SIG) ------------------------------
// Wiring: VCC -> ESP32 3V3, GND -> common GND, EN -> GND (always enabled), SIG -> GPIO34 (analog input), S0 -> GPIO4, S1 -> GPIO13, S2 -> GPIO2, S3 -> GPIO15.
// C0..C11 = 12 phototransistor receivers (the ring, they see the ball's own infrared emission); C12..C15 = optional extra receivers
// (planned: left / right looking), enabled with ':ring on <ch>' once fitted.
//
// WHY S2/S3 ARE ON GPIO2 AND GPIO15 (and not on GPIO12 / GPIO0): the only free output-capable GPIOs are 0, 2, 12 and 15, and all four are boot-strapping pins.
// GPIO0 = LOW at reset selects the serial bootloader (the robot would not start), GPIO12 = HIGH at reset selects 1.8 V flash (the board would not boot).
// GPIO2 and GPIO15 are the harmless pair: GPIO15 only decides whether the boot ROM prints its log, and GPIO2 only matters in download mode and only
// when it is driven high. The select inputs of the CD74HC4067 are high-impedance, so they leave the reset levels of these pins unchanged. See docs/RING_AND_MODULES.md.
// The firmware does not touch any select pin until the ESP32 has finished booting (ringBegin(), after RING_BOOT_HOLDOFF_MS).
#define P_MUX_S0 4
#define P_MUX_S1 13
#define P_MUX_S2 2
#define P_MUX_S3 15
#define P_MUX_SIG 34                               // ADC1_CH6, input-only, no pull resistors
#define RING_BOOT_HOLDOFF_MS 500                   // select pins stay untouched (inputs) until this long after reset
static const uint8_t PIN_MUX_SEL[4] = {P_MUX_S0, P_MUX_S1, P_MUX_S2, P_MUX_S3};
static const uint8_t RING_CH_MAX = 16;             // the CD74HC4067 has 16 channels
static const uint8_t RING_CH_FIXED = 12;           // C0..C11 are fitted; C12..C15 are optional

// ---- Two IR obstacle modules (3-pin: VCC / GND / OUT, own IR LED + photodiode + LM393, 3.3 V supply, digital output) --------------
// Module 0 sits at the front and reports "the ball is in the right position to shoot" (trimmer sets the distance). Module 1 is not planned yet
// ("maybe the back"): it is read and logged only. GPIO34 is the multiplexer SIG line, so the modules cannot use it: GPIO35 and GPIO36 are the
// next free input-only pins (PROPOSAL: confirm against the real wiring). Input-only pins have no pull resistors; the module output must be driven.
#define P_MOD0 35                                  // front module: ball in the shooting position
#define P_MOD1 36                                  // second module (position undecided)
static const uint8_t PIN_MOD[2] = {P_MOD0, P_MOD1};
static const uint8_t MOD_COUNT = 2;

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
   PIN_BIT(P_BR_IN2) | PIN_BIT(P_BR_EN) | PIN_BIT(P_SDA) | PIN_BIT(P_SCL))
#define MUX_SEL_MASK (PIN_BIT(P_MUX_S0) | PIN_BIT(P_MUX_S1) | PIN_BIT(P_MUX_S2) | PIN_BIT(P_MUX_S3))
#define ALL_OUT_MASK (OUTPUT_PIN_MASK | MUX_SEL_MASK)
static_assert(__builtin_popcountll(OUTPUT_PIN_MASK) == 14, "a GPIO is assigned twice (12 motor + 2 I2C)");
static_assert(__builtin_popcountll(ALL_OUT_MASK) == 18, "a multiplexer select pin collides with a motor or I2C pin");
#define IN_ONLY_MASK (PIN_BIT(P_MUX_SIG) | PIN_BIT(P_MOD0) | PIN_BIT(P_MOD1))
static_assert(__builtin_popcountll(IN_ONLY_MASK) == 3, "the multiplexer SIG pin and the two module pins must all be different");
static_assert((IN_ONLY_MASK & ALL_OUT_MASK) == 0, "SIG / module pins must not be motor, I2C or multiplexer select pins");
static_assert((MUX_SEL_MASK & (PIN_BIT(0) | PIN_BIT(12))) == 0, "the multiplexer select pins must not be on GPIO0 (boot mode) or GPIO12 (flash voltage)");
static_assert((ALL_OUT_MASK & (0x3FULL << 6)) == 0 && (IN_ONLY_MASK & (0x3FULL << 6)) == 0, "GPIO6..11 are the SPI flash pins");
#ifdef SOC_GPIO_VALID_OUTPUT_GPIO_MASK   // only visible on the real toolchain
static_assert((ALL_OUT_MASK & ~SOC_GPIO_VALID_OUTPUT_GPIO_MASK) == 0, "an output pin is not output-capable on this SoC");
static_assert((IN_ONLY_MASK & SOC_GPIO_VALID_GPIO_MASK) == IN_ONLY_MASK && (IN_ONLY_MASK & SOC_GPIO_VALID_OUTPUT_GPIO_MASK) == 0,
              "the multiplexer SIG pin and the module pins (GPIO34, GPIO35, GPIO36) must be valid INPUT-ONLY GPIOs");
#endif
