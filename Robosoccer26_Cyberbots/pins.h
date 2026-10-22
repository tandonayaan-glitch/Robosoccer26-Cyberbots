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

// ---- Front IR receivers (two 2-pin receivers, one GPIO each) and the two IR emitter LEDs -------------
// Both are input-only ADC1 pins. They have no internal pull-up/pull-down: whatever circuit makes a 2-pin
// photodiode/phototransistor produce a voltage (resistor to 3.3 V or GND) is external. Both can also be read as digital inputs.
#define P_IR0 35                                   // receiver 0: front CENTRE; decides "ball close enough to be in the shooting position"
#define P_IR1 36                                   // receiver 1: front, slightly OFFSET to one side; helps judge centred / off-centre
static const uint8_t PIN_IR[2] = {P_IR0, P_IR1};
static const uint8_t IR_SENSORS = 2;

// IR TRANSMITTER LEDs (one per receiver; the two pairs look forward from the front of the robot, reflected-light detection).
// PROPOSED pins, NOT taken from the project wiring notes: nothing in the notes assigns a GPIO to the emitters, so these two are free pins
// chosen by this firmware and must be confirmed against how you actually wire the LED drivers. The firmware NEVER touches them unless
// the operator enables emitter pulsing (':set iremit 1'); with the default iremit = 0 the emitters are assumed to be hard-wired on
// (or driven by something else) and the pins stay unused inputs. A GPIO cannot drive an IR LED directly (see docs/IR_SENSORS.md): use a transistor.
#define P_IR_EMIT0 4                               // emitter of pair 0 (next to sensor 0, front centre)
#define P_IR_EMIT1 13                              // emitter of pair 1 (next to sensor 1, front, offset)
static const uint8_t PIN_IR_EMIT[2] = {P_IR_EMIT0, P_IR_EMIT1};

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
static_assert(__builtin_popcountll(OUTPUT_PIN_MASK) == 14, "a GPIO is assigned twice (12 motor + 2 I2C)");
#define IR_PIN_MASK (PIN_BIT(P_IR0) | PIN_BIT(P_IR1))
static_assert(__builtin_popcountll(IR_PIN_MASK) == 2, "the two IR sensors must use different pins");
static_assert((IR_PIN_MASK & OUTPUT_PIN_MASK) == 0, "an IR sensor pin must not be a motor or I2C pin");
#define IR_EMIT_PIN_MASK (PIN_BIT(P_IR_EMIT0) | PIN_BIT(P_IR_EMIT1))
static_assert(__builtin_popcountll(IR_EMIT_PIN_MASK) == 2, "the two IR emitter pins must be different");
static_assert((IR_EMIT_PIN_MASK & (OUTPUT_PIN_MASK | IR_PIN_MASK)) == 0, "an IR emitter pin must not be a motor, I2C or IR receiver pin");
static_assert((IR_EMIT_PIN_MASK & ((0x3FULL << 6) | PIN_BIT(0) | PIN_BIT(12))) == 0, "emitter pins must not be flash pins, GPIO0 or GPIO12");
static_assert((OUTPUT_PIN_MASK & (0x3FULL << 6)) == 0, "GPIO6..11 are the SPI flash pins");
static_assert((OUTPUT_PIN_MASK & PIN_BIT(12)) == 0 && (OUTPUT_PIN_MASK & PIN_BIT(0)) == 0, "GPIO12 (flash voltage) and GPIO0 (boot mode) are unused on purpose");
#ifdef SOC_GPIO_VALID_OUTPUT_GPIO_MASK   // only visible on the real toolchain
static_assert((OUTPUT_PIN_MASK & ~SOC_GPIO_VALID_OUTPUT_GPIO_MASK) == 0, "an output pin is not output-capable on this SoC");
static_assert((IR_EMIT_PIN_MASK & ~SOC_GPIO_VALID_OUTPUT_GPIO_MASK) == 0, "an IR emitter pin is not output-capable on this SoC");
static_assert((IR_PIN_MASK & SOC_GPIO_VALID_GPIO_MASK) == IR_PIN_MASK && (IR_PIN_MASK & SOC_GPIO_VALID_OUTPUT_GPIO_MASK) == 0,
              "the IR sensor pins (GPIO35, GPIO36) must be valid INPUT-ONLY GPIOs");
#endif
