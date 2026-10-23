// pins.h - fixed hardware map for the ARDUINO MEGA 2560 (ATmega2560). Pin numbers are the Arduino "D" numbers (D22 = 22, A0 = analog 0).
// They are compile-time constants: nothing in the runtime configuration (Serial commands, EEPROM settings) can change a pin.
// (An earlier controller version, on a different board, is kept in git: commit 1963a3f.)
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
// Receiver n = An for A0..A5 (six fitted receivers, each with its own 10 kOhm). A6, A7 are spare (off until fitted and switched on with ':ring on <n>'). Where each receiver LOOKS is not known to the
// firmware: the default angles 0,60,120,180,-120,-60 are an even-spacing guess, flagged `bearingsSet = 0` until you enter the real ones with ':ring bearing <n> <deg>'.
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
static const uint8_t RING_CH_FIXED = 6;            // A0..A5 are fitted; A6, A7 are spare

// ---- Two IR obstacle modules (3-pin: VCC / GND / OUT, own IR LED + photodiode + LM393, digital output) ---------
// Module 0 sits at the front and reports "the ball is in the right position to shoot" (trimmer sets the distance). Module 1 is the second module
// (position undecided): it is read and logged only. The active level (HIGH or LOW when something is detected) is a SETTING per module (':mod level'),
// it is not assumed.
#define P_MOD0 34                                  // the bottom / front alignment sensor OUT (the only one fitted)
#define P_MOD1 35                                  // second module OUT (NOT fitted, off by default)
static const uint8_t PIN_MOD[2] = {P_MOD0, P_MOD1};
static const uint8_t MOD_COUNT = 2;

// ---- HC-05 Bluetooth Classic module (robot side), on the Mega's hardware UART1 ---------------------------------------------------
// Serial1 = TX1 on D18, RX1 on D19. These two pins are FREE in this pin map (the front-left motor is on D26/D27; the "D18/D19" of an earlier board numbering was a
// different board), and D10 stays the back-left motor PWM pin. USB debugging (Serial on D0/D1) is unaffected.
//   Mega TX1 (D18, 5 V) ---[1 kOhm]---+--- HC-05 RXD   (the 2 kOhm below makes a 5 V -> 3.3 V divider: 5 x 2/3 = 3.33 V)
//                                     +---[2 kOhm]--- GND
//   HC-05 TXD (3.3 V) ------------------- Mega RX1 (D19)   (a 3.3 V HIGH is above the Mega's ~3.0 V input threshold)
//   HC-05 VCC / GND: see docs/BLUETOOTH.md (the supply depends on the breakout board); GND must be common with the Mega.
#define P_BT_TX 18
#define P_BT_RX 19
static const uint32_t BT_BAUD = 9600;               // HC-05 factory data-mode speed (AT mode is usually 38400)

// ---- Bottom IR sensor used for ball alignment: ONE 3-pin obstacle module (VCC 3.3 V / GND / OUT) on D34 --------------------------------
// There is a single such sensor on the robot and it is wired to D34 (it used to be D40). It is the same input as module 0 above: when it detects the ball the
// ball is in the alignment / shooting position (`held`; with `modneedring` 1 the receivers must also see the ball straight ahead). The separate "underside" vote
// (`underenable`) reads the same pin, so it is OFF by default (it would only count the same signal twice); P_UNDER is kept so `:raw` and `underenable 1` still work.
// Its ACTIVE LEVEL is a setting (`:mod level`, default LOW when detecting, check with ':raw'). Powered from 3.3 V: its OUT then swings 0..3.3 V, which the Mega reads as HIGH/LOW.
#define P_UNDER P_MOD0

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
#define BT_MASK (PIN_BIT(P_BT_TX) | PIN_BIT(P_BT_RX))
#define SERIAL_MASK (PIN_BIT(0) | PIN_BIT(1))
// Arduino Mega 2560 PWM pins: D2..D13, D44, D45, D46
#define MEGA_PWM_MASK (((1ULL << 14) - (1ULL << 2)) | PIN_BIT(44) | PIN_BIT(45) | PIN_BIT(46))
// All digital pins used, counted once each: 8 direction + 4 PWM + 2 modules + 2 I2C + D0/D1 serial + 2 Bluetooth (D18/D19); the bottom sensor shares D34 with module 0 = 20; the receivers use the analog inputs A0..A7
#define ALL_DIGITAL_MASK (MOTOR_DIR_MASK | MOTOR_PWM_MASK | MOD_MASK | I2C_MASK | SERIAL_MASK | BT_MASK)
static_assert(__builtin_popcountll(MOTOR_DIR_MASK) == 8, "the 8 motor direction pins must be distinct");
static_assert(__builtin_popcountll(MOTOR_PWM_MASK) == 4, "the 4 motor PWM pins must be distinct");
static_assert(__builtin_popcountll(RECV_MASK) == 8, "the 8 receiver analog inputs must be distinct");
static_assert((RECV_MASK & ALL_DIGITAL_MASK) == 0, "a receiver analog input collides with a digital function");
static_assert(__builtin_popcountll(MOD_MASK) == 2, "the 2 module pins must be distinct");
static_assert(__builtin_popcountll(ALL_DIGITAL_MASK) == 8 + 4 + 2 + 2 + 2 + 2, "a pin is assigned to two functions");
static_assert((MOTOR_PWM_MASK & ~MEGA_PWM_MASK) == 0, "every motor enable pin must be a PWM-capable Mega pin (D2..D13, D44..D46)");
static_assert((I2C_MASK == (PIN_BIT(20) | PIN_BIT(21))), "the Mega hardware I2C pins are D20 (SDA) and D21 (SCL)");
static_assert(P_BT_TX == 18 && P_BT_RX == 19, "HC-05 on the Mega UART1 pins TX1 = D18, RX1 = D19");
static_assert(P_UNDER == 34, "the single bottom alignment IR sensor is on D34");
static_assert(P_RECV0 == 54 && P_RECV7 == 61, "the receivers must be on A0..A7");
