// Minimal stand-ins for the ESP32 Arduino core: lets the remote sketch be SYNTAX/TYPE checked on the host. This is NOT the real core and proves nothing about the ESP32 build.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
#define CONFIG_BT_ENABLED 1
#define CONFIG_BLUEDROID_ENABLED 1
#define CONFIG_BT_SPP_ENABLED 1
struct HWSerial { void begin(unsigned long) {} int available() { return 0; } int read() { return -1; } void print(const char*) {} void print(unsigned long) {} void print(long) {} void print(int) {} void print(unsigned) {} void print(char) {} void println(const char*) {} void println(unsigned long) {} void println(long) {} void println(int) {} void println() {} void write(char) {} void printf(const char*, ...) {} };
static HWSerial Serial;
static inline void pinMode(uint8_t, uint8_t) {}
static inline int digitalRead(uint8_t) { return 1; }
static inline uint32_t millis() { return 0; }
static inline void delay(uint32_t) {}
static inline void esp_fill_random(uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) b[i] = (uint8_t)i; }
