#pragma once
#include "Arduino.h"
struct BluetoothSerial { bool begin(const char*, bool) { return true; } bool connect(const char*) { return false; } bool connected(int) { return false; } int available() { return 0; } int read() { return -1; } void print(const char*) {} void print(char) {} };
