#!/usr/bin/env bash
# Type-checks remote_esp32/remote_esp32.ino against HOST STUBS of the ESP32 core (test/esp32_stub). It catches typos and type errors in the sketch and the shared headers.
# It is NOT an ESP32 build: the real core, the Bluetooth library and the linker are not involved. Build the sketch in the Arduino IDE / arduino-cli for the real check.
# If remote_esp32/secrets.h does not exist the placeholder template is used (never a real secret).
set -e
cd "$(dirname "$0")/.."
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
[ -f remote_esp32/secrets.h ] || cp remote_esp32/secrets.h.example "$T/secrets.h"
g++ -std=gnu++17 -fsyntax-only -Wall -Wextra -Itest/esp32_stub -I"$T" -Iremote_esp32 -x c++ remote_esp32/remote_esp32.ino
echo "ESP32 sketch: syntax/type check against host stubs PASSED (not a real ESP32 build)"
