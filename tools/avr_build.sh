#!/usr/bin/env bash
# Compile the ACTUAL sketch for the Arduino Mega 2560 (ATmega2560) with the REAL Arduino AVR core 1.8.x and avr-gcc, and report flash / RAM use.
# This script reproduces what the Arduino IDE / arduino-cli does (same flags as the core's platform.txt), without needing the IDE:
#
#   Debian/Ubuntu:   sudo apt-get install gcc-avr avr-libc binutils-avr arduino-core-avr      (core 1.8.6, avr-gcc 7.3)
#   Arduino CLI:     arduino-cli core install arduino:avr && arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 Robosoccer26_Cyberbots
#
# Usage: tools/avr_build.sh [-v]       Environment: AVR_CORE (default /usr/share/arduino/hardware/arduino/avr), AVR_OUT (default /tmp/avr_build)
set -u
cd "$(dirname "$0")/.."
CORE=${AVR_CORE:-/usr/share/arduino/hardware/arduino/avr}
OUT=${AVR_OUT:-/tmp/avr_build}
MCU=atmega2560
DEFS="-DF_CPU=16000000L -DARDUINO=10819 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR"
INC="-I$CORE/cores/arduino -I$CORE/variants/mega -I$CORE/libraries/Wire/src -I$CORE/libraries/EEPROM/src"
CFLAGS="-c -g -Os -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=$MCU $DEFS"
CXXFLAGS="-c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=$MCU $DEFS"
mkdir -p "$OUT/core" "$OUT/lib" "$OUT/sketch"
command -v avr-g++ >/dev/null || { echo "avr-g++ not found: install gcc-avr (see the header of this script)"; exit 2; }
[ -d "$CORE/cores/arduino" ] || { echo "Arduino AVR core not found at $CORE (set AVR_CORE)"; exit 2; }
echo "=== Arduino Mega 2560 (ATmega2560, 16 MHz): avr-gcc $(avr-gcc -dumpversion), Arduino AVR core at $CORE ==="
OBJS=()
build_core() {  # compile core + Wire once (cached by timestamp)
  for f in "$CORE"/cores/arduino/*.c "$CORE"/cores/arduino/*.cpp "$CORE"/cores/arduino/*.S "$CORE"/libraries/Wire/src/*.cpp "$CORE"/libraries/Wire/src/utility/*.c; do
    [ -f "$f" ] || continue
    o="$OUT/core/$(basename "$f").o"
    if [ ! -f "$o" ] || [ "$f" -nt "$o" ]; then
      case "$f" in
        *.c) avr-gcc $CFLAGS $INC "$f" -o "$o" || exit 1 ;;
        *.cpp) avr-g++ $CXXFLAGS $INC "$f" -o "$o" || exit 1 ;;
        *.S) avr-gcc -c -g -x assembler-with-cpp -flto -MMD -mmcu=$MCU $DEFS $INC "$f" -o "$o" || exit 1 ;;
      esac
    fi
    OBJS+=("$o")
  done
}
build_core
# The sketch: what the Arduino builder does with a .ino (prepend Arduino.h, compile as C++). Warnings ON for our own code.
# Use the sketch include directory below: a generated /c/... include path from
# Git Bash is not understood inside source code by the native Windows compiler.
printf '#include <Arduino.h>\n#include "Robosoccer26_Cyberbots.ino"\n' > "$OUT/sketch/sketch.cpp"
LOG="$OUT/sketch.log"
avr-g++ $CXXFLAGS -Wall -Wextra $INC -I"$PWD/Robosoccer26_Cyberbots" "$OUT/sketch/sketch.cpp" -o "$OUT/sketch/sketch.o" >"$LOG" 2>&1
RC=$?
[ "${1:-}" = "-v" ] && cat "$LOG" || grep -E "warning|error" "$LOG" | head -40
WARN=$(grep -c "warning:" "$LOG")
if [ $RC -ne 0 ]; then echo "COMPILE FAILED (exit $RC)"; exit 1; fi
avr-gcc -Wall -Wextra -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=$MCU -o "$OUT/sketch.elf" "$OUT/sketch/sketch.o" "${OBJS[@]}" -lm >>"$LOG" 2>&1 || { tail -20 "$LOG"; echo "LINK FAILED"; exit 1; }
avr-objcopy -O ihex -R .eeprom "$OUT/sketch.elf" "$OUT/sketch.hex"
read -r TEXT DATA BSS _ < <(avr-size -A "$OUT/sketch.elf" | awk '/^\.text/{t=$2} /^\.data/{d=$2} /^\.bss/{b=$2} /^\.noinit/{n=$2} END{print t+0, d+0, b+0, n+0}')
FLASH=$((TEXT + DATA)); RAM=$((DATA + BSS))
echo "Sketch uses $FLASH bytes ($((FLASH * 100 / 253952))%) of program storage space. Maximum is 253952 bytes (256 KB minus the 8 KB bootloader)."
echo "Global variables use $RAM bytes ($((RAM * 100 / 8192))%) of dynamic memory, leaving $((8192 - RAM)) bytes for the stack. Maximum is 8192 bytes."
echo "exit=0 warnings=$WARN   (hex: $OUT/sketch.hex - NOT uploaded)"
[ "$WARN" -eq 0 ] || exit 1
[ "$RAM" -le 6000 ] || { echo "WARNING: static RAM above 6000 bytes leaves little room for the stack"; }
