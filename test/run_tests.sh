#!/usr/bin/env bash
# Host-side verification: builds the sketch against a mock of the Arduino AVR core (Mega 2560 API shapes), with warnings and sanitizers,
# then runs every test case in its own process. This checks the LOGIC against simulated sensors; it says nothing about the physical robot.
# The REAL ATmega2560 compile (flash / RAM) is tools/avr_build.sh.
set -u
cd "$(dirname "$0")"
FAIL=0
TOTAL_T=0; TOTAL_P=0; TOTAL_C=0
rm -f /tmp/mockeeprom_*                       # clean, reproducible start: no state from earlier runs
# Hardware that is gone for good must not come back into the firmware sources: the ESP32 APIs and pin names, the TSOP digital receivers,
# the old scan engine, the emitter-LED pulsing, the flash filesystem.
if grep -rniE "ledc|esp_|CD74|4067|multiplex|PIN_MUX|P_MUX|MUX_|LittleFS|freertos|TSOP|TSSP|irscan|PIN_IR_EMIT|irEmit|\birLearn\b|GPIO[0-9]" ../Robosoccer26_Cyberbots/; then echo "OBSOLETE (ESP32 / old hardware) REFERENCE FOUND IN THE FIRMWARE SOURCES"; exit 1; fi
BIN=/tmp/rs26_test_mega
echo "=== building (mock AVR core, -Wall -Wextra -Wshadow, ASan+UBSan) ==="
g++ -std=c++20 -O1 -g -Wall -Wextra -Wshadow -Wformat=2 -fsanitize=address,undefined -fno-sanitize-recover=undefined \
    -I. -o "$BIN" test_main.cpp || { echo "BUILD FAILED"; exit 1; }
for T in $("$BIN" --list); do
  OUT=$(timeout 120 "$BIN" "$T" 2>&1); RC=$?
  LAST=$(echo "$OUT" | tail -1)
  echo "$LAST"
  echo "$OUT" | grep -E "^  (shot|launch):"
  CK=$(echo "$LAST" | sed -nE 's/.*\(([0-9]+) checks.*/\1/p'); TOTAL_C=$((TOTAL_C + ${CK:-0})); TOTAL_T=$((TOTAL_T + 1))
  if [ $RC -ne 0 ]; then echo "$OUT" | head -40; FAIL=1; else TOTAL_P=$((TOTAL_P + 1)); fi
done
echo "=== Python laptop client against the real firmware logic (host simulation) ==="
SIM=/tmp/rs26_sim
g++ -std=c++20 -O2 -I. -o "$SIM" firmware_sim.cpp || { echo "SIMULATOR BUILD FAILED"; exit 1; }
PYOUT=$(cd ../laptop/tests && RS26_SIM="$SIM" timeout 600 python3 -W ignore -m unittest discover -s . -p "test_*.py" 2>&1); PYRC=$?
echo "$PYOUT" | tail -4
if [ $PYRC -ne 0 ] || echo "$PYOUT" | grep -qi "skipped"; then echo "PYTHON TESTS FAILED OR SKIPPED"; FAIL=1; fi
echo "SUMMARY: $TOTAL_P/$TOTAL_T C++ test cases passed, $TOTAL_C checks executed in total"
[ $FAIL -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit $FAIL
