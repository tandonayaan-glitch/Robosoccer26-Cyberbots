#!/usr/bin/env bash
# Host-side verification: builds the sketch against a mock of the Arduino-ESP32 core (v3 and v2
# API shapes), with warnings and sanitizers, then runs every test case in its own process.
set -u
cd "$(dirname "$0")"
FAIL=0
TOTAL_T=0; TOTAL_P=0; TOTAL_C=0
rm -rf /tmp/mockfs /tmp/mockfs_*          # clean, reproducible start: no state from earlier runs
# Hardware that is gone for good must not come back into the firmware sources: the TSOP digital receivers, the old scan/learn engine,
# and the IR emitter-LED pulsing. (The multiplexer and the 12-receiver ring ARE the current hardware again.)
if grep -rniE "TSOP|TSSP|irscan|PIN_IR_EMIT|irEmit|\birLearn\b" ../Robosoccer26_Cyberbots/ ; then echo "OBSOLETE HARDWARE REFERENCE FOUND IN THE FIRMWARE SOURCES"; exit 1; fi
for CORE in 3 2; do
  BIN=/tmp/rs26_test_core$CORE
  echo "=== building (mock core v$CORE, -Wall -Wextra -Wshadow, ASan+UBSan) ==="
  g++ -std=c++20 -O1 -g -Wall -Wextra -Wshadow -Wformat=2 -fsanitize=address,undefined -fno-sanitize-recover=undefined \
      -DMOCK_CORE_V3=$([ $CORE = 3 ] && echo 1 || echo 0) -I. -o "$BIN" test_main.cpp || { echo "BUILD FAILED"; exit 1; }
  rm -rf /tmp/mockfs_reboot
  for T in $("$BIN" --list); do
    OUT=$("$BIN" "$T" 2>&1); RC=$?
    LAST=$(echo "$OUT" | tail -1)
    echo "[core v$CORE] $LAST"
    echo "$OUT" | grep -E "^  (shot|launch):"
    CK=$(echo "$LAST" | sed -nE 's/.*\(([0-9]+) checks.*/\1/p'); TOTAL_C=$((TOTAL_C + ${CK:-0})); TOTAL_T=$((TOTAL_T + 1))
    if [ $RC -ne 0 ]; then echo "$OUT" | head -40; FAIL=1; else TOTAL_P=$((TOTAL_P + 1)); fi
  done
done
echo "SUMMARY: $TOTAL_P/$TOTAL_T test runs passed ($((TOTAL_T / 2)) test cases x 2 core API shapes), $TOTAL_C checks executed in total"
[ $FAIL -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit $FAIL
