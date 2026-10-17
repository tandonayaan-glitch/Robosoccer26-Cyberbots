#!/usr/bin/env bash
# Host-side verification: builds the sketch against a mock of the Arduino-ESP32 core (v3 and v2
# API shapes), with warnings and sanitizers, then runs every test case in its own process.
set -u
cd "$(dirname "$0")"
FAIL=0
for CORE in 3 2; do
  BIN=/tmp/rs26_test_core$CORE
  echo "=== building (mock core v$CORE, -Wall -Wextra -Wshadow, ASan+UBSan) ==="
  g++ -std=c++17 -O1 -g -Wall -Wextra -Wshadow -Wformat=2 -fsanitize=address,undefined -fno-sanitize-recover=undefined \
      -DMOCK_CORE_V3=$([ $CORE = 3 ] && echo 1 || echo 0) -I. -o "$BIN" test_main.cpp || { echo "BUILD FAILED"; exit 1; }
  rm -rf /tmp/mockfs_reboot
  for T in $("$BIN" --list); do
    if [ "$T" = "t_reboot_phase2" ]; then :; fi
    OUT=$("$BIN" "$T" 2>&1); RC=$?
    echo "[core v$CORE] $(echo "$OUT" | tail -1)"
    if [ $RC -ne 0 ]; then echo "$OUT" | head -40; FAIL=1; fi
  done
done
[ $FAIL -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit $FAIL
