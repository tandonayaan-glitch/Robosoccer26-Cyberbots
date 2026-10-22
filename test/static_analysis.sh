#!/usr/bin/env bash
# Static / dynamic analysis of the sketch on the host (mock AVR core). Complements, never replaces, tools/avr_build.sh (the real ATmega2560 compile).
cd "$(dirname "$0")/.."
SK=Robosoccer26_Cyberbots/Robosoccer26_Cyberbots.ino
echo "=== cppcheck (warning,style,performance,portability) ==="
cppcheck --enable=warning,style,performance,portability --std=c++17 --inline-suppr --suppress=missingIncludeSystem \
  --suppress=unusedFunction --suppress=unusedStructMember --suppress=memsetClassFloat --suppress=uninitMemberVarNoCtor \
  --suppress=knownConditionTrueFalse --suppress=cstyleCast --suppress=dangerousTypeCast \
  -DHOST_TEST -Itest $SK 2>&1 | grep -E "^Robosoccer26_Cyberbots|^test/" | grep -v "test/mock_arduino.h" ; echo "(end cppcheck)"
echo "=== clang-tidy (clang-analyzer + bugprone) ==="
clang-tidy -checks='-*,clang-analyzer-*,bugprone-*,-bugprone-easily-swappable-parameters,-bugprone-reserved-identifier,-clang-analyzer-security.insecureAPI.*,-bugprone-narrowing-conversions' \
  $SK -- -x c++ -std=c++17 -DHOST_TEST=1 -Itest 2>&1 | grep -E "Robosoccer26_Cyberbots/.*(warning|error)" | sort -u ; echo "(end clang-tidy)"
echo "=== valgrind memcheck (uninitialised reads, invalid accesses) ==="
g++ -std=c++17 -O1 -g -Itest -o /tmp/rs26_vg test/test_main.cpp || exit 1
for T in t_minifmt t_report_paced t_cli_robust t_diag_no_motion t_config t_ring_cli t_ring_saturation_and_faults t_modules t_nav_approach_and_stop t_nav_blind_spot_search t_goal_register_and_aim_window t_shoot_sequence; do
  valgrind -q --error-exitcode=9 --track-origins=yes /tmp/rs26_vg $T 2>&1 | grep -E "^(PASS|FAIL)|==[0-9]+==" | head -3
done
