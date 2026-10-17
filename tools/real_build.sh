#!/usr/bin/env bash
# Compile the ACTUAL sketch with the REAL Arduino-ESP32 cores and Xtensa GCC (not the host mock).
# Usage: tools/real_build.sh [2.0.17|3.0.7|both]
#
# Normal machine (internet access):
#   arduino-cli core install esp32:esp32@2.0.17   (and again with @3.0.7 in a separate --config-dir/data dir)
#   arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all Robosoccer26_Cyberbots
#
# The build sandbox used for development blocks Espressif/Arduino download hosts, so the toolchains
# were obtained from the public Docker image arduinoci/ci-arduino-esp32 (tags v2.0.17, v3.0.7, which
# contain arduino-cli + the matching esp32 core) by fetching its layers via mirror.gcr.io and
# extracting them. This script expects them extracted under $RS_TOOLCHAINS (default /tmp/img):
#   $RS_TOOLCHAINS/root_v2.0.17/{cli-0.35.3,root/.arduino15}   $RS_TOOLCHAINS/root_v3.0.7/{cli-1.1.1,root/.arduino15}
# esptool needs:  pip install pyserial reedsolo bitstring ecdsa pyyaml cryptography intelhex
set -u
T=${RS_TOOLCHAINS:-/tmp/img}
cd "$(dirname "$0")/.."
WHICH=${1:-both}
RC=0
build() {
  V=$1
  if [ "$V" = 2.0.17 ]; then CLI=$T/root_v2.0.17/cli-0.35.3/arduino-cli; D=$T/root_v2.0.17/root/.arduino15
  else CLI=$T/root_v3.0.7/cli-1.1.1/arduino-cli; D=$T/root_v3.0.7/root/.arduino15; fi
  echo "=== Arduino-ESP32 $V (real core, FQBN esp32:esp32:esp32 = ESP32 Dev Module, default 4MB-with-spiffs partitions) ==="
  mkdir -p "$T/user_$V" "$T/dl_$V"
  OUT=$(ARDUINO_DIRECTORIES_DATA=$D ARDUINO_DIRECTORIES_USER=$T/user_$V ARDUINO_DIRECTORIES_DOWNLOADS=$T/dl_$V \
        "$CLI" compile --fqbn esp32:esp32:esp32 --warnings all --build-path "$T/build_$V" --no-color Robosoccer26_Cyberbots 2>&1)
  CODE=$?
  echo "$OUT" | grep -E "warning|error|Sketch uses|Global variables"
  WARN=$(echo "$OUT" | grep -c "warning:")
  echo "exit=$CODE warnings=$WARN"
  [ $CODE -eq 0 ] && [ "$WARN" -eq 0 ] || RC=1
}
[ "$WHICH" = both ] || [ "$WHICH" = 2.0.17 ] && build 2.0.17
[ "$WHICH" = both ] || [ "$WHICH" = 3.0.7 ] && build 3.0.7
exit $RC
