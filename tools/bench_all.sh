#!/usr/bin/env bash
# Rebuild the host benchmark and regenerate the raw benchmark outputs under docs/bench/.
#   tools/bench_all.sh [seed ...]          (default seeds: 12345 22345 32345)
# Everything here is SIMULATION on the host (virtual clock, mock Arduino core): no robot measurement.
# One full run takes about a minute per engine and seed.
set -eu
cd "$(dirname "$0")/.."
SEEDS=${*:-12345 22345 32345}
BIN=${RS_BENCH_BIN:-/tmp/rs26_bench_all}
g++ -std=c++20 -O2 -DMOCK_CORE_V3=1 -Itest -o "$BIN" test/test_main.cpp
mkdir -p docs/bench
for S in $SEEDS; do
  "$BIN" --bench engine=0 losthold=500 losthold_max=500 flickerhold=500 seed="$S" > "docs/bench/v3_engine0_legacy_seed${S}.txt"
  "$BIN" --bench seed="$S"                                              > "docs/bench/v3_engine2_default_seed${S}.txt"
  "$BIN" --bench seed="$S" suite=imm engine=2                           > "docs/bench/v3_engine2_burst_profile_seed${S}.txt"
  "$BIN" --bench seed="$S" suite=imm engine=0 losthold=500 losthold_max=500 flickerhold=500 > "docs/bench/v3_engine0_burst_profile_seed${S}.txt"
done
"$BIN" --bench suite=period engine=2 > docs/bench/v3_engine2_period_sweep.txt
"$BIN" --bench suite=period engine=0 losthold=500 losthold_max=500 flickerhold=500 > docs/bench/v3_engine0_period_sweep.txt
"$BIN" --bench suite=speed engine=2 > docs/bench/v3_engine2_approach_speed.txt
echo "wrote docs/bench/v3_*.txt"
