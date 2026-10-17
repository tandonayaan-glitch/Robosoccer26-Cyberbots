#!/usr/bin/env bash
# Rebuild the host benchmark and regenerate the raw outputs under docs/bench/ (SIMULATION on the host: virtual clock,
# mock Arduino core; no robot measurement).
#   tools/bench_all.sh [seed ...]       default seeds: 12345 22345 32345
# Full runs take about a minute per engine and seed; the profile and sweep tables take several more minutes.
set -eu
cd "$(dirname "$0")/.."
SEEDS=${*:-12345 22345 32345}
export RS_BIN=${RS_BENCH_BIN:-/tmp/rs26_bench_all}
g++ -std=c++20 -O2 -DMOCK_CORE_V3=1 -Itest -o "$RS_BIN" test/test_main.cpp
mkdir -p docs/bench
LEGACY="engine=0 losthold=500 losthold_max=500 flickerhold=500"
for S in $SEEDS; do
  "$RS_BIN" --bench $LEGACY seed="$S" > "docs/bench/v3_engine0_legacy_seed${S}.txt"
  "$RS_BIN" --bench seed="$S"         > "docs/bench/v3_engine2_default_seed${S}.txt"
done
{
  echo "# deterministic burst-acceptance profile (no ball present, 96 phases per cell)"
  for CFG in "$LEGACY" "engine=2 acqspan1=96 acqspan2=30" "engine=2" "engine=2 acqspan1=180 acqspan2=100"; do
    echo "## $CFG"; "$RS_BIN" --bench suite=imm $CFG | grep "E\[accept"
  done
} > docs/bench/v3_burst_profile.txt
{
  echo "# one-line summaries per configuration (seed 12345); see tools/bench_summary.py for the columns"
  for CFG in "$LEGACY" "engine=2 acqspan1=96 acqspan2=30" "engine=2" "engine=2 acqspan1=180 acqspan2=100" \
             "engine=2 acqstreak=3" "engine=2 acqstreak=5" "engine=2 epidle=150" "engine=2 epidle=400" "engine=2 focus=0" \
             "engine=2 losseps=3" "engine=2 losseps=5" "engine=2 losthold=120" "engine=2 losthold=180" "engine=2 flickerhold=250" \
             "engine=2 reacq=0" "engine=2 reacq=500" "engine=2 dirwindows=2" "engine=2 dirwindows=4" "engine=2 dirtau=25" \
             "engine=2 dirtau=60" "engine=2 gatedeg=70" "engine=2 gatedeg=130" "engine=2 irwindow=2400" "engine=2 irwindow=4000"; do
    python3 tools/bench_summary.py $CFG
  done
} > docs/bench/v3_sweeps_seed12345.txt
for D in 600 1200 2400 4000; do
  echo "## decel=$D"; "$RS_BIN" --bench suite=loss decel=$D | sed -n '/STRONG track, ball disappears, 1 receiver/,/STRONG track, ball disappears, 2 receivers/p'
done > docs/bench/v3_decel_sensitivity.txt
echo "wrote docs/bench/v3_*.txt"
