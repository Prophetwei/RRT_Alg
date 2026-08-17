#!/usr/bin/env bash
# Run one RTL simulation containing MAP_COUNT maps x NSEEDS deterministic cases.
# Usage: MAPDIR=../../3D/map ./01_run_10maps.sh [START_SEED] [NSEEDS]
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

START_SEED=${1:-42}
NSEEDS=${2:-50}
MAPDIR=${MAPDIR:-../../3D/map}
MAP_COUNT=${MAP_COUNT:-10}
TIMEOUT=${TIMEOUT:-40000000}
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-rtl_results_seed${START_SEED}_${TIMESTAMP}.txt}
SIM_LOG=${SIM_LOG:-rtl_batch_seed${START_SEED}_${TIMESTAMP}.log}

for value in "$START_SEED" "$NSEEDS" "$MAP_COUNT" "$TIMEOUT"; do
    if [[ ! "$value" =~ ^[0-9]+$ ]]; then
        echo "ERROR: seed/count/timeout values must be non-negative integers" >&2
        exit 2
    fi
done
if (( NSEEDS < 1 || MAP_COUNT < 1 || TIMEOUT < 1 )); then
    echo "ERROR: NSEEDS, MAP_COUNT and TIMEOUT must be positive" >&2
    exit 2
fi

for ((i=0; i<MAP_COUNT; i++)); do
    if [[ ! -f "$MAPDIR/map_$i.txt" ]]; then
        echo "ERROR: missing $MAPDIR/map_$i.txt" >&2
        exit 2
    fi
done

END_SEED=$((START_SEED + NSEEDS - 1))
echo "==> RTL batch maps=0..$((MAP_COUNT - 1)) seeds=${START_SEED}..${END_SEED}"
echo "==> MAPDIR: $MAPDIR"
echo "==> timeout per case: $TIMEOUT cycles"
echo "==> results: $OUT"
echo "==> simulator log: $SIM_LOG"

./01_run.sh \
    +BATCH \
    "+MAPDIR=$MAPDIR" \
    "+START_SEED=$START_SEED" \
    "+NSEEDS=$NSEEDS" \
    "+MAP_COUNT=$MAP_COUNT" \
    "+TIMEOUT=$TIMEOUT" \
    "+OUT=$OUT" 2>&1 | tee "$SIM_LOG"

expected=$((MAP_COUNT * NSEEDS))
rows=$(awk '!/^#/ && NF { count++ } END { print count+0 }' "$OUT")
passes=$(awk '!/^#/ && NF && $3==1 { count++ } END { print count+0 }' "$OUT")
failures=$((rows - passes))

if (( rows != expected )); then
    echo "ERROR: expected $expected result rows, found $rows" >&2
    exit 3
fi

echo "==> completed: cases=$rows pass=$passes fail=$failures"
if (( failures != 0 )); then
    exit 1
fi
