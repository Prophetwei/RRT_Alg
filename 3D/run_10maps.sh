#!/usr/bin/env bash
# Paired benchmark for optimized and conventional 3D RRT implementations.
#
# Usage: ./run_10maps.sh [START_SEED] [REPEATS] [NSEEDS] [WARMUPS]
#
# REPEATS reruns the exact same (map, seed) workload to measure timing noise.
# Different algorithm samples come from NSEEDS, not from advancing the RNG
# between repeats.  opt_run_time/con_run_time are wall-clock medians.
set -euo pipefail

START_SEED=${1:-42}
REPEATS=${2:-5}
NSEEDS=${3:-50}
WARMUPS=${4:-1}
MAPDIR=${MAPDIR:-map}
MAP_COUNT=${MAP_COUNT:-10}
MAX_ITERATIONS=${MAX_ITERATIONS:-4000000}
CPU_CORE=${CPU_CORE:-}
CXX=${CXX:-g++}
STATIC_LIBSTDCXX=${STATIC_LIBSTDCXX:-1}
SKIP_BUILD=${SKIP_BUILD:-0}
PAIR_COOLDOWN=${PAIR_COOLDOWN:-0}
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-results_seed${START_SEED}_${TIMESTAMP}.txt}

if (( REPEATS < 1 || NSEEDS < 1 || WARMUPS < 0 || MAX_ITERATIONS < 1 || MAP_COUNT < 1 )); then
    echo "ERROR: repeats/nseeds/max_iterations must be positive and warmups non-negative" >&2
    exit 2
fi

for i in $(seq 0 $((MAP_COUNT - 1))); do
    map_file="$MAPDIR/map_$i.txt"
    path_file="$MAPDIR/map_${i}_guaranteed_path.txt"
    if [[ ! -f "$map_file" ]]; then
        echo "ERROR: missing $map_file" >&2
        echo "Generate maps explicitly, e.g. python3 map_generator.py --density 70 --seed 1000" >&2
        exit 2
    fi
    if [[ ! -f "$path_file" ]]; then
        echo "ERROR: missing path certificate $path_file" >&2
        echo "Regenerate maps with the updated map_generator.py before benchmarking." >&2
        exit 2
    fi
    if ! awk '
        function abs(value) { return value < 0 ? -value : value }
        NR==1 { first=($1==0 && $2==0 && $3==0); valid=1;
                px=$1; py=$2; pz=$3; rows=NR; next }
        { if (abs($1-px)+abs($2-py)+abs($3-pz)!=1) valid=0;
          px=$1; py=$2; pz=$3; rows=NR }
        END { exit !(first && valid && px==255 && py==255 && pz==255 && rows==766) }
    ' "$path_file"; then
        echo "ERROR: invalid guaranteed path certificate $path_file" >&2
        exit 2
    fi
done

if [[ "$SKIP_BUILD" != "1" ]]; then
    flags=(-O3 -DNDEBUG -std=c++17)
    if [[ "$STATIC_LIBSTDCXX" == "1" && "$(uname -s)" == "Linux" ]]; then
        flags+=(-static-libstdc++)
    fi
    echo "==> compiling once with $CXX ${flags[*]}"
    "$CXX" "${flags[@]}" rrt.cpp -o rrt
    "$CXX" "${flags[@]}" rrt_con.cpp -o rrt_con
fi

run_model() {
    if [[ -n "$CPU_CORE" ]]; then
        if ! command -v taskset >/dev/null 2>&1; then
            echo "ERROR: CPU_CORE requires Linux taskset" >&2
            return 2
        fi
        taskset -c "$CPU_CORE" "$@"
    else
        "$@"
    fi
}

summary_field() {
    local output=$1
    local key=$2
    awk -v wanted="$key" '
        $1=="SUMMARY" {
            for (i=2; i<=NF; i++) {
                split($i, item, "=")
                if (item[1]==wanted) { print item[2]; exit }
            }
        }
    ' <<< "$output"
}

run_one() {
    local binary=$1
    local map_file=$2
    local seed=$3
    run_model "$binary" "$map_file" "$REPEATS" "$seed" "$WARMUPS" 0 "$MAX_ITERATIONS" 1
}

END_SEED=$((START_SEED + NSEEDS - 1))
echo "==> seeds ${START_SEED}..${END_SEED}, repeats=$REPEATS, warmups=$WARMUPS"
echo "==> common max_iterations=$MAX_ITERATIONS, CPU_CORE=${CPU_CORE:-unbound}"
echo "==> output: $OUT"

echo "# map seed opt_first repeats opt_success con_success opt_deterministic con_deterministic opt_run_time con_run_time opt_wall_mean con_wall_mean opt_cpu_time con_cpu_time opt_cpu_mean con_cpu_mean opt_iterations con_iterations opt_nodes con_nodes opt_nn_checks con_nn_checks opt_connection_checks con_connection_checks opt_collision_checks con_collision_checks opt_tree_mem con_tree_mem opt_path_mem con_path_mem" > "$OUT"

failures=0
for seed in $(seq "$START_SEED" "$END_SEED"); do
    for i in $(seq 0 $((MAP_COUNT - 1))); do
        map_file="$MAPDIR/map_$i.txt"
        if (( (seed + i) % 2 == 0 )); then
            opt_first=1
            oout=$(run_one ./rrt "$map_file" "$seed")
            [[ "$PAIR_COOLDOWN" == "0" ]] || sleep "$PAIR_COOLDOWN"
            cout=$(run_one ./rrt_con "$map_file" "$seed")
        else
            opt_first=0
            cout=$(run_one ./rrt_con "$map_file" "$seed")
            [[ "$PAIR_COOLDOWN" == "0" ]] || sleep "$PAIR_COOLDOWN"
            oout=$(run_one ./rrt "$map_file" "$seed")
        fi

        os=$(summary_field "$oout" success); cs=$(summary_field "$cout" success)
        od=$(summary_field "$oout" deterministic); cd=$(summary_field "$cout" deterministic)
        ort=$(summary_field "$oout" wall_median); crt=$(summary_field "$cout" wall_median)
        owm=$(summary_field "$oout" wall_mean); cwm=$(summary_field "$cout" wall_mean)
        oct=$(summary_field "$oout" cpu_median); cct=$(summary_field "$cout" cpu_median)
        ocm=$(summary_field "$oout" cpu_mean); ccm=$(summary_field "$cout" cpu_mean)
        oi=$(summary_field "$oout" iterations); ci=$(summary_field "$cout" iterations)
        on=$(summary_field "$oout" nodes); cn=$(summary_field "$cout" nodes)
        onn=$(summary_field "$oout" nn_checks); cnn=$(summary_field "$cout" nn_checks)
        onc=$(summary_field "$oout" connection_checks); cnc=$(summary_field "$cout" connection_checks)
        occ=$(summary_field "$oout" collision_checks); ccc=$(summary_field "$cout" collision_checks)
        otm=$(summary_field "$oout" tree_mem_kb); ctm=$(summary_field "$cout" tree_mem_kb)
        opm=$(summary_field "$oout" path_mem_kb); cpm=$(summary_field "$cout" path_mem_kb)

        required=($os $cs $od $cd $ort $crt $owm $cwm $oct $cct $ocm $ccm
                  $oi $ci $on $cn $onn $cnn $onc $cnc $occ $ccc $otm $ctm $opm $cpm)
        if (( ${#required[@]} != 26 )); then
            echo "ERROR: failed to parse SUMMARY for map_$i seed=$seed" >&2
            exit 3
        fi
        if [[ "$od" != "1" || "$cd" != "1" ]]; then
            echo "ERROR: non-deterministic workload at map_$i seed=$seed" >&2
            exit 3
        fi
        if [[ "$os" != "$REPEATS" || "$cs" != "$REPEATS" ]]; then
            echo "WARNING: map_$i seed=$seed success opt=$os/$REPEATS con=$cs/$REPEATS" >&2
            failures=$((failures + 1))
        fi

        echo "map_$i $seed $opt_first $REPEATS $os $cs $od $cd $ort $crt $owm $cwm $oct $cct $ocm $ccm $oi $ci $on $cn $onn $cnn $onc $cnc $occ $ccc $otm $ctm $opm $cpm" >> "$OUT"
    done
    echo "  seed $seed done"
done

echo "==> done: $OUT"
echo "==> rows: $((NSEEDS * MAP_COUNT)), rows with any failure: $failures"
echo "==> opt_run_time/con_run_time are paired wall-clock medians"
echo "==> analyze with: python3 state.py '$OUT'"
