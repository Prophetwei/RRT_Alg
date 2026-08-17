# 3-D RRT Software Models

This directory contains the software experiment used to evaluate the
accelerator's algorithmic optimizations independently of hardware parallelism,
arbitration, and pipeline timing.

## Implementations

| File | Purpose |
| --- | --- |
| `rrt.cpp` | Grid-indexed, dual-tree optimized RRT model |
| `rrt_con.cpp` | Conventional dual-tree RRT baseline with full-tree scans |
| `map_generator.py` | Density-balanced random 3-D map generator |
| `run_10maps.sh` | Paired, repeated benchmark driver |
| `state.py` | Successful-run statistics parser |
| `visualize_3D.py` | Tree and path visualizer |
| `result_*.txt` | Archived benchmark result tables |
| `tree/`, `tree_con/` | Example successful tree and path outputs |

`rrt.cpp` applies the same main algorithmic ideas as the RTL: grid indexing,
root-side cell probing, shift-based direction quantization, branch extension,
localized opposite-tree connection checks, and duplicate suppression. It is an
algorithmic software model, not a cycle-accurate or four-thread PE model.

The optimized software model permits 16 nodes per grid cell. The RTL stores 15
nodes because slot value `0xF` is reserved for the parent-pointer sentinel.

## Requirements

- Linux or macOS
- A C++17 compiler; Linux GCC 13 was used for the archived measurements
- Python 3
- NumPy for map generation
- Matplotlib for visualization only

## Generate Maps

Map data are not tracked because ten 256 x 256 x 256 text maps require several
gigabytes. Regenerate a deterministic set with:

```bash
python3 map_generator.py \
  --density 30 \
  --seed 1000 \
  --count 10 \
  --output-dir map
```

`--density` accepts `0.30`, `30`, or `30%`. With a fixed seed and the same
NumPy-compatible implementation, the generator produces the same map set. Each
map is accompanied by `map_N_guaranteed_path.txt`, which certifies a free,
6-connected route between the two roots.

For separate density sets:

```bash
python3 map_generator.py --density 30 --seed 1000 --count 10 --output-dir map_30
python3 map_generator.py --density 50 --seed 2000 --count 10 --output-dir map_50
python3 map_generator.py --density 70 --seed 3000 --count 10 --output-dir map_70
```

The generator evaluates regional obstacle density and chooses the most balanced
candidate. The distribution is statistical rather than symmetric.

## Build

```bash
g++ -O3 -DNDEBUG -std=c++17 rrt.cpp -o rrt
g++ -O3 -DNDEBUG -std=c++17 rrt_con.cpp -o rrt_con
```

`-O3` is used for both implementations so compiler optimization is controlled
fairly. Do not compare binaries built with different compilers or flags.

## Run a Paired Benchmark

```bash
CPU_CORE=2 MAPDIR=map_30 ./run_10maps.sh 42 5 50 1
```

Arguments are:

```text
./run_10maps.sh START_SEED REPEATS NSEEDS WARMUPS
```

The example evaluates 10 maps x 50 seeds. Each map/seed workload is repeated
five times after one warm-up. Optimized-first and conventional-first order is
alternated to reduce systematic thermal and scheduling bias. The reported
`opt_run_time` and `con_run_time` fields are wall-clock medians over identical
repeats.

For a stable Linux benchmark, bind both programs to the same physical core,
stop unrelated workloads, use the same compiler binary, and record the CPU
governor and machine configuration.

## Analyze Results

```bash
python3 state.py results_seed42_YYYYMMDD_HHMMSS.txt
```

Only successful samples are included in the reported runtime and memory
statistics. Keep the raw result table with any publication figure so failures
and sample counts remain auditable.
