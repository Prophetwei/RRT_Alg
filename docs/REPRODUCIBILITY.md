# Reproducibility Guide

This guide separates the open-source algorithm/RTL checks from the licensed ASIC
flow. The source and raw benchmark tables are tracked; multi-gigabyte occupancy
maps and commercial technology files are regenerated or supplied separately.

## 1. Clone and Record the Environment

```bash
git clone https://github.com/Prophetwei/RRT_Alg.git
cd RRT_Alg
g++ --version
python3 --version
iverilog -V
```

For publication-quality CPU timing, also record the OS, CPU model, selected
physical core, frequency governor, compiler flags, and whether other workloads
were active.

## 2. Generate Deterministic Map Sets

Install NumPy, then generate ten maps at each tested density:

```bash
cd 3D
python3 map_generator.py --density 30 --seed 1000 --count 10 --output-dir map_30
python3 map_generator.py --density 50 --seed 2000 --count 10 --output-dir map_50
python3 map_generator.py --density 70 --seed 3000 --count 10 --output-dir map_70
```

Each output map has a matching guaranteed-path certificate. The benchmark
driver validates every certificate before running either algorithm. Retain the
generator arguments and NumPy version with any new results.

## 3. Build and Benchmark the Software Models

```bash
g++ -O3 -DNDEBUG -std=c++17 rrt.cpp -o rrt
g++ -O3 -DNDEBUG -std=c++17 rrt_con.cpp -o rrt_con

CPU_CORE=2 MAPDIR=map_30 ./run_10maps.sh 42 5 50 1
CPU_CORE=2 MAPDIR=map_50 ./run_10maps.sh 42 5 50 1
CPU_CORE=2 MAPDIR=map_70 ./run_10maps.sh 42 5 50 1
```

`CPU_CORE` uses Linux `taskset`; omit it on macOS. Both programs must run on the
same host, core, compiler build, map set, seed set, and maximum iteration limit.
The script alternates execution order and writes one auditable row per map/seed.

Summarize a result file with:

```bash
python3 state.py results_seed42_YYYYMMDD_HHMMSS.txt
```

The analysis excludes failed runs from runtime and memory aggregates while
still reporting their count.

## 4. Compile and Simulate the RTL

Open-source compile and single-case simulation:

```bash
cd ../RTL_Project/01_RTL
iverilog -g2012 -DRTL_TOP -s TESTBED -o /tmp/rrt_rtl.vvp TESTBED.sv
vvp /tmp/rrt_rtl.vvp +NOMAP +SEED=42
vvp /tmp/rrt_rtl.vvp +MAP=../../3D/map_30/map_0.txt +SEED=42
```

Original Cadence flow:

```bash
./01_run.sh +NOMAP +SEED=42
./01_run.sh +MAP=../../3D/map_30/map_0.txt +SEED=42
MAPDIR=../../3D/map_30 ./01_run_10maps.sh 42 50
```

Repeat the batch command for each density directory. The expected workload is
10 maps x 50 seeds = 500 tasks per density. The archived result files report
500/500 passes for 30%, 50%, and 70% obstacle densities.

## 5. Regenerate Figures

From the repository root:

```bash
python3 RTL_Project/outputs/generate_charts.py
```

The generator reads the archived RTL result files and the CPU aggregate table
embedded in the script, then replaces the two SVG figures in `outputs/`.

## 6. Licensed Synthesis and APR Flow

The following steps require the contest CBDK/TSMC 130 nm package and licensed
Synopsys/Cadence tools. These dependencies cannot be redistributed.

1. Update technology and library paths in `RTL_Project/02_SYN/syn.tcl`.
2. Run `RTL_Project/02_SYN/02_run_dc.sh` under Design Compiler.
3. Run `RTL_Project/03_GATE/01_run.sh timing map` for SDF gate simulation.
4. Load `RTL_Project/04_APR/RRT_TOP.view` and
   `RTL_Project/04_APR/RRT_TOP_SYN.sdc` in Innovus.
5. Verify setup, hold, routing overflow, connectivity, DRC, and antenna reports
   at the final post-route database.

The checked-in netlist, SDF, reports, and constraints preserve the completed
experiment, but historical logs contain machine-specific absolute paths.

## 7. Result Provenance

| Result | Tracked source |
| --- | --- |
| RTL latency and pass rate | `RTL_Project/01_RTL/result/result_*.txt` |
| CPU benchmark samples | `3D/result_*.txt` |
| Figures | `RTL_Project/outputs/generate_charts.py` |
| Synthesis timing/netlist | `RTL_Project/02_SYN/` |
| Final APR summary | `docs/RESULTS.md`, run label `final_0.7` |

The final Innovus database and licensed technology collateral are not tracked.
Power is intentionally omitted because no validated activity-based power run is
available.
