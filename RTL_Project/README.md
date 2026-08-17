# RTL and ASIC Implementation

This directory contains the current four-PE accelerator, verification
environment, synthesis artifacts, gate-level flow, APR constraints, historical
architecture snapshots, and publication figures.

## Directory Guide

| Path | Status | Contents |
| --- | --- | --- |
| `01_RTL/` | Current | Synthesizable SystemVerilog and behavioral memories/testbench |
| `02_SYN/` | Current run | Design Compiler script, netlist, SDF, and reports |
| `03_GATE/` | Current | VCS functional/timing gate-simulation driver |
| `04_APR/` | Final constraints | Innovus MMMC view and 3.3 ns SDC |
| `4_core_v1/` | Archive | Earlier four-PE implementation |
| `4_core_v2/` | Archive | Pre-optimization four-PE backup |
| `8_core/` | Archive | Eight-PE scaling experiment |
| `Single Tree/` | Archive | Single-tree architecture experiment |
| `outputs/` | Results | Chart generator and SVG figures |

Use `01_RTL/` for all new verification. Archived variants are retained for
research traceability and are not maintained as equivalent final designs.

## Current RTL Modules

| Module | Responsibility |
| --- | --- |
| `rrt_top.sv` | Four-PE integration, tree ownership, and shared control |
| `PE.sv` | Sampling, nearest-neighbor search, vector quantization, and extension |
| `TREECONT.sv` | Arbitration for shared tree nearest-neighbor reads |
| `MAPCONT.sv` | Rotating-priority arbitration for the shared map read port |
| `connection_check.sv` | Duplicate filtering, opposite-tree search, insertion, and path reconstruction |
| `CCREADCONT.sv` | Connection-check access arbitration |
| `tree_mem.sv` | Behavioral dual-port grid-indexed tree memory |
| `map_mem.sv` | Behavioral 256^3 obstacle memory |
| `path_mem.sv` | Behavioral reconstructed-path memory |
| `PATTERN.sv` | Stimulus, batch driver, timeout handling, and path checker |
| `TESTBED.sv` | RTL/gate selection and memory/testbench integration |

The synthesized `RRT_TOP` is the logic core. The behavioral tree, map, and path
memories are external to that core and are not included in reported standard-
cell area.

## Open-Source Source Check

Icarus Verilog can compile and run the RTL testbench without the commercial EDA
flow:

```bash
cd 01_RTL
iverilog -g2012 -DRTL_TOP -s TESTBED -o /tmp/rrt_rtl.vvp TESTBED.sv
vvp /tmp/rrt_rtl.vvp +NOMAP +SEED=42
```

To run one generated map:

```bash
vvp /tmp/rrt_rtl.vvp +MAP=../../3D/map/map_0.txt +SEED=42
```

## Cadence RTL Simulation

The provided script uses `irun` and the Novas PLI available in the original lab
environment:

```bash
cd 01_RTL
./01_run.sh +NOMAP +SEED=42
./01_run.sh +MAP=../../3D/map/map_0.txt +SEED=42
```

Batch verification over 10 maps and 50 seeds:

```bash
MAPDIR=../../3D/map ./01_run_10maps.sh 42 50
```

The checked-in density result files contain 500 successful cases each. See
[`../docs/RESULTS.md`](../docs/RESULTS.md) for the summarized measurements.

## ASIC Flow Boundary

- Synthesis: Synopsys Design Compiler with `02_SYN/syn.tcl`.
- Gate simulation: Synopsys VCS with the netlist and SDF in `02_SYN/Netlist/`.
- Physical implementation: Cadence Innovus using `04_APR/RRT_TOP.view` and
  `04_APR/RRT_TOP_SYN.sdc`.
- Target: TSMC 130 nm contest technology, 3.3 ns clock.

The commercial tool executables, licensed standard-cell libraries, QRC files,
and PDK are not part of this repository. Absolute paths in historical logs
reflect the original lab environment and must be replaced for a new setup.

## Generated and Historical Files

The synthesis netlist, SDF, DDC, and reports are included to preserve the
completed experiment. They are generated artifacts, not the preferred source
of design edits. Simulation executables, waveforms, tool work directories, and
occupancy map text files are excluded through the repository `.gitignore`.
