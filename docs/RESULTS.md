# Results and Methodology

## RTL Planning Latency

Each density uses 10 independently generated maps and random seeds 42 through
91, producing 500 map/seed tasks. A task passes only if the reconstructed path
starts and ends at the required roots, uses legal neighboring steps, and does
not intersect an obstacle. Latency is calculated as:

```text
latency_ms = cycle_count x 3.3 ns / 1,000,000
```

| Density | Cases | Pass rate | Mean cycles | Mean latency | 95% CI of mean latency |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 30% | 500 | 100% | 227,348.810 | 0.7503 ms | +/- 0.0108 ms |
| 50% | 500 | 100% | 358,212.636 | 1.1821 ms | +/- 0.0173 ms |
| 70% | 500 | 100% | 555,433.548 | 1.8329 ms | +/- 0.0487 ms |

The equal-weight mean over all 1,500 tasks is **1.2551 ms/task**. Raw RTL rows
are stored in:

- [`result_30.txt`](../RTL_Project/01_RTL/result/result_30.txt)
- [`result_50.txt`](../RTL_Project/01_RTL/result/result_50.txt)
- [`result_70.txt`](../RTL_Project/01_RTL/result/result_70.txt)

The increase at 70% density is expected: more extensions collide, more random
samples are discarded, and the trees require more attempts before finding the
guaranteed free corridor.

## Software Comparison

The C++ experiment compares an optimized grid-indexed algorithm (`rrt.cpp`)
with a conventional full-tree-scan baseline (`rrt_con.cpp`). The accelerator is
reported against both because the conventional baseline isolates algorithmic
improvement, while the optimized software model is the stronger implementation
comparison.

Mean wall-clock runtime from the archived repeated batches:

| Host | Density | Optimized C++ | Conventional C++ | RTL | RTL speedup vs optimized | RTL speedup vs conventional |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Apple M4 Pro | 30% | 1.151 ms | 1.309 ms | 0.750 ms | 1.53x | 1.74x |
| Apple M4 Pro | 50% | 1.756 ms | 1.908 ms | 1.182 ms | 1.49x | 1.61x |
| Apple M4 Pro | 70% | 2.699 ms | 3.572 ms | 1.833 ms | 1.47x | 1.95x |
| Intel Core Ultra 7 | 30% | 2.518 ms | 1.853 ms | 0.750 ms | 3.36x | 2.47x |
| Intel Core Ultra 7 | 50% | 3.961 ms | 2.681 ms | 1.182 ms | 3.35x | 2.27x |
| Intel Core Ultra 7 | 70% | 6.093 ms | 4.962 ms | 1.833 ms | 3.32x | 2.71x |

These are platform-dependent wall-clock comparisons, not process-normalized
silicon comparisons. Compiler, CPU core selection, thermal state, frequency,
and operating system scheduling all influence software runtime. The paired
benchmark alternates program order and uses repeated medians to reduce, but not
eliminate, this variation.

The software and RTL planners share major algorithmic optimizations but are not
identical execution models. The C++ optimized model is single-threaded and uses
16 nodes per grid cell; the RTL uses four PEs, shared memory arbitration, and 15
nodes per cell.

## Physical Implementation

The final APR run was labeled `final_0.7` and targeted the TSMC 130 nm contest
technology with eight metal layers.

| Metric | Final post-route result |
| --- | ---: |
| Clock period / frequency | 3.3 ns / 303.03 MHz |
| Block dimensions | 527.16 um x 522.75 um |
| Block area | 0.275573 mm^2 |
| Core dimensions | 502.32 um x 498.15 um |
| Core area | 0.250231 mm^2 |
| Functional standard-cell area | 0.146385 mm^2 |
| Core utilization | 58.50% |
| Functional instances | 13,786 |
| Nets | 15,377 |
| Setup WNS / TNS | +0.008 ns / 0 ns |
| Hold WNS / TNS | +0.001 ns / 0 ns |
| Setup / hold violations | 0 / 0 |
| Clock sinks / CTS buffers | 1,778 / 19 |
| Clock skew / average insertion | 0.032 ns / 0.421 ns |
| Routed wire length | 506.591 mm |
| Vias | 91,342 |
| Routing overflow | 0 |

The timing result is post-route and clean for the analyzed corner. It should not
be described as full foundry signoff because a final post-filler ECO route and
complete DRC recheck were not archived in this repository.

## Area and Power Scope

The implemented block excludes the external map, start-tree, goal-tree, and
path memories used by `TESTBED.sv`. The physical area is therefore a logic-core
result. A memory macro implementation would increase total chip area and add
memory access energy.

No validated SAIF/VCD activity-based post-route power report was produced.
Accordingly, this repository does not claim power, energy/task, or an energy
comparison with the reference paper.

## Figure Generation

The checked-in figures are generated from the archived tables by:

```bash
python3 RTL_Project/outputs/generate_charts.py
```

The script includes only successful RTL tasks and emits:

- `run_time_comparison.svg`
- `tree_memory_comparison.svg`
