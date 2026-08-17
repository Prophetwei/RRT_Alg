#!/usr/bin/env bash
#=====================================================================
#  Gate-level simulation of the synthesized RRT_TOP core with VCS
#
#  Inputs produced by synthesis (syn.tcl):
#     Netlist/RRT_TOP_SYN.v      gate netlist   (module RRT_TOP, same ports)
#     Netlist/RRT_TOP_SYN.sdf    timing         (used only in "timing" mode)
#
#  TESTBED.sv selects the core by define:
#     (no define)  -> RTL core  (iverilog / RTL VCS)
#     +define+SYN  -> drop RTL include; use the gate netlist below
#     +define+SDF  -> $sdf_annotate("Netlist/RRT_TOP_SYN.sdf", u_TOP)
#
#  The behavioural memories (TREEMEM/MAPMEM/PATHMEM) and PATTERN stay RTL.
#  Design + testbench sources live in ../01_RTL ; synth output in ../02_SYN.
#  Run this from the 03_GATE directory.
#
#  Usage:
#     ./run_gate.sh func   nomap      # zero-delay gate sim, empty map (fast bring-up)
#     ./01_run.sh timing nomap 3.3      # SDF timing, empty map, 3.3 ns clock
#     ./01_run.sh timing map 5.0        # SDF timing, real map, 5.0 ns clock
#     ./01_run.sh timing "+SEED=42" 3.3 # pass custom plusargs and clock
#=====================================================================
set -e

#---------------------------------------------------------------------
#  TSMC 0.13 um Verilog simulation model matching slow.db.
#  standard cells -- the .v that matches the slow.db used in synthesis.
#  (e.g. the vendor's ".../verilog/<lib>.v"; use the _neg / unit-delay
#   model that carries the specify/timing blocks.)
#---------------------------------------------------------------------
CELL_V=../../CBDK_IC_Contest_v2.5/Verilog/tsmc13_neg.v
#  NOTE: the gate netlist is NOT listed here -- TESTBED.sv pulls it in itself
#  via `+define+GATE_TOP` (`include "../02_SYN/Netlist/RRT_TOP_SYN.v").  Passing
#  it again on the vcs line would declare module RRT_TOP twice (MPD error).

MODE=${1:-timing}          # func | timing
STIM=${2:-map}           # nomap | map | "+SEED=.. ..."
PERIOD=${3:-3.3}         # testbench clock period in ns

# -suppress=TFIPC : library flops are instantiated with an unused .Q pin
#   (DC only wires .QN) -> benign "too few port connections", silence the ~300.
COMMON="-full64 -sverilog +v2k -debug_access+all -timescale=1ns/10ps -suppress=TFIPC -l"

case "$MODE" in
  func)
    DEFS="+define+GATE_TOP +notimingcheck +nospecify +delay_mode_zero"
    OUT=simv_gate_func ;;
  timing)
    DEFS="+define+GATE_TOP+SDF +neg_tchk +sdfverbose"   # SDF back-annotated timing sim
    OUT=simv_gate_timing ;;
  *) echo "MODE must be 'func' or 'timing'"; exit 1 ;;
esac

DEFS="$DEFS +define+CYCLE_TIME=$PERIOD"

case "$STIM" in
  nomap) RUNARGS="+NOMAP +SEED=42" ;;
  map)   RUNARGS="+SEED=42" ;;
  *)     RUNARGS="$STIM" ;;                    # pass-through
esac

echo "=== VCS compile  (mode=$MODE, period=${PERIOD}ns) ==="
#  +incdir+../01_RTL : the design/testbench sources live in 01_RTL, and VCS
#  does NOT auto-search the including file's dir -- without this, the bare
#  `include "path_mem.sv" (etc.) inside TESTBED.sv cannot be found.
vcs $COMMON comp_${OUT}.log $DEFS +incdir+../01_RTL -o $OUT \
    ../01_RTL/TESTBED.sv $CELL_V

echo "=== VCS run  (args: $RUNARGS) ==="
./$OUT $RUNARGS -l sim_${OUT}.log

echo "=== done : see sim_${OUT}.log ==="
