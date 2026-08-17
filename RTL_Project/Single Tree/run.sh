#!/bin/bash

set -e

python3 ../PE_test/randnum_gen.py

./../PE_test/gold

iverilog -g2012 -o simulation.out TESTBED.sv

vvp simulation.out
