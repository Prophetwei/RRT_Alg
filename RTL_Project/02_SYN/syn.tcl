#======================================================================
# RRT_TOP synthesis flow for CBDK_IC_Contest_v2.5 / TSMC 0.13 um
# Run from any directory with: dc_shell -f 02_SYN/syn.tcl
#======================================================================

#---------------------------------------------------------------------
# Local, relocatable paths
#   RTL_Project/
#     01_RTL/
#     02_SYN/
#   CBDK_IC_Contest_v2.5/
#---------------------------------------------------------------------
set SCRIPT_DIR [file dirname [file normalize [info script]]]
cd $SCRIPT_DIR

set RTL_DIR    [file normalize [file join $SCRIPT_DIR .. 01_RTL]]
set PDK_ROOT   [file normalize [file join $SCRIPT_DIR .. .. CBDK_IC_Contest_v2.5]]
set PDK_DB_DIR [file join $PDK_ROOT SynopsysDC db]

foreach REQUIRED_FILE [list \
        [file join $RTL_DIR rrt_top.sv] \
        [file join $PDK_DB_DIR slow.db] \
        [file join $PDK_DB_DIR fast.db]] {
    if {![file readable $REQUIRED_FILE]} {
        error "Required synthesis file is missing or unreadable: $REQUIRED_FILE"
    }
}

file mkdir Report
file mkdir Netlist
file mkdir WORK

set_app_var search_path [concat [list $RTL_DIR $PDK_DB_DIR] $search_path]
set_app_var target_library [list slow.db]
set_app_var link_library [list "*" slow.db fast.db dw_foundation.sldb]
set_app_var synthetic_library [list dw_foundation.sldb]
define_design_lib WORK -path [file join $SCRIPT_DIR WORK]

# Settings carried over from the CBDK design example.
set_app_var verilogout_no_tri true
set_app_var edifout_netlist_only true
set_app_var report_default_significant_digits 4

#---------------------------------------------------------------------
# Design and interface constraints
#---------------------------------------------------------------------
set DESIGN             RRT_TOP
set CLOCK_NAME         clk
set CLOCK_PERIOD       3.3
set SETUP_UNCERTAINTY  0.10
set HOLD_UNCERTAINTY   0.05
set INPUT_DELAY        1.0
set OUTPUT_DELAY       1.0
set INPUT_DRIVE        1.0
set OUTPUT_PIN_LOAD    0.01

#---------------------------------------------------------------------
# Read and elaborate the parameterized SystemVerilog design
#---------------------------------------------------------------------
analyze -library WORK -format sverilog [file join $RTL_DIR rrt_top.sv]
elaborate $DESIGN
current_design $DESIGN
link

# The slow library is used for setup/mapping and fast for minimum-delay
# analysis.  The explicit WLM avoids the previous zero-wire-load estimate.
set_min_library slow.db -min_version fast.db
set_operating_conditions -min_library fast -min fast \
                         -max_library slow -max slow
set_wire_load_model -name tsmc13_wl10 -library slow
set_wire_load_mode top

create_clock -name $CLOCK_NAME -period $CLOCK_PERIOD \
             -waveform [list 0 [expr {$CLOCK_PERIOD / 2.0}]] [get_ports clk]
set_dont_touch_network [get_clocks $CLOCK_NAME]
set_fix_hold           [get_clocks $CLOCK_NAME]
set_clock_uncertainty -setup $SETUP_UNCERTAINTY [get_clocks $CLOCK_NAME]
set_clock_uncertainty -hold  $HOLD_UNCERTAINTY  [get_clocks $CLOCK_NAME]
set_ideal_network [get_ports clk]

set DATA_INPUTS [remove_from_collection [all_inputs] [get_ports {clk rst_n}]]
if {[sizeof_collection $DATA_INPUTS] > 0} {
    set_input_delay -max $INPUT_DELAY -clock $CLOCK_NAME $DATA_INPUTS
    set_input_delay -min 0.0          -clock $CLOCK_NAME $DATA_INPUTS
    set_drive $INPUT_DRIVE $DATA_INPUTS
}

set_output_delay -max $OUTPUT_DELAY -clock $CLOCK_NAME [all_outputs]
set_output_delay -min 0.0           -clock $CLOCK_NAME [all_outputs]
set_load -pin_load $OUTPUT_PIN_LOAD [all_outputs]

# Use pin collections for timing endpoints.  Register-output pins are replaced
# during technology mapping, so clock objects are the stable startpoints for
# sequential path groups.
set REG_Q_PINS [all_registers -output_pins]
set REG_D_PINS [all_registers -data_pins]
group_path -name REG2REG -from [get_clocks $CLOCK_NAME] -to $REG_D_PINS \
           -weight 3.0 -critical_range 0.5
group_path -name IN2REG -from $DATA_INPUTS -to $REG_D_PINS \
           -weight 2.0 -critical_range 0.5
group_path -name REG2OUT -from [get_clocks $CLOCK_NAME] -to [all_outputs] \
           -weight 2.0 -critical_range 0.5

#---------------------------------------------------------------------
# Optimization
#---------------------------------------------------------------------
uniquify
set_fix_multiple_port_nets -all -buffer_constants
compile_ultra

#---------------------------------------------------------------------
# Reports before name conversion retain recognizable RTL hierarchy names.
#---------------------------------------------------------------------
# Refresh the collections after mapping/ungrouping; the pre-compile generic
# register pins may have been replaced by technology-cell pins.
set REG_Q_PINS [all_registers -output_pins]
set REG_D_PINS [all_registers -data_pins]

check_design > Report/$DESIGN.check_design
check_timing > Report/$DESIGN.check_timing
report_constraint -all_violators > Report/$DESIGN.constraints
report_qor > Report/$DESIGN.qor
report_clock > Report/$DESIGN.clock
report_area > Report/$DESIGN.area
report_resource > Report/$DESIGN.resource

report_timing -delay_type max -max_paths 50 -nworst 5 -significant_digits 4 \
              -input_pins -nets -transition_time -capacitance \
              > Report/$DESIGN.timing
report_timing -delay_type max -from $REG_Q_PINS -to $REG_D_PINS \
              -max_paths 50 -nworst 5 -significant_digits 4 -input_pins -nets \
              -transition_time -capacitance \
              > Report/$DESIGN.timing_reg2reg
report_timing -delay_type max -from $DATA_INPUTS -to $REG_D_PINS \
              -max_paths 50 -nworst 5 -significant_digits 4 -input_pins -nets \
              -transition_time -capacitance \
              > Report/$DESIGN.timing_in2reg
report_timing -delay_type max -from $REG_Q_PINS -to [all_outputs] \
              -max_paths 50 -nworst 5 -significant_digits 4 -input_pins -nets \
              -transition_time -capacitance \
              > Report/$DESIGN.timing_reg2out
report_timing -delay_type min -from $REG_Q_PINS -to $REG_D_PINS \
              -max_paths 50 -nworst 5 -significant_digits 4 -input_pins -nets \
              -transition_time -capacitance \
              > Report/$DESIGN.timing_hold
report_timing -delay_type min -from $DATA_INPUTS -to $REG_D_PINS \
              -max_paths 50 -nworst 5 -significant_digits 4 -input_pins -nets \
              -transition_time -capacitance \
              > Report/$DESIGN.timing_hold_in2reg

#---------------------------------------------------------------------
# Verilog-safe names and APR handoff files
#---------------------------------------------------------------------
set bus_inference_style {%s[%d]}
set bus_naming_style {%s[%d]}
set hdlout_internal_busses true
change_names -hierarchy -rule verilog

define_name_rules name_rule -allowed {a-z A-Z 0-9 _} -max_length 255 -type cell
define_name_rules name_rule -allowed {a-z A-Z 0-9 _[]} -max_length 255 -type net
define_name_rules name_rule -map {{"\\*cell\\*" "cell"}}
define_name_rules name_rule -case_insensitive
change_names -hierarchy -rules name_rule

set verilogout_higher_designs_first true
write -hierarchy -format ddc     -output Netlist/$DESIGN.ddc
write -hierarchy -format verilog -output Netlist/$DESIGN\_SYN.v
write_sdf -version 3.0 -context verilog -load_delay cell \
          -significant_digits 6 Netlist/$DESIGN\_SYN.sdf
write_sdc Netlist/$DESIGN\_SYN.sdc

exit
