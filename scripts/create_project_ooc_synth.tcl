# create ooc synthesis project
create_project ooc_riscy ./vivado/ooc_riscy -part xc7z020clg400-1 -force
#xc7z020clg400-1
#xc7z010clg400-1

# Set the directory where the SystemVerilog files are located
#set source_dir "/home/ignaciogarciae/pdp_labs/base_riscy_project/src/design/riscy"

# Get a list of all SystemVerilog files in the directory (matching *.sv and *.svh files)
#set sv_files [glob -nocomplain $source_dir/*.sv]
#lappend sv_files [glob -nocomplain $source_dir/*.v]
#lappend sv_files [glob -nocomplain $source_dir/*.svh]
#lappend sv_files [glob -nocomplain $source_dir/*.vh]
##lappend sv_files [glob -nocomplain $source_dir/include/*.sv]
#lappend sv_files [glob -nocomplain $source_dir/include/*.v]
#lappend sv_files [glob -nocomplain $source_dir/include/*.svh]
#lappend sv_files [glob -nocomplain $source_dir/include/*.vh]

# Add all the found SystemVerilog files to the project
#foreach file $sv_files {
#    add_files $file
#}

# Add Verilog/SystemVerilog sources
# add_files /home/ignaciogarciae/pdp_labs/base_riscy_project/src/design/riscy/aes_decoder.sv
# add_files /home/ignaciogarciae/pdp_labs/base_riscy_project/src/design/riscy/*.v
# add_files /home/ignaciogarciae/pdp_labs/base_riscy_project/src/design/riscy/include/*.sv

# Optionally, add directories
add_files -norecurse ./src/design/riscy/
add_files -norecurse ./src/design/riscy/include/

# Make sure the include files/headers are recognized as such.
# If they had properly been named with the extension .vh we wouldn't need this.
set_property file_type {Verilog Header} [get_files apu_core_package.v]
set_property file_type {Verilog Header} [get_files apu_macros.v]
set_property file_type {Verilog Header} [get_files riscv_config.v]
set_property file_type {Verilog Header} [get_files riscv_defines.v]

update_compile_order -fileset sources_1

set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -value -mode_out_of_context -objects [get_runs synth_1]

# Set the default top-level module for the project
set_property top riscv_ooc_top_level_wrapper [current_fileset]
# set properties of the ooc top level wrapper: used in out of context synthesis
set_property USED_IN {synthesis implementation out_of_context} [get_files -all riscv_ooc_top_level_wrapper.sv]


# Add XDC constraints
# add_files ./src/design/constraints/ooc_constraints.xdc
add_files -fileset constrs_1 -norecurse ./src/design/constraints/ooc_constraints.xdc

# Run out-of-order synthesis
synth_design -top riscv_ooc_top_level_wrapper -constrset constrs_1 -part xc7z010clg400-1 -mode out_of_context

# Write synthesized design
write_checkpoint -force ./vivado/ooc_riscy/ooc_riscy.runs/ooc_synth/riscv_synth.dcp

# Report timing
report_timing_summary -delay_type min_max -report_unconstrained -check_timing_verbose -max_paths 10 -input_pins -routable_nets -name timing_1 -file ./vivado/ooc_riscy/ooc_riscy.runs/ooc_synth/ooc_timing_summary.txt

# Report Utilization
report_utilization -name utilization_1  -file ./vivado/ooc_riscy/ooc_riscy.runs/ooc_synth/ooc_utilization_summary.txt

# Report Power
# generating simulation activity file for accurate power reports (post synthesis simulation):
# https://docs.amd.com/r/en-US/ug997-vivado-power-analysis-optimization-tutorial/Step-7-Running-Functional-Simulation-with-SAIF-Output
# then use report power command with synthesis results and loading the .saif file:
# 

# Write synthesized design (already done)
# write_checkpoint -force ./vivado/ooc_riscy_aes/ooc_riscy_aes.runs/ooc_synth/riscv_synth.dcp

# Open synthesized design
# open_checkpoint ./vivado/ooc_riscy_aes/ooc_riscy_aes.runs/ooc_synth/riscv_synth.dcp