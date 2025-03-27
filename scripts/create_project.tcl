# create ooc synthesis project
create_project riscy ./vivado/riscy -part xc7z020clg400-1 -force
# xc7z010clg400-1
# xc7z020clg400-1

# Optionally, add directories
add_files -norecurse ./src/design/riscy/
add_files -norecurse ./src/design/riscy/include/
add_files -norecurse ./src/design/zynq_system/
add_files -norecurse ./src/simulation/

# Make sure the include files/headers are recognized as such.
# If they had properly been named with the extension .vh we wouldn't need this.
set_property file_type {Verilog Header} [get_files apu_core_package.v]
set_property file_type {Verilog Header} [get_files apu_macros.v]
set_property file_type {Verilog Header} [get_files riscv_config.v]
set_property file_type {Verilog Header} [get_files riscv_defines.v]

update_compile_order -fileset sources_1

#set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} -value -mode_out_of_context -objects [get_runs synth_1]

# Set the default top-level module for the project
# set_property top cv32e40s_core_ooc_top_level_wrapper [current_fileset]
# set properties of the ooc top level wrapper: used in out of context synthesis
# set_property USED_IN {synthesis implementation out_of_context} [get_files -all cv32e40s_core_ooc_top_level_wrapper.sv]


# Add XDC constraints
# add_files ./src/design/constraints/ooc_constraints.xdc
# add_files -fileset constrs_1 -norecurse ./src/design/constraints/ooc_constraints.xdc

source ./scripts/generate_fpga_bd.tcl

# Set the bd wrapper as top for both simulation and synthesis/design
set_property top riscv_wrapper [current_fileset]
update_compile_order -fileset sources_1

set_property top tb [get_filesets sim_1]
#set_property top_lib xil_defaultlib [get_filesets sim_1]