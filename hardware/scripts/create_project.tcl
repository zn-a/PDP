# create ooc synthesis project
create_project riscy ./vivado/riscy -part xc7z020clg400-1 -force

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

# Generate fpga system bd:
source ./scripts/generate_fpga_bd.tcl

# Generate simulation bd:
# source ./scripts/generate_sim_bd.tcl

# Set the bd wrapper as top for synthesis/implementation
set_property top riscv_wrapper [current_fileset]
update_compile_order -fileset sources_1

# Set the tb zynq_tb as top for simulation
set_property top zynq_tb [get_filesets sim_1]