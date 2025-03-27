# Xilinx Constraints File (XDC)

# Clock constraints
create_clock -period 100 [get_pins riscv_core_pmp/clk_i]  ; # 10 MHz clock with 100 ns period

# Input delay constraints for reset signal (rst_ni) synchronized to clk_i
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/rst_ni]

# Input delay constraints (2 ns input delay)
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/instr_req_o]
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_req_o]
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_we_o]
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_be_o]
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_wdata_o]
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/instr_addr_o]
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_addr_o]
set_input_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/debug_pc_o]

# Output delay constraints (2 ns output delay)
set_output_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/instr_req_o]
set_output_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_req_o]
set_output_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_we_o]
set_output_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_be_o]
set_output_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_wdata_o]
set_output_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/instr_addr_o]
set_output_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/data_addr_o]
set_output_delay -clock [get_clocks riscv_core_pmp/clk_i] -min 1 -max 2 [get_pins riscv_core_pmp/debug_pc_o]