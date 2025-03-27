# Xilinx Constraints File (XDC)

# Define the clock period variable (in nanoseconds)
set CLK_PERIOD 50.000  ;# 20 MHz clock -> 50 ns period

# Define clock uncertainty as 10% of the clock period
set CLK_UNCERTAINTY [expr $CLK_PERIOD * 0.1]

# Define input/output delay variables based on CLK_PERIOD
set MAX_IO_DELAY [expr $CLK_PERIOD / 2]  ;# Half of clock period
set MIN_IO_DELAY 2.000                   ;# Small minimum delay for hold margin


# Set the primary clock to 10 MHz (100 ns period)
create_clock -name clk_i -period $CLK_PERIOD [get_ports clk_i]
# Apply the calculated clock uncertainty to the primary clock
set_clock_uncertainty $CLK_UNCERTAINTY [get_clocks clk_i]

# Synchronous Reset Constraints
set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports rst_ni]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports rst_ni]

# Input Delays
set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports restart]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports restart]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports clock_en_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports clock_en_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports test_en_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports test_en_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports boot_addr_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports boot_addr_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports core_id_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports core_id_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports cluster_id_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports cluster_id_i]

# Instruction memory interface
set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports instr_gnt_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports instr_gnt_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports instr_rvalid_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports instr_rvalid_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports instr_rdata_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports instr_rdata_i]

# Data memory interface
set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_gnt_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_gnt_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_rvalid_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_rvalid_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_rdata_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_rdata_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_err_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_err_i]

# Interrupts
set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports irq_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports irq_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports irq_id_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports irq_id_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports irq_sec_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports irq_sec_i]

# Debug Interface
set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_req_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_req_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_addr_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_addr_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_we_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_we_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_wdata_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_wdata_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_halt_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_halt_i]

set_input_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_resume_i]
set_input_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_resume_i]

# Output Delays
set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports instr_req_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports instr_req_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports instr_addr_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports instr_addr_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_req_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_req_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_we_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_we_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_be_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_be_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_addr_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_addr_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports data_wdata_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports data_wdata_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports irq_ack_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports irq_ack_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports irq_id_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports irq_id_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports sec_lvl_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports sec_lvl_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_gnt_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_gnt_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_rvalid_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_rvalid_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_rdata_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_rdata_o]

set_output_delay -max $MAX_IO_DELAY -clock clk_i [get_ports debug_halted_o]
set_output_delay -min $MIN_IO_DELAY -clock clk_i [get_ports debug_halted_o]