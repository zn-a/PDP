# Xilinx Constraints File (XDC)

# Define the clock period variable (in nanoseconds)
set CLK_PERIOD 20.000  ;# 20 MHz clock -> 50 ns period. 50 MHz clock -> 20 ns period

# Define clock uncertainty as 10% of the clock period
set CLK_UNCERTAINTY [expr $CLK_PERIOD * 0.1]

# Define input/output delay variables based on CLK_PERIOD
set MAX_IO_DELAY [expr $CLK_PERIOD / 2]  ;# quarter of clock period
set MIN_IO_DELAY 2.000                   ;# Small minimum delay for hold margin


# Set the primary clock to 10 MHz (100 ns period)
create_clock -name clk_i -period $CLK_PERIOD [get_ports clk_i]