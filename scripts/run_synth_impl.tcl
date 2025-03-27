update_compile_order -fileset sources_1

# Launch implementation run:
#remove jobs 6?
launch_runs impl_1 -jobs 6

# Generate bitstream:
#remove jobs 6?
launch_runs impl_1 -to_step write_bitstream -jobs 6