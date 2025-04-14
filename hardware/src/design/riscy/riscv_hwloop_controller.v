module riscv_hwloop_controller
#(
  parameter N_REGS = 2
)
(
  // from id stage
  input  wire [31:0]              current_pc_i,

  // from hwloop_regs
  input  wire [31:0] hwlp_start_addr_0_i,
  input  wire [31:0] hwlp_end_addr_0_i,
  input  wire [31:0] hwlp_counter_0_i,
  input  wire [31:0] hwlp_start_addr_1_i,
  input  wire [31:0] hwlp_end_addr_1_i,
  input  wire [31:0] hwlp_counter_1_i,

  // to hwloop_regs
  output reg [N_REGS-1:0]        hwlp_dec_cnt_o,

  // from pipeline stages
  input  wire [N_REGS-1:0]        hwlp_dec_cnt_id_i,

  // to id stage
  output wire                     hwlp_jump_o,
  output reg [31:0]              hwlp_targ_addr_o
);


  reg [N_REGS-1:0] pc_is_end_addr;

  // end address detection
  integer j;


  // generate comparators. check for end address and the loop counter
  //genvar i;
  //generate
  //  for (i = 0; i < N_REGS; i++) begin
      always @(*)
      begin
        pc_is_end_addr[0] = 1'b0;

        if (current_pc_i == hwlp_end_addr_0_i) begin
          if (hwlp_counter_0_i[31:2] != 30'h0) begin
            pc_is_end_addr[0] = 1'b1;
          end else begin
            // hwlp_counter_i[i][31:2] == 32'h0
            case (hwlp_counter_0_i[1:0])
              2'b11:        pc_is_end_addr[0] = 1'b1;
              2'b10:        pc_is_end_addr[0] = ~hwlp_dec_cnt_id_i[0]; // only when there is nothing in flight
              2'b01, 2'b00: pc_is_end_addr[0] = 1'b0;
            endcase
          end
        end
      end
	  
	    always @(*)
      begin
        pc_is_end_addr[1] = 1'b0;

        if (current_pc_i == hwlp_end_addr_1_i) begin
          if (hwlp_counter_1_i[31:2] != 30'h0) begin
            pc_is_end_addr[1] = 1'b1;
          end else begin
            // hwlp_counter_i[i][31:2] == 32'h0
            case (hwlp_counter_1_i[1:0])
              2'b11:        pc_is_end_addr[1] = 1'b1;
              2'b10:        pc_is_end_addr[1] = ~hwlp_dec_cnt_id_i[1]; // only when there is nothing in flight
              2'b01, 2'b00: pc_is_end_addr[1] = 1'b0;
            endcase
          end
        end
      end
   // end
  //endgenerate

  // select corresponding start address and decrement counter
  always @(*)
  begin
    hwlp_targ_addr_o = 0;
    hwlp_dec_cnt_o   = 0;

      if (pc_is_end_addr[0]) begin
        hwlp_targ_addr_o  = hwlp_start_addr_0_i;
        hwlp_dec_cnt_o = 1'b1;
      end
	  else
		  if (pc_is_end_addr[1]) begin
			hwlp_targ_addr_o  = hwlp_start_addr_1_i;
			hwlp_dec_cnt_o = 1'b1;
		  end
  end

  // output signal for ID stage
  assign hwlp_jump_o = (|pc_is_end_addr);

endmodule
