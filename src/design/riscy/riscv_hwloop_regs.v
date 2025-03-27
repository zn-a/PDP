module riscv_hwloop_regs
#(
  parameter N_REGS     = 2,
  parameter N_REG_BITS = $clog2(N_REGS)
)
(
  input  wire                     clk,
  input  wire                     rst_n,

  // from ex stage
  input  wire           [31:0]    hwlp_start_data_i,
  input  wire           [31:0]    hwlp_end_data_i,
  input  wire           [31:0]    hwlp_cnt_data_i,
  input  wire            [2:0]    hwlp_we_i,
  input  wire [N_REG_BITS-1:0]    hwlp_regid_i,         // selects the register set

  // from controller
  input  wire                     valid_i,

  // from hwloop controller
  input  wire [N_REGS-1:0]        hwlp_dec_cnt_i,

  // to hwloop controller
  output wire [31:0] hwlp_start_addr_0_o,
  output wire [31:0] hwlp_end_addr_0_o,
  output wire [31:0] hwlp_counter_0_o,
  output wire [31:0] hwlp_start_addr_1_o,
  output wire [31:0] hwlp_end_addr_1_o,
  output wire [31:0] hwlp_counter_1_o
);


  reg [31:0] hwlp_start_0_q;
  reg [31:0] hwlp_end_0_q;
  reg  [31:0] hwlp_counter_0_q;
  wire [31:0] hwlp_counter_0_n;
  
    reg [31:0] hwlp_start_1_q;
  reg [31:0] hwlp_end_1_q;
  reg [31:0] hwlp_counter_1_q;
  wire [31:0] hwlp_counter_1_n;

  integer i;


  assign hwlp_start_addr_0_o = hwlp_start_0_q;
  assign hwlp_end_addr_0_o   = hwlp_end_0_q;
  assign hwlp_counter_0_o    = hwlp_counter_0_q;
  
  assign hwlp_start_addr_1_o = hwlp_start_1_q;
  assign hwlp_end_addr_1_o   = hwlp_end_1_q;
  assign hwlp_counter_1_o    = hwlp_counter_1_q;


  /////////////////////////////////////////////////////////////////////////////////
  // HWLOOP start-address register                                               //
  /////////////////////////////////////////////////////////////////////////////////
  always @(posedge clk, negedge rst_n)
  begin : HWLOOP_REGS_START
    if (rst_n == 1'b0)
    begin
      hwlp_start_0_q <= 0;
	  hwlp_start_1_q <= 0;
    end
    else if (hwlp_we_i[0] == 1'b1)
    begin
	  if (hwlp_regid_i)
	  hwlp_start_1_q <= hwlp_start_data_i;
	  else
	  hwlp_start_0_q <= hwlp_start_data_i;
    end
  end


  /////////////////////////////////////////////////////////////////////////////////
  // HWLOOP end-address register                                                 //
  /////////////////////////////////////////////////////////////////////////////////
  always @(posedge clk, negedge rst_n)
  begin : HWLOOP_REGS_END
    if (rst_n == 1'b0)
    begin
	  hwlp_end_0_q <= 0;
	  hwlp_end_1_q <= 0;
    end
    else if (hwlp_we_i[1] == 1'b1)
    begin
      if (hwlp_regid_i)
	  hwlp_end_1_q <= hwlp_end_data_i;
	  else
	  hwlp_end_0_q <= hwlp_end_data_i;
    end
  end


  /////////////////////////////////////////////////////////////////////////////////
  // HWLOOP counter register with decrement wire                                //
  /////////////////////////////////////////////////////////////////////////////////
  //genvar k;
  //for (k = 0; k < N_REGS; k++) begin
    assign hwlp_counter_0_n = hwlp_counter_0_q - 1;
	assign hwlp_counter_1_n = hwlp_counter_1_q - 1;
  //end

  always @(posedge clk, negedge rst_n)
  begin : HWLOOP_REGS_COUNTER
    if (rst_n == 1'b0)
    begin
      hwlp_counter_0_q <= 0;
	  hwlp_counter_1_q <= 0;
    end
    else
    begin
      //for (i = 0; i < N_REGS; i++)
      //begin
        if (hwlp_we_i[2] == 1'b1) begin
		  if (hwlp_regid_i)
			hwlp_counter_1_q <= hwlp_cnt_data_i;
		  else
			hwlp_counter_0_q <= hwlp_cnt_data_i;
        end else begin
          if (hwlp_dec_cnt_i[i] && valid_i) begin
            hwlp_counter_0_q <= hwlp_counter_0_n;
			hwlp_counter_1_q <= hwlp_counter_1_n;
		  end
        end
      //end
    end
  end

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------
  //`ifndef VERILATOR
    // do not decrement more than one counter at once
  //  assert property (
  //    @(posedge clk) (valid_i) |-> ($countones(hwlp_dec_cnt_i) <= 1) );
 // `endif
endmodule
