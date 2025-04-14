//`include "riscv_core.sv"
`timescale 1ns / 1ps
module riscv_top_ahb3lite #(
					parameter BOOT_ADDR        	  = 32'h0000,
				    parameter N_EXT_PERF_COUNTERS =  0,
					parameter INSTR_RDATA_WIDTH   = 32,
					parameter PULP_SECURE         =  0,
					parameter FPU                 =  0,
					parameter SHARED_FP           =  0,
					parameter SHARED_DSP_MULT     =  0,
					parameter SHARED_INT_DIV      =  0,
					parameter SHARED_FP_DIVSQRT   =  0,
					parameter WAPUTYPE            =  0,
					parameter APU_NARGS_CPU       =  3,
					parameter APU_WOP_CPU         =  6,
					parameter APU_NDSFLAGS_CPU    = 15,
					parameter APU_NUSFLAGS_CPU    =  5
 )
(
				  //AHB interfaces
					input  wire			HCLK,
					input  wire			HRESETn,
//				        input  wire [31:0]              BOOT_ADDR,
					input  wire			REBOOT,
				  // AHB-LITE MASTER PORT - INSTRUCTION
					output wire [31:0] 	ins_HADDR,
					output wire [ 2:0] 	ins_HBURST,
					output wire        	ins_HMASTLOCK,
					output wire [ 3:0] 	ins_HPROT,
					output wire [ 2:0] 	ins_HSIZE,
					output wire [ 1:0] 	ins_HTRANS,
					output wire [31:0] 	ins_HWDATA,
					output wire        	ins_HWRITE,
					input  wire [31:0] 	ins_HRDATA,
					input  wire        	ins_HREADY,
					input  wire        	ins_HRESP,
				
				// AHB-LITE MASTER PORT - DATA				
					output wire [31:0] 	dat_addr,
					output wire [ 3:0] 	dat_wen,
					input  wire [31:0] 	dat_rdata,
          output wire [31:0] 	dat_wdata,
				  
				  //Interrupts
					input  wire [31:0]       irqs,                 // level sensitive IR lines

					// Debug Interface
					input  wire        debug_req_i,
					output wire        debug_gnt_o,
					output wire        debug_rvalid_o,
					input  wire [14:0] debug_addr_i,
					input  wire        debug_we_i,
					input  wire [31:0] debug_wdata_i,
					output wire [31:0] debug_rdata_o,
					output wire        debug_halted_o,
					input  wire        debug_halt_i,
					input  wire        debug_resume_i,

					// CPU Control Signals
					input  wire        fetch_enable_i,
					output wire        core_busy_o
);

wire			core_instr_req;
wire			core_instr_gnt;
wire			core_instr_rvalid;
wire [31:0]		core_instr_addr;
wire [31:0]		core_instr_rdata;
 
wire			core_lsu_req;
wire			core_lsu_gnt;
wire			core_lsu_rvalid;
wire [31:0]		core_lsu_addr;
wire			core_lsu_we;
wire [3:0]		core_lsu_be;
wire [31:0]		core_lsu_rdata;
wire [31:0]		core_lsu_wdata;

integer i;
reg  [4:0] irq_id;
 
  always @(*)
  begin
   irq_id = 0;
   for (i = 0; i < 32; i=i+1) begin
    if(irqs[i]) begin
     irq_id = i[4:0];
    end
  end
 end
 
 
 riscv_core
 #(
  .N_EXT_PERF_COUNTERS (     0       ),
  .FPU                 (     0       ),
  .SHARED_FP           (     0       ),
  .SHARED_FP_DIVSQRT   (     2       )
 )
 RISCV_CORE
 (
  .clk_i           ( HCLK              ),
  .rst_ni          ( HRESETn           ),
  .restart         ( REBOOT            ),
  .clock_en_i      ( 1'b1 			   ),
  .test_en_i       ( 1'b0 	           ),

  .boot_addr_i     ( BOOT_ADDR        ),
  .core_id_i       ( 4'h0              ),
  .cluster_id_i    ( 6'h0              ),

  .instr_addr_o    ( core_instr_addr   ),
  .instr_req_o     ( core_instr_req    ),
  .instr_rdata_i   ( core_instr_rdata  ),
  .instr_gnt_i     ( core_instr_gnt    ),
  .instr_rvalid_i  ( core_instr_rvalid ),

  .data_addr_o     ( core_lsu_addr     ),
  .data_wdata_o    ( core_lsu_wdata    ),
  .data_we_o       ( core_lsu_we       ),
  .data_req_o      ( core_lsu_req      ),
  .data_be_o       ( core_lsu_be       ),
  .data_rdata_i    ( core_lsu_rdata    ),
  .data_gnt_i      ( core_lsu_gnt      ),
  .data_rvalid_i   ( core_lsu_rvalid   ),
  .data_err_i      ( 1'b0              ),

  .irq_i           ( (|irqs)),
  .irq_id_i        ( irq_id            ),
  .irq_ack_o       (                   ),
  .irq_id_o        (                   ),
  .irq_sec_i       ( 1'b0              ),
  .sec_lvl_o       (                   ),

  .debug_req_i     ( 1'b0              ),
  .debug_gnt_o     (                   ),
  .debug_rvalid_o  (                   ),
  .debug_addr_i    ( 15'd0              ),
  .debug_we_i      ( 1'b0              ),
  .debug_wdata_i   ( 32'd0              ),
  .debug_rdata_o   (                   ),
  .debug_halted_o  (                   ),
  .debug_halt_i    ( 1'b0              ),
  .debug_resume_i  ( 1'b0              ),

  .fetch_enable_i  ( fetch_enable_i 	 ),
  .core_busy_o     ( core_busy_o	     ),

  // apu-interconnect
  // handshake signals
  .apu_master_req_o      (             ),
  .apu_master_ready_o    (             ),
  .apu_master_gnt_i      ( 1'b1        ),
  // request channel
  .apu_master_operands_0_o(             ),
  .apu_master_operands_1_o(             ),
  .apu_master_operands_2_o(             ),
  .apu_master_op_o       (             ),
  .apu_master_type_o     (             ),
  .apu_master_flags_o    (             ),
  // response channel
  .apu_master_valid_i    ( 1'b0          ),
  .apu_master_result_i   ( 32'd0          ),
  .apu_master_flags_i    ( 5'd0          ),

  .ext_perf_counters_i (               )
 );
 
core2ahb3lite
#(
    .AHB_ADDR_WIDTH(32),
    .AHB_DATA_WIDTH(32)
)
INST_WRAPPER
(
    .clk_i				(HCLK),
    .rst_ni				(HRESETn),

    .req_i				( core_instr_req    ),
    .gnt_o				( core_instr_gnt    ),
    .rvalid_o			( core_instr_rvalid ),
    .addr_i				( core_instr_addr   ),
    .we_i				( 1'b0			    ),
    .be_i				( 4'b1111			),
    .rdata_o			( core_instr_rdata  ),
    .wdata_i			( 32'd0			    ),

	.HADDR_o			(ins_HADDR),
	.HWDATA_o			(ins_HWDATA),
	.HRDATA_i			(ins_HRDATA),
	.HWRITE_o			(ins_HWRITE),
	.HSIZE_o			(ins_HSIZE),
	.HBURST_o			(ins_HBURST),
	.HPROT_o			(ins_HPROT),
	.HTRANS_o			(ins_HTRANS),
	.HMASTLOCK_o		(ins_HMASTLOCK),
	.HREADY_i			(ins_HREADY),
	.HRESP_i			(ins_HRESP)
	
);


coredata_2_bram #(
    .R_LATENCY_IN_CYCLES(1)
)data_bram_if(
    .clk_i(HCLK),
    .rst_ni(HRESETn),
    .req_i(core_lsu_req),
    .gnt_o(core_lsu_gnt),
    .rvalid_o(core_lsu_rvalid),
    .addr_i(core_lsu_addr),
    .we_i(core_lsu_we),
    .be_i(core_lsu_be),
    .rdata_o(core_lsu_rdata),
    .wdata_i(core_lsu_wdata),

	  .addr(dat_addr),
    .dout(dat_wdata),
    .din(dat_rdata),
    .weout(dat_wen)
	
);

endmodule
