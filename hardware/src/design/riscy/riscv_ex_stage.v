`include "apu_macros.v"

module riscv_ex_stage
#(
`include "apu_core_package.v"
`include "riscv_defines.v"
  parameter FPU              =  0,
  parameter SHARED_FP        =  0,
  parameter SHARED_DSP_MULT  =  0,
  parameter SHARED_INT_DIV   =  0,
  parameter APU_NARGS_CPU    =  3,
  parameter APU_WOP_CPU      =  6,
  parameter APU_NDSFLAGS_CPU = 15,
  parameter APU_NUSFLAGS_CPU =  5
)
(
  input  wire        clk,
  input  wire        rst_n,

  // ALU signals from ID stage
  input  wire [ALU_OP_WIDTH-1:0] alu_operator_i,
  input  wire [31:0] alu_operand_a_i,
  input  wire [31:0] alu_operand_b_i,
  input  wire [31:0] alu_operand_c_i,
  input  wire        alu_en_i,
  input  wire [ 4:0] bmask_a_i,
  input  wire [ 4:0] bmask_b_i,
  input  wire [ 1:0] imm_vec_ext_i,
  input  wire [ 1:0] alu_vec_mode_i,

  // Multiplier signals
  input  wire [ 2:0] mult_operator_i,
  input  wire [31:0] mult_operand_a_i,
  input  wire [31:0] mult_operand_b_i,
  input  wire [31:0] mult_operand_c_i,
  input  wire        mult_en_i,
  input  wire        mult_sel_subword_i,
  input  wire [ 1:0] mult_signed_mode_i,
  input  wire [ 4:0] mult_imm_i,

  input  wire [31:0] mult_dot_op_a_i,
  input  wire [31:0] mult_dot_op_b_i,
  input  wire [31:0] mult_dot_op_c_i,
  input  wire [ 1:0] mult_dot_signed_i,

  output wire        mult_multicycle_o,

  // FPU signals
  input  wire [C_CMD-1:0]            fpu_op_i,
  input  wire [C_PC-1:0]             fpu_prec_i,
  output wire [C_FFLAG-1:0]          fpu_fflags_o,
  output wire                        fpu_fflags_we_o,

  // APU signals
  input  wire                        apu_en_i,
  input  wire [APU_WOP_CPU-1:0]      apu_op_i,
  input  wire [1:0]                  apu_lat_i,
  input  wire [31:0]                 apu_operands_0_i,
  input  wire [31:0]                 apu_operands_1_i,
  input  wire [31:0]                 apu_operands_2_i,
  input  wire [5:0]                  apu_waddr_i,
  input  wire [APU_NDSFLAGS_CPU-1:0] apu_flags_i,

  input  wire [5:0]             	 apu_read_regs_0_i,
  input  wire [5:0]             	 apu_read_regs_1_i,
  input  wire [5:0]             	 apu_read_regs_2_i,
  input  wire [2:0]                  apu_read_regs_valid_i,
  output wire                        apu_read_dep_o,
  input  wire [5:0]     	         apu_write_regs_0_i,
  input  wire [5:0]     	         apu_write_regs_1_i,
  input  wire [1:0]                  apu_write_regs_valid_i,
  output wire                        apu_write_dep_o,

  output wire                        apu_perf_type_o,
  output wire                        apu_perf_cont_o,
  output wire                        apu_perf_wb_o,

  output wire                        apu_busy_o,
  output wire                        apu_ready_wb_o,

  // apu-interconnect
  // handshake signals
  output wire                       apu_master_req_o,
  output wire                       apu_master_ready_o,
  input wire                        apu_master_gnt_i,
  // request channel
  output wire [31:0]                apu_master_operands_0_o,
  output wire [31:0]                apu_master_operands_1_o,
  output wire [31:0]                apu_master_operands_2_o,
  output wire [APU_WOP_CPU-1:0]     apu_master_op_o,
  // response channel
  input wire                        apu_master_valid_i,
  input wire [31:0]                 apu_master_result_i,

  input  wire        lsu_en_i,
  input  wire [31:0] lsu_rdata_i,

  // input from ID stage
  input  wire        branch_in_ex_i,
  input  wire [5:0]  regfile_alu_waddr_i,
  input  wire        regfile_alu_we_i,

  // directly passed through to WB stage, not used in EX
  input  wire        regfile_we_i,
  input  wire [5:0]  regfile_waddr_i,

  // CSR access
  input  wire        csr_access_i,
  input  wire [31:0] csr_rdata_i,

  // Output of EX stage pipeline
  output reg [5:0]  regfile_waddr_wb_o,
  output reg        regfile_we_wb_o,
  output reg [31:0] regfile_wdata_wb_o,

  // Forwarding ports : to ID stage
  output reg  [5:0] regfile_alu_waddr_fw_o,
  output reg        regfile_alu_we_fw_o,
  output reg [31:0] regfile_alu_wdata_fw_o,    // forward to RF and ID/EX pipe, ALU & MUL

  // To IF: Jump and branch target and decision
  output wire [31:0] jump_target_o,
  output wire        branch_decision_o,

  // Stall Control
  input  wire        lsu_ready_ex_i, // EX part of LSU is done

  output wire        ex_ready_o, // EX stage ready for new data
  output wire        ex_valid_o, // EX stage gets new data
  input  wire        wb_ready_i  // WB stage ready for new data
);


  wire [31:0]    alu_result;
  wire [31:0]    mult_result;
  wire           alu_cmp_result;

  reg           regfile_we_lsu;
  reg [5:0]     regfile_waddr_lsu;

  reg           wb_contention;
  reg           wb_contention_lsu;

  wire           alu_ready;
  wire           mult_ready;
  wire           fpu_busy;


  // APU signals
  wire           apu_valid;
  wire [5:0]     apu_waddr;
  wire [31:0]    apu_result;
  wire           apu_stall;
  wire           apu_active;
  wire           apu_singlecycle;
  wire           apu_multicycle;
  wire           apu_req;
  wire           apu_ready;
  wire           apu_gnt;

  // ALU write port mux
  always @(*)
  begin
    regfile_alu_wdata_fw_o = 0;
    regfile_alu_waddr_fw_o = 0;
    regfile_alu_we_fw_o    = 0;
    wb_contention          = 1'b0;

    // APU single cycle operations, and multicycle operations (>2cycles) are written back on ALU port
    if (apu_valid & (apu_singlecycle | apu_multicycle)) begin
      regfile_alu_we_fw_o    = 1'b1;
      regfile_alu_waddr_fw_o = apu_waddr;
      regfile_alu_wdata_fw_o = apu_result;

      if(regfile_alu_we_i & ~apu_en_i) begin
        wb_contention = 1'b1;
      end
    end else begin
      regfile_alu_we_fw_o      = regfile_alu_we_i & ~apu_en_i; // private fpu incomplete?
      regfile_alu_waddr_fw_o   = regfile_alu_waddr_i;
      if (alu_en_i)
        regfile_alu_wdata_fw_o = alu_result;
      if (mult_en_i)
        regfile_alu_wdata_fw_o = mult_result;
      if (csr_access_i)
        regfile_alu_wdata_fw_o = csr_rdata_i;
    end
  end

  // LSU write port mux
  always @(*)
  begin
    regfile_we_wb_o    = 1'b0;
    regfile_waddr_wb_o = regfile_waddr_lsu;
    regfile_wdata_wb_o = lsu_rdata_i;
    wb_contention_lsu  = 1'b0;

    if (regfile_we_lsu) begin
      regfile_we_wb_o = 1'b1;
      if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
         wb_contention_lsu = 1'b1;
//         $error("%t, wb-contention", $time);
      end
    // APU two-cycle operations are written back on LSU port
    end else if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
      regfile_we_wb_o    = 1'b1;
      regfile_waddr_wb_o = apu_waddr;
      regfile_wdata_wb_o = apu_result;
    end
  end

  // branch handling
  assign branch_decision_o = alu_cmp_result;
  assign jump_target_o     = alu_operand_c_i;


  ////////////////////////////
  //     _    _    _   _    //
  //    / \  | |  | | | |   //
  //   / _ \ | |  | | | |   //
  //  / ___ \| |__| |_| |   //
  // /_/   \_\_____\___/    //
  //                        //
  ////////////////////////////

  riscv_alu
  #(
    .SHARED_INT_DIV( SHARED_INT_DIV ),
    .FPU           ( FPU            )
    )
   alu_i
  (
    .clk                 ( clk             ),
    .rst_n               ( rst_n           ),
    .enable_i            ( alu_en_i        ),
    .operator_i          ( alu_operator_i  ),
    .operand_a_i         ( alu_operand_a_i ),
    .operand_b_i         ( alu_operand_b_i ),
    .operand_c_i         ( alu_operand_c_i ),

    .vector_mode_i       ( alu_vec_mode_i  ),
    .bmask_a_i           ( bmask_a_i       ),
    .bmask_b_i           ( bmask_b_i       ),
    .imm_vec_ext_i       ( imm_vec_ext_i   ),

    .result_o            ( alu_result      ),
    .comparison_result_o ( alu_cmp_result  ),

    .ready_o             ( alu_ready       ),
    .ex_ready_i          ( ex_ready_o      )
  );


  ////////////////////////////////////////////////////////////////
  //  __  __ _   _ _   _____ ___ ____  _     ___ _____ ____     //
  // |  \/  | | | | | |_   _|_ _|  _ \| |   |_ _| ____|  _ \    //
  // | |\/| | | | | |   | |  | || |_) | |    | ||  _| | |_) |   //
  // | |  | | |_| | |___| |  | ||  __/| |___ | || |___|  _ <    //
  // |_|  |_|\___/|_____|_| |___|_|   |_____|___|_____|_| \_\   //
  //                                                            //
  ////////////////////////////////////////////////////////////////

  riscv_mult
  #(
    .SHARED_DSP_MULT(SHARED_DSP_MULT)
   )
   mult_i
  (
    .clk             ( clk                  ),
    .rst_n           ( rst_n                ),

    .enable_i        ( mult_en_i            ),
    .operator_i      ( mult_operator_i      ),

    .short_subword_i ( mult_sel_subword_i   ),
    .short_signed_i  ( mult_signed_mode_i   ),

    .op_a_i          ( mult_operand_a_i     ),
    .op_b_i          ( mult_operand_b_i     ),
    .op_c_i          ( mult_operand_c_i     ),
    .imm_i           ( mult_imm_i           ),

    .dot_op_a_i      ( mult_dot_op_a_i      ),
    .dot_op_b_i      ( mult_dot_op_b_i      ),
    .dot_op_c_i      ( mult_dot_op_c_i      ),
    .dot_signed_i    ( mult_dot_signed_i    ),

    .result_o        ( mult_result          ),

    .multicycle_o    ( mult_multicycle_o    ),
    .ready_o         ( mult_ready           ),
    .ex_ready_i      ( ex_ready_o           )
  );

   generate
      if (FPU == 1) begin
         ////////////////////////////////////////////////////
         //     _    ____  _   _   ____ ___ ____  ____     //
         //    / \  |  _ \| | | | |  _ \_ _/ ___||  _ \    //
         //   / _ \ | |_) | | | | | | | | |\___ \| |_) |   //
         //  / ___ \|  __/| |_| | | |_| | | ___) |  __/    //
         // /_/   \_\_|    \___/  |____/___|____/|_|       //
         //                                                //
         ////////////////////////////////////////////////////

         riscv_apu_disp apu_disp_i
         (
         .clk_i              ( clk                            ),
         .rst_ni             ( rst_n                          ),

         .enable_i           ( apu_en_i                       ),
         .apu_lat_i          ( apu_lat_i                      ),
         .apu_waddr_i        ( apu_waddr_i                    ),

         .apu_waddr_o        ( apu_waddr                      ),
         .apu_multicycle_o   ( apu_multicycle                 ),
         .apu_singlecycle_o  ( apu_singlecycle                ),

         .active_o           ( apu_active                     ),
         .stall_o            ( apu_stall                      ),

         .read_regs_0_i        ( apu_read_regs_0_i                ),
		 .read_regs_1_i        ( apu_read_regs_1_i                ),
		 .read_regs_2_i        ( apu_read_regs_2_i                ),
         .read_regs_valid_i  ( apu_read_regs_valid_i          ),
         .read_dep_o         ( apu_read_dep_o                 ),
         .write_regs_0_i       ( apu_write_regs_0_i               ),
		 .write_regs_1_i       ( apu_write_regs_1_i               ),
         .write_regs_valid_i ( apu_write_regs_valid_i         ),
         .write_dep_o        ( apu_write_dep_o                ),

         .perf_type_o        ( apu_perf_type_o                ),
         .perf_cont_o        ( apu_perf_cont_o                ),

         // apu-interconnect
         // handshake signals
         .apu_master_req_o   ( apu_req                        ),
         .apu_master_ready_o ( apu_ready                      ),
         .apu_master_gnt_i   ( apu_gnt                        ),
         // response channel
         .apu_master_valid_i ( apu_valid                      )
         );

         assign apu_perf_wb_o  = wb_contention | wb_contention_lsu;
         assign apu_ready_wb_o = ~(apu_active | apu_en_i | apu_stall) | apu_valid;

         if ( SHARED_FP == 1) begin
            assign apu_master_req_o      = apu_req;
            assign apu_master_ready_o    = apu_ready;
            assign apu_gnt               = apu_master_gnt_i;
            assign apu_valid             = apu_master_valid_i;
            assign apu_master_operands_0_o = apu_operands_0_i;
			assign apu_master_operands_1_o = apu_operands_1_i;
			assign apu_master_operands_2_o = apu_operands_2_i;
            assign apu_master_op_o       = apu_op_i;
            assign apu_result            = apu_master_result_i;
            assign fpu_fflags_we_o       = apu_valid;
         end
         /*
         else begin

         //////////////////////////////
         //   ______ _____  _    _   //
         //  |  ____|  __ \| |  | |  //
         //  | |__  | |__) | |  | |  //
         //  |  __| |  ___/| |  | |  //
         //  | |    | |    | |__| |  //
         //  |_|    |_|     \____/   //
         //////////////////////////////

            fpu_private fpu_i
             (
              .clk_i          ( clk               ),
              .rst_ni         ( rst_n             ),
              // enable
              .fpu_en_i       ( apu_req           ),
              // inputs
              .operand_a_i    ( apu_operands_0_i ),
              .operand_b_i    ( apu_operands_1_i ),
              .operand_c_i    ( apu_operands_2_i ),
              .rm_i           ( apu_flags_i[2:0]  ),
              .fpu_op_i       ( fpu_op_i          ),
              .prec_i         ( fpu_prec_i        ),
              // outputs
              .result_o       ( apu_result        ),
              .valid_o        ( apu_valid         ),
              .flags_o        ( fpu_fflags_o      ),
              .divsqrt_busy_o ( fpu_busy          )
              );

            assign fpu_fflags_we_o          = apu_valid;
            assign apu_master_req_o         = 0;
            assign apu_master_ready_o       = 1'b1;
            assign apu_master_operands_0_o  = 0;
            assign apu_master_operands_1_o  = 0;
            assign apu_master_operands_2_o  = 0;
            assign apu_master_op_o          = 0;
            assign apu_gnt                  = 1'b1;

         end
*/
      end
      else begin
         // default assignements for the case when no FPU/APU is attached.
         assign apu_master_req_o         = 0;
         assign apu_master_ready_o       = 1'b1;
         assign apu_master_operands_0_o  = 0;
         assign apu_master_operands_1_o  = 0;
         assign apu_master_operands_2_o  = 0;
         assign apu_master_op_o          = 0;
         assign apu_valid       = 1'b0;
         assign apu_waddr       = 6'b0;
         assign apu_stall       = 1'b0;
         assign apu_active      = 1'b0;
         assign apu_ready_wb_o  = 1'b1;
         assign apu_perf_wb_o   = 1'b0;
         assign apu_perf_cont_o = 1'b0;
         assign apu_perf_type_o = 1'b0;
         assign apu_singlecycle = 1'b0;
         assign apu_multicycle  = 1'b0;
         assign apu_read_dep_o  = 1'b0;
         assign apu_write_dep_o = 1'b0;
         assign fpu_fflags_we_o = 1'b0;
         assign fpu_fflags_o    = 0;
      end
   endgenerate

   assign apu_busy_o = apu_active;

  ///////////////////////////////////////
  // EX/WB Pipeline Register           //
  ///////////////////////////////////////
  always @(posedge clk, negedge rst_n)
  begin : EX_WB_Pipeline_Register
    if (~rst_n)
    begin
      regfile_waddr_lsu   <= 0;
      regfile_we_lsu      <= 1'b0;
    end
    else
    begin
      if (ex_valid_o) // wb_ready_i is implied
      begin
        regfile_we_lsu    <= regfile_we_i;
        if (regfile_we_i) begin
          regfile_waddr_lsu <= regfile_waddr_i;
        end
      end else if (wb_ready_i) begin
        // we are ready for a new instruction, but there is none available,
        // so we just flush the current one out of the pipe
        regfile_we_lsu    <= 1'b0;
      end
    end
  end

  // As valid always goes to the right and ready to the left, and we are able
  // to finish branches without going to the WB stage, ex_valid does not
  // depend on ex_ready.
  assign ex_ready_o = (~apu_stall & alu_ready & mult_ready & lsu_ready_ex_i
                       & wb_ready_i & ~wb_contention) | (branch_in_ex_i);
  assign ex_valid_o = (apu_valid | alu_en_i | mult_en_i | csr_access_i | lsu_en_i)
                       & (alu_ready & mult_ready & lsu_ready_ex_i & wb_ready_i);

endmodule
