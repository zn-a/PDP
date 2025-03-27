//`include "riscv_config.v"

module riscv_core
#(
//`include "apu_core_package.v"
`include "riscv_defines.v"
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
  // Clock and Reset
  input  wire        clk_i,
  input  wire        rst_ni,
  input  wire        restart,

  input  wire        clock_en_i,    // enable clock, otherwise it is gated
  input  wire        test_en_i,     // enable all clock gates for testing

  // Core ID, Cluster ID and boot address are considered more or less static
  input  wire [31:0] boot_addr_i,
  input  wire [ 3:0] core_id_i,
  input  wire [ 5:0] cluster_id_i,

  // Instruction memory interface
  output wire                         instr_req_o,
  input  wire                         instr_gnt_i,
  input  wire                         instr_rvalid_i,
  output wire                  [31:0] instr_addr_o,
  input  wire [INSTR_RDATA_WIDTH-1:0] instr_rdata_i,

  // Data memory interface
  output wire        data_req_o,
  input  wire        data_gnt_i,
  input  wire        data_rvalid_i,
  output wire        data_we_o,
  output wire [3:0]  data_be_o,
  output wire [31:0] data_addr_o,
  output wire [31:0] data_wdata_o,
  input  wire [31:0] data_rdata_i,
  input  wire        data_err_i,

  // apu-interconnect
  // handshake signals
  output wire                       apu_master_req_o,
  output wire                       apu_master_ready_o,
  input wire                        apu_master_gnt_i,
  // request channel
  output wire [31:0]                 apu_master_operands_0_o,
  output wire [31:0]                 apu_master_operands_1_o,
  output wire [31:0]                 apu_master_operands_2_o,
  output wire [APU_WOP_CPU-1:0]      apu_master_op_o,
  output wire [WAPUTYPE-1:0]         apu_master_type_o,
  output wire [APU_NDSFLAGS_CPU-1:0] apu_master_flags_o,
  // response channel
  input wire                        apu_master_valid_i,
  input wire [31:0]                 apu_master_result_i,
  input wire [APU_NUSFLAGS_CPU-1:0] apu_master_flags_i,

  // Interrupt inputs
  input  wire        irq_i,                 // level sensitive IR lines
  input  wire [4:0]  irq_id_i,
  output wire        irq_ack_o,
  output wire [4:0]  irq_id_o,
  input  wire        irq_sec_i,

  output wire        sec_lvl_o,

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
  output wire        core_busy_o,

  input  wire [N_EXT_PERF_COUNTERS-1:0] ext_perf_counters_i
);

  localparam N_HWLP      = 2;
  localparam N_HWLP_BITS = 1;//$clog2(N_HWLP);
  localparam APU         = ((SHARED_DSP_MULT==1) | (SHARED_INT_DIV==1) | (FPU==1)) ? 1 : 0;

  // IF/ID signals
  wire              is_hwlp_id;
  wire [N_HWLP-1:0] hwlp_dec_cnt_id;
  wire              instr_valid_id;
  wire [31:0]       instr_rdata_id;    // Instruction sampled inside IF stage
  wire              is_compressed_id;
  wire              illegal_c_insn_id; // Illegal compressed instruction sent to ID stage
  wire [31:0]       pc_if;             // Program counter in IF stage
  wire [31:0]       pc_id;             // Program counter in ID stage

  wire              clear_instr_valid;
  wire              pc_set;
  wire [2:0]        pc_mux_id;     // Mux selector for next PC
  wire [1:0]        exc_pc_mux_id; // Mux selector for exception PC
  wire [5:0]        exc_cause;
  wire              trap_addr_mux;
  wire              lsu_load_err;
  wire              lsu_store_err;

  // ID performance counter signals
  wire        is_decoding;

  wire        useincr_addr_ex;   // Active when post increment
  wire        data_misaligned;

  wire        mult_multicycle;

  // Jump and branch target and decision (EX->IF)
  wire [31:0] jump_target_id, jump_target_ex;
  wire        branch_in_ex;
  wire        branch_decision;

  wire        ctrl_busy;
  wire        if_busy;
  wire        lsu_busy;
  wire        apu_busy;

  wire [31:0] pc_ex; // PC of last executed branch or p.elw

  // ALU Control
  wire        alu_en_ex;
  wire [ALU_OP_WIDTH-1:0] alu_operator_ex;
  wire [31:0] alu_operand_a_ex;
  wire [31:0] alu_operand_b_ex;
  wire [31:0] alu_operand_c_ex;
  wire [31:0] alu_operand_a_lsu;
  wire [31:0] alu_operand_b_lsu;
  wire [31:0] alu_operand_c_lsu;
  wire [ 4:0] bmask_a_ex;
  wire [ 4:0] bmask_b_ex;
  wire [ 1:0] imm_vec_ext_ex;
  wire [ 1:0] alu_vec_mode_ex;

  // Multiplier Control
  wire [ 2:0] mult_operator_ex;
  wire [31:0] mult_operand_a_ex;
  wire [31:0] mult_operand_b_ex;
  wire [31:0] mult_operand_c_ex;
  wire        mult_en_ex;
  wire        mult_sel_subword_ex;
  wire [ 1:0] mult_signed_mode_ex;
  wire [ 4:0] mult_imm_ex;
  wire [31:0] mult_dot_op_a_ex;
  wire [31:0] mult_dot_op_b_ex;
  wire [31:0] mult_dot_op_c_ex;
  wire [ 1:0] mult_dot_signed_ex;

  // FPU
  wire [C_CMD-1:0]           fpu_op_ex;
  wire [C_PC-1:0]            fprec_csr;
  wire [C_RM-1:0]            frm_csr;
  wire [C_FFLAG-1:0]         fflags;
  wire [C_FFLAG-1:0]         fflags_csr;
  wire                       fflags_we;


  // APU
  wire                        apu_en_ex;
  wire [WAPUTYPE-1:0]         apu_type_ex;
  wire [APU_NDSFLAGS_CPU-1:0] apu_flags_ex;
   
  wire [APU_WOP_CPU-1:0]     apu_op_ex;
  wire [1:0]                 apu_lat_ex;
  wire [31:0]                apu_operands_ex [APU_NARGS_CPU-1:0];
  wire [5:0]                 apu_waddr_ex;

  wire [5:0]     	         apu_read_regs [0:2];
  wire [2:0]                 apu_read_regs_valid;
  wire                       apu_read_dep;
  wire [5:0]     	         apu_write_regs [0:1];
  wire [1:0]                 apu_write_regs_valid;
  wire                       apu_write_dep;

  wire                       perf_apu_type;
  wire                       perf_apu_cont;
  wire                       perf_apu_dep;
  wire                       perf_apu_wb;

  // Register Write Control
  wire [5:0]  regfile_waddr_ex;
  wire        regfile_we_ex;
  wire [5:0]  regfile_waddr_fw_wb_o;        // From WB to ID
  wire        regfile_we_wb;
  wire [31:0] regfile_wdata;

  wire [5:0]  regfile_alu_waddr_ex;
  wire        regfile_alu_we_ex;

  wire [5:0]  regfile_alu_waddr_fw;
  wire        regfile_alu_we_fw;
  wire [31:0] regfile_alu_wdata_fw;

  // CSR control
  wire        csr_access_ex;
  wire  [1:0] csr_op_ex;
  wire [23:0] mtvec, utvec;

  wire        csr_access;
  wire  [1:0] csr_op;
  wire [11:0] csr_addr;
  wire [11:0] csr_addr_int;
  wire [31:0] csr_rdata;
  wire [31:0] csr_wdata;
  wire [PRIVLVL_T_SIZE-1:0]    current_priv_lvl;

  // Data Memory Control:  From ID stage (id-ex pipe) <--> load store unit
  wire        data_we_ex;
  wire [1:0]  data_type_ex;
  wire        data_sign_ext_ex;
  wire [1:0]  data_reg_offset_ex;
  wire        data_req_ex;
  wire        data_load_event_ex;
  wire        data_misaligned_ex;

  wire [31:0] lsu_rdata;

  // stall control
  wire        halt_if;
  wire        id_ready;
  wire        ex_ready;

  wire        id_valid;
  wire        ex_valid;
  wire        wb_valid;

  wire        lsu_ready_ex;
  wire        lsu_ready_wb;

  wire        apu_ready_wb;

  // Signals between instruction core interface and pipe (if and id stages)
  wire        instr_req_int;    // Id stage asserts a req to instruction core interface

  // Interrupts
  wire        m_irq_enable, u_irq_enable;
  wire        csr_irq_sec;
  wire [31:0] epc;

  wire        csr_save_cause;
  wire        csr_save_if;
  wire        csr_save_id;
  wire [5:0]  csr_cause;
  wire        csr_restore_mret_id;
  wire        csr_restore_uret_id;

  // Hardware loop controller signals
  wire [31:0] hwlp_start_0;
  wire [31:0] hwlp_end_0;
  wire [31:0] hwlp_cnt_0;
  wire [31:0] hwlp_start_1;
  wire [31:0] hwlp_end_1;
  wire [31:0] hwlp_cnt_1;

  // used to write from CS registers to hardware loop registers
  wire   [N_HWLP_BITS-1:0] csr_hwlp_regid;
  wire               [2:0] csr_hwlp_we;
  wire              [31:0] csr_hwlp_data;


  // Debug Unit
  wire [DBG_SETS_W-1:0] dbg_settings;
  wire        dbg_req;
  wire        dbg_ack;
  wire        dbg_stall;
  wire        dbg_trap;

  // Debug GPR Read Access
  wire        dbg_reg_rreq;
  wire [ 5:0] dbg_reg_raddr;
  wire [31:0] dbg_reg_rdata;

  // Debug GPR Write Access
  wire        dbg_reg_wreq;
  wire [ 5:0] dbg_reg_waddr;
  wire [31:0] dbg_reg_wdata;

  // Debug CSR Access
  wire        dbg_csr_req;
  wire [11:0] dbg_csr_addr;
  wire        dbg_csr_we;
  wire [31:0] dbg_csr_wdata;

  wire [31:0] dbg_jump_addr;
  wire        dbg_jump_req;

  // Performance Counters
  wire        perf_imiss;
  wire        perf_jump;
  wire        perf_jr_stall;
  wire        perf_ld_stall;

  //core busy signals
  wire        core_ctrl_firstfetch, core_busy_int; 
  reg		  core_busy_q;

  //Simchecker signal
  wire is_interrupt;
  assign is_interrupt = (pc_mux_id == PC_EXCEPTION) && (exc_pc_mux_id == EXC_PC_IRQ);

  // APU master signals
   generate
      if ( SHARED_FP == 1) begin
         assign apu_master_type_o  = apu_type_ex;
         assign apu_master_flags_o = apu_flags_ex;
         assign fflags_csr         = apu_master_flags_i;
      end
      else begin
         assign apu_master_type_o  = 0;
         assign apu_master_flags_o = 0;
         assign fflags_csr         = fflags;
      end
   endgenerate


`ifdef APU_TRACE

   int         apu_trace;
   string      fn;
   string      apu_waddr_trace;


   // open/close output file for writing
   initial
     begin
        wait(rst_ni == 1'b1);
        $sformat(fn, "apu_trace_core_%h_%h.log", cluster_id_i, core_id_i);
        $display("[APU_TRACER] Output filename is: %s", fn);
        apu_trace = $fopen(fn, "w");
        $fwrite(apu_trace, "time       register \tresult\n");
        
        while(1) begin
           
           @(negedge clk_i);
           if (ex_stage_i.apu_valid == 1'b1) begin
              if (ex_stage_i.apu_waddr>31)
                $sformat(apu_waddr_trace, "f%d",ex_stage_i.apu_waddr[4:0]);
              else
                $sformat(apu_waddr_trace, "x%d",ex_stage_i.apu_waddr[4:0]);
              $fwrite(apu_trace, "%t %s \t\t%h\n", $time, apu_waddr_trace, ex_stage_i.apu_result);
           end
        end

   end
   
   final
     begin
        $fclose(apu_trace);
     end
`endif
        
  //////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ _            _      __  __                                                   _    //
  //  / ___| | ___   ___| | __ |  \/  | __ _ _ __   __ _  __ _  ___ _ __ ___   ___ _ __ | |_  //
  // | |   | |/ _ \ / __| |/ / | |\/| |/ _` | '_ \ / _` |/ _` |/ _ \ '_ ` _ \ / _ \ '_ \| __| //
  // | |___| | (_) | (__|   <  | |  | | (_| | | | | (_| | (_| |  __/ | | | | |  __/ | | | |_  //
  //  \____|_|\___/ \___|_|\_\ |_|  |_|\__,_|_| |_|\__,_|\__, |\___|_| |_| |_|\___|_| |_|\__| //
  //                                                     |___/                                //
  //////////////////////////////////////////////////////////////////////////////////////////////

  wire        clk;

  wire        clock_en;
  wire        dbg_busy;

  wire        sleeping;

  // if we are sleeping on a barrier let's just wait on the instruction
  // interface to finish loading instructions
  assign core_busy_int = (data_load_event_ex & data_req_o) ? (if_busy | apu_busy) : (if_busy | ctrl_busy | lsu_busy | apu_busy);

  always @(posedge clk, negedge rst_ni)
  begin
    if (rst_ni == 1'b0) begin
      core_busy_q <= 1'b0;
    end else begin
      core_busy_q <= core_busy_int;
    end
  end

  assign core_busy_o = core_ctrl_firstfetch ? 1'b1 : core_busy_q;

  assign dbg_busy = dbg_req | dbg_csr_req | dbg_jump_req | dbg_reg_wreq | debug_req_i;

  assign clock_en = clock_en_i | core_busy_o | dbg_busy;

  assign sleeping = (~fetch_enable_i) & (~core_busy_o);


  // main clock gate of the core
  // generates all clocks except the one for the debug unit which is
  // independent
  cluster_clock_gating core_clock_gate_i
  (
    .clk_i     ( clk_i           ),
    .en_i      ( clock_en        ),
    .test_en_i ( test_en_i       ),
    .clk_o     ( clk             )
  );

  //////////////////////////////////////////////////
  //   ___ _____   ____ _____  _    ____ _____    //
  //  |_ _|  ___| / ___|_   _|/ \  / ___| ____|   //
  //   | || |_    \___ \ | | / _ \| |  _|  _|     //
  //   | ||  _|    ___) || |/ ___ \ |_| | |___    //
  //  |___|_|     |____/ |_/_/   \_\____|_____|   //
  //                                              //
  //////////////////////////////////////////////////
  riscv_if_stage
  #(
    .N_HWLP              ( N_HWLP            ),
    .RDATA_WIDTH         ( INSTR_RDATA_WIDTH ),
    .FPU                 ( FPU               )
  )
  if_stage_i
  (
    .clk                 ( clk               ),
    .rst_n               ( rst_ni            ),

    // boot address
    .boot_addr_i         ( boot_addr_i[31:8] ),

    // trap vector location
    .m_trap_base_addr_i  ( mtvec             ),
    .u_trap_base_addr_i  ( utvec             ),
    .trap_addr_mux_i     ( trap_addr_mux     ),

    // instruction request control
    .req_i               ( instr_req_int     ),

    // instruction cache interface
    .instr_req_o         ( instr_req_o       ),
    .instr_addr_o        ( instr_addr_o      ),
    .instr_gnt_i         ( instr_gnt_i       ),
    .instr_rvalid_i      ( instr_rvalid_i    ),
    .instr_rdata_i       ( instr_rdata_i     ),

    // outputs to ID stage
    .hwlp_dec_cnt_id_o   ( hwlp_dec_cnt_id   ),
    .is_hwlp_id_o        ( is_hwlp_id        ),
    .instr_valid_id_o    ( instr_valid_id    ),
    .instr_rdata_id_o    ( instr_rdata_id    ),
    .is_compressed_id_o  ( is_compressed_id  ),
    .illegal_c_insn_id_o ( illegal_c_insn_id ),
    .pc_if_o             ( pc_if             ),
    .pc_id_o             ( pc_id             ),

    // control signals
    .clear_instr_valid_i ( clear_instr_valid ),
    .pc_set_i            ( pc_set            ),
    .exception_pc_reg_i  ( epc               ), // exception return address
    .pc_mux_i            ( pc_mux_id         ), // sel for pc multiplexer
    .exc_pc_mux_i        ( exc_pc_mux_id     ),
    .exc_vec_pc_mux_i    ( exc_cause[4:0]    ),

    // from hwloop registers
    .hwlp_start_0_i        ( hwlp_start_0        ),
    .hwlp_end_0_i          ( hwlp_end_0          ),
    .hwlp_cnt_0_i          ( hwlp_cnt_0          ),
	.hwlp_start_1_i        ( hwlp_start_1        ),
    .hwlp_end_1_i          ( hwlp_end_1          ),
    .hwlp_cnt_1_i          ( hwlp_cnt_1          ),

    // from debug unit
    .dbg_jump_addr_i     ( dbg_jump_addr     ),
    .dbg_jump_req_i      ( dbg_jump_req      ),

    // Jump targets
    .jump_target_id_i    ( jump_target_id    ),
    .jump_target_ex_i    ( jump_target_ex    ),

    // pipeline stalls
    .halt_if_i           ( halt_if           ),
    .id_ready_i          ( id_ready          ),

    .if_busy_o           ( if_busy           ),
    .perf_imiss_o        ( perf_imiss        )
  );


  /////////////////////////////////////////////////
  //   ___ ____    ____ _____  _    ____ _____   //
  //  |_ _|  _ \  / ___|_   _|/ \  / ___| ____|  //
  //   | || | | | \___ \ | | / _ \| |  _|  _|    //
  //   | || |_| |  ___) || |/ ___ \ |_| | |___   //
  //  |___|____/  |____/ |_/_/   \_\____|_____|  //
  //                                             //
  /////////////////////////////////////////////////
  riscv_id_stage
  #(
    .N_HWLP                       ( N_HWLP               ),
    .PULP_SECURE                  ( PULP_SECURE          ),
    .FPU                          ( FPU                  ),
    .APU                          ( APU                  ),
    .SHARED_FP                    ( SHARED_FP            ),
    .SHARED_DSP_MULT              ( SHARED_DSP_MULT      ),
    .SHARED_INT_DIV               ( SHARED_INT_DIV       ),
    .SHARED_FP_DIVSQRT            ( SHARED_FP_DIVSQRT    ),
    .WAPUTYPE                     ( WAPUTYPE             ),
    .APU_NARGS_CPU                ( APU_NARGS_CPU        ),
    .APU_WOP_CPU                  ( APU_WOP_CPU          ),
    .APU_NDSFLAGS_CPU             ( APU_NDSFLAGS_CPU     ),
    .APU_NUSFLAGS_CPU             ( APU_NUSFLAGS_CPU     )
  )
  id_stage_i
  (
    .clk                          ( clk                  ),
    .rst_n                        ( rst_ni               ),
    .restart                      ( restart              ),

    .test_en_i                    ( test_en_i            ),

    // Processor Enable
    .fetch_enable_i               ( fetch_enable_i       ),
    .ctrl_busy_o                  ( ctrl_busy            ),
    .core_ctrl_firstfetch_o       ( core_ctrl_firstfetch ),
    .is_decoding_o                ( is_decoding          ),

    // Interface to instruction memory
    .hwlp_dec_cnt_i               ( hwlp_dec_cnt_id      ),
    .is_hwlp_i                    ( is_hwlp_id           ),
    .instr_valid_i                ( instr_valid_id       ),
    .instr_rdata_i                ( instr_rdata_id       ),
    .instr_req_o                  ( instr_req_int        ),

    // Jumps and branches
    .branch_in_ex_o               ( branch_in_ex         ),
    .branch_decision_i            ( branch_decision      ),
    .jump_target_o                ( jump_target_id       ),

    // IF and ID control signals
    .clear_instr_valid_o          ( clear_instr_valid    ),
    .pc_set_o                     ( pc_set               ),
    .pc_mux_o                     ( pc_mux_id            ),
    .exc_pc_mux_o                 ( exc_pc_mux_id        ),
    .exc_cause_o                  ( exc_cause            ),
    .trap_addr_mux_o              ( trap_addr_mux        ),
    .illegal_c_insn_i             ( illegal_c_insn_id    ),
    .is_compressed_i              ( is_compressed_id     ),

    .pc_if_i                      ( pc_if                ),
    .pc_id_i                      ( pc_id                ),

    // Stalls
    .halt_if_o                    ( halt_if              ),

    .id_ready_o                   ( id_ready             ),
    .ex_ready_i                   ( ex_ready             ),
    .wb_ready_i                   ( lsu_ready_wb         ),

    .id_valid_o                   ( id_valid             ),
    .ex_valid_i                   ( ex_valid             ),

    // From the Pipeline ID/EX
    .pc_ex_o                      ( pc_ex                ),

    .alu_en_ex_o                  ( alu_en_ex            ),
    .alu_operator_ex_o            ( alu_operator_ex      ),
    .alu_operand_a_ex_o           ( alu_operand_a_ex     ),
    .alu_operand_b_ex_o           ( alu_operand_b_ex     ),
    .alu_operand_c_ex_o           ( alu_operand_c_ex     ),
	.alu_operand_a_lsu_o           ( alu_operand_a_lsu     ),
    .alu_operand_b_lsu_o           ( alu_operand_b_lsu     ),
    .alu_operand_c_lsu_o           ( alu_operand_c_lsu     ),
    .bmask_a_ex_o                 ( bmask_a_ex           ),
    .bmask_b_ex_o                 ( bmask_b_ex           ),
    .imm_vec_ext_ex_o             ( imm_vec_ext_ex       ),
    .alu_vec_mode_ex_o            ( alu_vec_mode_ex      ),

    .regfile_waddr_ex_o           ( regfile_waddr_ex     ),
    .regfile_we_ex_o              ( regfile_we_ex        ),

    .regfile_alu_we_ex_o          ( regfile_alu_we_ex    ),
    .regfile_alu_waddr_ex_o       ( regfile_alu_waddr_ex ),

    // MUL
    .mult_operator_ex_o           ( mult_operator_ex     ), // from ID to EX stage
    .mult_en_ex_o                 ( mult_en_ex           ), // from ID to EX stage
    .mult_sel_subword_ex_o        ( mult_sel_subword_ex  ), // from ID to EX stage
    .mult_signed_mode_ex_o        ( mult_signed_mode_ex  ), // from ID to EX stage
    .mult_operand_a_ex_o          ( mult_operand_a_ex    ), // from ID to EX stage
    .mult_operand_b_ex_o          ( mult_operand_b_ex    ), // from ID to EX stage
    .mult_operand_c_ex_o          ( mult_operand_c_ex    ), // from ID to EX stage
    .mult_imm_ex_o                ( mult_imm_ex          ), // from ID to EX stage

    .mult_dot_op_a_ex_o           ( mult_dot_op_a_ex     ), // from ID to EX stage
    .mult_dot_op_b_ex_o           ( mult_dot_op_b_ex     ), // from ID to EX stage
    .mult_dot_op_c_ex_o           ( mult_dot_op_c_ex     ), // from ID to EX stage
    .mult_dot_signed_ex_o         ( mult_dot_signed_ex   ), // from ID to EX stage

    // FPU
    .fpu_op_ex_o                  ( fpu_op_ex               ),
   
    // APU
    .apu_en_ex_o                  ( apu_en_ex               ),
    .apu_type_ex_o                ( apu_type_ex             ),
    .apu_op_ex_o                  ( apu_op_ex               ),
    .apu_lat_ex_o                 ( apu_lat_ex              ),
    .apu_operands_ex_0_o            ( apu_operands_ex[0]         ),
	.apu_operands_ex_1_o            ( apu_operands_ex[1]         ),
	.apu_operands_ex_2_o            ( apu_operands_ex[2]         ),
    .apu_flags_ex_o               ( apu_flags_ex            ),
    .apu_waddr_ex_o               ( apu_waddr_ex            ),

    .apu_read_regs_0_o              ( apu_read_regs[0]           ),
	.apu_read_regs_1_o              ( apu_read_regs[1]           ),
	.apu_read_regs_2_o              ( apu_read_regs[2]           ),
    .apu_read_regs_valid_o        ( apu_read_regs_valid     ),
    .apu_read_dep_i               ( apu_read_dep            ),
    .apu_write_regs_0_o             ( apu_write_regs[0]          ),
	.apu_write_regs_1_o             ( apu_write_regs[1]          ),
    .apu_write_regs_valid_o       ( apu_write_regs_valid    ),
    .apu_write_dep_i              ( apu_write_dep           ),
    .apu_perf_dep_o               ( perf_apu_dep            ),
    .apu_busy_i                   ( apu_busy                ),
    .frm_i                        ( frm_csr                 ),

    // CSR ID/EX
    .csr_access_ex_o              ( csr_access_ex        ),
    .csr_op_ex_o                  ( csr_op_ex            ),
    .current_priv_lvl_i           ( current_priv_lvl     ),
    .csr_irq_sec_o                ( csr_irq_sec          ),
    .csr_cause_o                  ( csr_cause            ),
    .csr_save_if_o                ( csr_save_if          ), // control signal to save pc
    .csr_save_id_o                ( csr_save_id          ), // control signal to save pc
    .csr_restore_mret_id_o        ( csr_restore_mret_id  ), // control signal to restore pc
    .csr_restore_uret_id_o        ( csr_restore_uret_id  ), // control signal to restore pc
    .csr_save_cause_o             ( csr_save_cause       ),

    // hardware loop signals to IF hwlp controller
    .hwlp_start_0_o        ( hwlp_start_0        ),
    .hwlp_end_0_o          ( hwlp_end_0          ),
    .hwlp_cnt_0_o          ( hwlp_cnt_0         ),
	.hwlp_start_1_o        ( hwlp_start_1        ),
    .hwlp_end_1_o          ( hwlp_end_1          ),
    .hwlp_cnt_1_o          ( hwlp_cnt_1          ),

    // hardware loop signals from CSR
    .csr_hwlp_regid_i             ( csr_hwlp_regid       ),
    .csr_hwlp_we_i                ( csr_hwlp_we          ),
    .csr_hwlp_data_i              ( csr_hwlp_data        ),

    // LSU
    .data_req_ex_o                ( data_req_ex          ), // to load store unit
    .data_we_ex_o                 ( data_we_ex           ), // to load store unit
    .data_type_ex_o               ( data_type_ex         ), // to load store unit
    .data_sign_ext_ex_o           ( data_sign_ext_ex     ), // to load store unit
    .data_reg_offset_ex_o         ( data_reg_offset_ex   ), // to load store unit
    .data_load_event_ex_o         ( data_load_event_ex   ), // to load store unit

    .data_misaligned_ex_o         ( data_misaligned_ex   ), // to load store unit

    .prepost_useincr_ex_o         ( useincr_addr_ex      ),
    .data_misaligned_i            ( data_misaligned      ),

    // Interrupt Signals
    .irq_i                        ( irq_i                ), // incoming interrupts
    .irq_sec_i                    ( (PULP_SECURE) ? irq_sec_i : 1'b0 ),
    .irq_id_i                     ( irq_id_i             ),
    .m_irq_enable_i               ( m_irq_enable         ),
    .u_irq_enable_i               ( u_irq_enable         ),
    .irq_ack_o                    ( irq_ack_o            ),
    .irq_id_o                     ( irq_id_o             ),

    .lsu_load_err_i               ( lsu_load_err         ),
    .lsu_store_err_i              ( lsu_store_err        ),

    // Debug Unit Signals
    .dbg_settings_i               ( dbg_settings         ),
    .dbg_req_i                    ( dbg_req              ),
    .dbg_ack_o                    ( dbg_ack              ),
    .dbg_stall_i                  ( dbg_stall            ),
    .dbg_trap_o                   ( dbg_trap             ),

    .dbg_reg_rreq_i               ( dbg_reg_rreq         ),
    .dbg_reg_raddr_i              ( dbg_reg_raddr        ),
    .dbg_reg_rdata_o              ( dbg_reg_rdata        ),

    .dbg_reg_wreq_i               ( dbg_reg_wreq         ),
    .dbg_reg_waddr_i              ( dbg_reg_waddr        ),
    .dbg_reg_wdata_i              ( dbg_reg_wdata        ),

    .dbg_jump_req_i               ( dbg_jump_req         ),

    // Forward Signals
    .regfile_waddr_wb_i           ( regfile_waddr_fw_wb_o),  // Write address ex-wb pipeline
    .regfile_we_wb_i              ( regfile_we_wb        ),  // write enable for the register file
    .regfile_wdata_wb_i           ( regfile_wdata        ),  // write data to commit in the register file

    .regfile_alu_waddr_fw_i       ( regfile_alu_waddr_fw ),
    .regfile_alu_we_fw_i          ( regfile_alu_we_fw    ),
    .regfile_alu_wdata_fw_i       ( regfile_alu_wdata_fw ),

    // from ALU
    .mult_multicycle_i            ( mult_multicycle      ),

    // Performance Counters
    .perf_jump_o                  ( perf_jump            ),
    .perf_jr_stall_o              ( perf_jr_stall        ),
    .perf_ld_stall_o              ( perf_ld_stall        )
  );


  /////////////////////////////////////////////////////
  //   _______  __  ____ _____  _    ____ _____      //
  //  | ____\ \/ / / ___|_   _|/ \  / ___| ____|     //
  //  |  _|  \  /  \___ \ | | / _ \| |  _|  _|       //
  //  | |___ /  \   ___) || |/ ___ \ |_| | |___      //
  //  |_____/_/\_\ |____/ |_/_/   \_\____|_____|     //
  //                                                 //
  /////////////////////////////////////////////////////
  riscv_ex_stage
  #(
   .FPU              ( FPU                ),
   .SHARED_FP        ( SHARED_FP          ),
   .SHARED_DSP_MULT  ( SHARED_DSP_MULT    ),
   .SHARED_INT_DIV   ( SHARED_INT_DIV     ),
   .APU_NARGS_CPU    ( APU_NARGS_CPU      ),
   .APU_WOP_CPU      ( APU_WOP_CPU        ),
   .APU_NDSFLAGS_CPU ( APU_NDSFLAGS_CPU   ),
   .APU_NUSFLAGS_CPU ( APU_NUSFLAGS_CPU   )
  )
  ex_stage_i
  (
    // Global signals: Clock and active low asynchronous reset
    .clk                        ( clk                          ),
    .rst_n                      ( rst_ni                       ),

    // Alu signals from ID stage
    .alu_en_i                   ( alu_en_ex                    ),
    .alu_operator_i             ( alu_operator_ex              ), // from ID/EX pipe registers
    .alu_operand_a_i            ( alu_operand_a_ex             ), // from ID/EX pipe registers
    .alu_operand_b_i            ( alu_operand_b_ex             ), // from ID/EX pipe registers
    .alu_operand_c_i            ( alu_operand_c_ex             ), // from ID/EX pipe registers
    .bmask_a_i                  ( bmask_a_ex                   ), // from ID/EX pipe registers
    .bmask_b_i                  ( bmask_b_ex                   ), // from ID/EX pipe registers
    .imm_vec_ext_i              ( imm_vec_ext_ex               ), // from ID/EX pipe registers
    .alu_vec_mode_i             ( alu_vec_mode_ex              ), // from ID/EX pipe registers

    // Multipler
    .mult_operator_i            ( mult_operator_ex             ), // from ID/EX pipe registers
    .mult_operand_a_i           ( mult_operand_a_ex            ), // from ID/EX pipe registers
    .mult_operand_b_i           ( mult_operand_b_ex            ), // from ID/EX pipe registers
    .mult_operand_c_i           ( mult_operand_c_ex            ), // from ID/EX pipe registers
    .mult_en_i                  ( mult_en_ex                   ), // from ID/EX pipe registers
    .mult_sel_subword_i         ( mult_sel_subword_ex          ), // from ID/EX pipe registers
    .mult_signed_mode_i         ( mult_signed_mode_ex          ), // from ID/EX pipe registers
    .mult_imm_i                 ( mult_imm_ex                  ), // from ID/EX pipe registers
    .mult_dot_op_a_i            ( mult_dot_op_a_ex             ), // from ID/EX pipe registers
    .mult_dot_op_b_i            ( mult_dot_op_b_ex             ), // from ID/EX pipe registers
    .mult_dot_op_c_i            ( mult_dot_op_c_ex             ), // from ID/EX pipe registers
    .mult_dot_signed_i          ( mult_dot_signed_ex           ), // from ID/EX pipe registers

    .mult_multicycle_o          ( mult_multicycle              ), // to ID/EX pipe registers

    // FPU
    .fpu_op_i                   ( fpu_op_ex                    ),
    .fpu_prec_i                 ( fprec_csr                    ),
    .fpu_fflags_o               ( fflags                       ),
    .fpu_fflags_we_o            ( fflags_we                    ),
   
    // APU
    .apu_en_i                   ( apu_en_ex                    ),
    .apu_op_i                   ( apu_op_ex                    ),
    .apu_lat_i                  ( apu_lat_ex                   ),
    .apu_operands_0_i            ( apu_operands_ex[0]         ),
	.apu_operands_1_i            ( apu_operands_ex[1]         ),
	.apu_operands_2_i            ( apu_operands_ex[2]         ),
    .apu_waddr_i                ( apu_waddr_ex                 ),
    .apu_flags_i                ( apu_flags_ex                 ),

    .apu_read_regs_0_i              ( apu_read_regs[0]           ),
	.apu_read_regs_1_i              ( apu_read_regs[1]           ),
	.apu_read_regs_2_i              ( apu_read_regs[2]           ),
    .apu_read_regs_valid_i      ( apu_read_regs_valid          ),
    .apu_read_dep_o             ( apu_read_dep                 ),
    .apu_write_regs_0_i             ( apu_write_regs[0]          ),
	.apu_write_regs_1_i             ( apu_write_regs[1]          ),
    .apu_write_regs_valid_i     ( apu_write_regs_valid         ),
    .apu_write_dep_o            ( apu_write_dep                ),

    .apu_perf_type_o            ( perf_apu_type                ),
    .apu_perf_cont_o            ( perf_apu_cont                ),
    .apu_perf_wb_o              ( perf_apu_wb                  ),
    .apu_ready_wb_o             ( apu_ready_wb                 ),
    .apu_busy_o                 ( apu_busy                     ),

    // apu-interconnect
    // handshake signals
    .apu_master_req_o           ( apu_master_req_o             ),
    .apu_master_ready_o         ( apu_master_ready_o           ),
    .apu_master_gnt_i           ( apu_master_gnt_i             ),
    // request channel
    .apu_master_operands_0_o      ( apu_master_operands_0_o        ),
	.apu_master_operands_1_o      ( apu_master_operands_1_o        ),
	.apu_master_operands_2_o      ( apu_master_operands_2_o        ),
    .apu_master_op_o            ( apu_master_op_o              ),
    // response channel
    .apu_master_valid_i         ( apu_master_valid_i           ),
    .apu_master_result_i        ( apu_master_result_i          ),

    .lsu_en_i                   ( data_req_ex                  ),
    .lsu_rdata_i                ( lsu_rdata                    ),

    // interface with CSRs
    .csr_access_i               ( csr_access_ex                ),
    .csr_rdata_i                ( csr_rdata                    ),

    // From ID Stage: Regfile control signals
    .branch_in_ex_i             ( branch_in_ex                 ),
    .regfile_alu_waddr_i        ( regfile_alu_waddr_ex         ),
    .regfile_alu_we_i           ( regfile_alu_we_ex            ),

    .regfile_waddr_i            ( regfile_waddr_ex             ),
    .regfile_we_i               ( regfile_we_ex                ),

    // Output of ex stage pipeline
    .regfile_waddr_wb_o         ( regfile_waddr_fw_wb_o        ),
    .regfile_we_wb_o            ( regfile_we_wb                ),
    .regfile_wdata_wb_o         ( regfile_wdata                ),

    // To IF: Jump and branch target and decision
    .jump_target_o              ( jump_target_ex               ),
    .branch_decision_o          ( branch_decision              ),

    // To ID stage: Forwarding signals
    .regfile_alu_waddr_fw_o     ( regfile_alu_waddr_fw         ),
    .regfile_alu_we_fw_o        ( regfile_alu_we_fw            ),
    .regfile_alu_wdata_fw_o     ( regfile_alu_wdata_fw         ),

    // stall control
    .lsu_ready_ex_i             ( lsu_ready_ex                 ),

    .ex_ready_o                 ( ex_ready                     ),
    .ex_valid_o                 ( ex_valid                     ),
    .wb_ready_i                 ( lsu_ready_wb                 )
  );


  ////////////////////////////////////////////////////////////////////////////////////////
  //    _     ___    _    ____    ____ _____ ___  ____  _____   _   _ _   _ ___ _____   //
  //   | |   / _ \  / \  |  _ \  / ___|_   _/ _ \|  _ \| ____| | | | | \ | |_ _|_   _|  //
  //   | |  | | | |/ _ \ | | | | \___ \ | || | | | |_) |  _|   | | | |  \| || |  | |    //
  //   | |__| |_| / ___ \| |_| |  ___) || || |_| |  _ <| |___  | |_| | |\  || |  | |    //
  //   |_____\___/_/   \_\____/  |____/ |_| \___/|_| \_\_____|  \___/|_| \_|___| |_|    //
  //                                                                                    //
  ////////////////////////////////////////////////////////////////////////////////////////

  riscv_load_store_unit  load_store_unit_i
  (
    .clk                   ( clk                ),
    .rst_n                 ( rst_ni             ),

    //output to data memory
    .data_req_o            ( data_req_o         ),
    .data_gnt_i            ( data_gnt_i         ),
    .data_rvalid_i         ( data_rvalid_i      ),
    .data_err_i            ( data_err_i         ),

    .data_addr_o           ( data_addr_o        ),
    .data_we_o             ( data_we_o          ),
    .data_be_o             ( data_be_o          ),
    .data_wdata_o          ( data_wdata_o       ),
    .data_rdata_i          ( data_rdata_i       ),

    // signal from ex stage
    .data_we_ex_i          ( data_we_ex         ),
    .data_type_ex_i        ( data_type_ex       ),
    .data_wdata_ex_i       ( alu_operand_c_ex   ),
    .data_reg_offset_ex_i  ( data_reg_offset_ex ),
    .data_sign_ext_ex_i    ( data_sign_ext_ex   ),  // sign extension

    .data_rdata_ex_o       ( lsu_rdata          ),
    .data_req_ex_i         ( data_req_ex        ),
	.operand_a_ex_i        ( alu_operand_a_ex   ),
    .operand_b_ex_i        ( alu_operand_b_ex   ),
    .operand_a_lsu_i        ( alu_operand_a_lsu   ),
    .operand_b_lsu_i        ( alu_operand_b_lsu   ),
    .addr_useincr_ex_i     ( useincr_addr_ex    ),

    .data_misaligned_ex_i  ( data_misaligned_ex ), // from ID/EX pipeline
    .data_misaligned_o     ( data_misaligned    ),

    // exception signals
    .load_err_o            ( lsu_load_err       ),
    .store_err_o           ( lsu_store_err      ),

    // control signals
    .lsu_ready_ex_o        ( lsu_ready_ex       ),
    .lsu_ready_wb_o        ( lsu_ready_wb       ),

    .ex_valid_i            ( ex_valid           ),
    .busy_o                ( lsu_busy           )
  );

  assign wb_valid = lsu_ready_wb & apu_ready_wb;


  //////////////////////////////////////
  //        ____ ____  ____           //
  //       / ___/ ___||  _ \ ___      //
  //      | |   \___ \| |_) / __|     //
  //      | |___ ___) |  _ <\__ \     //
  //       \____|____/|_| \_\___/     //
  //                                  //
  //   Control and Status Registers   //
  //////////////////////////////////////

  riscv_cs_registers
  #(
    .N_EXT_CNT       ( N_EXT_PERF_COUNTERS   ),
    .FPU             ( FPU                   ),
    .APU             ( APU                   ),
    .PULP_SECURE     ( PULP_SECURE           )
  )
  cs_registers_i
  (
    .clk                     ( clk                ),
    .rst_n                   ( rst_ni             ),

    // Core and Cluster ID from outside
    .core_id_i               ( core_id_i          ),
    .cluster_id_i            ( cluster_id_i       ),
    .mtvec_o                 ( mtvec              ),
    .utvec_o                 ( utvec              ),
    // boot address
    .boot_addr_i             ( boot_addr_i[31:8]  ),
    // Interface to CSRs (SRAM like)
    .csr_access_i            ( csr_access         ),
    .csr_addr_i              ( csr_addr           ),
    .csr_wdata_i             ( csr_wdata          ),
    .csr_op_i                ( csr_op             ),
    .csr_rdata_o             ( csr_rdata          ),

    .frm_o                   ( frm_csr            ),
    .fprec_o                 ( fprec_csr          ),
    .fflags_i                ( fflags_csr         ),
    .fflags_we_i             ( fflags_we          ),
   
    // Interrupt related control signals
    .m_irq_enable_o          ( m_irq_enable       ),
    .u_irq_enable_o          ( u_irq_enable       ),
    .csr_irq_sec_i           ( csr_irq_sec        ),
    .sec_lvl_o               ( sec_lvl_o          ),
    .epc_o                   ( epc                ),
    .priv_lvl_o              ( current_priv_lvl   ),

    .pc_if_i                 ( pc_if              ),
    .pc_id_i                 ( pc_id              ), // from IF stage

    .csr_save_if_i           ( csr_save_if        ),
    .csr_save_id_i           ( csr_save_id        ),
    .csr_restore_mret_i      ( csr_restore_mret_id ),
    .csr_restore_uret_i      ( csr_restore_uret_id ),
    .csr_cause_i             ( csr_cause          ),
    .csr_save_cause_i        ( csr_save_cause     ),

    // from hwloop registers
    .hwlp_start_0_i        ( hwlp_start_0        ),
    .hwlp_end_0_i          ( hwlp_end_0          ),
    .hwlp_cnt_0_i          ( hwlp_cnt_0          ),
	.hwlp_start_1_i        ( hwlp_start_1        ),
    .hwlp_end_1_i          ( hwlp_end_1          ),
    .hwlp_cnt_1_i          ( hwlp_cnt_1          ),

    .hwlp_regid_o            ( csr_hwlp_regid     ),
    .hwlp_we_o               ( csr_hwlp_we        ),
    .hwlp_data_o             ( csr_hwlp_data      ),

    // performance counter related signals
    .id_valid_i              ( id_valid           ),
    .is_compressed_i         ( is_compressed_id   ),
    .is_decoding_i           ( is_decoding        ),

    .imiss_i                 ( perf_imiss         ),
    .pc_set_i                ( pc_set             ),
    .jump_i                  ( perf_jump          ),
    .branch_i                ( branch_in_ex       ),
    .branch_taken_i          ( branch_decision    ),
    .ld_stall_i              ( perf_ld_stall      ),
    .jr_stall_i              ( perf_jr_stall      ),

    .apu_typeconflict_i      ( perf_apu_type      ),
    .apu_contention_i        ( perf_apu_cont      ),
    .apu_dep_i               ( perf_apu_dep       ),
    .apu_wb_i                ( perf_apu_wb        ),

    .mem_load_i              ( data_req_o & data_gnt_i & (~data_we_o) ),
    .mem_store_i             ( data_req_o & data_gnt_i & data_we_o    ),

    .ext_counters_i          ( ext_perf_counters_i                    )
  );

  // Mux for CSR access through Debug Unit
  assign csr_access   = (dbg_csr_req == 1'b0) ? csr_access_ex    : 1'b1;
  assign csr_addr     = (dbg_csr_req == 1'b0) ? csr_addr_int     : dbg_csr_addr;
  assign csr_wdata    = (dbg_csr_req == 1'b0) ? alu_operand_a_ex : dbg_csr_wdata;
  assign csr_op       = (dbg_csr_req == 1'b0) ? csr_op_ex
                                              : (dbg_csr_we == 1'b1 ? CSR_OP_WRITE
                                                                    : CSR_OP_NONE );
  assign csr_addr_int = csr_access_ex ? alu_operand_b_ex[11:0] : 0;


  /////////////////////////////////////////////////////////////
  //  ____  _____ ____  _   _  ____   _   _ _   _ ___ _____  //
  // |  _ \| ____| __ )| | | |/ ___| | | | | \ | |_ _|_   _| //
  // | | | |  _| |  _ \| | | | |  _  | | | |  \| || |  | |   //
  // | |_| | |___| |_) | |_| | |_| | | |_| | |\  || |  | |   //
  // |____/|_____|____/ \___/ \____|  \___/|_| \_|___| |_|   //
  //                                                         //
  /////////////////////////////////////////////////////////////

  riscv_debug_unit debug_unit_i
  (
    .clk               ( clk_i              ), // always-running clock for debug
    .rst_n             ( rst_ni             ),

    // Debug Interface
    .debug_req_i       ( debug_req_i        ),
    .debug_gnt_o       ( debug_gnt_o        ),
    .debug_rvalid_o    ( debug_rvalid_o     ),
    .debug_addr_i      ( debug_addr_i       ),
    .debug_we_i        ( debug_we_i         ),
    .debug_wdata_i     ( debug_wdata_i      ),
    .debug_rdata_o     ( debug_rdata_o      ),
    .debug_halt_i      ( debug_halt_i       ),
    .debug_resume_i    ( debug_resume_i     ),
    .debug_halted_o    ( debug_halted_o     ),

    // To/From Core
    .settings_o        ( dbg_settings       ),
    .trap_i            ( dbg_trap           ),
    .exc_cause_i       ( exc_cause          ),
    .stall_o           ( dbg_stall          ),
    .dbg_req_o         ( dbg_req            ),
    .dbg_ack_i         ( dbg_ack            ),

    // register file read port
    .regfile_rreq_o    ( dbg_reg_rreq       ),
    .regfile_raddr_o   ( dbg_reg_raddr      ),
    .regfile_rdata_i   ( dbg_reg_rdata      ),

    // register file write port
    .regfile_wreq_o    ( dbg_reg_wreq       ),
    .regfile_waddr_o   ( dbg_reg_waddr      ),
    .regfile_wdata_o   ( dbg_reg_wdata      ),

    // CSR read/write port
    .csr_req_o         ( dbg_csr_req        ),
    .csr_addr_o        ( dbg_csr_addr       ),
    .csr_we_o          ( dbg_csr_we         ),
    .csr_wdata_o       ( dbg_csr_wdata      ),
    .csr_rdata_i       ( csr_rdata          ),

    // signals for PPC and NPC
    .pc_if_i           ( pc_if              ), // from IF stage
    .pc_id_i           ( pc_id              ), // from IF stage
    .pc_ex_i           ( pc_ex              ), // PC of last executed branch (in EX stage) or p.elw

    .data_load_event_i ( data_load_event_ex ),
    .instr_valid_id_i  ( instr_valid_id     ),

    .sleeping_i        ( sleeping           ),

    .branch_in_ex_i    ( branch_in_ex       ),
    .branch_taken_i    ( branch_decision    ),

    .jump_addr_o       ( dbg_jump_addr      ), // PC from debug unit
    .jump_req_o        ( dbg_jump_req       )  // set PC to new value
  );

endmodule
