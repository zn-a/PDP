
// Source/Destination register instruction index
`define REG_S1 19:15
`define REG_S2 24:20
`define REG_S4 31:27
`define REG_D  11:07

module riscv_id_stage
#(
`include "apu_core_package.v"
`include "riscv_defines.v"
  parameter N_HWLP            =  2,
  parameter N_HWLP_BITS       =  $clog2(N_HWLP),
  parameter PULP_SECURE       =  0,
  parameter FPU               =  0,
  parameter APU               =  0,
  parameter SHARED_FP         =  0,
  parameter SHARED_DSP_MULT   =  0,
  parameter SHARED_INT_DIV    =  0,
  parameter SHARED_FP_DIVSQRT =  0,
  parameter WAPUTYPE          =  0,
  parameter APU_NARGS_CPU     =  3,
  parameter APU_WOP_CPU       =  6,
  parameter APU_NDSFLAGS_CPU  = 15,
  parameter APU_NUSFLAGS_CPU  =  5
)
(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        restart,

    input  wire        test_en_i,

    input  wire        fetch_enable_i,
    output wire        ctrl_busy_o,
    output wire        core_ctrl_firstfetch_o,
    output wire        is_decoding_o,

    // Interface to IF stage
    input  wire [N_HWLP-1:0] hwlp_dec_cnt_i,
    input  wire              is_hwlp_i,
    input  wire              instr_valid_i,
    input  wire       [31:0] instr_rdata_i,      // comes from pipeline of IF stage
    output wire              instr_req_o,


    // Jumps and branches
    output reg         branch_in_ex_o,
    input  wire        branch_decision_i,
    output wire [31:0] jump_target_o,

    // IF and ID stage signals
    output wire        clear_instr_valid_o,
    output wire        pc_set_o,
    output wire [2:0]  pc_mux_o,
    output wire [1:0]  exc_pc_mux_o,
    output wire        trap_addr_mux_o,

    input  wire        illegal_c_insn_i,
    input  wire        is_compressed_i,

    input  wire [31:0] pc_if_i,
    input  wire [31:0] pc_id_i,

    // Stalls
    output wire        halt_if_o,      // controller requests a halt of the IF stage

    output wire        id_ready_o,     // ID stage is ready for the next instruction
    input  wire        ex_ready_i,     // EX stage is ready for the next instruction
    input  wire        wb_ready_i,     // WB stage is ready for the next instruction

    output wire        id_valid_o,     // ID stage is done
    input  wire        ex_valid_i,     // EX stage is done

    // Pipeline ID/EX
    output reg [31:0] pc_ex_o,

    output reg [31:0] alu_operand_a_ex_o,
    output reg [31:0] alu_operand_b_ex_o,
    output reg [31:0] alu_operand_c_ex_o,
    output wire [31:0] alu_operand_a_lsu_o,
    output wire [31:0] alu_operand_b_lsu_o,
    output wire [31:0] alu_operand_c_lsu_o,
    output reg [ 4:0] bmask_a_ex_o,
    output reg [ 4:0] bmask_b_ex_o,
    output reg [ 1:0] imm_vec_ext_ex_o,
    output reg [ 1:0] alu_vec_mode_ex_o,

    output reg [5:0]  regfile_waddr_ex_o,
    output reg        regfile_we_ex_o,

    output reg [5:0]  regfile_alu_waddr_ex_o,
    output reg        regfile_alu_we_ex_o,

    // ALU
    output reg         alu_en_ex_o,
    output reg  [ALU_OP_WIDTH-1:0] alu_operator_ex_o,


    // MUL
    output reg [ 2:0] mult_operator_ex_o,
    output reg [31:0] mult_operand_a_ex_o,
    output reg [31:0] mult_operand_b_ex_o,
    output reg [31:0] mult_operand_c_ex_o,
    output reg        mult_en_ex_o,
    output reg        mult_sel_subword_ex_o,
    output reg [ 1:0] mult_signed_mode_ex_o,
    output reg [ 4:0] mult_imm_ex_o,

    output reg [31:0] mult_dot_op_a_ex_o,
    output reg [31:0] mult_dot_op_b_ex_o,
    output reg [31:0] mult_dot_op_c_ex_o,
    output reg [ 1:0] mult_dot_signed_ex_o,

    // FPU
    output reg [C_CMD-1:0]           fpu_op_ex_o,

    // APU
    output reg                        apu_en_ex_o,
    output reg [WAPUTYPE-1:0]         apu_type_ex_o,
    output reg [APU_WOP_CPU-1:0]      apu_op_ex_o,
    output reg [1:0]                  apu_lat_ex_o,
    output reg [31:0]                 apu_operands_ex_0_o,
    output reg [31:0]                 apu_operands_ex_1_o,
	output reg [31:0]                 apu_operands_ex_2_o,
	output reg [APU_NDSFLAGS_CPU-1:0] apu_flags_ex_o,
    output reg [5:0]                  apu_waddr_ex_o,

    output wire [5:0]	              apu_read_regs_0_o,
    output wire [5:0]	              apu_read_regs_1_o,
    output wire [5:0]	              apu_read_regs_2_o,
    output wire [2:0]                 apu_read_regs_valid_o,
    input  wire                       apu_read_dep_i,
    output wire [5:0]    	          apu_write_regs_0_o,
    output wire [5:0]    	          apu_write_regs_1_o,
    output wire [1:0]                 apu_write_regs_valid_o,
    input  wire                       apu_write_dep_i,
    output wire                       apu_perf_dep_o,
    input  wire                       apu_busy_i,
    input wire [C_RM-1:0]             frm_i,

    // CSR ID/EX
    output reg         csr_access_ex_o,
    output reg  [1:0]  csr_op_ex_o,
    input  wire [PRIVLVL_T_SIZE-1:0]    current_priv_lvl_i,
    output wire        csr_irq_sec_o,
    output wire [5:0]  csr_cause_o,
    output wire        csr_save_if_o,
    output wire        csr_save_id_o,
    output wire        csr_restore_mret_id_o,
    output wire        csr_restore_uret_id_o,
    output wire        csr_save_cause_o,

    // hwloop signals
    output wire [31:0] hwlp_start_0_o,
    output wire [31:0] hwlp_end_0_o,
    output wire [31:0] hwlp_cnt_0_o,
    output wire [31:0] hwlp_start_1_o,
    output wire [31:0] hwlp_end_1_o,
    output wire [31:0] hwlp_cnt_1_o,

    // hwloop signals from CS register
    input  wire   [N_HWLP_BITS-1:0] csr_hwlp_regid_i,
    input  wire               [2:0] csr_hwlp_we_i,
    input  wire              [31:0] csr_hwlp_data_i,

    // Interface to load store unit
    output reg         data_req_ex_o,
    output reg         data_we_ex_o,
    output reg  [1:0]  data_type_ex_o,
    output reg         data_sign_ext_ex_o,
    output reg  [1:0]  data_reg_offset_ex_o,
    output reg         data_load_event_ex_o,

    output reg         data_misaligned_ex_o,

    output reg         prepost_useincr_ex_o,
    input  wire        data_misaligned_i,

    // Interrupt signals
    input  wire        irq_i,
    input  wire        irq_sec_i,
    input  wire [4:0]  irq_id_i,
    input  wire        m_irq_enable_i,
    input  wire        u_irq_enable_i,
    output wire        irq_ack_o,
    output wire [4:0]  irq_id_o,
    output wire [5:0]  exc_cause_o,

    input  wire        lsu_load_err_i,
    input  wire        lsu_store_err_i,

    // Debug Unit Signals
    input  wire [DBG_SETS_W-1:0] dbg_settings_i,
    input  wire        dbg_req_i,
    output wire        dbg_ack_o,
    input  wire        dbg_stall_i,
    output wire        dbg_trap_o,

    input  wire        dbg_reg_rreq_i,
    input  wire [ 5:0] dbg_reg_raddr_i,
    output wire [31:0] dbg_reg_rdata_o,

    input  wire        dbg_reg_wreq_i,
    input  wire [ 5:0] dbg_reg_waddr_i,
    input  wire [31:0] dbg_reg_wdata_i,

    input  wire        dbg_jump_req_i,

    // Forward Signals
    input  wire [5:0]  regfile_waddr_wb_i,
    input  wire        regfile_we_wb_i,
    input  wire [31:0] regfile_wdata_wb_i, // From wb_stage: selects data from data memory, ex_stage result and sp rdata

    input  wire [5:0]  regfile_alu_waddr_fw_i,
    input  wire        regfile_alu_we_fw_i,
    input  wire [31:0] regfile_alu_wdata_fw_i,

    // from ALU
    input  wire        mult_multicycle_i,    // when we need multiple cycles in the multiplier and use op c as storage

    // Performance Counters
    output wire        perf_jump_o,          // we are executing a jump instruction
    output wire        perf_jr_stall_o,      // jump-register-hazard
    output wire        perf_ld_stall_o      // load-use-hazard
);

  wire [31:0] instr;

  // Decoder/Controller ID stage internal signals
  wire        deassert_we;

  wire        illegal_insn_dec;
  wire        ebrk_insn;
  wire        mret_insn_dec;
  wire        uret_insn_dec;
  wire        ecall_insn_dec;
  wire        pipe_flush_dec;

  wire        rega_used_dec;
  wire        regb_used_dec;
  wire        regc_used_dec;

  wire        branch_taken_ex;
  wire [1:0]  jump_in_id;
  wire [1:0]  jump_in_dec;

  wire        misaligned_stall;
  wire        jr_stall;
  wire        load_stall;
  wire        csr_apu_stall;
  wire        instr_multicycle;

  wire        halt_id;


  // Immediate decoding and sign extension
  wire [31:0] imm_i_type;
  wire [31:0] imm_iz_type;
  wire [31:0] imm_s_type;
  wire [31:0] imm_sb_type;
  wire [31:0] imm_u_type;
  wire [31:0] imm_uj_type;
  wire [31:0] imm_z_type;
  wire [31:0] imm_s2_type;
  wire [31:0] imm_bi_type;
  wire [31:0] imm_s3_type;
  wire [31:0] imm_vs_type;
  wire [31:0] imm_vu_type;
  wire [31:0] imm_shuffleb_type;
  wire [31:0] imm_shuffleh_type;
  reg  [31:0] imm_shuffle_type;
  wire [31:0] imm_clip_type;

  reg  [31:0] imm_a;       // contains the immediate for operand b
  reg  [31:0] imm_b;       // contains the immediate for operand b

  reg [31:0] jump_target;       // calculated jump target (-> EX -> IF)



  // Signals running between controller and exception controller
  wire       irq_req_ctrl, irq_sec_ctrl;
  wire [4:0] irq_id_ctrl;
  wire       exc_ack, exc_kill;// handshake

  // Register file interface
  wire [5:0]  regfile_addr_ra_id;
  wire [5:0]  regfile_addr_rb_id;
  reg  [5:0]  regfile_addr_rc_id;

  wire        regfile_fp_a;
  wire        regfile_fp_b;
  wire        regfile_fp_c;
  wire        regfile_fp_d;

  wire [5:0]  regfile_waddr_id;
  wire [5:0]  regfile_alu_waddr_id;
  wire        regfile_alu_we_id;

  wire [31:0] regfile_data_ra_id;
  wire [31:0] regfile_data_rb_id;
  wire [31:0] regfile_data_rc_id;

  // ALU Control
  wire        alu_en;
  wire [ALU_OP_WIDTH-1:0] alu_operator;
  wire [2:0]  alu_op_a_mux_sel;
  wire [2:0]  alu_op_b_mux_sel;
  wire [1:0]  alu_op_c_mux_sel;
  wire [1:0]  regc_mux;

  wire [0:0]  imm_a_mux_sel;
  wire [3:0]  imm_b_mux_sel;
  wire [1:0]  jump_target_mux_sel;

  // Multiplier Control
  wire [2:0]  mult_operator;    // multiplication operation selection
  wire        mult_en;          // multiplication is used instead of ALU
  wire        mult_int_en;      // use integer multiplier
  wire        mult_sel_subword; // Select a subword when doing multiplications
  wire [1:0]  mult_signed_mode; // Signed mode multiplication at the output of the controller, and before the pipe registers
  wire        mult_dot_en;      // use dot product
  wire [1:0]  mult_dot_signed;  // Signed mode dot products (can be mixed types)

  // FPU signals
  wire [C_CMD-1:0]           fpu_op;

  // APU signals
  wire                        apu_en;
  wire [WAPUTYPE-1:0]         apu_type;
  wire [APU_WOP_CPU-1:0]      apu_op;
  wire [1:0]                  apu_lat;
  wire [31:0]                 apu_operands_0;
  wire [31:0]                 apu_operands_1;
  wire [31:0]                 apu_operands_2;
  reg  [APU_NDSFLAGS_CPU-1:0] apu_flags;
  wire [5:0]                  apu_waddr;

  reg  [5:0]            apu_read_regs_0;
  reg  [5:0]            apu_read_regs_1;
  reg  [5:0]            apu_read_regs_2;
  reg  [2:0]                 apu_read_regs_valid;
  wire [5:0]            apu_write_regs_0;
  wire [5:0]            apu_write_regs_1;
  wire [1:0]                 apu_write_regs_valid;

  wire [WAPUTYPE-1:0]        apu_flags_src;
  wire                       apu_stall;
  wire [2:0]                 fp_rnd_mode;

  // Register Write Control
  wire        regfile_we_id;
  wire        regfile_alu_waddr_mux_sel;

  // Data Memory Control
  wire        data_we_id;
  wire [1:0]  data_type_id;
  wire        data_sign_ext_id;
  wire [1:0]  data_reg_offset_id;
  wire        data_req_id;
  wire        data_load_event_id;

  // hwloop signals
  wire [N_HWLP_BITS-1:0] hwloop_regid, hwloop_regid_int;
  wire             [2:0] hwloop_we, hwloop_we_int;
  wire                   hwloop_target_mux_sel;
  wire                   hwloop_start_mux_sel;
  wire                   hwloop_cnt_mux_sel;

  reg             [31:0] hwloop_target;
  wire            [31:0] hwloop_start;
  reg			  [31:0] hwloop_start_int;
  wire            [31:0] hwloop_end;
  wire            [31:0] hwloop_cnt;
  reg             [31:0] hwloop_cnt_int;

  wire                   hwloop_valid;

  // CSR control
  wire        csr_access;
  wire [1:0]  csr_op;
  wire        csr_status;

  wire        prepost_useincr;

  // Forwarding
  wire [1:0]  operand_a_fw_mux_sel;
  wire [1:0]  operand_b_fw_mux_sel;
  wire [1:0]  operand_c_fw_mux_sel;
  reg  [31:0] operand_a_fw_id;
  reg  [31:0] operand_b_fw_id;
  reg  [31:0] operand_c_fw_id;

  reg [31:0] operand_b, operand_b_vec;

  reg [31:0] alu_operand_a;
  wire [31:0] alu_operand_b;
  reg [31:0] alu_operand_c;

  // Immediates for ID
  wire [0:0]  bmask_a_mux;
  wire [1:0]  bmask_b_mux;
  wire        alu_bmask_a_mux_sel;
  wire        alu_bmask_b_mux_sel;
  wire [0:0]  mult_imm_mux;

  reg  [ 4:0] bmask_a_id_imm;
  reg  [ 4:0] bmask_b_id_imm;
  reg  [ 4:0] bmask_a_id;
  reg  [ 4:0] bmask_b_id;
  wire [ 1:0] imm_vec_ext_id;
  reg  [ 4:0] mult_imm_id;

  wire [ 1:0] alu_vec_mode;
  wire        scalar_replication;

  // Forwarding detection signals
  wire        reg_d_ex_is_reg_a_id;
  wire        reg_d_ex_is_reg_b_id;
  wire        reg_d_ex_is_reg_c_id;
  wire        reg_d_wb_is_reg_a_id;
  wire        reg_d_wb_is_reg_b_id;
  wire        reg_d_wb_is_reg_c_id;
  wire        reg_d_alu_is_reg_a_id;
  wire        reg_d_alu_is_reg_b_id;
  wire        reg_d_alu_is_reg_c_id;


  assign		alu_operand_a_lsu_o        =  alu_operand_a;
  assign		alu_operand_b_lsu_o        =  alu_operand_b;
  assign		alu_operand_c_lsu_o        =  alu_operand_c;
  
  assign instr = instr_rdata_i;

  // immediate extraction and sign extension
  assign imm_i_type  = { {20 {instr[31]}}, instr[31:20] };
  assign imm_iz_type = {            20'b0, instr[31:20] };
  assign imm_s_type  = { {20 {instr[31]}}, instr[31:25], instr[11:7] };
  assign imm_sb_type = { {19 {instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 };
  assign imm_u_type  = { instr[31:12], 12'b0 };
  assign imm_uj_type = { {12 {instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0 };

  // immediate for CSR manipulatin (zero extended)
  assign imm_z_type  = { 27'b0, instr[`REG_S1] };

  assign imm_s2_type = { 27'b0, instr[24:20] };
  assign imm_bi_type = { {27{instr[24]}}, instr[24:20] };
  assign imm_s3_type = { 27'b0, instr[29:25] };
  assign imm_vs_type = { {26 {instr[24]}}, instr[24:20], instr[25] };
  assign imm_vu_type = { 26'b0, instr[24:20], instr[25] };

  // same format as rS2 for shuffle needs, expands immediate
  assign imm_shuffleb_type = {6'b0, instr[28:27], 6'b0, instr[24:23], 6'b0, instr[22:21], 6'b0, instr[20], instr[25]};
  assign imm_shuffleh_type = {15'h0, instr[20], 15'h0, instr[25]};

  // clipping immediate, uses a small barrel shifter to pre-process the
  // immediate and an adder to subtract 1
  // The end result is a mask that has 1's set in the lower part
  // TODO: check if this can be shared with the bit-manipulation unit
  assign imm_clip_type    = (32'h1 << instr[24:20]) - 1;

  //---------------------------------------------------------------------------
  // source register selection regfile_fp_x=1 <=> REG_x is a FP-register
  //---------------------------------------------------------------------------
  assign regfile_addr_ra_id = {regfile_fp_a, instr[`REG_S1]};
  assign regfile_addr_rb_id = {regfile_fp_b, instr[`REG_S2]};

  // register C mux
  always @(*)
  begin
     case (regc_mux)
      REGC_ZERO:  regfile_addr_rc_id = 0;
      REGC_RD:    regfile_addr_rc_id = {regfile_fp_c, instr[`REG_D]};
      REGC_S1:    regfile_addr_rc_id = {regfile_fp_c, instr[`REG_S1]};
      REGC_S4:    regfile_addr_rc_id = {regfile_fp_c, instr[`REG_S4]};
      default:    regfile_addr_rc_id = 0;
    endcase
  end

  //---------------------------------------------------------------------------
  // destination registers regfile_fp_d=1 <=> REG_D is a FP-register
  //---------------------------------------------------------------------------
  assign regfile_waddr_id = {regfile_fp_d, instr[`REG_D]};

  // Second Register Write Address Selection
  // Used for prepost load/store and multiplier
  assign regfile_alu_waddr_id = regfile_alu_waddr_mux_sel ?
                                regfile_waddr_id : regfile_addr_ra_id;

  // Forwarding control signals
  assign reg_d_ex_is_reg_a_id  = (regfile_waddr_ex_o     == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != 0);
  assign reg_d_ex_is_reg_b_id  = (regfile_waddr_ex_o     == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != 0);
  assign reg_d_ex_is_reg_c_id  = (regfile_waddr_ex_o     == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != 0);
  assign reg_d_wb_is_reg_a_id  = (regfile_waddr_wb_i     == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != 0);
  assign reg_d_wb_is_reg_b_id  = (regfile_waddr_wb_i     == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != 0);
  assign reg_d_wb_is_reg_c_id  = (regfile_waddr_wb_i     == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != 0);
  assign reg_d_alu_is_reg_a_id = (regfile_alu_waddr_fw_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != 0);
  assign reg_d_alu_is_reg_b_id = (regfile_alu_waddr_fw_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != 0);
  assign reg_d_alu_is_reg_c_id = (regfile_alu_waddr_fw_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != 0);



  // kill instruction in the IF/ID stage by setting the instr_valid_id control
  // signal to 0 for instructions that are done
  assign clear_instr_valid_o = id_ready_o | halt_id | branch_taken_ex;

  assign branch_taken_ex     = branch_in_ex_o & branch_decision_i;


  assign mult_en = mult_int_en | mult_dot_en;

  ///////////////////////////////////////////////
  //  _   ___        ___     ___   ___  ____   //
  // | | | \ \      / / |   / _ \ / _ \|  _ \  //
  // | |_| |\ \ /\ / /| |  | | | | | | | |_) | //
  // |  _  | \ V  V / | |__| |_| | |_| |  __/  //
  // |_| |_|  \_/\_/  |_____\___/ \___/|_|     //
  //                                           //
  ///////////////////////////////////////////////

  // hwloop register id
  assign hwloop_regid_int = instr[7];   // rd contains hwloop register id

  // hwloop target mux
  always @(*)
  begin
    case (hwloop_target_mux_sel)
      1'b0: hwloop_target = pc_id_i + {imm_iz_type[30:0], 1'b0};
      1'b1: hwloop_target = pc_id_i + {imm_z_type[30:0], 1'b0};
    endcase
  end

  // hwloop start mux
  always @(*)
  begin
    case (hwloop_start_mux_sel)
      1'b0: hwloop_start_int = hwloop_target;   // for PC + I imm
      1'b1: hwloop_start_int = pc_if_i;         // for next PC
    endcase
  end


  // hwloop cnt mux
  always @(*)
  begin : hwloop_cnt_mux
    case (hwloop_cnt_mux_sel)
      1'b0: hwloop_cnt_int = imm_iz_type;
      1'b1: hwloop_cnt_int = operand_a_fw_id;
    endcase
  end

  // multiplex between access from instructions and access via CSR registers
  assign hwloop_start = hwloop_we_int[0] ? hwloop_start_int : csr_hwlp_data_i;
  assign hwloop_end   = hwloop_we_int[1] ? hwloop_target    : csr_hwlp_data_i;
  assign hwloop_cnt   = hwloop_we_int[2] ? hwloop_cnt_int   : csr_hwlp_data_i;
  assign hwloop_regid = (|hwloop_we_int) ? hwloop_regid_int : csr_hwlp_regid_i;
  assign hwloop_we    = (|hwloop_we_int) ? hwloop_we_int    : csr_hwlp_we_i;


  //////////////////////////////////////////////////////////////////
  //      _                         _____                    _    //
  //     | |_   _ _ __ ___  _ __   |_   _|_ _ _ __ __ _  ___| |_  //
  //  _  | | | | | '_ ` _ \| '_ \    | |/ _` | '__/ _` |/ _ \ __| //
  // | |_| | |_| | | | | | | |_) |   | | (_| | | | (_| |  __/ |_  //
  //  \___/ \__,_|_| |_| |_| .__/    |_|\__,_|_|  \__, |\___|\__| //
  //                       |_|                    |___/           //
  //////////////////////////////////////////////////////////////////

  always @(*)
  begin : jump_target_mux
     case (jump_target_mux_sel)
      JT_JAL:  jump_target = pc_id_i + imm_uj_type;
      JT_COND: jump_target = pc_id_i + imm_sb_type;

      // JALR: Cannot forward RS1, since the path is too long
      JT_JALR: jump_target = regfile_data_ra_id + imm_i_type;
      default:  jump_target = regfile_data_ra_id + imm_i_type;
    endcase
  end

  assign jump_target_o = jump_target;


  ////////////////////////////////////////////////////////
  //   ___                                 _      _     //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| |    / \    //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` |   / _ \   //
  // | |_| | |_) |  __/ | | (_| | | | | (_| |  / ___ \  //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_| /_/   \_\ //
  //       |_|                                          //
  ////////////////////////////////////////////////////////

  // ALU_Op_a Mux
  always @(*)
  begin : alu_operand_a_mux
    case (alu_op_a_mux_sel)
      OP_A_REGA_OR_FWD:  alu_operand_a = operand_a_fw_id;
      OP_A_REGB_OR_FWD:  alu_operand_a = operand_b_fw_id;
      OP_A_REGC_OR_FWD:  alu_operand_a = operand_c_fw_id;
      OP_A_CURRPC:       alu_operand_a = pc_id_i;
      OP_A_IMM:          alu_operand_a = imm_a;
      default:           alu_operand_a = operand_a_fw_id;
    endcase // case (alu_op_a_mux_sel)
  end

  always @(*)
  begin : immediate_a_mux
     case (imm_a_mux_sel)
      IMMA_Z:      imm_a = imm_z_type;
      IMMA_ZERO:   imm_a = 0;
      default:     imm_a = 0;
    endcase
  end

  // Operand a forwarding mux
  always @(*)
  begin : operand_a_fw_mux
    case (operand_a_fw_mux_sel)
      SEL_FW_EX:    operand_a_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_a_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_a_fw_id = regfile_data_ra_id;
      default:      operand_a_fw_id = regfile_data_ra_id;
    endcase // case (operand_a_fw_mux_sel)
  end

  //////////////////////////////////////////////////////
  //   ___                                 _   ____   //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| | | __ )  //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` | |  _ \  //
  // | |_| | |_) |  __/ | | (_| | | | | (_| | | |_) | //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_| |____/  //
  //       |_|                                        //
  //////////////////////////////////////////////////////

  // Immediate Mux for operand B
  // TODO: check if sign-extension stuff works well here, maybe able to save
  // some area here
  always @(*)
  begin : immediate_b_mux
     case (imm_b_mux_sel)
      IMMB_I:      imm_b = imm_i_type;
      IMMB_S:      imm_b = imm_s_type;
      IMMB_U:      imm_b = imm_u_type;
      IMMB_PCINCR: imm_b = (is_compressed_i && (~data_misaligned_i)) ? 32'h2 : 32'h4;
      IMMB_S2:     imm_b = imm_s2_type;
      IMMB_BI:     imm_b = imm_bi_type;
      IMMB_S3:     imm_b = imm_s3_type;
      IMMB_VS:     imm_b = imm_vs_type;
      IMMB_VU:     imm_b = imm_vu_type;
      IMMB_SHUF:   imm_b = imm_shuffle_type;
      IMMB_CLIP:   imm_b = {1'b0, imm_clip_type[31:1]};
      default:     imm_b = imm_i_type;
    endcase
  end

  // ALU_Op_b Mux
  always @(*)
  begin : alu_operand_b_mux
    case (alu_op_b_mux_sel)
      OP_B_REGA_OR_FWD:  operand_b = operand_a_fw_id;
      OP_B_REGB_OR_FWD:  operand_b = operand_b_fw_id;
      OP_B_REGC_OR_FWD:  operand_b = operand_c_fw_id;
      OP_B_IMM:          operand_b = imm_b;
      OP_B_BMASK:        operand_b = $unsigned(operand_b_fw_id[4:0]);
      default:           operand_b = operand_b_fw_id;
    endcase // case (alu_op_b_mux_sel)
  end


  // scalar replication for operand B and shuffle type
  always @(*)
  begin
    if (alu_vec_mode == VEC_MODE8) begin
      operand_b_vec    = {4{operand_b[7:0]}};
      imm_shuffle_type = imm_shuffleb_type;
    end else begin
      operand_b_vec    = {2{operand_b[15:0]}};
      imm_shuffle_type = imm_shuffleh_type;
    end
  end

  // choose normal or scalar replicated version of operand b
  assign alu_operand_b = (scalar_replication == 1'b1) ? operand_b_vec : operand_b;


  // Operand b forwarding mux
  always @(*)
  begin : operand_b_fw_mux
    case (operand_b_fw_mux_sel)
      SEL_FW_EX:    operand_b_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_b_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_b_fw_id = regfile_data_rb_id;
      default:      operand_b_fw_id = regfile_data_rb_id;
    endcase // case (operand_b_fw_mux_sel)
  end


  //////////////////////////////////////////////////////
  //   ___                                 _    ____  //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| |  / ___| //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` | | |     //
  // | |_| | |_) |  __/ | | (_| | | | | (_| | | |___  //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_|  \____| //
  //       |_|                                        //
  //////////////////////////////////////////////////////

  // ALU OP C Mux
  always @(*)
  begin : alu_operand_c_mux
    case (alu_op_c_mux_sel)
      OP_C_REGC_OR_FWD:  alu_operand_c = operand_c_fw_id;
      OP_C_REGB_OR_FWD:  alu_operand_c = operand_b_fw_id;
      OP_C_JT:           alu_operand_c = jump_target;
      default:           alu_operand_c = operand_c_fw_id;
    endcase // case (alu_op_c_mux_sel)
  end

  // Operand c forwarding mux
  always @(*)
  begin : operand_c_fw_mux
    case (operand_c_fw_mux_sel)
      SEL_FW_EX:    operand_c_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_c_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_c_fw_id = regfile_data_rc_id;
      default:      operand_c_fw_id = regfile_data_rc_id;
    endcase // case (operand_c_fw_mux_sel)
  end


  ///////////////////////////////////////////////////////////////////////////
  //  ___                              _ _       _              ___ ____   //
  // |_ _|_ __ ___  _ __ ___   ___  __| (_) __ _| |_ ___  ___  |_ _|  _ \  //
  //  | || '_ ` _ \| '_ ` _ \ / _ \/ _` | |/ _` | __/ _ \/ __|  | || | | | //
  //  | || | | | | | | | | | |  __/ (_| | | (_| | ||  __/\__ \  | || |_| | //
  // |___|_| |_| |_|_| |_| |_|\___|\__,_|_|\__,_|\__\___||___/ |___|____/  //
  //                                                                       //
  ///////////////////////////////////////////////////////////////////////////

  always @(*)
  begin
     case (bmask_a_mux)
      BMASK_A_ZERO: bmask_a_id_imm = 0;
      BMASK_A_S3:   bmask_a_id_imm = imm_s3_type[4:0];
      default:      bmask_a_id_imm = 0;
    endcase
  end
  always @(*)
  begin
     case (bmask_b_mux)
      BMASK_B_ZERO: bmask_b_id_imm = 0;
      BMASK_B_ONE:  bmask_b_id_imm = 5'd1;
      BMASK_B_S2:   bmask_b_id_imm = imm_s2_type[4:0];
      BMASK_B_S3:   bmask_b_id_imm = imm_s3_type[4:0];
      default:      bmask_b_id_imm = 0;
    endcase
  end

  always @(*)
  begin
     case (alu_bmask_a_mux_sel)
      BMASK_A_IMM: bmask_a_id = bmask_a_id_imm;
      BMASK_A_REG: bmask_a_id = operand_b_fw_id[9:5];
      default:     bmask_a_id = bmask_a_id_imm;
    endcase
  end
  always @(*)
  begin
     case (alu_bmask_b_mux_sel)
      BMASK_B_IMM: bmask_b_id = bmask_b_id_imm;
      BMASK_B_REG: bmask_b_id = operand_b_fw_id[4:0];
      default:     bmask_b_id = bmask_b_id_imm;
    endcase
  end

  assign imm_vec_ext_id = imm_vu_type[1:0];


  always @(*)
  begin
     case (mult_imm_mux)
      MIMM_ZERO: mult_imm_id = 0;
      MIMM_S3:   mult_imm_id = imm_s3_type[4:0];
      default:   mult_imm_id = 0;
    endcase
  end

  /////////////////////////////
  // APU operand assignment  //
  /////////////////////////////
  // read regs
  generate
  if (APU == 1) begin : apu_op_preparation

     if (APU_NARGS_CPU >= 1)
       assign apu_operands_0 = alu_operand_a;
     if (APU_NARGS_CPU >= 2)
       assign apu_operands_1 = alu_operand_b;
     if (APU_NARGS_CPU >= 3)
       assign apu_operands_2 = alu_operand_c;

     // write reg
     assign apu_waddr = regfile_alu_waddr_id;

     // flags
     always @(*)
       begin
           case (apu_flags_src)
            APU_FLAGS_INT_MULT:
              apu_flags = {7'h0 , mult_imm_id, mult_signed_mode, mult_sel_subword};
            APU_FLAGS_DSP_MULT:
              apu_flags = {13'h0, mult_dot_signed};
            APU_FLAGS_FP:
              if (FPU == 1) begin
                 if (fp_rnd_mode == 3'b111)
                   apu_flags = frm_i;
                 else
                   apu_flags = fp_rnd_mode;
              end else
                apu_flags = 0;
            default:
              apu_flags = 0;
          endcase
       end

     // dependency checks
     always @(*)
       begin
           case (alu_op_a_mux_sel)
            OP_A_REGA_OR_FWD: begin
               apu_read_regs_0        = regfile_addr_ra_id;
               apu_read_regs_valid [0] = 1'b1;
            end // OP_A_REGA_OR_FWD:
            OP_A_REGB_OR_FWD: begin
               apu_read_regs_0        = regfile_addr_rb_id;
               apu_read_regs_valid[0]  = 1'b1;
            end
            default: begin
               apu_read_regs_0        = regfile_addr_ra_id;
               apu_read_regs_valid [0] = 1'b0;
            end
          endcase
       end

     always @(*)
       begin
           case (alu_op_b_mux_sel)
            OP_B_REGB_OR_FWD: begin
               apu_read_regs_1       = regfile_addr_rb_id;
               apu_read_regs_valid[1] = 1'b1;
            end
            OP_B_REGC_OR_FWD: begin
               apu_read_regs_1       = regfile_addr_rc_id;
               apu_read_regs_valid[1] = 1'b1;
            end
            default: begin
               apu_read_regs_1        = regfile_addr_rb_id;
               apu_read_regs_valid [1] = 1'b0;
            end
          endcase
       end

     always @(*)
       begin
           case (alu_op_c_mux_sel)
            OP_C_REGB_OR_FWD: begin
               apu_read_regs_2        = regfile_addr_rb_id;
               apu_read_regs_valid[2] = 1'b1;
            end
            OP_C_REGC_OR_FWD: begin
               apu_read_regs_2       = regfile_addr_rc_id;
               apu_read_regs_valid[2] = 1'b1;
            end
            default: begin
               apu_read_regs_2        = regfile_addr_rc_id;
               apu_read_regs_valid [2] = 1'b0;
            end
          endcase
       end

     assign apu_write_regs_0        = regfile_alu_waddr_id;
     assign apu_write_regs_valid [0] = regfile_alu_we_id;

     assign apu_write_regs_1        = regfile_waddr_id;
     assign apu_write_regs_valid[1]  = regfile_we_id;

     assign apu_read_regs_0_o          = apu_read_regs_0;
	 assign apu_read_regs_1_o          = apu_read_regs_1;
	 assign apu_read_regs_2_o          = apu_read_regs_2;
     assign apu_read_regs_valid_o    = apu_read_regs_valid;
	 

     assign apu_write_regs_0_o         = apu_write_regs_0;
	 assign apu_write_regs_1_o         = apu_write_regs_1;
     assign apu_write_regs_valid_o   = apu_write_regs_valid;
  end
     else begin
        assign apu_operands_0          = 0;
		assign apu_operands_1          = 0;
		assign apu_operands_2          = 0;
        assign apu_waddr               = 0;
        
        assign apu_write_regs_0_o        = 0;
		assign apu_write_regs_1_o        = 0;
        assign apu_read_regs_0_o         = 0;
		assign apu_read_regs_1_o         = 0;
		assign apu_read_regs_2_o         = 0;
        assign apu_write_regs_valid_o  = 0;
        assign apu_read_regs_valid_o   = 0;
		
		always @(*) apu_flags = 0; 
     end
  endgenerate

  assign apu_perf_dep_o      = apu_stall;
  // stall when we access the CSR after a multicycle APU instruction
  assign csr_apu_stall       = (csr_access & (apu_en_ex_o & (apu_lat_ex_o[1] == 1'b1) | apu_busy_i));

  /////////////////////////////////////////////////////////
  //  ____  _____ ____ ___ ____ _____ _____ ____  ____   //
  // |  _ \| ____/ ___|_ _/ ___|_   _| ____|  _ \/ ___|  //
  // | |_) |  _|| |  _ | |\___ \ | | |  _| | |_) \___ \  //
  // |  _ <| |__| |_| || | ___) || | | |___|  _ < ___) | //
  // |_| \_\_____\____|___|____/ |_| |_____|_| \_\____/  //
  //                                                     //
  /////////////////////////////////////////////////////////
  riscv_register_file
    #(
      .ADDR_WIDTH(6),
      .FPU(FPU)
     )
  registers_i
  (
    .clk          ( clk                ),
    .rst_n        ( rst_n              ),

    .test_en_i    ( test_en_i          ),

    // Read port a
    .raddr_a_i    ( regfile_addr_ra_id ),
    .rdata_a_o    ( regfile_data_ra_id ),

    // Read port b
    .raddr_b_i    ( regfile_addr_rb_id ),
    .rdata_b_o    ( regfile_data_rb_id ),

    // Read port c
    .raddr_c_i    ( (dbg_reg_rreq_i == 1'b0) ? regfile_addr_rc_id : dbg_reg_raddr_i),
    .rdata_c_o    ( regfile_data_rc_id ),

    // Write port a
    .waddr_a_i    ( regfile_waddr_wb_i ),
    .wdata_a_i    ( regfile_wdata_wb_i ),
    .we_a_i       ( regfile_we_wb_i    ),

    // Write port b
    .waddr_b_i    ( (dbg_reg_wreq_i == 1'b0) ? regfile_alu_waddr_fw_i : dbg_reg_waddr_i  ),
    .wdata_b_i    ( (dbg_reg_wreq_i == 1'b0) ? regfile_alu_wdata_fw_i : dbg_reg_wdata_i ),
    .we_b_i       ( (dbg_reg_wreq_i == 1'b0) ? regfile_alu_we_fw_i    : 1'b1            )
  );

  assign dbg_reg_rdata_o = regfile_data_rc_id;


  ///////////////////////////////////////////////
  //  ____  _____ ____ ___  ____  _____ ____   //
  // |  _ \| ____/ ___/ _ \|  _ \| ____|  _ \  //
  // | | | |  _|| |  | | | | | | |  _| | |_) | //
  // | |_| | |__| |__| |_| | |_| | |___|  _ <  //
  // |____/|_____\____\___/|____/|_____|_| \_\ //
  //                                           //
  ///////////////////////////////////////////////

  riscv_decoder
    #(
      .FPU                 ( FPU                  ),
      .PULP_SECURE         ( PULP_SECURE          ),
      .SHARED_FP           ( SHARED_FP            ),
      .SHARED_DSP_MULT     ( SHARED_DSP_MULT      ),
      .SHARED_INT_DIV      ( SHARED_INT_DIV       ),
      .SHARED_FP_DIVSQRT   ( SHARED_FP_DIVSQRT    ),
      .WAPUTYPE            ( WAPUTYPE             ),
      .APU_WOP_CPU         ( APU_WOP_CPU          )
      )
  decoder_i
  (
    // controller related signals
    .deassert_we_i                   ( deassert_we               ),
    .data_misaligned_i               ( data_misaligned_i         ),
    .mult_multicycle_i               ( mult_multicycle_i         ),
    .instr_multicycle_o              ( instr_multicycle          ),

    .illegal_insn_o                  ( illegal_insn_dec          ),
    .ebrk_insn_o                     ( ebrk_insn                 ),
    .mret_insn_o                     ( mret_insn_dec             ),
    .uret_insn_o                     ( uret_insn_dec             ),
    .ecall_insn_o                    ( ecall_insn_dec            ),
    .pipe_flush_o                    ( pipe_flush_dec            ),

    .rega_used_o                     ( rega_used_dec             ),
    .regb_used_o                     ( regb_used_dec             ),
    .regc_used_o                     ( regc_used_dec             ),

    .reg_fp_a_o                      ( regfile_fp_a              ),
    .reg_fp_b_o                      ( regfile_fp_b              ),
    .reg_fp_c_o                      ( regfile_fp_c              ),
    .reg_fp_d_o                      ( regfile_fp_d              ),

    .bmask_a_mux_o                   ( bmask_a_mux               ),
    .bmask_b_mux_o                   ( bmask_b_mux               ),
    .alu_bmask_a_mux_sel_o           ( alu_bmask_a_mux_sel       ),
    .alu_bmask_b_mux_sel_o           ( alu_bmask_b_mux_sel       ),

    // from IF/ID pipeline
    .instr_rdata_i                   ( instr                     ),
    .illegal_c_insn_i                ( illegal_c_insn_i          ),

    // ALU signals
    .alu_en_o                        ( alu_en                    ),
    .alu_operator_o                  ( alu_operator              ),
    .alu_op_a_mux_sel_o              ( alu_op_a_mux_sel          ),
    .alu_op_b_mux_sel_o              ( alu_op_b_mux_sel          ),
    .alu_op_c_mux_sel_o              ( alu_op_c_mux_sel          ),
    .alu_vec_mode_o                  ( alu_vec_mode              ),
    .scalar_replication_o            ( scalar_replication        ),
    .imm_a_mux_sel_o                 ( imm_a_mux_sel             ),
    .imm_b_mux_sel_o                 ( imm_b_mux_sel             ),
    .regc_mux_o                      ( regc_mux                  ),

    // MUL signals
    .mult_operator_o                 ( mult_operator             ),
    .mult_int_en_o                   ( mult_int_en               ),
    .mult_sel_subword_o              ( mult_sel_subword          ),
    .mult_signed_mode_o              ( mult_signed_mode          ),
    .mult_imm_mux_o                  ( mult_imm_mux              ),
    .mult_dot_en_o                   ( mult_dot_en               ),
    .mult_dot_signed_o               ( mult_dot_signed           ),

    .fpu_op_o                        ( fpu_op                    ),
    .apu_en_o                        ( apu_en                    ),
    .apu_type_o                      ( apu_type                  ),
    .apu_op_o                        ( apu_op                    ),
    .apu_lat_o                       ( apu_lat                   ),
    .apu_flags_src_o                 ( apu_flags_src             ),
    .fp_rnd_mode_o                   ( fp_rnd_mode               ),

    // Register file control signals
    .regfile_mem_we_o                ( regfile_we_id             ),
    .regfile_alu_we_o                ( regfile_alu_we_id         ),
    .regfile_alu_waddr_sel_o         ( regfile_alu_waddr_mux_sel ),

    // CSR control signals
    .csr_access_o                    ( csr_access                ),
    .csr_status_o                    ( csr_status                ),
    .csr_op_o                        ( csr_op                    ),
    .current_priv_lvl_i              ( current_priv_lvl_i        ),

    // Data bus interface
    .data_req_o                      ( data_req_id               ),
    .data_we_o                       ( data_we_id                ),
    .prepost_useincr_o               ( prepost_useincr           ),
    .data_type_o                     ( data_type_id              ),
    .data_sign_extension_o           ( data_sign_ext_id          ),
    .data_reg_offset_o               ( data_reg_offset_id        ),
    .data_load_event_o               ( data_load_event_id        ),

    // hwloop signals
    .hwloop_we_o                     ( hwloop_we_int             ),
    .hwloop_target_mux_sel_o         ( hwloop_target_mux_sel     ),
    .hwloop_start_mux_sel_o          ( hwloop_start_mux_sel      ),
    .hwloop_cnt_mux_sel_o            ( hwloop_cnt_mux_sel        ),

    // jump/branches
    .jump_in_dec_o                   ( jump_in_dec               ),
    .jump_in_id_o                    ( jump_in_id                ),
    .jump_target_mux_sel_o           ( jump_target_mux_sel       )

  );

  ////////////////////////////////////////////////////////////////////
  //    ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
  //   / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
  //  | |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
  //  | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
  //   \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
  //                                                                //
  ////////////////////////////////////////////////////////////////////

  riscv_controller
  #(
    .FPU ( FPU )
  )
  controller_i
  (
    .clk                            ( clk                    ),
    .rst_n                          ( rst_n                  ),
    .restart                        ( restart                ),

    .fetch_enable_i                 ( fetch_enable_i         ),
    .ctrl_busy_o                    ( ctrl_busy_o            ),
    .first_fetch_o                  ( core_ctrl_firstfetch_o ),
    .is_decoding_o                  ( is_decoding_o          ),

    // decoder related signals
    .deassert_we_o                  ( deassert_we            ),

    .illegal_insn_i                 ( illegal_insn_dec       ),
    .ecall_insn_i                   ( ecall_insn_dec         ),
    .mret_insn_i                    ( mret_insn_dec          ),
    .uret_insn_i                    ( uret_insn_dec          ),
    .pipe_flush_i                   ( pipe_flush_dec         ),
    .ebrk_insn_i                    ( ebrk_insn              ),
    .csr_status_i                   ( csr_status             ),
    .instr_multicycle_i             ( instr_multicycle       ),

    // from IF/ID pipeline
    .instr_valid_i                  ( instr_valid_i          ),

    // from prefetcher
    .instr_req_o                    ( instr_req_o            ),

    // to prefetcher
    .pc_set_o                       ( pc_set_o               ),
    .pc_mux_o                       ( pc_mux_o               ),
    .exc_pc_mux_o                   ( exc_pc_mux_o           ),
    .exc_cause_o                    ( exc_cause_o            ),
    .trap_addr_mux_o                ( trap_addr_mux_o        ),

    // LSU
    .data_req_ex_i                  ( data_req_ex_o          ),
    .data_misaligned_i              ( data_misaligned_i      ),
    .data_load_event_i              ( data_load_event_id     ),

    // ALU
    .mult_multicycle_i              ( mult_multicycle_i      ),

    // APU
    .apu_en_i                       ( apu_en                 ),
    .apu_read_dep_i                 ( apu_read_dep_i         ),
    .apu_write_dep_i                ( apu_write_dep_i        ),

    .apu_stall_o                    ( apu_stall              ),

    // jump/branch control
    .branch_taken_ex_i              ( branch_taken_ex        ),
    .jump_in_id_i                   ( jump_in_id             ),
    .jump_in_dec_i                  ( jump_in_dec            ),

    // Interrupt Controller Signals
    .irq_req_ctrl_i                 ( irq_req_ctrl           ),
    .irq_sec_ctrl_i                 ( irq_sec_ctrl           ),
    .irq_id_ctrl_i                  ( irq_id_ctrl            ),
    .m_IE_i                         ( m_irq_enable_i         ),
    .u_IE_i                         ( u_irq_enable_i         ),
    .current_priv_lvl_i             ( current_priv_lvl_i     ),

    .irq_ack_o                      ( irq_ack_o              ),
    .irq_id_o                       ( irq_id_o               ),

    .exc_ack_o                      ( exc_ack                ),
    .exc_kill_o                     ( exc_kill               ),

    // CSR Controller Signals
    .csr_save_cause_o               ( csr_save_cause_o       ),
    .csr_cause_o                    ( csr_cause_o            ),
    .csr_save_if_o                  ( csr_save_if_o          ),
    .csr_save_id_o                  ( csr_save_id_o          ),
    .csr_restore_mret_id_o          ( csr_restore_mret_id_o  ),
    .csr_restore_uret_id_o          ( csr_restore_uret_id_o  ),
    .csr_irq_sec_o                  ( csr_irq_sec_o          ),

    // Debug Unit Signals
    .dbg_req_i                      ( dbg_req_i              ),
    .dbg_ack_o                      ( dbg_ack_o              ),
    .dbg_stall_i                    ( dbg_stall_i            ),
    .dbg_jump_req_i                 ( dbg_jump_req_i         ),
    .dbg_settings_i                 ( dbg_settings_i         ),
    .dbg_trap_o                     ( dbg_trap_o             ),

    // Write targets from ID
    .regfile_alu_waddr_id_i         ( regfile_alu_waddr_id   ),

    // Forwarding signals from regfile
    .regfile_we_ex_i                ( regfile_we_ex_o        ),
    .regfile_waddr_ex_i             ( regfile_waddr_ex_o     ),
    .regfile_we_wb_i                ( regfile_we_wb_i        ),

    // regfile port 2
    .regfile_alu_we_fw_i            ( regfile_alu_we_fw_i    ),

    // Forwarding detection signals
    .reg_d_ex_is_reg_a_i            ( reg_d_ex_is_reg_a_id   ),
    .reg_d_ex_is_reg_b_i            ( reg_d_ex_is_reg_b_id   ),
    .reg_d_ex_is_reg_c_i            ( reg_d_ex_is_reg_c_id   ),
    .reg_d_wb_is_reg_a_i            ( reg_d_wb_is_reg_a_id   ),
    .reg_d_wb_is_reg_b_i            ( reg_d_wb_is_reg_b_id   ),
    .reg_d_wb_is_reg_c_i            ( reg_d_wb_is_reg_c_id   ),
    .reg_d_alu_is_reg_a_i           ( reg_d_alu_is_reg_a_id  ),
    .reg_d_alu_is_reg_b_i           ( reg_d_alu_is_reg_b_id  ),
    .reg_d_alu_is_reg_c_i           ( reg_d_alu_is_reg_c_id  ),

    // Forwarding signals
    .operand_a_fw_mux_sel_o         ( operand_a_fw_mux_sel   ),
    .operand_b_fw_mux_sel_o         ( operand_b_fw_mux_sel   ),
    .operand_c_fw_mux_sel_o         ( operand_c_fw_mux_sel   ),

    // Stall signals
    .halt_if_o                      ( halt_if_o              ),
    .halt_id_o                      ( halt_id                ),

    .misaligned_stall_o             ( misaligned_stall       ),
    .jr_stall_o                     ( jr_stall               ),
    .load_stall_o                   ( load_stall             ),

    .id_ready_i                     ( id_ready_o             ),

    .ex_valid_i                     ( ex_valid_i             ),

    .wb_ready_i                     ( wb_ready_i             ),

    // Performance Counters
    .perf_jump_o                    ( perf_jump_o            ),
    .perf_jr_stall_o                ( perf_jr_stall_o        ),
    .perf_ld_stall_o                ( perf_ld_stall_o        )
  );


////////////////////////////////////////////////////////////////////////
//  _____      _       _____             _             _ _            //
// |_   _|    | |     /  __ \           | |           | | |           //
//   | | _ __ | |_    | /  \/ ___  _ __ | |_ _ __ ___ | | | ___ _ __  //
//   | || '_ \| __|   | |    / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__| //
//  _| || | | | |_ _  | \__/\ (_) | | | | |_| | | (_) | | |  __/ |    //
//  \___/_| |_|\__(_)  \____/\___/|_| |_|\__|_|  \___/|_|_|\___|_|    //
//                                                                    //
////////////////////////////////////////////////////////////////////////

  riscv_int_controller
  #(
    .PULP_SECURE(PULP_SECURE)
   )
  int_controller_i
  (
    .clk                  ( clk                ),
    .rst_n                ( rst_n              ),

    // to controller
    .irq_req_ctrl_o       ( irq_req_ctrl       ),
    .irq_sec_ctrl_o       ( irq_sec_ctrl       ),
    .irq_id_ctrl_o        ( irq_id_ctrl        ),

    .ctrl_ack_i           ( exc_ack            ),
    .ctrl_kill_i          ( exc_kill           ),

    // Interrupt signals
    .irq_i                ( irq_i              ),
    .irq_sec_i            ( irq_sec_i          ),
    .irq_id_i             ( irq_id_i           ),

    .m_IE_i               ( m_irq_enable_i     ),
    .u_IE_i               ( u_irq_enable_i     ),
    .current_priv_lvl_i   ( current_priv_lvl_i )

  );


  //////////////////////////////////////////////////////////////////////////
  //          ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
  //         / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
  // HWLOOP-| |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
  //        | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
  //         \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
  //                                                                      //
  //////////////////////////////////////////////////////////////////////////

  riscv_hwloop_regs
  #(
    .N_REGS ( N_HWLP )
  )
  hwloop_regs_i
  (
    .clk                   ( clk                       ),
    .rst_n                 ( rst_n                     ),

    // from ID
    .hwlp_start_data_i     ( hwloop_start              ),
    .hwlp_end_data_i       ( hwloop_end                ),
    .hwlp_cnt_data_i       ( hwloop_cnt                ),
    .hwlp_we_i             ( hwloop_we                 ),
    .hwlp_regid_i          ( hwloop_regid              ),

    // from controller
    .valid_i               ( hwloop_valid              ),

    // to hwloop controller
    .hwlp_start_addr_0_o     ( hwlp_start_0_o              ),
    .hwlp_end_addr_0_o       ( hwlp_end_0_o                ),
    .hwlp_counter_0_o        ( hwlp_cnt_0_o                ),
	.hwlp_start_addr_1_o     ( hwlp_start_1_o              ),
    .hwlp_end_addr_1_o       ( hwlp_end_1_o                ),
    .hwlp_counter_1_o        ( hwlp_cnt_1_o                ),

    // from hwloop controller
    .hwlp_dec_cnt_i        ( hwlp_dec_cnt_i            )
  );

  assign hwloop_valid = instr_valid_i & clear_instr_valid_o & is_hwlp_i;


  /////////////////////////////////////////////////////////////////////////////////
  //   ___ ____        _______  __  ____ ___ ____  _____ _     ___ _   _ _____   //
  //  |_ _|  _ \      | ____\ \/ / |  _ \_ _|  _ \| ____| |   |_ _| \ | | ____|  //
  //   | || | | |_____|  _|  \  /  | |_) | || |_) |  _| | |    | ||  \| |  _|    //
  //   | || |_| |_____| |___ /  \  |  __/| ||  __/| |___| |___ | || |\  | |___   //
  //  |___|____/      |_____/_/\_\ |_|  |___|_|   |_____|_____|___|_| \_|_____|  //
  //                                                                             //
  /////////////////////////////////////////////////////////////////////////////////

  always @(posedge clk, negedge rst_n)
  begin : ID_EX_PIPE_REGISTERS
    if (rst_n == 1'b0)
    begin
      alu_en_ex_o                 <= 0;
      alu_operator_ex_o           <= ALU_SLTU;
      alu_operand_a_ex_o          <= 0;
      alu_operand_b_ex_o          <= 0;
      alu_operand_c_ex_o          <= 0;
      bmask_a_ex_o                <= 0;
      bmask_b_ex_o                <= 0;
      imm_vec_ext_ex_o            <= 0;
      alu_vec_mode_ex_o           <= 0;

      mult_operator_ex_o          <= 0;
      mult_operand_a_ex_o         <= 0;
      mult_operand_b_ex_o         <= 0;
      mult_operand_c_ex_o         <= 0;
      mult_en_ex_o                <= 1'b0;
      mult_sel_subword_ex_o       <= 1'b0;
      mult_signed_mode_ex_o       <= 2'b00;
      mult_imm_ex_o               <= 0;

      mult_dot_op_a_ex_o          <= 0;
      mult_dot_op_b_ex_o          <= 0;
      mult_dot_op_c_ex_o          <= 0;
      mult_dot_signed_ex_o        <= 0;

      fpu_op_ex_o                 <= 0;

      apu_en_ex_o                 <= 0;
      apu_type_ex_o               <= 0;
      apu_op_ex_o                 <= 0;
      apu_lat_ex_o                <= 0;
      apu_operands_ex_0_o        <= 0;
      apu_operands_ex_1_o        <= 0;
      apu_operands_ex_2_o        <= 0;
      apu_flags_ex_o              <= 0;
      apu_waddr_ex_o              <= 0;


      regfile_waddr_ex_o          <= 6'b0;
      regfile_we_ex_o             <= 1'b0;

      regfile_alu_waddr_ex_o      <= 6'b0;
      regfile_alu_we_ex_o         <= 1'b0;
      prepost_useincr_ex_o        <= 1'b0;

      csr_access_ex_o             <= 1'b0;
      csr_op_ex_o                 <= CSR_OP_NONE;

      data_we_ex_o                <= 1'b0;
      data_type_ex_o              <= 2'b0;
      data_sign_ext_ex_o          <= 1'b0;
      data_reg_offset_ex_o        <= 2'b0;
      data_req_ex_o               <= 1'b0;
      data_load_event_ex_o        <= 1'b0;

      data_misaligned_ex_o        <= 1'b0;

      pc_ex_o                     <= 0;

      branch_in_ex_o              <= 1'b0;

    end
    else if (data_misaligned_i) begin
      // misaligned data access case
      if (ex_ready_i)
      begin // misaligned access case, only unstall alu operands

        // if we are using post increments, then we have to use the
        // original value of the register for the second memory access
        // => keep it stalled
        if (prepost_useincr_ex_o == 1'b1)
        begin
          alu_operand_a_ex_o        <= alu_operand_a;
        end

        alu_operand_b_ex_o          <= alu_operand_b;
        regfile_alu_we_ex_o         <= regfile_alu_we_id;
        prepost_useincr_ex_o        <= prepost_useincr;

        data_misaligned_ex_o        <= 1'b1;
      end
    end else if (mult_multicycle_i) begin
      mult_operand_c_ex_o <= alu_operand_c;
    end
    else begin
      // normal pipeline unstall case

      if (id_valid_o)
      begin // unstall the whole pipeline

        alu_en_ex_o                 <= alu_en;
        if (alu_en)
        begin // only change those registers when we actually need to
          alu_operator_ex_o         <= alu_operator;
          alu_operand_a_ex_o        <= alu_operand_a;
          alu_operand_b_ex_o        <= alu_operand_b;
          alu_operand_c_ex_o        <= alu_operand_c;
          bmask_a_ex_o              <= bmask_a_id;
          bmask_b_ex_o              <= bmask_b_id;
          imm_vec_ext_ex_o          <= imm_vec_ext_id;
          alu_vec_mode_ex_o         <= alu_vec_mode;
        end

        mult_en_ex_o                <= mult_en;
        if (mult_int_en) begin
          mult_operator_ex_o        <= mult_operator;
          mult_sel_subword_ex_o     <= mult_sel_subword;
          mult_signed_mode_ex_o     <= mult_signed_mode;
          mult_operand_a_ex_o       <= alu_operand_a;
          mult_operand_b_ex_o       <= alu_operand_b;
          mult_operand_c_ex_o       <= alu_operand_c;
          mult_imm_ex_o             <= mult_imm_id;
        end
        if (mult_dot_en) begin
          mult_operator_ex_o        <= mult_operator;
          mult_dot_signed_ex_o      <= mult_dot_signed;
          mult_dot_op_a_ex_o        <= alu_operand_a;
          mult_dot_op_b_ex_o        <= alu_operand_b;
          mult_dot_op_c_ex_o        <= alu_operand_c;
        end

        // APU pipeline
        apu_en_ex_o                 <= apu_en;
        if (apu_en) begin
          fpu_op_ex_o               <= fpu_op;
          apu_type_ex_o             <= apu_type;
          apu_op_ex_o               <= apu_op;
          apu_lat_ex_o              <= apu_lat;
          apu_operands_ex_0_o         <= apu_operands_0;
		  apu_operands_ex_1_o         <= apu_operands_1;
		  apu_operands_ex_2_o         <= apu_operands_2;
          apu_flags_ex_o            <= apu_flags;
          apu_waddr_ex_o            <= apu_waddr;
        end

        regfile_we_ex_o             <= regfile_we_id;
        if (regfile_we_id) begin
          regfile_waddr_ex_o        <= regfile_waddr_id;
        end

        regfile_alu_we_ex_o         <= regfile_alu_we_id;
        if (regfile_alu_we_id) begin
          regfile_alu_waddr_ex_o    <= regfile_alu_waddr_id;
        end

        prepost_useincr_ex_o        <= prepost_useincr;

        csr_access_ex_o             <= csr_access;
        csr_op_ex_o                 <= csr_op;

        data_req_ex_o               <= data_req_id;
        if (data_req_id)
        begin // only needed for LSU when there is an active request
          data_we_ex_o              <= data_we_id;
          data_type_ex_o            <= data_type_id;
          data_sign_ext_ex_o        <= data_sign_ext_id;
          data_reg_offset_ex_o      <= data_reg_offset_id;
          data_load_event_ex_o      <= data_load_event_id;
        end else begin
          data_load_event_ex_o      <= 1'b0;
        end

        data_misaligned_ex_o        <= 1'b0;

        if ((jump_in_id == BRANCH_COND) || data_load_event_id) begin
          pc_ex_o                   <= pc_id_i;
        end

        branch_in_ex_o              <= jump_in_id == BRANCH_COND;
      end else if(ex_ready_i) begin
        // EX stage is ready but we don't have a new instruction for it,
        // so we set all write enables to 0, but unstall the pipe

        regfile_we_ex_o             <= 1'b0;

        regfile_alu_we_ex_o         <= 1'b0;

        csr_op_ex_o                 <= CSR_OP_NONE;

        data_req_ex_o               <= 1'b0;

        data_load_event_ex_o        <= 1'b0;

        data_misaligned_ex_o        <= 1'b0;

        branch_in_ex_o              <= 1'b0;

        apu_en_ex_o                 <= 1'b0;

        alu_operator_ex_o           <= ALU_SLTU;

        mult_en_ex_o                <= 1'b0;

      end else if (csr_access_ex_o) begin
       //In the EX stage there was a CSR access, to avoid multiple
       //writes to the RF, disable regfile_alu_we_ex_o.
       //Not doing it can overwrite the RF file with the currennt CSR value rather than the old one
       regfile_alu_we_ex_o         <= 1'b0;
      end
    end
  end


  // stall control
  assign id_ready_o = ((~misaligned_stall) & (~jr_stall) & (~load_stall) & (~apu_stall) & (~csr_apu_stall) & ex_ready_i);
  assign id_valid_o = (~halt_id) & id_ready_o;


  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------
  // `ifndef VERILATOR
    // make sure that branch decision is valid when jumping
  //   assert property (
  //     @(posedge clk) (branch_in_ex_o) |-> (branch_decision_i !== 1'bx) ) else $display("%t, Branch // // decision is X", $time);
// 
    // the instruction delivered to the ID stage should always be valid
  //   assert property (
  //     @(posedge clk) (instr_valid_i & (~illegal_c_insn_i)) |-> (!$isunknown(instr_rdata_i)) ) else // $display("Instruction is valid, but has at least one X");
 // `endif
endmodule
