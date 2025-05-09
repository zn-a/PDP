
`ifndef PULP_FPGA_EMUL
 `ifdef SYNTHESIS
  `define ASIC_SYNTHESIS
 `endif
`endif

module riscv_cs_registers
#(
`include "riscv_defines.v"
  parameter N_HWLP       = 2,
  parameter N_HWLP_BITS  = $clog2(N_HWLP),
  parameter N_EXT_CNT    = 0,
  parameter APU          = 0,
  parameter FPU          = 0,
  parameter PULP_SECURE  = 0
)
(
  // Clock and Reset
  input  wire            clk,
  input  wire            rst_n,

  // Core and Cluster ID
  input  wire  [3:0]     core_id_i,
  input  wire  [5:0]     cluster_id_i,
  output wire [23:0]     mtvec_o,
  output wire [23:0]     utvec_o,

  // Used for boot address
  input  wire [23:0]     boot_addr_i,

  // Interface to registers (SRAM like)
  input  wire            csr_access_i,
  input  wire [11:0]     csr_addr_i,
  input  wire [31:0]     csr_wdata_i,
  input  wire  [1:0]     csr_op_i,
  output reg  [31:0]     csr_rdata_o,

  output wire [2:0]         frm_o,
  output wire [C_PC-1:0]    fprec_o,
  input  wire [C_FFLAG-1:0] fflags_i,
  input  wire               fflags_we_i,

  // Interrupts
  output wire            m_irq_enable_o,
  output wire            u_irq_enable_o,
  //csr_irq_sec_i is always 0 if PULP_SECURE is zero
  input  wire            csr_irq_sec_i,
  output wire            sec_lvl_o,
  output reg  [31:0]     epc_o,
  
  output wire [PRIVLVL_T_SIZE-1:0] priv_lvl_o,

  input  wire [31:0]     pc_if_i,
  input  wire [31:0]     pc_id_i,
  input  wire            csr_save_if_i,
  input  wire            csr_save_id_i,
  input  wire            csr_restore_mret_i,
  input  wire            csr_restore_uret_i,
  //coming from controller
  input  wire [5:0]      csr_cause_i,
  //coming from controller
  input  wire            csr_save_cause_i,

  // Hardware loops
  input  wire [31:0] hwlp_start_0_i,
  input  wire [31:0] hwlp_end_0_i,
  input  wire [31:0] hwlp_cnt_0_i,
  input  wire [31:0] hwlp_start_1_i,
  input  wire [31:0] hwlp_end_1_i,
  input  wire [31:0] hwlp_cnt_1_i,

  output wire [31:0]              hwlp_data_o,
  output reg  [N_HWLP_BITS-1:0]   hwlp_regid_o,
  output reg  [2:0]               hwlp_we_o,

  // Performance Counters
  input  wire                 id_valid_i,        // ID stage is done
  input  wire                 is_compressed_i,   // compressed instruction in ID
  input  wire                 is_decoding_i,     // controller is in DECODE state

  input  wire                 imiss_i,           // instruction fetch
  input  wire                 pc_set_i,          // pc was set to a new value
  input  wire                 jump_i,            // jump instruction seen   (j, jr, jal, jalr)
  input  wire                 branch_i,          // branch instruction seen (bf, bnf)
  input  wire                 branch_taken_i,    // branch was taken
  input  wire                 ld_stall_i,        // load use hazard
  input  wire                 jr_stall_i,        // jump register use hazard

  input  wire                 apu_typeconflict_i,
  input  wire                 apu_contention_i,
  input  wire                 apu_dep_i,
  input  wire                 apu_wb_i,

  input  wire                 mem_load_i,        // load from memory in this cycle
  input  wire                 mem_store_i,       // store to memory in this cycle

  input  wire [N_EXT_CNT-1:0] ext_counters_i
);


  localparam N_APU_CNT       = (APU==1) ? 4 : 0;
  localparam N_PERF_COUNTERS = 12 + N_EXT_CNT + N_APU_CNT;

  localparam PERF_EXT_ID   = 11;
  localparam PERF_APU_ID   = PERF_EXT_ID + 1 + N_EXT_CNT;


`ifdef ASIC_SYNTHESIS
  localparam N_PERF_REGS     = 1;
`else
  localparam N_PERF_REGS     = N_PERF_COUNTERS;
`endif

  `define MSTATUS_UIE_BITS        0
  `define MSTATUS_SIE_BITS        1
  `define MSTATUS_MIE_BITS        3
  `define MSTATUS_UPIE_BITS       4
  `define MSTATUS_SPIE_BITS       5
  `define MSTATUS_MPIE_BITS       7
  `define MSTATUS_SPP_BITS        8
  `define MSTATUS_MPP_BITS    12:11

  localparam STATUS_T_SIZE = 8;
  localparam uie = 0, sie = 1, hie = 2, mie = 3, upie = 4, spie = 5, hpie = 6, mpie = 7;
  

  // CSR update wire
  reg  [31:0] csr_wdata_int;
  reg  [31:0] csr_rdata_int;
  reg         csr_we_int;
  reg  [C_RM-1:0]     frm_q, frm_n;
  reg  [C_FFLAG-1:0]  fflags_q, fflags_n;
  reg  [C_PC-1:0]     fprec_q, fprec_n;

  // Interrupt control signals
  reg  [31:0] mepc_q, mepc_n;
  reg  [31:0] uepc_q, uepc_n;
  reg  [31:0] exception_pc;
  reg  [STATUS_T_SIZE-1:0] mstatus_q, mstatus_n;
  reg  [PRIVLVL_T_SIZE-1:0] mstatus_q_mpp, mstatus_n_mpp; 
  reg  [ 5:0] mcause_q, mcause_n;
  reg  [ 5:0] ucause_q, ucause_n;
  //not implemented yet
  reg  [23:0] mtvec_n, mtvec_reg_q;
  wire [23:0]  mtvec_q;
  reg  [23:0] utvec_n, utvec_q;

  wire is_irq;
  reg	[PRIVLVL_T_SIZE-1:0] priv_lvl_n, priv_lvl_q, priv_lvl_reg_q;

  // Performance Counter Signals
  reg                          id_valid_q;
  wire[N_PERF_COUNTERS-1:0]    PCCR_in;  // input signals for each counter category
  reg [N_PERF_COUNTERS-1:0]    PCCR_inc, PCCR_inc_q; // should the counter be increased?

  reg [31:0] PCCR_q [0:N_PERF_REGS-1];
  reg [31:0] PCCR_n [0:N_PERF_REGS-1]; // performance counters counter register
  reg  [1:0]                    PCMR_n, PCMR_q; // mode register, controls saturation and global enable
  reg [N_PERF_COUNTERS-1:0]    PCER_n, PCER_q; // selected counter input

  reg  [31:0]                   perf_rdata;
  reg  [4:0]                    pccr_index;
  reg                           pccr_all_sel;
  reg                           is_pccr;
  reg                           is_pcer;
  reg                           is_pcmr;


  assign is_irq = csr_cause_i[5];

  ////////////////////////////////////////////
  //   ____ ____  ____    ____              //
  //  / ___/ ___||  _ \  |  _ \ ___  __ _   //
  // | |   \___ \| |_) | | |_) / _ \/ _` |  //
  // | |___ ___) |  _ <  |  _ <  __/ (_| |  //
  //  \____|____/|_| \_\ |_| \_\___|\__, |  //
  //                                |___/   //
  ////////////////////////////////////////////
generate
if(PULP_SECURE==1) begin
  // read wire
  always @(*)
  begin
    case (csr_addr_i)
      // fcsr: Floating-Point Control and Status Register (frm + fflags).
      12'h001: csr_rdata_int = (FPU == 1) ? {27'b0, fflags_q}        : 0;
      12'h002: csr_rdata_int = (FPU == 1) ? {29'b0, frm_q}           : 0;
      12'h003: csr_rdata_int = (FPU == 1) ? {24'b0, frm_q, fflags_q} : 0;
      12'h006: csr_rdata_int = (FPU == 1) ? {27'b0, fprec_q}         : 0; // Optional precision control for FP DIV/SQRT Unit
      // mstatus
      12'h300: csr_rdata_int = {
                                  19'b0,
                                  mstatus_q_mpp,
                                  3'b0,
                                  mstatus_q[mpie],
                                  2'h0,
                                  mstatus_q[upie],
                                  mstatus_q[mie],
                                  2'h0,
                                  mstatus_q[uie]
                                };
      // mtvec: machine trap-handler base address
      12'h305: csr_rdata_int = {mtvec_q, 8'h0};
      // mepc: exception program counter
      12'h341: csr_rdata_int = mepc_q;
      // mcause: exception cause
      12'h342: csr_rdata_int = {mcause_q[5], 26'b0, mcause_q[4:0]};
      // mhartid:  hardware thread id
      12'hF14: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
      // hardware loops  (not official)
      12'h7B0: csr_rdata_int = hwlp_start_0_i;
      12'h7B1: csr_rdata_int = hwlp_end_0_i;
      12'h7B2: csr_rdata_int = hwlp_cnt_0_i;
      12'h7B4: csr_rdata_int = hwlp_start_1_i;
      12'h7B5: csr_rdata_int = hwlp_end_1_i;
      12'h7B6: csr_rdata_int = hwlp_cnt_1_i;
      /* USER CSR */
      // ustatus
      12'h000: csr_rdata_int = {
                                  27'b0,
                                  mstatus_q[upie],
                                  3'h0,
                                  mstatus_q[uie]
                                };
      // utvec: user trap-handler base address
      12'h005: csr_rdata_int = {utvec_q, 8'h0};
      // dublicated mhartid:  hardware thread id (not official)
      12'h014: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
      // uepc: exception program counter
      12'h041: csr_rdata_int = uepc_q;
      // ucause: exception cause
      12'h042: csr_rdata_int = {ucause_q[5], 26'h0, ucause_q[4:0]};
      // current priv level (not official)
      12'hC10: csr_rdata_int = {30'h0, priv_lvl_q};
      default:
        csr_rdata_int = 0;
    endcase
  end
end else begin //PULP_SECURE == 0
  // read wire
  always @(*)
  begin

    case (csr_addr_i)
      // fcsr: Floating-Point Control and Status Register (frm + fflags).
      12'h001: csr_rdata_int = (FPU == 1) ? {27'b0, fflags_q}        : 0;
      12'h002: csr_rdata_int = (FPU == 1) ? {29'b0, frm_q}           : 0;
      12'h003: csr_rdata_int = (FPU == 1) ? {24'b0, frm_q, fflags_q} : 0;
      12'h006: csr_rdata_int = (FPU == 1) ? {27'b0, fprec_q}         : 0; // Optional precision control for FP DIV/SQRT Unit
      // mstatus: always M-mode, contains IE bit
      12'h300: csr_rdata_int = {
                                  19'b0,
                                  mstatus_q_mpp,
                                  3'b0,
                                  mstatus_q[mpie],
                                  2'h0,
                                  mstatus_q[upie],
                                  mstatus_q[mie],
                                  2'h0,
                                  mstatus_q[uie]
                                };
      //misa: (no allocated ID yet)
      12'h301: csr_rdata_int = 32'h0;
      // mtvec: machine trap-handler base address
      12'h305: csr_rdata_int = {mtvec_q, 8'h0};
      // mepc: exception program counter
      12'h341: csr_rdata_int = mepc_q;
      // mcause: exception cause
      12'h342: csr_rdata_int = {mcause_q[5], 26'b0, mcause_q[4:0]};
      // mhartid:  hardware thread id
      12'hF14: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
      // hardware loops  (not official)
      12'h7B0: csr_rdata_int = hwlp_start_0_i;
      12'h7B1: csr_rdata_int = hwlp_end_0_i;
      12'h7B2: csr_rdata_int = hwlp_cnt_0_i;
      12'h7B4: csr_rdata_int = hwlp_start_1_i;
      12'h7B5: csr_rdata_int = hwlp_end_1_i;
      12'h7B6: csr_rdata_int = hwlp_cnt_1_i;
      /* USER CSR */
      // dublicated mhartid:  hardware thread id (not official)
      12'h014: csr_rdata_int = {21'b0, cluster_id_i[5:0], 1'b0, core_id_i[3:0]};
      // current priv level (not official)
      12'hC10: csr_rdata_int = {30'h0, priv_lvl_q};
      default:
        csr_rdata_int = 0;
    endcase
  end
end //PULP_SECURE
endgenerate

generate
if(PULP_SECURE==1) begin
  // write wire
  always @(*)
  begin
    fflags_n     = fflags_q;
    frm_n        = frm_q;
    fprec_n      = fprec_q;
    epc_o        = mepc_q;
    mepc_n       = mepc_q;
    uepc_n       = uepc_q;
    mstatus_n    = mstatus_q;
	mstatus_n_mpp= mstatus_q_mpp;
    mcause_n     = mcause_q;
    ucause_n     = ucause_q;
    hwlp_we_o    = 0;
    hwlp_regid_o = 0;
    exception_pc = pc_id_i;
    priv_lvl_n   = priv_lvl_q;
    mtvec_n      = mtvec_q;
    utvec_n      = utvec_q;

    if (FPU == 1) if (fflags_we_i) fflags_n = fflags_i | fflags_q;

    case (csr_addr_i)
      // fcsr: Floating-Point Control and Status Register (frm, fflags, fprec).
      12'h001: if (csr_we_int) fflags_n = (FPU == 1) ? csr_wdata_int[C_FFLAG-1:0] : 0;
      12'h002: if (csr_we_int) frm_n    = (FPU == 1) ? csr_wdata_int[C_RM-1:0]    : 0;
      12'h003: if (csr_we_int) begin
         fflags_n = (FPU == 1) ? csr_wdata_int[C_FFLAG-1:0]            : 0;
         frm_n    = (FPU == 1) ? csr_wdata_int[C_RM+C_FFLAG-1:C_FFLAG] : 0;
      end
      12'h006: if (csr_we_int) fprec_n = (FPU == 1) ? csr_wdata_int[C_PC-1:0]    : 0;

      // mstatus: IE bit
      12'h300: if (csr_we_int) begin
        mstatus_n = {csr_wdata_int[`MSTATUS_UIE_BITS],1'b0,1'b0,csr_wdata_int[`MSTATUS_MIE_BITS],csr_wdata_int[`MSTATUS_UPIE_BITS],1'b0,1'b0,csr_wdata_int[`MSTATUS_MPIE_BITS]};
        mstatus_n_mpp = csr_wdata_int[`MSTATUS_MPP_BITS];
      end
      // mtvec: machine trap-handler base address
      12'h305: if (csr_we_int) begin
        mtvec_n    = csr_wdata_int[31:8];
      end
      // mepc: exception program counter
      12'h341: if (csr_we_int) begin
        mepc_n       = csr_wdata_int;
      end
      // mcause
      12'h342: if (csr_we_int) mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};

      // hardware loops
      12'h7B0: if (csr_we_int) begin hwlp_we_o = 3'b001; hwlp_regid_o = 1'b0; end
      12'h7B1: if (csr_we_int) begin hwlp_we_o = 3'b010; hwlp_regid_o = 1'b0; end
      12'h7B2: if (csr_we_int) begin hwlp_we_o = 3'b100; hwlp_regid_o = 1'b0; end
      12'h7B4: if (csr_we_int) begin hwlp_we_o = 3'b001; hwlp_regid_o = 1'b1; end
      12'h7B5: if (csr_we_int) begin hwlp_we_o = 3'b010; hwlp_regid_o = 1'b1; end
      12'h7B6: if (csr_we_int) begin hwlp_we_o = 3'b100; hwlp_regid_o = 1'b1; end
      /* USER CSR */
      // ucause: exception cause
      12'h000: if (csr_we_int) begin
        mstatus_n = {csr_wdata_int[`MSTATUS_UIE_BITS],1'b0,1'b0,mstatus_q[mie],csr_wdata_int[`MSTATUS_UPIE_BITS],1'b0,1'b0,mstatus_q[mpie]};
        mstatus_n_mpp = mstatus_q_mpp;
      end
      // utvec: user trap-handler base address
      12'h005: if (csr_we_int) begin
        utvec_n    = csr_wdata_int[31:8];
      end
      // uepc: exception program counter
      12'h041: if (csr_we_int) begin
        uepc_n     = csr_wdata_int;
      end
      // ucause: exception cause
      12'h042: if (csr_we_int) ucause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
    endcase

    // exception controller gets priority over other writes
     case (1'b1)

      csr_save_cause_i: begin

         case (1'b1)
          csr_save_if_i:
            exception_pc = pc_if_i;
          csr_save_id_i:
            exception_pc = pc_id_i;
          default:;
        endcase

         case (priv_lvl_q)

          PRIV_LVL_U: begin
            if(~is_irq) begin
              //Exceptions, Ecall U --> M
              priv_lvl_n     = PRIV_LVL_M;
              mstatus_n[mpie]= mstatus_q[uie];
              mstatus_n[mie] = 1'b0;
              mstatus_n_mpp  = PRIV_LVL_U;
              mepc_n         = exception_pc;
              mcause_n       = csr_cause_i;
            end
            else begin
              if(~csr_irq_sec_i) begin
              //U --> U
                priv_lvl_n     = PRIV_LVL_U;
                mstatus_n[upie]= mstatus_q[uie];
                mstatus_n[uie] = 1'b0;
                uepc_n         = exception_pc;
                ucause_n       = csr_cause_i;
              end else begin
              //U --> M
                priv_lvl_n     = PRIV_LVL_M;
                mstatus_n[mpie]= mstatus_q[uie];
                mstatus_n[mie] = 1'b0;
                mstatus_n_mpp  = PRIV_LVL_U;
                mepc_n         = exception_pc;
                mcause_n       = csr_cause_i;
              end
            end
          end //PRIV_LVL_U

          PRIV_LVL_M: begin
            //Exceptions or Interrupts from PRIV_LVL_M always do M --> M
            priv_lvl_n     = PRIV_LVL_M;
            mstatus_n[mpie]= mstatus_q[mie];
            mstatus_n[mie] = 1'b0;
            mstatus_n_mpp  = PRIV_LVL_M;
            mepc_n         = exception_pc;
            mcause_n       = csr_cause_i;
          end //PRIV_LVL_M
          default:;

        endcase

      end //csr_save_cause_i

      csr_restore_uret_i: begin //URET
        //mstatus_q.upp is implicitly 0, i.e PRIV_LVL_U
        mstatus_n[uie] = mstatus_q[upie];
        priv_lvl_n     = PRIV_LVL_U;
        mstatus_n[upie]= 1'b1;
        epc_o          = uepc_q;
      end //csr_restore_uret_i

      csr_restore_mret_i: begin //MRET
         case (mstatus_q_mpp)
          PRIV_LVL_U: begin
            mstatus_n[uie] = mstatus_q[mpie];
            priv_lvl_n     = PRIV_LVL_U;
            mstatus_n[mpie]= 1'b1;
            mstatus_n_mpp  = PRIV_LVL_U;
          end
          PRIV_LVL_M: begin
            mstatus_n[mie] = mstatus_q[mpie];
            priv_lvl_n     = PRIV_LVL_M;
            mstatus_n[mpie]= 1'b1;
            mstatus_n_mpp  = PRIV_LVL_U;
          end
          default:;
        endcase
        epc_o              = mepc_q;
      end //csr_restore_mret_i
      default:;
    endcase
  end
end else begin //PULP_SECURE == 0
  // write wire
  always @(*)
  begin
    fflags_n     = fflags_q;
    frm_n        = frm_q;
    fprec_n      = fprec_q;
    epc_o        = mepc_q;
    mepc_n       = mepc_q;
    mstatus_n    = mstatus_q;
	mstatus_n_mpp= mstatus_q_mpp ;
    mcause_n     = mcause_q;
    hwlp_we_o    = 0;
    hwlp_regid_o = 0;
    exception_pc = pc_id_i;
    priv_lvl_n   = priv_lvl_q;
    mtvec_n      = mtvec_q;

    if (FPU == 1) if (fflags_we_i) fflags_n = fflags_i | fflags_q;

    case (csr_addr_i)
      // fcsr: Floating-Point Control and Status Register (frm, fflags, fprec).
      12'h001: if (csr_we_int) fflags_n = (FPU == 1) ? csr_wdata_int[C_FFLAG-1:0] : 0;
      12'h002: if (csr_we_int) frm_n    = (FPU == 1) ? csr_wdata_int[C_RM-1:0]    : 0;
      12'h003: if (csr_we_int) begin
         fflags_n = (FPU == 1) ? csr_wdata_int[C_FFLAG-1:0]            : 0;
         frm_n    = (FPU == 1) ? csr_wdata_int[C_RM+C_FFLAG-1:C_FFLAG] : 0;
      end
      12'h006: if (csr_we_int) fprec_n = (FPU == 1) ? csr_wdata_int[C_PC-1:0]    : 0;

      // mstatus: IE bit
      12'h300: if (csr_we_int) begin
		mstatus_n = {csr_wdata_int[`MSTATUS_UIE_BITS],1'b0,1'b0,csr_wdata_int[`MSTATUS_MIE_BITS],csr_wdata_int[`MSTATUS_UPIE_BITS],1'b0,1'b0,csr_wdata_int[`MSTATUS_MPIE_BITS]};
        mstatus_n_mpp = csr_wdata_int[`MSTATUS_MPP_BITS];
      end
      // mepc: exception program counter
      12'h341: if (csr_we_int) begin
        mepc_n       = csr_wdata_int;
      end
      // mcause
      12'h342: if (csr_we_int) mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};

      // hardware loops
      12'h7B0: if (csr_we_int) begin hwlp_we_o = 3'b001; hwlp_regid_o = 1'b0; end
      12'h7B1: if (csr_we_int) begin hwlp_we_o = 3'b010; hwlp_regid_o = 1'b0; end
      12'h7B2: if (csr_we_int) begin hwlp_we_o = 3'b100; hwlp_regid_o = 1'b0; end
      12'h7B4: if (csr_we_int) begin hwlp_we_o = 3'b001; hwlp_regid_o = 1'b1; end
      12'h7B5: if (csr_we_int) begin hwlp_we_o = 3'b010; hwlp_regid_o = 1'b1; end
      12'h7B6: if (csr_we_int) begin hwlp_we_o = 3'b100; hwlp_regid_o = 1'b1; end
    endcase

    // exception controller gets priority over other writes
     case (1'b1)

      csr_save_cause_i: begin

         case (1'b1)
          csr_save_if_i:
            exception_pc = pc_if_i;
          csr_save_id_i:
            exception_pc = pc_id_i;
          default:;
        endcase

        priv_lvl_n     = PRIV_LVL_M;
        mstatus_n[mpie]= mstatus_q[mie];
        mstatus_n[mie] = 1'b0;
        mstatus_n_mpp  = PRIV_LVL_M;
        mepc_n         = exception_pc;
        mcause_n       = csr_cause_i;
      end //csr_save_cause_i

      csr_restore_mret_i: begin //MRET
        mstatus_n[mie] = mstatus_q[mpie];
        priv_lvl_n     = PRIV_LVL_M;
        mstatus_n[mpie]= 1'b1;
        mstatus_n_mpp  = PRIV_LVL_M;
        epc_o          = mepc_q;
      end //csr_restore_mret_i
      default:;
    endcase
  end
end //PULP_SECURE
endgenerate

  assign hwlp_data_o = csr_wdata_int;

  // CSR operation wire
  always @(*)
  begin
    csr_wdata_int = csr_wdata_i;
    csr_we_int    = 1'b1;

     case (csr_op_i)
      CSR_OP_WRITE: csr_wdata_int = csr_wdata_i;
      CSR_OP_SET:   csr_wdata_int = csr_wdata_i | csr_rdata_o;
      CSR_OP_CLEAR: csr_wdata_int = (~csr_wdata_i) & csr_rdata_o;

      CSR_OP_NONE: begin
        csr_wdata_int = csr_wdata_i;
        csr_we_int    = 1'b0;
      end

      default:;
    endcase
  end


  // output mux
  always @(*)
  begin
    csr_rdata_o = csr_rdata_int;

    // performance counters
    if (is_pccr || is_pcer || is_pcmr)
      csr_rdata_o = perf_rdata;
  end


  // directly output some registers
  assign m_irq_enable_o  = mstatus_q[mie] & priv_lvl_q == PRIV_LVL_M;
  assign u_irq_enable_o  = mstatus_q[uie] & priv_lvl_q == PRIV_LVL_U;
  assign priv_lvl_o      = priv_lvl_q;
  assign sec_lvl_o       = priv_lvl_q[0];
  assign frm_o           = (FPU == 1) ? frm_q : 0;
  assign fprec_o         = (FPU == 1) ? fprec_q : 0;

  assign mtvec_o         = mtvec_q;
  assign utvec_o         = utvec_q;

  // actual registers
  always @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0)
    begin
      if (FPU == 1) begin
        frm_q          <= 0;
        fflags_q       <= 0;
        fprec_q        <= 0;
      end
      if (PULP_SECURE == 1) begin
        uepc_q         <= 0;
        ucause_q       <= 0;
        mtvec_reg_q    <= 0;
	utvec_q        <= 0;
      end
      priv_lvl_q     <= PRIV_LVL_M;
	  mstatus_q  	   <= {1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0};
      mstatus_q_mpp	<=  PRIV_LVL_M;
      mepc_q      <= 0;
      mcause_q    <= 0;
    end
    else
    begin
      // update CSRs
      if(FPU == 1) begin
        frm_q      <= frm_n;
        fflags_q   <= fflags_n;
        fprec_q    <= fprec_n;
      end
      if (PULP_SECURE == 1) begin
        mstatus_q      <= mstatus_n ;
		mstatus_q_mpp  <= mstatus_n_mpp ;
        uepc_q         <= uepc_n    ;
        ucause_q       <= ucause_n  ;
        priv_lvl_q     <= priv_lvl_n;
        utvec_q        <= utvec_n;
        mtvec_reg_q    <= mtvec_n;
      end else begin
        mstatus_q  	   <= {1'b0,1'b0,1'b0,mstatus_n[mie],1'b0,1'b0,1'b0,mstatus_n[mpie]};
        mstatus_q_mpp  <= PRIV_LVL_M;
        priv_lvl_q     <= PRIV_LVL_M;
      end
      mepc_q     <= mepc_n    ;
      mcause_q   <= mcause_n  ;
    end
  end

  assign mtvec_q = (PULP_SECURE) ? mtvec_reg_q : boot_addr_i;

  /////////////////////////////////////////////////////////////////
  //   ____            __     ____                  _            //
  // |  _ \ ___ _ __ / _|   / ___|___  _   _ _ __ | |_ ___ _ __  //
  // | |_) / _ \ '__| |_   | |   / _ \| | | | '_ \| __/ _ \ '__| //
  // |  __/  __/ |  |  _|  | |__| (_) | |_| | | | | ||  __/ |    //
  // |_|   \___|_|  |_|(_)  \____\___/ \__,_|_| |_|\__\___|_|    //
  //                                                             //
  /////////////////////////////////////////////////////////////////

  assign PCCR_in[0]  = 1'b1;                          // cycle counter
  assign PCCR_in[1]  = id_valid_i & is_decoding_i;    // instruction counter
  assign PCCR_in[2]  = ld_stall_i & id_valid_q;       // nr of load use hazards
  assign PCCR_in[3]  = jr_stall_i & id_valid_q;       // nr of jump register hazards
  assign PCCR_in[4]  = imiss_i & (~pc_set_i);         // cycles waiting for instruction fetches, excluding jumps and branches
  assign PCCR_in[5]  = mem_load_i;                    // nr of loads
  assign PCCR_in[6]  = mem_store_i;                   // nr of stores
  assign PCCR_in[7]  = jump_i                     & id_valid_q; // nr of jumps (unconditional)
  assign PCCR_in[8]  = branch_i                   & id_valid_q; // nr of branches (conditional)
  assign PCCR_in[9]  = branch_i & branch_taken_i  & id_valid_q; // nr of taken branches (conditional)
  assign PCCR_in[10] = id_valid_i & is_decoding_i & is_compressed_i;  // compressed instruction counter

  generate
  if (APU == 1) begin
     assign PCCR_in[PERF_APU_ID  ] = apu_typeconflict_i & ~apu_dep_i;
     assign PCCR_in[PERF_APU_ID+1] = apu_contention_i;
     assign PCCR_in[PERF_APU_ID+2] = apu_dep_i & ~apu_contention_i;
     assign PCCR_in[PERF_APU_ID+3] = apu_wb_i;
  end
  endgenerate
  // assign external performance counters
  generate
    genvar j;
    for(j = 0; j < N_EXT_CNT; j=j+1)
    begin : PCCR_gen
      assign PCCR_in[PERF_EXT_ID + j] = ext_counters_i[j];
    end
  endgenerate

  // address decoder for performance counter registers
  always @(*)
  begin
    is_pccr      = 1'b0;
    is_pcmr      = 1'b0;
    is_pcer      = 1'b0;
    pccr_all_sel = 1'b0;
    pccr_index   = 0;
    perf_rdata   = 0;

    // only perform csr access if we actually care about the read data
    if (csr_access_i) begin
       case (csr_addr_i)
        12'h7A0: begin
          is_pcer = 1'b1;
          perf_rdata[N_PERF_COUNTERS-1:0] = PCER_q;
        end
        12'h7A1: begin
          is_pcmr = 1'b1;
          perf_rdata[1:0] = PCMR_q;
        end
        12'h79F: begin
          is_pccr = 1'b1;
          pccr_all_sel = 1'b1;
        end
        default:;
      endcase

      // look for 780 to 79F, Performance Counter Counter Registers
      if (csr_addr_i[11:5] == 7'b0111100) begin
        is_pccr     = 1'b1;

        pccr_index = csr_addr_i[4:0];
`ifdef  ASIC_SYNTHESIS
        perf_rdata = PCCR_q[0];
`else
        perf_rdata = PCCR_q[csr_addr_i[4:0]];
`endif
      end
    end
  end

      integer i;
  // performance counter counter update wire
`ifdef ASIC_SYNTHESIS
  // for synthesis we just have one performance counter register
  always @(*) PCCR_inc[0] = (|(PCCR_in & PCER_q)) & PCMR_q[0];

  always @(*)
  begin
    PCCR_n[0]   = PCCR_q[0];

    if ((PCCR_inc_q[0] == 1'b1) && ((PCCR_q[0] != 32'hFFFFFFFF) || (PCMR_q[1] == 1'b0)))
      PCCR_n[0] = PCCR_q[0] + 1;

    if (is_pccr == 1'b1) begin
       case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCCR_n[0] = csr_wdata_i;
        CSR_OP_SET:    PCCR_n[0] = csr_wdata_i | PCCR_q[0];
        CSR_OP_CLEAR:  PCCR_n[0] = csr_wdata_i & ~(PCCR_q[0]);
      endcase
    end
  end
`else
  always @(*)
  begin
    for(i = 0; i < N_PERF_COUNTERS; i=i+1)
    begin : PERF_CNT_INC
      PCCR_inc[i] = PCCR_in[i] & PCER_q[i] & PCMR_q[0];

      PCCR_n[i]   = PCCR_q[i];

      if ((PCCR_inc_q[i] == 1'b1) && ((PCCR_q[i] != 32'hFFFFFFFF) || (PCMR_q[1] == 1'b0)))
        PCCR_n[i] = PCCR_q[i] + 1;

      if (is_pccr == 1'b1 && (pccr_all_sel == 1'b1 || pccr_index == i)) begin
         case (csr_op_i)
          CSR_OP_NONE:   ;
          CSR_OP_WRITE:  PCCR_n[i] = csr_wdata_i;
          CSR_OP_SET:    PCCR_n[i] = csr_wdata_i | PCCR_q[i];
          CSR_OP_CLEAR:  PCCR_n[i] = csr_wdata_i & ~(PCCR_q[i]);
        endcase
      end
    end
  end
`endif

  // update PCMR and PCER
  always @(*)
  begin
    PCMR_n = PCMR_q;
    PCER_n = PCER_q;

    if (is_pcmr) begin
       case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCMR_n = csr_wdata_i[1:0];
        CSR_OP_SET:    PCMR_n = csr_wdata_i[1:0] | PCMR_q;
        CSR_OP_CLEAR:  PCMR_n = csr_wdata_i[1:0] & ~(PCMR_q);
      endcase
    end

    if (is_pcer) begin
       case (csr_op_i)
        CSR_OP_NONE:   ;
        CSR_OP_WRITE:  PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0];
        CSR_OP_SET:    PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0] | PCER_q;
        CSR_OP_CLEAR:  PCER_n = csr_wdata_i[N_PERF_COUNTERS-1:0] & ~(PCER_q);
      endcase
    end
  end

  // Performance Counter Registers
  always @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0)
    begin
      id_valid_q <= 1'b0;

      PCER_q <= 0;
      PCMR_q <= 2'h3;

      for(i = 0; i < N_PERF_REGS; i=i+1)
      begin
        PCCR_q[i]     <= 0;
        PCCR_inc_q[i] <= 0;
      end
    end
    else
    begin
      id_valid_q <= id_valid_i;

      PCER_q <= PCER_n;
      PCMR_q <= PCMR_n;

      for(i = 0; i < N_PERF_REGS; i=i+1)
      begin
        PCCR_q[i]     <= PCCR_n[i];
        PCCR_inc_q[i] <= PCCR_inc[i];
      end

    end
  end

endmodule
