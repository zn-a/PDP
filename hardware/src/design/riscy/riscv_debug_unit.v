
module riscv_debug_unit
#(
	`include "riscv_defines.v"
	parameter DUMMY = 0
)
(
  input wire         clk,
  input wire         rst_n,

  // Debug Interface
  input  wire        debug_req_i,
  output reg         debug_gnt_o,
  output reg         debug_rvalid_o,
  input  wire [14:0] debug_addr_i,
  input  wire        debug_we_i,
  input  wire [31:0] debug_wdata_i,
  output reg  [31:0] debug_rdata_o,

  output reg         debug_halted_o,
  input  wire        debug_halt_i,
  input  wire        debug_resume_i,

  // signals to core
  output wire  [DBG_SETS_W-1:0] settings_o,

  input  wire        trap_i,      // trap found, need to stop the core now
  input  wire [5:0]  exc_cause_i, // if it was a trap, then the exception controller knows more
  output reg         stall_o,     // after we got control, we control the stall signal
  output reg         dbg_req_o,
  input  wire        dbg_ack_i,

  // register file read port
  output wire         regfile_rreq_o,
  output wire  [ 5:0] regfile_raddr_o,
  input  wire [31:0] regfile_rdata_i,

  // register file write port
  output wire        regfile_wreq_o,
  output wire [ 5:0] regfile_waddr_o,
  output wire [31:0] regfile_wdata_o,

  // CSR read/write port
  output wire        csr_req_o,
  output wire [11:0] csr_addr_o,
  output reg        csr_we_o,
  output wire [31:0] csr_wdata_o,
  input  wire [31:0] csr_rdata_i,

  // Signals for PPC & NPC register
  input  wire [31:0] pc_if_i,
  input  wire [31:0] pc_id_i,
  input  wire [31:0] pc_ex_i,

  input  wire        data_load_event_i,
  input  wire        instr_valid_id_i,

  input  wire        sleeping_i,

  input  wire        branch_in_ex_i,
  input  wire        branch_taken_i,

  output wire        jump_req_o,
  output wire [31:0] jump_addr_o
);

  localparam RD_NONE=0, RD_CSR=1, RD_GPR=2, RD_DBGA=3, RD_DBGS=4;
  reg	 [2:0] rdata_sel_q, rdata_sel_n;

  localparam FIRST=0, SECOND=1;
  reg	 state_q, state_n;

  reg [DBG_SETS_W-1:0] settings_q, settings_n;

  // for timing critical register file access we need to keep those in FFs
  reg [14:0] addr_q;
  reg [31:0] wdata_q; // mainly for jumps
  reg        regfile_rreq_q, regfile_rreq_n;
  reg        regfile_fp_sel_q, regfile_fp_sel_n;
  reg        jump_req_q, jump_req_n;

  // not timing critical
  reg        csr_req_q, csr_req_n;
  reg        regfile_wreq;
  reg        regfile_fp_wr;
  
  localparam  RUNNING=0, HALT_REQ=1, HALT=2;
  reg [1:0]  stall_cs, stall_ns;
  reg [31:0] dbg_rdata;
  reg        dbg_resume;
  reg        dbg_halt;
  reg [5:0]  dbg_cause_q, dbg_cause_n;
  reg        dbg_ssth_q,  dbg_ssth_n;

  reg        ssth_clear;


  // ppc/npc tracking
  localparam IFID=0, IFEX=1, IDEX=2;
  reg [1:0] pc_tracking_fsm_cs, pc_tracking_fsm_ns;
  reg [31:0] ppc_int, npc_int;


  // address decoding, write and read controller
  always @(*)
  begin
    rdata_sel_n    = RD_NONE;
    state_n        = FIRST;

    debug_gnt_o    = 1'b0;

    regfile_rreq_n = 1'b0;
    regfile_wreq   = 1'b0;
    csr_req_n      = 1'b0;
    csr_we_o       = 1'b0;
    jump_req_n     = 1'b0;

    dbg_resume     = 1'b0;
    dbg_halt       = 1'b0;
    settings_n     = settings_q;

    ssth_clear     = 1'b0;

    regfile_fp_sel_n = 1'b0;
    regfile_fp_wr    = 1'b0;
    
    if (debug_req_i) begin
      if (debug_we_i) begin
        //----------------------------------------------------------------------------
        // write access
        //----------------------------------------------------------------------------
        if (debug_addr_i[14]) begin
          // CSR access
          if (state_q == FIRST) begin
            // only grant in second cycle, address and data have been latched by then
            debug_gnt_o = 1'b0;
            state_n     = SECOND;

            if (debug_halted_o) begin
              // access to internal registers are only allowed when the core is halted
              csr_req_n = 1'b1;
            end
          end else begin
            debug_gnt_o = 1'b1; // grant it even when invalid access to not block
            state_n     = FIRST;
            csr_we_o    = 1'b1;
          end
        end else begin
          // non-CSR access
           case (debug_addr_i[13:8])
            6'b00_0000: begin // Debug Registers, always accessible
              debug_gnt_o = 1'b1;

               case (debug_addr_i[6:2])
                5'b0_0000: begin // DBG_CTRL
                  if (debug_wdata_i[16]) begin
                    // HALT set
                    if (~debug_halted_o) begin
                      // not halt, so STOP
                      dbg_halt = 1'b1;
                    end
                  end else begin
                    // RESUME set
                    if (debug_halted_o) begin
                      dbg_resume = 1'b1;
                    end
                  end

                  settings_n[DBG_SETS_SSTE] = debug_wdata_i[0];
                end
                5'b0_0001: begin // DBG_HIT
                  ssth_clear = debug_wdata_i[0];
                end
                5'b0_0010: begin // DBG_IE
                  settings_n[DBG_SETS_ECALL] = debug_wdata_i[11];
                  settings_n[DBG_SETS_ELSU]  = debug_wdata_i[7] | debug_wdata_i[5];
                  settings_n[DBG_SETS_EBRK]  = debug_wdata_i[3];
                  settings_n[DBG_SETS_EILL]  = debug_wdata_i[2];
                end
                default:;
              endcase
            end

            6'b10_0000: begin // Debug Registers, only accessible when in debug
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              if (debug_halted_o) begin
                 case (debug_addr_i[6:2])
                  5'b0_0000: jump_req_n = 1'b1; // DNPC
                  default:;
                endcase
              end
            end

            6'b00_0100: begin // General-Purpose Registers
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              if (debug_halted_o) begin
                regfile_wreq = 1'b1;
              end
            end

            6'b00_0101: begin // General-Purpose Floating-Point Registers
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              if (debug_halted_o) begin
                regfile_wreq  = 1'b1;
                regfile_fp_wr = 1'b1;
              end
            end

            default: debug_gnt_o = 1'b1; // grant it even when invalid access to not block
          endcase
        end
      end else begin
        //----------------------------------------------------------------------------
        // read access
        //----------------------------------------------------------------------------
        if (debug_addr_i[14]) begin
          debug_gnt_o = 1'b1; // grant it even when invalid access to not block

          // CSR access
          if (debug_halted_o) begin
            // access to internal registers are only allowed when the core is halted
            csr_req_n   = 1'b1;
            rdata_sel_n = RD_CSR;
          end
        end else begin
          // non-CSR access
           case (debug_addr_i[13:8])
            6'b00_0000: begin // Debug Registers, always accessible
              debug_gnt_o = 1'b1;

              rdata_sel_n = RD_DBGA;
            end

            6'b10_0000: begin // Debug Registers, only accessible when in debug
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              rdata_sel_n = RD_DBGS;
            end

            6'b00_0100: begin // General-Purpose Registers
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              if (debug_halted_o) begin
                regfile_rreq_n = 1'b1;
                rdata_sel_n    = RD_GPR;
              end
            end
        
            6'b00_0101: begin // General-Purpose Floating-Point Registers
              debug_gnt_o = 1'b1; // grant it even when invalid access to not block

              if (debug_halted_o) begin
                regfile_rreq_n   = 1'b1;
                regfile_fp_sel_n = 1'b1;
                rdata_sel_n      = RD_GPR;
              end
            end
            default: debug_gnt_o = 1'b1; // grant it even when invalid access to not block
          endcase
        end
      end
    end
  end

  //----------------------------------------------------------------------------
  // debug register read access
  //
  // Since those are combinational, we can do it in the cycle where we set
  // rvalid. The address has been latched into addr_q
  //----------------------------------------------------------------------------
  always @(*)
  begin
    dbg_rdata = 0;

    case (rdata_sel_q)
      RD_DBGA: begin
         case (addr_q[6:2])
          5'h00: dbg_rdata[31:0] = {15'b0, debug_halted_o, 15'b0, settings_q[DBG_SETS_SSTE]}; // DBG_CTRL
          5'h01: dbg_rdata[31:0] = {15'b0, sleeping_i, 15'b0, dbg_ssth_q}; // DBG_HIT
          5'h02: begin // DBG_IE
            dbg_rdata[31:16] = 0;
            dbg_rdata[15:12] = 0;
            dbg_rdata[11]    = settings_q[DBG_SETS_ECALL];
            dbg_rdata[10: 8] = 0;
            dbg_rdata[ 7]    = settings_q[DBG_SETS_ELSU];
            dbg_rdata[ 6]    = 1'b0;
            dbg_rdata[ 5]    = settings_q[DBG_SETS_ELSU];
            dbg_rdata[ 4]    = 1'b0;
            dbg_rdata[ 3]    = settings_q[DBG_SETS_EBRK];
            dbg_rdata[ 2]    = settings_q[DBG_SETS_EILL];
            dbg_rdata[ 1: 0] = 0;
          end
          5'h03: dbg_rdata = {dbg_cause_q[5], 26'b0, dbg_cause_q[4:0]}; // DBG_CAUSE
          5'h10: dbg_rdata = 0; // DBG_BPCTRL0
          5'h12: dbg_rdata = 0; // DBG_BPCTRL1
          5'h14: dbg_rdata = 0; // DBG_BPCTRL2
          5'h16: dbg_rdata = 0; // DBG_BPCTRL3
          5'h18: dbg_rdata = 0; // DBG_BPCTRL4
          5'h1A: dbg_rdata = 0; // DBG_BPCTRL5
          5'h1C: dbg_rdata = 0; // DBG_BPCTRL6
          5'h1E: dbg_rdata = 0; // DBG_BPCTRL7
          default:;
        endcase
      end

      RD_DBGS: begin
         case (addr_q[2:2])
          1'b0: dbg_rdata = npc_int; // DBG_NPC
          1'b1: dbg_rdata = ppc_int; // DBG_PPC
          default:;
        endcase
      end

      default:;
    endcase
  end

  //----------------------------------------------------------------------------
  // read data mux
  //----------------------------------------------------------------------------
  always @(*)
  begin
    debug_rdata_o = 0;

    case (rdata_sel_q)
      RD_CSR:  debug_rdata_o = csr_rdata_i;
      RD_GPR:  debug_rdata_o = regfile_rdata_i;
      RD_DBGA: debug_rdata_o = dbg_rdata;
      RD_DBGS: debug_rdata_o = dbg_rdata;
    endcase
  end

  //----------------------------------------------------------------------------
  // rvalid generation
  //----------------------------------------------------------------------------
  always @(posedge clk, negedge rst_n)
  begin
    if (~rst_n) begin
      debug_rvalid_o <= 1'b0;
    end else begin
      debug_rvalid_o <= debug_gnt_o; // always give the rvalid one cycle after gnt
    end
  end

  //----------------------------------------------------------------------------
  // stall control
  //----------------------------------------------------------------------------
  always @(*)
  begin
    stall_ns       = stall_cs;
    dbg_req_o      = 1'b0;
    stall_o        = 1'b0;
    debug_halted_o = 1'b0;
    dbg_cause_n    = dbg_cause_q;
    dbg_ssth_n     = dbg_ssth_q;

    case (stall_cs)
      RUNNING: begin
        dbg_ssth_n = 1'b0;

        if (dbg_halt | debug_halt_i | trap_i) begin
          dbg_req_o   = 1'b1;
          stall_ns    = HALT_REQ;

          if (trap_i) begin
            if (settings_q[DBG_SETS_SSTE])
              dbg_ssth_n = 1'b1;

            dbg_cause_n = exc_cause_i;
          end else begin
            dbg_cause_n = DBG_CAUSE_HALT;
          end
        end
      end

      HALT_REQ: begin
        dbg_req_o = 1'b1;

        if (dbg_ack_i)
          stall_ns = HALT;

        if (dbg_resume | debug_resume_i)
          stall_ns = RUNNING;
      end

      HALT: begin
        stall_o        = 1'b1;
        debug_halted_o = 1'b1;

        if (dbg_resume | debug_resume_i) begin
          stall_ns = RUNNING;
          stall_o  = 1'b0;
        end
      end
    endcase

    if (ssth_clear)
      dbg_ssth_n = 1'b0;
  end

  always @(posedge clk, negedge rst_n)
  begin
    if (~rst_n) begin
      stall_cs    <= RUNNING;
      dbg_cause_q <= DBG_CAUSE_HALT;
      dbg_ssth_q  <= 1'b0;
    end else begin
      stall_cs    <= stall_ns;
      dbg_cause_q <= dbg_cause_n;
      dbg_ssth_q  <= dbg_ssth_n;
    end
  end

  //----------------------------------------------------------------------------
  // NPC/PPC selection
  //----------------------------------------------------------------------------
  always @(*)
  begin
    pc_tracking_fsm_ns = pc_tracking_fsm_cs;

    ppc_int = pc_id_i;
    npc_int = pc_if_i;

    // PPC/NPC mux
     case (pc_tracking_fsm_cs)
      IFID: begin
        ppc_int = pc_id_i;
        npc_int = pc_if_i;
      end

      IFEX: begin
        ppc_int = pc_ex_i;
        npc_int = pc_if_i;
      end

      IDEX: begin
        ppc_int = pc_ex_i;
        npc_int = pc_id_i;

        if (jump_req_o)
          pc_tracking_fsm_ns = IFEX;
      end

      default: begin
        pc_tracking_fsm_ns = IFID;
      end
    endcase

    // set state if trap is encountered
    if (dbg_ack_i) begin
      pc_tracking_fsm_ns = IFID;

      if (branch_in_ex_i) begin
        if (branch_taken_i)
          pc_tracking_fsm_ns = IFEX;
        else
          pc_tracking_fsm_ns = IDEX;
      end else if (data_load_event_i) begin
        // for p.elw
        if (instr_valid_id_i)
           pc_tracking_fsm_ns = IDEX;
        else
           pc_tracking_fsm_ns = IFEX;
      end
    end
  end


  always @(posedge clk, negedge rst_n)
  begin
    if (~rst_n) begin
      pc_tracking_fsm_cs <= IFID;

      addr_q             <= 0;
      wdata_q            <= 0;
      state_q            <= FIRST;
      rdata_sel_q        <= RD_NONE;
      regfile_rreq_q     <= 1'b0;
      regfile_fp_sel_q   <= 1'b0;
      csr_req_q          <= 1'b0;
      jump_req_q         <= 1'b0;

      settings_q         <= 1'b0;
    end else begin
      pc_tracking_fsm_cs <= pc_tracking_fsm_ns;

      // settings
      settings_q         <= settings_n;

      // for the actual interface
      if (debug_req_i) begin
        addr_q  <= debug_addr_i;
        wdata_q <= debug_wdata_i;
        state_q <= state_n;
      end

      if (debug_req_i | debug_rvalid_o) begin
        // wait for either req or rvalid to set those FFs
        // This makes sure that they are only triggered once when there is
        // only one request, and the FFs can be properly clock gated otherwise
        regfile_rreq_q   <= regfile_rreq_n;
        regfile_fp_sel_q <= regfile_fp_sel_n;
        csr_req_q        <= csr_req_n;
        jump_req_q       <= jump_req_n;
        rdata_sel_q      <= rdata_sel_n;
      end
    end
  end

  assign regfile_rreq_o  = regfile_rreq_q;
  assign regfile_raddr_o = {regfile_fp_sel_q, addr_q[6:2]};

  
  assign regfile_wreq_o  = regfile_wreq;
  assign regfile_waddr_o = {regfile_fp_wr,debug_addr_i[6:2]};
  assign regfile_wdata_o = debug_wdata_i;

  assign csr_req_o       = csr_req_q;
  assign csr_addr_o      = addr_q[13:2];
  assign csr_wdata_o     = wdata_q;

  assign jump_req_o      = jump_req_q;
  assign jump_addr_o     = wdata_q;

  assign settings_o      = settings_q;

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------
  //  `ifndef VERILATOR
    // check that no registers are accessed when we are not in debug mode
   //   assert property (
     //   @(posedge clk) (debug_req_i) |-> ((debug_halted_o == 1'b1) ||
   //                                       ((debug_addr_i[14] != 1'b1) &&
  //                                         (debug_addr_i[13:7] != 5'b0_1001)  &&
  //  //                                         (debug_addr_i[13:7] != 5'b0_1000)) ) )
  //      else $warning("Trying to access internal debug registers while core is not stalled");
  //
   //   // check that all accesses are word-aligned
   //  //   assert property (
   //     @(posedge clk) (debug_req_i) |-> (debug_addr_i[1:0] == 2'b00) );
  //  `endif
endmodule // debug_unit
