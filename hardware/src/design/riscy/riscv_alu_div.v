module riscv_alu_div
#(
   parameter C_WIDTH     = 32,
   parameter C_LOG_WIDTH = 6
)
(
    input  wire                    Clk_CI,
    input  wire                    Rst_RBI,
    // input IF
    input  wire [C_WIDTH-1:0]      OpA_DI,
    input  wire [C_WIDTH-1:0]      OpB_DI,
    input  wire [C_LOG_WIDTH-1:0]  OpBShift_DI,
    input  wire                    OpBIsZero_SI,
    //
    input  wire                    OpBSign_SI, // gate this to 0 in case of unsigned ops
    input  wire [1:0]              OpCode_SI,  // 0: udiv, 2: urem, 1: div, 3: rem
    // handshake
    input  wire                    InVld_SI,
    // output IF
    input  wire                    OutRdy_SI,
    output reg                     OutVld_SO,
    output wire [C_WIDTH-1:0]      Res_DO
  );

  ///////////////////////////////////////////////////////////////////////////////
  // signal declarations
  ///////////////////////////////////////////////////////////////////////////////

  wire  [C_WIDTH-1:0] ResReg_DN;
  wire   [C_WIDTH-1:0] ResReg_DP_rev;
  wire  [C_WIDTH-1:0] AReg_DN;
  wire  [C_WIDTH-1:0] BReg_DN;
  reg   [C_WIDTH-1:0] ResReg_DP;  
  reg   [C_WIDTH-1:0] AReg_DP;
  reg   [C_WIDTH-1:0] BReg_DP;

  wire  RemSel_SN;
  wire  CompInv_SN;
  wire  ResInv_SN;
  reg   RemSel_SP;
  reg   CompInv_SP;
  reg   ResInv_SP;
	
  
  
  wire [C_WIDTH-1:0] AddMux_D;
  wire [C_WIDTH-1:0] AddOut_D;
  wire [C_WIDTH-1:0] AddTmp_D;
  wire [C_WIDTH-1:0] BMux_D;
  wire [C_WIDTH-1:0] OutMux_D;

  wire [C_LOG_WIDTH-1:0] Cnt_DN;
  reg  [C_LOG_WIDTH-1:0] Cnt_DP;
  wire CntZero_S;

  wire  ABComp_S, PmSel_S; 
  reg   ARegEn_S, BRegEn_S, ResRegEn_S, LoadEn_S;

  localparam	IDLE = 0, DIVIDE = 1, FINISH = 2;
  reg  [1:0] 	State_SN, State_SP;


  ///////////////////////////////////////////////////////////////////////////////
  // datapath
  ///////////////////////////////////////////////////////////////////////////////

  assign PmSel_S     = LoadEn_S & ~(OpCode_SI[0] & (OpA_DI[C_WIDTH-1] ^ OpBSign_SI));

  // muxes
  assign AddMux_D    = (LoadEn_S) ? OpA_DI  : BReg_DP;

  // attention: wireal shift in case of negative operand B!
  assign BMux_D      = (LoadEn_S) ? OpB_DI : {CompInv_SP, (BReg_DP[C_WIDTH-1:1])};

  //assign ResReg_DP_rev = {<<{ResReg_DP}}; NOT SUPPORTED
  generate
  genvar i;
  for (i = 0; i < C_WIDTH; i=i+1) begin : Stream_op_gen
      assign ResReg_DP_rev[i] = ResReg_DP[C_WIDTH - 1 - i];
  end
  endgenerate
  
  assign OutMux_D    = (RemSel_SP) ? AReg_DP : ResReg_DP_rev;

  // invert if necessary
  assign Res_DO      = (ResInv_SP) ? -$signed(OutMux_D) : OutMux_D;

  // main comparator
  assign ABComp_S    = ((AReg_DP == BReg_DP) | ((AReg_DP > BReg_DP) ^ CompInv_SP)) & ((|AReg_DP) | OpBIsZero_SI);

  // main adder
  assign AddTmp_D    = (LoadEn_S) ? 0 : AReg_DP;
  assign AddOut_D    = (PmSel_S)  ? AddTmp_D + AddMux_D : AddTmp_D - $signed(AddMux_D);

  ///////////////////////////////////////////////////////////////////////////////
  // counter
  ///////////////////////////////////////////////////////////////////////////////

  assign Cnt_DN      = (LoadEn_S)   ? OpBShift_DI :
                       (~CntZero_S) ? Cnt_DP - 1  : Cnt_DP;

  assign CntZero_S   = ~(|Cnt_DP);

  ///////////////////////////////////////////////////////////////////////////////
  // FSM
  ///////////////////////////////////////////////////////////////////////////////

  always @(*)
  begin : p_fsm
    // default
    State_SN       = State_SP;

    OutVld_SO      = 1'b0;

    LoadEn_S       = 1'b0;

    ARegEn_S       = 1'b0;
    BRegEn_S       = 1'b0;
    ResRegEn_S     = 1'b0;

    case (State_SP)
      /////////////////////////////////
      IDLE: begin
        OutVld_SO    = 1'b1;

        if(InVld_SI) begin
          OutVld_SO  = 1'b0;
          ARegEn_S   = 1'b1;
          BRegEn_S   = 1'b1;
          LoadEn_S   = 1'b1;
          State_SN   = DIVIDE;
        end
      end
      /////////////////////////////////
      DIVIDE: begin

        ARegEn_S     = ABComp_S;
        BRegEn_S     = 1'b1;
        ResRegEn_S   = 1'b1;

        // calculation finished
        // one more divide cycle (32nd divide cycle)
        if (CntZero_S) begin
          State_SN   = FINISH;
        end
      end
      /////////////////////////////////
      FINISH: begin
        OutVld_SO = 1'b1;

        if(OutRdy_SI) begin
          State_SN  = IDLE;
        end
      end
      /////////////////////////////////
      //default : /* default */ ;
      /////////////////////////////////
    endcase
  end


  ///////////////////////////////////////////////////////////////////////////////
  // regs
  ///////////////////////////////////////////////////////////////////////////////

  // get flags
  assign RemSel_SN  = (LoadEn_S) ? OpCode_SI[1] : RemSel_SP;
  assign CompInv_SN = (LoadEn_S) ? OpBSign_SI   : CompInv_SP;
  assign ResInv_SN  = (LoadEn_S) ? (~OpBIsZero_SI | OpCode_SI[1]) & OpCode_SI[0] & (OpA_DI[C_WIDTH-1] ^ OpBSign_SI) : ResInv_SP;

  assign AReg_DN   = (ARegEn_S)   ? AddOut_D : AReg_DP;
  assign BReg_DN   = (BRegEn_S)   ? BMux_D   : BReg_DP;
  assign ResReg_DN = (LoadEn_S)   ? 0       :
                     (ResRegEn_S) ? {ABComp_S, ResReg_DP[C_WIDTH-1:1]} : ResReg_DP;

  always @(posedge Clk_CI or negedge Rst_RBI) begin : p_regs
    if(~Rst_RBI) begin
       State_SP   <= IDLE;
       AReg_DP    <= 0;
       BReg_DP    <= 0;
       ResReg_DP  <= 0;
       Cnt_DP     <= 0;
       RemSel_SP  <= 1'b0;
       CompInv_SP <= 1'b0;
       ResInv_SP  <= 1'b0;
    end else begin
       State_SP   <= State_SN;
       AReg_DP    <= AReg_DN;
       BReg_DP    <= BReg_DN;
       ResReg_DP  <= ResReg_DN;
       Cnt_DP     <= Cnt_DN;
       RemSel_SP  <= RemSel_SN;
       CompInv_SP <= CompInv_SN;
       ResInv_SP  <= ResInv_SN;
    end
  end

  ///////////////////////////////////////////////////////////////////////////////
  // assertions
  ///////////////////////////////////////////////////////////////////////////////

//`ifndef SYNTHESIS
//  initial
//  begin : p_assertions
//    assert (C_LOG_WIDTH == $clog2(C_WIDTH+1)) else $error("C_LOG_WIDTH must be $clog2(C_WIDTH+1)");
//  end
//`endif

endmodule // serDiv
