
module riscv_mult
#(
`include "riscv_defines.v"
  parameter SHARED_DSP_MULT = 1
  )
(
  input  wire        clk,
  input  wire        rst_n,

  input  wire        enable_i,
  input  wire [ 2:0] operator_i,

  // integer and short multiplier
  input  wire        short_subword_i,
  input  wire [ 1:0] short_signed_i,

  input  wire [31:0] op_a_i,
  input  wire [31:0] op_b_i,
  input  wire [31:0] op_c_i,

  input  wire [ 4:0] imm_i,


  // dot multiplier
  input  wire [ 1:0] dot_signed_i,
  input  wire [31:0] dot_op_a_i,
  input  wire [31:0] dot_op_b_i,
  input  wire [31:0] dot_op_c_i,

  output reg  [31:0] result_o,

  output reg         multicycle_o,
  output wire        ready_o,
  input  wire        ex_ready_i
);

  ///////////////////////////////////////////////////////////////
  //  ___ _  _ _____ ___ ___ ___ ___   __  __ _   _ _  _____   //
  // |_ _| \| |_   _| __/ __| __| _ \ |  \/  | | | | ||_   _|  //
  //  | || .  | | | | _| (_ | _||   / | |\/| | |_| | |__| |    //
  // |___|_|\_| |_| |___\___|___|_|_\ |_|  |_|\___/|____|_|    //
  //                                                           //
  ///////////////////////////////////////////////////////////////

  wire [16:0] short_op_a;
  wire [16:0] short_op_b;
  wire [32:0] short_op_c;
  wire [33:0] short_mul;
  wire [33:0] short_mac;
  wire [31:0] short_round, short_round_tmp;
  wire [33:0] short_result;

  wire        short_mac_msb1;
  wire        short_mac_msb0;

  wire [ 4:0] short_imm;
  wire [ 1:0] short_subword;
  wire [ 1:0] short_signed;
  wire        short_shift_arith;
  reg  [ 4:0] mulh_imm;
  reg  [ 1:0] mulh_subword;
  reg  [ 1:0] mulh_signed;
  reg         mulh_shift_arith;
  reg         mulh_carry_q;
  reg         mulh_active;
  reg         mulh_save;
  reg         mulh_clearcarry;
  reg         mulh_ready;

  localparam IDLE=0, STEP0=1, STEP1=2, STEP2=3, FINISH=4;
  reg [2:0]  mulh_CS, mulh_NS;

  // prepare the rounding value
  assign short_round_tmp = (32'h00000001) << imm_i;
  assign short_round = (operator_i == MUL_IR) ? {1'b0, short_round_tmp[31:1]} : 0;

  // perform subword selection and sign extensions
  assign short_op_a[15:0] = short_subword[0] ? op_a_i[31:16] : op_a_i[15:0];
  assign short_op_b[15:0] = short_subword[1] ? op_b_i[31:16] : op_b_i[15:0];

  assign short_op_a[16]   = short_signed[0] & short_op_a[15];
  assign short_op_b[16]   = short_signed[1] & short_op_b[15];

  assign short_op_c       = mulh_active ? $signed({mulh_carry_q, op_c_i}) : $signed(op_c_i);

  assign short_mul        = $signed(short_op_a) * $signed(short_op_b);
  assign short_mac        = $signed(short_op_c) + $signed(short_mul) + $signed(short_round);

   //we use only short_signed_i[0] as it cannot be short_signed_i[1] 1 and short_signed_i[0] 0
  assign short_result     = $signed({short_shift_arith & short_mac_msb1, short_shift_arith & short_mac_msb0, short_mac[31:0]}) >>> short_imm;

  // choose between normal short multiplication operation and mulh operation
  assign short_imm         = mulh_active ? mulh_imm         : imm_i;
  assign short_subword     = mulh_active ? mulh_subword     : {2{short_subword_i}};
  assign short_signed      = mulh_active ? mulh_signed      : short_signed_i;
  assign short_shift_arith = mulh_active ? mulh_shift_arith : short_signed_i[0];

  assign short_mac_msb1    = mulh_active ? short_mac[33] : short_mac[31];
  assign short_mac_msb0    = mulh_active ? short_mac[32] : short_mac[31];


  always @(*)
  begin
    mulh_NS          = mulh_CS;
    mulh_imm         = 5'd0;
    mulh_subword     = 2'b00;
    mulh_signed      = 2'b00;
    mulh_shift_arith = 1'b0;
    mulh_ready       = 1'b0;
    mulh_active      = 1'b1;
    mulh_save        = 1'b0;
    mulh_clearcarry  = 1'b0;
    multicycle_o     = 1'b0;

    case (mulh_CS)
      IDLE: begin
        mulh_active = 1'b0;
        mulh_ready  = 1'b1;
        mulh_save   = 1'b0;
        if ((operator_i == MUL_H) && enable_i) begin
          mulh_ready  = 1'b0;
          mulh_NS     = STEP0;
        end
      end

      STEP0: begin
        multicycle_o = 1'b1;
        mulh_imm         = 5'd16;
        mulh_active      = 1'b1;
        //AL*BL never overflows
        mulh_save        = 1'b0;
        mulh_NS          = STEP1;
        //Here always a 32'b unsigned result (no carry)
      end

      STEP1: begin
        multicycle_o = 1'b1;
        //AL*BH is signed iff B is signed
        mulh_signed      = {short_signed_i[1], 1'b0};
        mulh_subword     = 2'b10;
        mulh_save        = 1'b1;
        mulh_shift_arith = 1'b1;
        mulh_NS          = STEP2;
        //Here signed 32'b + unsigned 32'b result.
        //Result is a signed 33'b
        //Store the carry as it will be used as sign extension, we do
        //not shift
      end

      STEP2: begin
        multicycle_o = 1'b1;
        //AH*BL is signed iff A is signed
        mulh_signed      = {1'b0, short_signed_i[0]};
        mulh_subword     = 2'b01;
        mulh_imm         = 5'd16;
        mulh_save        = 1'b1;
        mulh_clearcarry  = 1'b1;
        mulh_shift_arith = 1'b1;
        mulh_NS          = FINISH;
        //Here signed 32'b + signed 33'b result.
        //Result is a signed 34'b
        //We do not store the carries as the bits 34:33 are shifted back, so we clear it
      end

      FINISH: begin
        mulh_signed  = short_signed_i;
        mulh_subword = 2'b11;
        mulh_ready   = 1'b1;
        if (ex_ready_i)
          mulh_NS = IDLE;
      end
    endcase
  end

  always @(posedge clk, negedge rst_n)
  begin
    if (~rst_n)
    begin
      mulh_CS      <= IDLE;
      mulh_carry_q <= 1'b0;
    end else begin
      mulh_CS      <= mulh_NS;

      if (mulh_save)
        mulh_carry_q <= ~mulh_clearcarry & short_mac[32];
      else if (ex_ready_i) // clear carry when we are going to the next instruction
        mulh_carry_q <= 1'b0;
    end
  end

  // 32x32 = 32-bit multiplier
  wire [31:0] int_op_a_msu;
  wire [31:0] int_op_b_msu;
  wire [31:0] int_result;

  wire        int_is_msu;

  assign int_is_msu = (operator_i == MUL_MSU32); // TODO: think about using a separate signal here, could prevent some switching

  assign int_op_a_msu = op_a_i ^ {32{int_is_msu}};
  assign int_op_b_msu = op_b_i & {32{int_is_msu}};

  assign int_result = $signed(op_c_i) + $signed(int_op_b_msu) + $signed(int_op_a_msu) * $signed(op_b_i);

  ///////////////////////////////////////////////
  //  ___   ___ _____   __  __ _   _ _  _____  //
  // |   \ / _ \_   _| |  \/  | | | | ||_   _| //
  // | |) | (_) || |   | |\/| | |_| | |__| |   //
  // |___/ \___/ |_|   |_|  |_|\___/|____|_|   //
  //                                           //
  ///////////////////////////////////////////////

  wire [31:0] dot_char_result;
  wire [31:0] dot_short_result;

   generate
     if (SHARED_DSP_MULT == 0) begin

        wire [ 8:0] dot_char_op_a [0:3];
        wire [ 8:0] dot_char_op_b [0:3];
        wire [17:0] dot_char_mul [0:3];

        wire [16:0] dot_short_op_a [0:1];
        wire [16:0] dot_short_op_b [0:1];
        wire [33:0] dot_short_mul [0:1];

        assign dot_char_op_a[0] = {dot_signed_i[1] & dot_op_a_i[ 7], dot_op_a_i[ 7: 0]};
        assign dot_char_op_a[1] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15: 8]};
        assign dot_char_op_a[2] = {dot_signed_i[1] & dot_op_a_i[23], dot_op_a_i[23:16]};
        assign dot_char_op_a[3] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:24]};

        assign dot_char_op_b[0] = {dot_signed_i[0] & dot_op_b_i[ 7], dot_op_b_i[ 7: 0]};
        assign dot_char_op_b[1] = {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15: 8]};
        assign dot_char_op_b[2] = {dot_signed_i[0] & dot_op_b_i[23], dot_op_b_i[23:16]};
        assign dot_char_op_b[3] = {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:24]};

        assign dot_char_mul[0]  = $signed(dot_char_op_a[0]) * $signed(dot_char_op_b[0]);
        assign dot_char_mul[1]  = $signed(dot_char_op_a[1]) * $signed(dot_char_op_b[1]);
        assign dot_char_mul[2]  = $signed(dot_char_op_a[2]) * $signed(dot_char_op_b[2]);
        assign dot_char_mul[3]  = $signed(dot_char_op_a[3]) * $signed(dot_char_op_b[3]);

        assign dot_char_result  = $signed(dot_char_mul[0]) + $signed(dot_char_mul[1]) +
                                  $signed(dot_char_mul[2]) + $signed(dot_char_mul[3]) +
                                  $signed(dot_op_c_i);


        assign dot_short_op_a[0] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15: 0]};
        assign dot_short_op_a[1] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:16]};

        assign dot_short_op_b[0] = {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15: 0]};
        assign dot_short_op_b[1] = {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:16]};

        assign dot_short_mul[0]  = $signed(dot_short_op_a[0]) * $signed(dot_short_op_b[0]);
        assign dot_short_mul[1]  = $signed(dot_short_op_a[1]) * $signed(dot_short_op_b[1]);

        assign dot_short_result  = $signed(dot_short_mul[0][31:0]) + $signed(dot_short_mul[1][31:0]) + $signed(dot_op_c_i);

     end else begin
        assign dot_char_result  = 0;
        assign dot_short_result = 0;
     end
  endgenerate

  ////////////////////////////////////////////////////////
  //   ____                 _ _     __  __              //
  //  |  _ \ ___  ___ _   _| | |_  |  \/  |_   ___  __  //
  //  | |_) / _ \/ __| | | | | __| | |\/| | | | \ \/ /  //
  //  |  _ <  __/\__ \ |_| | | |_  | |  | | |_| |>  <   //
  //  |_| \_\___||___/\__,_|_|\__| |_|  |_|\__,_/_/\_\  //
  //                                                    //
  ////////////////////////////////////////////////////////

  always @(*)
  begin
    result_o   = 0;

    case (operator_i)
      MUL_MAC32, MUL_MSU32: result_o = int_result[31:0];

      MUL_I, MUL_IR, MUL_H: result_o = short_result[31:0];

      MUL_DOT8:  result_o = dot_char_result[31:0];
      MUL_DOT16: result_o = dot_short_result[31:0];

      default: ; // default case to suppress unique warning
    endcase
  end

  assign ready_o      = mulh_ready;

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // check multiplication result for mulh
  //`ifndef VERILATOR
  //assert property (
  //  @(posedge clk) ((mulh_CS == FINISH) && (operator_i == MUL_H) && (short_signed_i == 2'b11))
  //  |->
  //  (result_o == (($signed({{32{op_a_i[31]}}, op_a_i}) * $signed({{32{op_b_i[31]}}, op_b_i})) >>> 32) ) );

  // check multiplication result for mulhsu
  //assert property (
  //  @(posedge clk) ((mulh_CS == FINISH) && (operator_i == MUL_H) && (short_signed_i == 2'b01))
  //  |->
  //  (result_o == (($signed({{32{op_a_i[31]}}, op_a_i}) * {32'b0, op_b_i}) >> 32) ) );

  // check multiplication result for mulhu
  //assert property (
  //  @(posedge clk) ((mulh_CS == FINISH) && (operator_i == MUL_H) && (short_signed_i == 2'b00))
  //  |->
  //  (result_o == (({32'b0, op_a_i} * {32'b0, op_b_i}) >> 32) ) );
  //`endif
endmodule
