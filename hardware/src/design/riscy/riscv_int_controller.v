
module riscv_int_controller
#(
  `include "riscv_defines.v"
  parameter PULP_SECURE = 0
)
(
  input  wire        clk,
  input  wire        rst_n,

  // irq_req for controller
  output wire        irq_req_ctrl_o,
  output wire        irq_sec_ctrl_o,
  output wire  [4:0] irq_id_ctrl_o,

  // handshake signals to controller
  input  wire        ctrl_ack_i,
  input  wire        ctrl_kill_i,

  // external interrupt lines
  input  wire        irq_i,          // level-triggered interrupt inputs
  input  wire        irq_sec_i,      // interrupt secure bit from EU
  input  wire  [4:0] irq_id_i,       // interrupt id [0,1,....31]

  input  wire        m_IE_i,         // interrupt enable bit from CSR (M mode)
  input  wire        u_IE_i,         // interrupt enable bit from CSR (U mode)
  input  [PRIVLVL_T_SIZE-1:0]    current_priv_lvl_i

);

  localparam IDLE=0, IRQ_PENDING=1, IRQ_DONE=2; 
  reg [1:0] exc_ctrl_cs, exc_ctrl_ns;

  wire irq_enable_ext;
  reg [4:0] irq_id_q;
  reg irq_sec_q;

  generate
		if(PULP_SECURE)
		  assign irq_enable_ext =  ((u_IE_i | irq_sec_i) & current_priv_lvl_i == PRIV_LVL_U) | (m_IE_i & current_priv_lvl_i == PRIV_LVL_M);
		else
		  assign irq_enable_ext =  m_IE_i;
  endgenerate

  assign irq_req_ctrl_o = exc_ctrl_cs == IRQ_PENDING;
  assign irq_sec_ctrl_o = irq_sec_q;
  assign irq_id_ctrl_o  = irq_id_q;

  always @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0) begin

      irq_id_q    <= 0;
      irq_sec_q   <= 1'b0;
      exc_ctrl_cs <= IDLE;

    end else begin

       case (exc_ctrl_cs)

        IDLE:
        begin
          if(irq_enable_ext & irq_i) begin
            exc_ctrl_cs <= IRQ_PENDING;
            irq_id_q    <= irq_id_i;
            irq_sec_q   <= irq_sec_i;
          end
        end

        IRQ_PENDING:
        begin
           case(1'b1)
            ctrl_ack_i:
              exc_ctrl_cs <= IRQ_DONE;
            ctrl_kill_i:
              exc_ctrl_cs <= IDLE;
            default:
              exc_ctrl_cs <= IRQ_PENDING;
          endcase
        end

        IRQ_DONE:
        begin
          irq_sec_q   <= 1'b0;
          exc_ctrl_cs <= IDLE;
        end

      endcase

    end
  end


`ifndef SYNTHESIS
  // synopsys translate_off
  // evaluate at falling edge to avoid duplicates during glitches
  // Removed this message as it pollutes too much the output and makes tests fail
  //always_ff @(negedge clk)
  //begin
  //  if (rst_n && exc_ctrl_cs == IRQ_DONE)
  //    $display("%t: Entering interrupt service routine. [%m]", $time);
  //end
  // synopsys translate_on
`endif

endmodule
