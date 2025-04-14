module riscv_register_file
#(
    parameter ADDR_WIDTH    = 5,
    parameter DATA_WIDTH    = 32,
    parameter FPU           = 0
)
(
    // Clock and Reset
    input  wire         clk,
    input  wire         rst_n,

    input  wire                   test_en_i,

    //Read port R1
    input  wire [ADDR_WIDTH-1:0]  raddr_a_i,
    output wire [DATA_WIDTH-1:0]  rdata_a_o,

    //Read port R2
    input  wire [ADDR_WIDTH-1:0]  raddr_b_i,
    output wire [DATA_WIDTH-1:0]  rdata_b_o,

    //Read port R3
    input  wire [ADDR_WIDTH-1:0]  raddr_c_i,
    output wire [DATA_WIDTH-1:0]  rdata_c_o,

    // Write port W1
    input wire [ADDR_WIDTH-1:0]   waddr_a_i,
    input wire [DATA_WIDTH-1:0]   wdata_a_i,
    input wire                    we_a_i,

    // Write port W2
    input wire [ADDR_WIDTH-1:0]   waddr_b_i,
    input wire [DATA_WIDTH-1:0]   wdata_b_i,
    input wire                    we_b_i
);

  // number of integer registers
  localparam    NUM_WORDS     = 2**(ADDR_WIDTH-1);
  // number of floating point registers
  localparam    NUM_FP_WORDS  = 2**(ADDR_WIDTH-1);
  localparam    NUM_TOT_WORDS = FPU ? NUM_WORDS + NUM_FP_WORDS : NUM_WORDS;

  // integer register file
  reg  [DATA_WIDTH-1:0]     mem [0:NUM_WORDS-1];

  // fp register file
  reg  [DATA_WIDTH-1:0]  mem_fp [0:NUM_FP_WORDS-1];

  // write enable signals for all registers
  reg [NUM_TOT_WORDS-1:0]                 we_a_dec;
  reg [NUM_TOT_WORDS-1:0]                 we_b_dec;

   //-----------------------------------------------------------------------------
   //-- READ : Read address decoder RAD
   //-----------------------------------------------------------------------------
   generate
	if (FPU == 1) begin
      assign rdata_a_o = raddr_a_i[5] ? mem_fp[raddr_a_i[4:0]] : mem[raddr_a_i[4:0]];
      assign rdata_b_o = raddr_b_i[5] ? mem_fp[raddr_b_i[4:0]] : mem[raddr_b_i[4:0]];
      assign rdata_c_o = raddr_c_i[5] ? mem_fp[raddr_c_i[4:0]] : mem[raddr_c_i[4:0]];
   end else begin
      assign rdata_a_o = mem[raddr_a_i[4:0]];
      assign rdata_b_o = mem[raddr_b_i[4:0]];
      assign rdata_c_o = mem[raddr_c_i[4:0]];
   end
	endgenerate
  //-----------------------------------------------------------------------------
  //-- WRITE : Write Address Decoder (WAD), combinatorial process
  //-----------------------------------------------------------------------------
  integer i;
  always @(*)
  begin : we_a_decoder
    for (i = 0; i < NUM_TOT_WORDS; i=i+1) begin
      if (waddr_a_i == i)
        we_a_dec[i] = we_a_i;
      else
        we_a_dec[i] = 1'b0;
    end
  end

  always @(*)
  begin : we_b_decoder
    for (i=0; i<NUM_TOT_WORDS; i=i+1) begin
      if (waddr_b_i == i)
        we_b_dec[i] = we_b_i;
      else
        we_b_dec[i] = 1'b0;
    end
  end

  genvar j,l;
  generate

   //-----------------------------------------------------------------------------
   //-- WRITE : Write operation
   //-----------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
      if(~rst_n) begin
        // R0 is nil
        mem[0] <= 32'b0;
      end else begin
        // R0 is nil
        mem[0] <= 32'b0;
      end
    end

    // R0 is nil
    // loop from 1 to NUM_WORDS-1 as R0 is nil
    for (j = 1; j < NUM_WORDS; j=j+1)
    begin : rf_gen

      always @(posedge clk, negedge rst_n)
      begin : register_write_behavioral
        if (rst_n==1'b0) begin
          mem[j] <= 32'b0;
        end else begin
          if(we_b_dec[j] == 1'b1)
            mem[j] <= wdata_b_i;
          else if(we_a_dec[j] == 1'b1)
            mem[j] <= wdata_a_i;
        end
      end

    end
    
     if (FPU == 1) begin
        // Floating point registers 
        for(l = 0; l < NUM_FP_WORDS; l=l+1) begin : fp_regs_gen
           always @(posedge clk, negedge rst_n)
             begin : fp_regs
                if (rst_n==1'b0)
                  mem_fp[l] <= 0;
                else if(we_b_dec[l+NUM_WORDS] == 1'b1)
                  mem_fp[l] <= wdata_b_i;
                else if(we_a_dec[l+NUM_WORDS] == 1'b1)
                  mem_fp[l] <= wdata_a_i;
             end
        end
     end
  endgenerate

endmodule
