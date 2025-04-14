
module cluster_clock_gating
(
    input  wire clk_i,
    input  wire en_i,
    input  wire test_en_i,
    output wire clk_o
  );

  assign clk_o = clk_i;

endmodule
