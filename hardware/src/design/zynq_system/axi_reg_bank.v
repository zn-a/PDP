
`timescale 1 ns / 1 ps 

module axi_reg_bank (
        reg0_input,
        reg1_input,
        reg2_input,
        reg3_input,
        reg4_input,
        reg5_input,
        reg6_input,
        reg7_input,
        s_axi_AWVALID,
        s_axi_AWREADY,
        s_axi_AWADDR,
        s_axi_WVALID,
        s_axi_WREADY,
        s_axi_WDATA,
        s_axi_WSTRB,
        s_axi_ARVALID,
        s_axi_ARREADY,
        s_axi_ARADDR,
        s_axi_RVALID,
        s_axi_RREADY,
        s_axi_RDATA,
        s_axi_RRESP,
        s_axi_BVALID,
        s_axi_BREADY,
        s_axi_BRESP,
        s_axi_clk,
        s_axi_rstn
);

parameter    C_S_AXI_DATA_WIDTH = 32;
parameter    C_S_AXI_ADDR_WIDTH = 5;

parameter C_S_AXI_WSTRB_WIDTH = (32 / 8);

input  [C_S_AXI_DATA_WIDTH-1:0] reg0_input;
input  [C_S_AXI_DATA_WIDTH-1:0] reg1_input;
input  [C_S_AXI_DATA_WIDTH-1:0] reg2_input;
input  [C_S_AXI_DATA_WIDTH-1:0] reg3_input;
input  [C_S_AXI_DATA_WIDTH-1:0] reg4_input;
input  [C_S_AXI_DATA_WIDTH-1:0] reg5_input;
input  [C_S_AXI_DATA_WIDTH-1:0] reg6_input;
input  [C_S_AXI_DATA_WIDTH-1:0] reg7_input;
input   s_axi_AWVALID;
output   s_axi_AWREADY;
input  [C_S_AXI_ADDR_WIDTH - 1:0] s_axi_AWADDR;
input   s_axi_WVALID;
output   s_axi_WREADY;
input  [C_S_AXI_DATA_WIDTH - 1:0] s_axi_WDATA;
input  [C_S_AXI_WSTRB_WIDTH - 1:0] s_axi_WSTRB;
input   s_axi_ARVALID;
output   s_axi_ARREADY;
input  [C_S_AXI_ADDR_WIDTH - 1:0] s_axi_ARADDR;
output   s_axi_RVALID;
input   s_axi_RREADY;
output  [C_S_AXI_DATA_WIDTH - 1:0] s_axi_RDATA;
output  [1:0] s_axi_RRESP;
output   s_axi_BVALID;
input   s_axi_BREADY;
output  [1:0] s_axi_BRESP;
//input   ap_clk;
//input   ap_rst_n;
input   s_axi_clk;
input   s_axi_rstn;

wire [255:0] reg_inputs;// [7:0];

axi_reg_bank_controller #(
    .C_S_AXI_ADDR_WIDTH( C_S_AXI_ADDR_WIDTH ),
    .C_S_AXI_DATA_WIDTH( C_S_AXI_DATA_WIDTH ))
axi_reg_bank_controller_i(
    .AWVALID(s_axi_AWVALID),
    .AWREADY(s_axi_AWREADY),
    .AWADDR(s_axi_AWADDR),
    .WVALID(s_axi_WVALID),
    .WREADY(s_axi_WREADY),
    .WDATA(s_axi_WDATA),
    .WSTRB(s_axi_WSTRB),
    .ARVALID(s_axi_ARVALID),
    .ARREADY(s_axi_ARREADY),
    .ARADDR(s_axi_ARADDR),
    .RVALID(s_axi_RVALID),
    .RREADY(s_axi_RREADY),
    .RDATA(s_axi_RDATA),
    .RRESP(s_axi_RRESP),
    .BVALID(s_axi_BVALID),
    .BREADY(s_axi_BREADY),
    .BRESP(s_axi_BRESP),
    .ACLK(s_axi_clk),
    .ARESETN(s_axi_rstn),
    .ACLK_EN(1'b1),
    .reg_inputs(reg_inputs)
);

assign reg_inputs[31:0]    = reg0_input;
assign reg_inputs[63:32]   = reg1_input;
assign reg_inputs[95:64]   = reg2_input;
assign reg_inputs[127:96]  = reg3_input;
assign reg_inputs[159:128] = reg4_input;
assign reg_inputs[191:160] = reg5_input;
assign reg_inputs[223:192] = reg6_input;
assign reg_inputs[255:224] = reg7_input;

endmodule //reboot_riscv

