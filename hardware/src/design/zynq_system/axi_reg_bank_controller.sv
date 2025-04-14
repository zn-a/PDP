// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.1 (64-bit)
// Tool Version Limit: 2023.05
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
`timescale 1ns/1ps
module axi_reg_bank_controller #(
    parameter    C_S_AXI_ADDR_WIDTH = 5,
    parameter    C_S_AXI_DATA_WIDTH = 32
)(
    input  logic                          ACLK,
    input  logic                          ARESETN,
    input  logic                          ACLK_EN,
    input  logic [C_S_AXI_ADDR_WIDTH-1:0] AWADDR,
    input  logic                          AWVALID,
    output logic                          AWREADY,
    input  logic [C_S_AXI_DATA_WIDTH-1:0] WDATA,
    input  logic [C_S_AXI_DATA_WIDTH/8-1:0] WSTRB,
    input  logic                          WVALID,
    output logic                          WREADY,
    output logic [1:0]                    BRESP,
    output logic                          BVALID,
    input  logic                          BREADY,
    input  logic [C_S_AXI_ADDR_WIDTH-1:0] ARADDR,
    input  logic                          ARVALID,
    output logic                          ARREADY,
    output logic [C_S_AXI_DATA_WIDTH-1:0] RDATA,
    output logic [1:0]                    RRESP,
    output logic                          RVALID,
    input  logic                          RREADY,
    input  logic [255:0]                  reg_inputs 
);
//------------------------Address Info-------------------
// 0x00 : Reg 0
// 0x04 : Reg 1
// 0x08 : Reg 2
// 0x0c : Reg 3
// 0x10 : Reg 4
// 0x14 : Reg 5
// 0x18 : Reg 6
// 0x1C : Reg 7
localparam
    WRIDLE                   = 2'd0,
    WRDATA                   = 2'd1,
    WRRESP                   = 2'd2,
    WRRESET                  = 2'd3,
    RDIDLE                   = 2'd0,
    RDDATA                   = 2'd1,
    RDRESET                  = 2'd2,
    ADDR_BITS                = 5;

//------------------------Local signal-------------------
    logic  [1:0]                    wstate = WRRESET;
    logic  [1:0]                    wnext;
    logic  [ADDR_BITS-1:0]          waddr;
    logic  [C_S_AXI_DATA_WIDTH-1:0] wmask;
    logic                           aw_hs;
    logic                           w_hs;
    logic  [1:0]                    rstate = RDRESET;
    logic  [1:0]                    rnext;
    logic  [C_S_AXI_DATA_WIDTH-1:0] rdata;
    logic                           ar_hs;
    logic  [ADDR_BITS-1:0]          raddr;
    // internal registers
    logic  [31:0]                   registers [7:0];


//------------------------Instantiation------------------


//------------------------AXI write fsm------------------
assign AWREADY = (wstate == WRIDLE);
assign WREADY  = (wstate == WRDATA);
assign BRESP   = 2'b00;  // OKAY
assign BVALID  = (wstate == WRRESP);
assign wmask   = { {8{WSTRB[3]}}, {8{WSTRB[2]}}, {8{WSTRB[1]}}, {8{WSTRB[0]}} };
assign aw_hs   = AWVALID & AWREADY;
assign w_hs    = WVALID & WREADY;

// wstate
always_ff @(posedge ACLK) begin
    if (!ARESETN)
        wstate <= WRRESET;
    else if (ACLK_EN)
        wstate <= wnext;
end

// wnext
always_comb begin
    case (wstate)
        WRIDLE:
            if (AWVALID)
                wnext = WRDATA;
            else
                wnext = WRIDLE;
        WRDATA:
            if (WVALID)
                wnext = WRRESP;
            else
                wnext = WRDATA;
        WRRESP:
            if (BREADY)
                wnext = WRIDLE;
            else
                wnext = WRRESP;
        default:
            wnext = WRIDLE;
    endcase
end

// waddr
always_ff @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (aw_hs)
            waddr <= AWADDR[ADDR_BITS-1:0];
    end
end

//------------------------AXI read fsm-------------------
assign ARREADY = (rstate == RDIDLE);
assign RDATA   = rdata;
assign RRESP   = 2'b00;  // OKAY
assign RVALID  = (rstate == RDDATA);
assign ar_hs   = ARVALID & ARREADY;
assign raddr   = ARADDR[ADDR_BITS-1:0];

// rstate
always_ff @(posedge ACLK) begin
    if (!ARESETN)
        rstate <= RDRESET;
    else if (ACLK_EN)
        rstate <= rnext;
end

// rnext
always_comb begin
    case (rstate)
        RDIDLE:
            if (ARVALID)
                rnext = RDDATA;
            else
                rnext = RDIDLE;
        RDDATA:
            if (RREADY & RVALID)
                rnext = RDIDLE;
            else
                rnext = RDDATA;
        default:
            rnext = RDIDLE;
    endcase
end

// rdata
always_ff @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (ar_hs) begin
            rdata <= '0;
            case (raddr)
                32'h00: begin
                    rdata <= registers[0];
                end
                32'h04: begin
                    rdata <= registers[1];
                end
                32'h08: begin
                    rdata <= registers[2];
                end
                32'h0C: begin
                    rdata <= registers[3];
                end
                32'h10: begin
                    rdata <= registers[4];
                end
                32'h14: begin
                    rdata <= registers[5];
                end
                32'h18: begin
                    rdata <= registers[6];
                end
                32'h1C: begin
                    rdata <= registers[7];
                end
                default: rdata <= 32'hFFFF_FFFF;
            endcase
        end
    end
end


//------------------------Register logic-----------------

genvar i;
generate
    for(i=0; i<8; i++) begin
        always_ff @(posedge ACLK) begin
            if (!ARESETN)
                registers[i] <= '0;
            else if (ACLK_EN) begin
                registers[i] <= reg_inputs[i*32 +: 32];
            end
        end
    end
endgenerate

endmodule