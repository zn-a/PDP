// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.1 (64-bit)
// Tool Version Limit: 2023.05
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
`timescale 1ns/1ps
/*
module mem_snoop_match #(
    parameter    C_ADDR_WIDTH = 32,
    parameter    C_DATA_WIDTH = 32,
    parameter    C_SNOOP_ADDR = 0,
    parameter    C_SNOOP_DATA = 0
)(
    input  logic                            ACLK,
    input  logic                            RSTN,
    input  logic                            CLK_EN,
    input  logic                            WEN,
    input  logic [C_ADDR_WIDTH-1:0]         WADDR,
    input  logic [C_DATA_WIDTH-1:0]         WDATA,
    output logic [31:0]                     CLK_COUNT,
    output logic                            MATCH
);

//------------------------Local signal-------------------
    logic  [31:0]                   clk_counter = '0;
    logic                           match_found = 1'b0;
    logic                           addr_match;
    logic                           data_match;


//------------------------Instantiation------------------

    // we increase the clock counter always when clk enable is asserted and while we haven't found a match:
    always_ff @(posedge ACLK) begin
        if (!RSTN) begin
            clk_counter <= '0;
        end else begin
            clk_counter <= (CLK_EN && !match_found) ? (clk_counter + 1) : clk_counter;
        end
    end

   // register to keep track of having found a match
   always_ff @(posedge ACLK) begin
        if (!RSTN) begin
            match_found <= 1'b0;
        end else begin
            match_found <= (addr_match && data_match) ? (1'b1) : match_found;
        end
    end

    // data and address comparison
    assign data_match = (WEN && WDATA == C_SNOOP_DATA) ? 1'b1 : 1'b0;
    assign addr_match = (WEN && WADDR == C_SNOOP_ADDR) ? 1'b1 : 1'b0;

endmodule
*/

module mem_snoop_match #(
    parameter C_ADDR_WIDTH = 32,
    parameter C_DATA_WIDTH = 32,
    parameter C_SNOOP_ADDR = 0,
    parameter C_SNOOP_DATA = 0
)(
    input  ACLK,
    input  RSTN,
    input  CLK_EN,
    input  WEN,
    input  [C_ADDR_WIDTH-1:0] WADDR,
    input  [C_DATA_WIDTH-1:0] WDATA,
    output [31:0] CLK_COUNT,
    output MATCH
);

//------------------------Local signal-------------------
reg  [31:0] clk_counter;
reg        match_found;
wire       addr_match;
wire       data_match;

//------------------------Sequential logic------------------

always @(posedge ACLK) begin
    if (!RSTN) begin
        clk_counter <= 32'd0;
    end else begin
        if (CLK_EN && !match_found)
            clk_counter <= clk_counter + 1;
        else
            clk_counter <= clk_counter;
    end
end

always @(posedge ACLK) begin
    if (!RSTN) begin
        match_found <= 1'b0;
    end else begin
        if (addr_match && data_match)
            match_found <= 1'b1;
        else
            match_found <= match_found;
    end
end

//------------------------Combinational logic------------------

assign data_match = (WEN && WDATA == C_SNOOP_DATA) ? 1'b1 : 1'b0;
assign addr_match = (WEN && WADDR == C_SNOOP_ADDR) ? 1'b1 : 1'b0;

//------------------------Outputs------------------

assign CLK_COUNT = clk_counter;
assign MATCH = match_found;

endmodule