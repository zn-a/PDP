`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/18/2024 04:59:48 PM
// Design Name: 
// Module Name: shift_addr
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module shift_addr #(parameter ADDR_WIDTH=15)( 

input [ADDR_WIDTH-1:0]  in_addr,
output [ADDR_WIDTH-3:0] out_addr

    );
    
    
    assign out_addr = {2'b00,in_addr[ADDR_WIDTH-1:2] };
endmodule
