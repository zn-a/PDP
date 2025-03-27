/* # ########################################################################
# Copyright (C) 2019, Xilinx Inc - All rights reserved

# Licensed under the Apache License, Version 2.0 (the "License"). You may
# not use this file except in compliance with the License. A copy of the
# License is located at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
# ######################################################################## */

`timescale 1ns / 1ps

module tb;

    reg [31:0] base_addr  = 32'h4000_0000;
    reg [31:0] off_set    = 32'h0000_4320;
    reg [31:0] addr = 32'h4000_0000;
    reg [31:0] base_addr_data   = 32'h4200_0000;
    reg [31:0] base_addr_reboot = 32'h4000_8000;
    reg tb_ACLK;
    reg tb_ARESETn;
   
    wire temp_clk;
    wire temp_rstn;
    wire sys_clk;
    wire sys_rstn; 
   
    reg [31:0] expected_datas [0:3] = '{32'h94F15EFF, 32'hDACB2853, 32'hD0DE141B ,32'h92207E67};
    reg [31:0] read_data[0:3];
    reg [31:0] write_data;
    //wire [3:0] leds;
    reg resp; 
    
//     //------------------------------------------------------------------------
//    // Code file open & check
//    //------------------------------------------------------------------------   
//    integer data_file, scan_file;
//    logic [127:0] capture_data;
//    initial begin
//    data_file = $fopen("code_and_data_led_checked.coe","r");
//    if(data_file == 0) begin
//        $display("data_file handler was null");
//        $finish;
//        end
//     end
    
    initial 
    begin       
        tb_ACLK = 1'b0;
    end
    
    //------------------------------------------------------------------------
    // Simple Clock Generator
    //------------------------------------------------------------------------
    
    always #10 tb_ACLK = !tb_ACLK;


    //------------------------------------------------------------------------
    // Main body of the testbench
    //------------------------------------------------------------------------   
    initial
    begin
        tb.zynq_sys.riscv_i.processing_system7_0.inst.set_debug_level_info(0);

        $display ("running the tb");
        tb_ARESETn = 1'b0;
        repeat(20)@(posedge tb_ACLK);        
        tb_ARESETn = 1'b1;
        @(posedge tb_ACLK);
        
        repeat(5) @(posedge tb_ACLK);
          
        //Reset the PL
        tb.zynq_sys.riscv_i.processing_system7_0.inst.fpga_soft_reset(32'h1);
        tb.zynq_sys.riscv_i.processing_system7_0.inst.fpga_soft_reset(32'h0);
		#2000
        //This drives the LEDs on the GPIO output
//        tb.zynq_sys.riscv.processing_system7_0.inst.write_data(32'h41200000,4, 32'hFFFFFFFF, resp);
		#200
        //$display ("LEDs are toggled, observe the waveform");
        
//        scan_file = $fscanf(data_file, "%s\n", capture_data);
//        scan_file = $fscanf(data_file, "%s\n", capture_data);
        
        // set fetch_enable pin of the riscv to 1.
        //@(posedge sys_clk);
        
        main_core_execution();

        $display ("Simulation completed");
        $stop;
    end

    assign temp_clk = tb_ACLK;
    assign temp_rstn = tb_ARESETn;
    assign sys_clk = tb.zynq_sys.riscv_i.processing_system7_0.inst.FCLK_CLK0;
    assign sys_rstn = tb.zynq_sys.riscv_i.processing_system7_0.inst.FCLK_RESET0_N;
   
    riscv_wrapper zynq_sys(
        .DDR_addr(),
        .DDR_ba(),
        .DDR_cas_n(),
        .DDR_ck_n(),
        .DDR_ck_p(),
        //.DDR_cke(),
        .DDR_cs_n(),
        .DDR_dm(),
        .DDR_dq(),
        .DDR_dqs_n(),
        .DDR_dqs_p(),
        .DDR_odt(),
        .DDR_ras_n(),
        .DDR_reset_n(),
        .DDR_we_n(),
        .FIXED_IO_ddr_vrn(),
        .FIXED_IO_ddr_vrp(),
        .FIXED_IO_mio(),
        .FIXED_IO_ps_clk(temp_clk),
        .FIXED_IO_ps_porb(temp_rstn ),
        .FIXED_IO_ps_srstb(temp_rstn)
        //.ext_reset(tb_ARESETn)
    );
    
endmodule

task main_core_execution();

    $display("Setting fetch_enable pin of the riscv to 1...");
    tb.zynq_sys.riscv_i.processing_system7_0.inst.write_data((tb.base_addr_reboot + 8'h10), 4, 32'h0000_0010, tb.resp);
    repeat(5) @(posedge tb.sys_clk);
    
    repeat (800) begin
//       scan_file = $fscanf(data_file, "%d\n", capture_data);
        tb.zynq_sys.riscv_i.processing_system7_0.inst.read_data(tb.base_addr+tb.off_set+32'hA0,4,tb.read_data[0],tb.resp);
        #100;
        tb.zynq_sys.riscv_i.processing_system7_0.inst.read_data(tb.addr+tb.off_set+32'hA4,4,tb.read_data[1],tb.resp);
        #100;
        tb.zynq_sys.riscv_i.processing_system7_0.inst.read_data(tb.addr+tb.off_set+32'hA8,4,tb.read_data[2],tb.resp);
        #100;
        tb.zynq_sys.riscv_i.processing_system7_0.inst.read_data(tb.addr+tb.off_set+32'hAc,4,tb.read_data[3],tb.resp);
	    #100;
	    if(tb.read_data[0] == tb.expected_datas[0]) begin
            $display ("AXI VIP Test PASSED");
            $stop;
        end
    end
    if(tb.read_data[0] != tb.expected_datas[0]) begin
       $display ("AXI VIP Test FAILED");
    end

endtask

task test_initialization();
    // set reboot pin of the riscv to 1.
    @(posedge tb.sys_clk);
    $display("Setting reboot pin of the riscv to 1...");
    tb.zynq_sys.riscv_i.processing_system7_0.inst.write_data((tb.base_addr_reboot + 8'h10), 4, 32'h0000_0001, tb.resp);
    repeat(5) @(posedge tb.sys_clk);

    // set reboot pin of the riscv to 0.
    @(posedge tb.sys_clk);
    $display("Setting reboot pin of the riscv to 0...");
    tb.zynq_sys.riscv_i.processing_system7_0.inst.write_data((tb.base_addr_reboot + 8'h10), 4, 32'h0000_0000, tb.resp);
    repeat(5) @(posedge tb.sys_clk);

    // set fetch_enable pin of the riscv to 1.
    //@(posedge sys_clk);
    $display("Setting fetch_enable pin of the riscv to 1...");
    tb.zynq_sys.riscv_i.processing_system7_0.inst.write_data((tb.base_addr_reboot + 8'h10), 4, 32'h0000_0010, tb.resp);
    repeat(5) @(posedge tb.sys_clk);

    $display("Allowing the core to resume execution for 200 clk cycles...");
    repeat(200) @(posedge tb.sys_clk);

    // set fetch_enable pin of the riscv to 1.
    //@(posedge sys_clk);
    $display("Setting fetch_enable pin of the riscv to 0...");
    tb.zynq_sys.riscv_i.processing_system7_0.inst.write_data((tb.base_addr_reboot + 8'h10), 4, 32'h0000_0010, tb.resp);
    repeat(5) @(posedge tb.sys_clk);

    // set reboot pin of the riscv to 1.
    @(posedge tb.sys_clk);
    $display("Setting reboot pin of the riscv to 1...");
    tb.zynq_sys.riscv_i.processing_system7_0.inst.write_data((tb.base_addr_reboot + 8'h10), 4, 32'h0000_0001, tb.resp);
    repeat(5) @(posedge tb.sys_clk);

    // set reboot pin of the riscv to 0.
    @(posedge tb.sys_clk);
    $display("Setting reboot pin of the riscv to 0...");
    tb.zynq_sys.riscv_i.processing_system7_0.inst.write_data((tb.base_addr_reboot + 8'h10), 4, 32'h0000_0000, tb.resp);
    repeat(5) @(posedge tb.sys_clk);

endtask

task test_read_data_memory();
    // https://adaptivesupport.amd.com/s/question/0D54U00007BExvhSAD/can-i-access-the-saxilpd-of-the-zynq-ultrascale-using-the-command-writedata-and-readdata-of-zynq-ultrascale-vip-apis?language=en_US
    // tb.zynq_sys.riscv_i.processing_system7_0.inst.write_data(in [31:0]addr, in [7:0]wr_size, in [1023:0]wr_data, out response);
    // vip info: https://docs.amd.com/v/u/en-US/ds940-zynq-vip
    // Read addresses from data memory:
    tb.addr=tb.base_addr_data;
    @(posedge tb.sys_clk);
    for (int i = 0; i < 5; i++) begin
        tb.zynq_sys.riscv_i.processing_system7_0.inst.read_data(tb.addr,4,tb.read_data[0],tb.resp);
        $display("Read data (%0h): %0h", tb.addr, tb.read_data[0]);
        tb.addr += 32'h4;
        repeat(5) @(posedge tb.sys_clk);
    end

endtask

