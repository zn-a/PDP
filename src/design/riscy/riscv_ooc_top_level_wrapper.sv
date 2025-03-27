// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Matthias Baer - baermatt@student.ethz.ch                   //
//                                                                            //
// Additional contributions by:                                               //
//                 Igor Loi - igor.loi@unibo.it                               //
//                 Andreas Traber - atraber@student.ethz.ch                   //
//                 Sven Stucki - svstucki@student.ethz.ch                     //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                 Halfdan Bechmann - halfdan.bechmann@silabs.com             //
//                 Øystein Knauserud - oystein.knauserud@silabs.com           //
//                 Michael Platzer - michael.platzer@tuwien.ac.at             //
//                                                                            //
// Design Name:    Top level module                                           //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Top level module of the RISC-V core.                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module riscv_ooc_top_level_wrapper #()
(
  // Clock and Reset
  input  wire        clk_i,
  input  wire        rst_ni,
  input  wire        restart,

  input  wire        clock_en_i,    // enable clock, otherwise it is gated
  input  wire        test_en_i,     // enable all clock gates for testing

  // Core ID, Cluster ID and boot address are considered more or less static
  input  wire [31:0] boot_addr_i,
  input  wire [ 3:0] core_id_i,
  input  wire [ 5:0] cluster_id_i,

  // Instruction memory interface
  output wire                         instr_req_o,
  input  wire                         instr_gnt_i,
  input  wire                         instr_rvalid_i,
  output wire                  [31:0] instr_addr_o,
  input  wire                  [31:0] instr_rdata_i,

  // Data memory interface
  output wire        data_req_o,
  input  wire        data_gnt_i,
  input  wire        data_rvalid_i,
  output wire        data_we_o,
  output wire [3:0]  data_be_o,
  output wire [31:0] data_addr_o,
  output wire [31:0] data_wdata_o,
  input  wire [31:0] data_rdata_i,
  input  wire        data_err_i,

  // apu-interconnect
  // handshake signals
  output wire                       apu_master_req_o,
  output wire                       apu_master_ready_o,
  input wire                        apu_master_gnt_i,
  // request channel
  output wire [31:0]                 apu_master_operands_0_o,
  output wire [31:0]                 apu_master_operands_1_o,
  output wire [31:0]                 apu_master_operands_2_o,
  output wire [5:0]                  apu_master_op_o,
  output wire                        apu_master_type_o,
  output wire [14:0]                 apu_master_flags_o,
  // response channel
  input wire                        apu_master_valid_i,
  input wire [31:0]                 apu_master_result_i,
  input wire [4:0]                  apu_master_flags_i,

  // Interrupt inputs
  input  wire        irq_i,                 // level sensitive IR lines
  input  wire [4:0]  irq_id_i,
  output wire        irq_ack_o,
  output wire [4:0]  irq_id_o,
  input  wire        irq_sec_i,

  output wire        sec_lvl_o,

  // Debug Interface
  input  wire        debug_req_i,
  output wire        debug_gnt_o,
  output wire        debug_rvalid_o,
  input  wire [14:0] debug_addr_i,
  input  wire        debug_we_i,
  input  wire [31:0] debug_wdata_i,
  output wire [31:0] debug_rdata_o,
  output wire        debug_halted_o,
  input  wire        debug_halt_i,
  input  wire        debug_resume_i,

  // CPU Control Signals
  input  wire        fetch_enable_i,
  output wire        core_busy_o,

  input  wire        ext_perf_counters_i
);
 // Instantiate the core module
 riscv_core
 #(
  .N_EXT_PERF_COUNTERS (     0       ),
  .FPU                 (     0       ),
  .SHARED_FP           (     0       ),
  .SHARED_FP_DIVSQRT   (     2       )
 )
 RISCV_CORE
 (
  .*
 );
endmodule
