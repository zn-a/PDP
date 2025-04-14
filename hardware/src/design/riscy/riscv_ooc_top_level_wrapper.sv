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
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        restart,

  input  logic        clock_en_i,    // enable clock, otherwise it is gated
  input  logic        test_en_i,     // enable all clock gates for testing

  // Core ID, Cluster ID and boot address are considered more or less static
  input  logic [31:0] boot_addr_i,
  input  logic [ 3:0] core_id_i,
  input  logic [ 5:0] cluster_id_i,

  // Instruction memory interface
  output logic                         instr_req_o,
  input  logic                         instr_gnt_i,
  input  logic                         instr_rvalid_i,
  output logic                  [31:0] instr_addr_o,
  input  logic                  [31:0] instr_rdata_i,

  // Data memory interface
  output logic        data_req_o,
  input  logic        data_gnt_i,
  input  logic        data_rvalid_i,
  output logic        data_we_o,
  output logic [3:0]  data_be_o,
  output logic [31:0] data_addr_o,
  output logic [31:0] data_wdata_o,
  input  logic [31:0] data_rdata_i,
  input  logic        data_err_i,

  // apu-interconnect
  // handshake signals
  output logic                       apu_master_req_o,
  output logic                       apu_master_ready_o,
  input logic                        apu_master_gnt_i,
  // request channel
  output logic [31:0]                 apu_master_operands_0_o,
  output logic [31:0]                 apu_master_operands_1_o,
  output logic [31:0]                 apu_master_operands_2_o,
  output logic [5:0]                  apu_master_op_o,
  output logic                        apu_master_type_o,
  output logic [14:0]                 apu_master_flags_o,
  // response channel
  input logic                        apu_master_valid_i,
  input logic [31:0]                 apu_master_result_i,
  input logic [4:0]                  apu_master_flags_i,

  // Interrupt inputs
  input  logic        irq_i,                 // level sensitive IR lines
  input  logic [4:0]  irq_id_i,
  output logic        irq_ack_o,
  output logic [4:0]  irq_id_o,
  input  logic        irq_sec_i,

  output logic        sec_lvl_o,

  // Debug Interface
  input  logic        debug_req_i,
  output logic        debug_gnt_o,
  output logic        debug_rvalid_o,
  input  logic [14:0] debug_addr_i,
  input  logic        debug_we_i,
  input  logic [31:0] debug_wdata_i,
  output logic [31:0] debug_rdata_o,
  output logic        debug_halted_o,
  input  logic        debug_halt_i,
  input  logic        debug_resume_i,

  // CPU Control Signals
  input  logic        fetch_enable_i,
  output logic        core_busy_o,

  input  logic        ext_perf_counters_i
);

    // Flip-flop signals
    logic restart_ff;
    logic clock_en_i_ff;
    logic test_en_i_ff;
    logic [31:0] boot_addr_i_ff;
    logic [3:0] core_id_i_ff;
    logic [5:0] cluster_id_i_ff;
    logic instr_req_o_ff;
    logic instr_gnt_i_ff;
    logic instr_rvalid_i_ff;
    logic [31:0] instr_addr_o_ff;
    logic [31:0] instr_rdata_i_ff;
    logic data_req_o_ff;
    logic data_gnt_i_ff;
    logic data_rvalid_i_ff;
    logic data_we_o_ff;
    logic [3:0] data_be_o_ff;
    logic [31:0] data_addr_o_ff;
    logic [31:0] data_wdata_o_ff;
    logic [31:0] data_rdata_i_ff;
    logic data_err_i_ff;
    logic apu_master_req_o_ff;
    logic apu_master_ready_o_ff;
    logic apu_master_gnt_i_ff;
    logic [31:0] apu_master_operands_0_o_ff;
    logic [31:0] apu_master_operands_1_o_ff;
    logic [31:0] apu_master_operands_2_o_ff;
    logic [5:0] apu_master_op_o_ff;
    logic  apu_master_type_o_ff;
    logic [14:0] apu_master_flags_o_ff;
    logic apu_master_valid_i_ff;
    logic [31:0] apu_master_result_i_ff;
    logic [4:0] apu_master_flags_i_ff;
    logic irq_i_ff;
    logic [4:0] irq_id_i_ff;
    logic irq_ack_o_ff;
    logic [4:0] irq_id_o_ff;
    logic irq_sec_i_ff;
    logic sec_lvl_o_ff;
    logic debug_req_i_ff;
    logic debug_gnt_o_ff;
    logic debug_rvalid_o_ff;
    logic [14:0] debug_addr_i_ff;
    logic debug_we_i_ff;
    logic [31:0] debug_wdata_i_ff;
    logic [31:0] debug_rdata_o_ff;
    logic debug_halted_o_ff;
    logic debug_halt_i_ff;
    logic debug_resume_i_ff;
    logic fetch_enable_i_ff;
    logic core_busy_o_ff;
    logic ext_perf_counters_i_ff;

    always_ff @(posedge clk_i) begin
        instr_req_o            <= instr_req_o_ff;
        instr_addr_o          <= instr_addr_o_ff;
        data_req_o             <= data_req_o_ff;
        data_we_o              <= data_we_o_ff;
        data_be_o              <= data_be_o_ff;
        data_addr_o            <= data_addr_o_ff;
        data_wdata_o           <= data_wdata_o_ff;
        apu_master_req_o       <= apu_master_req_o_ff;
        apu_master_ready_o     <= apu_master_ready_o_ff;
        apu_master_operands_0_o <= apu_master_operands_0_o_ff;
        apu_master_operands_1_o <= apu_master_operands_1_o_ff;
        apu_master_operands_2_o <= apu_master_operands_2_o_ff;
        apu_master_op_o        <= apu_master_op_o_ff;
        apu_master_type_o      <= apu_master_type_o_ff;
        apu_master_flags_o     <= apu_master_flags_o_ff;
        irq_ack_o              <= irq_ack_o_ff;
        irq_id_o               <= irq_id_o_ff;
        sec_lvl_o              <= sec_lvl_o_ff;
        debug_gnt_o            <= debug_gnt_o_ff;
        debug_rvalid_o         <= debug_rvalid_o_ff;
        debug_rdata_o          <= debug_rdata_o_ff;
        debug_halted_o         <= debug_halted_o_ff;
        core_busy_o            <= core_busy_o_ff;
        restart_ff        <= restart;
        clock_en_i_ff     <= clock_en_i;
        test_en_i_ff      <= test_en_i;
        boot_addr_i_ff    <= boot_addr_i;
        core_id_i_ff      <= core_id_i;
        cluster_id_i_ff   <= cluster_id_i;
        instr_gnt_i_ff    <= instr_gnt_i;
        instr_rvalid_i_ff <= instr_rvalid_i;
        instr_rdata_i_ff  <= instr_rdata_i;
        data_gnt_i_ff     <= data_gnt_i;
        data_rvalid_i_ff  <= data_rvalid_i;
        data_rdata_i_ff   <= data_rdata_i;
        data_err_i_ff     <= data_err_i;
        apu_master_gnt_i_ff    <= apu_master_gnt_i;
        apu_master_valid_i_ff  <= apu_master_valid_i;
        apu_master_result_i_ff <= apu_master_result_i;
        apu_master_flags_i_ff  <= apu_master_flags_i;
        irq_i_ff          <= irq_i;
        irq_id_i_ff       <= irq_id_i;
        irq_sec_i_ff      <= irq_sec_i;
        debug_req_i_ff    <= debug_req_i;
        debug_addr_i_ff   <= debug_addr_i;
        debug_we_i_ff     <= debug_we_i;
        debug_wdata_i_ff  <= debug_wdata_i;
        debug_halt_i_ff   <= debug_halt_i;
        debug_resume_i_ff <= debug_resume_i;
        fetch_enable_i_ff <= fetch_enable_i;
        ext_perf_counters_i_ff <= ext_perf_counters_i;
    end




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
  .clk_i               (clk_i),
  .rst_ni              (rst_ni),
  .restart             (restart_ff),
  .clock_en_i          (clock_en_i_ff),
  .test_en_i           (test_en_i_ff),
  .boot_addr_i         (boot_addr_i_ff),
  .core_id_i           (core_id_i_ff),
  .cluster_id_i        (cluster_id_i),
  .instr_req_o         (instr_req_o_ff),
  .instr_gnt_i         (instr_gnt_i_ff),
  .instr_rvalid_i      (instr_rvalid_i_ff),
  .instr_addr_o        (instr_addr_o_ff),
  .instr_rdata_i       (instr_rdata_i_ff),
  .data_req_o          (data_req_o_ff),
  .data_gnt_i          (data_gnt_i_ff),
  .data_rvalid_i       (data_rvalid_i_ff),
  .data_we_o           (data_we_o_ff),
  .data_be_o           (data_be_o_ff),
  .data_addr_o         (data_addr_o_ff),
  .data_wdata_o        (data_wdata_o_ff),
  .data_rdata_i        (data_rdata_i_ff),
  .data_err_i          (data_err_i_ff),
  .apu_master_req_o    (apu_master_req_o_ff),
  .apu_master_ready_o  (apu_master_ready_o_ff),
  .apu_master_gnt_i    (apu_master_gnt_i_ff),
  .apu_master_operands_0_o (apu_master_operands_0_o_ff),
  .apu_master_operands_1_o (apu_master_operands_1_o_ff),
  .apu_master_operands_2_o (apu_master_operands_2_o_ff),
  .apu_master_op_o     (apu_master_op_o_ff),
  .apu_master_type_o   (apu_master_type_o_ff),
  .apu_master_flags_o  (apu_master_flags_o_ff),
  .apu_master_valid_i  (apu_master_valid_i_ff),
  .apu_master_result_i (apu_master_result_i_ff),
  .apu_master_flags_i  (apu_master_flags_i_ff),
  .irq_i               (irq_i_ff),
  .irq_id_i            (irq_id_i_ff),
  .irq_ack_o           (irq_ack_o_ff),
  .irq_id_o            (irq_id_o_ff),
  .irq_sec_i           (irq_sec_i_ff),
  .sec_lvl_o           (sec_lvl_o_ff),
  .debug_req_i         (debug_req_i_ff),
  .debug_gnt_o         (debug_gnt_o_ff),
  .debug_rvalid_o      (debug_rvalid_o_ff),
  .debug_addr_i        (debug_addr_i_ff),
  .debug_we_i          (debug_we_i_ff),
  .debug_wdata_i       (debug_wdata_i_ff),
  .debug_rdata_o       (debug_rdata_o_ff),
  .debug_halted_o      (debug_halted_o_ff),
  .debug_halt_i        (debug_halt_i_ff),
  .debug_resume_i      (debug_resume_i_ff),
  .fetch_enable_i      (fetch_enable_i_ff),
  .core_busy_o         (core_busy_o_ff),
  .ext_perf_counters_i (ext_perf_counters_i_ff)
 );
endmodule
