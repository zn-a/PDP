
module riscv_prefetch_buffer
(
  input  wire        clk,
  input  wire        rst_n,

  input  wire        req_i,

  input  wire        branch_i,
  input  wire [31:0] addr_i,

  input  wire        hwloop_i,
  input  wire [31:0] hwloop_target_i,
  output wire        hwlp_branch_o,

  input  wire        ready_i,
  output wire        valid_o,
  output wire [31:0] rdata_o,
  output wire [31:0] addr_o,
  output wire        is_hwlp_o, // is set when the currently served data is from a hwloop

  // goes to instruction memory / instruction cache
  output reg         instr_req_o,
  input  wire        instr_gnt_i,
  output reg  [31:0] instr_addr_o,
  input  wire [31:0] instr_rdata_i,
  input  wire        instr_rvalid_i,

  // Prefetch Buffer Status
  output wire        busy_o
);
  localparam IDLE=0, WAIT_GNT=1, WAIT_RVALID=2, WAIT_ABORTED=3;
  reg [1:0]  CS, NS;
  localparam HWLP_NONE=0, HWLP_IN=1, HWLP_FETCHING=2, HWLP_DONE=3, HWLP_UNALIGNED_COMPRESSED=4;
  reg [2:0]  hwlp_CS, hwlp_NS;

  reg  [31:0] instr_addr_q;
  wire [31:0] fetch_addr;
  reg         fetch_is_hwlp;
  reg         addr_valid;

  reg         fifo_valid;
  wire        fifo_ready;
  reg         fifo_clear;
  reg         fifo_hwlp;

  wire        valid_stored;
  reg         hwlp_masked; 
  reg		  hwlp_branch, hwloop_speculative;
  wire        unaligned_is_compressed;


  //////////////////////////////////////////////////////////////////////////////
  // prefetch buffer status
  //////////////////////////////////////////////////////////////////////////////

  assign busy_o = (CS != IDLE) || instr_req_o;

  //////////////////////////////////////////////////////////////////////////////
  // fetch fifo
  // consumes addresses and rdata
  //////////////////////////////////////////////////////////////////////////////

  riscv_fetch_fifo fifo_i
  (
    .clk                   ( clk               ),
    .rst_n                 ( rst_n             ),

    .clear_i               ( fifo_clear        ),

    .in_addr_i             ( instr_addr_q      ),
    .in_rdata_i            ( instr_rdata_i     ),
    .in_valid_i            ( fifo_valid        ),
    .in_ready_o            ( fifo_ready        ),

    .in_replace2_i         ( fifo_hwlp         ),
    .in_is_hwlp_i          ( fifo_hwlp         ),

    .out_valid_o           ( valid_o           ),
    .out_ready_i           ( ready_i           ),
    .out_rdata_o           ( rdata_o           ),
    .out_addr_o            ( addr_o            ),
    .unaligned_is_compressed_o ( unaligned_is_compressed ),
    .out_valid_stored_o    ( valid_stored      ),
    .out_is_hwlp_o         ( is_hwlp_o         )
  );


  //////////////////////////////////////////////////////////////////////////////
  // fetch addr
  //////////////////////////////////////////////////////////////////////////////

  assign fetch_addr    = {instr_addr_q[31:2], 2'b00} + 32'd4;

  assign hwlp_branch_o = hwlp_branch;

  always @(*)
  begin
    hwlp_NS            = hwlp_CS;
    fifo_hwlp          = 1'b0;
    fifo_clear         = 1'b0;
    hwlp_branch        = 1'b0;
    hwloop_speculative = 1'b0;

     case (hwlp_CS)
      HWLP_NONE: begin
        if (hwloop_i) begin
          hwlp_masked = ~instr_addr_q[1];

          if(valid_o & unaligned_is_compressed & instr_addr_q[1]) begin
              /* We did not jump (hwlp_masked) because
                 as the instruction was unaligned, so we have to finish
                 the fetch of the second part
                 We did not jump but once the instruction came back from the iMem,
                 we was that it is compressed, therefore we could jump.
                 The pc_if contains the HWLoop final address (hwloop_i)
                 and because we did not jump, the pc_if will be pc_if+4 and not the target of the HWloop.
                 At the next cycle, do a jump to the HWloop target. We will use a "strong" jump signal as the branch_i one
                 as we need to invalidate the IF instruction
               */
               hwlp_NS            = HWLP_UNALIGNED_COMPRESSED;
               hwloop_speculative = 1'b1;
          end else begin
              if (fetch_is_hwlp)
                hwlp_NS = HWLP_FETCHING;
              else
                hwlp_NS = HWLP_IN;
          end

          if (ready_i)
           fifo_clear = 1'b1;

        end
        else begin
          hwlp_masked = 1'b0;
        end
      end

      HWLP_UNALIGNED_COMPRESSED: begin
        hwlp_branch  = 1'b1;
        hwlp_NS      = HWLP_FETCHING;
        fifo_clear   = 1'b1;
      end


      HWLP_IN: begin
        hwlp_masked = 1'b1;

        if (fetch_is_hwlp)
          hwlp_NS = HWLP_FETCHING;

        if (ready_i)
          fifo_clear = 1'b1;
      end

      // just waiting for rvalid really
      HWLP_FETCHING: begin
        hwlp_masked = 1'b0;

        fifo_hwlp = 1'b1;

        if (instr_rvalid_i & (CS != WAIT_ABORTED)) begin
          if (valid_o & is_hwlp_o)
            hwlp_NS = HWLP_NONE;
          else
            hwlp_NS = HWLP_DONE;
        end else begin
          if (ready_i)
            fifo_clear = 1'b1;
        end
      end

      HWLP_DONE: begin
        hwlp_masked = 1'b0;

        if (valid_o & is_hwlp_o)
          hwlp_NS = HWLP_NONE;
      end

      default: begin
        hwlp_masked = 1'b0;

        hwlp_NS = HWLP_NONE;
      end
    endcase

    if (branch_i) begin
      hwlp_NS    = HWLP_NONE;
      fifo_clear = 1'b1;
    end
  end

  //////////////////////////////////////////////////////////////////////////////
  // instruction fetch FSM
  // deals with instruction memory / instruction cache
  //////////////////////////////////////////////////////////////////////////////

  always @(*)
  begin
    instr_req_o   = 1'b0;
    instr_addr_o  = fetch_addr;
    fifo_valid    = 1'b0;
    addr_valid    = 1'b0;
    fetch_is_hwlp = 1'b0;
    NS            = CS;

     case(CS)
      // default state, not waiting for requested data
      IDLE:
      begin
        instr_addr_o = fetch_addr;
        instr_req_o  = 1'b0;

        if (branch_i | hwlp_branch)
          instr_addr_o = branch_i ? addr_i : instr_addr_q;
        else if(hwlp_masked & valid_stored)
          instr_addr_o = hwloop_target_i;

        if (req_i & (fifo_ready | branch_i | hwlp_branch | (hwlp_masked & valid_stored))) begin
          instr_req_o = 1'b1;
          addr_valid  = 1'b1;

          if (hwlp_masked & valid_stored) begin
            fetch_is_hwlp = 1'b1;
          end

          if(instr_gnt_i) //~>  granted request
            NS = WAIT_RVALID;
          else begin //~> got a request but no grant
            NS = WAIT_GNT;
          end
        end
      end // case: IDLE

      // we sent a request but did not yet get a grant
      WAIT_GNT:
      begin
        instr_addr_o = instr_addr_q;
        instr_req_o  = 1'b1;

        if (branch_i | hwlp_branch) begin
          instr_addr_o = branch_i ? addr_i : instr_addr_q;
          addr_valid   = 1'b1;
        end else if (hwlp_masked & valid_stored) begin
          instr_addr_o  = hwloop_target_i;
          addr_valid    = 1'b1;
          fetch_is_hwlp = 1'b1;
        end

        if(instr_gnt_i)
          NS = WAIT_RVALID;
        else
          NS = WAIT_GNT;
      end // case: WAIT_GNT

      // we wait for rvalid, after that we are ready to serve a new request
      WAIT_RVALID: begin
        instr_addr_o = fetch_addr;

        if (branch_i | hwlp_branch)
          instr_addr_o = branch_i ? addr_i : instr_addr_q;
        else if (hwlp_masked)
          instr_addr_o  = hwloop_target_i;

        if (req_i & (fifo_ready | branch_i | hwlp_branch |hwlp_masked)) begin
          // prepare for next request

          if (instr_rvalid_i) begin
            instr_req_o = 1'b1;
            fifo_valid  = 1'b1;
            addr_valid  = 1'b1;

            if (hwlp_masked) begin
              fetch_is_hwlp = 1'b1;
            end

            if (instr_gnt_i) begin
              NS = WAIT_RVALID;
            end else begin
              NS = WAIT_GNT;
            end
          end else begin
            // we are requested to abort our current request
            // we didn't get an rvalid yet, so wait for it
            if (branch_i | hwlp_branch) begin
              addr_valid = 1'b1;
              NS         = WAIT_ABORTED;
            end else if (hwlp_masked & valid_o) begin
              addr_valid    = 1'b1;
              fetch_is_hwlp = 1'b1;
              NS            = WAIT_ABORTED;
            end
          end
        end else begin
          // just wait for rvalid and go back to IDLE, no new request

          if (instr_rvalid_i) begin
            fifo_valid = 1'b1;
            NS         = IDLE;
          end
        end
      end // case: WAIT_RVALID

      // our last request was aborted, but we didn't yet get a rvalid and
      // there was no new request sent yet
      // we assume that req_i is set to high
      WAIT_ABORTED: begin
        instr_addr_o = instr_addr_q;

        if (branch_i | hwlp_branch) begin
          instr_addr_o = branch_i ? addr_i : instr_addr_q;
          addr_valid   = 1'b1;
        end

        if (instr_rvalid_i) begin
          instr_req_o  = 1'b1;
          // no need to send address, already done in WAIT_RVALID

          if (instr_gnt_i) begin
            NS = WAIT_RVALID;
          end else begin
            NS = WAIT_GNT;
          end
        end
      end

      default:
      begin
        NS          = IDLE;
        instr_req_o = 1'b0;
      end
    endcase
  end

  //////////////////////////////////////////////////////////////////////////////
  // registers
  //////////////////////////////////////////////////////////////////////////////

  always @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      CS              <= IDLE;
      hwlp_CS         <= HWLP_NONE;
      instr_addr_q    <= 0;
    end
    else
    begin
      CS              <= NS;
      hwlp_CS         <= hwlp_NS;

      if (addr_valid) begin
        instr_addr_q    <= (hwloop_speculative & ~branch_i) ? hwloop_target_i : instr_addr_o;
      end
    end
  end

endmodule
