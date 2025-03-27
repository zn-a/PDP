module riscv_fetch_fifo
(
    input  wire        clk,
    input  wire        rst_n,

    // control signals
    input  wire        clear_i,          // clears the contents of the fifo

    // input port
    input  wire [31:0] in_addr_i,
    input  wire [31:0] in_rdata_i,
    input  wire        in_valid_i,
    output wire        in_ready_o,

    input  wire        in_replace2_i, // replaces second entry if there is one: "to be served after this instr"
    input  wire        in_is_hwlp_i,

    // output port
    output reg        out_valid_o,
    input  wire        out_ready_i,
    output reg [31:0] out_rdata_o,
    output wire [31:0] out_addr_o,
    output wire        unaligned_is_compressed_o,
    output reg        out_valid_stored_o, // same as out_valid_o, except that if something is incoming now it is not included. This signal is available immediately as it comes directly out of FFs
    output wire        out_is_hwlp_o
  );

  localparam DEPTH = 4; // must be 3 or greater

  // index 0 is used for output
  reg [31:0]  addr_n [0:DEPTH-1];
  reg [31:0]  addr_int [0:DEPTH-1];
  reg [31:0]  addr_Q [0:DEPTH-1];
  reg [31:0]  rdata_n [0:DEPTH-1];
  reg [31:0]  rdata_int [0:DEPTH-1];  
  reg [31:0]  rdata_Q [0:DEPTH-1];
  reg [0:DEPTH-1]         valid_n,   valid_int,   valid_Q;
  reg [0:1      ]         is_hwlp_n, is_hwlp_int, is_hwlp_Q;

  wire             [31:0]  addr_next;
  wire             [31:0]  rdata, rdata_unaligned;
  wire                     valid, valid_unaligned;

  wire                     aligned_is_compressed, unaligned_is_compressed;
  wire                     aligned_is_compressed_st, unaligned_is_compressed_st;

  //////////////////////////////////////////////////////////////////////////////
  // output port
  //////////////////////////////////////////////////////////////////////////////


  assign rdata = (valid_Q[0]) ? rdata_Q[0] : in_rdata_i;
  assign valid = valid_Q[0] || in_valid_i || is_hwlp_Q[1];

  assign rdata_unaligned = (valid_Q[1]) ? {rdata_Q[1][15:0], rdata[31:16]} : {in_rdata_i[15:0], rdata[31:16]};
  // it is implied that rdata_valid_Q[0] is set
  assign valid_unaligned = (valid_Q[1] || (valid_Q[0] && in_valid_i));

  assign unaligned_is_compressed_o  = unaligned_is_compressed;

  assign unaligned_is_compressed    = rdata[17:16] != 2'b11;
  assign aligned_is_compressed      = rdata[1:0] != 2'b11;
  assign unaligned_is_compressed_st = rdata_Q[0][17:16] != 2'b11;
  assign aligned_is_compressed_st   = rdata_Q[0][1:0] != 2'b11;

  //////////////////////////////////////////////////////////////////////////////
  // instruction aligner (if unaligned)
  //////////////////////////////////////////////////////////////////////////////

  always @(*)
  begin
    // serve the aligned case even though the output address is unaligned when
    // the next instruction will be from a hardware loop target
    // in this case the current instruction is already prealigned in element 0
    if (out_addr_o[1] && (~is_hwlp_Q[1])) begin
      // unaligned case
      out_rdata_o = rdata_unaligned;

      if (unaligned_is_compressed)
        out_valid_o = valid;
      else
        out_valid_o = valid_unaligned;
    end else begin
      // aligned case
      out_rdata_o = rdata;
      out_valid_o = valid;
    end
  end

  assign out_addr_o    = (valid_Q[0]) ? addr_Q[0] : in_addr_i;
  assign out_is_hwlp_o = (valid_Q[0]) ? is_hwlp_Q[0] : in_is_hwlp_i;

  // this valid signal must not depend on signals from outside!
  always @(*)
  begin
    out_valid_stored_o = 1'b1;

    if (out_addr_o[1] && (~is_hwlp_Q[1])) begin
      if (unaligned_is_compressed_st)
        out_valid_stored_o = 1'b1;
      else
        out_valid_stored_o = valid_Q[1];
    end else begin
      out_valid_stored_o = valid_Q[0];
    end
  end


  //////////////////////////////////////////////////////////////////////////////
  // input port
  //////////////////////////////////////////////////////////////////////////////

  // we accept data as long as our fifo is not full
  // we don't care about clear here as the data will be received one cycle
  // later anyway
  assign in_ready_o = ~valid_Q[DEPTH-2];


  //////////////////////////////////////////////////////////////////////////////
  // FIFO management
  //////////////////////////////////////////////////////////////////////////////
	integer i,j;
  always @(*)
  begin
	for (i=0;i<DEPTH;i=i+1)
	begin
		addr_int[i]    = addr_Q[i];
		rdata_int[i]   = rdata_Q[i];
		valid_int[i]   = valid_Q[i];
	end
	is_hwlp_int[0] = is_hwlp_Q[0];
	is_hwlp_int[1] = is_hwlp_Q[1];
    if (in_valid_i) begin
      
        if (~valid_Q[0]) begin
          addr_int[0]  = in_addr_i;
          rdata_int[0] = in_rdata_i;
          valid_int[0] = 1'b1;
        end
		else if (~valid_Q[1]) begin
          addr_int[1]  = in_addr_i;
          rdata_int[1] = in_rdata_i;
          valid_int[1] = 1'b1;
        end
		else if (~valid_Q[2]) begin
          addr_int[2]  = in_addr_i;
          rdata_int[2] = in_rdata_i;
          valid_int[2] = 1'b1;
        end
		else if (~valid_Q[3]) begin
          addr_int[3]  = in_addr_i;
          rdata_int[3] = in_rdata_i;
          valid_int[3] = 1'b1;
        end

      // replace 2nd entry
      if (in_replace2_i) begin
        if (valid_Q[0]) begin
          addr_int[1]          = in_addr_i;

          // if we replace the 2nd entry, let's cache the output word in case we
          // still need it and it would span two words in the FIFO
          rdata_int[0]         = out_rdata_o;
          rdata_int[1]         = in_rdata_i;
          valid_int[1]         = 1'b1;
          valid_int[2:DEPTH-1] = 0;

          // hardware loop incoming?
          is_hwlp_int[1] = in_is_hwlp_i;
        end else begin
          is_hwlp_int[0] = in_is_hwlp_i;
        end
      end
    end
  end

  assign addr_next = {addr_int[0][31:2], 2'b00} + 32'h4;

  // move everything by one step
  always @(*)
  begin
    for (i=0;i<DEPTH;i=i+1)
	begin
		addr_n[i]    = addr_int[i];
		rdata_n[i]   = rdata_int[i];
		valid_n[i]   = valid_int[i];
		
	end
	is_hwlp_n[0] = is_hwlp_int[0];
	is_hwlp_n[1] = is_hwlp_int[1];
   
	if (out_ready_i && out_valid_o) begin
      is_hwlp_n = {is_hwlp_int[1], 1'b0};

      if (is_hwlp_int[1]) begin
        addr_n[0] = addr_int[1][31:0];
        for (i = 0; i < DEPTH - 1; i=i+1)
        begin
          rdata_n[i] = rdata_int[i + 1];
        end
        rdata_n[DEPTH - 1] = 32'b0;
        valid_n   = {valid_int[1:DEPTH-1], 1'b0};
      end else begin
        if (addr_int[0][1]) begin
          // unaligned case
          if (unaligned_is_compressed) begin
            addr_n[0] = {addr_next[31:2], 2'b00};
          end else begin
            addr_n[0] = {addr_next[31:2], 2'b10};
          end
          for (i = 0; i < DEPTH - 1; i=i+1)
          begin
            rdata_n[i] = rdata_int[i + 1];
          end
          rdata_n[DEPTH - 1] = 32'b0;
          valid_n  = {valid_int[1:DEPTH-1], 1'b0};
        end else begin
          // aligned case
          if (aligned_is_compressed) begin
            // just increase address, do not move to next entry in FIFO
            addr_n[0] = {addr_int[0][31:2], 2'b10};
          end else begin
            // move to next entry in FIFO
            addr_n[0] = {addr_next[31:2], 2'b00};
            for (i = 0; i < DEPTH - 1; i=i+1)
            begin
              rdata_n[i] = rdata_int[i + 1];
            end
            rdata_n[DEPTH - 1] = 32'b0;
            valid_n   = {valid_int[1:DEPTH-1], 1'b0};
          end
        end
      end
    end
  end

  //////////////////////////////////////////////////////////////////////////////
  // registers
  //////////////////////////////////////////////////////////////////////////////

  always @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
		for (i=0;i<DEPTH;i=i+1)
		begin
			addr_Q[i]    <= 0;
			rdata_Q[i]   <= 0;
			valid_Q[i]   <= 0;			
		end
		is_hwlp_Q[0] <= 0;
		is_hwlp_Q[1] <= 0;
    end
    else
    begin
      // on a clear signal from outside we invalidate the content of the FIFO
      // completely and start from an empty state
      if (clear_i) begin
        valid_Q    <= 0;
        is_hwlp_Q  <= 0;
      end else begin
		for (i=0;i<DEPTH;i=i+1)
		begin
			addr_Q[i]    <= addr_n[i];
			rdata_Q[i]   <= rdata_n[i];
			valid_Q[i]   <= valid_n[i];
		end
		is_hwlp_Q[0] <= is_hwlp_n[0];
		is_hwlp_Q[1] <= is_hwlp_n[1];
      end
    end
  end

  //----------------------------------------------------------------------------
  // Assertions
  //----------------------------------------------------------------------------

  // check for FIFO overflows
  // assert property (
  //   @(posedge clk) (in_valid_i) |-> ((valid_Q[DEPTH-1] == 1'b0) || (clear_i == 1'b1) || (in_replace2_i == 1'b1)) );

endmodule
