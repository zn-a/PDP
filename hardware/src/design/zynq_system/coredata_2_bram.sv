module coredata_2_bram #(
    parameter R_LATENCY_IN_CYCLES	= 1
)(
    input logic                       clk_i,
    input logic                       rst_ni,

    input  logic                      req_i,
    output logic                      gnt_o,
    output logic                      rvalid_o,
    input  logic  [31:0]              addr_i,
    input  logic                      we_i,
    input  logic  [3:0]               be_i,
    output logic  [31:0]              rdata_o,
    input  logic  [31:0]              wdata_i,

	  output  logic [31:0]              addr,
    output  logic [31:0]              dout,
    input   logic [31:0]              din,
    output  logic [3:0]               weout
	
);

  localparam IDLE=0, READ_WAIT=1, WRITE_WAIT=2;
  logic 	[1:0]  	state, next_state;
  logic           rvalid;
  logic   [7:0]   read_latency_cntr; 

  //ASSIGNS
  assign gnt_o 		= req_i;
  assign rdata_o	= we_i ? 0 : din;
  assign addr     = addr_i;
  assign dout	    = we_i ? wdata_i : '0;
  
  // STATE
  always_ff @(posedge clk_i)begin
	  if (!rst_ni)
		  state <= IDLE;
	  else
		  state <= next_state;
  end

  // always_ff @(posedge clk_i) begin
  //     addr <= addr_i;
  //     dout <= we_i ? wdata_i : '0;
  // end

  // NS of FSM
  always_comb begin
	case (state)
		IDLE: 		next_state = req_i 	? (we_i  ? WRITE_WAIT : READ_WAIT) 	: IDLE;
		READ_WAIT:	next_state = rvalid	? (req_i ? (we_i ? WRITE_WAIT  : READ_WAIT) : IDLE)	: READ_WAIT;
		WRITE_WAIT: next_state = req_i	? (we_i ? WRITE_WAIT  : READ_WAIT)	: IDLE;
		default:	next_state = IDLE;
	endcase  
  end
	// out of FSM
  always_comb
  begin
	weout = (req_i & we_i) ? be_i : '0;
	rvalid_o = '0;
    case (state)
      IDLE: begin
        //weout = '0;
	      rvalid_o = '0;
	    end
	    READ_WAIT: begin     
        //weout    = '0;
	      rvalid_o = rvalid; //modify
      end
      WRITE_WAIT:
      begin
        //weout    = be_i;
		    rvalid_o = 1'b1;
      end
      endcase
  end

  // Read latency counter
  generate
  if (R_LATENCY_IN_CYCLES > 1) begin
	// always_ff @(posedge clk_i or negedge rst_ni) begin
	//   if (!rst_ni) begin
	// 	read_latency_cntr <= R_LATENCY_IN_CYCLES-1;
	//   end else begin
	// 	if (state == READ_WAIT) begin
    //         if (read_latency_cntr != 1) begin
	// 			read_latency_cntr <= read_latency_cntr - 1;
	// 		end else begin
	// 		end
	// 	end else begin
    //         read_latency_cntr <= R_LATENCY_IN_CYCLES-1;
	// 	end
	//   end
	// end
	// TO BE IMPLEMENTED
	    assign rvalid = 1'b1;
  end else begin
      assign rvalid = 1'b1;
  end
  endgenerate
  
 endmodule
 