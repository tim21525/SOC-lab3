`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  reg                      awready,
    output  reg                      wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
	
    output  reg                      arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  reg                      rvalid,
    output  reg  [(pDATA_WIDTH-1):0] rdata,   
	
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 			//32bits
    input   wire                     ss_tlast, 
    output  reg                      ss_tready, 
	
    input   wire                     sm_tready, 
    output  reg                      sm_tvalid, 
    output  reg  [(pDATA_WIDTH-1):0] sm_tdata, 			
    output  reg                      sm_tlast, 
    
    // bram for tap RAM
    output  reg [3:0]               tap_WE,
    output  wire                    tap_EN,
    output  reg [(pDATA_WIDTH-1):0] tap_Di,
    output  reg [(pADDR_WIDTH-1):0] tap_A,
    input       [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  reg [3:0]               data_WE,
    output  wire                    data_EN,
    output  reg [(pDATA_WIDTH-1):0] data_Di,
    output  reg [(pADDR_WIDTH-1):0] data_A,       //12
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

localparam TAP    = 3'd0;
localparam READ   = 3'd1;
localparam R_BRAM = 3'd2;
localparam W_BRAM = 3'd3;
localparam ADD    = 3'd4;
localparam OUTPUT = 3'd5;


reg ap_start;
reg ap_done;
reg ap_idle;
reg [31:0] count;
reg [2:0] state,next_state;

reg [4:0] N;
reg [3:0] loop;
reg [31:0] data_temp;
reg [31:0] read_bramdata;
reg [31:0] acc;
reg [31:0] h;

reg [11:0] data_length;
reg [11:0] tap_A_mem;
reg [63:0] product;


always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) state <= TAP;
	else            state <= next_state;
end

always@(*) begin
	next_state = state;
	case(state)
		TAP : if(ap_start==1'd1)                       next_state = READ;
		READ: if(ss_tvalid==1'd1 && ss_tready==1'd1)   next_state = R_BRAM;
		R_BRAM:                                        next_state = W_BRAM;
		W_BRAM:                                        next_state = ADD;
		ADD:  	if(loop == 4'd0)                       next_state = OUTPUT;
				else                                   next_state = R_BRAM;
		OUTPUT: if(sm_tready==1'd1 && sm_tvalid==1'd1) next_state = READ;                   
	endcase
end


//--------------	AXI read	---------------------//

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		awready <= 1'd0;
	end
	else begin
		if(awvalid==1'd1 && awready==1'd0 && wready==1'd0) awready <= 1'd1;
		else if(awvalid==1'd1 && awready==1'd1) 		   awready <= 1'd0;
		else                                               awready <= awready;
	end
end
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		wready <= 1'd0;
	end
	else begin
		if(awvalid==1'd1 && awready==1'd1)      wready <= 1'd1;
		else if(wvalid==1'd1 && wready == 1'd1) wready <= 1'd0;
		else                                    wready <= wready;
	end
end

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		data_length <= 12'd0;
	end
	else begin
		if(awaddr==12'h10) begin
			data_length <= wdata;
		end
		else begin
			data_length <= data_length;
		end
	end
end

//--------------	AXI write	---------------------//

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		arready <= 1'd0;
	end
	else begin
		if(arvalid==1'd1 && arready==1'd0 && rvalid==1'd0) arready <= 1'd1;
		else if(arvalid==1'd1 && arready==1'd1)            arready <= 1'd0;
		else                                               arready <= arready;
	end
end
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		rvalid <= 1'd0;
	end
	else begin
		if(arready==1'd1 && arvalid==1'd1)    rvalid <= 1'd1;
		else if(rready==1'd1 && rvalid==1'd1) rvalid <= 1'd0;
		else                                  rvalid <= rvalid;
	end
end

always@(*) begin
	case(state)
	TAP: begin
		if(rvalid==1'd1 && rready==1'd1) rdata = tap_Do;
		else 							 rdata = 32'h0;
	end
	default: begin
		if(rvalid==1'd1 && rready==1'd1 && awaddr==12'h0) begin
			if     (ap_done==1'd1 && ap_idle==1'd1) rdata = 32'h6;
			else if(ap_done==1'd1 && ap_idle==1'd0) rdata = 32'h2;
			else if(ap_done==1'd0 && ap_idle==1'd1) rdata = 32'h4;
			else                                    rdata = 32'h0;
		end
		else begin
			rdata = 32'h0;
		end
	end
	endcase
end

//--------------	ap control	---------------------//

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		ap_start <= 1'd0;
	end
	else begin
		case(state)
		TAP: begin
			if(awaddr==12'h00 && wdata==32'h0000_0001) ap_start <= 1'd1;
			else 									   ap_start <= 1'd0;
		end
		default: begin
			ap_start <= 1'd0;
		end
		endcase
	end
end


always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		ap_idle <= 1'd1;
	end
	else begin
		if(ap_start==1'd1)      ap_idle <= 1'd0;
		else if(ss_tlast==1'd1) ap_idle <= 1'd1;
		else                    ap_idle <= ap_idle;
	end
end

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		ap_done <= 1'd0;
	end
	else begin
		if(sm_tready==1'd1 && sm_tvalid==1'd1) begin
			if(count+1'd1==data_length) ap_done <= 1'd1;
		end
		else if(awaddr==12'h0 && (rdata == 32'h2 || rdata == 32'h6)) begin
			ap_done <= 1'd0;
		end
		else begin
			ap_done <= ap_done;
		end
	end
end

//--------------	tap BRAM11	---------------------//

assign tap_EN = axis_rst_n;

always@(*) begin
	case(state)
	TAP: begin
		if(awaddr>=12'h20 && awvalid==1'd1 && awready==1'd1)      tap_A = awaddr-12'h20;
		else if(awaddr>=12'h20 && arvalid==1'd1 && arready==1'd1) tap_A = araddr-12'h20;
		else                                                      tap_A = tap_A_mem;
	end
	R_BRAM: begin
		tap_A = loop << 2;
	end
	default: begin
		tap_A = 12'h0;
	end
	endcase
end

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		tap_A_mem <= 12'h0;
	end
	else begin
		case(state)	
		TAP: begin
			if(awaddr>=12'h20 && awvalid==1'd1 && awready==1'd1)      tap_A_mem <= tap_A;
			else if(awaddr>=12'h20 && arvalid==1'd1 && arready==1'd1) tap_A_mem <= tap_A;
			else                                                      tap_A_mem <= 12'h0;
		end
		default: begin
			tap_A_mem <= 12'h0;
		end
		endcase
	end
end

always@(*) begin
	case(state)
	TAP: begin
		if(wvalid==1'd1 && wready==1'd1)   tap_Di = wdata;
		else 							   tap_Di = 32'd0; 
	end
	default: begin
		tap_Di = 32'd0; 
	end
	endcase
end

always@(*) begin
	case(state)
	TAP: begin
		if(wvalid==1'd1 && wready==1'd1)   tap_WE = 4'b1111;
		else 							   tap_WE = 4'b0000;
	end
	default: begin
		tap_WE = 4'b0000;
	end
	endcase
end


/*always@(*) begin
	case(state)
	W_BRAM: begin
		h = tap_Do;
	end
	default: begin
		h = 32'd0;
	end
	endcase

end*/


//---------------------------------------------------//
//                  ss stream                        //
//---------------------------------------------------//

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		ss_tready <= 1'd0;
	end
	else begin
		case(state)
		READ: begin
			if(ss_tvalid==1'd1 && ss_tready==1'd1) begin
				ss_tready <= 1'd0;
			end
			else if(ss_tvalid==1'd1 && ss_tready==1'd0) begin
				ss_tready <= 1'd1;
			end
		end
		R_BRAM: begin
			ss_tready <= 1'd0;
		end
		default: begin
			ss_tready <= 1'd0;
		end
		endcase
	end
end

//--------------	data BRAM11	---------------------//

assign data_EN = axis_rst_n;

always@(*) begin
	case(state)
	W_BRAM: begin
		//data_Di = read_bramdata;
		data_Di = data_Do;
	end
	ADD: begin
		data_Di = data_temp;
	end
	default: begin
		data_Di = 32'd0;
	end
	endcase
end

always@(*) begin
	case(state)
	W_BRAM: begin
		read_bramdata = data_Do;
	end
	default: begin
		read_bramdata = 32'd0;
	end
	endcase
end

always@(*) begin
	case(state)
	R_BRAM: begin
		if(loop != 4'd0) data_A = (loop - 4'd1) << 2;
		else             data_A = 12'd0;
	end
	W_BRAM: begin
		data_A = loop << 2;
	end
	ADD: begin
		data_A = 12'h0;
	end
	default: begin
		data_A = 12'd0;
	end
	endcase
end

always@(*) begin
	case(state)
	R_BRAM: begin
		data_WE = 4'b0000;
	end
	W_BRAM: begin
		data_WE = 4'b1111;
	end
	ADD: begin
		if(loop==4'd1) data_WE = 4'b1111;
		else           data_WE = 4'b0000;
	end
	default: begin
		data_WE = 4'b0000;
	end
	endcase
end




//--------------	fir stream	---------------------//

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		data_temp <= 32'd0;
	end
	else begin
		case(state)
		READ: begin
			if(ss_tvalid==1'd1 && ss_tready==1'd1) 
				data_temp <= ss_tdata;
		end
		default: begin
			data_temp <= data_temp;
		end
		endcase
	end
end


always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		product <= 64'd0; 
	end
	else begin
		case(state)
		READ: begin
			product <= 64'd0;
		end
		W_BRAM: begin
			product <= read_bramdata * tap_Do;
			//product <= data_Do * tap_Do;
		end
		default: begin
			product <= product;
		end
		endcase
	end
end

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		acc <= 32'd0; 
	end
	else begin
		case(state)
		READ: begin
			acc <= 32'd0;
		end
		ADD: begin
			acc <= acc + product;
		end
		default: begin
			acc <= acc;
		end
		endcase
	end
end


always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		loop <= 4'd0;
	end
	else begin
		case(state)
		ADD: begin
			if(loop==4'd0) loop <= 4'd0;
			else           loop <= loop - 4'd1;
		end
		OUTPUT: begin
			loop <= N;
		end
		default: begin
			loop <= loop;
		end
		endcase
	end
end

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		N <= 5'd0;
	end
	else begin
		case(state)
		READ: begin
			if(ss_tvalid==1'd1 && ss_tready==1'd1) begin
				if(N == 5'd10) N <= 5'd10;
				else           N <= N + 5'd1;
			end
		end
		default: begin
			N <= N;
		end
		endcase
	end
end

//--------------	sm stream	---------------------//

always@(*) begin
	case(state)
	OUTPUT: begin
		if(sm_tready==1'd1 && sm_tvalid==1'd1) begin
			sm_tdata = acc;
		end
		else begin
			sm_tdata = 32'd0;
		end
	end
	default: begin
		sm_tdata = 32'd0;
	end
	endcase
end

always@(*) begin
	case(state)
	OUTPUT: begin
		if(sm_tready==1'd1 && sm_tvalid==1'd1 && count+1'd1 == data_length) begin
			sm_tlast = 1'd1;
		end
		else begin
			sm_tlast = 1'd0;
		end
	end
	default: begin
		sm_tlast = 1'd0;
	end
	endcase
end


always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		sm_tvalid <= 1'd0;
	end
	else begin
		case(state)
		ADD: begin
			if(loop==4'd0) sm_tvalid <= 1'd1;
			else           sm_tvalid <= 1'd0;
		end
		OUTPUT: begin
			if(sm_tready==1'd1) begin
				sm_tvalid <= 1'd0;
			end
		end
		default: begin
			sm_tvalid <= sm_tvalid;
		end
		endcase
	end
end

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(~axis_rst_n) begin
		count <= 32'd0;
	end
	else begin
		case(state)
		OUTPUT: begin
			if(sm_tready==1'd1 && sm_tvalid==1'd1) count <= count + 1'd1;
		end
		default: begin
			if(ap_start==1'd1) begin
				count <= 32'd0;
			end
		end
		endcase
	end
end


endmodule



















