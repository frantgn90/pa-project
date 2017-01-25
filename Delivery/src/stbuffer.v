
`ifndef _stbuffer
`define _stbuffer

`include "define.v"


//TO connect;
//need to declare STBUFF_DEPTH that is stbuff size in bytes

module stbuffer(
	input wire        clk,
	input wire        reset,
	input wire [`ADDR_SIZE:0] addr_in,			//Address to read/check/writetomemory
	input wire [`WIDTH:0] data_in,			//Data to read/check/writetomemory
	input wire        is_byte,			//Data to write/read from stbuff is_byte
	input wire        is_store,			
	input wire        is_load,		
	input wire        dc_hit,

	output reg [`ADDR_SIZE:0] addr_out = 0,
	output reg [`WIDTH:0] data_out = 0,
	output reg        is_byte_out = 0,
	output reg        dc_store = 0,

	output reg        stb_hit = 0,
	output reg        stb_full = 0
);

//Keep in mind that Store Buffer's depth is "STBUFF_DEPTH"-1 due to pointers!!
parameter ENTRIES = `STBUFF_DEPTH;
localparam ENTRYBITS = $clog2(ENTRIES);

// Pointers to the newest and oldest entry beg_p -> oldest entry
reg [ENTRYBITS-1:0] beg_p = 0, end_p = 0;
// Temporal pointer
reg [ENTRYBITS-1:0] temp = 0;

// Store buffer entry model :: { address || data || type }
reg [`ADDR_SIZE:0] stb_addr [ENTRIES-1:0];
reg [`WIDTH:0] stb_data [ENTRIES-1:0];
reg        stb_is_byte [ENTRIES-1:0];
integer i = 0;
integer len = 0;
reg stb_empty = 1;

always @(posedge clk) begin
	if (reset) begin
		beg_p <= 0;
		end_p <= 0;
		len <= 0;
		stb_empty <= 1;
		for (i=0; i<ENTRIES; i=i+1) begin
			stb_addr[i] <= {32{1'b0}};
			stb_data[i] <= {32{1'b0}};
			stb_is_byte[i] <= 1'b0;
		end 
	else begin
		if(is_load) begin
			dc_store  = 0;
			if (stb_empty) begin
				stb_hit = 0;
				data_out = {32{1'b0}};
				addr_out = {32{1'b0}};
				is_byte_out = 1'b0;
			end else begin
				stb_hit = 0;
				i = 0;
				temp = (end_p+ENTRIES-1)%ENTRIES;
				while ((beg_p+ENTRIES-1)%ENTRIES != temp && !stb_hit) begin
					if (stb_addr[temp] == addr_in) begin
						data_out = stb_data[temp];
						addr_out = addr_in;
						type_out = type_in;
						`INFO(("[stb] i have the data %x %x %d", addr_out, data_out, temp))
						dc_enable = 0;
						stb_hit = 1;
					end
					i=i+1;
					// Go backwards
					temp=(temp+ENTRIES-1)%ENTRIES;
				end
				if (!stb_hit) begin
					// Clean up
					data_out = {32{1'b0}};
					addr_out = {32{1'b0}};
					type_out = 1'b0;
				end
			end
		end
		// STORE Operation
		// Try to fit the value in the buffer
		// We do NOT update values
		else if (is_store) begin
			dc_store = 0;
			`INFO(("[stb] store %x %x", addr_in, data_in))
			if (beg_p == end_p && !stb_empty) begin // STB is full
				`INFO(("[stb] full"))
				stb_full = 1; // We will stall the CPU here
				// Set data to send to cache
				data_out = stb_data[end_p];
				addr_out = stb_addr[end_p];
				type_out = stb_is_byte[end_p];
				dc_store = 1;
				// Advance pointer
				end_p = (end_p+1) % ENTRIES;
			end else begin // STB not full
				len = len+1;
				stb_empty = 0;
				// Save the data
				stb_data[beg_p] = data_in;
				stb_addr[beg_p] = addr_in;
				stb_is_byte[beg_p] = type_in;
				// Advance pointer
				beg_p = (beg_p+1) % ENTRIES;
			end
		end
		// ALU Operation
		// DCache is idle here, so we can try to store the head
		else begin
			if (stb_empty) begin // We are empty
				data_out <= {32{1'b0}};
				addr_out <= {32{1'b0}};
				type_out <= 1'b0;
				dc_store = 0;
			end else begin
				// Set data to send to cache
				data_out = stb_data[end_p];
				addr_out = stb_addr[end_p];
				type_out = stb_is_byte[end_p];
				`INFO(("[stb] send data %x %x", addr_out, data_out))
				dc_store = 1;
				// Advance pointer
				end_p = (end_p+1) % ENTRIES;
			end
		end
	end
end

endmodule

`endif