`timescale 1ns/1ns
module PC(input clk , reset,  input [31:0] PC , 	output reg [31:0] PC_out);

	always @(posedge clk) begin
	  
		if (reset)
			PC_out <=32'b0;
		else 
			PC_out <= PC;
	end
	
endmodule