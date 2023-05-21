module PC_inc ( input [31:0] PC , output [31:0] PC_inc_out);
  
	assign PC_inc_out = PC + 4;

endmodule