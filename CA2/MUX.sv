/* Mux Utilities */
module Mux2to1_32bit( input [31:0] In1 , In2 ,input Sel , output [31:0]Out);
	

	assign Out = (Sel == 1'b0) ? In1 : 
	             (Sel == 1'b1) ? In2 : 32'bx;

endmodule

module Mux3to1_5bit( input [4:0] In1 , In2, In3 , input [1:0]Sel , output [4:0]Out);
	

	assign Out = (Sel == 2'b00) ? In1 : 
	             (Sel == 2'b01) ? In2 : 
	             (Sel == 2'b10) ? In3 : 5'bx;

endmodule



module Mux3to1_32bit( input [31:0] In1 , In2, In3 , input [1:0]Sel , output [31:0]Out);
	

	assign Out = (Sel == 2'b00) ? In1 : 
	             (Sel == 2'b01) ? In2 : 
	             (Sel == 2'b10) ? In3 : 32'bx;

endmodule

module Mux4to1_32bit( input [31:0] In1 , In2, In3 , In4 , input [1:0]Sel , output [31:0]Out);
	

	assign Out = (Sel == 2'b00) ? In1 : 
	             (Sel == 2'b01) ? In2 : 
	             (Sel == 2'b10) ? In3 : 
	             (Sel == 2'b11) ? In4 : 32'bx;
endmodule
