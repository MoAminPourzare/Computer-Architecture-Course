module address_inc( input [31:0] In1 , In2 , output [31:0] adr_inc_out);
  
	assign adr_inc_out = In1 + In2;

endmodule	