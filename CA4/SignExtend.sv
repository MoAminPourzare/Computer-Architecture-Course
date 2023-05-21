module Sign_extend ( input [15:0] In , output reg [31:0] Out);
	
	integer i;
	always @ (*)
	begin 
		for ( i=16 ; i<32 ; i = i+1 )
		begin 
			Out [i] = In[15];
			Out [i-16] = In[i-16];
		end
	end

endmodule