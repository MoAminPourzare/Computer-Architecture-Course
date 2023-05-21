`timescale 1ns/1ns
module Register_file ( input clk, reset , input [4:0]  Read_reg2 , write_reg ,  input [31:0] write_data , input reg_write ,output [31:0] Read_data1 ,Read_data2 );


	reg [31:0] Reg_data [0:31];
	logic [4:0] Read_reg1 = 4'b0;
 	wire [31:0] test;
	assign test = Reg_data[9];
	initial begin
		Reg_data[0] = 32'b0;
	end
  
	integer i;

	assign Read_data1 = Reg_data[Read_reg1];
	assign Read_data2 = Reg_data[Read_reg2];

	always @ (posedge clk)
	begin 
		if ( reset )
			begin
				for ( i=0 ; i < 32 ; i = i+1)
					Reg_data [i] <= 0;
			end
		else 
			begin 
				if ( reg_write )
					if ( write_reg != 5'b0 )
						Reg_data[write_reg] = write_data;
					else
					  Reg_data[write_reg] = Reg_data[write_reg];
				
			end
	end

	
endmodule