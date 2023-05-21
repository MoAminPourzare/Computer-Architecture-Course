module ALU_control(input [5:0] func , input [2:0] ALUop , output reg [3:0] ALUoperation);  
  
always @ (*)
	begin 
	  case(ALUop)
	    
	    0: ALUoperation = 4'b0000; //sw , lw , addi
	    1: ALUoperation = 4'b0111; //beq
	    2: ALUoperation = 4'b1000; //bne
	    3: ALUoperation = 4'b0110; //slti
	    4: ALUoperation = 4'b0010; //mult
	    5: case(func)
	         
	         0: ALUoperation = 4'b0000; //add
	         1: ALUoperation = 4'b0001; //sub
	         2: ALUoperation = 4'b0011; //and
	         3: ALUoperation = 4'b0100; //or
	         4: ALUoperation = 4'b0101; //xor
	         5: ALUoperation = 4'b0110; //slt
	     
	         endcase
	         
	     endcase
	         
	    
  end
  
endmodule