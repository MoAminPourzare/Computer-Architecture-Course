module controller (input [5:0] opcode , output reg MemtoReg , Reg_Write , Mem_Read , Mem_Write ,  Branch , ALUsrc , JrSel , Jump ,output reg [1:0] regDst ,  writeDst , output reg  [2:0] ALUOp);

	// opc
	parameter [5:0] R_TYPE = 6'b000000;
	parameter [5:0] ADD_IMEDIATE = 6'b000001; // addi
	parameter [5:0] SLT_IMEDIATE = 6'b000010; // slti
	parameter [5:0] LOAD_WORD = 6'b000011; // ld
	parameter [5:0] SAVE_WORD = 6'b000100; // sw
	parameter [5:0] BRANCH_EQUAL = 6'b000101; // beq
	parameter [5:0] JUMP = 6'b000110; // j
	parameter [5:0] JUMP_REG = 6'b000111; // jr
	parameter [5:0] JUMP_AND_LINK = 6'b001000; // jal

	always @ (*)
	begin 
		{MemtoReg , Reg_Write , Mem_Read , Branch , ALUsrc , regDst , writeDst , Mem_Write ,  JrSel , Jump , ALUOp} = 15'b0;
		Mem_Write = 1'b0;
		case (opcode)
		R_TYPE: begin //R_type
			{regDst, Reg_Write, ALUsrc, MemtoReg, writeDst, JrSel, Jump} = 9'b011000000;
			ALUOp = 3'b101;
			
		end

		ADD_IMEDIATE: begin //addi
			{regDst, Reg_Write, ALUsrc, MemtoReg, writeDst, JrSel, Jump} = 9'b001100000;
			ALUOp = 3'b000;
		end
			
		SLT_IMEDIATE: begin //slti
			{regDst, Reg_Write, ALUsrc, MemtoReg, writeDst, JrSel, Jump} = 9'b001100000;
			ALUOp = 3'b011;
		end	
			
		LOAD_WORD: begin //lw
			{regDst, MemtoReg, Reg_Write, Mem_Read, ALUsrc, writeDst, JrSel, Jump} = 10'b0011110000;
			ALUOp = 3'b000;
			end	
				 
		SAVE_WORD:begin //sw
			{ALUsrc, Mem_Write ,JrSel, Jump} = 4'b1100;
			ALUOp = 3'b000;
			end
			
		BRANCH_EQUAL:begin //beq
			{Branch, JrSel, Jump} = 3'b100;
			ALUOp = 3'b001;
		end

			
		JUMP:begin //jump
			{JrSel, Jump} = 2'b01;
			end	
			
		JUMP_AND_LINK:begin //jal
			{Reg_Write, regDst, writeDst, JrSel, Jump} = 7'b1100101;
			end
			
		JUMP_REG:begin //jr
			{JrSel, Jump} = 2'b11;
			end		
		endcase
	end

endmodule
