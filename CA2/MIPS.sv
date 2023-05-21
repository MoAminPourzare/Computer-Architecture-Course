`timescale 1ns/1ns
module MIPS(input clk, rst, input reg [31:0] instMem[0:1023]);
	wire Is_SV, MemtoReg, Reg_Write, Mem_Read, Mem_Write ,  Branch, ALUsrc, Jump, JrSel;
	wire [1:0] regDst, writeDst;
	wire [2:0] ALUOp;
	wire [3:0] ALUOperation;
	wire [5:0] func, opcode;

	Data_path DP(clk , rst  , MemtoReg , Reg_Write , Mem_Read, Mem_Write , Branch , ALUsrc ,  JrSel , Jump , regDst ,   writeDst , ALUOperation, func, opcode, instMem);
	controller CU(opcode , MemtoReg , Reg_Write , Mem_Read ,Mem_Write ,  Branch , ALUsrc , JrSel , Jump , regDst ,  writeDst , ALUOp);
	ALU_control ALU_CU(func , ALUOp , ALUOperation);
endmodule
