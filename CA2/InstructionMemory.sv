module Instruction_memory (input reg[31:0] instructionMemory[0:1023], input [31:0] address , output reg [31:0] instruction);
	always @(*) instruction = instructionMemory[address];
endmodule 