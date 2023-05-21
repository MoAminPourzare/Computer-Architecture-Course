`timescale 1ns/1ns
module Data_path (input clk , reset , MemtoReg , Reg_Write , Mem_Read, Mem_Write  , Branch , ALUsrc, JrSel, Jump ,input [1:0] regDst , writeDst , input[3:0] ALUOperation, output [5:0]func, opcode, input reg [31:0] instMem[0:1023]);
  wire [31:0] adrToPC, PC_out, PC_inc_out, Extend_data, instruction, WritetoReg_data,
  write_data, Read_data1, Read_data2, Operand_data2, branch_data
  , Mem_read_value , shifted_ext , adr_inc_out , concated , JOut;
  wire [31:0] ALU_result;
  wire Zero;
  wire [4:0] write_reg;
  wire [27:0] shifted_inst;

  PC CUT_PC(clk , reset, adrToPC , PC_out);
  
  PC_inc CUT_PCinc(PC_out, PC_inc_out);
  
  Sign_extend CUT_SE(instruction[15:0] , Extend_data);
 
  Instruction_memory CUT_InstrcMem(instMem, PC_out , instruction);
 
  Mux3to1_32bit CUT_mux4_32( WritetoReg_data , PC_inc_out, ALU_result , writeDst , write_data);
 
  Mux3to1_5bit CUT_mux3_5( instruction[20:16] , instruction[15:11], 5'b11111, regDst , write_reg);
 
  Register_file CUT_RF(clk , reset ,  instruction[25:21] , instruction[20:16] , write_reg , write_data , Reg_Write , Read_data1 ,Read_data2);
 
  Mux2to1_32bit CUT_mux2_32( Read_data2 ,Extend_data , ALUsrc , Operand_data2);
 
  Shl2_32 CUT_shl2(Extend_data , shifted_ext);
 
  address_inc CUT_inc_adr( PC_inc_out, shifted_ext , adr_inc_out);
  
  assign can_branch = Zero & Branch;
  
  Mux2to1_32bit CUT3_mux2_32( PC_inc_out , adr_inc_out , can_branch , branch_data);
  
  Shl2_26 CUT2_shl2( instruction[25:0] , shifted_inst);
  
  assign concated = {PC_inc_out[31:28], shifted_inst};

  Mux2to1_32bit m4(concated, Read_data1, JrSel, JOut);
	
  Mux2to1_32bit m5( branch_data , JOut , Jump , adrToPC);
 
  ALU_32bit CUT_ALU(Read_data1, Operand_data2 , ALUOperation , ALU_result, Zero);

  Data_Memory CUT_DataMem(clk,ALU_result, Read_data2 ,  Mem_Read , Mem_Write , Mem_read_value) ;
  
  Mux2to1_32bit CUT2_mux2_32( ALU_result , Mem_read_value ,MemtoReg , WritetoReg_data);
  
  assign func = instruction[5:0];
  assign opcode = instruction[31:26];
  
 endmodule
