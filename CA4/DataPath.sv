`timescale 1ns/1ns
module Data_path (input clk , reset , MemtoReg , Reg_Write , Mem_Read, Mem_Write  , Branch , ALUsrc, JrSel, Jump ,input [1:0] regDst , writeDst , input[3:0] ALUOperation, output [5:0]func, opcode, input reg [31:0] instMem[0:1023]);
  wire [15:0] adrToPC, PC_out, PC_inc_out, Extend_data, instruction, WritetoReg_data,
  write_data, read_R0, Read_data2, Operand_data2, branch_data
  , Mem_read_value , shifted_ext , adr_inc_out , concated , JOut,inpmux2,inpmux3,read_r0_reg;
  wire [31:0] ALU_result;
  wire Zero;
  wire [3:0] write_reg;
  wire [27:0] shifted_inst;

  PC CUT_PC(clk , reset, adrToPC , PC_out);
  
  PC_inc CUT_PCinc(PC_out, PC_inc_out);

  REG16 IRreg( rst, clk, ld, Mem_read_value, instruction);

  REG16 Dreg( rst, clk, ld, Mem_read_value, inpmux2);
  
  Sign_extend CUT_SE(instruction[11:0] , Extend_data);
 
  Mux3to1_12bit CUT_mux4_12( WritetoReg_data , PC_inc_out, ALU_result , writeDst , write_data);
 
  Mux3to1_5bit CUT_mux3_4( instruction[11:8] , instruction[8:5], 5'b11111, regDst , write_reg);
 
  Register_file CUT_RF(clk , reset ,  instruction[11:9] , write_reg , write_data , Reg_Write , read_R0 ,Read_data2);
 
  REG16 R0( rst, clk, ld, read_R0, read_r0_reg);

  REG16 Ri( rst, clk, ld, Mem_read_value, inpmux2);

  Mux2to1_12bit CUT_mux2_12( Read_data2 ,Extend_data , ALUsrc , Operand_data2);
 
  Shl2_12 CUT_shl2(Extend_data , shifted_ext);
 
  address_inc CUT_inc_adr( PC_inc_out, shifted_ext , adr_inc_out);
  
  assign can_branch = Zero & Branch;
  
  Mux2to1_12bit CUT3_mux2_12( PC_inc_out , adr_inc_out , can_branch , branch_data);
  
  Shl2_26 CUT2_shl2( instruction[11:0] , shifted_inst);
  
  assign concated = {PC_inc_out[10:0], shifted_inst};

  Mux2to1_12bit m4(concated, read_R0, JrSel, JOut);
	
  Mux2to1_12bit m5( branch_data , JOut , Jump , adrToPC);
 
  ALU_16bit CUT_ALU(read_R0, Operand_data2 , ALUOperation , ALU_result, Zero);

  REG16 ALUregout( rst, clk, ld, ALU_result, inpmux3);

  Memory CUT_Mem(clk,ALU_result, Read_data2 ,  Mem_Read , Mem_Write , Mem_read_value) ;
  
  Mux2to1_32bit CUT2_mux2_32( ALU_result , Mem_read_value ,MemtoReg , WritetoReg_data);
  
  assign func = instruction[8:0];
  assign opcode = instruction[11:8];
  
 endmodule
