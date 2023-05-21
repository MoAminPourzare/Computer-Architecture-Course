module datapath (input clk, rst, output [31:0] addressInstruction, input  [31:0] inst, output [31:0] address_d, dataOut, input  [31:0] in,input regDst,mToReg, alusrcmain, pcsrc, jRegister, Jump,  input  [2:0] ControllerAlu, input inRegWrite, memWin,output memWout,input inMemoryRead,output outMemoryRead,output EQ,output [31:0] instructionOut,input PCWrite, ifidw, ifidF,input [1:0] aForwarding, bForwarding,output [4:0] idxrt, idxri,output idxreadout,output outMBWRegWrite,output outEXMRegWrite,output [4:0] outEXMDst, output [4:0] outMBWDst );

  wire [4:0]  multiplexer_3, outIdexRD;
  wire [31:0] outPCounter, ifid_instructionOut, outIfIdPC;
  wire [31:0] out1Addr;
  wire [31:0] dataRead1, daraRead2, aOut, bOut, outSGN, aOutMux, bOutMux;
  wire [31:0] mpo;
  wire [31:0] mjo;
  wire [31:0] multiplexer_1;
  wire [31:0] multiplexer_2;
  wire [31:0] sgn_ext_out;
  wire [31:0] mux_outALUSRC;
  wire [31:0] outALU, outALU_EXM, outALU_MBW, inMBW;
  wire [31:0] out2Addr;
  wire [31:0] outShifter2;
  wire [2:0] idex_ControlleroutALU;
  wire outALUSrc, outRegDST, outPCSRC, outMToReg, outRegWrite, MWOut;
  wire outComp, outPCSRC_EXM, outMToReg_EXM, outMToReg_MBW;
  wire outJRegister, outJumb, outJRegister_EXM, outJRegister_MBW, outIFIDF;
  assign addressInstruction = outPCounter;
  assign instructionOut = ifid_instructionOut;
  assign address_d = outALU_EXM;
  mux41 mux1 (bOut, multiplexer_1, outALU_EXM, , bForwarding, bOutMux);  
  mux21 mux12(bOutMux, outSGN, outALUSrc, mux_outALUSRC); 
  mux21b MUX_RegDst(idxrt, outIdexRD, outRegDST, multiplexer_3);
  regnorm PC(mjo, rst, PCWrite, clk, outPCounter);
  Adder ADDER_1 (outPCounter , 32'd4, 1'b0, , out1Addr);
  IFID ifid_reg (clk, ifidF || rst, ifidw, inst, ifid_instructionOut, out1Addr, outIfIdPC);
  mux21 mux13(outALU_MBW, inMBW, outMToReg_MBW, multiplexer_1);
  shl2 SHL2(sgn_ext_out, outShifter2); 
  Adder ADDER_2(outIfIdPC, outShifter2, 1'b0, , out2Addr); 
  mux21 mux14(out1Addr, out2Addr, pcsrc, mpo);
  assign EQ = outComp;
  mux21 MUX_jRegister({outIfIdPC[31:28], ifid_instructionOut[25:0], 2'b00}, dataRead1, jRegister, multiplexer_2);
  mux21 MUX_Jump(mpo, multiplexer_2, Jump, mjo);
  comparator cmp(dataRead1, daraRead2, outComp);
  IDEX idex (clk, rst, dataRead1, aOut, daraRead2, bOut, sgn_ext_out, outSGN, ifid_instructionOut[20:16], idxrt, ifid_instructionOut[15:11], outIdexRD, ifid_instructionOut[25:21], idxri,alusrcmain, outALUSrc, ControllerAlu, idex_ControlleroutALU, regDst, outRegDST,memWin, MWOut, inMemoryRead, idxreadout, pcsrc, outPCSRC,mToReg, outMToReg, inRegWrite, outRegWrite, jRegister, outJRegister, Jump, outJumb, ifidF, outIFIDF);
  mux41 mux1A (aOut, multiplexer_1, outALU_EXM, , aForwarding, aOutMux); 
  regfile  RF(multiplexer_1, ifid_instructionOut[25:21], ifid_instructionOut[20:16], outMBWDst, outMBWRegWrite, rst, clk, dataRead1, daraRead2);
  exmem exmem (clk, rst, outALU, outALU_EXM, bOutMux, dataOut, multiplexer_3, outEXMDst, MWOut, memWout, idxreadout, outMemoryRead, 
  outPCSRC, outPCSRC_EXM, outMToReg, outMToReg_EXM, outRegWrite, outEXMRegWrite, outJRegister, outJRegister_EXM, outJumb, exmem_outJump);
  membw membw (clk, rst, outALU_EXM, outALU_MBW, in, inMBW, outEXMDst, outMBWDst, outMToReg_EXM, outMToReg_MBW, outEXMRegWrite, outMBWRegWrite);
  SE SEdot(ifid_instructionOut[15:0], sgn_ext_out);
  alu ALU(aOutMux, mux_outALUSRC, idex_ControlleroutALU, outALU, );
endmodule
//====================================================================
module Shl21(input [11:0] In , 	output [11:0] Out);
    assign Out = {In[29:0], 2'b00};
endmodule
//====================================================================
module Shl22(input [25:0] In , 	output [27:0] Out);
    assign Out = {In, 2'b00};
endmodule
//====================================================================
module address_inc( input [31:0] In1 , In2 , output [31:0] inAddressc_out);
	assign inAddressc_out = In1 + In2;
endmodule	
//====================================================================
module alu (a, b, ctrl, y, zero);
  input [31:0] a, b;
  input [2:0] ctrl;
  output [31:0] y;
  output zero;
  assign y =  (ctrl == 3'b000) ? (a & b) :
              (ctrl == 3'b001) ? (a | b) : 
              (ctrl == 3'b010) ? (a + b) :
              (ctrl == 3'b110) ? (a - b) :
			  (ctrl == 3'b111) ? (($signed(a) < $signed(b)) ? 32'd1: 32'd0) : 32'bx;
			  //(ctrl == 3'b111) ? ((a < b) ? 32'd1: 32'd0) : 32'bx;
              //(ctrl == 3'b111) ? ((a < b) ? 32'd1: 32'd0) : 32'b0;
  assign zero = (y == 32'd0) ? 1'b1 : 1'b0;
endmodule
//====================================================================
module comparator(A,B,R);
	input [31:0] A,B;
	output R;
	assign R = (A==B)?1:0;
endmodule
//====================================================================
module PC_inccc ( input [31:0] PC , output [31:0] outPC);
	assign outPC = PC + 4;
endmodule
//====================================================================
module data_mem (adr, d_in, mrd, mwr, clk, d_out);
  input [31:0] adr;
  input [31:0] d_in;
  input mrd, mwr, clk;
  output [31:0] d_out;
  reg [7:0] mem[0:65535];
  initial
  begin
  $readmemb("data_mem.bin", mem, 1000, 1079);
  end
  always @(posedge clk)
    if (mwr==1'b1)
      {mem[adr+3], mem[adr+2], mem[adr+1], mem[adr]} = d_in;
  assign d_out = (mrd==1'b1) ? {mem[adr+3], mem[adr+2], mem[adr+1], mem[adr]} : 32'd0; //??
endmodule  
//====================================================================
module exmem ( input clk, input acc,input  [31:0] inALUResult, output [31:0] outALUResult, input  [31:0] bInput, output [31:0] bOutput, input [4:0] inDST, output [4:0] outDST,              input inMemWrite, output outMemWrite, input inMemRead, output outMemRead, input inPCSRC, output outPCSRC,  input inMToReg, output outMToReg, input inRegWrite, output outRegWrite, input inJRegister, output outJRegister, input inJump, output outJump   );  regnorm ALUResult (inALUResult, acc, 1'b1, clk, outALUResult);
  regnorm B (bInput, acc, 1'b1, clk, bOutput);
  reg5 Dst (inDST, acc, 1'b1, clk, outDST);
  f_f MemWrite (inMemWrite, acc, 1'b1, clk, outMemWrite);
  f_f MemRead (inMemRead, acc, 1'b1, clk, outMemRead);
  f_f PCSrc (inPCSRC, acc, 1'b1, clk, outPCSRC);
  f_f MemToReg (inMToReg, acc, 1'b1, clk, outMToReg);
  f_f RegWrite (inRegWrite, acc, 1'b1, clk, outRegWrite);
  f_f jRegister (inJRegister, acc, 1'b1, clk, outJRegister);
  f_f Jump (inJump, acc, 1'b1, clk, outJump);
endmodule
//====================================================================
module f_f (d_in, acc, ld, clk, d_out);
  input d_in;
  input acc, ld, clk;
  output  d_out;
  reg  d_out;
  always @(posedge clk, posedge acc)
  begin
    if (acc==1'b1)
      d_out <= 1'b0;
    else if (ld)
      d_out <= d_in;
  end
endmodule
//====================================================================
module Forwarding 
  (
  aForwarding,
  bForwarding,
  regWriteMBW,
  regWriteEXM,
  rdEXM,
  rdMWB,
  rsIDEX,
  rtIDEX
  );
  output reg [1:0] aForwarding,bForwarding;
  input regWriteMBW, regWriteEXM;
  input [4:0] rdEXM,rdMWB, rsIDEX,rtIDEX;
  always @(*)begin
    {aForwarding,bForwarding} = 4'b0000; 
    if ((regWriteEXM == 1'b1) && (rdEXM == rsIDEX) && (rdEXM != 5'd0))
      aForwarding = 2'b10;
    else if((regWriteMBW == 1'b1) && (rdMWB == rsIDEX) && (rdMWB != 5'd0))
      aForwarding = 2'b01;
      
    if ((regWriteEXM == 1'b1) && (rdEXM == rtIDEX) && (rdEXM != 5'd0))
      bForwarding = 2'b10;
    else if((regWriteMBW == 1'b1) && (rdMWB == rtIDEX) && (rdMWB != 5'd0))
      bForwarding = 2'b01;
    end
endmodule 
//====================================================================
module Adder (in1 , in2, cin, cout, s);
  input [31:0] in1, in2;
  input cin;
  output cout;
  output [31:0] s;
  assign {cout, s} = in1 + in2 + cin;
endmodule
//====================================================================
module Hazard
  (
  memoryReadIDEX,
  rtIDEX,
  rs,
  rt,
  NOP_Sel,
  writeIFID,
  PCWrite, Jump
  );
  output reg NOP_Sel,writeIFID,PCWrite;
  input [4:0] rtIDEX, rs, rt;
  input memoryReadIDEX, Jump;
  always @(*) begin
	{NOP_Sel, PCWrite, writeIFID} = 3'b011;
    if (memoryReadIDEX == 1'b1)
      if ((rtIDEX == rt) || (rtIDEX == rs) && (rtIDEX != 5'd0)) begin
        NOP_Sel = 1'b1;
        PCWrite = 1'b0;
        writeIFID = 1'b0;
      end
  end
endmodule
//====================================================================
module IDEX (input clk,input acc,
              input  [31:0] aInput,output [31:0] aOutput,
              input  [31:0] bInput,output [31:0] bOutput,
              input  [31:0] inAddress,output [31:0] outAddress,input [4:0] inrt ,
              output [4:0] outrt,input [4:0] inRD,output [4:0] outRD,input [4:0] inrs,output [4:0] outrs,
              input inALUSRC, output outALUSRC,input [2:0] inALUOper,output [2:0] outALUOper,input ReginDST, output RegoutDST,  
              input inMemWrite,output outMemWrite,input inMemRead,output outMemRead,input inPCSRC,output outPCSRC,
              input inMToReg,output outMToReg,input inRegWrite,output outRegWrite,input inJRegister,output outJRegister,input inJump,output outJump,input ifidF_in,output ifidF_out
                );  
  f_f MemToReg (inMToReg, acc, 1'b1, clk, outMToReg);
  f_f RegWrite (inRegWrite, acc, 1'b1, clk, outRegWrite);
  f_f jRegister (inJRegister, acc, 1'b1, clk, outJRegister);
  regnorm A (aInput, acc, 1'b1, clk, aOutput); 
  regnorm B (bInput, acc, 1'b1, clk, bOutput); 
  f_f Jump (inJump, acc, 1'b1, clk, outJump);
  f_f ifidF (ifidF_in, acc, 1'b1, clk, ifidF_out);
  regnorm adr (inAddress, acc, 1'b1, clk, outAddress); 
  reg5 rt (inrt, acc, 1'b1, clk, outrt); 
  reg5 rs (inrs, acc, 1'b1, clk, outrs); 
  f_f ALUSrc (inALUSRC, acc, 1'b1, clk, outALUSRC); 
  reg3 ALUOperation (inALUOper, acc, 1'b1, clk, outALUOper); 
  f_f RegDst (ReginDST, acc, 1'b1, clk, RegoutDST); 
  f_f MemWrite (inMemWrite, acc, 1'b1, clk, outMemWrite); 
  reg5 Rd (inRD, acc, 1'b1, clk, outRD);  
  f_f MemRead (inMemRead, acc, 1'b1, clk, outMemRead); 
  f_f PCSrc (inPCSRC, acc, 1'b1, clk, outPCSRC);
endmodule
//====================================================================
module IFID ( clk, acc, ld, inst_in, instructionOut, PC4_in, PC4_out);
  output [31:0] instructionOut;
  output [31:0] PC4_out;
  input  [31:0] inst_in;
  input  clk, acc, ld;
  input  [31:0] PC4_in; 
  regnorm inst (inst_in, acc, ld, clk, instructionOut);
  regnorm PC4 (PC4_in, acc, ld, clk, PC4_out);
endmodule
//======================================================================
module inst_mem (adr, d_out);
  input [31:0] adr;
  output [31:0] d_out;
  reg [7:0] mem[0:65535];
  initial
  begin
	$readmemb("inst_mem.bin", mem);	
  end
  assign d_out = {mem[adr[15:0]+3], mem[adr[15:0]+2], mem[adr[15:0]+1], mem[adr[15:0]]};
endmodule
//======================================================================
module membw (input clk, acc,input  [31:0] inALUResult,output [31:0] outALUResult,input  [31:0] inRD,
output [31:0] outRD,input [4:0] inDST,output [4:0] outDST,input inMToReg,
output outMToReg,input inRegWrite,output outRegWrite );

  reg5 dst (inDST, acc, 1'b1, clk, outDST);
  f_f mreg (inMToReg, acc, 1'b1, clk, outMToReg); 
  regnorm res (inALUResult, acc, 1'b1, clk, outALUResult); 
  f_f rwrite (inRegWrite, acc, 1'b1, clk, outRegWrite);
  regnorm regn (inRD, acc, 1'b1, clk, outRD); 
endmodule
//======================================================================
module pipeline (input rst, clk,output [31:0] addressInstruction,input  [31:0] inst,output [31:0] address_d,
 input  [31:0] in,output [31:0] dataOut,output readMemory, writeMemory);

  wire regDst, mToReg, alusrcmain, pcsrc, jRegister, Jump, reg_write, EQ, readMemory_local, writeMemory_local, PCWrite, ifidw, ifidF, idex_readMemory;
  wire membw_reg_write, exmem_reg_write; 
  wire [1:0] aForwarding, bForwarding;
  wire [2:0] ControllerAlu;
  wire [4:0] rtIDEX, rs_IDEX, dstEXM, dstMWB;
  wire [31:0] instructionOut;
  datapath DP(clk, rst, addressInstruction, inst, address_d, dataOut, in, regDst, mToReg, alusrcmain, pcsrc, jRegister, Jump, ControllerAlu, reg_write, writeMemory_local, writeMemory, readMemory_local, readMemory, EQ, instructionOut, PCWrite, ifidw, ifidF, aForwarding, bForwarding, rtIDEX, rs_IDEX, idex_readMemory, membw_reg_write, exmem_reg_write, dstEXM, dstMWB );
  controller CU(instructionOut, EQ, mToReg, regDst, reg_write, alusrcmain, readMemory_local, writeMemory_local, pcsrc, Jump, jRegister, ifidw, ifidF, membw_reg_write, exmem_reg_write, idex_readMemory, ControllerAlu, PCWrite, dstEXM, dstMWB,rs_IDEX, rtIDEX, aForwarding, bForwarding);
endmodule
//======================================================================
module mux21b (i0, i1, sel, y);
  input [4:0] i0, i1;
  input sel;
  output [4:0] y;
  assign y = (sel==1'b1) ? i1 : i0;
endmodule
//======================================================================
module mux21 (i0, i1, sel, y);
  input [31:0] i0, i1;
  input sel;
  output [31:0] y;
  assign y = (sel==1'b1) ? i1 : i0;
endmodule
//======================================================================
module mux41 (i0, i1, i2, i3, sel, y);
  input [31:0] i0, i1, i2, i3;
  input [1:0] sel;
  output [31:0] y;
  assign y = (sel==2'd0) ? i0 : (sel==2'd1) ? i1 : (sel==2'd2) ? i2 : i3;
endmodule
//======================================================================
module reg3 (d_in, acc, ld, clk, d_out);
  input [2:0] d_in;
  input acc, ld, clk;
  output [2:0] d_out;
  reg [2:0] d_out;
  always @(posedge clk, posedge acc)
  begin
    if (acc==1'b1)
      d_out <= 3'd0;
    else if (ld)
      d_out <= d_in;
  end
endmodule
//======================================================================
module reg5 (d_in, acc, ld, clk, d_out);
  input [4:0] d_in;
  input acc, ld, clk;
  output [4:0] d_out;
  reg [4:0] d_out;
  always @(posedge clk, posedge acc)
  begin
    if (acc==1'b1)
      d_out <= 5'd0;
    else if (ld)
      d_out <= d_in;
  end
endmodule
//======================================================================
module regnorm (d_in, acc, ld, clk, d_out);
  input [31:0] d_in;
  input acc, ld, clk;
  output [31:0] d_out;
  reg [31:0] d_out;
  always @(posedge clk)
  begin
    if (acc==1'b1)
      d_out <= 32'd0;
    else if (ld)
      d_out <= d_in;
  end
endmodule
//======================================================================
module regfile (wr_data, rd_reg1, rd_reg2, wr_reg, reg_write, rst, clk, rd_data1, rd_data2);
  input [31:0] wr_data;
  input [4:0] rd_reg1, rd_reg2, wr_reg;
  input reg_write, rst, clk;
  output [31:0] rd_data1, rd_data2;
  //reg [31:0] register_file_master [0:31];
  reg [31:0] register_file [0:31];
  integer i;
  assign rd_data1 = (rd_reg1 == 5'b0) ? 32'd0 : register_file[rd_reg1];
  assign rd_data2 = (rd_reg2 == 5'b0) ? 32'd0 : register_file[rd_reg2];
  /*  always @(negedge clk)
     if (reg_write == 1'b1)
      if(wr_reg != 5'd0)
        register_file_master[wr_reg] <= wr_data;
	always @(posedge clk)
		if (rst == 1'b1)
      for (i = 0; i < 32; i = i + 1) begin
		register_file_master[i] <= 32'd0;
        register_file[i] <= 32'd0;
		end
		else if (reg_write == 1'b1)
			if(wr_reg != 5'd0)
        register_file[wr_reg] <= register_file_master[wr_reg];
		*/
  always @(negedge clk)
    if (rst == 1'b1)
      for (i = 0; i < 32; i = i + 1)
        register_file[i] <= 32'd0;
    else if (reg_write == 1'b1)
      if(wr_reg != 5'd0)
        register_file[wr_reg] <= wr_data;
endmodule
//======================================================================
module shl2 (d_in, d_out);
  input [31:0] d_in;
  output [31:0] d_out;
  assign d_out = d_in << 2;
endmodule
//======================================================================
module SE (d_in, d_out);
  input [15:0] d_in;
  output [31:0] d_out;
  assign d_out = {{16{d_in[15]}}, d_in};
endmodule
//======================================================================