`define rtype 6'b000000
`define lw 6'b100011
`define sw 6'b101011
`define beq 6'b000100
`define addi 6'b001001
`define slti 6'b001010
`define jump 6'b000010
`define JRegister 6'b000110


module controller (input [31:0] instruction,input EQ, output reg MToReg,output reg regDst,output reg regWrite,output reg ALUSRC,output reg MemoryRead,output reg memoryWrite,output SRCPCounter,output reg Jump,output reg JRegister,output writeIfId,output lushIfIdF,input regWriteMWB, input regWriteEXM,input memoryReadIDEX,output [2:0] Oper,output writePCounter,input [4:0] rdEXM,input [4:0] rdMWB,input [4:0] rsIDEX,input [4:0] rtIDEX,output [1:0] aForwarding, output [1:0] bForwarding);   
     
    reg [1:0] OperationALU;     
    reg branch;   
    wire [5:0] OPC;
    wire [4:0] rt,rs;
    wire [5:0] Function_a;
    assign OPC = instruction[31:26];
    assign Function_a = instruction[5:0];
    assign rs = instruction[25:21];
    assign rt = instruction[20:16];
    Forwarding F_You (aForwarding, bForwarding, regWriteMWB, regWriteEXM, rdEXM, rdMWB, rsIDEX, rtIDEX);
    Hazard H_You (memoryReadIDEX, rtIDEX, rs, rt, NOP_Sel, writeIfId, writePCounter, Jump);
    alu_controller ALU_CTRL(OperationALU, Function_a, Oper);
    always @(OPC, NOP_Sel)
    begin
      {regDst, ALUSRC, MToReg, regWrite, MemoryRead, memoryWrite, branch, OperationALU, JRegister, Jump} = 11'b0;
      case (OPC)
        // rtype instructionructions
        `rtype : {regDst, regWrite, OperationALU} = 4'b1110;   
        // Load Word (lw) instructionruction           
        `lw : {ALUSRC, MToReg, regWrite, MemoryRead} = 4'b1111; 
        // Store Word (sw) instructionruction
        `sw : {ALUSRC, memoryWrite} = 2'b11;                                 
        // Branch on EQ (beq) instructionruction
        `beq : {branch, OperationALU} = 3'b101; 
        // Add immediate (addi) instructionruction
        `addi : {regWrite, ALUSRC} = 2'b11; 
        // Set one less than immediate(slti) instructionructions
        `slti : {ALUSRC, regWrite, OperationALU} = 4'b1111;    
        // Jump (j) instructionruction           
        `jump :  Jump = 1'b1;                              
        // Jump Register(jr) instructionruction
        `JRegister : {JRegister, Jump} = 2'b11;         
      endcase
	  {regDst, ALUSRC, MToReg, regWrite, MemoryRead, memoryWrite, branch, OperationALU, JRegister, Jump} = NOP_Sel ? 11'b0 : 
           {regDst, ALUSRC, MToReg, regWrite, MemoryRead, memoryWrite, branch, OperationALU, JRegister, Jump};
    end
    assign SRCPCounter = branch & EQ;
	assign lushIfIdF = SRCPCounter || Jump || JRegister;
endmodule
//===========================================================================
module alu_controller (OperationALU, Function_a, Oper);
  input [1:0] OperationALU;
  input [5:0] Function_a;
  output [2:0] Oper;
  reg [2:0] Oper;
  always @(OperationALU, Function_a)
  begin
    Oper = 3'b000;
    if (OperationALU == 2'b00)        // lw or sw
      Oper = 3'b010;
    else if (OperationALU == 2'b01)   // beq
      Oper = 3'b110;
    else if (OperationALU == 2'b11)   // slti
      Oper = 3'b111;
    else // R type
      begin
        case (Function_a)
          6'b100000: Oper = 3'b010;  // add
          6'b100011: Oper = 3'b110;  // sub
          6'b100100: Oper = 3'b000;  // and
          6'b100101: Oper = 3'b001;  // or
          6'b101010: Oper = 3'b111;  // slt
          default:   Oper = 3'b000;
        endcase
      end
  end
endmodule
//===========================================================================