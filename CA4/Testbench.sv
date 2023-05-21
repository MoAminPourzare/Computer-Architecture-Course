`timescale 1ns/1ns
module TB();
  reg clk = 0;
  reg rst = 0;
  
  reg [31:0] Inst_mem_data[0:1023];
  
  initial $readmemb("instruction_memory.mem",Inst_mem_data);

  MIPS mips(clk, rst, Inst_mem_data);
  
  initial begin
    forever #20 clk = ~clk;
  end

  initial begin
    // //Instructions
    // Inst_mem_data [0]={6'b000001, 5'b00000, 5'b01001, 16'b0000001111101000};//addi R9, R0, 1000 (R9 Stores the max index)
    // Inst_mem_data [4]={6'b000011, 5'b00000, 5'b01111, 16'b0000001111101000};//lw R15, 1000(R0) (R15 Stores the max)
    // Inst_mem_data [8]={6'b000001, 5'b00000, 5'b00001, 16'b0000000001010000};//addi R1, R0, 80

    // Inst_mem_data [12] = 32'b0;//LOOP
    // Inst_mem_data [16]={6'b000101, 5'b00001, 5'b00000,/*R*/ 16'b0000000000000111};//beq R1, R0, END_LOOP
    // Inst_mem_data [20]={6'b000011, 5'b00001, 5'b01110, 16'b0000001111101000};//lw R14, 1000(R1)

    // Inst_mem_data [24]={6'b000000, 5'b01111, 5'b01110, 5'b01010, 5'b00000, 6'b000101}; //slt R10, R15, R14 (R14 > R15)
    // Inst_mem_data [28]={6'b000101, 5'b00000, 5'b01010,R 16'b0000000000000010};//beq R10, R0, INST 40

    // Inst_mem_data [32]={6'b000000, 5'b01110, 5'b00000, 5'b01111, 5'b00000, 6'b000000};//add R15, R14, R0

    // Inst_mem_data [36]={6'b000000, 5'b00001, 5'b00000, 5'b01001, 5'b00000, 6'b000000};//add R9, R1, R0
    // Inst_mem_data [40]={6'b000001, 5'b00001, 5'b00001, 16'b1111111111111100};//addi R1, R1, -4

    // Inst_mem_data [44]={6'b000110, 26'b00000000000000000000000011};//jump LOOP
    // Inst_mem_data [48] = 32'b0;//END_LOOP

    // Inst_mem_data [52]={6'b000001, 5'b01001, 5'b01001, 16'b0000001111101000};//addi R9, R9, 1000

    // Inst_mem_data [56]={6'b000100, 5'b00000, 5'b01001, 16'b0000011111010100};//sw R9, 2004(R0)
    // Inst_mem_data [60]={6'b000100, 5'b00000, 5'b01111, 16'b0000011111010000};//sw R15, 2000(R0)


    clk = 0;
      rst = 0;
      #220
      rst = 1;
      #110
      rst =0;
      #100000
      $stop;
  end

endmodule
