`timescale 1ns/1ns

module REG16(input rst, clk, ld, input [15:0]data, output reg [15:0]out);

  
  always @(posedge clk,posedge rst) begin
    
      if(rst) out<=16'b0;
        
      else if(ld==1) out<=data;
        
      else out<=out;
  end
    
    
endmodule

`timescale 1ns/1ns

module REG32(input rst, clk, ld, input [31:0]data, output reg [31:0]out);

  
  always @(posedge clk,posedge rst) begin
    
      if(rst) out<=31'b0;
        
      else if(ld==1) out<=data;
        
      else out<=out;
  end
    
    
endmodule
