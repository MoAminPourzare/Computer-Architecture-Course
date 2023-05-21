`timescale 1ns/1ns

module REG32(input rst, clk, ld, input [31:0]data, output reg [31:0]out);

  
  always @(posedge clk,posedge rst) begin
    
      if(rst) out<=32'b0;
        
      else if(ld==1) out<=data;
        
      else out<=out;
  end
    
    
endmodule
