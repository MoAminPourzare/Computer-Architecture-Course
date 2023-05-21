module TB();
  
  reg clk = 0, rst = 0;
  wire [31:0] Curinst, instMem, AdrD, data, out;
  wire mem_read, mem_write;
  
  
pipeline cmain(rst, clk, Curinst, instMem, AdrD, out, data, mem_read, mem_write);
  inst_mem c2 (Curinst, instMem);
  data_mem c1 (AdrD, data, mem_read, mem_write, clk, out);
  
  always begin #2 clk = ~clk;end

  initial
  begin
    rst = 1'b1;
    clk = 1'b0;
    #20 rst = 1'b0;
    #1250 $stop;
  end
  
  

  
endmodule



