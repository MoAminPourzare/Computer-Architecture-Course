module Data_Memory (input clk,input [31:0] Address, Write_data , input Mem_read , Mem_write , output reg [31:0] Mem_read_value) ;

  reg [31:0] Mem_data [0:4095];
  // Read Data Memory
  initial $readmemb("data_memory.mem",Mem_data);
  wire [31:0] max_element;
  wire [31:0] max_element_i;
  assign max_element = Mem_data[2000];
  assign max_element_i = Mem_data[2004];
  
  always @(posedge clk) begin
    if( Mem_write )
      Mem_data[Address] <= Write_data;
  end

  assign Mem_read_value = (Mem_read) ? Mem_data[Address] : Mem_read_value;

endmodule