module Shl2_12(input [11:0] In , 	output [11:0] Out);
    assign Out = {In[29:0], 2'b00};
endmodule

module Shl2_26(input [25:0] In , 	output [27:0] Out);
    assign Out = {In, 2'b00};
endmodule