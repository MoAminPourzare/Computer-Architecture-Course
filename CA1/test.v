
`timescale 1ns/1ns
module TBregressor();
reg s=0,rst=1,clk=0;
reg [19:0]xi,yi;
wire [19:0]error,b0,b1;
wire busy,ready;

reg [19:0]memX[0:149];
reg [19:0]memY[0:149];


integer i;


regressor c1(xi, yi,clk,s,error, b0, b1,ready);
always #10 clk = ~clk;
initial begin

$readmemb("x_value.txt", memX);
$readmemb("y_value.txt", memY);


for (i = 0; i < 150; i = i + 1)begin: zz
                xi = memX[i];
		yi = memX[i];
	#10;
end


#40 rst = 0;
// full test
repeat (10000) #3 s = $random;

#40 $stop;
end
endmodule