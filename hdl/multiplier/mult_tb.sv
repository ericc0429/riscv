module mult_tb();

timeunit 10ns;	
timeprecision 1ns;

logic [31:0] a, b;
logic [63:0] f;

// To store expected results
logic [63:0] ans_pp, ans_pn, ans_np, ans_nn;

integer ErrorCnt = 0;

// Instantiating the DUT
// Make sure the module and signal names match with those in your design
wall_tree wt(.*);	

// // Toggle the clock
// // #1 means wait for a delay of 1 timeunit
// always begin : CLOCK_GENERATION
// #1 clk = ~clk;
// end

// initial begin: CLOCK_INITIALIZATION
//     clk = 0;
// end 

// Testing begins here
// The initial block is not synthesizable
// Everything happens sequentially inside an initial block
// as in a software program
initial begin: test_mult

a = 32'd5;
b = 32'd6;

#2

ans_pp = 64'd30;

#45 
	if (f != ans_pp)
		ErrorCnt++;


#2        	

a = -32'd5;
b = 32'd6;

#2

ans_np = -64'd30;

#45 
	if (f != ans_np)
		ErrorCnt++;
					
		
if (ErrorCnt == 0)
	$display("Success!");  // Command line output in ModelSim
else
	$display("%d error(s) detected. Try again!", ErrorCnt);
end
endmodule