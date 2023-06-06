// 1 bit full adder 
module full_adder
import rv32i_types::*;
(
    input a,
    input b,
    input c_in,

    output logic s,
    output logic c_out
);

assign s = (a ^ b) ^ c_in;
assign c_out = (a & b) | ((a ^ b) & c_in);

endmodule : full_adder

// 64 bit full adder
module fa
import rv32i_types::*;
(
    input [63:0] row0,
    input [63:0] row1,
    input [63:0] row2,

    output logic [63:0] x,
    output logic [63:0] y
);


logic [63:0] upper63;
assign x = row0 ^ row1 ^ row2;
assign upper63 = (row0 & row1) | (row0 & row2) | (row1 & row2);
assign y = {upper63[63:1], 1'b0};

endmodule : fa

// 64 bit carry propagate adder
module cpa 
(
    input [63:0] row0,
    input [63:0] row1,

    output logic [63:0] x
);

logic [64:0] c;
assign c[0] = '0;

genvar i;
generate for (i=0; i<64; i++) begin
    full_adder fa_chain (
        .a(row0[i]),
        .b(row1[i]),
        .c_in(c[i]),

        .s(x[i]),
        .c_out(c[i+1])
    );
end endgenerate

endmodule : cpa

// 64 bit carry propagate adder
module cpa32 
(
    input [31:0] row0,
    input [31:0] row1,

    output logic [31:0] x
);

logic [32:0] c;
assign c[0] = '0;

genvar i;
generate for (i=0; i<32; i++) begin
    full_adder fa_chain (
        .a(row0[i]),
        .b(row1[i]),
        .c_in(c[i]),

        .s(x[i]),
        .c_out(c[i+1])
    );
end endgenerate

endmodule : cpa32