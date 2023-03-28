
module cmp #(
    parameter op_length = 32
)
(
    cmpop,
    a, b,
    f
);

import rv32i_types::*;

input branch_funct3_t cmpop;
input [op_length-1:0] a, b;
output logic f;

always_comb
begin
    unique case (cmpop)
        beq:  f = (a==b) ? '1 : '0;
        bne:  f = (a!=b) ? '1 : '0;
        blt:  f = ($signed(a)<$signed(b))  ? '1 : '0; 
        bge:  f = ($signed(a)>=$signed(b)) ? '1 : '0;
        bltu: f = (a<b)  ? '1 : '0;
        bgeu: f = (a>=b) ? '1 : '0;
        default: f = '0;
    endcase
end

endmodule : cmp
