module wall_tree 
(
    input [31:0] a,
    input [31:0] b,
    input op1sign,
    input op2sign,

    output logic [63:0] f
);

// take care of signs
logic final_sign;
logic [31:0] op1, op2;
logic sign1, sign2;

always_comb begin
    sign1 = '0;
    sign2 = '0;
    op1 = a;
    op2 = b;

    if (op1sign && op2sign) begin  // signed, signed
        if (a[31]) begin
            op1 = ~a + 1;
            sign1 = '1;
        end
        if (b[31]) begin
            op2 = ~b + 1;
            sign2 = '1;
        end
        final_sign = sign1 ^ sign2;
    end

    else if (op1sign) begin // signed, unsigned
        if (a[31]) begin
            op1 = ~a + 1;
            sign1 = '1;
        end
        final_sign = sign1;
    end

    else        // unsigned, unsigned
        final_sign = '0;
end

// calculate partials

logic [63:0] partials [32];
logic [31:0] p;
logic [63:0] p2;

always_comb begin
    for (int i=0; i<32; i++) begin
        // p = 64'b0;
        p = op1 & {32{op2[i]}};
        p2 = {32'b0, p};
        partials[i] = p2 << i;
    end
end

// stage 1 (32 -> 22)
logic [63:0] s1 [20];

fa fa0 (.row0(partials[0]),     .row1(partials[1]),     .row2(partials[2]),     .x(s1[0]),      .y(s1[1]));
fa fa1 (.row0(partials[3]),     .row1(partials[4]),     .row2(partials[5]),     .x(s1[2]),      .y(s1[3]));
fa fa2 (.row0(partials[6]),     .row1(partials[7]),     .row2(partials[8]),     .x(s1[4]),      .y(s1[5]));
fa fa3 (.row0(partials[9]),     .row1(partials[10]),    .row2(partials[11]),    .x(s1[6]),      .y(s1[7]));
fa fa4 (.row0(partials[12]),    .row1(partials[13]),    .row2(partials[14]),    .x(s1[8]),      .y(s1[9]));
fa fa5 (.row0(partials[15]),    .row1(partials[16]),    .row2(partials[17]),    .x(s1[10]),     .y(s1[11]));
fa fa6 (.row0(partials[18]),    .row1(partials[19]),    .row2(partials[20]),    .x(s1[12]),     .y(s1[13]));
fa fa7 (.row0(partials[21]),    .row1(partials[22]),    .row2(partials[23]),    .x(s1[14]),     .y(s1[15]));
fa fa8 (.row0(partials[24]),    .row1(partials[25]),    .row2(partials[26]),    .x(s1[16]),     .y(s1[17]));
fa fa9 (.row0(partials[27]),    .row1(partials[28]),    .row2(partials[29]),    .x(s1[18]),     .y(s1[19]));

// stage 2 (22 -> 15)
logic [63:0] s2 [14];

fa fa10 (.row0(s1[0]),     .row1(s1[1]),     .row2(s1[2]),     .x(s2[0]),      .y(s2[1]));
fa fa11 (.row0(s1[3]),     .row1(s1[4]),     .row2(s1[5]),     .x(s2[2]),      .y(s2[3]));
fa fa12 (.row0(s1[6]),     .row1(s1[7]),     .row2(s1[8]),     .x(s2[4]),      .y(s2[5]));
fa fa13 (.row0(s1[9]),     .row1(s1[10]),    .row2(s1[11]),    .x(s2[6]),      .y(s2[7]));
fa fa14 (.row0(s1[12]),    .row1(s1[13]),    .row2(s1[14]),    .x(s2[8]),      .y(s2[9]));
fa fa15 (.row0(s1[15]),    .row1(s1[16]),    .row2(s1[17]),    .x(s2[10]),     .y(s2[11]));
fa fa16 (.row0(s1[18]),    .row1(s1[19]),    .row2(partials[30]),    .x(s2[12]),     .y(s2[13]));

// stage 3 (15 -> 10)
logic [63:0] s3 [10];

fa fa17 (.row0(s2[0]),     .row1(s2[1]),     .row2(s2[2]),     .x(s3[0]),      .y(s3[1]));
fa fa18 (.row0(s2[3]),     .row1(s2[4]),     .row2(s2[5]),     .x(s3[2]),      .y(s3[3]));
fa fa19 (.row0(s2[6]),     .row1(s2[7]),     .row2(s2[8]),     .x(s3[4]),      .y(s3[5]));
fa fa20 (.row0(s2[9]),     .row1(s2[10]),    .row2(s2[11]),    .x(s3[6]),      .y(s3[7]));
fa fa21 (.row0(s2[12]),    .row1(s2[13]),    .row2(partials[31]),    .x(s3[8]),      .y(s3[9]));

// stage 4 (10 -> 7)
logic [63:0] s4 [6];

fa fa22 (.row0(s3[0]),     .row1(s3[1]),     .row2(s3[2]),     .x(s4[0]),      .y(s4[1]));
fa fa23 (.row0(s3[3]),     .row1(s3[4]),     .row2(s3[5]),     .x(s4[2]),      .y(s4[3]));
fa fa24 (.row0(s3[6]),     .row1(s3[7]),     .row2(s3[8]),     .x(s4[4]),      .y(s4[5]));

// stage 5 (7 -> 5)
logic [63:0] s5 [4];

fa fa25 (.row0(s4[0]),     .row1(s4[1]),     .row2(s4[2]),     .x(s5[0]),      .y(s5[1]));
fa fa26 (.row0(s4[3]),     .row1(s4[4]),     .row2(s4[5]),     .x(s5[2]),      .y(s5[3]));

// stage 6 (5 -> 4)
logic [63:0] s6 [2];

fa fa27 (.row0(s5[0]),     .row1(s5[1]),     .row2(s5[2]),     .x(s6[0]),      .y(s6[1]));

// stage 7 (4 -> 3)
logic [63:0] s7 [2];

fa fa28 (.row0(s6[0]),     .row1(s6[1]),     .row2(s5[3]),     .x(s7[0]),      .y(s7[1]));

// stage 8 (3 -> 2)
logic [63:0] s8 [2];

fa fa29 (.row0(s7[0]),     .row1(s7[1]),     .row2(s3[9]),     .x(s8[0]),      .y(s8[1]));

// final summation
logic [63:0] mag;
cpa cpa (.row0(s8[0]), .row1(s8[1]), .x(mag));


always_comb begin
    if (final_sign) begin
        f = {{32{1'b1}}, mag[31:0]};
    end

    else
        f = mag;
       
end
// assign f = {final_sign, (~(mag-1))[62:0]  };

endmodule : wall_tree