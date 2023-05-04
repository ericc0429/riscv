module divider 
(
    input [31:0] a,
    input [31:0] b,
    input sign,

    output logic [31:0] f,
    output logic [31:0] rem
);

logic [63:0] acc_a;
logic [31:0] q;
logic [31:0] acc_b; // accumulator minus b

// assign acc_a = {32'b0, a};
// assign q = 32'b0;
logic sign1, sign2, final_sign;
logic [31:0] op1, op2;

always_comb begin
    sign1 = '0;
    sign2 = '0;
    op1 = a;
    op2 = b;

    if (sign) begin
        if (a[31]) begin
            op1 = ~a + 1;
            sign1 = '1;
        end
        if (b[31]) begin
            op1 = ~a + 1;
            sign2 = '1;
        end
    end

    final_sign = sign1 ^ sign2;
end

always_comb begin
    acc_a = {32'b0, op1};
    q = 32'b0;

    // divide by zero
    if (op2 == '0) begin
        if (sign) begin
            f = -1;
            rem = op1;
        end

        else begin
            f = {32{1'b1}};
            rem = op1;
        end
    end

    // overflow
    if (op1[31] == '1 && op1[30:0] == '0 && op2 == -1) begin
        f = op1;
        rem = 32'b0;
    end 


    for (int i=0; i<32; i++) begin
        acc_a = acc_a << 1;
        q = q << 1;

        if (acc_a[63:32] >= op2) begin
            acc_b = acc_a[63:32] - op2;
            acc_a = {acc_b, acc_a[31:0]};
            q[0] = 1'b1;
        end
    end

    if (final_sign) begin
        f = -q;
        rem = -acc_a[63:32];
    end
    else begin
        f = q;
        rem = acc_a[63:32];
    end
end

endmodule : divider
