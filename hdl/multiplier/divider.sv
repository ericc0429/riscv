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

always_comb begin
    acc_a = {32'b0, a};
    q = 32'b0;

    // divide by zero
    if (b == '0) begin
        if (sign) begin
            f = -1;
            rem = a;
        end

        else begin
            f = {32{1'b1}};
            rem = a;
        end
    end

    // overflow
    if (a[31] == '1 && a[30:0] == '0 && b == -1) begin
        f = a;
        rem = 32'b0;
    end 


    for (int i=0; i<32; i++) begin
        acc_a = acc_a << 1;
        q = q << 1;

        if (acc_a[63:32] >= b) begin
            acc_b = acc_a[63:32] - b;
            acc_a = {acc_b, acc_a[31:0]};
            q[0] = 1'b1;
        end
    end

    f = q;
    rem = acc_a[63:32];
end

endmodule : divider
