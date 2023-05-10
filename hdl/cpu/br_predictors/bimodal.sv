module brp_bimodal
import rv32i_types::*;
(
    input logic clk,
    input logic rst,
    input logic load,
    input logic update,

    input logic brp_ex,

    output logic br_prediction
);

enum int unsigned {
    s_snt,  // Strongly not-taken
    s_wnt,  // Weakly not-taken
    s_wt,   // Weakly taken
    s_st    // Strongly taken
} state, next_state;

function void predict(logic prediction);
    if (load) br_prediction = prediction;
endfunction

always_comb
begin : state_actions
    case (state)
        s_wt: predict(1'b1);
        s_st: predict(1'b1);
        default: predict(1'b0);
    endcase
end

always_comb
begin : next_state_logic
    if (brp_ex.mp_valid)
    begin
        case (state)
            s_snt:
            begin
                if (brp_ex.mispredicted) next_state = s_wnt;
                else next_state = s_snt;
            end

            s_wnt:
            begin
                if (brp_ex.mispredicted) next_state = s_wt;
                else next_state = s_snt;
            end

            s_wt:
            begin
                if (brp_ex.mispredicted) next_state = s_wnt;
                else next_state = s_st;
            end

            s_st:
            begin
                if (brp_ex.mispredicted) next_state = s_wt;
                else next_state = s_st;
            end

            default: next_state = s_wnt;
        endcase
    end
end

always_ff @(posedge clk)
begin : next_state_assignment
    if (rst) state <= s_wnt;
    else if (update) state <= next_state;
    else state <= state;
end

endmodule : brp_bimodal
