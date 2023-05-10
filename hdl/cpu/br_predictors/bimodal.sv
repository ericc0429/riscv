module brp_bimodal
import rv32i_types::*;
(
    input logic clk,
    input logic rst,
    input logic load,
    input logic bp_correctness,     // Whether last prediction was correct; 1 = correct prediction
    
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
    case (state)
        s_snt:
        begin
            if (bp_correctness) next_state = s_snt;
            else next_state = s_wnt;
        end

        s_wnt:
        begin
            if (bp_correctness) next_state = s_snt;
            else next_state = s_wt;
        end

        s_wt:
        begin
            if (bp_correctness) next_state = s_st;
            else next_state = s_wnt;
        end

        s_st:
        begin
            if (bp_correctness) next_state = s_st;
            else next_state = s_wt;
        end

        default: next_state = s_wnt;
    endcase
end

always_ff @(posedge clk)
begin : next_state_assignment
    if (rst) state <= s_wnt;
    else if (load) state <= next_state;
    else state <= state;
end

endmodule : brp_bimodal
