module brp
import rv32i_types::*;
(
    input logic clk,
    input logic rst,
    input logic load,
    input logic update,

    // IF instruction data
    input rv32i_word pc_if,
    input rv32i_opcode opcode_if,
    // Offsets
    input rv32i_word b_imm,
    input rv32i_word j_imm,
    
    // For accuracy counter
    input rv32i_brp_word brp_ex,
    
    // Outputs brp for current instruction
    output rv32i_brp_word brp_if
);

logic brp_prediction;
rv32i_brp_word brp;
int c_total;    // Total #predictions counter
int c_correct;  // #correct predictions counter


brp_bimodal bimodal (
    .clk,
    .rst,
    .load   (opcode_if == op_br), // Only use predictor when it's a branch instruction
    .update (update/* brp_ex.predicted && brp_ex.mp_valid */),
    .brp_ex (brp_ex),

    .br_prediction (brp_prediction)
);

function void set_defaults ();
    brp.predicted = '0;
    brp.prediction = '0;
    brp.brp_target = '0;
    brp.brp_alt = '0;
    brp.mp_valid = '0;
    brp.mispredicted = '0;
endfunction

function void predict(logic _prediction, rv32i_word offset);
    // brp_if.predicted = '1;
    brp.prediction = _prediction;

    brp.brp_target = (_prediction) ? (pc_if + offset) : (pc_if + 4);
    brp.brp_alt = !(_prediction) ? (pc_if + offset) : (pc_if + 4);
endfunction


always_comb
begin : predictor
    set_defaults();

    // INSTR is a branch/jal, so we want prediction output
    // Not handling JALR instructions because of potential for hazards in RS1
    case (opcode_if)
        op_jal:
        begin
            brp.predicted = '0;
            predict(1'b1, j_imm);
        end
        
        op_br:
        begin
            brp.predicted = '1;
            predict(brp_prediction, b_imm);
        end
    endcase
end

// always_ff @(posedge clk)
always_comb
begin : gen_brp_if
    if (rst) brp_if = '0;
    else if (load)
    begin
        brp_if = brp;
    end
    else begin
        brp_if = '0;  // If load isn't high, we still reset instead of keeping the previous brp output
    end
end

// always_ff @(posedge clk)
/* always_comb
begin : gen_brp_ex_out
    if (rst) brp_ex_out = '0;
    else if (update)
    begin
        brp_ex_out.predicted = brp_ex.predicted;
        brp_ex_out.prediction = brp_ex.prediction;
        brp_ex_out.brp_target = brp_ex.brp_target;
        brp_ex_out.brp_alt = brp_ex.brp_alt;
        brp_ex_out.mp_valid = '1;
        brp_ex_out.mispredicted = (brp_ex.prediction == br_en) ? '0 : '1;
    end
    else brp_ex_out = brp_ex;
end */

always_ff @(posedge clk)
begin : correctness
    // Reset signal
    if (rst)
    begin
        c_total <= 0;
        c_correct <= 0;
    end
    // We predicted for the instruction in EX stage
    else if (update)
    begin
        c_total <= c_total + 1;
        if (brp_ex.mp_valid && !brp_ex.mispredicted) c_correct <= c_correct + 1;
    end
end


endmodule : brp
