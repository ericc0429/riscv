module arbiter
import rv32i_types::*;
(
    input clk,
    // From caches
    input logic instr_read,
    input rv32i_word instr_addr,

    input logic data_read,
    input rv32i_word data_addr,
    input logic data_write,
    input [255:0] data_mem_wdata,

    // To caches
    output logic instr_mem_resp,
    output logic [255:0] instr_cacheline,

    output logic data_mem_resp,
    output logic [255:0] data_cacheline,

    // From cacheline adapter
    input [255:0] pmem_rdata,
    input pmem_resp,

    // To cacheline adapter
    output logic pmem_read,
    output logic pmem_write,
    output rv32i_word pmem_address,
    output logic [255:0] pmem_wdata
);

logic service;

function void set_defaults();
    pmem_read = '0;
    pmem_write = '0;
endfunction

always_comb begin
    set_defaults();

    case (service)
        '0: begin
            pmem_read = instr_read;
            pmem_address = instr_addr;
            pmem_wdata = '0;
        end
        
        '1: begin
            pmem_read = data_read;
            pmem_address = data_addr;
            pmem_wdata = data_mem_wdata;
        end
    endcase

    pmem_write = data_write & service;
end

always_comb begin
    instr_mem_resp = '0;
    data_mem_resp = '0;
    instr_cacheline = pmem_rdata;
    data_cacheline = pmem_rdata;

    if (pmem_resp && (service == '0)) begin
        instr_mem_resp  = pmem_resp;
    end

    else if (pmem_resp && (service == '1)) begin
        data_mem_resp  = pmem_resp;
    end
end

always_ff @(posedge clk) begin
    if (instr_read)
        service <= '0;
    if (data_read || data_write) 
        service <= '1;
    else service <= '0;
end

endmodule : arbiter
