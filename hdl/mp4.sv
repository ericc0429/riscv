module mp4
import rv32i_types::*;
(
    input clk,
    input rst,
	
	//Remove after CP1
    /* input 				instr_mem_resp,
    input rv32i_word 	instr_mem_rdata,
	input 				data_mem_resp,
    input rv32i_word 	data_mem_rdata, 
    output logic 		instr_read,
	output rv32i_word 	instr_mem_address,
    output logic 		data_read,
    output logic 		data_write,
    output logic [3:0] 	data_mbe,
    output rv32i_word 	data_mem_address,
    output rv32i_word 	data_mem_wdata */

	// For CP2
    input pmem_resp,
    input [63:0] pmem_rdata,

	// To physical memory
    output logic pmem_read,
    output logic pmem_write,
    output rv32i_word pmem_address,
    output [63:0] pmem_wdata
	
);

/* ================ INTERNAL SIGNALS ================ */

// control to datapath
rv32i_control_word ctrl;
    
// datapath to control
rv32i_opcode opcode;
logic [2:0] funct3;
logic [6:0] funct7;

/* === Memory stuff === */

// Instruction Fetch cache miss
rv32i_word Icache_address;
logic Icache_read;
logic [255:0] Icache_rdata;
// logic Icache_wdata;
// logic Icache_write;
logic Icache_resp;

// Memory Fetch cache miss
rv32i_word Dcache_address;
logic [255:0] Dcache_wdata;
logic [255:0] Dcache_rdata;
logic Dcache_read;
logic Dcache_write;
logic Dcache_resp;


// CPU --> Caches
// Instruction 
logic instr_mem_resp;
rv32i_word 	instr_mem_rdata;
logic 		instr_read;
rv32i_word 	instr_mem_address;

// Memory
logic data_mem_resp;
rv32i_word 	data_mem_rdata;
logic 		data_read;
logic 		data_write;
logic [3:0] 	data_mbe;
rv32i_word 	data_mem_address;
rv32i_word 	data_mem_wdata;

// Between CL Adapter and Arbiter
logic [255:0] pmem_rdata256;
logic [255:0] pmem_wdata256;
rv32i_word pmem_c_addr;
logic pmem_c_read;
logic pmem_c_write;
logic pmem_c_resp;


/* ================ TOP-LEVEL BLOCKS ================ */

datapath datapath(.*);

cache    Icache( // Instruction 
    /* Physical memory signals */
    // inputs
    .pmem_resp(Icache_resp),
    .pmem_rdata(Icache_rdata),

    // outputs
    .pmem_address(Icache_address),
    .pmem_read(Icache_read),
    // .pmem_wdata(),
    // .pmem_write(),

    /* CPU memory signals */
    // inputs
    .mem_read(instr_read),
    .mem_write('0),
    .mem_byte_enable_cpu('1),
    .mem_address(instr_mem_address),
    .mem_wdata_cpu('0),

    // outputs
    .mem_resp(instr_mem_resp),
    .mem_rdata_cpu(instr_mem_rdata)
);

cache    Dcache( // Data
    /* Physical memory signals */
    // inputs
    .pmem_resp(Dcache_resp),
    .pmem_rdata(Dcache_rdata),

    // outputs
    .pmem_address(Dcache_address),
    .pmem_wdata(Dcache_wdata),
    .pmem_read(Dcache_read),
    .pmem_write(Dcache_write),

    /* CPU memory signals */
    // inputs
    .mem_read(data_read),
    .mem_write(data_write),
    .mem_byte_enable_cpu(data_mbe),
    .mem_address(data_mem_address),
    .mem_wdata_cpu(data_mem_wdata),

    // outputs
    .mem_resp(data_mem_resp),
    .mem_rdata_cpu(data_mem_rdata)
);

arbiter   arbiter(
    .clk,
    // From caches
    .instr_read     (Icache_read),
    .instr_addr     (Icache_address),
    
    .data_read      (Dcache_read),
    .data_addr      (Dcache_address),
    .data_write     (Dcache_write),
    .data_mem_wdata (Dcache_wdata),

    // To caches
    .instr_mem_resp     (Icache_resp),
    .instr_cacheline    (Icache_rdata),
    .data_mem_resp      (Dcache_resp),
    .data_cacheline     (Dcache_rdata),

    // From cacheline adapter
    .pmem_rdata     (pmem_rdata256),
    .pmem_resp      (pmem_c_resp),

    // To cacheline adapter
    .pmem_read      (pmem_c_read),
    .pmem_write     (pmem_c_write),
    .pmem_address   (pmem_c_addr),
    .pmem_wdata     (pmem_wdata256)
);

cacheline_adapter cacheline_adapter (
    .clk        (clk),
    .reset_n    (~rst),    
    // Port to LLC (Lowest Level Cache)
    // inputs
    .line_i     (pmem_wdata256),
    .address_i  (pmem_c_addr),
    .read_i     (pmem_c_read),
    .write_i    (pmem_c_write),

    // outputs
    .line_o     (pmem_rdata256),
    .resp_o     (pmem_c_resp),

    // Port to memory
    // inputs
    .burst_i    (pmem_rdata),
    .resp_i     (pmem_resp),

    // outputs
    .burst_o    (pmem_wdata),
    .address_o  (pmem_address),
    .read_o     (pmem_read),
    .write_o    (pmem_write)
);

endmodule : mp4
