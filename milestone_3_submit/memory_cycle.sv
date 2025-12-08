`ifndef MEMORY_CYCLE
`define MEMORY_CYCLE
////`include "add_sub_32_bit.sv"
////`include "lsu_new.sv"
module memory_cycle(
    input logic         i_clk,
    input logic         i_reset,

    // Inputs từ Execute giữ hộ inst
    input logic [31:0]  i_mem_inst,

    // Input từ Execute giá trị được sử dụng trong MEM
    input logic [31:0]  i_mem_pc,
    input logic [31:0]  i_mem_rs2_data,
    input logic         i_mem_br_equal,
    input logic         i_mem_br_less,
    input logic [31:0]  i_mem_alu_data,

    // input từ Execute MEM control
    input logic         i_mem_lsu_wren,
    input logic [2:0]   i_mem_slt_sl,

    // input từ Execute giữ hộ Writeback control
    input logic [1:0]   i_mem_wb_sel,
    input logic         i_mem_rd_wren,
    input logic         i_mem_insn_vld,
    input logic         i_mem_ctrl,

    // Input IO switch
    input logic [31:0]  i_io_sw,

    // Outputs từ Memory tới Writeback
    output logic [31:0] o_mem_pc_add4_wb, 
    output logic [31:0] o_mem_alu_data_wb,
    output logic        o_mem_insn_vld_wb,
    output logic        o_mem_ctrl,

    //output logic [31:0] mem_WB,  thay cái này bằng 1 đống IO
    output logic [31:0] o_mem_ld_data_wb , // Data thực sự writeback
    output logic [31:0] o_mem_io_ledr_wb , 
    output logic [31:0] o_mem_io_ledg_wb ,
    output logic [6:0]  o_mem_io_hex0_wb , 
    output logic [6:0]  o_mem_io_hex1_wb , 
    output logic [6:0]  o_mem_io_hex2_wb ,   
    output logic [6:0]  o_mem_io_hex3_wb , 
    output logic [6:0]  o_mem_io_hex4_wb , 
    output logic [6:0]  o_mem_io_hex5_wb , 
    output logic [6:0]  o_mem_io_hex6_wb ,   
    output logic [6:0]  o_mem_io_hex7_wb , 
    output logic [31:0] o_mem_io_lcd_wb ,
    
    output logic [31:0] o_mem_inst_wb,
    output logic [31:0] o_mem_pc_debug,

    // Output tới Writeback Control
    output logic [1:0]  o_mem_wb_sel_wb,
    output logic        o_mem_rd_wren_wb,

    // Output thêm rd ở tầng Memory để forward  
    output logic [4:0]  o_mem_rd_addr_fwd 
);
    
    // Internal signals
    logic [31:0]    PC_add4_internal;

    // Pipeline registers
    logic [31:0]    pc_add4_reg;
    logic [31:0]    alu_data_reg;
    logic [31:0]    inst_reg;
    logic [1:0]     wb_sel_reg;
    logic           rd_wren_reg;
    logic           insn_vld_reg;
    

    logic [31:0]   ld_data, ld_data_reg;
    logic [31:0]   io_ledr, io_ledr_reg;
    logic [31:0]   io_ledg, io_ledg_reg;
    logic [6:0]    io_hex0, io_hex0_reg; 
    logic [6:0]    io_hex1, io_hex1_reg;
    logic [6:0]    io_hex2, io_hex2_reg;
    logic [6:0]    io_hex3, io_hex3_reg;
    logic [6:0]    io_hex4, io_hex4_reg;
    logic [6:0]    io_hex5, io_hex5_reg;
    logic [6:0]    io_hex6, io_hex6_reg;
    logic [6:0]    io_hex7, io_hex7_reg;
    logic [31:0]   io_lcd,  io_lcd_reg;
    logic [31:0]   pc_debug_reg;
    logic          ctrl_reg;
    logic [6:0]    opcode_mem;
    logic          internal_stall;

    assign opcode_mem = i_mem_inst[6:0];

    //assign internal_stall = (opcode_mem == 7'b0000011) 1'b1 : 1'b0;

    // PC + 4
    add_sub_32_bit PC_add4_at_memory (
        .A(i_mem_pc),
        .B(32'd4),
        .Sel(1'b0),
        .Result(PC_add4_internal)
    );

    lsu_new lsu_memory(
        .i_clk          (i_clk),
        .i_reset        (i_reset),

        .i_lsu_wren     (i_mem_lsu_wren),
        .i_lsu_addr     (i_mem_alu_data),
        .i_st_data      (i_mem_rs2_data),

        .slt_sl         (i_mem_slt_sl),

        .i_io_sw        (i_io_sw),

        .o_ld_data     (ld_data),
        .o_io_lcd      (io_lcd), 
        .o_io_ledg     (io_ledg), 
        .o_io_ledr     (io_ledr), 
        .o_io_hex0     (io_hex0), 
        .o_io_hex1     (io_hex1), 
        .o_io_hex2     (io_hex2), 
        .o_io_hex3     (io_hex3),
        .o_io_hex4     (io_hex4), 
        .o_io_hex5     (io_hex5), 
        .o_io_hex6     (io_hex6), 
        .o_io_hex7     (io_hex7)
        );

    always_ff @(posedge i_clk /*or posedge i_reset*/) begin
        if (i_reset) begin
            pc_add4_reg <= 32'd0;
            alu_data_reg<= 32'd0;
            inst_reg    <= 32'd0;
            wb_sel_reg  <= 0;
            rd_wren_reg <= 0;
            ld_data_reg <= 32'b0;
            io_ledr_reg <= 32'b0;
            io_ledg_reg <= 32'b0;
            io_hex0_reg <= 7'b0;
            io_hex1_reg <= 7'b0;
            io_hex2_reg <= 7'b0;
            io_hex3_reg <= 7'b0;
            io_hex4_reg <= 7'b0;
            io_hex5_reg <= 7'b0;
            io_hex6_reg <= 7'b0;
            io_hex7_reg <= 7'b0;
            io_lcd_reg  <= 32'b0;
            insn_vld_reg<= 0;
            pc_debug_reg<= 0;
            ctrl_reg    <= 0;  

        end 
        /*
        else if (internal_stall) begin
            inst_reg    <= inst_reg;
        end
        */
        else begin
            pc_add4_reg     <= PC_add4_internal;
            alu_data_reg    <= i_mem_alu_data;
            inst_reg        <= i_mem_inst;
            wb_sel_reg      <= i_mem_wb_sel;
            rd_wren_reg     <= i_mem_rd_wren;

            ld_data_reg <= ld_data;
            /*
            io_ledr_reg <= io_ledr;
            io_ledg_reg <= io_ledg;
            io_hex0_reg <= io_hex0;
            io_hex1_reg <= io_hex1;
            io_hex2_reg <= io_hex2;
            io_hex3_reg <= io_hex3;
            io_hex4_reg <= io_hex4;
            io_hex5_reg <= io_hex5;
            io_hex6_reg <= io_hex6;
            io_hex7_reg <= io_hex7;
            io_lcd_reg  <= io_lcd;
            */
            insn_vld_reg <= i_mem_insn_vld;
            pc_debug_reg <= i_mem_pc;
            ctrl_reg     <= i_mem_ctrl;
        end
    end

    assign o_mem_pc_add4_wb     = pc_add4_reg;
    assign o_mem_alu_data_wb    = alu_data_reg;
    assign o_mem_inst_wb        = inst_reg;
    assign o_mem_wb_sel_wb      = wb_sel_reg;
    assign o_mem_rd_wren_wb     = rd_wren_reg;
    assign o_mem_ld_data_wb     = ld_data;//_reg;
    assign o_mem_io_ledr_wb     = io_ledr;
    assign o_mem_io_ledg_wb     = io_ledg;
    assign o_mem_io_hex0_wb     = io_hex0; 
    assign o_mem_io_hex1_wb     = io_hex1;
    assign o_mem_io_hex2_wb     = io_hex2;   
    assign o_mem_io_hex3_wb     = io_hex3; 
    assign o_mem_io_hex4_wb     = io_hex4; 
    assign o_mem_io_hex5_wb     = io_hex5; 
    assign o_mem_io_hex6_wb     = io_hex6;   
    assign o_mem_io_hex7_wb     = io_hex7; 
    assign o_mem_io_lcd_wb      = io_lcd; 
    assign o_mem_rd_addr_fwd    = i_mem_inst[11:7]; // o_mem_rd_addr_fwd cho forward
    assign o_mem_insn_vld_wb    = insn_vld_reg;
    assign o_mem_pc_debug       = pc_debug_reg;
    assign o_mem_ctrl           = ctrl_reg;
endmodule
`endif
