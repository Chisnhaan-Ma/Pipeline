`include "brc.sv"
`include "alu.sv"
`include "control_unit.sv"
`include "imm_gen.sv"
`include "inst_memory.sv"
`include "lsu.sv"
`include "mux2_1.sv"
`include "pc.sv"
`include "regfile.sv"
module singlecycle(
    input   logic         i_clk,
    input   logic         i_reset,
    output  logic [31:0]    o_pc_debug,
    output  logic           o_insn_vld
    /*
    input   logic         i_io_sw,
    output  logic [31:0]    o_pc_debug,
    output  logic           o_insn_vld,
    output  logic [31:0]    o_io_ledr,
    output  logic [31:0]    o_io_ledg,
    output  logic  [6:0]    o_io_hex0, 
	output  logic  [6:0]    o_io_hex1, 
	output  logic  [6:0]    o_io_hex2, 
	output  logic  [6:0]    o_io_hex3,
	output  logic  [6:0]    o_io_hex4,
	output  logic  [6:0]    o_io_hex5,
	output  logic  [6:0]    o_io_hex6,
	output  logic  [6:0]    o_io_hex7
    */
);
	///- processor internal signal -///

    //For pc
	logic [31:0] pc_data_in;
	logic [31:0] pc_data_out;
    logic [31:0] pc_add4_out;
    logic        pc_sel;

    //For inst memory
	logic [31:0] inst;

    //For regfile
    logic [31:0] rs1_data;
	logic [31:0] rs2_data;
    logic [31:0] rd_data;
    logic        rd_wren;

    
    logic [4:0] rd_addr;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr; 

    //For brach control
    logic br_equal;
	logic br_less;
	logic br_un;

    //For operand selection
	logic asel;
	logic bsel;

    //For alu
    logic [31:0] operand_a;
	logic [31:0] operand_b;
    logic [3:0]  alu_op;
	logic [31:0] alu_data;

    //For immediate generator
    logic [31:0] imm_out;
    logic [2:0]  imm_sel;

    //For lsu
    logic [31:0] rdata;
    logic [2:0]  load_type;
	logic [31:0] load_result;
    logic [2:0]  slt_sl;
    logic        wren;

    //For write back
    logic [1:0] wb_sel;
    //logic [31:0]wb_data;

    logic inst_vld;

    assign rs1_addr[4:0] = inst[19:15];
    assign rs2_addr[4:0] = inst[24:20];
    assign rd_addr[4:0] =  inst[11:7];
    

    pc pc_top (
        .i_clk(i_clk),
        .i_reset(i_reset),
        .i_pc_data_in(pc_data_in),
        .o_pc_data_out(pc_data_out)
        );

    add_sub_32_bit pc_add4 (
        .A(pc_data_out),
        .B(32'd4),
        .Sel(1'b0),
        .Result(pc_add4_out));

    inst_memory inst_memory_top (
        .i_addr(pc_data_out),
        .o_rdata(inst)
        );
    
    imm_gen imm_gen_top(
        .i_imm_sel(imm_sel),
        .i_inst(inst),
        .o_imm_out(imm_out));

    alu alu_top(
        .i_operand_a(operand_a),
        .i_operand_b(operand_b),
        .i_alu_op(alu_op),
        .o_alu_data(alu_data));

    lsu lsu_top(
        .i_clk(i_clk),
        .i_reset(i_reset),
        .i_addr(alu_data),
        .i_wdata(rs2_data),
        .i_wren(wren),
        .i_load_type(load_type),
        .o_load_result(load_result));

    mux_3_1 mux3_1_top_writeback(
        .sel_i(wb_sel),
        .data_0_i(load_result),
        .data_1_i(alu_data),
        .data_2_i(pc_add4_out),
        .data_out_o(rd_data));

    brc brc_top(
        .i_rs1_data(rs1_data),
        .i_rs2_data(rs2_data),
        .i_br_un(br_un),
        
        .o_br_equal(br_equal),
        .o_br_less(br_less));

    control_unit control_unit_top(
        .i_inst(inst),
        .i_br_equal(br_equal),
        .i_br_less(br_less),

        .o_insn_vld_ctrl(insn_vld),
        .o_pc_sel(pc_sel),
        .o_imm_sel(imm_sel),
        .o_rd_wren(rd_wren),
        .o_br_un(br_un),
        .o_asel(asel),
        .o_bsel(bsel),
        .o_alu_op(alu_op),
        .o_wren(wren),
        .o_slt_sl(slt_sl),
        .o_load_type(load_type),
        .o_wb_sel(wb_sel)
    );
    
    regfile regfile_top(
        .i_clk(i_clk),
        .i_reset(i_reset),

        .i_rs1_addr(rs1_addr),
        .i_rs2_addr(rs2_addr),

        .i_rd_addr(rd_addr),
        .i_rd_data(rd_data),
        .i_rd_wren(rd_wren),

        .o_rs1_data(rs1_data),
        .o_rs2_data(rs2_data)
    );
    mux_2_1 mux2_1_asel_top(
        .data_0_i(rs1_data),
        .data_1_i(pc_data_out),
        .sel_i(asel),
        .data_out_o(operand_a));
    
    mux_2_1 mux2_1_bsel_top(
        .data_0_i(rs2_data),
        .data_1_i(imm_out),
        .sel_i(bsel),
        .data_out_o(operand_b));

    mux_2_1 mux2_1_pc_sel(
        .data_0_i(pc_add4_out),
        .data_1_i(alu_data),
        .sel_i(pc_sel),
        .data_out_o(pc_data_in));

    always_ff @( posedge i_clk or posedge i_reset ) begin
        if(i_reset) begin    
            o_insn_vld <= insn_vld;
            o_pc_debug <= 32'b0;
        end
        else begin
            o_insn_vld <= insn_vld;
            o_pc_debug <= pc_data_out;
        end
    end
endmodule