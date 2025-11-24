`include "brc.sv"
`include "mux3_1.sv"
`include "mux2_1.sv"
`include "alu.sv"
//chưa add insn valid///////
module execute_cycle(
    input logic         i_execute_clk,
    input logic         i_execute_reset,

    // Input từ Decode PC, inst giữ nguyên
    input logic [31:0]  i_execute_pc, 
    input logic [31:0]  i_execute_inst,

    // Input từ Decode, giữ hộ MEM control
    input logic         i_execute_lsu_wren,
    input logic [2:0]   i_execute_slt_sl,

    // Input từ Decode, giữ hộ Writeback control
    input logic [1:0]   i_execute_wb_sel,
    input logic         i_execute_rd_wren,

    // Input từ Decode được sử dụng trong Execute
    input logic         i_execute_asel,
    input logic         i_execute_bsel,
    input logic [31:0]  i_execute_rs1_data,
    input logic [31:0]  i_execute_rs2_data,
    input logic [31:0]  i_execute_imm_out,
    input logic [3:0]   i_execute_alu_op,
    input logic         i_execute_br_un,

    // Input từ MEM cho flush
    input logic         flush,

    // Input từ Forwarding control
    input logic [1:0]   i_execute_fwd_operand_a,
    input logic [1:0]   i_execute_fwd_operand_b,
    //input logic [1:0]   i_execute_fwd_rs2,
    //input logic [1:0]   i_execute_fwd_brc_a, 
    //input logic [1:0]   i_execute_fwd_brc_b,

    // Data forwarding từ MEM và WriteBack
    input logic [31:0]  i_execute_fwd_alu_data,
    input logic [31:0]  i_execute_fwd_wb_data,

    // Outputs tới MEM giá trị tính được từ Execute
    output logic        o_execute_br_equal_mem,
    output logic        o_execute_br_less_mem,
    output logic [31:0] o_execute_alu_data,

    // Output tới MEM các giá trị giữ hộ
    output logic [31:0] o_execute_pc_mem,
    output logic [31:0] o_execute_rs2_data_mem,
    output logic [31:0] o_execute_inst_mem,

    // Output to Execute MEM control
    output logic        o_execute_lsu_wren_mem,
    output logic [2:0]  o_execute_slt_sl_mem,

    // Output to Execute Writeback control
    output logic [1:0]  o_execute_wb_sel_mem,
    output logic        o_execute_rd_wren_mem,

    // Output rs1 và rs2 để forward
    output logic [31:0] o_execute_inst_fwd,

    //Output to decode để forward
    output logic  [31:0] o_execute_alu_data_decode
);

    logic [31:0] alu_data;
    logic [31:0] forward_a_out;
    logic [31:0] forward_b_out;

    logic [31:0] pc_reg;
    logic [31:0] inst_reg;
    logic [31:0] rs2_data;
    logic [31:0] rs2_data_reg;
    logic [31:0] alu_data_reg;
    logic br_equal_reg, br_equal;
    logic br_less_reg, br_less;
    logic wren_reg;
    logic [2:0] slt_sl_reg;
    logic [1:0] wb_sel_reg;
    logic rd_wren_reg;
    logic [31:0] asel_out, bsel_out;
    logic [31:0]brc_operand_a_out, brc_operand_b_out;
    assign o_execute_inst_fwd = i_execute_inst;

    /* Forwarding multiplexers
    mux_3_1 mux_forward_a_out (
        .sel_i(i_execute_fwd_operand_a),
        .data_0_i(asel_out),
        .data_1_i(i_execute_fwd_alu_data),
        .data_2_i(i_execute_fwd_wb_data),
        .data_out_o(forward_a_out)
    );

    mux_3_1 mux_forward_b_out (
        .sel_i(i_execute_fwd_operand_b),
        .data_0_i(bsel_out),
        .data_1_i(i_execute_fwd_alu_data),
        .data_2_i(i_execute_fwd_wb_data),
        .data_out_o(forward_b_out)
    );
    
    mux_3_1 mux_forward_rs2_data (
        .sel_i(i_execute_fwd_rs2),
        .data_0_i(i_execute_rs2_data),
        .data_1_i(i_execute_fwd_alu_data),
        .data_2_i(i_execute_fwd_wb_data),
        .data_out_o(rs2_data)
    );

    mux_3_1 mux_forward_brc_operand_a (
        .sel_i(i_execute_fwd_brc_a),
        .data_0_i(i_execute_rs1_data),
        .data_1_i(i_execute_fwd_alu_data),
        .data_2_i(i_execute_fwd_wb_data),
        .data_out_o(brc_operand_a_out)
    );

    mux_3_1 mux_forward_brc_operand_b (
        .sel_i(i_execute_fwd_brc_b),
        .data_0_i(i_execute_rs2_data),
        .data_1_i(i_execute_fwd_alu_data),
        .data_2_i(i_execute_fwd_wb_data),
        .data_out_o(brc_operand_b_out)
    );

    mux_2_1 op_a(
        .data_0_i(i_execute_rs1_data),
        .data_1_i(i_execute_pc),
        .sel_i(i_execute_asel),
        .data_out_o(asel_out)
    );
    mux_2_1 op_b(
        .data_0_i(i_execute_rs2_data),
        .data_1_i(i_execute_imm_out),
        .sel_i(i_execute_bsel),
        .data_out_o(bsel_out)
    );
    */
    mux_3_1 mux_forward_operand_a (
        .data_0_i       (i_execute_rs1_data),
        .data_1_i       (i_execute_fwd_alu_data),
        .data_2_i       (i_execute_fwd_wb_data),
        .sel_i          (i_execute_fwd_operand_a),
        .data_out_o     (forward_a_out)
    );

    mux_3_1 mux_forward_operand_b (
        .data_0_i       (i_execute_rs2_data),
        .data_1_i       (i_execute_fwd_alu_data),
        .data_2_i       (i_execute_fwd_wb_data),
        .sel_i          (i_execute_fwd_operand_b),
        .data_out_o     (forward_b_out)
    );

    mux_2_1 mux_asel (
        .data_0_i       (forward_a_out),
        .data_1_i       (i_execute_pc),
        .sel_i          (i_execute_asel),
        .data_out_o     (asel_out)
    );

    mux_2_1 mux_bsel (
        .data_0_i       (forward_b_out),
        .data_1_i       (i_execute_imm_out),
        .sel_i          (i_execute_bsel),
        .data_out_o     (bsel_out)
    );

    // Branch comparator
    brc brc_at_execute (
        .i_br_un        (i_execute_br_un),
        .i_rs1_data     (forward_a_out),
        .i_rs2_data     (forward_b_out),
        .o_br_equal     (br_equal),
        .o_br_less      (br_less)
    );

    // ALU
    alu alu_at_execute(
        .i_operand_a    (asel_out),
        .i_operand_b    (bsel_out), 
        .o_alu_data     (alu_data),
        .i_alu_op       (i_execute_alu_op)
    );

    // Pipeline registers
    always_ff @ (posedge i_execute_clk or posedge i_execute_reset) begin
        if (i_execute_reset) begin
            pc_reg          <= 32'd0;
            inst_reg        <= 32'd0;
            rs2_data_reg    <= 32'd0;
            alu_data_reg    <= 32'd0;
            wren_reg        <= 0;
            slt_sl_reg      <= 0;
            wb_sel_reg      <= 0;
            rd_wren_reg     <= 0;
            br_equal_reg    <= 0;
            br_less_reg     <= 0;    
        end 
		else if (flush) begin
			pc_reg       <= 32'd0;
            inst_reg     <= 32'd0;
            rs2_data_reg <= 32'd0;
            alu_data_reg <= 32'd0;
            wren_reg     <= 0;
            slt_sl_reg   <= 0;
            wb_sel_reg   <= 0;
            rd_wren_reg  <= 0;
            br_equal_reg <= 0;
            br_less_reg  <= 0;    
          end	

		else begin
            pc_reg       <= i_execute_pc;
            inst_reg     <= i_execute_inst;
            rs2_data_reg <= forward_b_out;
            alu_data_reg <= alu_data;
            wren_reg     <= i_execute_lsu_wren;
            slt_sl_reg   <= i_execute_slt_sl;
            wb_sel_reg   <= i_execute_wb_sel;
            rd_wren_reg  <= i_execute_rd_wren;
            br_equal_reg <= br_equal;
            br_less_reg  <= br_less;
        end
    end

    assign o_execute_alu_data       = alu_data_reg;
    assign o_execute_pc_mem         = pc_reg;
    assign o_execute_rs2_data_mem   = rs2_data_reg;
    assign o_execute_inst_mem       = inst_reg;
    assign o_execute_lsu_wren_mem   = wren_reg;
    assign o_execute_slt_sl_mem     = slt_sl_reg;
    assign o_execute_wb_sel_mem     = wb_sel_reg;
    assign o_execute_rd_wren_mem    = rd_wren_reg;
    assign o_execute_br_equal_mem   = br_equal_reg;
    assign o_execute_br_less_mem    = br_less_reg;

    assign o_execute_alu_data_decode = alu_data;
endmodule