`ifndef WRITEBACK_CYCLE
`define WRITEBACK_CYCLE

`include "mux3_1.sv"

module writeback_cycle(
    //input logic         i_clk,
    //input logic         i_reset,

    //Input instruction
    input logic [31:0]  i_wb_inst,

    //Input từ Execute data writeback
    input logic [31:0]  i_wb_pc_add4,
    input logic [31:0]  i_wb_alu_data,
       
    input logic [31:0]  i_wb_ld_data,   
    /*Input from Execute LSU dataa
    input logic [31:0]  i_wb_ld_data,
    input logic [31:0]  i_wb_io_ledr, 
    input logic [31:0]  i_wb_io_ledg,
    input logic [6:0]   i_wb_io_hex0, 
    input logic [6:0]   i_wb_io_hex1, 
    input logic [6:0]   i_wb_io_hex2,   
    input logic [6:0]   i_wb_io_hex3, 
    input logic [6:0]   i_wb_io_hex4, 
    input logic [6:0]   i_wb_io_hex5, 
    input logic [6:0]   i_wb_io_hex6,   
    input logic [6:0]   i_wb_io_hex7, 
    input logic [31:0]  i_wb_io_lcd,
    */

    //Input control signals
    input logic [1:0]   i_wb_wb_sel,
    input logic         i_wb_rd_wren,

    //Output data writeback
    output logic [31:0] o_wb_data_wb,
    output logic [4:0]  o_wb_rd_addr,
    output logic        o_wb_rd_wren,
    //Output for forwarding
    output logic [4:0]  o_wb_rd_data_fwd
);
    mux_3_1 mux_3_1_at_writeback (
        .data_0_i      (i_wb_ld_data), 
        .data_1_i      (i_wb_alu_data), 
        .data_2_i      (i_wb_pc_add4), 
        .sel_i         (i_wb_wb_sel  ), 
        .data_out_o    (o_wb_data_wb )
    );
    assign o_wb_rd_addr = i_wb_inst[11:7];
    assign o_wb_rd_wren = i_wb_rd_wren;

    assign o_wb_rd_data_fwd = i_wb_inst[11:7];

endmodule
`endif