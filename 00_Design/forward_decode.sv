module forward_decode (
    input logic [4:0] i_rd_addr_execute,
    input logic [4:0] i_rs1_addr_decode,
    input logic [4:0] i_rs2_addr_decode,
    input logic i_rd_wren_execute,
    //output logic o_sel_1,
    output logic o_sel_2
);
always_comb begin
    /*
    if ((i_rs1_addr_decode != 5'h0)&&(i_rs1_addr_decode == i_rd_addr_execute) && i_rd_wren_execute) begin
        o_sel_1 = 1'b1;
    end
    else begin
        o_sel_1 = 1'b0;
    end
    */
    if ((i_rs2_addr_decode != 5'h0)&&(i_rs2_addr_decode == i_rd_addr_execute) && i_rd_wren_execute) begin
        o_sel_2 = 1'b1;
    end
    else begin
        o_sel_2 = 1'b0;
    end
end
endmodule