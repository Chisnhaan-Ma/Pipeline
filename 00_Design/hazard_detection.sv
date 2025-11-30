module hazard_detection(
    input  logic [31:0] i_hazard_inst_execute,
    input  logic [4:0]  i_hazard_rs1_addr_decode,
    input  logic [4:0]  i_hazard_rs2_addr_decode,
    input  logic [1:0]  i_hazard_wb_sel_execute,        // Xác định nếu lệnh trước là load
    input  logic        i_hazard_rd_wren_execute,       // Có ghi thanh ghi không
    input  logic        i_stall_lsu,
    output logic        Stall
);
    logic [4:0] rd_addr_execute;

    // Tách rd từ instruction ở EX stage
    assign rd_addr_execute = i_hazard_inst_execute[11:7];

    always_comb begin
        // Nếu lệnh ở EX là load (WBSel = 2'b00) và ghi vào thanh ghi (i_hazard_rd_wren_execute)
        // và rd_addr_execute khớp với i_hazard_rs1_addr_decode hoặc i_hazard_rs2_addr_decode của lệnh hiện tại ở Decode
			if (i_stall_lsu | (i_hazard_wb_sel_execute == 2'b00 && i_hazard_rd_wren_execute == 1'b1) && (rd_addr_execute != 5'd0) && (rd_addr_execute == i_hazard_rs1_addr_decode || rd_addr_execute == i_hazard_rs2_addr_decode)) begin
				 Stall = 1'b1;
			end
			else Stall = 1'b0;

            
    end
endmodule