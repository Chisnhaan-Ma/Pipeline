module branch_taken (
	input logic i_br_less_mem,
	input logic i_br_equal_mem,
	input logic [31:0] i_inst_mem,
	output logic o_pc_sel,
	output logic flush);
	
	always @ (*) begin
        // B Type
        if (i_inst_mem[6:0] == 7'b1100011) begin
            case (i_inst_mem[14:12])
                3'b000: o_pc_sel = i_br_equal_mem;        // BEQ
                3'b001: o_pc_sel = ~i_br_equal_mem;       // BNE
                3'b100: o_pc_sel = i_br_less_mem;         // BLT
                3'b101: o_pc_sel = ~i_br_less_mem;        // BGE
                3'b110: o_pc_sel = i_br_less_mem;         // BLTU
                3'b111: o_pc_sel = ~i_br_less_mem;        // BGEU
                default: o_pc_sel= 1'bz;
            endcase
        end
        // JAL & JALR
        else if(i_inst_mem[6:0] == 7'b1101111) o_pc_sel = 1;
        else if(i_inst_mem[6:0] == 7'b1100111) o_pc_sel = 1;
        else o_pc_sel = 0;
    end
	assign flush = o_pc_sel;
endmodule