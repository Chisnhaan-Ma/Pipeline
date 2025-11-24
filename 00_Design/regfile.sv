`ifndef REGFILE
`define REGFILE
module regfile (
  input  logic        i_clk,
  input  logic        i_reset,

  input  logic [4:0]  i_rs1_addr, 
  input  logic [4:0]  i_rs2_addr,

  input  logic [4:0]  i_rd_addr,
  input  logic [31:0] i_rd_data,
  input  logic        i_rd_wren,

  output logic [31:0] o_rs1_data, 
  output logic [31:0] o_rs2_data
);

  logic [31:0] Reg [31:0];

  // --- RESET + WRITE LOGIC ---
  always_ff @(posedge i_clk or posedge i_reset) begin
      if (i_reset) begin
        // Reset tất cả 32 thanh ghi về 0
        for (int i = 0; i < 32; i++) begin
          Reg[i] <= 32'h0;
        end 
      end
      else begin end
        // Chỉ ghi nếu có enable và không ghi vào x0 (r0)
      if (i_rd_wren && (i_rd_addr != 5'd0))begin
          Reg[i_rd_addr] <= i_rd_data;
      end
      else begin end
    end
    //o_rs1_data <= (i_rs1_addr == 5'd0) ? 32'h0 : Reg[i_rs1_addr];
    //o_rs2_data <= (i_rs2_addr == 5'd0) ? 32'h0 : Reg[i_rs2_addr];

    always_comb begin
      if ((i_rs1_addr == 5'd0))begin
        o_rs1_data = 32'b0;
      end
      else if (i_rs1_addr == i_rd_addr) begin
        o_rs1_data = i_rd_data;
      end
      else begin
        o_rs1_data = Reg[i_rs1_addr];
      end

      // rs2
      if ((i_rs2_addr == 5'd0))begin
        o_rs2_data = 32'b0;
      end
      else if (i_rs2_addr == i_rd_addr) begin
        o_rs2_data = i_rd_data;
      end
      else begin
        o_rs2_data = Reg[i_rs2_addr];
      end
    end
  
    //assign o_rs1_data = (i_rs1_addr == 5'd0) ? 32'h0 : Reg[i_rs1_addr];
    //assign o_rs2_data = (i_rs2_addr == 5'd0) ? 32'h0 : Reg[i_rs2_addr];

  /* --- READ LOGIC ---
   always_ff @(negedge i_clk) begin
    o_rs1_data = (i_rs1_addr == 5'd0) ? 32'h0 : Reg[i_rs1_addr];
    o_rs2_data = (i_rs2_addr == 5'd0) ? 32'h0 : Reg[i_rs2_addr];
  end
*/
  
endmodule
`endif