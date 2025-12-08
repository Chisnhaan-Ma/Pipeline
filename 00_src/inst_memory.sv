`ifndef INST_MEMORY
`define INST_MEMORY
module inst_memory (
  output logic [31:0] o_rdata,
  input  logic [31:0] i_addr
);

  logic [31:0] imem [0:2048];
  initial begin
    $readmemh("D:/HCMUT/Year_2025_2026/251/Conmputer_Organization/milestone_3/00_src/Test_Inst_Pl.dump",imem);
    //D:/HCMUT/Year_2025_2026/251/Conmputer_Organization/milestone_3/00_src/Test_Inst_Pl.dump
  end
  always @(*) begin
      o_rdata = imem[i_addr[31:2]];  
  end
endmodule
`endif
