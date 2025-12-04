`ifndef INST_MEMORY
`define INST_MEMORY
module inst_memory (
  output logic [31:0] o_rdata,
  input  logic [31:0] i_addr
);

  logic [31:0] imem [0:2048];
  initial begin
    $readmemh("/home/yellow/ctmt_l01_l02_4/workspace/milestone_3/02_test/isa_4b_ms2.hex",imem);
    
  end
  always @(*) begin
      o_rdata = imem[i_addr[31:2]];  
  end
endmodule
`endif
