`ifndef PIPELINE_TB
`define PIPELINE_TB
`include "pipeline.sv"
`timescale 1ps/1ps

module Pipeline_Tb ();
    logic tb_clk;
    logic tb_reset;
    logic [31:0] tb_io_sw;
    pipelined pipeline_test (
        .i_clk(tb_clk),
        .i_reset(tb_reset),
        .i_io_sw(tb_io_sw)
    );
   // Clock generation
    always #5 tb_clk = ~tb_clk;
	
    initial begin
        $dumpfile("wave.vcd");      // file VCD sẽ sinh ra
        $dumpvars(0, Pipeline_Tb); //tên module testbench top-level
        tb_clk = 0;
        tb_reset = 1;    // Reset để PC = 0
        #3ps;
        force  tb_reset = 0; 
        force  tb_io_sw = 32'ha;
        #1000ps;
        $finish;  
    end
endmodule

`endif