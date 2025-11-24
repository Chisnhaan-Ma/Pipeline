`ifndef DATA_MEMORY
`define DATA_MEMORY
module data_memory(
    /////////- input -///////////
    input   logic       i_clk,
    input   logic       i_reset,
    input   logic [31:0]i_addr,
    input   logic [31:0]i_wdata,
    input   logic       i_wren,
    /////////- output -///////////
    output  logic  [31:0] o_rdata
);

    logic [31:0]memory[0:2048];

    always_ff @(posedge i_clk or posedge i_reset) begin
        // RESET
        if(i_reset) begin
            for (int i=0; i<512;i++) memory[i] <= 32'b0;
        end

        // i_wren = 1 for read
        else if (i_wren) begin
            memory[i_addr[31:2]] <= i_wdata;
        end
        else;
    end

    assign o_rdata = (i_reset == 1'b1) ? 32'h0: memory[i_addr[31:2]];


endmodule
`endif