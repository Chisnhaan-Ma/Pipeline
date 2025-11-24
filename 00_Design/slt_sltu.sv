`ifndef SLT_SLTU
`define SLT_SLTU 
`include "add_sub_32_bit.sv"
module slt_sltu (
    input  logic [31:0] A, B,  // Input A, B
    input  logic Sel,          // 0 = SLT (có dấu), 1 = SLTU (không dấu)
    output logic [31:0] Result); // Kết quả

    logic [31:0] diff_out;  // Kết quả phép trừ A - B
    logic carry_out;        // Carry/Borrow từ phép trừ

    add_sub_32_bit SUB(
        .A(A),
        .B(B), 
        .Sel(1'b1), 
        .Result(diff_out),
        .Cout(carry_out)
    );

    // So sánh
    always @(*) begin
        if (Sel == 1'b0) begin // SLT
            Result = {31'd0,diff_out[31]};  
        end else begin //SLTU
            Result = {31'd0,~carry_out};
        end
    end

endmodule
`endif 