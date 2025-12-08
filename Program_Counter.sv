module Program_Counter(
        input logic clk,
        input logic [31:0] pc_next,
        input logic enable,
        input reset,
        output logic [31:0] pc
    );

    initial begin
        pc = 32'h00001000;
    end

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h00001000;
        end else if (enable) begin
            pc <= pc_next;
        end
    end
endmodule