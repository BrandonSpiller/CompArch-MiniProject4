module Register(
        input logic clk,
        input logic [4:0] Address1,
        input logic [4:0] Address2,
        input logic [4:0] Address3,
        input logic WriteEnable,
        input logic [31:0] WriteData,
        output logic [31:0] ReadData1,
        output logic [31:0] ReadData2
    );

    logic [31:0] register_array [31:0]; //an array of 32 32-bit registers

    initial begin //start with register x0 being 0
    register_array[0] = 32'd0;
    end

    always_comb begin //read address and make sure x0 is 0
        ReadData1 = (Address1 == 5'd0) ? 32'd0 : register_array[Address1];
        ReadData2 = (Address2 == 5'd0) ? 32'd0 : register_array[Address2];
    end

    always_ff @(posedge clk) begin
        if (WriteEnable && Address3 != 5'd0) begin
            register_array[Address3] <= WriteData;
        end
    end
endmodule