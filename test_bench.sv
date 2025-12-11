`timescale 1ns/1ps
`include "top.sv"

    //ALSO MAKE AN INSTRUCTION MEMORY -- TAKE EXAMPLE FROM BRAD'S GITHUB REPO!

module test_bench;

    //Parameters
    logic clk = 0;
    logic SW = 0;
    logic BOOT = 0;
    logic reset = 0;
    
    //Output signals to be read on GTKWave
    logic led;
    logic red;
    logic green;
    logic blue;

    top u0 (
        .clk    (clk),
        .SW     (SW),
        .BOOT   (BOOT),
        .reset  (reset),
        .led    (led),
        .red    (red),
        .green  (green),
        .blue   (blue)
    );

    //Clock Cycle
    always begin
        #41.67 clk = ~clk;  //Toggle every 41.67ns for 12MHz clock
    end

    //Test sequence
    initial begin
        $dumpfile("test_bench.vcd");
        $dumpvars(0, test_bench);
        
        //Initialize
        reset = 0;
        #100;
        reset = 1;
    
        #10000; //time runnning here
        
        $finish;
    end

    // Display instruction execution
    always @(posedge clk) begin
        if (u0.IRWrite) begin
            $display("PC=%h  Instr=%h", u0.pc, u0.imem_data_out);
        end
    end

endmodule