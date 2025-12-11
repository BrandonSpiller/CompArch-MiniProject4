`include "ALU.sv"
`include "Program_Counter.sv"
`include "Register.sv"
`include "Controller.sv"
`include "memory.sv"

//Top module instantiates all modules, connects the datapaths through muxes and wires, and...

module top(  
    input logic     clk, 
    input logic     SW, //maybe remove
    input logic     BOOT, //maybe remove
    output logic    led,
    output logic    red,
    output logic    green,
    output logic    blue
);
    //DataPath Connections
    logic [31:0] SrcA_toALU;
    logic [31:0] SrcB_toALU;
    logic [3:0] ALUcontrol_Signal;
    logic [31:0] ALUresult;
    logic zero;

    logic [31:0] pc_next;
    logic enable;
    logic [31:0] pc;

    logic [4:0] Address1;
    logic [4:0] Address2;
    logic [4:0] Address3;
    logic WriteEnable;
    logic [31:0] WriteData;
    logic [31:0] ReadData1;
    logic [31:0] ReadData2;

    logic [31:0] instr;
    logic [31:0] ImmExt;
    logic [6:0] opcode;
    logic [1:0] ResultSrc;
    logic MemWrite;
    logic ALUSrc;
    logic RegWrite;
    logic Branch;
    logic Jump;
    logic AdrSrc;
    logic PCWrite;
    logic IRWrite;
    logic [1:0] SrcA_mux_signal;
    logic [1:0] SrcB_mux_signal;
    logic [1:0] ResultSrc_fsm;
    logic RegWrite_fsm;
    logic MemWrite_fsm;
    logic Branch_fsm;

    logic [2:0] funct3;
    logic dmem_wren;
    logic [31:0] dmem_address;
    logic [31:0] dmem_data_in;
    logic [31:0] imem_address;
    logic [31:0] imem_data_out;
    logic [31:0] dmem_data_out;

    //Current State Registers
    logic [31:0] Instr_reg;
    logic [31:0] Data_reg;
    logic [31:0] ALUOut_reg;
    logic [31:0] RD1_reg;
    logic [31:0] RD2_reg;
    logic [31:0] OldPC_reg;

    //Instantiate all modules

    //instance ALU
    ALU u1 (
        .SrcA           (SrcA_toALU),
        .SrcB           (SrcB_toALU),
        .ALUcontrol     (ALUcontrol),
        .ALUresult      (ALUresult),
        .zero           (zero)
    );

    //instance Program_Counter
    Program_Counter u2 (
        .clk            (clk),
        .pc_next        (pc_next),
        .enable         (enable),
        .reset          (reset),
        .pc             (pc)
    );

    //instance Register File
    Register u3 (
        .clk        (clk),
        .Address1   (Address1),
        .Address2   (Address2),
        .Address3   (Address3),
        .WriteEnable    (WriteEnable),
        .WriteData  (WriteData),
        .ReadData1  (ReadData1),
        .ReadData2  (ReadData2)
    );

    //Instance the Controller
    Controller u4 (
        .clk        (clk),
        //Instruction Relevant Variables
        .instr      (instr),
        .ImmExt     (ImmExt),
        .opcode     (opcode),
        .reset      (reset),
        //Main Decoder Relevant Variables
        .ResultSrc  (ResultSrc),
        .MemWrite   (MemWrite),
        .ALUsrc     (ALUSrc),
        .RegWrite   (RegWrite),
        .Branch     (Branch),
        .Jump       (Jump),
        //ALU Decoder Relevant Variables
        .ALUControl (ALUcontrol_Signal),
        //Instructions Relevant to FSM
        .AdrSrc     (AdrSrc),
        .PCWrite    (PCWrite),
        .IRWrite    (IRWrite),
        .SrcA       (SrcA_mux_signal),
        .SrcB       (SrcB_mux_signal),
        .ResultSrc_fsm  (ResultSrc_fsm),
        .RegWrite_fsm   (RegWrite_fsm),
        .MemWrite_fsm   (MemWrite_fsm),
        .Branch_fsm (Branch_fsm)
    );

    //Instance the memory file
    memory u5 (
        .clk        (clk),
        .funct3     (funct3),
        .dmem_wren  (dmem_wren),
        .dmem_address   (dmem_address),
        .dmem_data_in   (dmem_data_in),
        .imem_address   (imem_address),
        .imem_data_out  (imem_data_out),
        .dmem_data_out  (dmem_data_out),
        //.reset      (reset),
        .led        (led),      //Active-high PWM output for user LED 
        .red        (red),      //Active-high PWM output for user red LED
        .green      (green),    //Active-high PWM output for user green LED
        .blue       (blue)      //Active-high PWM output for user blue LED
    );

    
    //Assign register addresses
    assign Address1 = Instr_reg[19:15]; //register source 1 (rs1)
    assign Address2 = Instr_reg[24:20]; //register source 2 (rs2)
    assign Address3 = Instr_reg[11:7]; //register destination (rd)
    
    assign funct3 = Instr_reg[14:12];
    
    //Assign data memory signals?? what is going on here
    assign dmem_wren = MemWrite_fsm;
    assign dmem_data_in = RD2_reg;
    
    //Instruction Register (loads on IRWrite enabled)
    always_ff @(posedge clk) begin
        if (IRWrite) begin
            Instr_reg <= imem_data_out;
        end
    end
    
    //Data Register (stores memory read data)
    always_ff @(posedge clk) begin
        Data_reg <= dmem_data_out;
    end
    
    //ALU Output Register (stores ALU result)
    always_ff @(posedge clk) begin
        ALUOut_reg <= ALUresult;
    end
    
    //Register File Output Registers
    always_ff @(posedge clk) begin
        RD1_reg <= ReadData1;
        RD2_reg <= ReadData2;
    end
    
    //Old PC Register - stores PC value
    always_ff @(posedge clk) begin
        OldPC_reg <= pc;
    end
    
    //Pass instruction to controller
    assign instr = Instr_reg;
    
    //Memory Address Mux (AdrSrc)
    assign imem_address = AdrSrc ? ALUOut_reg : pc;
    assign dmem_address = ALUOut_reg;
    
    //SrcA Mux (for first ALU input)
    always_comb begin
        case (SrcA_mux_signal)
            2'b00: SrcA_toALU = pc; // PC
            2'b01: SrcA_toALU = RD1_reg; // Register rs1
            2'b10: SrcA_toALU = OldPC_reg; // Old PC
            default: SrcA_toALU = 32'd0;
        endcase
    end
    
    //SrcB Mux (forsecond ALU input)
    always_comb begin
        case (SrcB_mux_signal)
            2'b00: SrcB_toALU = RD2_reg; // Register rs2
            2'b01: SrcB_toALU = ImmExt; // Immediate
            2'b10: SrcB_toALU = 32'd4; // Constant 4
            default: SrcB_toALU = 32'd0;
        endcase
    end
    
    //ALU Control assignment
    assign ALUcontrol = ALUcontrol_Signal;
    
    //Result Mux (selects which data to write to register file)
    always_comb begin
        case (ResultSrc_fsm)
            2'b00: WriteData = ALUOut_reg; //ALU result
            2'b01: WriteData = Data_reg; //Memory data
            2'b10: WriteData = ALUOut_reg; //PC+4 for JAL/JALR instructions
            2'b11: WriteData = ImmExt;
            default: WriteData = 32'd0;
        endcase
    end
    
    //Register Write Enable
    assign WriteEnable = RegWrite_fsm;
    
    //PC Next Logic (look more into branches and jump instructions)
    always_comb begin
        if (Branch_fsm && ((funct3 == 3'b000 && zero) || (funct3 == 3'b001 && !zero))) begin  //BNE for some reason needs this
            //Branched
            pc_next = ALUOut_reg;
        end else if (Jump) begin
            //Jump (JAL/JALR)
            pc_next = ALUOut_reg;
        end else begin
            //Normal PC increment
            pc_next = ALUresult;
        end
    end
    
    //PC Enable (write to PC)
    assign enable = PCWrite || (Branch_fsm && zero) || Jump;


endmodule
//Instantiate all modules here
//Connect Datapath and put muxes here