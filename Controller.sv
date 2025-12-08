module Controller(
    input clk
    //Instruction relevant variables
    input logic [31:0] instr,
    output logic [31:0] ImmExt, //extended immediate
    output logic [6:0] opcode,

    //Main Decoder Relevant Variables
    output loigc [1:0] ResultSrc,
    output logic MemWrite,
    output logic RegWrite,
    output logic ALUsrc,
    output logic RegWrite,
    output logic Branch,
    output logic Jump,
    output logic PCWrite,
    output logic AdrSrc,
    output logic IRWrite,

    //ALU Decoder Relevant Variables
    output logic [2:0] ALUcontrol,
    output logic [1:0] ALUOp,

    //Instructions Relevant to FSM
    input logic WriteData,
    input logic Address1,
);

    //Instruction relevant Variables
    loigc [6:0] funct7;
    loigc [2:0] funct3;
    logic [31:0] Imm; //immediate from instruction
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic [4:0] rd;

    //Main Decoder Relevant Variables
    logic [2:0] ImmSrc; //control signal for sign extender/Instruction Decoder

    //FSM Relevant Variables
    logic state;
    logic next_state;
    logic current_state;


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // BEGINNING OF INSTRUCTION DECODER + SIGN EXTENDOR                 BEGINNING OF INSTRUCTION DECODER + SIGN EXTENDOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    always_ff @(posedge clk) begin //maybe add some sort of trigger ontop of clk for when there is a new instruction
        ImmExt <= 32'd0;
        Imm <= 32'd0;
        funct7 <= 7'd0;
        funct3 <= 3'd0;
        rs1 <= 5'd0;
        rs2 <= 5'd0;
        rd <= 5'd0;
        opcode = instr[6:0];
    end

    always_comb
        begin
            case (opcode)
                7'b0110011: begin //R-type
                    rs1 = instr[19:15];
                    rs2 = instr[24:20];
                    rd = instr[11:7];
                    funct7 = instr[31:25];
                    funct3 = instr[14:12];

                    ImmExt = 32'd0; // No ImmExt for R-type
                end

                7'b0000011: begin //I-type (Loads)
                    rs1 = instr[19:15];
                    rd = instr[11:7];
                    funct3 = instr[14:12];

                    Imm = instr[31:20];
                    ImmExt = {{20{Imm[11]}}, Imm[11:0]}; //takes the MSB of Imm and makes it the MSBs of ImmExt
                end

                7'b0010011: begin //I-type (ALU) 
                    rs1 = instr[19:15];
                    rd = instr[11:7];
                    funct3 = instr[14:12];

                    case (funct3)
                        3'b001: begin // SLLI
                            funct7 = instr[31:25];
                            ImmExt = {27'd0, instr[24:20]};
                        end
                        
                        3'b101: begin // SRLI or SRAI 
                            funct7 = instr[31:25]; 
                            ImmExt = {27'd0, instr[24:20]};
                        end
                        
                        default: begin // ADDI, SLTI, SLTIU, XORI, ORI, ANDI
                            Imm = instr[31:20];
                            ImmExt = {{20{instr[31]}}, instr[31:20]}; // Sign-extend all 12 bits
                        end
                    endcase
                end

                7'b1100111: begin //I-type (JALR)
                    rs1 = instr[19:15];
                    rd = instr[11:7];
                    funct3 = instr[14:12];

                    Imm = instr[31:20];
                    ImmExt = {{20{Imm[11]}}, Imm[11:0]};
                end

                7'b0100011: begin //S-type (Stores)
                    rs1 = instr[19:15];
                    rs2 = instr[24:20];
                    funct3 = instr[14:12];
                
                    Imm[11:5] = Instr[31:25];
                    Imm[4:0] = Instr[11:7];
                    Imm_Ext = {{20{Imm[11]}}, Imm[11:0]};
                end

                7'b1100011: begin //B-type (Branches)
                    rs1 = instr[19:15];
                    rs2 = instr[24:20];
                    funct3 = instr[14:12];

                    Imm[12] = Instr[31];
                    Imm[10:5] = Instr[30:25];
                    Imm[4:1] = Instr[11:8];
                    Imm[11] = Instr[7];
                    Imm_Ext = {{19{Imm[12]}}, Imm[12:1], 1'b0};
                end

                7'b0110111: begin //U-type (LUI)
                    rd = instr[11:7];

                    Imm = Instr[31:12];
                    Imm_Ext = {Imm[31:12], 12'b0}
                end

                7'b0010111: begin //U-type (AUIPC)
                    rd = instr[11:7];

                    Imm = Instr[31:12];
                    Imm_Ext = {Imm[31:12], 12'b0}
                end

                7'b1101111: begin //J-type (JAL)
                    rd = instr[11:7];

                    Imm[20] = instr[31];
                    Imm[10:1] = instr[30:21];
                    Imm[11] = instr[20];
                    Imm[19:12] = instr[19:12];
                    Imm_Ext = {{11{Imm[20]}}, Imm[20:1], 1'b0};
                end

                default: begin
                    ImmExt <= 32'd0;
                    Imm <= 32'd0;
                    funct7 <= 7'd0;
                    funct3 <= 3'd0;
                    rs1 <= 5'd0;
                    rs2 <= 5'd0;
                    rd <= 5'd0;
                end
            endcase
        end

    //////////////////////////////////////////////////////////////////////////
    // BEGINNING OF MAIN DECODER          BEGINNING OF MAIN DECODER
    //////////////////////////////////////////////////////////////////////////

    //should be a giant opcode case statement that does different things based on the opcode
    always_comb begin
        //set everything to 0 by default
        RegWrite <= 1'b0;
        MemWrite <= 1'b0;
        ALUSrc <= 1'b0;
        ResultSrc <= 2'b00;
        Branch <= 1'b0;
        Jump <= 1'b0;
        ALUOp <= 2'b00;

        case(opcode)
            7'b0110011: begin //R-type Instructions(ADD, SUB, AND, OR, SLL, SLT, SLTU, XOR, SRL, SRA)
                RegWrite = 1'b1;
                ALUSrc = 1'b0;      //rs2
                ResultSrc = 2'b00;  //Write ALU result
                ALUOp = 2'b10;      //R-type ALU operation
            end
            
            7'b0010011: begin //I-type ALU Instructions (ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI)
                RegWrite = 1'b1;
                ALUSrc = 1'b1;      //immediate
                ResultSrc = 2'b00;  //Write ALU result
                ALUOp = 2'b10;      //I-type ALU operation
            end
            
            7'b0000011: begin //I-type Load Instructions(LW, LH, LB, LBU, LHU)
                RegWrite = 1'b1;
                ALUSrc = 1'b1;      //immediate
                ResultSrc = 2'b01;  //Write memory data
                ALUOp = 2'b00;      //Add for address calculation
            end
            
            7'b0100011: begin //S-type Instructions (Store) (SW, SH, SB)
                MemWrite = 1'b1;
                ALUSrc = 1'b1;      //immediate
                ALUOp = 2'b00;      //Add for address calculation
            end
            
            7'b1100011: begin //B-type Instructions (Branch) (BEQ, BNE, BLT, BGE, BLTU, BGEU)
                Branch = 1'b1;
                ALUSrc = 1'b0;      //Compare rs1 and rs2
                ALUOp = 2'b01;      //Branch comparison
            end
            
            7'b1101111: begin //JAL
                RegWrite = 1'b1;
                Jump = 1'b1;
                ResultSrc = 2'b10;  // Write PC+4
            end
            
            7'b1100111: begin //JALR
                RegWrite = 1'b1;
                Jump = 1'b1;
                ALUSrc = 1'b1; //Use immediate
                ResultSrc = 2'b10; //Write PC+4
                ALUOp = 2'b00; //Add for target address
            end
            
            7'b0110111: begin //LUI
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                ResultSrc = 2'b11; //Write immediate directly
            end
            
            7'b0010111: begin // AUIPC
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                ResultSrc = 2'b00; // Write ALU result
                ALUOp = 2'b00; // Add
            end
            
            default: begin
                // All signals remain at default (0)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            end
        endcase
    end

    //////////////////////////////////////////////////////////////////////////
    // BEGINNING OF ALU DECODER      BEGINNING OF ALU DECODER
    //////////////////////////////////////////////////////////////////////////

    // ALU Decoder outputs ALUControl based on ALUOp, funct3, and sometimes funct7
always_comb begin
    case(ALUOp)
        2'b00: begin 
            ALUControl = 4'b0000; // ADD 
        end
        
        2'b01: begin // Branch operations
            case(funct3)
                3'b000: begin
                    ALUControl = 4'b0001; // BEQ (subtract)
                end

                3'b001: begin 
                    ALUControl = 4'b0001; // BNE (subtract)
                end

                3'b100: begin
                    ALUControl = 4'b1000; // BLT (set less than)
                end

                3'b101: begin 
                    ALUControl = 4'b1000; // BGE (set less than)
                end

                3'b110: begin 
                    ALUControl = 4'b1001; // BLTU (set less than unsigned)
                end

                3'b111: begin ALUControl = 4'b1001; // BGEU (set less than unsigned)
                end 
            
                default: begin
                    ALUControl = 4'b0000;
                end

            endcase
        end
        
        2'b10: begin // R-type and I-type ALU operations
            case(funct3)
                3'b000: begin
                    if (opcode == 7'b0110011 && funct7[5]) // SUB
                        ALUControl = 4'b0001;
                    else // ADD or ADDI
                        ALUControl = 4'b0000;
                end
                3'b001: begin 
                    ALUControl = 4'b0101; // SLL or SLLI
                end

                3'b010: begin 
                    ALUControl = 4'b1000; // SLT or SLTI
                end

                3'b011: begin
                    ALUControl = 4'b1001; // SLTU or SLTIU
                end

                3'b100: begin 
                    ALUControl = 4'b0100; // XOR or XORI
                end 

                3'b101: begin
                    if (funct7[5]) // SRA or SRAI
                        ALUControl = 4'b0111;
                    else // SRL or SRLI
                        ALUControl = 4'b0110;
                end

                3'b110: begin
                    ALUControl = 4'b0011; // OR or ORI
                end

                3'b111: begin 
                    ALUControl = 4'b0010; // AND or ANDI
                end

                default: begin 
                    ALUControl = 4'b0000;
                end

            endcase
        end
        
        default: begin 
            ALUControl = 4'b0000;
        end

    endcase
end

    //////////////////////////////////////////////////////////////////////////
    // BEGINNING OF MAIN FSM          BEGINNING OF MAIN FSM
    //////////////////////////////////////////////////////////////////////////

    // Encode States
    typedef enum logic [3:0] {
        Fetch = 4'b0000, //S0
        Decode = 4'b0001, //S1
        MemAdr = 4'b0010, //S2
        MemRead = 4'b0011, //S3
        MemWB = 4'b0100, //S4
        MemWrite_state = 4'b0101, //S5
        ExecuteR = 4'b0110, //S6
        ALUWb = 4'b0111, //S7
        ExecuteI = 4'b1000, //S8
        jal = 4'b1001, //S9
        beq = 4'b1010, //S10

    } state_t;

    state_t current_state, next_state;

    // State register
    always_ff @(posedge clk or negedge reset) begin
        if (!reset)
            current_state <= Fetch;
        else
            current_state <= next_state;
    end

    // Next state logic
    always_comb begin
        AdrSrc = 0;
        IRWrite = 0;
        RegWrite = 0;
        SrcA = 0;
        SrcB = 2'b00;
        ResultSrc = 2'b00;
        MemWrite = 0;
        PCWrite = 0;
        Branch = 0;
        ALUOp = 2'b00;


        case (current_state)
            Fetch: begin
                //Next State
                next_state = Decode; 
                
                //Action
                AdrSrc = 0;
                IRWrite = 1;
                SrcA = 0; //PC
                SrcB = 2'b10; //4
                ALUOp = 2'b00; //Add
                PCWrite = 1;
                ResultSrc = 2'b10; //ALUResult to PC
                
            end 

            Decode: begin
                //Next State
                case (opcode)
                    7'b0000011: begin 
                        next_state = MemAdr; // Load (lw, lh, lb, etc.)
                    end

                    7'b0100011: begin 
                        next_state = MemAdr; // Store (sw, sh, sb)
                    end 

                    7'b0110011: begin
                        next_state = ExecuteR; // R-type (add, sub, and, or, etc.)
                    end

                    7'b0010011: begin
                        next_state = ExecuteI; // I-type ALU (addi, slti, andi, etc.)
                    end

                    7'b1101111: begin
                        next_state = jal; // JAL
                    end

                    7'b1100011: begin
                        next_state = beq; // Branch (beq, bne, blt, etc.)
                    end 

                    default: begin 
                        next_state = Fetch;
                    end 
                endcase 

                //Action
                SrcA = 0; //PC
                SrcB = 2'b01; //ImmExt
                ALUOp = 2'b00; // Add (for branch target calculation) 
            end

            MemAdr: begin
                //Next State
                if (opcode == 7'b0000011)// Load
                    next_state = MemRead;
                else if (opcode == 7'b0100011) // Store
                    next_state = MemWrite;
                else
                    next_state = Fetch;
                
                //Action
                SrcA = 1; //Register data
                SrcB = 2'b01; //ImmExt
                ALUOp = 2'b00; //Add
            end

            MemRead: begin
                next_state = MemWB;

                AdrSrc = 1; //Use ALUResult as memory address
                ResultSrc = 2'b00; //Memory data
            end 

            MemWB: begin 
                next_state = Fetch;

                ResultSrc = 2'b00; //Memory data
                RegWrite = 1; //Write to register
            end 

            MemWrite: begin 
                next_state = Fetch;

                AdrSrc = 1; //Use ALUResult as memory address
                MemWrite_sig = 1; //Enable memory write
            end 

            ExecuteR: begin
                next_state = ALUWb;

                SrcA = 1; //Register rs1
                SrcB = 2'b00; //Register rs2
                ALUOp = 2'b10; //R-type operation (determined by funct3/funct7)
            end 

            ExecuteI: begin 
                next_state = ALUWb;

                SrcA = 1; // Register rs1
                SrcB = 2'b01; // ImmExt
                ALUOp = 2'b10; // I-type operation (determined by funct3)
            end 

            ALUWb: begin
                next_state = Fetch;

                ResultSrc = 2'b00;// ALUResult
                RegWrite = 1; //Write to register
            end 

            jal: begin
                next_state = Fetch;

                ResultSrc = 2'b00;// PC+4 (saved in ALUOut from Decode)
                RegWrite = 1; //Write return address to rd
                PCWrite = 1; //Update PC to jump target
            end

            beq: begin
                next_state = Fetch;

                Branch = 1; // Enable branch logic
                SrcA = 1; // Register rs1
                SrcB = 2'b00;//Register rs2
                ALUOp = 2'b01; //Subtract (for comparison)
            end

            default: begin
                next_state = Fetch;
            end
        endcase
    end

    // Output logic
    assign state = current_state;

endmodule