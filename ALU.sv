module ALU(
        input logic [31:0] SrcA,
        input logic [31:0] SrcB,
        input logic [3:0] ALUcontrol,
        output logic [31:0] ALUresult,
        output logic zero
    );

    always_comb begin
       case(ALUcontrol)
            4'b0000:begin //ADD
                zero = 0;
                ALUresult = SrcA + SrcB; 
            end

            4'b0001:begin //SUB
                zero = 0; 
                ALUresult = SrcA - SrcB;
            end

            4'b0010:begin //AND
                zero = 0;
                ALUresult = SrcA & SrcB;
            end

            4'b0011:begin //OR
                zero = 0;
                ALUresult = SrcA | SrcB;
            end

            4'b0100:begin //XOR
                zero = 0;
                ALUresult = SrcA ^ SrcB;
            end

            4'b0101:begin //SLL
                zero = 0;
                ALUresult = SrcA << SrcB[4:0];
            end

            4'b0110:begin //SRL
                zero = 0;
                ALUresult = SrcA >> SrcB[4:0];
            end

            4'b0111:begin //Shift Right
                zero = 0;
                ALUresult = $signed(SrcA) >>> SrcB[4:0];
            end

            4'b1000:begin //Set Less Than (SLT)
                zero = 0;
                ALUresult = ($signed(SrcA) < $signed(SrcB)) ? 32'd1 : 32'd0;
            end
            4'b1001:begin //Set Less Than Unsigned
                zero = 0;
                ALUresult = (SrcA < SrcB) ? 32'd1 : 32'd0;
            end

            default:begin
                zero = 0;
                ALUresult = 32'd0;
            end
            
       endcase

       zero = (ALUresult == 32'd0);
    end
endmodule