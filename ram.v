module m_ram(input [3:0] adr, input [3:0] wdata, input we, output [3:0] rdata);
    reg[3:0] mem[0:15];
    assign rdata=mem[adr];
    always @(posedge we) begin
        mem[adr] = wdata;
    end
endmodule

module m_seven_segment(input[3:0] idata, output [7:0] odata);
    parameter dot = 1'b1;
    function [7:0] LedDec;
        input [3:0] num;
        begin
            case (num)
            endcase
        end
    endfunction
endmodule
