module m_rom(input [3:0] adr, output[7:0] dat);
    reg [7:0] data;
    assign dat = data;
    always @(adr) begin
        case (adr)
            4'h0: data = 8'b1010001;

        endcase
    end
endmodule

module m_chattering(input clk, input sw_in, output sw_out);
    reg [15:0] cnt;
    reg swreg;
    wire iclk;

    assign sw_out = swreg;
    always @(posedge clk) begin
        cnt = cnt + 1;
    end
    assign iclk=cnt[15];

    always @(posedge iclk) begin
        swreg=sw_in;
    end
endmodule
