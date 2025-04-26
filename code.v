module m_adder(w_in1, w_in2, w_out);
    input wire[31:0] w_in1, w_in2;
    output wire[31:0] w_out;
    assign w_out = w_in1 + w_in2;
endmodule

module m_cmp(w_in1, w_in2, w_out);
    input wire[31:0] w_in1, w_in2;
    output wire[31:0] w_out;
    assign w_out = (w_in1 == w_in2);
endmodule

module m_mux(w_in1, w_in2, w_s, w_out);
    input wire [31:0] w_in1, w_in2;
    input wire w_s;
    output wire [31:0] w_out;
    assign w_out = (w_s) ? w_in1 : w_in2;
endmodule

module m_am_imem(w_adr, w_ir);
    input wire [31:0] w_adr;
    output wire [31:0] w_ir;
    assign w_ir =
        (w_adr==0) ? {7'd0,5'd1,5'd0,3'd0,5'd1,7'b0110011} :
        (w_adr==4) ? {7'd0,5'd0,5'd1,3'd0,5'd1,7'b0110011} :
                     {7'd0,5'd1,5'd1,3'd0,5'd1,7'b0110011};
endmodule

module m_circuit1(w_clk);
    input wire w_clk;
    reg[31:0] r_pc = 0;
    wire[31:0] w_npc;
    m_adder m (32'h4, r_pc, w_npc);
    always @(posedge w_clk) r_pc <= w_npc;
endmodule

module m_circuit2(w_clk);
    input wire w_clk;
    reg[31:0] r_pc = 0;
    wire[31:0] w_npc;
    m_adder m(32'h4, r_pc, w_npc);
    always @(posedge w_clk) r_pc <= w_npc;
    wire[31:0] w_ir;
    m_am_imem m2(r_pc, w_ir);
endmodule

module m_top();
    reg r_clk = 0; initial #150 forever #50 r_clk = ~r_clk;
    m_circuit2 m(r_clk);
    initial #99 forever #100
        $display("%3d, %h %h", $time, m.r_pc, m.w_ir);
    initial #400 $finish;
endmodule

module m_proc1(w_clk);
    input wire w_clk;
    wire [31:0] w_npc, w_ir, w_r1, w_r2, w_rt;
    wire w_cmp1, w_cmp2;
    reg [31:0] r_pc = 1, r_x1 = 3;
    assign w_npc = 32'h4 + r_pc;
    assign w_ir = 
        (r_pc==0) ? {7'd0,5'd1,5'd0,3'd0,5'd1,7'b0110011} :
        (r_pc==4) ? {7'd0,5'd0,5'd1,3'd0,5'd1,7'b0110011} :
                     {7'd0,5'd1,5'd1,3'd0,5'd1,7'b0110011};
    assign w_cmp1 = (5'd1 == w_ir[19:15]);
    assign w_cmp2 = (5'd1 == w_ir[24:20]);
    assign w_r1 = (w_cmp1) ? r_x1 : 32'h0;
    assign w_r2 = (w_cmp2) ? r_x1 : 32'h0;
    assign w_rt = w_r1 + w_r2;
    always @(posedge w_clk) r_pc <= w_npc;
    always @(posedge w_clk) r_x1 <= w_rt;
endmodule

module m_RF(w_clk, w_ra1, w_ra2, w_rd1, w_wa, w_we, w_wd);
    input wire w_clk, w_we;
    input wire [4:0] w_ra1, w_ra2, w_wa;
    output wire [31:0] w_rd1, w_rd2;
    input wire [31:0] w_wd;
    reg[31:0] mem[0:31];
    assign w_rd1 = (w_ra1==5'd0) ? 32'd0 : mem[w_ra1];
    assign w_rd2 = (w_ra2==5'd0) ? 32'd0 : mem[w_ra2];
    always @(posedge w_clk) if (w_we) mem[w_wa] <= w_wd;
    always @(posedge w_clk) if (w_we & w_wa==5'd30) $finish;
    integer i; initial for (i=0;i<32;i=i+1) mem[i] = 0;
endmodule
