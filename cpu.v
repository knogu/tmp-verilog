module m_adder(w_in1, w_in2, w_out);
    input wire [31:0] w_in1, w_in2;
    output wire [31:0] w_out;
    assign w_out = w_in1 + w_in2;
endmodule

module m_mux(w_in1, w_in2, w_sel, w_out);
    input wire [31:0] w_in1, w_in2;
    input wire w_sel;
    output wire [31:0] w_out;
    assign w_out = (w_sel) ? w_in2 : w_in1;
endmodule

module m_RF(w_clk, w_ra1, w_ra2, w_rd1, w_rd2, w_wa, w_we, w_wd);
    input wire w_clk, w_we;
    input wire [4:0] w_ra1, w_ra2, w_wa;
    output wire [31:0] w_rd1, w_rd2;
    input wire [31:0] w_wd;
    reg [31:0] mem [0:31];
    assign w_rd1 = (w_ra1 == 5'd0) ? 32'd0 : mem[w_ra1];
    assign w_rd2 = (w_ra2 == 5'd0) ? 32'd0 : mem[w_ra2];
    always @(posedge w_clk) if (w_we) mem[w_wa] = w_wd;
    always @(posedge w_clk) if (w_we & w_wa == 5'd30) $finish;
    integer i; initial for (i=0; i<32; i=i+1) mem[i] = 0;
endmodule

module m_am_imem(w_pc, w_insn);
    input  wire [31:0] w_pc;
    output wire [31:0] w_insn;
    reg [31:0] mem [0:63];
    assign w_insn = mem[w_pc[7:2]];
    integer i; initial for (i=0; i<64; i=i+1) mem[i] = 32'd0;
endmodule

module m_get_type(opcode5, r, i, s, b, u, j);
    input wire [4:0] opcode5;
    output wire r, i, s, b, u, j;
    assign j = (opcode5 == 5'b11011);
    assign b = (opcode5 == 5'b11000);
    assign s = (opcode5 == 5'b01000);
    assign r = (opcode5 == 5'b01100);
    assign u = (opcode5 == 5'b01101 || opcode5 ==5'b00101);
    assign i = ~(j | b | s | r | u);
endmodule

module m_get_imm(ir, i, s, b, u, j, w_imm);
    input wire [31:0] ir;
    input wire i, s, b, u, j;
    output wire [31:0] w_imm;
    assign w_imm = (i) ? { {20{ir[31]}}, ir[31:20] } :
                   (s) ? { {20{ir[31]}}, ir[31:25], ir[11:7] } :
                   (b) ? { {20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} :
                   (u) ? { ir[31:12], 12'b0 } :
                   (j) ? { {12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0 } :
                   0;
endmodule

module m_gen_imm(w_ir, w_imm, w_r, w_i, w_s, w_b, w_u, w_j, w_ld);
    input  wire [31:0] w_ir;
    output wire [31:0] w_imm;
    output wire w_r, w_i, w_s, w_b, w_u, w_j, w_ld;
    m_get_type m1 (w_ir[6:2], w_r, w_i, w_s, w_b, w_u, w_j);
    m_get_imm m2 (w_ir, w_i, w_s, w_b, w_u, w_j, w_imm);
    assign w_ld = (w_ir[6:2]==0);
endmodule

module m_am_dmem(w_clk, w_adr, w_we, w_wd, w_rd);
    input  wire w_clk, w_we;
    input  wire [31:0] w_adr, w_wd;
    output wire [31:0] w_rd;
    reg [31:0] mem [0:63];
    assign w_rd = mem[w_adr[7:2]];
    always @(posedge w_clk) if (w_we) mem[w_adr[7:2]] <= w_wd;
    integer i; initial for (i=0; i<64; i=i+1) mem[i] = 32'd0;
endmodule

module m_proc4(w_clk);
    input wire w_clk;
    wire [31:0] w_npc, w_ir, w_imm, w_r1, w_r2, w_s2, w_rt;
    wire [31:0] w_alu, w_ldd;
    reg [31:0] r_pc = 0;
    wire w_r, w_i, w_s, w_b, w_u, w_j, w_ld;
    m_adder m2 (32'h4, r_pc, w_npc);
    m_am_imem m3 (r_pc, w_ir);
    m_gen_imm m4 (w_ir, w_imm, w_r, w_i, w_s, w_b, w_u, w_j, w_ld);
    m_RF m5 (w_clk, w_ir[19:15], w_ir[24:20], w_r1, w_r2, w_ir[11:7], !w_s, w_rt);
    m_mux m6 (w_r2, w_imm, !w_r, w_s2);
    m_adder m7 (w_r1, w_s2, w_alu);
    m_am_dmem m8 (w_clk, w_alu, w_s, w_r2, w_ldd);
    m_mux m9 (w_alu, w_ldd, w_ld, w_rt);
    always @(posedge w_clk) r_pc <= w_npc;
endmodule

module m_top();
    reg r_clk=0; initial #150 forever #50 r_clk = ~r_clk;
    m_proc4 m (r_clk);
    initial begin
        m.m3.mem[0]={12'd7,5'd0,3'd0,5'd1,7'h13};     // addi x1,x0,7
        m.m3.mem[1]={7'd0,5'd1,5'd0,3'h2,5'd8,7'h23}; // sw x1, 8(x0)
        m.m3.mem[2]={12'd8,5'd0,3'b010,5'd2,7'h3};    // lw x2, 8(x0)
    end
    initial #99 forever #100 $display("%3d %d %d %d", $time, m.w_r1, m.w_s2, m.w_rt);
    initial #400 $finish;
endmodule
