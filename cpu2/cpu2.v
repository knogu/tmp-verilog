module m_pc_adder(w_in1, w_in2, w_out);
    input wire [31:0] w_in1, w_in2;
    output wire [31:0] w_out;
    assign w_out = (w_in1 + w_in2 <= 12) ? w_in1 + w_in2 : 12; // for test before beq is made
endmodule

module m_adder(w_in1, w_in2, w_out);
    input wire [31:0] w_in1, w_in2;
    output wire [31:0] w_out;
    assign w_out = w_in1 + w_in2;
endmodule

module m_am_imem(w_pc, w_insn);
  input  wire [31:0] w_pc;
  output wire [31:0] w_insn;
  reg [31:0] mem [0:63]; /* synthesis ram_init_file = "imem.mif" */;
  assign w_insn = mem[w_pc[7:2]];
//   integer i; initial for (i=0; i<64; i=i+1) mem[i] = 32'd0;
endmodule

module m_mux(w_in1, w_in2, w_sel, w_out);
    input wire [31:0] w_in1, w_in2;
    input wire w_sel;
    output wire [31:0] w_out;
    assign w_out = (w_sel) ? w_in2 : w_in1;
endmodule

module m_RF(w_clk, w_rs1, w_rs2, w_rs1_val, w_rs2_val, w_write_addr, w_write_enabled, w_write_data, w_a0);
  input  wire w_clk, w_write_enabled;
  input  wire [4:0] w_rs1, w_rs2, w_write_addr;
  output wire [31:0] w_rs1_val, w_rs2_val, w_a0;
  input  wire [31:0] w_write_data;
  reg [31:0] mem [0:31];
  assign w_rs1_val = (w_rs1==5'd0) ? 32'd0 : mem[w_rs1];
  assign w_rs2_val = (w_rs2==5'd0) ? 32'd0 : mem[w_rs2];
  assign w_a0 = mem[10];
  always @(posedge w_clk) if (w_write_enabled) mem[w_write_addr] <= w_write_data;
//   always @(posedge w_clk) if (w_write_enabled & w_write_addr==5'd30) $finish;
  integer i; initial for (i=0; i<32; i=i+1) mem[i] = 0;
endmodule

module m_get_imm(ir, i, s, b, u, j, imm);
  input wire [31:0] ir;
  input wire i, s, b, u, j;
  output wire [31:0] imm;
  assign imm= (i) ? {{20{ir[31]}},ir[31:20]} :
              (s) ? {{20{ir[31]}},ir[31:25],ir[11:7]} :
              (b) ? {{20{ir[31]}},ir[7],ir[30:25],ir[11:8],1'b0} :
              (u) ? {ir[31:12],12'b0} :
              (j) ? {{12{ir[31]}},ir[19:12],ir[20],ir[30:21],1'b0} : 0;
endmodule

module m_get_type(opcode5, r, i, s, b, u, j);
  input  wire [4:0] opcode5;
  output wire r, i, s, b, u, j;
  assign j = (opcode5==5'b11011);
  assign b = (opcode5==5'b11000);
  assign s = (opcode5==5'b01000);
  assign r = (opcode5==5'b01100);
  assign u = (opcode5==5'b01101 || opcode5==5'b00101);
  assign i = ~(j | b | s | r | u);
endmodule

module m_gen_imm(w_ir, w_imm, w_r, w_i, w_s, w_b, w_u, w_j, w_ld);
  input  wire [31:0] w_ir;
  output wire [31:0] w_imm;
  output wire w_r, w_i, w_s, w_b, w_u, w_j, w_ld;
  m_get_type m1 (w_ir[6:2], w_r, w_i, w_s, w_b, w_u, w_j);
  m_get_imm m2 (w_ir, w_i, w_s, w_b, w_u, w_j, w_imm);
  assign w_ld = (w_ir[6:2]==0);
endmodule

module m_proc3(w_clk, w_a0);
  input wire w_clk;
  wire [31:0] w_npc, w_inst, w_imm, w_rs1_val, w_rs2_val, w_s2, w_alu_out;
  output wire [31:0] w_a0;
  wire w_r, w_i, w_s, w_b, w_u, w_j, w_ld;
  reg [31:0] r_pc = 0;
  m_pc_adder pc_adder (32'h4, r_pc, w_npc);
  m_am_imem m3 (r_pc, w_inst);
  m_gen_imm m4 (w_inst, w_imm, w_r, w_i, w_s, w_b, w_u, w_j, w_ld);
  m_RF rf(w_clk, w_inst[19:15], w_inst[24:20], w_rs1_val, w_rs2_val,
           w_inst[11:7], 1'b1, w_alu_out, w_a0);
  m_mux m6 (w_rs2_val, w_imm, w_i, w_s2);
  m_adder m7 (w_rs1_val, w_s2, w_alu_out);
  always @(posedge w_clk) r_pc <= w_npc;
endmodule

module m_top();
  reg r_clk=0; initial #150 forever #50 r_clk = ~r_clk;
  wire [31:0] w_a0;
  m_proc3 m (r_clk, w_a0);
  initial begin
    m.m3.mem[0]={12'd3,5'd0,3'd0,5'd1,7'h13};  // addi x1,x0,3
    m.m3.mem[1]={12'd4,5'd1,3'd0,5'd2,7'h13};  // addi x2,x1,4
    m.m3.mem[2]={12'd5,5'd2,3'd0,5'd10,7'h13};  // addi x10,x2,5
    m.m3.mem[3]={12'd0,5'd0,3'd0,5'b0,7'b0110011};  // addi x0,x0,x0
  end
  initial #99 forever begin
    #100;
    $display("pc:          %5d", m.r_pc);
    $display("inst:        %b ", m.w_inst);
    $display("rd:          %5d ", m.w_inst[11:7]);
    $display("rs1:         %5d ", m.w_inst[19:15]);
    $display("rs1_val:     %5d ", m.w_rs1_val);
    $display("rs2:         %5d ", m.w_inst[24:20]);
    $display("rs2_val:     %5d ", m.w_rs2_val);
    $display("alu_out:     %5d", m.w_alu_out);
    $display("a0:          %5d", w_a0);
    $display("==========");
  end
  initial #700 $finish;
endmodule
