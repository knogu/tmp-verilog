module m_adder(w_in1, w_in2, w_out);
    input wire [31:0] w_in1, w_in2;
    output wire [31:0] w_out;
    assign w_out = w_in1 + w_in2;
endmodule

module m_alu(w_in1, w_in2, w_out, w_tkn);
    input wire [31:0] w_in1, w_in2;
    output wire [31:0] w_out;
    output wire w_tkn;
    assign w_out = w_in1 + w_in2;
    assign w_tkn = w_in1 != w_in2;
endmodule

module m_mux(w_in1, w_in2, w_sel, w_out);
    input wire [31:0] w_in1, w_in2;
    input wire w_sel;
    output wire [31:0] w_out;
    assign w_out = (w_sel) ? w_in2 : w_in1;
endmodule

module m_RF(w_clk, w_rs1, w_rs2, w_rs1_val, w_rs2_val, w_rd_idx, w_reg_is_written, w_wb_data);
    input wire w_clk, w_reg_is_written;
    input wire [4:0] w_rs1, w_rs2, w_rd_idx;
    output wire [31:0] w_rs1_val, w_rs2_val;
    input wire [31:0] w_wb_data;
    reg [31:0] mem [0:31];
    assign w_rs1_val = (w_rs1 == 5'd0) ? 32'd0 : mem[w_rs1];
    assign w_rs2_val = (w_rs2 == 5'd0) ? 32'd0 : mem[w_rs2];
    always @(posedge w_clk) if (w_reg_is_written) mem[w_rd_idx] = w_wb_data;
    always @(posedge w_clk) if (w_reg_is_written & w_rd_idx == 5'd30) $finish;
    integer i; initial for (i=0; i<32; i=i+1) mem[i] = 0;
endmodule

module m_am_imem(w_pc, w_inst);
  input  wire [31:0] w_pc;
  output wire [31:0] w_inst;
  reg[31:0] mem[0:63];
  assign w_inst = mem[w_pc[7:2]];
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

module m_proc(w_clk);
    input wire w_clk;
    
    // Instruction Fetch
    wire[31:0] w_npc, w_inst;
    reg[31:0] r_pc = 0;
    m_adder m_adder_pc(r_pc, 32'h4, w_npc);
    m_am_imem m_insts_memory(r_pc, w_inst);

    // Instruction decode
    wire[31:0] w_rs1_val, w_rs2_val, w_wbdata;
    m_RF m_RF_(w_clk, w_inst[19:15], w_inst[24:20], w_rs1_val, w_rs2_val, w_inst[11:7], 1'b1, w_wbdata);
    wire w_r, w_i, w_s, w_b, w_u, w_j, w_ld;
    wire[31:0] w_imm;
    m_gen_imm m_gen_imm_(w_inst, w_imm, w_r, w_i, w_s, w_b, w_u, w_j, w_ld);
    wire[31:0] w_second_operand;
    m_mux m_second_oprand_chooser(w_rs2_val, w_imm, w_i, w_second_operand);

    // Execution
    m_adder m_ex(w_rs1_val, w_second_operand, w_wbdata);

    always @(posedge w_clk) r_pc <= w_npc;
endmodule

module m_top();
    reg r_clk=0; initial #150 forever #50 r_clk = ~r_clk;
    m_proc m (r_clk);
    initial begin
        `define MM m.m_insts_memory.mem
        `include "asm.txt"
    end
    initial begin
        #99;
        forever begin
            #100;
            $display("time:        %5d ", $time);
            $display("rs1_val:     %5d ", m.w_rs1_val);
            $display("2nd operand: %5d", m.w_second_operand);
            $display("wbdata:      %5d", m.w_wbdata);
            $display("\n");
        end
    end
    initial #400 $finish;
endmodule
