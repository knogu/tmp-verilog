module m_adder(w_in1, w_in2, w_out);
    input wire [31:0] w_in1, w_in2;
    output wire [31:0] w_out;
    assign w_out = w_in1 + w_in2;
endmodule

module m_alu(w_in1, w_in2, w_out, w_alu_control, w_alu_zero);
    input wire [31:0] w_in1, w_in2;
    input wire [2:0] w_alu_control;
    output wire [31:0] w_out;
    assign w_out = (w_alu_control == 3'b000) ? w_in1 + w_in2 :
                   (w_alu_control == 3'b001) ? w_in1 - w_in2 :
                   w_in1; // todo
    output wire w_alu_zero;
    assign w_alu_zero = (w_out == 0);
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

module m_gen_imm(w_ir, w_imm, w_r, w_i, w_s, w_b, w_u, w_j, w_is_ld);
    input  wire [31:0] w_ir;
    output wire [31:0] w_imm;
    output wire w_r, w_i, w_s, w_b, w_u, w_j, w_is_ld;
    m_get_type m1 (w_ir[6:2], w_r, w_i, w_s, w_b, w_u, w_j);
    m_get_imm m2 (w_ir, w_i, w_s, w_b, w_u, w_j, w_imm);
    assign w_is_ld = (w_ir[6:2]==0);
endmodule

module m_data_memory(w_clk, w_addr, w_write_enabled, w_write_data, w_mem_output);
    input  wire w_clk, w_write_enabled;
    input  wire [31:0] w_addr, w_write_data;
    output wire [31:0] w_mem_output;
    reg [31:0] mem [0:63];
    assign w_mem_output = mem[w_addr[7:2]];
    always @(posedge w_clk) if (w_write_enabled) mem[w_addr[7:2]] <= w_write_data;
    integer i; initial for (i=0; i<64; i=i+1) mem[i] = 32'd0;
endmodule

module main_decoder(w_opcode, w_funct3, w_funct7, w_alu_op);
    input wire[6:0] w_opcode, w_funct7;
    input wire[2:0] w_funct3;
    output wire[1:0] w_alu_op;
    assign w_alu_op = (w_opcode == 7'b0000011 | w_opcode == 7'b0100011) ? 2'b00 :
                      (w_opcode == 7'b0110011 | w_opcode == 7'b0010011) ? 2'b10 :
                       2'b01;
endmodule

module alu_decoder(w_alu_op, w_funct3, w_opcode, w_funct7, w_alu_control);
    input wire [1:0] w_alu_op;
    input wire [2:0] w_funct3;
    input wire [6:0] w_opcode;
    input wire [6:0] w_funct7;
    output wire [2:0] w_alu_control;
    assign w_alu_control = (w_alu_op == 2'b00) ? 3'b000 : // add for lw,sw
                           (w_alu_op == 2'b01) ? 3'b001 : // subtract for beq
                           (w_funct3 == 3'b000 & w_opcode[5] == 1'b1 & w_funct7[5] == 1'b1) ? 3'b001 : // subtract TODO: w_opcode[5]の値が正しくない？
                           (w_funct3 == 3'b010) ? 3'b101 : // set less than
                           (w_funct3 == 3'b110) ? 3'b110 : // or
                           (w_funct3 == 3'b111) ? 3'b010 : // and
                           3'b000; // add
  endmodule

module m_proc(w_clk);
    input wire w_clk;
    
    // Instruction Fetch
    wire[31:0] w_pc_incr, w_inst, w_branched_pc, w_next_pc;
    reg[31:0] r_pc = 0;
    m_adder m_adder_incr_pc(r_pc, 32'h4, w_pc_incr);
    m_adder m_adder_branched_pc(r_pc, w_imm, w_branched_pc);
    m_mux m_next_pc_chooser(w_pc_incr, w_branched_pc, w_b & w_alu_zero, w_next_pc);
    m_am_imem m_insts_memory(r_pc, w_inst);

    // Instruction decode
    wire[31:0] w_rs1_val, w_rs2_val, w_wbdata, w_alu_out, w_memory_out;
    m_RF m_RF_(w_clk, w_inst[19:15], w_inst[24:20], w_rs1_val, w_rs2_val, w_inst[11:7], !w_s & !w_b, w_wbdata);
    wire w_r, w_i, w_s, w_b, w_u, w_j, w_is_ld;
    wire[31:0] w_imm;
    m_gen_imm m_gen_imm_(w_inst, w_imm, w_r, w_i, w_s, w_b, w_u, w_j, w_is_ld);
    wire[31:0] w_second_operand;
    m_mux m_second_oprand_chooser(w_rs2_val, w_imm, !w_r & !w_b, w_second_operand);
    wire[1:0] w_alu_op;
    main_decoder main_decoder_(w_inst[6:0], w_inst[14:12], w_inst[31:25], w_alu_op);
    wire[2:0] w_alu_control;
    alu_decoder alu_decoder_(w_alu_op, w_inst[14:12], w_inst[6:0], w_inst[31:25], w_alu_control);

    // Execution
    wire w_alu_zero;
    m_alu m_ex(w_rs1_val, w_second_operand, w_alu_out, w_alu_control, w_alu_zero);

    // Memory Access
    m_data_memory m_data_memory_(w_clk, w_alu_out, w_s, w_rs2_val, w_memory_out);

    // Write Back
    m_mux m_wbdata_chooser(w_alu_out, w_memory_out, w_is_ld, w_wbdata);

    always @(posedge w_clk) r_pc <= w_next_pc;
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
            $display("pc:          %5d", m.r_pc);
            // ID
            $display("// ID");
            $display("rd:          %5d ", m.w_inst[11:7]);
            $display("rs1:         %5d ", m.w_inst[19:15]);
            $display("rs1_val:     %5d ", m.w_rs1_val);
            $display("rs2:         %5d ", m.w_inst[24:20]);
            $display("rs2_val:     %5d ", m.w_rs2_val);
            $display("imm:         %5d", m.w_imm);
            $display("2nd_operand: %5d", m.w_second_operand);
            $display("is_ld:       %5d", m.w_is_ld);
            $display("opcode:      %7b", m.w_inst[6:0]);

            // EX
            $display("alu_control:  %3b", m.w_alu_control);
            $display("alu_out:     %5d", m.w_alu_out);
            // MA
            $display("memory_out:  %5d", m.w_memory_out);
            // WB
            $display("wbdata:      %5d", m.w_wbdata);
            // for debug
            $display("x0:          %5d", m.m_RF_.mem[0]);
            $display("x1:          %5d", m.m_RF_.mem[1]);
            $display("x2:          %5d", m.m_RF_.mem[2]);
            $display("========");
        end
    end
    initial #1400 $finish;
endmodule
