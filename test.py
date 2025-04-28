import dataclasses
import os
import subprocess


def collect_status(result: list):
    all_states = []
    cur_state: dict = {}
    for line in result:
        if "===" in line:
            all_states.append(cur_state)
            cur_state = {}
            continue
        if ":" in line and not "cpu.v" in line:
            label, val = line.split(":")
            cur_state[label] = int(val)
    return all_states


insts1 = [
    "`MM[0]={12'd5,5'd0,3'h0,5'd1,7'h13};       //  addi x1,x0,5",
    "`MM[1]={7'd0,5'd1,5'd1,3'h0,5'd2,7'h33};   //  add  x2,x1,x1",
    "`MM[2]={12'd1,5'd1,3'd0,5'd1,7'h13};       //L:addi x1,x1,1",
    "`MM[3]={~7'd0,5'd2,5'd1,3'h1,5'h1d,7'h63}; //  bne  x1,x2,L",
    "`MM[4]={12'd9,5'd1,3'd0,5'd10,7'h13};      //  addi x10,x1,9"
]

assertions1 = [
    {"pc": 0, "rs1_val": 0, "imm": 5, "2nd_operand": 5, "wbdata": 5, "rd": 1},
    {"pc": 4, "rs1_val": 5, "rs2_val": 5, "2nd_operand": 5, "wbdata": 10, "rd": 2},
    {"pc": 8, "rs1_val": 5, "imm": 1, "2nd_operand": 1, "wbdata": 6, "rd": 1},
    {"pc": 12, "rs1_val": 6, "2nd_operand": 10},
    {"pc": 8, "rs1_val": 6, "imm": 1, "2nd_operand": 1, "wbdata": 7, "rd": 1},
    {"pc": 12, "rs1_val": 7, "2nd_operand": 10},
    {"pc": 8, "rs1_val": 7, "imm": 1, "2nd_operand": 1, "wbdata": 8, "rd": 1},
    {"pc": 12, "rs1_val": 8, "2nd_operand": 10},
    {"pc": 8, "rs1_val": 8, "imm": 1, "2nd_operand": 1, "wbdata": 9, "rd": 1},
    {"pc": 12, "rs1_val": 9, "2nd_operand": 10},
    {"pc": 8, "rs1_val": 9, "imm": 1, "2nd_operand": 1, "wbdata": 10, "rd": 1},
    {"pc": 12, "rs1_val": 10, "2nd_operand": 10},
]

scenarios = [(insts1, assertions1)]

for insts, assertions in scenarios:
    # Prepare instructions
    with open(os.path.expanduser('asm.txt'), 'w', encoding='utf-8') as f:
        for inst in insts:
            f.write(inst + '\n')
    # Execute simulation
    result = subprocess.run(['iverilog cpu.v && vvp a.out'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    # Check result
    status = collect_status(result.stdout.splitlines())

    for i, assertion in enumerate(assertions):
        for label, val in assertion.items():
            if status[i][label] != val:
                print("Assertion failed.")
                print("actual PC: " + str(status[i]["pc"]))
                print("assertion index: " + str(i))
                print(label)
                print("expected: ", val)
                print("actual: ", status[i][label])
                exit(1)

    print("succeeded")
