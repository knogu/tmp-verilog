import dataclasses
import subprocess
# @dataclasses.dataclass
# class CpuState:
#     pc: int
#     rs1_val: int
#     rs2_val: int
#     imm: int
#     oprand_2nd: int
#     alu_out: int

#     def set(self, field_name: str, val: int):
#         match field_name:
#             case "rs1_val":
#                 self.rs1_val = val
#             case "rs2_val":
#                 self.rs2_val = val
#             case "imm":
#                 self.imm = val
#             case _:
#                 raise

result = subprocess.run(['iverilog cpu.v && vvp a.out'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

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

print(result.stdout)
status = collect_status(result.stdout.splitlines())

assertions = [
    {"pc": 0, "rs1_val": 0, "rs2_val": 0, "imm": 5, "2nd_operand": 5, "wbdata": 5}
]

is_failed = False
for i, assertion in enumerate(assertions):
    for label, val in assertion.items():
        if status[i][label] != val:
            is_failed = True
if (is_failed):
    print("failed")
else:
    print("succeeded")
