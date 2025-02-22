import re
from collections import defaultdict

def parse_line(line):
    """
    解析一行，返回 (instr, cycle)
    若行中包含 "||"，则实际的指令为 "||" 后部分的第一个非括号内容（若遇到 [数字] 则取下一个 token）。
    """
    line = line.strip()
    if not line:
        return None, None

    # 提取周期：寻找 @ 后面的数字
    cycle_match = re.search(r'@(\d+)', line)
    if cycle_match:
        cycle = int(cycle_match.group(1))
    else:
        cycle = None

    # 判断是否有 "||"
    if "||" in line:
        # 分割取 "||" 后的部分
        parts = line.split("||", 1)
        tail = parts[1].strip()
        tokens = tail.split()
        # 如果第一个 token 是 [数字]，则取下一个 token作为实际指令
        if tokens and tokens[0].startswith('[') and len(tokens) > 1:
            instr = tokens[1]
        else:
            instr = tokens[0] if tokens else None
    else:
        # 否则直接取行首的第一个 token作为指令
        tokens = line.split()
        instr = tokens[0] if tokens else None

    return instr, cycle

def main():
    filename = "gemv1024_trace_bench_processed.txt"
    TOTAL_CYCLES = 3662        #! 程序最终结束周期数
    instructions = []
    
    # 逐行读取文件，解析出 (instr, cycle)
    with open(filename, "r") as f:
        for line in f:
            instr, cycle = parse_line(line)
            if instr is not None and cycle is not None:
                instructions.append((instr, cycle))
    
    # 统计各指令占用的周期数总和
    instr_cycles = defaultdict(int)
    n = len(instructions)
    for i, (instr, cycle) in enumerate(instructions):
        # 找到下一条周期数与当前行不同的指令
        j = i + 1
        while j < n and instructions[j][1] == cycle:
            j += 1
        if j < n:
            next_cycle = instructions[j][1]
        else:
            next_cycle = TOTAL_CYCLES  # 程序最终结束周期数
        duration = next_cycle - cycle
        instr_cycles[instr] += duration

    # 按周期数从大到小排序后输出
    for instr, total in sorted(instr_cycles.items(), key=lambda x: x[1], reverse=True):
        print(f"{instr}: {total} cycles",f"({total/TOTAL_CYCLES:.2%})")


    print(f"Total cycles: {TOTAL_CYCLES} cycles")

if __name__ == "__main__":
    main()
