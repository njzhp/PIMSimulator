def process_trace_file(input_file, output_file):
    """
    处理DRAM_PIM仿真跟踪文件，对相同周期的不同指令保留每种指令的第一行

    Args:
        input_file (str): 输入文件路径
        output_file (str): 输出文件路径
    """
    # 存储处理后的指令，以(时钟周期, 指令类型)为键
    processed_instructions = {}

    with open(input_file, 'r') as f:
        for line in f:
            # 检查是否到达终止标记
            if "==== Channel [0] ====" in line:
                break

            # 跳过空行
            if not line.strip():
                continue

            # 处理包含@的行
            if '@' in line:
                original_line = line.strip()

                if ' tag :' in original_line:
                    # 对于包含tag的行，直接保留原始行，确保@也被保留
                    full_instruction = original_line
                    # 从@后面提取周期数字
                    at_index = original_line.find('@')
                    cycle_str = original_line[at_index+1:].split()[0]
                    cycle = int(cycle_str)
                else:
                    # @在行末尾的情况
                    parts = original_line.split('@')
                    instruction_part = parts[0].strip()
                    cycle = int(parts[1].strip())
                    full_instruction = instruction_part

                # 获取指令类型（第一个单词）
                instruction_type = original_line.split()[0]

                # 如果这个周期的这类指令还没有记录，则记录它
                if (cycle, instruction_type) not in processed_instructions:
                    processed_instructions[(cycle, instruction_type)] = full_instruction

    # 按时钟周期排序并写入输出文件
    sorted_entries = sorted(processed_instructions.items(), key=lambda x: (x[0][0], x[0][1]))

    with open(output_file, 'w') as f:
        for (cycle, _), instruction in sorted_entries:
            # 对于包含tag的行，直接写入（原始格式已经包含@周期）
            if ' tag :' in instruction:
                f.write(f"{instruction}\n")
            else:
                # 对于不包含tag的行，将@添加到末尾
                f.write(f"{instruction} @{cycle}\n")


# 使用示例
if __name__ == "__main__":
    input_file = "gemv1024_trace_bench.txt"
    output_file = "gemv1024_trace_bench_processed.txt"
    process_trace_file(input_file, output_file)
