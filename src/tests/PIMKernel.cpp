/***************************************************************************************************
 * Copyright (C) 2021 Samsung Electronics Co. LTD
 *
 * This software is a property of Samsung Electronics.
 * No part of this software, either material or conceptual may be copied or distributed,
 * transmitted, transcribed, stored in a retrieval system, or translated into any human
 * or computer language in any form by any means,electronic, mechanical, manual or otherwise,
 * or disclosed to third parties without the express written permission of Samsung Electronics.
 * (Use of the Software is restricted to non-commercial, personal or academic, research purpose
 * only)
 **************************************************************************************************/

#include "tests/PIMKernel.h"

#include <iomanip>
#include <string>

#include "AddressMapping.h"
#include "tests/PIMCmdGen.h"

/*
GEMV 矩阵-向量在 DRAM 上的映射总览（只加注释，不改动任何代码）:
- W 矩阵维度约定：operand->bShape[0] 为输出维度（行数），operand->bShape[1] 为输入维度（列数）。
- Tile 切分（与 GRF 数对应）:
  - 输入维度按 num_grfA_ 为步长切 tile：input_tile_size = num_grfA_
  - 输出维度按 (num_grfB_ * num_total_pim_blocks_) 为步长切大 tile：output_tile_size = num_grfB_ * num_total_pim_blocks_
    - 在每个大 tile 内，再以 num_grfB_ 为步长划分给各个 PIM Block（PB），每个 PB 对应 num_grfB_ 行（GRF_B 行束）
- 奇偶 bank 交替规则:
  - input tile 的奇偶性决定写入 bank 的奇偶：t_in = floor(j / num_grfA_)，is_odd = (t_in % 2)
  - is_odd == 0 时写 EVEN_BANK；is_odd == 1 时写 ODD_BANK
  - 这样在计算阶段可以分别触发偶/奇 bank 的 MAC，流水地累计部分和
- W[i, j]（行 i, 列 j）存放位置的直观推导（概念性，不涉及具体地址位宽）:
  - t_in  = j / num_grfA_            （输入 tile 序号）
  - t_out = i / (num_grfB_ * num_total_pim_blocks_)   （输出大 tile 序号）
  - pb_id = (i % (num_grfB_ * num_total_pim_blocks_)) / num_grfB_   （该行所在的 PIM Block 序号）
  - gB    = i % num_grfB_            （该行在 GRF_B 束内的索引）
  - gA    = j % num_grfA_            （该列在 GRF_A 束内的索引）
  - bank 奇偶 = (t_in % 2)           （偶 tile -> EVEN_BANK，奇 tile -> ODD_BANK）
  - 在 preloadGemv 中，按上述索引把 W[i,j] 写入到 (ch, rank, bg, bank奇偶, row, col)
    其中 ch/rank/bg/bank 的遍历顺序由 changeBank 决定，会在不同 PB 间前进；
    row/col 从 even/odd 的起始 (starting_row/starting_col) 出发，col 随 gA 递增
- 计算阶段:
  - computeGemv 先把输入向量按 GRF_A 写入寄存器（WRIO_TO_GRF_），再通过 "MAC_" 触发从对应奇/偶 bank 读取权重并乘加到 GRF_B
  - 核心列基址 col = (num_grfA_ * num_grfB_) * (inputTile/2 + outputTile * num_input_tiles / 2)
    对應 “每兩塊 input tile 合成一組（偶/奇），另叠加 output tile 偏移”，保證 MAC 時對齊到預加載好的權重塊
- 结果读取:
  - readResult 根据 pb_type(偶/奇) 选择 bank_offset，并以 num_grf_ 步长顺序读出部分和/最终结果
*/

void PIMKernel::runPIM()
{
    while (mem_->hasPendingTransactions())
    {
        cycle_++;
        mem_->update();
    }
}

uint64_t PIMKernel::getCycle()
{
    return cycle_;
}

void PIMKernel::parkIn()
{
    // 进入 PIM 工作前的“编队/占位”事务：对所有通道/Rank/BankGroup/Bank 广播 START_/END_ 标记，
    // 使多通道/多Rank 执行步调一致；地址位 (row = 1<<13, col=0) 仅作同步锚点使用
    addBarrier();
    for (int& ch_idx : pim_chans_)
    {
        for (int& ra_idx : pim_ranks_)
        {
            for (int bank_idx = 0; bank_idx < num_banks_ / num_bank_groups_; bank_idx++)
            {
                for (int bg_idx = 0; bg_idx < num_bank_groups_; bg_idx++)
                {
                    string str = "PARK_IN_";
                    if (bg_idx == 0 && bank_idx == 0)
                        str = "START_" + str;
                    else if (bg_idx == 3 && bank_idx == 3)
                        str = "END_" + str;
                    mem_->addTransaction(
                        false,
                        pim_addr_mgr_->addrGen(ch_idx, ra_idx, bg_idx, bank_idx, (1 << 13), 0), str,
                        &null_bst_);
                }
            }
        }
    }
    addBarrier();
}

void PIMKernel::parkOut()
{
    // 退出 PIM 工作后的“解散/收尾”事务：与 parkIn 类似的 START_/END_ 广播，用于确保尾部一致性
    for (int& ch_idx : pim_chans_)
    {
        for (int& ra_idx : pim_ranks_)
        {
            for (int bank_idx = 0; bank_idx < num_banks_ / num_bank_groups_; bank_idx++)
            {
                for (int bg_idx = 0; bg_idx < num_bank_groups_; bg_idx++)
                {
                    string str = "PARK_OUT_";
                    if (bg_idx == 0 && bank_idx == 0)
                        str = "START_" + str;
                    else if (bg_idx == 3 && bank_idx == 3)
                        str = "END_" + str;
                    mem_->addTransaction(
                        false,
                        pim_addr_mgr_->addrGen(ch_idx, ra_idx, bg_idx, bank_idx, (1 << 13), 0), str,
                        &null_bst_);
                }
            }
        }
    }
    addBarrier();
}

void PIMKernel::addTransactionAll(bool is_write, int bg_idx, int bank_idx, int row, int col,
                                  const string tag, BurstType* bst, bool use_barrier, int num_loop)
{
    // 将同一 (row,col) 起点的 num_loop 次事务，广播到所有 PIM 的通道/Rank；
    // 每次事务 col++，按列方向线性前进；如果 use_barrier=true，则在末尾对每个通道插入屏障
    for (int& ch_idx : pim_chans_)
        for (int& ra_idx : pim_ranks_)
        {
            unsigned local_row = row;
            unsigned local_col = col;
            for (int i = 0; i < num_loop; i++)
            {
                uint64_t addr = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx,
                                                           local_row, local_col);
                (tag != "") ? mem_->addTransaction(is_write, addr, tag, bst)
                            : mem_->addTransaction(is_write, addr, bst);
                local_col++;
            }
        }

    if (use_barrier)
        addBarrier();
}

void PIMKernel::addTransactionAll(bool is_write, int bg_idx, int bank_idx, int row, int col,
                                  BurstType* bst, bool use_barrier, int num_loop)
{
    addTransactionAll(is_write, bg_idx, bank_idx, row, col, "", bst, use_barrier, num_loop);
}

void PIMKernel::addBarrier()
{
    for (int& ch_idx : pim_chans_) mem_->addBarrier(ch_idx);
}

void PIMKernel::changePIMMode(dramMode curMode, dramMode nextMode)
{
    // SB <-> HAB <-> HAB_PIM 模式切换序列（通过特殊寄存器行/列地址触发）
    // START_/END_ 标签便于日志/时序对齐；对外表现为写事务，但语义用于控制状态机
    if (curMode == dramMode::SB && nextMode == dramMode::HAB)
    {
        addTransactionAll(true, 0, 0, pim_abmr_ra_, 0x1f, "START_SB_TO_HAB_", &null_bst_);
        addTransactionAll(true, 0, 1, pim_abmr_ra_, 0x1f, &null_bst_);
        if (num_banks_ >= 2)
        {
            addTransactionAll(true, 2, 0, pim_abmr_ra_, 0x1f, &null_bst_);
            addTransactionAll(true, 2, 1, pim_abmr_ra_, 0x1f, "END_SB_TO_HAB_", &null_bst_);
        }
    }
    else if (curMode == dramMode::HAB)
    {
        if (nextMode == dramMode::SB)
        {
            addTransactionAll(true, 0, 0, pim_sbmr_ra_, 0x1f, "START_HAB_TO_SB", &null_bst_);
            addTransactionAll(true, 0, 1, pim_sbmr_ra_, 0x1f, "END_HAB_TO_SB", &null_bst_);
        }
        else if (nextMode == dramMode::HAB_PIM)
        {
            addTransactionAll(true, 0, 0, pim_reg_ra_, 0x0, "PIM", &bst_hab_pim_);
        }
    }
    else if (curMode == dramMode::HAB_PIM && nextMode == dramMode::HAB)
        addTransactionAll(true, 0, 0, pim_reg_ra_, 0x0, "PIM", &bst_hab_);

    addBarrier();
}

/*
void PIMKernel::preprocessBn(NumpyBurstType* mean_npbst, NumpyBurstType* var_npbst,
                             NumpyBurstType* gamma_npbst, NumpyBurstType* beta_npbst,
                             NumpyBurstType* input_npbst, fp16** params, float eps)
{
    for (int i = 0; i < input_npbst->bShape[0]; i++)
    {
        params[i][0] = 1 / sqrt((float)var_npbst->getBurst(i / 16).fp16Data_[i % 16] + eps);
        params[i][1] = gamma_npbst->getBurst(i / 16).fp16Data_[i % 16];
        params[i][2] = -mean_npbst->getBurst(i / 16).fp16Data_[i % 16] /
                       sqrt((float)var_npbst->getBurst(i / 16).fp16Data_[i % 16] + eps);
        params[i][3] = beta_npbst->getBurst(i / 16).fp16Data_[i % 16];
    }
}

// FIXME : FIX size of srf_bst_. if ch_model is bigger than memory channel, it is not defined.
void PIMKernel::preprocessSrf(NumpyBurstType* input_npbst, fp16** params, int burst_offset,
                              int num_srf_usage)
{
    int ch_idx = 0;
    int ra_idx = 0;
    int burst_idx = 0;
    int num_stride_reg = 2;
    srf_bst_ = new BurstType[num_pim_chans_ * num_pim_ranks_];

    for (int ch_model = 0; ch_model < input_npbst->bShape[0]; ch_model++)
    {
        srf_bst_[ch_idx * num_pim_ranks_ + ra_idx].fp16Data_[burst_idx] =
            params[ch_model][0]; // scale
        srf_bst_[ch_idx * num_pim_ranks_ + ra_idx].fp16Data_[burst_idx + 1] =
            params[ch_model][1]; // gamma
        srf_bst_[ch_idx * num_pim_ranks_ + ra_idx].fp16Data_[burst_idx + 8] =
            params[ch_model][2]; // shift
        srf_bst_[ch_idx * num_pim_ranks_ + ra_idx].fp16Data_[burst_idx + 9] =
            params[ch_model][3]; // beta

        ra_idx++;
        if (ra_idx >= num_pim_ranks_)
        {
            ra_idx = 0;
            ch_idx++;
        }
        if (ch_idx >= num_pim_chans_)
        {
            ch_idx = 0;
            burst_idx += num_stride_reg;
        }
        if (burst_idx >= 8)
        {
            cout << "error: this is not defined" <<endl;
        }
    }
}

void PIMKernel::programSrf()
{
    for (int ch_idx = 0; ch_idx < num_pim_chans_; ch_idx++)
    {
        for (int ra_idx = 0; ra_idx < num_pim_ranks_; ra_idx++)
        {
            mem_->addTransaction(true,
                                 pim_addr_mgr_->addrGen(ch_idx, ra_idx, 0, 0, pim_reg_ra_, 0x1),
                                 &srf_bst_[ch_idx * num_pim_ranks_ + ra_idx]);
        }
    }
    addBarrier();
}
*/

void PIMKernel::programCrf(vector<PIMCmd>& cmds)
{
    // 将最多 32 条（4*8）PIM 指令写入 CRF：每 8 条指令为一次 burst传输，依次写到DRAM中 pim_reg_ra_ 的 0x4+i
    // nop 预填充确保未用槽位为 NOP，避免越界读取
    PIMCmd nop_cmd(PIMCmdType::NOP, 0);
    for (int i = 0; i < 4; i++)
    {
        if (i * 8 >= cmds.size())
            break;
        crf_bst_[i].set(nop_cmd.toInt(), nop_cmd.toInt(), nop_cmd.toInt(), nop_cmd.toInt(),
                        nop_cmd.toInt(), nop_cmd.toInt(), nop_cmd.toInt(), nop_cmd.toInt());
        for (int j = 0; j < 8; j++)
        {
            if (i * 8 + j >= cmds.size())
                break;
            crf_bst_[i].u32Data_[j] = cmds[i * 8 + j].toInt();
        }
        addTransactionAll(true, 0, 1, pim_reg_ra_, 0x4 + i, "PROGRAM_CRF", &(crf_bst_[i]));
    }
    addBarrier();
}

void PIMKernel::setControl(BurstType* bst, bool pim_op, int crf_toggle_cond, bool grfA_zero,
                           bool grfB_zero)
{
    // 设置 PIM 控制位：
    // - u8[0]:   是否启用 PIM
    // - u8[16]:  CRF PC 的切换/翻转条件（外部以“切换条件”决定 JUMP 次数/轮次）
    // - u8[20]:  是否强制清零 GRF_A
    // - u8[21]:  是否强制清零 GRF_B
    bst->u8Data_[0] = pim_op;
    bst->u8Data_[16] = crf_toggle_cond;
    bst->u8Data_[20] = grfA_zero;
    bst->u8Data_[21] = grfB_zero;
}

unsigned PIMKernel::getResultColGemv(int input_dim, int output_dim)
{
    // 计算 GEMV 结果列的基址（单位：列）：
    // num_output_tiles * num_input_tiles / 2 * (num_grfA_ * num_grfB_)
    // 直观理解：每两块 input tile（偶/奇）在一个输出 tile 上占用一片 (GRF_A * GRF_B) 的列窗口
    int num_output_tiles = ceil(((double)output_dim / (num_total_pim_blocks_)) / num_grfB_);
    int num_input_tiles = ceil((double)input_dim / (double)num_grfA_);

    return num_output_tiles * num_input_tiles / 2 * num_grfA_ * num_grfB_;
}

void PIMKernel::changeBank(pimBankType pb_type, int& ch_idx, int& ra_idx, int& bg_idx,
                           int& bank_idx, unsigned& starting_row, unsigned& starting_col,
                           unsigned& row, unsigned& col)
{
    // 在 PB 之间前进的游标推进器：
    // - pb_type == ALL_BANK：bank_idx 每次 +1，逐个 bank 走
    // - pb_type == EVEN/ODD：bank_idx 每次 + (NUM_BANKS / NUM_PIM_BLOCKS)，即跨过同一 PB 的另一半
    // 当 bank 跨界 -> bg 跨界 -> rank 跨界 -> channel 跨界 时，逐级回绕，并把 starting_row/starting_col
    // 更新为当前 row/col，以便下一轮信息从这里续写
    bank_idx += (pb_type == pimBankType::ALL_BANK) ? 1 : (num_banks_ / num_pim_blocks_);

    if (bank_idx >= (num_banks_ / num_bank_groups_))
    {
        bank_idx = 0;
        if (++bg_idx >= num_bank_groups_)
        {
            bg_idx = 0;
            if (++ra_idx >= num_pim_ranks_)
            {
                ra_idx = 0;
                if (++ch_idx >= num_pim_chans_)
                {
                    ch_idx = 0;
                    starting_row = row;
                    starting_col = col;
                }
            }
        }
    }
}

void PIMKernel::preloadGemv(NumpyBurstType* operand, unsigned starting_row, unsigned starting_col)
{
    // 将权重矩阵 W 以 tile 方式预加载到 DRAM：
    // - 输入维（列）按 num_grfA_ 为步长切块（input_tile）
    // - 输出维（行）按 num_grfB_*num_total_pim_blocks_ 为步长切块（output 大 tile）
    // - 同一 input tile 的奇偶性决定写入偶/奇 bank（bank_idx + is_odd）
    // - grfb/grfa 双重循环把 num_grfB_ x num_grfA_ 的一个权重小块写成一组连续列
    // - 每写完一个 num_grfB_ 组（即覆盖一个 PB 的行束），调用 changeBank() 前进到下一个 PB 所在的 bank/组
    // 这样，W[i,j] 会被放到：
    //   bank 偶/奇 = ((j / num_grfA_) % 2)
    //   PB = ((i % (num_grfB_ * num_total_pim_blocks_)) / num_grfB_)
    //   gB = (i % num_grfB_), gA = (j % num_grfA_)
    //   ch/rank/bg/bank 由 changeBank 的推进顺序决定；row/col 从 even/odd 起点出发，col 随 gA 增长
    int input_tile_size = num_grfA_;
    int output_tile_size = num_grfB_ * num_total_pim_blocks_;

    int ch_idx = 0, ra_idx = 0, bg_idx = 0, bank_idx = 0;
    unsigned row = 0, col = 0;
    uint64_t addr;

    unsigned even_starting_row = starting_row, odd_starting_row = starting_row;
    unsigned even_starting_col = starting_col, odd_starting_col = starting_col;

    for (int y = 0; y < operand->bShape[0]; y += output_tile_size)
    {
        // y：当前输出大 tile 的起始行（覆盖 num_grfB_ * num_total_pim_blocks_ 行）
        for (int x = 0; x < operand->bShape[1]; x += input_tile_size)
        {
            // x：当前输入 tile 的起始列（覆盖 num_grfA_ 列）
            bool is_odd = ((x / input_tile_size) % 2 == 1) ? true : false; // 输入 tile 奇偶 -> 选奇/偶 bank

            for (int tiled_y = 0; tiled_y < output_tile_size; tiled_y += num_grfB_)
            {
                // tiled_y：在一个输出大 tile 内，按 num_grfB_ 步长迭代到每个 PB（每个 PB 对应 num_grfB_ 行）
                row = (is_odd) ? odd_starting_row : even_starting_row;
                col = (is_odd) ? odd_starting_col : even_starting_col;

                for (int grfb_idx = 0; grfb_idx < num_grfB_; grfb_idx++)
                {
                    // grfb_idx：在 PB 内的行束索引（对应 GRF_B 的行）
                    for (int grfa_idx = 0; grfa_idx < num_grfA_; grfa_idx++, col++)
                    {
                        // grfa_idx：在输入 tile 内的列束索引（对应 GRF_A 的列），col 随列线性递增
                        // bank_idx + is_odd：同一 PB 的偶/奇 bank 选择
                        // d_idx：将 (y + tiled_y + grfb_idx, x + grfa_idx) 映射回一维行主序下标
                        addr = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx + is_odd,
                                                          row, col);
                        int d_idx = (y + tiled_y + grfb_idx) * operand->bShape[1] + x + grfa_idx;
                        mem_->addTransaction(true, addr, &operand->bData[d_idx]);
                    }
                }
                // 写完一个 PB（num_grfB_ 行）后，前进到下一个 PB：
                // - 偶 input tile -> EVEN_BANK；奇 input tile -> ODD_BANK
                // - changeBank 会在 ch/rank/bg/bank 维度上推进，跨 PB 走
                is_odd ? changeBank(pimBankType::ODD_BANK, ch_idx, ra_idx, bg_idx, bank_idx,
                                    odd_starting_row, odd_starting_col, row, col)
                       : changeBank(pimBankType::EVEN_BANK, ch_idx, ra_idx, bg_idx, bank_idx,
                                    even_starting_row, even_starting_col, row, col);
            }
        }
    }
}

void PIMKernel::preloadNoReplacement(NumpyBurstType* operand, unsigned starting_row,
                                     unsigned starting_col)
{
    // 线性写入（不做奇偶/分块替换），用于简单基准/校验
    // 起始地址 init_addr 后续每个 burst 以 transaction_size_ 递增
    uint64_t init_addr = pim_addr_mgr_->addrGenSafe(0, 0, 0, 0, starting_row, starting_col);

    for (int x = 0; x < operand->getTotalDim(); x++)
    {
        uint64_t addr = init_addr + x * transaction_size_;
        mem_->addTransaction(true, addr, &operand->bData[x]);
    }
}
/*
void PIMKernel::preloadEltwise(NumpyBurstType* operand, pimBankType pb_type,
                              unsigned starting_row, unsigned starting_col)
{
   int ch_idx = 0;
   int ra_idx = 0;
   int bg_idx = 0;
   int bank_idx = 0;
   int bank_offset =  (int)pb_type % 2;
   uint64_t addr_op;
   int dim_operand = operand->getTotalDim();

   for (int x=0; x < dim_operand; x+=num_grf_)
   {
       unsigned col = starting_col;
       unsigned row = starting_row;

       for (int grf_idx = 0; grf_idx < num_grf_; grf_idx++)
       {
           addr_op = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx + bank_offset, row,
                                                col);
           mem_->addTransaction(true, addr_op, &operand->bData[x + grf_idx]);
           col++;
       }
       changeBank(pb_type, ch_idx, ra_idx, bg_idx, bank_idx, starting_row, starting_col, row, col);
   }
}
*/
void PIMKernel::executeGemv(NumpyBurstType* w_data, NumpyBurstType* i_data, bool is_tree)
{
    // 计算 tile 数（与 GRF 规模和 PB 数相关）:
    // - num_output_tiles：输出维按 (num_total_pim_blocks_ * num_grfB_) 聚合的 tile 数
    // - num_input_tiles： 输入维按 num_grfA_ 聚合的 tile 数
    // - 树形（is_tree=true）：每次处理一个 input tile，立刻把 GRF_B 写回并清零，再进行下一块
    // - 普通（is_tree=false）：先累加所有偶数 input tile，再累加所有奇数 input tile，最后写回
    int num_output_tiles = ceil(((double)w_data->bShape[0] / (num_total_pim_blocks_)) / num_grfB_);
    int num_input_tiles = ceil((double)w_data->bShape[1] / (double)num_grfA_);
    int num_batch = i_data->bShape[0];
    int zero_row = 1000;

    if (is_tree)
    {
        // 树形模式：在零行 zero_row 上做 GRF_B 的初始化/清除，使每轮累加互不干扰
        for (int ch = 0; ch < num_pim_chans_; ch++)
        {
            for (int bg_idx = 0; bg_idx < num_bank_groups_; bg_idx++)
            {
                for (int ba = 0; ba < num_banks_ / num_bank_groups_; ba++)
                {
                    for (int ca = 0; ca < num_grfA_; ca++)
                    {
                        uint64_t addr = pim_addr_mgr_->addrGen(ch, 0, bg_idx, ba, zero_row, ca);
                        mem_->addTransaction(true, addr, &null_bst_);
                    }
                }
            }
        }
    }

    vector<PIMCmd> pim_cmds;
    if (is_tree)
    {
        int num_jump = ceil((double)num_input_tiles / 2) - 1;
        pim_cmds = PIMCmdGen::getPIMCmds(KernelType::GEMVTREE, num_jump, 0, 0);
    }
    else
    {
        int num_jump_of_even_bank = num_grfB_ * ceil((double)num_input_tiles / 2) - 1;
        int num_jump_of_odd_bank = num_grfB_ * floor(num_input_tiles / 2) - 1;
        pim_cmds =
            PIMCmdGen::getPIMCmds(KernelType::GEMV, 0, num_jump_of_odd_bank, num_jump_of_even_bank);
    }

    // 初始化PIM模式
    setControl(&bst_hab_pim_, true, getToggleCond(), false, true);
    parkIn();
    changePIMMode(dramMode::SB, dramMode::HAB);
    programCrf(pim_cmds);

    for (int j = 0; j < num_output_tiles; j++)
    {
        for (int b = 0; b < num_batch; b++)
        {
            // 每个输出 tile + batch 之前做一次 PC reset（切 HAB->HAB_PIM）
            // col 基址：为当前 (outputTile=j, batch=b) 组合计算出结果写回的列窗口起点
            changePIMMode(dramMode::HAB, dramMode::HAB_PIM);  // PC reset.

            int col = num_output_tiles * num_input_tiles / 2 * num_grfA_ * num_grfB_ +
                      (j + b) * num_grfB_;
            if (is_tree)
            {
                // 树形：按 input tile 次序依次 MAC -> 写回 -> 清 GRF_B
                // 即沿着并行方向parallel first
                for (int i = 0; i < num_input_tiles; i++, col += num_grfB_)
                {
                    computeGemv(i_data, num_input_tiles, num_output_tiles, i, j, b,
                                (i % 2 == 0) ? pimBankType::EVEN_BANK : pimBankType::ODD_BANK);
                    addTransactionAll(true, 0, 1, 0, col, "GRFB_TO_BANK_", &null_bst_, true,
                                      num_grf_);
                    addTransactionAll(false, 0, 0, zero_row, 0, "RESET_GRF_B", &null_bst_, true,
                                      num_grfB_);
                }
            }
            else
            {
                // 普通：先处理所有偶 input tile（EVEN_BANK），再处理所有奇 input tile（ODD_BANK）
                // 最后一次性将 GRF_B 写回
                // 即沿着累加方向accumulate first
                for (int i = 0; i < num_input_tiles; i += 2)
                    computeGemv(i_data, num_input_tiles, num_output_tiles, i, j, b,
                                pimBankType::EVEN_BANK);
                for (int i = 1; i < num_input_tiles; i += 2)
                    computeGemv(i_data, num_input_tiles, num_output_tiles, i, j, b,
                                pimBankType::ODD_BANK);
                addTransactionAll(true, 0, 1, 0, col, "GRFB_TO_BANK_", &null_bst_, true, num_grf_);
            }
            changePIMMode(dramMode::HAB_PIM, dramMode::HAB);  // for grfBReset
        }
    }
    changePIMMode(dramMode::HAB, dramMode::SB);
    parkOut();
}

void PIMKernel::computeGemv(NumpyBurstType* data, int num_input_tiles, int num_output_tiles,
                            int inputTile, int outputTile, int batchIdx, pimBankType pb_type)
{
    // 步骤1：将输入向量（按 batchIdx 和 inputTile）写入 GRF_A（地址 pim_reg_ra_ 的 0x8 + gidx）
    // input_idx = batch * (num_input_tiles * num_grfA_) + inputTile * num_grfA_ + gidx
    for (int ch_idx = 0; ch_idx < num_pim_chans_; ch_idx++)
    {
        for (int ra_idx = 0; ra_idx < num_pim_ranks_; ra_idx++)
        {
            // input upload to GRF   遍历通道和rank,将输入数据写入GRF
            for (int gidx = 0; gidx < num_grfA_; gidx++)
            {
                string str = "WRIO_TO_GRF_";
                uint64_t addr =
                    pim_addr_mgr_->addrGen(ch_idx, ra_idx, 0, 1, pim_reg_ra_, 0x8 + gidx);
                int input_idx =
                    batchIdx * num_grfA_ * num_input_tiles + inputTile * num_grfA_ + gidx;
                mem_->addTransaction(true, addr, str, &data->bData[input_idx]);
            }
            mem_->addBarrier(ch_idx);
        }
    }

    unsigned row = 0;
    // 步骤2：计算权重块的列基址：
    //   col = (num_grfA_ * num_grfB_) * (inputTile/2 + outputTile * num_input_tiles / 2)
    // 解释：
    //   - 两个 input tile（偶/奇）共享一组 (num_grfA_*num_grfB_) 的权重窗口，分别落在 EVEN/ODD bank
    //   - outputTile 决定了在更高维度上的窗口偏移
    unsigned col = (num_grfA_ * num_grfB_) * (inputTile / 2 + outputTile * num_input_tiles / 2);

    // 步骤3：触发 MAC_：
    //   - 使用 addTransactionAll(false, ..., bank=(int)pb_type, row, col + c_idx, "MAC_", ...)
    //   - pb_type=EVEN_BANK/ODD_BANK，确保只从与当前 input tile 奇偶匹配的 bank 取权重
    //   - num_grfA_ 次读取使得一个 (GRF_B x GRF_A) 的块完成一次乘加
    for (int c_idx = 0; c_idx < 64; c_idx += 8)
        // c_idx 为列偏移的分组步进（实现细节与内部通道/组织对齐相关），不影响“奇偶 bank 对齐”的核心规则
        addTransactionAll(false, 0, (int)pb_type, row, col + c_idx, "MAC_", &null_bst_, true,
                          num_grfA_);
}

void PIMKernel::readResult(BurstType* resultBst, pimBankType pb_type, int output_dim,
                           uint64_t base_addr, unsigned starting_row, unsigned starting_col)
{
    // 从指定奇/偶 bank 读取结果：
    // - bank_offset = (int)pb_type % 2
    // - 每次读取 num_grf_ 个 burst（与 GRF 宽度一致），按 col++ 线性前进
    // - changeBank(pb_type, ...) 在 PB 之间前进
    int ch_idx = 0;
    int ra_idx = 0;
    int bg_idx = 0;
    int bank_idx = 0;
    int bank_offset = (int)pb_type % 2;
    uint64_t addr;

    for (int x = 0; x < output_dim; x += num_grf_)
    {
        unsigned row = starting_row;
        unsigned col = starting_col;

        for (int grf_idx = 0; grf_idx < num_grf_; grf_idx++)
        {
            addr = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx + bank_offset, row,
                                              col);
            mem_->addTransaction(false, base_addr + addr, "output", &resultBst[x + grf_idx]);
            col++;
        }
        changeBank(pb_type, ch_idx, ra_idx, bg_idx, bank_idx, starting_row, starting_col, row, col);
    }
}

void PIMKernel::executeEltwise(int dim, pimBankType pb_type, KernelType ktype, int input0_row,
                               int result_row, int input1_row)
{
    // Eltwise 的 tile 维度：dim / (num_banks_ * num_pim_chans_ * num_pim_ranks_ * num_grf_)
    // - 这里不区分奇偶 tile，直接按 pb_type 指定 EVEN/ODD 或 ALL_BANK 的扫描模式
    // - 流程：parkIn -> HAB -> programCrf -> HAB_PIM 执行 -> HAB -> SB -> parkOut
    int num_tile = dim / (num_banks_ * num_pim_chans_ * num_pim_ranks_ * num_grf_);
    int num_jump_to_be_taken = num_tile - 1;
    vector<PIMCmd> pim_cmds = PIMCmdGen::getPIMCmds(ktype, num_jump_to_be_taken, 0, 0);

    setControl(&bst_hab_pim_, true, getToggleCond(pb_type), false, false);
    setControl(&bst_hab_, false, getToggleCond(pb_type), false, false);

    parkIn();
    changePIMMode(dramMode::SB, dramMode::HAB);
    programCrf(pim_cmds);
    changePIMMode(dramMode::HAB, dramMode::HAB_PIM);

    if (ktype == KernelType::ADD || ktype == KernelType::MUL)
        computeAddOrMul(num_tile, input0_row, result_row, input1_row);
    else if (ktype == KernelType::RELU)
        computeRelu(num_tile, input0_row, result_row);
    /*
       else if (ktype == KernelType::BN)
       computeBn(num_tile, input0_row, result_row);
     */

    changePIMMode(dramMode::HAB_PIM, dramMode::HAB);
    changePIMMode(dramMode::HAB, dramMode::SB);
    parkOut();
}

void PIMKernel::computeAddOrMul(int num_tile, int input0_row, int result_row, int input1_row)
{
    // 每个 tile 上：
    // - 从 bank 读入到 GRF（BANK_TO_GRF_）
    // - 执行 ADD 或 MUL（ADD/MUL tag）
    // - 将 GRF 写回 bank（GRF_TO_BANK）
    // 偶/奇 bank 分别处理，保持与预置数据的奇偶一致
    for (int i = 0; i < num_tile; i++)
    {
        int c = num_grf_ * i;
        for (int b = 0; b < 2; b++)  // for even/odd banks, respectively
        {
            addTransactionAll(false, 0, b, input0_row, c, "BANK_TO_GRF_", &null_bst_, true,
                              num_grf_);
            addTransactionAll(false, 0, b, input1_row, c, "ADD", &null_bst_, true, num_grf_);
            addTransactionAll(true, 0, b, result_row, c, "GRF_TO_BANK", &null_bst_, true, num_grf_);
        }
    }
}

/*
void PIMKernel::computeBn(int num_tile, int input0_row, int result_row)
{
    for (int ch_idx = 0; ch_idx < num_pim_chans_; ch_idx++)
    {
        for (int ra_idx = 0; ra_idx < num_pim_ranks_; ra_idx++)
        {
            int srf_bst_num = (input0_row != result_row)? (ch_idx * num_pim_ranks_ + ra_idx) : 0;
            mem_->addTransaction(true, pim_addr_mgr_->addrGen(ch_idx, ra_idx, 0, 0, pim_reg_ra_,
                                       0x1), &srf_bst_[srf_bst_num]);
        }
    }
    addBarrier();

    if (input0_row != result_row)
        input0_row = result_row = 0;
    for (int i = 0; i < num_tile; i++)
    {
        for (int b = 0; b < 2; b++) // for even/ddd banks, respectively
        {
            addTransactionAll(false, 0, b, input0_row, num_grf_ * i, "MAD1", &null_bst_,
                              true, num_grf_);
            addTransactionAll(false, 0, b, input0_row, num_grf_ * i, "MAD2", &null_bst_,
                              true, num_grf_);
            addTransactionAll(true , 0, b, result_row, num_grf_ * i, "GRF_TO_BANK", &null_bst_,
                              true, num_grf_);
        }
    }
}
*/

void PIMKernel::computeRelu(int num_tile, int input0_row, int result_row)
{
    // ReLU：分别用 EVEN/ODD bank 路径把 GRF_A/GRF_B 的结果写回
    for (int i = 0; i < num_tile; i++)
    {
        int c = num_grf_ * i;
        addTransactionAll(false, 0, 0, input0_row, c, "FILL&ReLU", &null_bst_, true, num_grf_);
        addTransactionAll(true, 0, 0, result_row, c, "GRF_A_TO_EVEN_BANK", &null_bst_, true,
                          num_grf_);
        addTransactionAll(false, 0, 1, input0_row, c, "FILL&ReLU", &null_bst_, true, num_grf_);
        addTransactionAll(true, 0, 1, result_row, c, "GRF_B_TO_ODD_BANK", &null_bst_, true,
                          num_grf_);
    }
}

void PIMKernel::readData(BurstType* bst_data, size_t bst_cnt, unsigned starting_row,
                         unsigned starting_col)
{
    // 线性读：以 transaction_size_ 为步长，自起点地址顺序读出 bst_cnt 个 burst
    uint64_t init_addr = pim_addr_mgr_->addrGenSafe(0, 0, 0, 0, starting_row, starting_col);

    for (uint64_t addr = init_addr, i = 0; i < bst_cnt; addr += transaction_size_, i++)
    {
        mem_->addTransaction(false, addr, &bst_data[i]);
    }
}

void PIMKernel::adderTree(BurstType* result, int output_dim, int num_tile, int step, fp16* temp)
{
    // 简单二叉加法树：把 num_tile 份部分和规约成 1 份
    // step==0 时从 result[*output_dim] 里抽取各自的累加值，之后递归归并
    if (num_tile == 1)
        return;

    int iter = num_tile / 2;
    if (step == 0)
    {
        for (int i = 0; i < iter; i++)
        {
            temp[i] = result[2 * i * output_dim].fp16AdderTree() +
                      result[(2 * i + 1) * output_dim].fp16AdderTree();
        }
    }
    else
    {
        for (int i = 0; i < iter; i++) temp[i] = temp[i * 2] + temp[i * 2 + 1];

        if (num_tile % 2 == 1)
            temp[iter] = temp[num_tile];
    }

    adderTree(result, output_dim, ceil(double(num_tile) / (double)2), step + 1, temp);

    return;
}
