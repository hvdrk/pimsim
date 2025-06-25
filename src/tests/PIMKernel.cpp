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

    for (int& ch_idx : pim_chans_)
        for (int& ra_idx : pim_ranks_)
        {
            unsigned local_row = row;
            unsigned local_col = col;
            for (int i = 0; i < num_loop; i++)
            {
                // if (tag == "PART1") {
                //     std::cout << "local_row is " << local_row << ", local_col is " << local_col << ", num_loop is " << num_loop << ",i is " << i << std::endl;
                // }

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
    bst->u8Data_[0] = pim_op;
    bst->u8Data_[16] = crf_toggle_cond;
    bst->u8Data_[20] = grfA_zero;
    bst->u8Data_[21] = grfB_zero;
}

unsigned PIMKernel::getResultColGemv(int input_dim, int output_dim)
{
    int num_output_tiles = ceil(((double)output_dim / (num_total_pim_blocks_)) / num_grfB_);
    int num_input_tiles = ceil((double)input_dim / (double)num_grfA_);

    return num_output_tiles * num_input_tiles / 2 * num_grfA_ * num_grfB_;
}

void PIMKernel::changeBank(pimBankType pb_type, int& ch_idx, int& ra_idx, int& bg_idx,
                           int& bank_idx, unsigned& starting_row, unsigned& starting_col,
                           unsigned& row, unsigned& col)
{
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
    int input_tile_size = num_grfA_;
    int output_tile_size = num_grfB_ * num_total_pim_blocks_;

    int ch_idx = 0, ra_idx = 0, bg_idx = 0, bank_idx = 0;
    unsigned row = 0, col = 0;
    uint64_t addr;

    unsigned even_starting_row = starting_row, odd_starting_row = starting_row;
    unsigned even_starting_col = starting_col, odd_starting_col = starting_col;

    for (int y = 0; y < operand->bShape[0]; y += output_tile_size)
    {
        for (int x = 0; x < operand->bShape[1]; x += input_tile_size)
        {
            bool is_odd = ((x / input_tile_size) % 2 == 1) ? true : false;

            for (int tiled_y = 0; tiled_y < output_tile_size; tiled_y += num_grfB_)
            {
                row = (is_odd) ? odd_starting_row : even_starting_row;
                col = (is_odd) ? odd_starting_col : even_starting_col;

                for (int grfb_idx = 0; grfb_idx < num_grfB_; grfb_idx++)
                {
                    for (int grfa_idx = 0; grfa_idx < num_grfA_; grfa_idx++, col++)
                    {
                        addr = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx + is_odd,
                                                          row, col);
                        int d_idx = (y + tiled_y + grfb_idx) * operand->bShape[1] + x + grfa_idx;
                        mem_->addTransaction(true, addr, &operand->bData[d_idx]);
                    }
                }
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
    int num_output_tiles = ceil(((double)w_data->bShape[0] / (num_total_pim_blocks_)) / num_grfB_);
    int num_input_tiles = ceil((double)w_data->bShape[1] / (double)num_grfA_);
    int num_batch = i_data->bShape[0];
    int zero_row = 1000;

    if (is_tree)
    {
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
    setControl(&bst_hab_pim_, true, getToggleCond(), false, true);
    parkIn();
    changePIMMode(dramMode::SB, dramMode::HAB);
    programCrf(pim_cmds);

    for (int j = 0; j < num_output_tiles; j++)
    {
        for (int b = 0; b < num_batch; b++)
        {
            changePIMMode(dramMode::HAB, dramMode::HAB_PIM);  // PC reset.

            int col = num_output_tiles * num_input_tiles / 2 * num_grfA_ * num_grfB_ +
                      (j + b) * num_grfB_;
            if (is_tree)
            {
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
    for (int ch_idx = 0; ch_idx < num_pim_chans_; ch_idx++)
    {
        for (int ra_idx = 0; ra_idx < num_pim_ranks_; ra_idx++)
        {
            // input upload to GRF
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
    unsigned col = (num_grfA_ * num_grfB_) * (inputTile / 2 + outputTile * num_input_tiles / 2);

    for (int c_idx = 0; c_idx < 64; c_idx += 8)
        addTransactionAll(false, 0, (int)pb_type, row, col + c_idx, "MAC_", &null_bst_, true,
                          num_grfA_);
}

void PIMKernel::readResult(BurstType* resultBst, pimBankType pb_type, int output_dim,
                           uint64_t base_addr, unsigned starting_row, unsigned starting_col)
{
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
    int num_tile = dim / (num_banks_ * num_pim_chans_ * num_pim_ranks_ * num_grf_);
    std::cout << "num_tile is " << num_tile << std::endl;
    int num_jump_to_be_taken = num_tile - 1;
    vector<PIMCmd> pim_cmds = PIMCmdGen::getPIMCmds(ktype, num_jump_to_be_taken, 0, 0);

    // for (int i = 0; i < pim_cmds.size(); i++) {
    //     std::cout << pim_cmds[i].toStr();
    // }

    // setControl(&bst_hab_pim_, true, 0, false, false);
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
    uint64_t init_addr = pim_addr_mgr_->addrGenSafe(0, 0, 0, 0, starting_row, starting_col);

    // std::cout << "bst_cnt is " << bst_cnt << std::endl;

    for (uint64_t addr = init_addr, i = 0; i < bst_cnt; addr += transaction_size_, i++)
    {
        mem_->addTransaction(false, addr, &bst_data[i]);
    }
}

void PIMKernel::adderTree(BurstType* result, int output_dim, int num_tile, int step, fp16* temp)
{
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

// void PIMKernel::computeAddOrMul(int num_tile, int input0_row, int result_row, int input1_row)
// {
//     for (int i = 0; i < num_tile; i++)
//     {
//         int c = num_grf_ * i;
//         for (int b = 0; b < 2; b++)  // for even/odd banks, respectively
//         {
//             addTransactionAll(false, 0, b, input0_row, c, "BANK_TO_GRF_", &null_bst_, true,
//                               num_grf_);
//             addTransactionAll(false, 0, b, input1_row, c, "ADD", &null_bst_, true, num_grf_);
//             addTransactionAll(true, 0, b, result_row, c, "GRF_TO_BANK", &null_bst_, true, num_grf_);
//         }
//     }
// }


void PIMKernel::executeFirstPartition(int dim0, int dim1, pimBankType pb_type, KernelType ktype, int input0_row,
    int input1_row) //dim : # of burst
{

    int max_inner_row = dim0/32/16;
    int max_outer_row = dim1/32/16;

    // std::cout << "dim0, " << dim0 << std::endl;

    int num_jump_to_be_taken = max_inner_row - 1;
    // int num_jump_to_be_taken_inner = max_inner_row - 1;
    // int num_jump_to_be_taken_outer = max_outer_row - 1;


    vector<PIMCmd> pim_cmds = PIMCmdGen::getPIMCmds(ktype, num_jump_to_be_taken, 0, 0);


    setControl(&bst_hab_pim_, true, getToggleCond(pb_type), false, false);
    setControl(&bst_hab_, false, getToggleCond(pb_type), false, false);


    parkIn();
    changePIMMode(dramMode::SB, dramMode::HAB);
    programCrf(pim_cmds);
    changePIMMode(dramMode::HAB, dramMode::HAB_PIM);




    ///////////////////// first partition //////////////////////
    /////// for inner
    for (unsigned int row=0; row < max_inner_row; row++) {
        addTransactionAll(false, 0, 0, row, 0, "PART1", &null_bst_, true, 32);

        for (unsigned int src_bnk = 0; src_bnk < 16; src_bnk++) {
            for (unsigned int dst_bg = 0; dst_bg < 4; dst_bg++) {
                for (unsigned int dst_bnk = 0; dst_bnk < 4; dst_bnk++) {
                    addTransactionAll(true, 0, dst_bnk%2, (row + 8192), 0, "STB1", &null_bst_, true, 32);
                }
            }
        }
    }

    for (unsigned int src_bnk=0; src_bnk < 16; src_bnk++) {
        for (unsigned int dst_bg = 0; dst_bg < 4; dst_bg++) {
            for (unsigned int dst_bnk = 0; dst_bnk < 4; dst_bnk++) {
                addTransactionAll(false, 0, 0, max_inner_row + 8192 - 1, 0, "STS", &null_bst_, true, 32);
            }
        }

        addTransactionAll(true, 0, 0, max_inner_row + 8192 - 1, 0, "STB2", &null_bst_, true, 32);
    }

    addTransactionAll(true, 0, 0, max_inner_row + 8192 - 1, 0, "STB3", &null_bst_, true, 32);

    /////// for outer
    for (unsigned int row=0; row < max_outer_row; row++) {
        addTransactionAll(false, 0, 0, input1_row + row, 0, "PART1", &null_bst_, true, 32);

        for (unsigned int src_bnk = 0; src_bnk < 16; src_bnk++) {
            for (unsigned int dst_bg = 0; dst_bg < 4; dst_bg++) {
                for (unsigned int dst_bnk = 0; dst_bnk < 4; dst_bnk++) {
                    addTransactionAll(true, 0, dst_bnk%2, (input1_row + row + 8192), 0, "STB1", &null_bst_, true, 32);
                }
            }
        }
    }

    for (unsigned int src_bnk=0; src_bnk < 16; src_bnk++) {
        for (unsigned int dst_bg = 0; dst_bg < 4; dst_bg++) {
            for (unsigned int dst_bnk = 0; dst_bnk < 4; dst_bnk++) {
                addTransactionAll(false, 0, 0, input1_row + max_outer_row + 8192 - 1, 0, "STS", &null_bst_, true, 32);
            }
        }

        addTransactionAll(true, 0, 0, input1_row + max_outer_row + 8192 - 1, 0, "STB2", &null_bst_, true, 32);
    }

    addTransactionAll(true, 0, 0, input1_row + max_outer_row + input1_row + 8192 - 1, 0, "STB3", &null_bst_, true, 32);
    
    ///////////////////// first partition end //////////////////////

    changePIMMode(dramMode::HAB_PIM, dramMode::HAB);
    changePIMMode(dramMode::HAB, dramMode::SB);
    parkOut();
    return;
}


void PIMKernel::executeJoinBench(int dim0, int dim1, pimBankType pb_type, KernelType ktype, int input0_row,
    int input1_row) //dim : # of burst
{

    int max_inner_row = dim0/32/16;
    int max_outer_row = dim1/32/16;

    std::cout << "dim0, " << dim0 << std::endl;

    // int num_jump_to_be_taken = max_inner_row - 1;
    // int num_jump_to_be_taken_inner = max_inner_row - 1;
    // int num_jump_to_be_taken_outer = max_outer_row - 1;

    

    int total_cmds = 0;

    int is_uniform_part1 = 0;
    int is_uniform_part2 = 0;
    int is_uniform_part3 = 0;
    double bias_part1 = 1/16;
    double bias_part2 = 1/16;
    double bias_part3 = 1/16;


    std::vector<int> tuples1;
    tuples1 = make_tuples(max_inner_row * 16 * 128, is_uniform_part1, 128*16, bias_part1);

    
    std::vector<int> tuples1_2;
    tuples1_2 = make_tuples(max_outer_row * 16 * 128, is_uniform_part1, 128*16, bias_part1);



    std::vector<int> tuples2[16];
    for (int i=0; i<16; i++) {
        tuples2[i] = make_tuples(max_inner_row * 128, is_uniform_part2, 128, bias_part2);
    }

    std::vector<int> tuples2_2[16];
    for (int i=0; i<16; i++) {
        tuples2_2[i] = make_tuples(max_inner_row * 128, is_uniform_part2, 128, bias_part2);
    }

    int local_changes = 0;
    int max_changes = 0;


    int total_part = 16;
    std::vector<int> tuples3[total_part][16];
    for (int n_part=0; n_part < total_part; n_part++) {
        for (int i=0; i<16; i++) {
            tuples3[n_part][i] = make_tuples(max_inner_row/total_part * 128, is_uniform_part3, 128, bias_part3);
        }
    }

    std::vector<int> tuples3_2[total_part][16];
    for (int n_part=0; n_part < total_part; n_part++) {
        for (int i=0; i<16; i++) {
            tuples3_2[n_part][i] = make_tuples(max_inner_row/total_part * 128, is_uniform_part3, 128, bias_part3);
        }
    }


    int sram_state[16][16] = {0};
    size_t idx = 0;
    int part_num = 0;
    int last_bank = 0;
    int bank_num = 0;


    ///////////////////// first partition //////////////////////
    /////// for left

    sram_state[16][16] = {0};
    idx = 0;

    std::cout << "max_inner_row is " << max_inner_row << std::endl;

    for (int row=0; row < max_inner_row; row++) {
        total_cmds++;
        for (int bnk=0; bnk < 16; bnk++) {
            for (int col=0; col < 128; col++) {
                part_num = tuples1[idx] & 15;
                sram_state[bnk][part_num]++;
                if (sram_state[bnk][part_num] >= 128) {
                    sram_state[bnk][part_num] -= 128;
                    total_cmds++;
                }
                idx++;
            }
        }
    }

    std::cout << "total_cmds is " << total_cmds << std::endl;

    

    for (unsigned int src_bnk=0; src_bnk < 16; src_bnk++) {
        for (unsigned int dst_bg = 0; dst_bg < 4; dst_bg++) {
            for (unsigned int dst_bnk = 0; dst_bnk < 4; dst_bnk++) {
                total_cmds++;
                // addTransactionAll(false, 0, 0, 0, 0, "STS", &null_bst_, true, 32);
            }
        }
        total_cmds++;
        // addTransactionAll(true, 0, 0, 1, 0, "STB2", &null_bst_, true, 32);
    }
    total_cmds++;
    // addTransactionAll(true, 0, 0, 0, 0, "STB3", &null_bst_, true, 32);


    /////// for right

    sram_state[16][16] = {0};
    idx = 0;

    for (int row=0; row < max_outer_row; row++) {
        total_cmds++;
        // addTransactionAll(false, 0, 0, row, 0, "PART1", &null_bst_, true, 32);
        last_bank = 0;
        bank_num = 0;
        for (int bnk=0; bnk < 16; bnk++) {
            for (int col=0; col < 128; col++) {
                part_num = tuples1_2[idx] & 15;
                sram_state[bnk][part_num]++;
                if (sram_state[bnk][part_num] >= 128) {
                    sram_state[bnk][part_num] -= 128;
                    if (last_bank != part_num) {
                        bank_num = (bank_num + 1)%2;
                    }
                    total_cmds++;
                    // addTransactionAll(true, 0, bank_num, (row + 8192), 0, "STB1", &null_bst_, true, 32);
                }
                idx++;
            }
        }
    }

    for (unsigned int src_bnk=0; src_bnk < 16; src_bnk++) {
        for (unsigned int dst_bg = 0; dst_bg < 4; dst_bg++) {
            for (unsigned int dst_bnk = 0; dst_bnk < 4; dst_bnk++) {
                total_cmds++;
                // addTransactionAll(false, 0, 0, 0, 0, "STS", &null_bst_, true, 32);
            }
        }
        total_cmds++;
        // addTransactionAll(true, 0, 0, 1, 0, "STB2", &null_bst_, true, 32);
    }

    total_cmds++;
    // addTransactionAll(true, 0, 0, 2, 0, "STB3", &null_bst_, true, 32);

    ///////////////////// first partition end //////////////////////
    std::cout << "first partitoin end. total_cmds is " << total_cmds << std::endl;

    ///////////////////// second partition //////////////////////
    /////// for left

    

    local_changes = 0;
    max_changes = 0;
    sram_state[16][16] = {0};
    idx = 0;

    for (int row=0; row < max_inner_row; row++) {
        total_cmds++;
        // addTransactionAll(false, 0, 0, row, 0, "PART2", &null_bst_, true, 32);
        max_changes = 0;
        for (int bnk=0; bnk < 16; bnk++) {
            local_changes = 0;
            for (int col=0; col < 128; col++) {
                part_num = tuples2[bnk][idx] & 15;
                sram_state[bnk][part_num]++;
                if (sram_state[bnk][part_num] >= 128) {
                    sram_state[bnk][part_num] -= 128;
                    local_changes++;
                }
                idx++;
            }
            idx -= 128;
            if (local_changes > max_changes) {
                max_changes = local_changes;
            }
        }
        for (int n_change=0; n_change<max_changes; n_change++) {
            total_cmds++;
            // addTransactionAll(true, 0, 0, (row + 8192 + n_change), 0, "STB4", &null_bst_, true, 32);
        }
        idx += 128;
    }
    total_cmds++;
    // addTransactionAll(true, 0, 0, 2, 0, "STB5", &null_bst_, true, 32);

    /////// for right

    local_changes = 0;
    max_changes = 0;
    sram_state[16][16] = {0};
    idx = 0;

    for (int row=0; row < max_outer_row; row++) {
        total_cmds++;
        // addTransactionAll(false, 0, 0, row, 0, "PART2", &null_bst_, true, 32);
        max_changes = 0;
        for (int bnk=0; bnk < 16; bnk++) {
            local_changes = 0;
            for (int col=0; col < 128; col++) {
                part_num = tuples2_2[bnk][idx] & 15;
                sram_state[bnk][part_num]++;
                if (sram_state[bnk][part_num] >= 128) {
                    sram_state[bnk][part_num] -= 128;
                    local_changes++;
                }
                idx++;
            }
            idx -= 128;
            if (local_changes > max_changes) {
                max_changes = local_changes;
            }
        }
        for (int n_change=0; n_change<max_changes; n_change++) {
            total_cmds++;
            // addTransactionAll(true, 0, 0, (row + 8192 + n_change), 0, "STB4", &null_bst_, true, 32);
        }
        idx += 128;
    }
    total_cmds++;
    // addTransactionAll(true, 0, 0, 2, 0, "STB5", &null_bst_, true, 32);

    ///////////////////// second partition end //////////////////////


    std::cout << "second partitoin end. total_cmds is " << total_cmds << std::endl;

    ///////////////////// third partition //////////////////////
    /////// for left

    

    local_changes = 0;
    max_changes = 0;

    for (int n_part=0; n_part < total_part; n_part++){

        sram_state[16][16] = {0};
        idx = 0;
        for (int row=0; row < max_inner_row/total_part; row++) {
            total_cmds++;
            // addTransactionAll(false, 0, 0, row, 0, "PART3", &null_bst_, true, 32);
            max_changes = 0;
            for (int bnk=0; bnk < 16; bnk++) {
                local_changes = 0;
                for (int col=0; col < 128; col++) {
                    part_num = tuples3[n_part][bnk][idx] & 15;
                    sram_state[bnk][part_num]++;
                    if (sram_state[bnk][part_num] >= 128) {
                        sram_state[bnk][part_num] -= 128;
                        local_changes++;
                    }
                    idx++;
                }
                idx -= 128;
                if (local_changes > max_changes) {
                    max_changes = local_changes;
                }
            }
            for (int n_change=0; n_change<max_changes; n_change++) {
                total_cmds++;
                // addTransactionAll(true, 0, 0, (row + 8192 + n_change), 0, "STB4", &null_bst_, true, 32);
            }
            idx += 128;
        } 

        total_cmds++;
        // addTransactionAll(true, 0, 0, 2, 0, "STB5", &null_bst_, true, 32);
    }


    /////// for right

    local_changes = 0;
    max_changes = 0;


    for (int n_part=0; n_part < total_part; n_part++){

        sram_state[16][16] = {0};
        idx = 0;
        for (int row=0; row < max_outer_row/total_part; row++) {
            total_cmds++;
            // addTransactionAll(false, 0, 0, row, 0, "PART3", &null_bst_, true, 32);
            max_changes = 0;
            for (int bnk=0; bnk < 16; bnk++) {
                local_changes = 0;
                for (int col=0; col < 128; col++) {
                    part_num = tuples3_2[n_part][bnk][idx] & 15;
                    sram_state[bnk][part_num]++;
                    if (sram_state[bnk][part_num] >= 128) {
                        sram_state[bnk][part_num] -= 128;
                        local_changes++;
                    }
                    idx++;
                }
                idx -= 128;
                if (local_changes > max_changes) {
                    max_changes = local_changes;
                }
            }
            for (int n_change=0; n_change<max_changes; n_change++) {
                total_cmds++;
                // addTransactionAll(true, 0, 0, (row + 8192 + n_change), 0, "STB4", &null_bst_, true, 32);
            }
            idx += 128;
        } 

        total_cmds++;
        // addTransactionAll(true, 0, 0, 2, 0, "STB5", &null_bst_, true, 32);
    }
    std::cout << "third partitoin end. total_cmds is " << total_cmds << std::endl;

    total_part *= 16;

    ///////////////////// third partition end //////////////////////

    ///////////////////// join start //////////////////////

    for (int n_part=0; n_part < total_part; n_part++){
        
        for (int row=0; row < max_inner_row/total_part; row++) {
            total_cmds++;
            // addTransactionAll(false, 0, 0, row, 0, "BUILD_TABLE", &null_bst_, true, 32);
        }

        for (int row=0; row < max_inner_row/total_part; row++) {
            total_cmds++;
            // addTransactionAll(false, 0, 0, row + input1_row, 0, "PROBE_TABLE", &null_bst_, true, 32);
        }

        for (int row=0; row < max_inner_row/total_part; row++) {
            total_cmds++;
            // addTransactionAll(true, 0, 0, row + 8192, 0, "STB", &null_bst_, true, 32);
        }

    }
    std::cout << "join end. total_cmds is " << total_cmds << std::endl;

    ///////////////////// join end //////////////////////

    int num_jump_to_be_taken = total_cmds - 1;
    std::cout << "num_jump_to_be_taken is " << num_jump_to_be_taken << std::endl;
    vector<PIMCmd> pim_cmds = PIMCmdGen::getPIMCmds(ktype, num_jump_to_be_taken, 0, 0, 1);

    setControl(&bst_hab_pim_, true, getToggleCond(pb_type), false, false);
    setControl(&bst_hab_, false, getToggleCond(pb_type), false, false);


    parkIn();
    changePIMMode(dramMode::SB, dramMode::HAB);
    programCrf(pim_cmds);
    changePIMMode(dramMode::HAB, dramMode::HAB_PIM);



    ///////////////////// first partition //////////////////////
    /////// for left

    sram_state[16][16] = {0};
    idx = 0;

    for (int row=0; row < max_inner_row; row++) {
        addTransactionAll(false, 0, 0, row, 0, "PART1", &null_bst_, true, 32);
        last_bank = 0;
        bank_num = 0;
        for (int bnk=0; bnk < 16; bnk++) {
            for (int col=0; col < 128; col++) {
                part_num = tuples1[idx] & 15;
                sram_state[bnk][part_num]++;
                if (sram_state[bnk][part_num] >= 128) {
                    sram_state[bnk][part_num] -= 128;
                    if (last_bank != part_num) {
                        bank_num = (bank_num + 1)%2;
                    }
                    addTransactionAll(true, 0, bank_num, (row + 8192), 0, "STB1", &null_bst_, true, 32);
                }
                idx++;
            }
        }
    }

    for (unsigned int src_bnk=0; src_bnk < 16; src_bnk++) {
        for (unsigned int dst_bg = 0; dst_bg < 4; dst_bg++) {
            for (unsigned int dst_bnk = 0; dst_bnk < 4; dst_bnk++) {
                addTransactionAll(false, 0, 0, 0, 0, "STS", &null_bst_, true, 32);
            }
        }

        addTransactionAll(true, 0, 0, 1, 0, "STB2", &null_bst_, true, 32);
    }

    addTransactionAll(true, 0, 0, 0, 0, "STB3", &null_bst_, true, 32);

    

    /////// for right

    sram_state[16][16] = {0};
    idx = 0;

    for (int row=0; row < max_outer_row; row++) {
        addTransactionAll(false, 0, 0, row, 0, "PART1", &null_bst_, true, 32);
        last_bank = 0;
        bank_num = 0;
        for (int bnk=0; bnk < 16; bnk++) {
            for (int col=0; col < 128; col++) {
                part_num = tuples1_2[idx] & 15;
                sram_state[bnk][part_num]++;
                if (sram_state[bnk][part_num] >= 128) {
                    sram_state[bnk][part_num] -= 128;
                    if (last_bank != part_num) {
                        bank_num = (bank_num + 1)%2;
                    }
                    addTransactionAll(true, 0, bank_num, (row + 8192), 0, "STB1", &null_bst_, true, 32);
                }
                idx++;
            }
        }
    }

    for (unsigned int src_bnk=0; src_bnk < 16; src_bnk++) {
        for (unsigned int dst_bg = 0; dst_bg < 4; dst_bg++) {
            for (unsigned int dst_bnk = 0; dst_bnk < 4; dst_bnk++) {
                addTransactionAll(false, 0, 0, 0, 0, "STS", &null_bst_, true, 32);
            }
        }

        addTransactionAll(true, 0, 0, 1, 0, "STB2", &null_bst_, true, 32);
    }

    addTransactionAll(true, 0, 0, 2, 0, "STB3", &null_bst_, true, 32);

    ///////////////////// first partition end //////////////////////

    ///////////////////// second partition //////////////////////
    /////// for left

    

    local_changes = 0;
    max_changes = 0;
    sram_state[16][16] = {0};
    idx = 0;

    for (int row=0; row < max_inner_row; row++) {
        addTransactionAll(false, 0, 0, row, 0, "PART2", &null_bst_, true, 32);
        max_changes = 0;
        for (int bnk=0; bnk < 16; bnk++) {
            local_changes = 0;
            for (int col=0; col < 128; col++) {
                part_num = tuples2[bnk][idx] & 15;
                sram_state[bnk][part_num]++;
                if (sram_state[bnk][part_num] >= 128) {
                    sram_state[bnk][part_num] -= 128;
                    local_changes++;
                }
                idx++;
            }
            idx -= 128;
            if (local_changes > max_changes) {
                max_changes = local_changes;
            }
        }
        for (int n_change=0; n_change<max_changes; n_change++) {
            addTransactionAll(true, 0, 0, (row + 8192 + n_change), 0, "STB4", &null_bst_, true, 32);
        }
        idx += 128;
    }

    addTransactionAll(true, 0, 0, 2, 0, "STB5", &null_bst_, true, 32);

    /////// for right

    local_changes = 0;
    max_changes = 0;
    sram_state[16][16] = {0};
    idx = 0;

    for (int row=0; row < max_outer_row; row++) {
        addTransactionAll(false, 0, 0, row, 0, "PART2", &null_bst_, true, 32);
        max_changes = 0;
        for (int bnk=0; bnk < 16; bnk++) {
            local_changes = 0;
            for (int col=0; col < 128; col++) {
                part_num = tuples2_2[bnk][idx] & 15;
                sram_state[bnk][part_num]++;
                if (sram_state[bnk][part_num] >= 128) {
                    sram_state[bnk][part_num] -= 128;
                    local_changes++;
                }
                idx++;
            }
            idx -= 128;
            if (local_changes > max_changes) {
                max_changes = local_changes;
            }
        }
        for (int n_change=0; n_change<max_changes; n_change++) {
            addTransactionAll(true, 0, 0, (row + 8192 + n_change), 0, "STB4", &null_bst_, true, 32);
        }
        idx += 128;
    }

    addTransactionAll(true, 0, 0, 2, 0, "STB5", &null_bst_, true, 32);

    ///////////////////// second partition end //////////////////////



    ///////////////////// third partition //////////////////////
    /////// for left

    
    total_part = 16;
    local_changes = 0;
    max_changes = 0;

    for (int n_part=0; n_part < total_part; n_part++){
        
        // std::cout << "2 ^^^^^^^^" << std::endl;
        sram_state[16][16] = {0};
        idx = 0;
        for (int row=0; row < max_inner_row/total_part; row++) {
            
            addTransactionAll(false, 0, 0, row, 0, "PART3", &null_bst_, true, 32);
            max_changes = 0;
            for (int bnk=0; bnk < 16; bnk++) {
                local_changes = 0;
                for (int col=0; col < 128; col++) {
                    part_num = tuples3[n_part][bnk][idx] & 15;
                    sram_state[bnk][part_num]++;
                    if (sram_state[bnk][part_num] >= 128) {
                        sram_state[bnk][part_num] -= 128;
                        local_changes++;
                    }
                    idx++;
                }
                idx -= 128;
                if (local_changes > max_changes) {
                    max_changes = local_changes;
                }
            }
            for (int n_change=0; n_change<max_changes; n_change++) {
                addTransactionAll(true, 0, 0, (row + 8192 + n_change), 0, "STB4", &null_bst_, true, 32);
            }
            idx += 128;
        } 

        addTransactionAll(true, 0, 0, 2, 0, "STB5", &null_bst_, true, 32);
    }


    /////// for right

    local_changes = 0;
    max_changes = 0;


    for (int n_part=0; n_part < total_part; n_part++){

        sram_state[16][16] = {0};
        idx = 0;
        for (int row=0; row < max_outer_row/total_part; row++) {
            addTransactionAll(false, 0, 0, row, 0, "PART3", &null_bst_, true, 32);
            max_changes = 0;
            for (int bnk=0; bnk < 16; bnk++) {
                local_changes = 0;
                for (int col=0; col < 128; col++) {
                    part_num = tuples3_2[n_part][bnk][idx] & 15;
                    sram_state[bnk][part_num]++;
                    if (sram_state[bnk][part_num] >= 128) {
                        sram_state[bnk][part_num] -= 128;
                        local_changes++;
                    }
                    idx++;
                }
                idx -= 128;
                if (local_changes > max_changes) {
                    max_changes = local_changes;
                }
            }
            for (int n_change=0; n_change<max_changes; n_change++) {
                addTransactionAll(true, 0, 0, (row + 8192 + n_change), 0, "STB4", &null_bst_, true, 32);
            }
            idx += 128;
        } 

        addTransactionAll(true, 0, 0, 2, 0, "STB5", &null_bst_, true, 32);
    }

    total_part *= 16;

    ///////////////////// third partition end //////////////////////

    ///////////////////// join start //////////////////////

    for (int n_part=0; n_part < total_part; n_part++){
        
        for (int row=0; row < max_inner_row/total_part; row++) {
            addTransactionAll(false, 0, 0, row, 0, "BUILD_TABLE", &null_bst_, true, 32);
        }

        for (int row=0; row < max_inner_row/total_part; row++) {
            addTransactionAll(false, 0, 0, row + input1_row, 0, "PROBE_TABLE", &null_bst_, true, 32);
        }

        for (int row=0; row < max_inner_row/total_part; row++) {
            addTransactionAll(true, 0, 0, row + 8192, 0, "STB", &null_bst_, true, 32);
        }

    }

    ///////////////////// join end //////////////////////

    

    changePIMMode(dramMode::HAB_PIM, dramMode::HAB);
    changePIMMode(dramMode::HAB, dramMode::SB);
    parkOut();

    return;
}

// void PIMKernel::preloadNoReplacement(NumpyBurstType* operand, unsigned starting_row,
//     unsigned starting_col)
// {
// uint64_t init_addr = pim_addr_mgr_->addrGenSafe(0, 0, 0, 0, starting_row, starting_col);

// for (int x = 0; x < operand->getTotalDim(); x++)
// {
// uint64_t addr = init_addr + x * transaction_size_;
// mem_->addTransaction(true, addr, &operand->bData[x]);
// }
// }

std::vector<int> PIMKernel::make_tuples(std::size_t n, int is_uniform, std::size_t chunk_size, double bias)
{
    // ---- 입력 검증 ----
    if (n == 0 || n % 16 != 0){
        std::cout << "n: " << n << std::endl;
        throw std::invalid_argument("n must be a positive multiple of 16");
    }
        
    if (chunk_size == 0 || chunk_size % 16 != 0 || n % chunk_size != 0)
        throw std::invalid_argument("chunk_size must divide n and be a multiple of 16");
    if (bias < 0.0 || bias > 1.0)
        throw std::invalid_argument("bias must be in [0,1]");

    constexpr int K = 16;                    // 값 종류
    const std::size_t per_value = n / K;     // 각 값이 가져야 하는 개수

    std::array<std::size_t, K> remaining;
    remaining.fill(per_value);

    std::vector<int> result;
    result.reserve(n);

    std::random_device rd;
    std::mt19937       rng(rd());

    // ---- 1. is_uniform == 1 : 완전 균등 ----
    if (is_uniform == 1) {
        for (int v = 0; v < K; ++v)
            result.insert(result.end(), per_value, v);
        std::shuffle(result.begin(), result.end(), rng);
        return result;
    }

    // ---- 2. 편향 모드 ----
    const std::size_t num_chunks = n / chunk_size;
    std::uniform_int_distribution<int> pick_value(0, K - 1);

    for (std::size_t c = 0; c < num_chunks; ++c) {

        // (a) dominant 값 선정 (아직 남은 것 중 랜덤)
        int dominant = pick_value(rng);
        for (int i = 0; i < K && remaining[dominant] == 0; ++i)
            dominant = (dominant + 1) % K;

        // (b) 청크 내부 각 위치 채우기
        for (std::size_t slot = 0; slot < chunk_size; ++slot) {
            std::array<double, K> probs{};
            std::size_t slots_left = (chunk_size - slot);  // 현재 위치 포함 남은 슬롯 수

            // --- dominant 확률 ---
            double p_dom = bias;
            double max_dom = static_cast<double>(remaining[dominant]) / slots_left;
            if (p_dom > max_dom) p_dom = max_dom;          // 남은 개수 한계로 보정

            // --- non-dominant 확률 ---
            double rest_prob = 1.0 - p_dom;
            int non_dom_cnt = 0;
            for (int v = 0; v < K; ++v)
                if (v != dominant && remaining[v] > 0) ++non_dom_cnt;

            // dominant 남은 개수도 0이면 p_dom = 0
            if (remaining[dominant] == 0) {
                p_dom = 0.0;
                rest_prob = 1.0;
            }

            double each_non_dom = (non_dom_cnt > 0) ? rest_prob / non_dom_cnt : 0.0;

            for (int v = 0; v < K; ++v) {
                if (v == dominant) probs[v] = p_dom;
                else probs[v] = (remaining[v] > 0) ? each_non_dom : 0.0;
            }

            // (c) 실제 값 샘플링
            std::discrete_distribution<int> dist(probs.begin(), probs.end());
            int picked = dist(rng);

            // 방어적: 만일 remaining[picked] == 0 이면, 다른 값 찾기
            if (remaining[picked] == 0) {
                for (int v = 0; v < K; ++v)
                    if (remaining[v] > 0) { picked = v; break; }
            }

            result.push_back(picked);
            --remaining[picked];
        }
    }

    // ---- 3. 남은 개수(이론상 0이어야 함) 확인 ----
    for (int v = 0; v < K; ++v)
        if (remaining[v] != 0)
            throw std::logic_error("count mismatch: algorithm bug");

    return result;
}