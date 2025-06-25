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

#include "PIMRank.h"

#include <bitset>
#include <iostream>

#include "AddressMapping.h"
#include "PIMCmd.h"

using namespace std;
using namespace DRAMSim;

PIMRank::PIMRank(ostream& simLog, Configuration& configuration)
    : chanId(-1),
      rankId(-1),
      dramsimLog(simLog),
      pimPC_(0),
      lastJumpIdx_(-1),
      numJumpToBeTaken_(-1),
      lastRepeatIdx_(-1),
      numRepeatToBeDone_(-1),
      crfExit_(false),
      config(configuration),
      pimBlocks(getConfigParam(UINT, "NUM_PIM_BLOCKS"),
                PIMBlock(PIMConfiguration::getPIMPrecision()))
{
    currentClockCycle = 0;
}

void PIMRank::attachRank(Rank* r)
{
    this->rank = r;
}

void PIMRank::setChanId(int id)
{
    this->chanId = id;
}

void PIMRank::setRankId(int id)
{
    this->rankId = id;
}

int PIMRank::getChanId() const
{
    return this->chanId;
}

int PIMRank::getRankId() const
{
    return this->rankId;
}

void PIMRank::update() {}

void PIMRank::controlPIM(BusPacket* packet)
{
    uint8_t grf_a_zeroize = packet->data->u8Data_[20];
    if (grf_a_zeroize)
    {
        if (DEBUG_CMD_TRACE)
        {
            PRINTC(RED, OUTLOG_CH_RA("GRF_A_ZEROIZE"));
        }
        BurstType burst_zero;
        for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
        {
            for (int i = 0; i < 8; i++) pimBlocks[pb].grfA[i] = burst_zero;
        }
    }
    uint8_t grf_b_zeroize = packet->data->u8Data_[21];
    if (grf_b_zeroize)
    {
        if (DEBUG_CMD_TRACE)
        {
            PRINTC(RED, OUTLOG_CH_RA("GRF_B_ZEROIZE"));
        }
        BurstType burst_zero;
        for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
        {
            for (int i = 0; i < 8; i++) pimBlocks[pb].grfB[i] = burst_zero;
        }
    }
    pimOpMode_ = packet->data->u8Data_[0] & 1;
    toggleEvenBank_ = !(packet->data->u8Data_[16] & 1);
    toggleOddBank_ = !(packet->data->u8Data_[16] & 2);
    toggleRa13h_ = (packet->data->u8Data_[16] & 4);

    if (pimOpMode_)
    {
        rank->mode_ = dramMode::HAB_PIM;
        pimPC_ = 0;
        lastJumpIdx_ = numJumpToBeTaken_ = lastRepeatIdx_ = numRepeatToBeDone_ = -1;
        crfExit_ = false;
        PRINTC(RED, OUTLOG_CH_RA("HAB_PIM"));
    }
    else
    {
        rank->mode_ = dramMode::HAB;
        PRINTC(RED, OUTLOG_CH_RA("HAB mode"));
    }
}

bool PIMRank::isToggleCond(BusPacket* packet)
{
    if (crfExit_) return false;
    if (packet->tag.compare(0, 11, "GRF_TO_BANK") == 0 
        || packet->tag.compare(0, 4, "PART") == 0
        || packet->tag.compare(0, 3, "STB") == 0
        || packet->tag.compare(0, 3, "STS") == 0
        || packet->tag.compare(0, 3, "HASH") == 0
        ) {
        return true;
    }
    if (pimOpMode_ && !crfExit_)
    {
        if (toggleRa13h_)
        {
            if (toggleEvenBank_ && ((packet->bank & 1) == 0))
                return true;
            else if (toggleOddBank_ && ((packet->bank & 1) == 1))
                return true;
            return false;
        }
        else if (!toggleRa13h_ && !isReservedRA(packet->row))
        {
            if (toggleEvenBank_ && ((packet->bank & 1) == 0))
                return true;
            else if (toggleOddBank_ && ((packet->bank & 1) == 1))
                return true;
            return false;
        }
        return false;
    }
    else
    {
        return false;
    }
}

void PIMRank::readHab(BusPacket* packet)
{
    if (isReservedRA(packet->row))  // ignored
    {
        PRINTC(GRAY, OUTLOG_ALL("READ"));
    }
    else
    {
        PRINTC(GRAY, OUTLOG_ALL("BANK_TO_PIM"));
#ifndef NO_STORAGE
        int grf_id = getGrfIdx(packet->column);
        for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
        {
            rank->banks[pb * 2 + packet->bank].read(packet);
            pimBlocks[pb].grfB[grf_id] = *(packet->data);
        }
#endif
    }
}

void PIMRank::writeHab(BusPacket* packet)
{
    if (packet->row == config.PIM_REG_RA)  // WRIO to PIM Broadcasting
    {
        if (packet->column == 0x00)
            controlPIM(packet);
        if ((0x08 <= packet->column && packet->column <= 0x0f) ||
            (0x18 <= packet->column && packet->column <= 0x1f))
        {
            if (DEBUG_CMD_TRACE)
            {
                if (packet->column - 8 < 8)
                    PRINTC(GREEN, OUTLOG_B_GRF_A("BWRITE_GRF_A"));
                else
                    PRINTC(GREEN, OUTLOG_B_GRF_B("BWRITE_GRF_B"));
            }
#ifndef NO_STORAGE
            for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
            {
                if (packet->column - 8 < 8)
                    pimBlocks[pb].grfA[packet->column - 0x8] = *(packet->data);
                else
                    pimBlocks[pb].grfB[packet->column - 0x18] = *(packet->data);
            }
#endif
        }
        else if (0x04 <= packet->column && packet->column <= 0x07)
        {
            if (DEBUG_CMD_TRACE)
                PRINTC(GREEN, OUTLOG_B_CRF("BWRITE_CRF"));
            crf.bst[packet->column - 0x04] = *(packet->data);
        }
        else if (packet->column == 0x1)
        {
            if (DEBUG_CMD_TRACE)
                PRINTC(GREEN, OUTLOG_CH_RA("BWRITE_SRF"));
            for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++) pimBlocks[pb].srf = *(packet->data);
        }
    }
    else if (isReservedRA(packet->row))
    {
        PRINTC(GRAY, OUTLOG_ALL("WRITE"));
    }
    else  // PIM (only GRF) to Bank Move
    {
        PRINTC(GREEN, OUTLOG_ALL("PIM_TO_BANK"));

#ifndef NO_STORAGE
        int grf_id = getGrfIdx(packet->column);
        for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
        {
            if (packet->bank == 0)
            {
                *(packet->data) = pimBlocks[pb].grfA[grf_id];
                rank->banks[pb * 2].write(packet);  // basically read from bank;
            }
            else if (packet->bank == 1)
            {
                *(packet->data) = pimBlocks[pb].grfB[grf_id];
                rank->banks[pb * 2 + 1].write(packet);  // basically read from bank.
            }
        }
#endif
    }
}

void PIMRank::readOpd(int pb, BurstType& bst, PIMOpdType type, BusPacket* packet, int idx,
                      bool is_auto, bool is_mac, int sram_idx)
{
    // idx = getGrfIdx(idx);

    switch (type)
    {
        case PIMOpdType::A_OUT:
            bst = pimBlocks[pb].aOut;
            return;
        case PIMOpdType::M_OUT:
            bst = pimBlocks[pb].mOut;
            return;
        case PIMOpdType::EVEN_BANK:
            if (packet->bank % 2 != 0)
                PRINT("Warning, CRF bank coding and bank id from packet are inconsistent");
            rank->banks[pb * 2].read(packet);  // basically read from bank.
            bst = *(packet->data);
            return;
        case PIMOpdType::ODD_BANK:
            if (packet->bank % 2 == 0)
                PRINT("Warning, CRF bank coding and bank id from packet are inconsistent");
            rank->banks[pb * 2 + 1].read(packet);  // basically read from bank.
            bst = *(packet->data);
            return;
        case PIMOpdType::GRF_A:
            bst = pimBlocks[pb].grfA[(is_auto) ? getGrfIdx(packet->column) : getGrfIdx(idx)];
            return;
        case PIMOpdType::GRF_B:
            if (is_auto)
                bst = pimBlocks[pb].grfB[(is_mac) ? getGrfIdxHigh(packet->row, packet->column)
                                                  : getGrfIdx(packet->column)];
            else
                bst = pimBlocks[pb].grfB[getGrfIdx(idx)];
            return;
        // case PIMOpdType::SRF_M:
        //     bst.set(pimBlocks[pb].srf.fp16Data_[getGrfIdx(idx)]);
        //     return;
        // case PIMOpdType::SRF_A:
        //     bst.set(pimBlocks[pb].srf.fp16Data_[getGrfIdx(idx) + 8]);
        //     return;
        case PIMOpdType::SRAM:
            idx = idx & 15;
            for (int i=0; i<4; i++) {
                sram_idx = sram_idx & 255;
                if (is_auto && (pimBlocks[pb].idx_flag[idx] == false) && (pimBlocks[pb].sram_idx[idx] <= sram_idx)) {
                    bst.TupleData_[i].key = -1;
                    bst.TupleData_[i].value = -1;
                }
                else {
                    bst.TupleData_[i].key = pimBlocks[pb].sram[idx][sram_idx].key;
                    bst.TupleData_[i].value = pimBlocks[pb].sram[idx][sram_idx].value;
                }
                sram_idx++;
            }
            return;
        case PIMOpdType::BANK:
            rank->banks[pb].read(packet);  // basically read from bank.
            bst = *(packet->data);
            return;
    }
}

void PIMRank::writeOpd(int pb, BurstType& bst, PIMOpdType type, BusPacket* packet, int idx,
                       bool is_auto, bool is_mac, int sram_idx)
{
    // idx = getGrfIdx(idx);

    switch (type)
    {
        case PIMOpdType::A_OUT:
            pimBlocks[pb].aOut = bst;
            return;
        case PIMOpdType::M_OUT:
            pimBlocks[pb].mOut = bst;
            return;
        case PIMOpdType::EVEN_BANK:
            if (packet->bank % 2 != 0)
            {
                PRINT("CRF bank coding and bank id from packet are inconsistent");
            }
            *(packet->data) = bst;
            rank->banks[pb * 2].write(packet);  // basically read from bank.
            return;
        case PIMOpdType::ODD_BANK:
            if (packet->bank % 2 == 0)
            {
                PRINT("CRF bank coding and bank id from packet are inconsistent");
                exit(-1);
            }
            *(packet->data) = bst;
            rank->banks[pb * 2 + 1].write(packet);  // basically read from bank.
            return;
        case PIMOpdType::GRF_A:
            pimBlocks[pb].grfA[(is_auto) ? getGrfIdx(packet->column) : getGrfIdx(idx)] = bst;
            return;
        case PIMOpdType::GRF_B:
            if (is_auto)
                pimBlocks[pb].grfB[(is_mac) ? getGrfIdxHigh(packet->row, packet->column)
                                            : getGrfIdx(packet->column)] = bst;
            else
                pimBlocks[pb].grfB[getGrfIdx(idx)] = bst;
            return;
        // case PIMOpdType::SRF_M:
        //     pimBlocks[pb].srf = bst;
        //     return;
        // case PIMOpdType::SRF_A:
        //     pimBlocks[pb].srf = bst;
        //     return;
        case PIMOpdType::SRAM:
            idx = idx & 15;
            sram_idx = sram_idx & 255;
            for (int i=0; i<4; i++) {
                if ((!is_auto) || (bst.TupleData_[i].key != -1)) {
                    pimBlocks[pb].sram[idx][sram_idx].key = bst.TupleData_[i].key;
                    pimBlocks[pb].sram[idx][sram_idx].value = bst.TupleData_[i].value;
                    sram_idx++;
                    sram_idx = sram_idx & 255;
                    if (is_auto) {
                        if ((sram_idx & 128) == 0) pimBlocks[pb].idx_flag[idx] = true;
                        pimBlocks[pb].sram_idx[idx] = sram_idx;
                    }
                }

            }
            return;
        case PIMOpdType::BANK:
            *(packet->data) = bst;
            rank->banks[pb].write(packet);
            return;
    }
}

static int i=0;
void PIMRank::doPIM(BusPacket* packet)
{
    PIMCmd cCmd;
    do
    {
        cCmd.fromInt(crf.data[pimPC_]);
        if (DEBUG_CMD_TRACE)
        {
            PRINTC(CYAN, string((packet->busPacketType == READ) ? "READ ch" : "WRITE ch")
                             << getChanId() << " ra" << getRankId() << " bg"
                             << config.addrMapping.bankgroupId(packet->bank) << " b" << packet->bank
                             << " r" << packet->row << " c" << packet->column << "|| [" << pimPC_
                             << "] " << cCmd.toStr() << " @ " << currentClockCycle);
        }

        if (cCmd.type_ == PIMCmdType::EXIT)
        {
            // std::cout << "####### EXIT"<< std::endl;
            if (i==0) {
                // std::cout << "EXIT, i is "<< i << std::endl;
                std::cout << "####### EXIT, i is "<< i << "currentClockCycle is "<< currentClockCycle << std::endl;
                // std::cout << "currentClockCycle is "<< currentClockCycle << std::endl;
                i++;
            }
            // std::cout << "EXIT, i is "<< i << std::endl;

            // std::cout << "currentClockCycle is "<< currentClockCycle << std::endl;
            // i++;

            crfExit_ = true;
            break;
        }
        else if (cCmd.type_ == PIMCmdType::JUMP)
        {
            
            // std::cout << "JUMP, i is "<< i << ", currentClockCycle is " << currentClockCycle << ", numJumpToBeTaken_ is" << numJumpToBeTaken_ << std::endl;
            // i++;
            if (lastJumpIdx_ != pimPC_)
            {
                if (cCmd.loopCounter_ > 0)
                {
                    lastJumpIdx_ = pimPC_;
                    numJumpToBeTaken_ = cCmd.loopCounter_;
                }
            }
            if (numJumpToBeTaken_-- > 0)
            {
                pimPC_ -= cCmd.loopOffset_;
                // numJumpToBeTaken_--;
                PIMCmd jumped_cmd;
                jumped_cmd.fromInt(crf.data[pimPC_ + 1]);
            }
        }
        else
        {
            if (cCmd.type_ == PIMCmdType::FILL || cCmd.isAuto_)
            {
                if (lastRepeatIdx_ != pimPC_)
                {
                    lastRepeatIdx_ = pimPC_;

                    
                    if (cCmd.type_ == PIMCmdType::PART) numRepeatToBeDone_ = 32 - 1;
                    else if (cCmd.type_ == PIMCmdType::STB) {
                        if (cCmd.round_ == 0) numRepeatToBeDone_ = 16*16*32 - 1;
                        else if (cCmd.round_ == 1) numRepeatToBeDone_ = 32 - 1;
                        else if (cCmd.round_ == 2) numRepeatToBeDone_ = 32 - 1;
                    }
                    else if (cCmd.type_ == PIMCmdType::STS) numRepeatToBeDone_ = 16*32 - 1;
                    else numRepeatToBeDone_ = 8 - 1;

                }

                if (numRepeatToBeDone_-- > 0)
                {
                    pimPC_ -= 1;
                    // numRepeatToBeDone_--;
                }
                else
                    lastRepeatIdx_ = -1;
            }
            else if (cCmd.type_ == PIMCmdType::NOP)
            {
                // std::cout << "NOP, i is "<< i << std::endl;
                // std::cout << "packet->row is " << packet->row << std::endl;
                if (lastRepeatIdx_ != pimPC_)
                {
                    lastRepeatIdx_ = pimPC_;
                    numRepeatToBeDone_ = cCmd.loopCounter_;
                }

                if (numRepeatToBeDone_-- > 0)
                {
                    pimPC_ -= 1;
                    // numRepeatToBeDone_--;
                }
                else
                    lastRepeatIdx_ = -1;
            }

            for (int pimblock_id = 0; pimblock_id < config.NUM_PIM_BLOCKS; pimblock_id++)
            {
                //////////////////////////////////////////////////////////////////
                if (pimblock_id < 8) {
                    doPIMBlock(packet, cCmd, pimblock_id);
                }
                else if ((cCmd.type_ == PIMCmdType::PART || cCmd.type_ == PIMCmdType::STB || cCmd.type_ == PIMCmdType::STS || cCmd.type_ == PIMCmdType::HASH) && (config.NUM_PIM_BLOCKS == 16)) {
                    doPIMBlock(packet, cCmd, pimblock_id);
                }
                //////////////////////////////////////////////////////////////////


                if (DEBUG_PIM_BLOCK && pimblock_id == 0)
                {
                    PRINT("[BANK_R]" << packet->data->fp16ToStr());
                    PRINT("[CMD]" << bitset<32>(cCmd.toInt()) << "(" << cCmd.toStr() << ")");
                    PRINT(pimBlocks[pimblock_id].print());
                    PRINT("----------");
                }


            }
        }
        pimPC_++;
        // EXIT check
        PIMCmd next_cmd;
        next_cmd.fromInt(crf.data[pimPC_]);

        // std::cout << "next pimPC_ is " << pimPC_ << ", cmd is " << next_cmd.cmdToStr(next_cmd.type_) << std::endl;

        
        if (next_cmd.type_ == PIMCmdType::EXIT)
            {
                crfExit_ = true;
            }
    } while (cCmd.type_ == PIMCmdType::JUMP);
}

// static int q=0;
// static int counter=0;

void PIMRank::doPIMBlock(BusPacket* packet, PIMCmd cCmd, int pimblock_id)
{
    
    if (cCmd.type_ == PIMCmdType::FILL || cCmd.type_ == PIMCmdType::MOV)
    {
        BurstType bst;
        bool is_auto = (cCmd.type_ == PIMCmdType::FILL) ? true : false;

        readOpd(pimblock_id, bst, cCmd.src0_, packet, cCmd.src0Idx_, is_auto, false);
        if (cCmd.isRelu_)
        {
            for (int i = 0; i < 16; i++)
                bst.u16Data_[i] = (bst.u16Data_[i] & (1 << 15)) ? 0 : bst.u16Data_[i];
        }
        writeOpd(pimblock_id, bst, cCmd.dst_, packet, cCmd.dstIdx_, is_auto, false);
    }
    else if (cCmd.type_ == PIMCmdType::ADD || cCmd.type_ == PIMCmdType::MUL)
    {
        BurstType dstBst;
        BurstType src0Bst;
        BurstType src1Bst;
        readOpd(pimblock_id, src0Bst, cCmd.src0_, packet, cCmd.src0Idx_, cCmd.isAuto_, false);
        readOpd(pimblock_id, src1Bst, cCmd.src1_, packet, cCmd.src1Idx_, cCmd.isAuto_, false);

        if (cCmd.type_ == PIMCmdType::ADD)
            // dstBst = src0Bst + src1Bst;
            pimBlocks[pimblock_id].add(dstBst, src0Bst, src1Bst);
        else if (cCmd.type_ == PIMCmdType::MUL)
            // dstBst = src0Bst * src1Bst;
            pimBlocks[pimblock_id].mul(dstBst, src0Bst, src1Bst);

        writeOpd(pimblock_id, dstBst, cCmd.dst_, packet, cCmd.dstIdx_, cCmd.isAuto_, false);
        // dstBst.fp16Data_[i] = src0Bst.fp16Data_[i] * src1Bst.fp16Data_[i] + src2Bst.fp16Data_[i];
    }
    else if (cCmd.type_ == PIMCmdType::MAC || cCmd.type_ == PIMCmdType::MAD)
    {
        BurstType dstBst;
        BurstType src0Bst;
        BurstType src1Bst;
        bool is_mac = (cCmd.type_ == PIMCmdType::MAC) ? true : false;

        readOpd(pimblock_id, src0Bst, cCmd.src0_, packet, cCmd.src0Idx_, cCmd.isAuto_, is_mac);
        readOpd(pimblock_id, src1Bst, cCmd.src1_, packet, cCmd.src1Idx_, cCmd.isAuto_, is_mac);
        if (is_mac)
        {
            readOpd(pimblock_id, dstBst, cCmd.dst_, packet, cCmd.dstIdx_, cCmd.isAuto_, is_mac);
            // dstBst = src0Bst * src1Bst + dstBst;
            pimBlocks[pimblock_id].mac(dstBst, src0Bst, src1Bst);
        }
        else
        {
            BurstType src2Bst;
            readOpd(pimblock_id, src2Bst, cCmd.src2_, packet, cCmd.src1Idx_, cCmd.isAuto_, is_mac);
            // dstBst = src0Bst * src1Bst + src2Bst;
            pimBlocks[pimblock_id].mad(dstBst, src0Bst, src1Bst, src2Bst);
        }

        writeOpd(pimblock_id, dstBst, cCmd.dst_, packet, cCmd.dstIdx_, cCmd.isAuto_, is_mac);
    }
    else if (cCmd.type_ == PIMCmdType::NOP && packet->busPacketType == WRITE)
    {
        int grf_id = getGrfIdx(packet->column);
        if (packet->bank == 0)
        {
            *(packet->data) = pimBlocks[pimblock_id].grfA[grf_id];
            rank->banks[pimblock_id * 2].write(packet);  // basically read from bank;
        }
        else if (packet->bank == 1)
        {
            *(packet->data) = pimBlocks[pimblock_id].grfB[grf_id];
            rank->banks[pimblock_id * 2 + 1].write(packet);  // basically read from bank.
        }
    }
    else if (cCmd.type_ == PIMCmdType::PART)
    {

        // if (pimblock_id == 0) {
        //     std::cout << "PART, row : " << packet->row << ", col : " << packet->column << ", round : " << cCmd.round_ << std::endl;
        // }


        BurstType src0Bst;

        readOpd(pimblock_id, src0Bst, cCmd.src0_, packet, cCmd.src0Idx_, cCmd.isAuto_, false);

        int partition_num[4];
        
        int partition_round = cCmd.round_;
        for (int i=0; i<4; i++) {
            partition_num[i] = ((src0Bst.TupleData_[i].key >> (partition_round * 4)) & 15);
        }

        // first partition start
        if ((partition_round == 0) && ((packet->row & 4095) == 0) && ((packet->column) == 0)) {
            pimBlocks[pimblock_id].row_count[pimblock_id] = packet->row + 8192;
            pimBlocks[pimblock_id].col_count = 0;
        }
        // second partition start
        if ((partition_round == 1) && ((packet->row & 4095) == 0) && ((packet->column) == 0)) {
            ;
        }
        // third partition start
        if ((partition_round == 2) && ((packet->row & 255) == 0) && ((packet->column) == 0)) {
            ;
        }

        for (int i=0; i<4; i++) {
            int dest_idx = pimBlocks[pimblock_id].sram_idx[partition_num[i]];
            if (dest_idx == 127 || dest_idx == 255) {
                pimBlocks[pimblock_id].idx_flag[partition_num[i]] = true;
            }
            // ++pimBlocks[pimblock_id].sram_idx[partition_num[i]];

            pimBlocks[pimblock_id].sram_idx[partition_num[i]] = (pimBlocks[pimblock_id].sram_idx[partition_num[i]] + 1) & 255;
            
            pimBlocks[pimblock_id].sram[partition_num[i]][dest_idx] = src0Bst.TupleData_[i];
        }
    }
    else if (cCmd.type_ == PIMCmdType::STB)
    {
        int round = cCmd.round_;
        int dst_bank;   // 4bits
        int src_bank;   // 4bits
        int dst_col;    // 5bits
        int counter;
        int dst_row;
        int src_idx0;   // 4bits
        int src_idx1;    // 8bits
        int write_flag = 0;

        if (cCmd.isAuto_) {
            if (round == 0) {
                // for first partition's first STB
                // sram to another bank. share global I/O. only one bank works. 
                counter = (16*16*32 - 1) - (numRepeatToBeDone_ + 1);
                dst_col = counter & 31;
                dst_bank = (counter >> 5) & 15;
                src_bank = (counter >> 9) & 15;
                dst_row = pimBlocks[dst_bank].row_count[dst_bank];
                src_idx0 = dst_bank;
                // if (pimBlocks[src_bank].sram_idx[dst_bank] < 128) src_idx1 += 128;
                src_idx1 = (pimBlocks[src_bank].sram_idx[dst_bank] < 128) ? (dst_col * 4 + 128) : (dst_col * 4);
                write_flag = ((pimBlocks[src_bank].idx_flag[dst_bank] == true) && (pimblock_id == dst_bank));
            }
            else if (round == 1) {
                // for first partition's second STB
                // all bank work in parallel. local I/O
                counter = (16*32 - 1) - (numRepeatToBeDone_ + 1);
                dst_bank = pimblock_id;
                dst_col = counter;
                src_bank = pimblock_id;
                dst_row = pimBlocks[dst_bank].row_count[dst_bank];
                src_idx0 = dst_bank;
                // if (pimBlocks[src_bank].sram_idx[dst_bank] < 128) src_idx1 += 128;
                src_idx1 = (pimBlocks[src_bank].sram_idx[dst_bank] < 128) ? (dst_col * 4 + 128) : (dst_col * 4);
                write_flag = ((pimBlocks[src_bank].idx_flag[dst_bank] == true));
            }
            else if (round == 2) {
                // for first partition's last STB
                // all bank work in parallel. local I/O. handle the remains.
                counter = (32 - 1) - (numRepeatToBeDone_ + 1);
                dst_bank = pimblock_id;
                dst_col = counter;
                src_bank = pimblock_id;
                dst_row = pimBlocks[dst_bank].row_count[dst_bank];
                src_idx0 = dst_bank;
                // if (pimBlocks[src_bank].sram_idx[dst_bank] >= 128) src_idx1 += 128;
                src_idx1 = (pimBlocks[src_bank].sram_idx[dst_bank] < 128) ? (dst_col * 4) : (dst_col * 4 + 128);
                write_flag = 1;
            }
        }
        else {
            // normal case
            counter = 0;
            dst_bank = pimblock_id;
            dst_col = packet->column;
            src_bank = cCmd.src0Idx_;
            dst_row = packet->row;
            src_idx0 = cCmd.src0Idx_;
            src_idx1 = cCmd.src1Idx_;
            write_flag = 1;
        }

        if ((cCmd.isAuto_) && (counter == 0)) {
            int y=0;
            for (int i=0; i<16; i++) {
                if (pimBlocks[pimblock_id].idx_flag[i] == true) {
                    if (y==0) {
                        // std::cout << "pimblock_id : " << pimblock_id << std::endl;
                        y=1;
                    }
                    // // std::cout << "(" << i << ", " << static_cast<int>(pimBlocks[pimblock_id].sram_idx[i]) << "), ";
                    // std::cout << "(" << i << ", " << pimBlocks[pimblock_id].sram_idx[i] << "), ";
                }
            }
            if (y==1) {
                // std::cout << std::endl;
            }
        }

        if (write_flag){
            BurstType bst;
            readOpd(src_bank, bst, cCmd.src0_, packet, src_idx0, cCmd.isAuto_, false, src_idx1);
            packet->row = dst_row;
            packet->column = dst_col;
            packet->bank = dst_bank;
            writeOpd(dst_bank, bst, cCmd.dst_, packet, cCmd.dstIdx_, cCmd.isAuto_, false);
            if ((pimBlocks[src_bank].idx_flag[dst_bank] == true) && (dst_col == 31)) {
                pimBlocks[src_bank].idx_flag[dst_bank] = false;
                ++pimBlocks[dst_bank].row_count[dst_bank];
            }
        }
    }
    else if (cCmd.type_ == PIMCmdType::STS)
    {
        int counter;
        int src_col;    // 5bits
        int dst_bank;   // 4bits
        int dst_idx0;
        int dst_idx1;
        int src_bank;   // 4bits
        int src_idx0;
        int src_idx1;
        int write_flag = 0;


        if (cCmd.isAuto_) {
            // for first partition
            counter = (16*32 - 1) - (numRepeatToBeDone_ + 1);
            src_col = counter & 31;
            dst_bank = (counter >> 5) & 15;
            dst_idx0 = dst_bank;
            dst_idx1 = pimBlocks[dst_bank].sram_idx[dst_bank];
            src_bank = (15 - numJumpToBeTaken_) & 15;
            src_idx0 = dst_bank;
            src_idx1 = (pimBlocks[src_bank].sram_idx[dst_bank] < 128) ? (src_col * 4) : (src_col * 4 + 128);
            write_flag = ((dst_bank == pimblock_id) && (dst_bank != src_bank));
        }
        else {
            // normal case
            // TODO
            counter = 0;
            src_col = 0;
            dst_bank = 0;
            dst_idx0 = 0;
            dst_idx1 = 0;
            src_bank = 0;
            src_idx0 = 0;
            src_idx1 = 0;
            write_flag = 1;
        }

        if (write_flag) {

            BurstType bst;
            readOpd(src_bank, bst, cCmd.src0_, packet, src_idx0, cCmd.isAuto_, false, src_idx1);
            writeOpd(dst_bank, bst, cCmd.dst_, packet, dst_idx0, cCmd.isAuto_, false, dst_idx1);
            if (cCmd.isAuto_ && (src_col == 31)) pimBlocks[src_bank].sram_idx[src_idx0] = 0;
        }

    }
}
