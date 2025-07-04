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

#ifndef __PIM_KERNEL_GEN_H__
#define __PIM_KERNEL_GEN_H__

#include <vector>

#include "MultiChannelMemorySystem.h"
#include "PIMCmd.h"
#include "SystemConfiguration.h"
#include "tests/KernelAddrGen.h"

using namespace std;
using namespace DRAMSim;

// static int l=0;

class IPIMCmd
{
  public:
    IPIMCmd(KernelType ktype) : kernelType(ktype) {}
    virtual vector<PIMCmd> generateKernel(int num_jump_to_be_taken,
                                          int num_jump_to_be_taken_odd_bank,
                                          int num_jump_to_be_taken_even_bank,
                                          int is_join_bench) = 0;

  protected:
    KernelType kernelType;
};

class EltwisePIMKernel : public IPIMCmd
{
  public:
    EltwisePIMKernel(KernelType ktype) : IPIMCmd(ktype) {}
    virtual vector<PIMCmd> generateKernel(int num_jump_to_be_taken,
                                          int num_jump_to_be_taken_odd_bank = 0,
                                          int num_jump_to_be_taken_even_bank = 0,
                                          int is_join_bench = 0) override
    {
        vector<PIMCmd> pim_cmds;
        PIMCmdType pimType = getPIMCmdType();
        vector<PIMCmd> tmp_cmds{
            PIMCmd(PIMCmdType::FILL, PIMOpdType::GRF_A, PIMOpdType::EVEN_BANK),
            PIMCmd(pimType, PIMOpdType::GRF_A, PIMOpdType::GRF_A, PIMOpdType::EVEN_BANK, 1),
            PIMCmd(PIMCmdType::NOP, 7),
            PIMCmd(PIMCmdType::FILL, PIMOpdType::GRF_B, PIMOpdType::ODD_BANK),
            PIMCmd(pimType, PIMOpdType::GRF_B, PIMOpdType::GRF_B, PIMOpdType::ODD_BANK, 1),
            PIMCmd(PIMCmdType::NOP, 7)};
        pim_cmds.assign(tmp_cmds.begin(), tmp_cmds.end());
        if (num_jump_to_be_taken != 0)
        {
            pim_cmds.push_back(PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken, pim_cmds.size() + 1));
        }
        pim_cmds.push_back(PIMCmd(PIMCmdType::EXIT, 0));
        // std::cout << "l is " << l++ << std::endl;
        return pim_cmds;
    }

  private:
    PIMCmdType getPIMCmdType()
    {
        if (kernelType == KernelType::ADD)
            return PIMCmdType::ADD;
        else if (kernelType == KernelType::MUL)
            return PIMCmdType::MUL;
        else
            throw invalid_argument("Not supported element-wise operation");
    }
};

class JoinPIMKernel : public IPIMCmd
{
  public:
  JoinPIMKernel(KernelType ktype) : IPIMCmd(ktype) {}
    virtual vector<PIMCmd> generateKernel(int num_jump_to_be_taken,
                                          int num_jump_to_be_taken_odd_bank = 0,
                                          int num_jump_to_be_taken_even_bank = 0,
                                          int is_join_bench = 0) override
    {
        if (is_join_bench == 0) {
            vector<PIMCmd> pim_cmds{
                //first partition
                PIMCmd(1, PIMCmdType::PART, PIMOpdType::SRAM, PIMOpdType::BANK, 0),
                PIMCmd(1, PIMCmdType::STB, PIMOpdType::BANK, PIMOpdType::SRAM, 0),
                PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken, 2 + 1),

                PIMCmd(1, PIMCmdType::STS, PIMOpdType::BANK, PIMOpdType::SRAM),
                PIMCmd(1, PIMCmdType::STB, PIMOpdType::BANK, PIMOpdType::SRAM, 1),
                PIMCmd(PIMCmdType::JUMP, 15, 2 + 1),

                PIMCmd(1, PIMCmdType::STB, PIMOpdType::BANK, PIMOpdType::SRAM, 2),

                PIMCmd(PIMCmdType::JUMP, 1, 7 + 1),
                

                // // second partition
                // PIMCmd(1, PIMCmdType::PART, PIMOpdType::SRAM, PIMOpdType::BANK, 1),
                // PIMCmd(1, PIMCmdType::STB, PIMOpdType::BANK, PIMOpdType::SRAM, 3),
                // PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken, 2 + 1),

                // PIMCmd(1, PIMCmdType::STB, PIMOpdType::BANK, PIMOpdType::SRAM, 4),
                // PIMCmd(PIMCmdType::JUMP, 1, 4 + 1),


                // // third partition
                // PIMCmd(1, PIMCmdType::PART, PIMOpdType::SRAM, PIMOpdType::BANK, 1),
                // PIMCmd(1, PIMCmdType::STB, PIMOpdType::BANK, PIMOpdType::SRAM, 3),
                // PIMCmd(PIMCmdType::JUMP, (num_jump_to_be_taken + 1) / 16 - 1, 2 + 1),

                // PIMCmd(1, PIMCmdType::STB, PIMOpdType::BANK, PIMOpdType::SRAM, 4),
                // PIMCmd(PIMCmdType::JUMP, 16, 4 + 1),
                // PIMCmd(PIMCmdType::JUMP, 1, 5 + 1),


                // // joining
                // // PIMCmd(1, PIMCmdType::HASH, PIMOpdType::SRAM, PIMOpdType::BANK, 1),
                // // PIMCmd(1, PIMCmdType::HASH, PIMOpdType::SRAM, PIMOpdType::BANK, 1),
                // PIMCmd(1, PIMCmdType::STB, PIMOpdType::BANK, PIMOpdType::SRAM, 5),
                // PIMCmd(PIMCmdType::JUMP, 16 * 16 - 1, 3 + 1),



                PIMCmd(PIMCmdType::EXIT, 0)
            };

            // tmp_cmds.push_back(PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken, 2 + 1));
            // pim_cmds.assign(tmp_cmds.begin(), tmp_cmds.end());

            // pim_cmds.push_back(PIMCmd(PIMCmdType::EXIT, 0));
            return pim_cmds;
        }
        else {
            vector<PIMCmd> pim_cmds{
                //first partition
                PIMCmd(PIMCmdType::NOP, 32),
                PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken, 1 + 1),
                PIMCmd(PIMCmdType::EXIT, 0)
            };
            return pim_cmds;
        }

    }

};

class ActPIMKernel : public IPIMCmd
{
  public:
    ActPIMKernel(KernelType ktype) : IPIMCmd(ktype) {}
    virtual vector<PIMCmd> generateKernel(int num_jump_to_be_taken,
                                          int num_jump_to_be_taken_odd_bank = 0,
                                          int num_jump_to_be_taken_even_bank = 0,
                                          int is_join_bench = 0) override
    {
        vector<PIMCmd> pim_cmds;
        if (kernelType == KernelType::RELU)
        {
            vector<PIMCmd> tmp_cmds{
                PIMCmd(PIMCmdType::FILL, PIMOpdType::GRF_A, PIMOpdType::EVEN_BANK, 1, 0, 0, 0, 1),
                PIMCmd(PIMCmdType::NOP, 7),
                PIMCmd(PIMCmdType::FILL, PIMOpdType::GRF_B, PIMOpdType::ODD_BANK, 1, 0, 0, 0, 1),
                PIMCmd(PIMCmdType::NOP, 7)};
            pim_cmds.assign(tmp_cmds.begin(), tmp_cmds.end());
        }
        else
        {
            throw invalid_argument("Not supported activation");
        }
        if (num_jump_to_be_taken != 0)
        {
            pim_cmds.push_back(PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken, pim_cmds.size() + 1));
        }
        pim_cmds.push_back(PIMCmd(PIMCmdType::EXIT, 0));
        return pim_cmds;
    }
};

class GemvPIMKernel : public IPIMCmd
{
  public:
    GemvPIMKernel(KernelType ktype) : IPIMCmd(ktype) {}
    virtual vector<PIMCmd> generateKernel(int num_jump_to_be_taken,
                                          int num_jump_to_be_taken_odd_bank,
                                          int num_jump_to_be_taken_even_bank,
                                          int is_join_bench = 0) override
    {
        vector<PIMCmd> pim_cmds;
        if (kernelType == KernelType::GEMV)
        {
            vector<PIMCmd> tmp_cmds{
                PIMCmd(PIMCmdType::MAC, PIMOpdType::GRF_B, PIMOpdType::GRF_A, PIMOpdType::EVEN_BANK,
                       1, 0, 0, 0),
                PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken_even_bank, 2),
                PIMCmd(PIMCmdType::MAC, PIMOpdType::GRF_B, PIMOpdType::GRF_A, PIMOpdType::ODD_BANK,
                       1, 0, 0, 0),
                PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken_odd_bank, 2),
                PIMCmd(PIMCmdType::NOP, 7),
            };
            pim_cmds.assign(tmp_cmds.begin(), tmp_cmds.end());
        }
        else if (kernelType == KernelType::GEMVTREE)
        {
            vector<PIMCmd> tmp_cmds{
                PIMCmd(PIMCmdType::MAC, PIMOpdType::GRF_B, PIMOpdType::GRF_A, PIMOpdType::EVEN_BANK,
                       1, 0, 0, 0),
                // FIXME: hard coding
                PIMCmd(PIMCmdType::JUMP, 7, 2), PIMCmd(PIMCmdType::NOP, 7),
                PIMCmd(PIMCmdType::MUL, PIMOpdType::GRF_B, PIMOpdType::GRF_B, PIMOpdType::EVEN_BANK,
                       1),
                PIMCmd(PIMCmdType::MAC, PIMOpdType::GRF_B, PIMOpdType::GRF_A, PIMOpdType::ODD_BANK,
                       1, 0, 0, 0),
                PIMCmd(PIMCmdType::JUMP, 7, 2), PIMCmd(PIMCmdType::NOP, 7),
                PIMCmd(PIMCmdType::MUL, PIMOpdType::GRF_B, PIMOpdType::GRF_B, PIMOpdType::EVEN_BANK,
                       1),
                // PIMCmd(PIMCmdType::JUMP, num_jump, 7), /*it used that tile is 2*/
            };
            pim_cmds.assign(tmp_cmds.begin(), tmp_cmds.end());
        }
        else
        {
            throw invalid_argument("Not supported gemv operation");
        }
        if (num_jump_to_be_taken != 0)
        {
            pim_cmds.push_back(PIMCmd(PIMCmdType::JUMP, num_jump_to_be_taken, pim_cmds.size() + 1));
        }
        pim_cmds.push_back(PIMCmd(PIMCmdType::EXIT, 0));
        return pim_cmds;
    }
};


class PIMCmdGen
{
  public:
    static vector<PIMCmd> getPIMCmds(KernelType ktype, int num_jump_to_be_taken,
                                     int num_jump_to_be_taken_odd_bank,
                                     int num_jump_to_be_taken_even_bank,
                                     int is_join_bench = 0);
};

#endif  // __PIM_KERNEL_GEN_H__
