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

#ifndef __PIM_BLOCK_HPP__
#define __PIM_BLOCK_HPP__

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#include "Burst.h"
#include "SystemConfiguration.h"

using namespace std;

namespace DRAMSim
{
class PIMBlock
{
  public:
    PIMBlock()
    {
        pimPrecision_ = PIMConfiguration::getPIMPrecision();
        // part1_dst_row_idx = 
        // std::fill(std::begin(idx_flag), std::end(idx_flag), false);
    }
    PIMBlock(const PIMPrecision& pimPrecision) : pimPrecision_(pimPrecision) {}

    BurstType srf;
    BurstType grfA[8];  // FIXME: hard coding shcha
    BurstType grfB[8];
    BurstType mOut;
    BurstType aOut;

    // //for join.
    // BurstType sram[16384];
    Tuple sram[16][256] = {0};
    bool idx_flag[16] = {0};

    int sram_idx[16] = {0};

    int row_count[16] = {0};
    int col_count = 0;


    //

    void add(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst);
    void mac(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst);
    void mul(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst);
    void mad(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst, BurstType& src2Bst);

    std::string print();

  private:
    PIMPrecision pimPrecision_;
};

}  // namespace DRAMSim
#endif
