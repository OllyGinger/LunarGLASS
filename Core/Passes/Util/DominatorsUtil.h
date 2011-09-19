//===- DominatorsUtil.h - Utilities for (post)dominator trees/frontiers ---===//
//
// LunarGLASS: An Open Modular Shader Compiler Architecture
// Copyright (C) 2010-2011 LunarG, Inc.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; version 2 of the
// License.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
// 02110-1301, USA.
//
//===----------------------------------------------------------------------===//
//
// Author: Michael Ilseman, LunarG
//
// Provides utility functions for (post)dominator trees and frontiers
//
//===----------------------------------------------------------------------===//

#ifndef GLA_DOMINATORSUTIL_H
#define GLA_DOMINATORSUTIL_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/Analysis/Dominators.h"
#include "llvm/Analysis/PostDominators.h"

#include "Passes/Util/BasicBlockUtil.h"

namespace gla_llvm {
    using namespace llvm;

    // Get all the (non-terminator) (post)dominated instructions, and put them into
    // result.
    // Complexity: Linear in the number of instructions in the dominator subtree
    void GetAllDominatedInstructions(Instruction* inst, DominatorTreeBase<BasicBlock>& dt, std::vector<Instruction*>& result)
    {
        BasicBlock* origBlock = inst->getParent();

        SmallVector<BasicBlock*, 32> domBlocks;
        GetDominatedChildren(dt.getNode(origBlock), origBlock, domBlocks);

        // Add in the non-terminator instructions (post)dominated by the given instruction
        BasicBlock::InstListType& instList = origBlock->getInstList();

        // Start from the beginning for post-dominators, start at the end for
        // dominators. I don't know of a way to type-unify iplist's iterator
        // with reverse_iterator, hence the separate loops.
        if (dt.isPostDominator()) {
            for (BasicBlock::iterator i = instList.begin(); &*i != inst; ++i) {
                if (i->isTerminator())
                    continue;

                result.push_back(i);
            }
        } else {
            for (iplist<Instruction>::reverse_iterator i = instList.rbegin(); &*i != inst; ++i) {
                if (i->isTerminator())
                    continue;

                result.push_back(&*i);
            }
        }

        // Add in all the non-terminator instructions of all the (post)
        // dominated blocks.
        for (SmallVector<BasicBlock*,32>::iterator bbI = domBlocks.begin(), bbE = domBlocks.end(); bbI != bbE; ++bbI) {
            // Skip the original block
            if (*bbI == origBlock)
                continue;

            for (BasicBlock::iterator instI = (*bbI)->begin(), instE = (*bbI)->end(); instI != instE; ++instI) {
                if (instI->isTerminator())
                    continue;

                result.push_back(instI);
            }
        }
    }

} // end namespace gla_llvm

#endif /* GLA_DOMINATORSUTIL_H */
