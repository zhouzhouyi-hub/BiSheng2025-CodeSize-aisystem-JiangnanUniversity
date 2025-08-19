#include "MCTargetDesc/RISCVMCExpr.h"
#include "MCTargetDesc/RISCVMCTargetDesc.h"
#include "bolt/Passes/RISCV/RISCVMemorySSA.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include <map>
#include <list>
#include <deque>

using namespace llvm;

namespace llvm {
namespace bolt {

static bool isSW(const MCInst &I) { return I.getOpcode() == RISCV::SW; }
static bool isLW(const MCInst &I) { return I.getOpcode() == RISCV::LW; }

static bool isSH(const MCInst &I) { return I.getOpcode() == RISCV::SH; }
static bool isLH(const MCInst &I) { return I.getOpcode() == RISCV::LH; }


// 用来存储 SW/LW 信息的结构体（增加了 BBPtr，用于跨基本块删除）
struct MemAccess {
  MCPhysReg DstReg;           // SW 中存入的寄存器
  int64_t Imm;                // 偏移量
  MCPhysReg BaseReg;          // 基址寄存器
  MCInst* InstPtr;            // 指向该指令的指针
  BinaryBasicBlock* BBPtr;    // 该指令所在的基本块
  bool isLoad;                // true if LW, false if SW
};

void RISCVMemorySSA::runOnFunction(BinaryFunction &BF) {
  // 处理 SW/LW 对
  processMemChain(BF, isSW, isLW, RISCV::C_MV);
  // 处理 SH/LH 对
  processMemChain(BF, isSH, isLH, RISCV::C_MV); // 或者用专门的 C_MV_H 针对半字指令（如果存在）
}


template<typename IsStoreFn, typename IsLoadFn>
void RISCVMemorySSA::processMemChain(BinaryFunction &BF,
                     IsStoreFn isStore, IsLoadFn isLoad,
                     unsigned MVOpcode) {
  auto &BC = BF.getBinaryContext();
  std::deque<MemAccess> memChain;  // 存储 SW/SH 指令链

  // 遍历函数中的所有基本块
  for (auto &BB : BF) {
    for (auto II = BB.begin(); II != BB.end(); ) {
      if (isStore(*II)) {
        // 获取存储指令参数——可以通过传参或约定：operand0为源寄存器，operand1为基址，operand2为立即数
        MCPhysReg SrcReg = II->getOperand(0).getReg();
        if (!II->getOperand(2).isImm()) {
          memChain.clear();
          ++II;
          errs() << "非存储异常指令!\n";
          continue;
        }
        int64_t Imm = II->getOperand(2).getImm();
        MCPhysReg BaseReg = II->getOperand(1).getReg();
        dbgs() << "Processing STORE: ";

        // 存入链，同时记录所在基本块
        memChain.push_back(MemAccess{SrcReg, Imm, BaseReg, &(*II), &BB, false});
        ++II;
      }
      else if (isLoad(*II)) {
        // 获取加载指令参数——假设 operand0 为目标寄存器，operand1为基址，operand2为立即数
        MCPhysReg DstReg = II->getOperand(0).getReg();
        if (!II->getOperand(2).isImm()) {
          memChain.clear();
          ++II;
          errs() << "非加载异常指令!\n";
          continue;
        }
        int64_t Imm = II->getOperand(2).getImm();
        MCPhysReg BaseReg = II->getOperand(1).getReg();
        dbgs() << "Processing LOAD: ";

        if (memChain.empty()) {
          dbgs() << "But Chain Empty!\n";
          ++II;
          continue;
        }

        bool matched = false;
        for (auto it = memChain.begin(); it != memChain.end(); ++it) {
          if (!it->isLoad &&
              it->Imm == Imm &&
              it->BaseReg == BaseReg) {

            dbgs() << "Found store/load pair on same mem location\n";
            dbgs() << "Store info: ";

            if (it->DstReg != DstReg) {
              // 目标不匹配时，用 C_MV 替换加载指令
              dbgs() << "Target register mismatch: replacing load with C_MV\n";
              MCInst NewMv;
              NewMv.setOpcode(MVOpcode); // MVOpcode 传入对应的 C_MV（例如 RISCV::C_MV）
              NewMv.addOperand(MCOperand::createReg(DstReg));  // 目标
              NewMv.addOperand(MCOperand::createReg(it->DstReg)); // 源
              BB.replaceInstruction(II, {NewMv});
              ++II;
            } else {
              // 如果匹配则直接删除加载指令
              II = BB.eraseInstruction(II);
            }
            // 删除对应的存储指令
            auto SWIter = std::find_if(it->BBPtr->begin(), it->BBPtr->end(),
              [inst = it->InstPtr](const MCInst &I) { return &I == inst; });
            if (SWIter != it->BBPtr->end())
              it->BBPtr->eraseInstruction(SWIter);

            memChain.erase(it);
            matched = true;
            break;
          }
          else {
            dbgs() << "Operator Not Match for current entry.\n";
          }
        }
        if (!matched) {
          dbgs() << "No matching store for load; clearing chain.\n";
          memChain.clear();
          ++II;
        }
      }
      else {
        dbgs() << "Non-store/load instruction encountered, clearing chain.\n";
        memChain.clear();
        ++II;
      }
    }
  }
}



void RISCVMemorySSA::runOnFunctions(BinaryContext &BC) {
  for (auto &BFIt : BC.getBinaryFunctions()) {
    runOnFunction(BFIt.second);
  }
}


} // namespace bolt
} // namespace llvm
