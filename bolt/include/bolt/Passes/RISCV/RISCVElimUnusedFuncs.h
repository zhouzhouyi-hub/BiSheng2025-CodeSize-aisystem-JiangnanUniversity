//
// Created by yz on 25-4-3.
//

#ifndef RISCVELIMUNSEDFUNCS
#define RISCVELIMUNSEDFUNCS

#include "bolt/Passes/BinaryPasses.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/MC/MCInstrDesc.h"

namespace llvm {
namespace bolt {

class RISCVElimUnusedFuncs : public BinaryFunctionPass {
    void runOnFunction(BinaryFunction &BF);

public:
    explicit RISCVElimUnusedFuncs(const cl::opt<bool> &PrintPass)
        : BinaryFunctionPass(PrintPass) {}
    const char *getName() const override { return "riscv-elim-unused-funcs"; }

    void runOnFunctions(BinaryContext &BC) override;
};

} // namespace bolt
} // namespace llvm

#endif //RISCVELIMUNSEDFUNCS
