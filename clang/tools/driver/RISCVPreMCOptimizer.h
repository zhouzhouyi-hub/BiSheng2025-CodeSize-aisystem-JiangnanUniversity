//===-- RISCVPreMCOptimizer.h - RISC-V Pre-Machine Code Optimizer ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the definition of the RISCVPreMCOptimizer class, which is
// responsible for performing pre-machine code optimizations for RISC-V. 
//
//===----------------------------------------------------------------------===//

#ifndef RISCV_PRE_MC_OPTIMIZER_H
#define RISCV_PRE_MC_OPTIMIZER_H

#include "clang/Frontend/Utils.h"
#include "llvm/ADT/StringExtras.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>
#include <random>
#include <ctime>

using namespace clang;
using namespace llvm;

namespace RISCVOptimizer {

/**
 * @brief Type alias for a vector of strings with a reserved capacity of 32.
 */
using StringVec = llvm::SmallVector<std::string, 32>;

/**
 * @brief Type alias for a vector of StringRefs with a reserved capacity of 32.
 */
using StringRefVec = llvm::SmallVector<llvm::StringRef, 32>;

/**
 * @brief Type alias for a vector of size_t with a reserved capacity of 32.
 */
using SizeVec = llvm::SmallVector<size_t, 32>;

/**
 * @brief Type alias for a unique pointer to a MemoryBuffer.
 */
using MemoryBufferPtr = std::unique_ptr<llvm::MemoryBuffer>;

/**
 * @brief Structure representing a symbol with its usage information and location.
 */
struct sdataSymbol {
    std::string symName; // The name of the symbol.
    int symUseTimes;     // The number of times the symbol is used.
    size_t startLine;    // The starting line of the symbol.
    size_t endLine;      // The ending line of the symbol.
};

/**
 * @brief Structure representing function information.
 */
struct FunctionInfo {
    std::string name;              // The name of the function.
    size_t startLine;              // The starting line of the function.
    size_t endLine;                // The ending line of the function.
    bool hasStackOperations;       // Indicates if the function has stack operations.
    std::vector<std::string> callees; // List of functions called by this function.
};

/**
 * @brief Class for performing pre-machine code optimizations on RISC-V assembly.
 */
class RISCVPreMCOptimizer final{

public:
    StringVec RV32Insns;
    // 存储符号映射表
    std::unordered_map<std::string, std::string> symbol_map;

    /**
     * @brief Constructs an optimizer object and initializes it with the input file and buffer.
     * @param inputFile The name of the input file.
     * @param buffer A unique pointer to a MemoryBuffer containing the file contents.
     */
    RISCVPreMCOptimizer(const std::string& inputFile, MemoryBufferPtr buffer){
        getFileName(inputFile);
        copyFile = std::move(buffer);
        llvm::StringRef Content = copyFile->getBuffer();
        Content.split(Lines, '\n');
        RV32Insns.reserve(Lines.size());
        for (const llvm::StringRef &line : Lines) {
            // Check if line starts with ".loc" or ".cfi"
            llvm::StringRef trimmedLine = line.trim();
            if (!trimmedLine.startswith(".loc") && !trimmedLine.startswith(".cfi")) {
                RV32Insns.emplace_back(line.data(), line.size());
            }
        }
    }

    void buildCompactSymbolMap(StringVec& RV32Insns);

    void applySymbolRemap(StringVec& RV32Insns);

    /**
     * @brief Performs various optimization tasks on the assembly instructions.
     */
    void performOptimization();

    /**
     * @brief Finalizes the buffer after optimizations are complete.
     * @return A unique pointer to the finalized MemoryBuffer.
     */
    MemoryBufferPtr finalizeBuffer();

    /**
     * @brief Replaces .rodata sections with .data sections.
     * @param RV32Insns Vector of RV32 instructions.
     */
    void replaceRodataWithData(StringVec& RV32Insns);


    /**
     * @brief Removes AMS sections from RV32 instructions.
     * @param RV32Insns Vector of RV32 instructions.
     */
    void removeAMSSections(StringVec& RV32Insns);

    /**
     * @brief Saves and restores the return address register (RA) around function calls.
     * @param RV32Insns Vector of RV32 instructions.
     */
    void saveAndRestoreRA(StringVec &RV32Insns);

    void undoReplacement (StringVec &RV32Insns);

    /**
     * @brief Creates a jump board for frequently called functions.
     * @param RV32Insns Vector of RV32 instructions.
     */
    void createJumpBoard(StringVec& RV32Insns);

    /**
     * @brief Inserts function sections into the RV32 instruction vector.
     * @param RV32Insns Vector of RV32 instructions.
     */
    void insertFunctionSections(StringVec& RV32Insns);

    /**
     * @brief Removes dead code from the RV32 instruction vector.
     */
    void removeDeadCode();

    /**
     * @brief Merges duplicate constants in the RV32 instruction vector.
     */
    void mergeConstants();

    /**
     * @brief Optimizes branch instructions in the RV32 instruction vector.
     */
    void optimizeBranches();

    /**
     * @brief Optimizes stack push operations in the RV32 instruction vector.
     */
    void optimizeStackPush();

    /**
     * @brief Optimizes stack pop operations in the RV32 instruction vector.
     */
    void optimizeStackPop();

    /**
     * @brief Optimizes the sequence of LUI and ADDI instructions to a single LI instruction.
     */
    void optimizeLuiAddiToLi();

    /**
     * @brief Scans functions and replaces stack operations with temporary register moves.
     */
    void replaceStackWithTp();

    /**
     * @brief Removes unused functions from the instruction list.
     */
    void removeUnusedfunc();

    void simplifyLoop(StringVec &RV32Insns);

    /**
     * @brief Writes the optimized RV32 instructions to a specified output file.
     * @param outputFile The name of the output file. Defaults to "opt.s" if not provided.
     */
    void writeToFile(const std::string &outputFile);

    /**
     * @brief Removes definitions of weak RISC-V symbols from the instruction stream.
     *        For the given fileName, looks up a predefined list of g_<num> symbols
     *        and clears the entire definition block (from `.type` to the next blank
     *        line) for each matching symbol in RV32Insns.
     *
     * @param fileName Name of the assembly file used to select the symbol list.
     * @param RV32Insns Vector of RISC-V assembly lines to process in-place.
     */
    void removeWeakSymbols(const std::string &fileName, StringVec &RV32Insns);
private:

    /**
     * @brief Extracts the file name without extension from a given file path.
     * @param filePath The full path of the file.
     */
    void getFileName(const std::string& filePath);

    /**
     * @brief Checks if a given string represents the main function definition.
     * @param str The string to be checked.
     * @return true if the string represents the main function definition, false otherwise.
     */
    bool isMainFunction(const std::string str) const;

    /**
     * @brief Checks if a given string starts with a "call" instruction.
     * @param str The string to be checked.
     * @return true if the string starts with "call" instruction, false otherwise.
     */
    bool startsWithCallInst(const std::string str) const;

    /**
     * @brief Finds the first non-whitespace character in a given string.
     * @param str The string to be checked.
     * @return The first non-whitespace character in the string. If no such character is found, returns `'\0'`.
     */
    char getFirstNonSpaceChar(const std::string &str) const;

    /**
     * @brief Extracts the function name from a given string if it starts with "call".
     * @param input The input string from which to extract the function name.
     * @return A substring representing the function name if "call" is present; otherwise, an empty string.
     */
    std::string getFunctionName(const std::string& input) const;

    /**
     * @brief Inserts a new section directive into the assembly instructions.
     * @param i The index in the `RV32Insns` vector where the new section directive should be inserted.
     * @param RV32Insns A reference to the vector of assembly instructions where the new section directive will be added.
     * @param s The name of the new section to be created.
     */
    void createNewSection(size_t i, StringVec &RV32Insns, std::string s);

    /**
     * @brief Extracts the section name from the input string.
     * @param input The input string containing the section information.
     * @return The extracted section name as a string.
     */
    std::string getSectionName(const std::string& input) const;

    /**
     * @brief Extracts the symbol name from the input string.
     * @param input The input string containing the symbol information.
     * @return The extracted symbol name as a string.
     */
    std::string getSymbolName(const std::string& input) const;

    /**
     * @brief Extracts a name based on the specified type from the input string.
     * @param input The input string containing the type information.
     * @param type The type to search for in the input string.
     * @return The extracted name as a string. Returns an empty string if the type is not found.
     */
    std::string getNameByType(const std::string& input, const std::string& type) const;

    /**
     * @brief Extracts the name from the input string within %hi() delimiters.
     * @param input The input string containing the %hi() information.
     * @return The extracted name as a string. Returns an empty string if the delimiters are not found.
     */
    std::string getSdataName(const std::string& input) const;

    /**
     * @brief Removes all instances of the string "needforcleanup" from the given vector.
     * @param RV32Insns The vector of strings to be cleaned up.
     */
    void cleanStringVec(StringVec &RV32Insns);

    /**
     * @brief Resets the symbols in the .sdata section of the given vector of strings.
     * @param RV32Insns The vector of strings representing the assembly instructions.
     */
    void resetSdataSymbols(StringVec &RV32Insns);

    /**
     * @brief Checks if the given text contains the specified substring.
     * @param text The text to be searched.
     * @param substring The substring to search for within the text.
     * @return true if the substring is found within the text, false otherwise.
     */
    bool containsSubstring(const std::string& text, const std::string& substring) const;

    /**
     * @brief Finds the position of a line within a given range that contains specific substrings.
     * @param RV32Insns A vector of strings representing assembly instructions.
     * @param i The starting index of the range to search.
     * @param n The number of lines to consider in the search.
     * @return The position of the line containing one of the specified substrings, or 0 if no match is found
     *         or the range exceeds the container size.
     */
    size_t findSubstringInRange(StringVec &RV32Insns, size_t i, size_t n) const;

    /**
     * @brief Executes the Pa pass by modifying specific instructions in RV32Insns.
     * @param n The number of lines to mark for cleanup.
     * @param RV32Insns The vector of instructions to modify.
     * @param Dot A vector of positions within RV32Insns that need modification.
     * @param i The current position in RV32Insns to start the modifications.
     */
    void executehandlePasses(size_t n, StringVec& RV32Insns, SizeVec& Dot, size_t i);

    /**
     * @brief Creates a new function and inserts it into the Func vector.
     * @param i The starting position in RV32Insns from which to copy instructions.
     * @param n The number of lines to copy from RV32Insns.
     * @param Func The vector where the new function definition will be inserted.
     * @param RV32Insns The vector of instructions to copy from.
     */
    void createNewFunction(size_t i, size_t n, StringVec &Func, StringVec &RV32Insns);

    /**
     * @brief Determines if the cost model for a call is valid.
     * @param i The starting index in RV32Insns to check.
     * @param RV32Insns The vector of instructions to check.
     * @param n The number of instructions to check from index i.
     * @return true if the cost model is valid, false otherwise.
     */
    bool isCostModelForCallValid(size_t i, StringVec &RV32Insns, size_t n) const;

    /**
     * @brief Performs the handlePasses optimization.
     * @param n The number of instructions to process in each pass.
     * @param RV32Insns The vector of RISC-V instructions to optimize.
     * @param Dot The vector of size_t positions related to Dot.
     * @param Func The vector of strings representing function instructions.
     */
    void handlePasses(size_t n, StringVec& RV32Insns, SizeVec& Dot, StringVec& Func);

    /**
     * @brief Processes the handlePasses optimization on the given instructions.
     * @param n The number of instructions to process in each pass.
     * @param RV32Insns The vector of RISC-V instructions to optimize.
     * @param Dot The vector of size_t positions related to Dot.
     * @param Func The vector of strings representing function instructions.
     * @param endLoc The end location in the instruction vector to process up to.
     */
    void processPasses(size_t n, StringVec& RV32Insns, SizeVec& Dot, StringVec& Func, size_t endLoc);

    /**
     * @brief Checks if the given range of instructions contains an error symbol.
     * @param RV32Insns The vector of RISC-V instructions to check.
     * @param i The starting index of the range to check.
     * @param n The number of instructions to check in the range.
     * @return true if an error symbol is found within the range, false otherwise.
     */
    bool hasErrorSymbol(const StringVec& RV32Insns, size_t i, size_t n) const;

    /**
     * @brief Checks if two ranges of lines in the instruction vector are equal.
     * @param RV32Insns The vector of RISC-V instructions.
     * @param i The starting index of the first range.
     * @param j The starting index of the second range.
     * @param n The number of lines to compare.
     * @return true if the two ranges of lines are equal, false otherwise.
     */
    bool areLinesEqual(const StringVec& RV32Insns, size_t i, size_t j, size_t n) const;

    /**
     * @brief Inserts function declarations into the instruction vector at the main function location.
     * @param Func The vector containing function declarations to be inserted.
     * @param RV32Insns The vector of RISC-V instructions where the function declarations will be inserted.
     */
    void insertFunctionsAtMain(StringVec& Func, StringVec& RV32Insns);

    /**
     * @brief Scans the RV32 instructions to identify function boundaries.
     */
    void scanFunctions();

    /**
     * @brief Records function callees and checks for stack operations.
     */
    void recordCalleesAndCheckStackOps();
    /**
     * @brief Replaces specific stack operations with temporary register moves.
     */
    void replaceStackWithMv();

    /**
     * @brief Compares two characters case-insensitively.
     * @param a First character.
     * @param b Second character.
     * @return True if characters are equal ignoring case, otherwise false.
     */
    static inline bool caseInsensitiveCompare(char a, char b) {
        return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b));
    }

private:
    // Map of function names to their usage times.
    std::unordered_map<std::string, int> funcUseTimes; 

    // Map of function names to their information.
    std::unordered_map<std::string, FunctionInfo> functions; 

    // Vector of sdata symbols.
    std::vector<sdataSymbol> sdataSymbols; 

    // Map of symbol names to sdata symbols.
    std::unordered_map<std::string, sdataSymbol> symbolMap; 

    // Map of fnum to code symbols.
    std::unordered_map<int, StringVec> fNumToLinesMap;

    // Random number for jNum.
    size_t jNum = rand(); 

    // Random number for fNum.
    size_t fNum = rand(); 

    // Name of the file being optimized.
    std::string fileName; 

    // Vector of size_t for Dot.
    SizeVec Dot; 

    // Vector of strings for Func.
    StringVec Func; 

    // Vector of StringRefs for lines of the file.
    StringRefVec Lines; 

    // Unique pointer to the MemoryBuffer.
    MemoryBufferPtr copyFile; 

};

#endif // RISCV_PRE_MC_OPTIMIZER_H

}
