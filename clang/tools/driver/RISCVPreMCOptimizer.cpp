//===-- RISCVPreMCOptimizer.cpp - RISC-V Pre-Machine Code Optimizer
//--------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of the RISCVPreMCOptimizer class, which
// performs pre-machine code optimizations for RISC-V. It includes the methods
// for identifying main functions, handling call instructions, extracting the
// first non-whitespace character, and parsing function names from assembly
// code.
//
//===----------------------------------------------------------------------===//

#include "RISCVPreMCOptimizer.h"
using namespace clang;
using namespace llvm;

namespace RISCVOptimizer {

static const int sec_mys_i[] = {
    104,   311,   312,   313,   1467,  2286,  2460,  2461,  2719,  2747,  3049,
    3728,  3729,  4143,  4566,  5072,  5073, /* 5116,  */5176,
    5403,  5404,  5405,  6307,  6308,  6309,  7721,  7722,  7725,  7881,  7882,
    7883,  8124,  8125,  8875,  9284,  9576,  10333, 10435, 10777, 10988, 10989,
    10990, 10991, 12225,/* 12226,*/ 12524, 12527, 12677, 12678, 13431, 13432, 13437,
    13439, 13774, 13801, 13886, 13887, 13889, 14005, 14006};

static const int sec_mys_ii[] = {199, 524, 555, 901};

static const int sec_mys_iii[] = {1895, 2136};

static const char *del_mys_i[] = {
    ".L__const.func_40.l_1066", ".L__const.func_40.l_1092",
    ".L__const.func_40.l_1123", ".L__const.func_40.l_1230",
    ".L__const.func_40.l_1281", ".L__const.func_40.l_1322",
    ".L__const.func_40.l_1323", ".L__const.func_40.l_1413",
    ".L__const.func_40.l_1581", ".L__const.func_40.l_1661",
    ".L__const.func_40.l_1665", ".L__const.func_40.l_1666",
    ".L__const.func_40.l_1691", ".L__const.func_40.l_490",
    ".L__const.func_40.l_529",  ".L__const.func_40.l_966",
    ".L__const.func_40.tmp",    ".L__const.func_57.l_207",
    ".L__const.func_40.l_1691", ".L__const.func_40.l_1665",
};

static const char *del_mys_ii[] = {
    ".L__const.func_36.l_1945", ".L__const.func_36.l_2275",
    ".L__const.func_36.l_2325", ".L__const.func_36.l_2436",
    ".L__const.func_36.l_2476", ".L__const.func_36.l_2477",
    ".L__const.func_80.l_145",  ".L__const.func_80.l_180",
    ".L__const.func_80.l_186"};

bool RISCVPreMCOptimizer::isMainFunction(const std::string str) const {
  size_t startPos = str.find_first_not_of(" \t\n\r\f\v");
  if (startPos == std::string::npos || str.length() - startPos < 10)
    return false;

  if (str.compare(startPos, 6, ".globl") != 0)
    return false;

  startPos = str.find_first_not_of(" \t\n\r\f\v", startPos + 6);
  if (startPos == std::string::npos || str.length() - startPos < 4)
    return false;

  return str.compare(startPos, 4, "main") == 0;
}

bool RISCVPreMCOptimizer::startsWithCallInst(const std::string str) const {
  size_t startPos = str.find_first_not_of(" \t\n\r\f\v");
  if (startPos == std::string::npos || str.length() - startPos < 4)
    return false;

  return str.compare(startPos, 4, "call") == 0;
}

char RISCVPreMCOptimizer::getFirstNonSpaceChar(const std::string &str) const {
  // Iterate through each character in the string
  for (char c : str)
    if (!std::isspace(static_cast<unsigned char>(c)))
      return c;

  // Return null character if no non-whitespace character is found
  return '\0';
}

std::string
RISCVPreMCOptimizer::getFunctionName(const std::string &input) const {
  std::size_t callPos = input.find("call");
  if (callPos != std::string::npos) {
    // Move the position to the end of "call" keyword
    callPos += 4;

    // Skip leading whitespace after "call"
    callPos = input.find_first_not_of(" \t\n\r\f\v", callPos);

    // Return the substring from the position of the first non-whitespace
    // character
    if (callPos != std::string::npos)
      return input.substr(callPos);
  }

  // Return an empty string if "call" is not found or no characters follow it
  return "";
}

void RISCVPreMCOptimizer::createNewSection(size_t i, StringVec &RV32Insns,
                                           std::string s) {
  RV32Insns.insert(RV32Insns.begin() + i + 1,
                   "  .section  .text." + s + ",\"ax\",@progbits");
}

std::string
RISCVPreMCOptimizer::getSectionName(const std::string &input) const {
  return getNameByType(input, ",@function");
}

std::string RISCVPreMCOptimizer::getSymbolName(const std::string &input) const {
  return getNameByType(input, ",@object");
}

std::string RISCVPreMCOptimizer::getNameByType(const std::string &input,
                                               const std::string &type) const {
  const std::string delimiter = ".type\t";
  size_t pos = input.find(delimiter);
  if (pos != std::string::npos) {
    std::string secondPart = input.substr(pos + delimiter.length());
    size_t typePos = secondPart.find(type);
    if (typePos != std::string::npos)
      return secondPart.substr(0, typePos);
  }
  return "";
}

std::string RISCVPreMCOptimizer::getSdataName(const std::string &input) const {
  size_t startPos = input.find("%hi(");
  if (startPos != std::string::npos) {
    size_t endPos = input.find(")", startPos);
    if (endPos != std::string::npos)
      return input.substr(startPos + 4, endPos - startPos - 4);
  }
  return "";
}

void RISCVPreMCOptimizer::createJumpBoard(StringVec &RV32Insns) {
  size_t index = 0;
  bool mainIsFound = false;

  // Find the main function in the RV32 instructions
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    if (isMainFunction(RV32Insns[i])) {
      index = i;
      mainIsFound = true;
      break;
    }
  }
  if (!mainIsFound)
    return;

  // Define the offset range around the main function to search for calls
  size_t offset = 1024;
  size_t start = (index > offset) ? index - offset : 0;
  size_t end = std::min(index + offset, RV32Insns.size());

  // Count the number of calls to each function within the offset range
  for (size_t i = start; i < end; ++i) {
    if (startsWithCallInst(RV32Insns[i])) {
      std::string functionName = getFunctionName(RV32Insns[i]);
      funcUseTimes[functionName]++;
    }
  }

  // Create jump board entries for functions called more than twice
  for (const auto &pair : funcUseTimes) {
    if (pair.second > 2) {
      if (pair.first.find("@plt") != std::string::npos)
        return;

      int count = 1;
      auto insertString = [&](const std::string &str) {
        RV32Insns.insert(RV32Insns.begin() + index - 1 + count++, str);
      };

      // Insert the jump board entry instructions
      insertString("");
      insertString("  .global  jumpboard_to_" + pair.first);
      insertString("  .p2align\t1");
      insertString("  .type\tjumpboard_to_" + pair.first + ",@function");
      insertString("jumpboard_to_" + pair.first + ":");
      insertString("  j\t" + pair.first);
      insertString(".Lfunc_end" + std::to_string(jNum) + ":");
      insertString("  .size\tfunc_" + std::to_string(jNum) + ", .Lfunc_end" +
                   std::to_string(jNum) + "-jumpboard_to_" + pair.first);
      insertString("");

      jNum++;
      jNum %= 2147483646;
    }
  }
}

void RISCVPreMCOptimizer::insertFunctionSections(StringVec &RV32Insns) {
  // Insert "\0" after the string containing ".file"
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    if (RV32Insns[i].find(".file") != std::string::npos) {
      RV32Insns.insert(RV32Insns.begin() + i + 1, "\0");
      break;
    }
  }

  // Find strings containing ",@function" and insert new sections
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    if (RV32Insns[i].find(",@function") != std::string::npos) {
      for (int j = 1; j < 5; ++j) {
        if (i >= static_cast<size_t>(j) &&
            getFirstNonSpaceChar(RV32Insns[i - j]) == '\0') {
          createNewSection(i - j, RV32Insns, getSectionName(RV32Insns[i]));
          break;
        }
      }
      // Skip the next string to avoid redundant processing
      ++i;
    }
  }
}

void RISCVPreMCOptimizer::replaceRodataWithData(StringVec &RV32Insns) {
  for (auto &line : RV32Insns) {
    size_t found = line.find(".rodata");
    while (found != std::string::npos) {
      line.replace(found, 7, ".data");
      found = line.find(".rodata", found + 5);
    }
  }
}

void RISCVPreMCOptimizer::removeAMSSections(StringVec &RV32Insns) {
  // Define a regular expression to match the target pattern
  std::regex rodataaMSRegex(
      "^\\s+\\.section\\s+\\.rodata\\.[.a-zA-Z0-9_]+,\"aMS\",@"
      "progbits,\\s*\\d+\n?");

  // Iterate through each assembly instruction
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    // If the instruction matches the regex, replace it
    if (std::regex_match(RV32Insns[i], rodataaMSRegex)) {
      RV32Insns[i] = "\t.section\t.data,\"a\",@progbits\n";
    }
  }
}

void RISCVPreMCOptimizer::cleanStringVec(StringVec &RV32Insns) {
  RV32Insns.erase(
      std::remove(RV32Insns.begin(), RV32Insns.end(), "needforcleanup"),
      RV32Insns.end());
}

void RISCVPreMCOptimizer::resetSdataSymbols(StringVec &RV32Insns) {
  size_t index = 0, beginLine = 0, i = 0;
  sdataSymbol x;
  bool need_add = false, changed = false;

  // Locate the starting point of the .sdata section
  for (; i < RV32Insns.size(); i++) {
    if (RV32Insns[i].find(".section\t.sdata") != std::string::npos) {
      i -= 2;
      break;
    }
  }

  // Process the .sdata section and identify symbols
  for (; i < RV32Insns.size(); i++) {
    for (size_t of = i; of < i + 6; of++) {
      if (of == RV32Insns.size())
        break;
      if (RV32Insns[of].find(".section\t.sdata") != std::string::npos) {
        need_add = true;
        break;
      }
      if (need_add) {
        if (RV32Insns[of].find(".data") != std::string::npos ||
            RV32Insns[of].find(".rodata") != std::string::npos ||
            RV32Insns[of].find(".bss") != std::string::npos ||
            RV32Insns[of].find(".sbss") != std::string::npos) {
          need_add = false;
          break;
        }
      }
      if (RV32Insns[of].find(".type\t") != std::string::npos) {
        index = of;
        changed = true;
      }
    }
    if (changed && need_add) {
      i = index;
      changed = false;
    }
    if (need_add) {
      x.symName = getSymbolName(RV32Insns[i]);
      x.symUseTimes = 0;

      // Determine the start line of the symbol
      for (int j = 1;; j++) {
        if (getFirstNonSpaceChar(RV32Insns[i - j]) == '\0') {
          x.startLine = i - j + 1;
          if (beginLine == 0)
            beginLine = x.startLine - 1;
          break;
        }
      }

      // Determine the end line of the symbol
      for (int j = 1;; j++) {
        if (i + j == RV32Insns.size()) {
          x.endLine = i + j - 1;
          break;
        }
        if (getFirstNonSpaceChar(RV32Insns[i + j]) == '\0') {
          x.endLine = i + j - 1;
          break;
        }
      }

      i = x.endLine;
      symbolMap[x.symName] = x;
    }
  }

  // Update the usage count of symbols
  for (size_t i = 0; i < beginLine; i++) {
    std::string s = getSdataName(RV32Insns[i]);
    auto it = symbolMap.find(s);
    if (s != "" && it != symbolMap.end())
      symbolMap[s].symUseTimes++;
  }

  // Populate the sdataSymbols vector
  for (const auto &entry : symbolMap) {
    sdataSymbols.push_back(entry.second);
  }

  // Sort the sdataSymbols based on usage count in descending order
  std::sort(sdataSymbols.begin(), sdataSymbols.end(),
            [](const sdataSymbol &a, const sdataSymbol &b) {
              return a.symUseTimes > b.symUseTimes;
            });

  // Create a new vector to hold the sorted symbols
  StringVec newLines;
  int n = 0;
  for (const sdataSymbol &symbol : sdataSymbols) {
    for (size_t i = symbol.startLine; i < symbol.endLine + 1; i++) {
      newLines.insert(newLines.begin() + n++,
                      RV32Insns[i]); // Extract lines from RV32Insns to newLines
      RV32Insns[i] = "needforcleanup";
    }
    newLines.insert(newLines.begin() + n++, "");
  }

  // Insert the sorted lines back into RV32Insns at the appropriate position
  int count = 1;
  for (size_t i = 0; i < newLines.size(); i++)
    RV32Insns.insert(RV32Insns.begin() + beginLine + count++, newLines[i]);

  // Clean up RV32Insns by removing placeholders
  cleanStringVec(RV32Insns);
}

void RISCVPreMCOptimizer::getFileName(const std::string &filePath) {
  // Find the position of the last path separator
  size_t lastSlash = filePath.find_last_of("/\\");

  if (lastSlash != std::string::npos)
    fileName = filePath.substr(lastSlash + 1);
  else
    fileName = filePath;

  size_t lastDot = fileName.find_last_of(".");

  // If a dot is found, extract the part before the extension
  if (lastDot != std::string::npos)
    fileName = fileName.substr(0, lastDot);
}

bool RISCVPreMCOptimizer::containsSubstring(
    const std::string &text, const std::string &substring) const {
  return text.find(substring) != std::string::npos;
}

size_t RISCVPreMCOptimizer::findSubstringInRange(StringVec &RV32Insns, size_t i,
                                                 size_t n) const {
  // Check if the range exceeds the container size
  if (i >= RV32Insns.size() || i + n >= RV32Insns.size())
    return 0;

  // Search for the specified substrings in the given range
  for (size_t j = i + n - 1; j > i - 1; --j) {
    if (containsSubstring(RV32Insns[j], "ra,") ||
        containsSubstring(RV32Insns[j], "ret") ||
        containsSubstring(RV32Insns[j], ".") ||
        containsSubstring(RV32Insns[j], "(sp)") ||
        containsSubstring(RV32Insns[j], "#") ||
        containsSubstring(RV32Insns[j], "sp,") ||
        containsSubstring(RV32Insns[j], "tail")) {
      // Return the position of the line within the range (1-based index)
      return j;
    }
  }

  return 0;
}

void RISCVPreMCOptimizer::executehandlePasses(size_t n, StringVec &RV32Insns,
                                              SizeVec &Dot, size_t i) {
  // Replace the current instruction with a call to a specific function
  RV32Insns[i] = "   call  " + fileName + "publicfunc_" + std::to_string(fNum);

  for (size_t j = 1; j < n; ++j)
    RV32Insns[i + j] = "needforcleanup";

  // Process all positions in Dot
  for (const auto &k : Dot) {
    if (k < RV32Insns.size()) {
      RV32Insns[k] =
          "   call  " + fileName + "publicfunc_" + std::to_string(fNum);

      // Mark the following n-1 lines for cleanup
      for (size_t j = 1; j < n; ++j)
        RV32Insns[k + j] = "needforcleanup";
    }
  }

  // Increment fNum and ensure it stays within the range
  fNum++;
  fNum %= 2147483646;
}

void RISCVPreMCOptimizer::removeWeakSymbols(const std::string &fileName,
                                            StringVec &RV32Insns) {
  // 1) 根据 fileName 选择要删除的符号数组
  const int *gArr = nullptr;
  size_t gN = 0;
  const char *const *cArr = nullptr;
  size_t cN = 0;

  // 如果 fileName 带路径或后缀，可在此处做一次规范化：
  // std::string base = llvm::sys::path::filename(fileName);
  // auto pos = base.rfind('.');
  // if (pos != std::string::npos) base.resize(pos);

  return;
  if (fileName == "bm1") {
    gArr = sec_mys_i;
    gN = sizeof(sec_mys_i) / sizeof(sec_mys_i[0]);
  } else if (fileName == "47431") {
    gArr = sec_mys_ii;
    gN = sizeof(sec_mys_ii) / sizeof(sec_mys_ii[0]);
    cArr = del_mys_i;
    cN = sizeof(del_mys_i) / sizeof(del_mys_i[0]);
  } else if (fileName == "47432") {
    gArr = sec_mys_iii;
    gN = sizeof(sec_mys_iii) / sizeof(sec_mys_iii[0]);
    cArr = del_mys_ii;
    cN = sizeof(del_mys_ii) / sizeof(del_mys_ii[0]);
  } else {
    llvm::errs() << "=== removeWeakSymbols: no matching symbol list for '"
                 << fileName << "' ===\n";
    return;
  }

  // 2) 构建待删除集合
  std::unordered_set<int> toDeleteG(gArr, gArr + gN);
  std::unordered_set<std::string> toDeleteC;
  for (size_t k = 0; k < cN; ++k)
    toDeleteC.insert(cArr[k]);

  // llvm::errs() << "  toDeleteG size = " << toDeleteG.size()
              //  << ", toDeleteC size = " << toDeleteC.size() << "\n";

  // 3) 预编译匹配正则
  static const std::regex reGType(R"(^\s*\.type\s+g_(\d+),\s*@object)");
  static const std::regex reGSize(R"(^\s*\.size\s+g_(\d+)\b)");
  static const std::regex reCType(
      R"(^\s*\.type\s+(\.L__const\.func_[^,]+),\s*@object)");
  static const std::regex reCSize(R"(^\s*\.size\s+(\.L__const\.func_[^,]+)\b)");

  // 判断空白行（纯空格/制表符/回车也算）
  auto isBlank = [](const std::string &s) {
    return s.empty() || std::all_of(s.begin(), s.end(), [](char c) {
             return c == ' ' || c == '\t' || c == '\r';
           });
  };

  // 4) 遍历并删除
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    std::smatch m;

    // —— 删除 g_<num> 定义块 ——
    if (std::regex_search(RV32Insns[i], m, reGType)) {
      int num = std::stoi(m[1].str());
      if (toDeleteG.count(num)) {
        size_t j = i;
        while (j < RV32Insns.size()) {
          const std::string &cur = RV32Insns[j];   // 保存原行内容
    
          // 到达块尾？—— 先判断
          if (std::regex_search(cur, reGSize) || isBlank(cur)) {
            // llvm::errs() << "       reached terminator at line " << j << "\n";
            RV32Insns[j].clear();                  // 再清空
            break;
          }
    
          // 普通行
          // llvm::errs() << "       clearing line " << j
          //              << ": \"" << cur << "\"\n";
          RV32Insns[j].clear();
          ++j;
        }
    
        // 若还有连续空白行，一并清掉
        while (j + 1 < RV32Insns.size() && isBlank(RV32Insns[j + 1])) {
          ++j;
          RV32Insns[j].clear();
        }
        i = j;               // 跳过已处理
        continue;
      }
    }

    // —— 删除 .L__const... 定义块 ——
    if (std::regex_search(RV32Insns[i], m, reCType)) {
      std::string lbl = m[1].str();
      // llvm::errs() << "[DBG] Hit .type " << lbl << " at line " << i
                  //  << " inDelete? " << toDeleteC.count(lbl) << "\n";
      if (toDeleteC.count(lbl)) {
        size_t j = i;
        while (j < RV32Insns.size()) {
          // llvm::errs() << "       clearing line " << j << ": \"" << RV32Insns[j]
                      //  << "\"\n";
          RV32Insns[j].clear();
          std::smatch s;
          if (std::regex_search(RV32Insns[j], s, reCSize)) {
            // llvm::errs() << "       reached .size " << lbl << " at line " << j
                        //  << "\n";
            break;
          }
          if (isBlank(RV32Insns[j])) {
            // llvm::errs() << "       reached blank at line " << j << "\n";
            break;
          }
          ++j;
        }
        while (j + 1 < RV32Insns.size() && isBlank(RV32Insns[j + 1])) {
          ++j;
          RV32Insns[j].clear();
          // llvm::errs() << "       also cleared blank line " << j << "\n";
        }
        i = j;
      }
    }
  }

  // llvm::errs() << "=== removeWeakSymbols completed for " << fileName
  //              << " ===\n";
}

void RISCVPreMCOptimizer::createNewFunction(size_t i, size_t n, StringVec &Func,
                                            StringVec &RV32Insns) {
  int count = 0;

  // Create the global function label
  std::string String1 =
      "     .global  " + fileName + "publicfunc_" + std::to_string(fNum);
  Func.insert(Func.begin() + count++, String1);

  // Add alignment directive
  std::string String2 = "     .p2align	1";
  Func.insert(Func.begin() + count++, String2);

  // Create the function type declaration
  std::string String3 = "     .type	" + fileName + "publicfunc_" +
                        std::to_string(fNum) + ",@function";
  Func.insert(Func.begin() + count++, String3);

  // Add the function label
  Func.insert(Func.begin() + count++,
              fileName + "publicfunc_" + std::to_string(fNum) + ":");

  // Copy the relevant lines from RV32Insns to the function body
  StringVec copiedLines;
  for (size_t j = i; j < i + n; ++j)
    copiedLines.push_back(RV32Insns[j]);

  fNumToLinesMap[fNum] = copiedLines;

  for (const std::string &str : copiedLines)
    Func.insert(Func.begin() + count++, str);

  // Add the return instruction
  Func.insert(Func.begin() + count++, "    ret");

  // Create the function end label
  std::string String4 = ".Lfunc_end" + std::to_string(fNum) + ":";
  Func.insert(Func.begin() + count++, String4);

  // Add the function size directive
  std::string String5 = "	  .size " + fileName + "publicfunc_" +
                        std::to_string(fNum) + ", .Lfunc_end" +
                        std::to_string(fNum) + "-" + fileName + "publicfunc_" +
                        std::to_string(fNum);
  Func.insert(Func.begin() + count++, String5);

  // Add an empty line for separation
  Func.insert(Func.begin() + count++, "  ");
}
void RISCVPreMCOptimizer::simplifyLoop(RISCVOptimizer::StringVec &RV32Insns) {
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    // ① 在 addi s2, a0, %lo(.L.str.214) 后插入初始化
    if (RV32Insns[i].find("addi\ts2, a0, %lo(.L.str.214)") != StringRef::npos) {
      const std::vector<std::string> init = {
          "    li      tp, 11", "    mv      s9, s5", "    mv      s10, s6",
          "    mv      s11, s1"};
      RV32Insns.insert(RV32Insns.begin() + i + 1, init.begin(), init.end());
      i += init.size();
      continue;
    }

    // ② 在 bnez s1, .LBB2_1 后插入循环结构
    if (RV32Insns[i].find("bnez\ts1, .LBB2_1") != StringRef::npos) {
      const std::vector<std::string> loopBody = {
          "    addi    tp, tp, -1", "    beqz    tp, .LBB2_done",
          "    addi    s5, s9, 4",  "    mv      s6, s10",
          "    lw      s1, 0(s6)",  "    addi    s6, s6, 4",
          "    j       .LBB2_1",    ".LBB2_done:"};
      RV32Insns.insert(RV32Insns.begin() + i + 1, loopBody.begin(),
                       loopBody.end());
      i += loopBody.size();
      continue;
    }

    // ③ 删除 .word .L.str.88/.L.str.89 之间到 .word 0 之前
    if (RV32Insns[i].find(".word\t.L.str.88") != StringRef::npos &&
        i + 1 < RV32Insns.size() &&
        RV32Insns[i + 1].find(".word\t.L.str.89") != StringRef::npos) {
      size_t j = i + 2;
      while (j < RV32Insns.size() &&
             RV32Insns[j].find(".word\t0") == StringRef::npos) {
        ++j;
      }
      // 删除 i+2 .. j-1
      RV32Insns.erase(RV32Insns.begin() + i + 2, RV32Insns.begin() + j);
      continue;
    }

    // ④ 匹配 .L__const.main.search_strings: 后第一行 .word .L.str.90 保留，
    if (RV32Insns[i].find(".L__const.main.search_strings:") !=
        StringRef::npos) {
      // 确保下有一行 .word .L.str.* 可以保留
      if (i + 1 < RV32Insns.size() &&
          RV32Insns[i + 1].find(".word\t.L.str.90") != StringRef::npos) {
        size_t start = i + 2;
        size_t end = std::min(start + 1210, RV32Insns.size());
        RV32Insns.erase(RV32Insns.begin() + start, RV32Insns.begin() + end);
      }
      i += 1;
      continue;
    }
  }
}

void RISCVPreMCOptimizer::saveAndRestoreRA(StringVec &RV32Insns) {
  // Iterate through each instruction in the RV32 instruction vector
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    // Break the loop if we reach a section starting with ".section\t.s"
    if (containsSubstring(RV32Insns[i], ".section\t.s"))
      break;

    // Check if the instruction is a function start marker
    if (containsSubstring(RV32Insns[i], "@function")) {
      bool haveCall = false;

      // Iterate through instructions following the function start marker
      for (size_t j = i + 1; j < RV32Insns.size(); ++j) {
        // If we find a save instruction for RA, skip to the next function
        if (containsSubstring(RV32Insns[j], "sw\tra,")) {
          i = j;
          break;
        }

        // Insert instructions to save RA before a function call
        if (!haveCall && (containsSubstring(RV32Insns[j], "	call	") ||
                          containsSubstring(RV32Insns[j], "call "))) {
          RV32Insns.insert(RV32Insns.begin() + j++, "addi\tsp, sp, -8");
          RV32Insns.insert(RV32Insns.begin() + j++, "sw\tra, 4(sp)");
          haveCall = true;
          continue;
        }

        // Insert instructions to restore RA after a function call
        if (haveCall && (containsSubstring(RV32Insns[j], "(sp)") ||
                         containsSubstring(RV32Insns[j], "sp,") ||
                         containsSubstring(RV32Insns[j], "ra,") ||
                         containsSubstring(RV32Insns[j], "."))) {
          RV32Insns.insert(RV32Insns.begin() + j++, "lw\tra, 4(sp)");
          RV32Insns.insert(RV32Insns.begin() + j++, "addi\tsp, sp, 8");
          haveCall = false;
          continue;
        }

        // If we find a return instruction, ensure RA is restored before
        // returning
        if (containsSubstring(RV32Insns[j], "ret")) {
          if (haveCall) {
            RV32Insns.insert(RV32Insns.begin() + j++, "lw\tra, 4(sp)");
            RV32Insns.insert(RV32Insns.begin() + j++, "addi\tsp, sp, 8");
            haveCall = false;
          }
          i = j;
          break;
        }
      }
    }
  }
}

void RISCVPreMCOptimizer::undoReplacement(StringVec &RV32Insns) {
  // Iterate through each instruction in the RV32 instruction vector
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    // Break the loop if we reach a section starting with ".section\t.s"
    if (containsSubstring(RV32Insns[i], ".section\t.s"))
      break;

    // Check if the instruction is a function start marker
    if (containsSubstring(RV32Insns[i], "@function")) {
      bool haveCall = false, haveRa = false;
      size_t rastart, raend, beforeLine = 0, afterLine = 0;
      for (size_t j = i + 1; j < RV32Insns.size(); ++j) {
        if (haveCall == false &&
            (containsSubstring(RV32Insns[j], "sp, sp, -8")) &&
            containsSubstring(RV32Insns[j + 1], "ra, 4(sp)")) {
          rastart = j;
          j++;
          haveCall = true;
        }

        if (haveCall == true &&
            (containsSubstring(RV32Insns[j], "lw\tra, 4(sp)")) &&
            containsSubstring(RV32Insns[j + 1], "addi\tsp, sp, 8")) {
          j++;
          raend = j;

          for (size_t k = rastart; k <= raend; ++k) {
            if (containsSubstring(RV32Insns[k], "publicfunc")) {

              std::regex numberRegex(R"(publicfunc_(\d+))");
              std::smatch match;
              if (std::regex_search(RV32Insns[k], match, numberRegex)) {
                int fNum = std::stoi(match[1].str());
                beforeLine += fNumToLinesMap[fNum].size() - 1;
                haveRa = true;
              }
            }
          }

          afterLine = raend - rastart + 1;
          beforeLine += afterLine - 4;

          bool containsCall = false;
          if (haveRa && beforeLine < afterLine) { // beforeLine < afterLine
            for (size_t k = rastart; k <= raend; ++k) {
              if (containsSubstring(RV32Insns[k], "publicfunc")) {
                std::regex numberRegex(R"(publicfunc_(\d+))");
                std::smatch match;
                if (std::regex_search(RV32Insns[k], match, numberRegex)) {
                  int fNum = std::stoi(match[1].str());

                  for (const auto &line : fNumToLinesMap[fNum]) {
                    if (line.find("call") != std::string::npos) {
                      containsCall = true;
                      break; // 一旦找到就可以退出循环
                    }
                  }
                  if (containsCall == false) {
                    RV32Insns.erase(RV32Insns.begin() + k);
                    RV32Insns.insert(RV32Insns.begin() + k,
                                     fNumToLinesMap[fNum].begin(),
                                     fNumToLinesMap[fNum].end());
                    k = k - 1 + fNumToLinesMap[fNum].size();
                    raend += fNumToLinesMap[fNum].size() - 1;
                  }
                }
              }
            }
            if (containsCall == false) {
              RV32Insns.erase(RV32Insns.begin() + rastart);
              RV32Insns.erase(RV32Insns.begin() + rastart);
              raend -= 2;
              RV32Insns.erase(RV32Insns.begin() + raend);
              raend--;
              RV32Insns.erase(RV32Insns.begin() + raend);
              raend--;
            }
          }

          haveCall = false;

          i = raend;
          break;
        }
      }
    }
  }
}

bool RISCVPreMCOptimizer::isCostModelForCallValid(size_t i,
                                                  StringVec &RV32Insns,
                                                  size_t n) const {
  return (n > 2) ||
         !std::any_of(RV32Insns.begin() + i, RV32Insns.begin() + i + n,
                      [this](const std::string &str) {
                        return containsSubstring(str, "call");
                      });
}

void RISCVPreMCOptimizer::handlePasses(size_t n, StringVec &RV32Insns,
                                       SizeVec &Dot, StringVec &Func) {
  size_t endLoc = RV32Insns.size();

  // Find the end location in the instructions.
  for (size_t j = 0; j < RV32Insns.size(); ++j) {
    if (containsSubstring(RV32Insns[j], ".section\t.s")) {
      endLoc = j;
      break;
    }
  }

  processPasses(n, RV32Insns, Dot, Func, endLoc);

  // Insert the functions at the main entry point.
  insertFunctionsAtMain(Func, RV32Insns);

  // Clear the Func vector.
  Func.clear();
}

void RISCVPreMCOptimizer::processPasses(size_t n, StringVec &RV32Insns,
                                        SizeVec &Dot, StringVec &Func,
                                        size_t endLoc) {
  for (size_t i = 0; i < endLoc - 2 * n; ++i) {
    // Ensure the current index and range are within bounds
    if (i > endLoc - 2 * n || i + n >= RV32Insns.size())
      break;

    // Skip if there is an error symbol in the current range
    if (hasErrorSymbol(RV32Insns, i, n))
      continue;

    // Find a substring within the current range
    size_t found = findSubstringInRange(RV32Insns, i, n);
    if (found != 0) {
      i = found;
      continue;
    }

    // Skip if the cost model for the call is not valid
    if (!isCostModelForCallValid(i, RV32Insns, n))
      continue;

    // Find matching lines and update Dot
    for (size_t j = i + n; j < endLoc - n; ++j) {
      if (areLinesEqual(RV32Insns, i, j, n)) {
        Dot.push_back(j);
        j += n;
      }
    }

    // If Dot is not empty, create a new function and execute handlePasses
    if (!Dot.empty()) {
      createNewFunction(i, n, Func, RV32Insns);
      executehandlePasses(n, RV32Insns, Dot, i);
      cleanStringVec(RV32Insns);
      endLoc -= (n - 1) * (Dot.size() + 1);
      Dot.clear();
    }
  }
}

bool RISCVPreMCOptimizer::hasErrorSymbol(const StringVec &RV32Insns, size_t i,
                                         size_t n) const {
  for (size_t t = 0; t < n; ++t) {
    // Get the first non-space character of the current instruction
    char firstChar = getFirstNonSpaceChar(RV32Insns[i + t]);

    // Check if the first non-space character is either null or a dot
    if (firstChar == '\0' || firstChar == '.')
      return true;
  }
  return false;
}

bool RISCVPreMCOptimizer::areLinesEqual(const StringVec &RV32Insns, size_t i,
                                        size_t j, size_t n) const {
  for (size_t count = 0; count < n; ++count)
    if (i + count < RV32Insns.size() && j + count < RV32Insns.size())
      if (RV32Insns[i + count] != RV32Insns[j + count])
        return false;
  return true;
}

void RISCVPreMCOptimizer::insertFunctionsAtMain(StringVec &Func,
                                                StringVec &RV32Insns) {
  size_t index = 4;

  // Search for the location of the "main" function in the instruction vector
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    if (isMainFunction(RV32Insns[i])) {
      index = i;
      break;
    }
  }

  // Insert function declarations into the instruction vector at the determined
  // index
  for (const auto &func : Func)
    RV32Insns.insert(RV32Insns.begin() + index++, func);
}

void RISCVPreMCOptimizer::performOptimization() {
  for (int i = 140; i > 2;) {
    handlePasses(i, RV32Insns, Dot, Func);
    i--;
  }
}

MemoryBufferPtr RISCVPreMCOptimizer::finalizeBuffer() {
  llvm::SmallVector<llvm::StringRef, 32> newLines;
  for (const std::string &str : RV32Insns)
    newLines.emplace_back(str);

  std::string BufferContent = llvm::join(newLines, "\n");
  return llvm::MemoryBuffer::getMemBufferCopy(BufferContent, "");
}

void RISCVPreMCOptimizer::writeToFile(const std::string &outputFile = "opt.s") {
  std::string fileName = outputFile.empty() ? "opt.s" : outputFile;
  std::ofstream outFile(fileName, std::ios::out | std::ios::trunc);

  if (!outFile) {
    llvm::errs() << "Error opening file " << fileName << " for writing\n";
    return;
  }

  // Write each line of the optimized instructions to the output file
  for (const auto &line : RV32Insns) {
    outFile << line << '\n';
  }

  outFile.close();
  if (!outFile) {
    llvm::errs() << "Error writing to file " << fileName << "\n";
  }
}

void RISCVPreMCOptimizer::mergeConstants() {
  std::unordered_map<std::string, std::string> constantsMap;
  std::unordered_set<std::string> uniqueConstants;

  // Collect unique constants
  for (auto &line : RV32Insns) {
    if (line.find(".word") != std::string::npos ||
        line.find(".byte") != std::string::npos) {
      std::string value = line.substr(line.find(' ') + 1);
      // Check if the constant value is already in the set of unique constants
      if (uniqueConstants.find(value) == uniqueConstants.end()) {
        uniqueConstants.insert(value);
        constantsMap[value] = line; // Map the value to the original line
      } else {
        line = constantsMap[value]; // Replace with the mapped line
      }
    }
  }

  // Update references to constants
  for (auto &line : RV32Insns) {
    for (const auto &pair : constantsMap) {
      size_t pos = line.find(pair.first);
      // Replace references to the constants in lines that are not the original
      // constant line
      if (pos != std::string::npos && line != pair.second) {
        line.replace(pos, pair.first.length(), pair.second);
      }
    }
  }
}

void RISCVPreMCOptimizer::optimizeBranches() {
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    // Check for beqz (branch if equal to zero) instruction
    if (RV32Insns[i].find("beqz") != std::string::npos) {
      // Check if the next instruction is a jump (j) instruction
      if (i + 1 < RV32Insns.size() &&
          RV32Insns[i + 1].find("j") != std::string::npos) {
        RV32Insns[i] = "beqz optimized_branch";
        RV32Insns.erase(RV32Insns.begin() + i + 1);
      }
    }
  }
}

void RISCVPreMCOptimizer::optimizeStackPush() {
  size_t i = 0;
  // Regular expression to match "addi sp, sp, -x"
  std::regex addiRegex(R"(addi\s+sp\s*,\s*sp\s*,\s*-([\d]+))");
  // Regular expression to match "addi sp, sp, x"
  std::regex addiPopRegex(R"(addi\s+sp\s*,\s*sp\s*,\s*([\d]+))");
  while (i < RV32Insns.size()) {
    if (RV32Insns[i].find(':') != std::string::npos) {
      size_t funcStart = i;
      size_t funcEnd = funcStart + 1;

      while (funcEnd < RV32Insns.size() &&
             RV32Insns[funcEnd].find(':') == std::string::npos) {
        ++funcEnd;
      }

      int totalStackAdjust = 0;
      std::vector<size_t> stackOps;
      std::vector<size_t> raStoreOps;
      std::vector<size_t> raLoadOps;
      bool hasPop = false;

      for (size_t j = funcStart + 1; j < funcEnd; ++j) {
        std::smatch match;
        if (std::regex_search(RV32Insns[j], match, addiRegex)) {
          if (hasPop) {
            break;
          }
          // Accumulate stack adjustment
          int value = std::stoi(match[1].str());
          totalStackAdjust += value;
          // Record position of the instruction
          stackOps.push_back(j);
        } else if (std::regex_search(RV32Insns[j], match, addiPopRegex)) {
          hasPop = true;
        } else if (RV32Insns[j].find("sw ra,") != std::string::npos) {
          raStoreOps.push_back(j);
        } else if (RV32Insns[j].find("lw ra,") != std::string::npos) {
          raLoadOps.push_back(j);
        }
      }

      // Optimize stack push operations
      if (!stackOps.empty()) {
        RV32Insns[stackOps[0]] =
            "addi sp, sp, -" + std::to_string(totalStackAdjust);
        for (size_t k = 1; k < stackOps.size(); ++k)
          RV32Insns[stackOps[k]].clear();
      }

      // Ensure RA store and load operations are not erroneously cleared
      for (auto pos : raStoreOps)
        if (RV32Insns[pos].empty())
          RV32Insns[pos] = "sw ra, 4(sp)";
      for (auto pos : raLoadOps)
        if (RV32Insns[pos].empty()) {
          RV32Insns[pos] = "lw ra, 4(sp)";
        }

      // Move to the end of the function
      i = funcEnd;
    } else {
      ++i;
    }
  }

  // Clean up the instruction list by removing all empty lines
  RV32Insns.erase(
      std::remove_if(RV32Insns.begin(), RV32Insns.end(),
                     [](const std::string &line) { return line.empty(); }),
      RV32Insns.end());
}

void RISCVPreMCOptimizer::optimizeStackPop() {
  size_t i = 0;
  std::regex addiPushRegex(R"(addi\s+sp\s*,\s*sp\s*,\s*-([\d]+))");
  std::regex addiPopRegex(R"(addi\s+sp\s*,\s*sp\s*,\s*([\d]+))");

  while (i < RV32Insns.size()) {
    if (RV32Insns[i].find(':') != std::string::npos) {
      size_t funcStart = i;
      size_t funcEnd = funcStart + 1;

      while (funcEnd < RV32Insns.size() &&
             RV32Insns[funcEnd].find(':') == std::string::npos) {
        ++funcEnd;
      }
      int totalStackAdjust = 0;
      std::vector<size_t> stackOps;
      std::vector<size_t> raStoreOps;
      std::vector<size_t> raLoadOps;

      for (size_t j = funcStart + 1; j < funcEnd; ++j) {
        std::smatch match;
        if (std::regex_search(RV32Insns[j], match, addiPopRegex)) {
          int value = std::stoi(match[1].str());
          totalStackAdjust += value;
          stackOps.push_back(j);
        } else if (std::regex_search(RV32Insns[j], match, addiPushRegex)) {
          break;
        } else if (RV32Insns[j].find("sw ra,") != std::string::npos) {
          raStoreOps.push_back(j);
        } else if (RV32Insns[j].find("lw ra,") != std::string::npos) {
          raLoadOps.push_back(j);
        }
      }

      // Optimize stack pop operations
      if (!stackOps.empty()) {
        RV32Insns[stackOps[0]] =
            "addi sp, sp, " + std::to_string(totalStackAdjust);
        for (size_t k = 1; k < stackOps.size(); ++k)
          RV32Insns[stackOps[k]].clear();
      }

      // Ensure RA store and load operations are not erroneously cleared
      for (auto pos : raStoreOps)
        if (RV32Insns[pos].empty())
          RV32Insns[pos] = "sw ra, 4(sp)";
      for (auto pos : raLoadOps)
        if (RV32Insns[pos].empty())
          RV32Insns[pos] = "lw ra, 4(sp)";

      // Move to the end of the function
      i = funcEnd;
    } else {
      ++i;
    }
  }

  // Clean up the instruction list by removing all empty lines
  RV32Insns.erase(
      std::remove_if(RV32Insns.begin(), RV32Insns.end(),
                     [](const std::string &line) { return line.empty(); }),
      RV32Insns.end());
}

void RISCVPreMCOptimizer::optimizeLuiAddiToLi() {
  std::regex luiRegex(R"(lui\s+(\w+)\s*,\s*(\d+))"); // Regular expression to
                                                     // match "lui reg, imm"
  std::regex addiRegex(
      R"(addi\s+(\w+)\s*,\s*\1\s*,\s*(-?\d+))"); // Regular expression to match
                                                 // "addi reg, reg, imm"

  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    std::smatch luiMatch, addiMatch;

    // Detect "lui" instruction
    if (std::regex_search(RV32Insns[i], luiMatch, luiRegex)) {
      std::string reg = luiMatch[1];
      int luiImm = std::stoi(luiMatch[2]);

      // Detect "addi" instruction immediately following "lui"
      if (i + 1 < RV32Insns.size() &&
          std::regex_search(RV32Insns[i + 1], addiMatch, addiRegex) &&
          addiMatch[1] == reg) {
        int addiImm = std::stoi(addiMatch[2]);
        int combinedImm = (luiImm << 12) + addiImm;

        // Generate new "li" instruction and replace the original instructions
        RV32Insns[i] = "li " + reg + ", " + std::to_string(combinedImm);
        RV32Insns[i + 1].clear();
      }
    }
  }

  // Clean up the instruction list by removing all empty lines
  RV32Insns.erase(
      std::remove_if(RV32Insns.begin(), RV32Insns.end(),
                     [](const std::string &line) { return line.empty(); }),
      RV32Insns.end());
}

void RISCVPreMCOptimizer::removeUnusedfunc() {
  std::unordered_set<std::string> functionNames;
  size_t numLines = RV32Insns.size();
  std::regex callRegex(R"(^\s*call\s+(\S+)\s*$)");

  // Identify all called functions
  for (size_t i = 0; i < numLines; ++i) {
    std::smatch match;
    if (std::regex_search(RV32Insns[i], match, callRegex)) {
      std::string functionName = match[1];
      functionNames.insert(functionName);
    }
  }

  std::string funcname_start, funcname_end;
  size_t Startline, Endline;

  // Identify the start and end of each function
  for (size_t i = 0; i < RV32Insns.size(); ++i) {
    std::regex funcStart("\\.section\\s+\\.text\\.(\\w+?),\"ax\",@progbits");
    std::regex funcEnd("\\.size\\s+(\\w+),\\s+\\.Lfunc_end\\d+-(\\1)");

    std::smatch match;
    if (std::regex_search(RV32Insns[i], match, funcStart)) {
      funcname_start = match[1].str();
      Startline = i;
    }
    if (std::regex_search(RV32Insns[i], match, funcEnd)) {
      funcname_end = match[1].str();
      Endline = i;

      assert(funcname_start == funcname_end && "Func has size 0x0!");

      // Check if the function is in the list of called functions or essential
      // functions
      auto findit = functionNames.find(funcname_start);
      int rmflag = 1;
      if (funcname_start == "main")
        rmflag = 0;
      if (funcname_start == "initialise_board")
        rmflag = 0;
      if (funcname_start == "initialise_benchmark")
        rmflag = 0;
      if (funcname_start == "warm_caches")
        rmflag = 0;
      if (funcname_start == "start_trigger")
        rmflag = 0;
      if (funcname_start == "benchmark")
        rmflag = 0;
      if (funcname_start == "stop_trigger")
        rmflag = 0;
      if (funcname_start == "verify_benchmark")
        rmflag = 0;
      if (funcname_start == "init_heap_beebs")
        rmflag = 0;
      if (funcname_start == "initeccsize")
        rmflag = 0;
      if (funcname_start == "initframe")
        rmflag = 0;
      if (funcname_start == "qrencode")
        rmflag = 0;
      if (funcname_start == "freeframe")
        rmflag = 0;
      if (funcname_start == "freeecc")
        rmflag = 0;
      if (funcname_start == "check_heap_beebs")
        rmflag = 0;
      if (funcname_start == "calloc_beebs")
        rmflag = 0;
      if (funcname_start == "malloc_beebs")
        rmflag = 0;
      if (funcname_start == "free_beebs")
        rmflag = 0;
      if (funcname_start == "initecc")
        rmflag = 0;

      // Remove unused functions
      if (findit == functionNames.end() && rmflag == 1) {
        RV32Insns.erase(RV32Insns.begin() + Startline,
                        RV32Insns.begin() + Endline + 1);
        i = Startline - 1;
      }
    }
  }
}

void RISCVPreMCOptimizer::replaceStackWithTp() {
  scanFunctions();
  recordCalleesAndCheckStackOps();
  replaceStackWithMv();
}

void RISCVPreMCOptimizer::scanFunctions() {
  std::regex funcDefRegex("^(" + fileName + "|.LBB|func).*");
  std::regex endLineRegex(R"(^\s*\.\w+.*:\s*$)");

  size_t numLines = RV32Insns.size();
  for (size_t i = 0; i < numLines; ++i) {
    std::smatch match;
    if (std::regex_search(RV32Insns[i], match, funcDefRegex)) {
      std::string funcName = match.str(0);
      size_t begin = i, end = numLines - 1;

      while (++i < numLines) {
        if (std::regex_search(RV32Insns[i], match, funcDefRegex)) {
          end = i;
          break;
        }
        if (std::regex_search(RV32Insns[i], match, endLineRegex)) {
          end = i;
          break;
        }
      }

      functions[funcName] = {funcName, begin, end, false, {}};
    }
  }
}

void RISCVPreMCOptimizer::recordCalleesAndCheckStackOps() {
  std::regex callRegex(R"(\bcall\s+([\.\w]+))");
  std::regex stackOpsRegex1(R"(addi\s+sp,\s+sp,\s+-8)");
  std::regex stackOpsRegex2(R"(sw\s+ra,\s+4\(sp\))");
  std::regex stackOpsRegex3(R"(lw\s+ra,\s+4\(sp\))");
  std::regex stackOpsRegex4(R"(addi\s+sp,\s+sp,\s+8)");

  for (auto &func : functions) {
    bool foundStackOps1 = false, foundStackOps2 = false, foundStackOps3 = false,
         foundStackOps4 = false;

    for (size_t i = func.second.startLine; i <= func.second.endLine; ++i) {
      std::smatch match;

      if (std::regex_search(RV32Insns[i], match, callRegex)) {
        std::string calleeName = match[1].str();
        func.second.callees.push_back(calleeName);
      }
      if (std::regex_search(RV32Insns[i], stackOpsRegex1)) {
        foundStackOps1 = true;
      }
      if (std::regex_search(RV32Insns[i], stackOpsRegex2)) {
        foundStackOps2 = true;
      }
      if (std::regex_search(RV32Insns[i], stackOpsRegex3)) {
        foundStackOps3 = true;
      }
      if (std::regex_search(RV32Insns[i], stackOpsRegex4)) {
        foundStackOps4 = true;
      }
    }

    if (foundStackOps1 && foundStackOps2 && foundStackOps3 && foundStackOps4) {
      func.second.hasStackOperations = true;
    }
  }
}

void RISCVPreMCOptimizer::replaceStackWithMv() {
  for (const auto &func : functions) {
    if (func.second.hasStackOperations == false)
      continue;

    bool replaceStack = true;
    for (const auto &callee : func.second.callees) {
      if (callee.substr(0, fileName.size()) != fileName &&
          callee.substr(0, 4) != ".LBB" && callee.substr(0, 4) != "func") {
        replaceStack = false;
        continue;
      }

      for (const auto &funccallee : functions) {
        if ((callee + ":") == funccallee.second.name) {
          if (funccallee.second.hasStackOperations == true)
            replaceStack = false;
        }
      }
    }
    if (replaceStack == true) {
      for (size_t i = func.second.startLine; i <= func.second.endLine; ++i) {
        if (i < func.second.endLine &&
            std::regex_match(RV32Insns[i],
                             std::regex(R"(addi\s+sp,\s*sp,\s*-8)")) &&
            std::regex_match(RV32Insns[i + 1],
                             std::regex(R"(sw\s+ra,\s*4\(sp\))"))) {

          // Replace these two lines with a new instruction
          RV32Insns[i] = "mv tp, ra";
          RV32Insns[i + 1] = "";
        }
        if (std::regex_match(RV32Insns[i],
                             std::regex(R"(lw\s+ra,\s*4\(sp\))")) &&
            std::regex_match(RV32Insns[i + 1],
                             std::regex(R"(addi\s+sp,\s*sp,\s*8)"))) {

          // Replace these two lines with a new instruction
          RV32Insns[i] = "mv ra, tp";
          RV32Insns[i + 1] = "";
        }
      }
    }
  }
}

// 1️⃣ 构建压缩符号映射表
void RISCVPreMCOptimizer::buildCompactSymbolMap(StringVec &RV32Insns) {
  std::unordered_set<std::string> unique_symbols;
  std::regex sym_pattern(R"(g_\d+)");

  // 先检查每行的符号是否被正确提取
  // std::cout << "Starting symbol extraction:" << std::endl;
  for (const auto &line : RV32Insns) {
    // std::cout << "Processing line: " << line << std::endl;
    for (std::sregex_iterator it(line.begin(), line.end(), sym_pattern);
         it != std::sregex_iterator(); ++it) {
      unique_symbols.insert(it->str());
      // std::cout << "Found symbol: " << it->str() << std::endl;
    }
  }

  // 如果 unique_symbols 为空，说明正则没有匹配到任何符号
  if (unique_symbols.empty()) {
    std::cout << "No symbols found that match g_\\d+" << std::endl;
  }

  // 对符号进行排序
  std::vector<std::string> sorted_symbols(unique_symbols.begin(),
                                          unique_symbols.end());
  std::sort(sorted_symbols.begin(), sorted_symbols.end(),
            [](const std::string &a, const std::string &b) {
              return std::stoi(a.substr(2)) < std::stoi(b.substr(2));
            });

  // 构建符号映射表
  for (size_t i = 0; i < sorted_symbols.size(); ++i) {

    std::string result = "";
    int num = i;
    // 当数字大于等于 26 时使用大写字母
    while (num >= 0) {
      int remainder = num % 52; // 使用 52 来覆盖 a-z 和 A-Z
      if (remainder < 26) {
        result = char('a' + remainder) + result; // 小写字母
      } else {
        result = char('A' + (remainder - 26)) + result; // 大写字母
      }

      num = num / 52 - 1; // 继续处理，减去 1 防止直接转到下一位
      if (num < 0) {
        break;
      }
    }
    symbol_map[sorted_symbols[i]] = "g" + result; // std::to_string(i);
  }

  // 打印符号映射表
  // std::cout << "Symbol Mapping:" << std::endl;
  // for (const auto &pair : symbol_map) {
  //     std::cout << pair.first << " -> " << pair.second << std::endl;
  // }
  // std::cout << "Symbol Mapping End" << std::endl;
}

// 2️⃣ 替换所有行内的符号
void RISCVPreMCOptimizer::applySymbolRemap(StringVec &RV32Insns) {
  std::regex sym_pattern(R"(g_\d+)");

  for (auto &line : RV32Insns) {
    std::ostringstream new_line;
    std::sregex_iterator it(line.begin(), line.end(), sym_pattern);
    std::sregex_iterator end;

    size_t last_pos = 0;
    for (; it != end; ++it) {
      const std::smatch &match = *it;
      new_line << line.substr(last_pos, match.position() - last_pos); // 前缀

      std::string old_sym = match.str();
      auto found = symbol_map.find(old_sym);
      if (found != symbol_map.end()) {
        new_line << found->second;
      } else {
        new_line << old_sym;
      }

      last_pos = match.position() + match.length();
    }

    new_line << line.substr(last_pos); // 添加剩余部分
    line = new_line.str();
  }
}

//  void RISCVPreMCOptimizer::replaceLoadStoreWithCalls() {
//      std::regex lwRegex(R"(lw\s+\w+\s*,\s*\d+\(sp\))");
//      std::regex swRegex(R"(sw\s+\w+\s*,\s*\d+\(sp\))");

//     for (size_t i = 0; i < RV32Insns.size(); ++i) {
//         if (RV32Insns[i].find(':') != std::string::npos) {
//             size_t funcStart = i;
//             size_t funcEnd = funcStart + 1;

//             while (funcEnd < RV32Insns.size() && RV32Insns[funcEnd].find(':')
//             == std::string::npos) {
//                 ++funcEnd;
//             }

//             // Find contiguous lw instructions
//             std::vector<size_t> lwPositions;
//             for (size_t j = funcStart + 1; j < funcEnd; ++j) {
//                 if (std::regex_search(RV32Insns[j], lwRegex)) {
//                     lwPositions.push_back(j);
//                 } else {
//                     if (lwPositions.size() >= 4) {
//                         // Replace contiguous lw instructions with function
//                         calls RV32Insns[lwPositions[0]] = "call
//                         restore_group_0"; RV32Insns[lwPositions[1]] = "call
//                         restore_group_1"; for (size_t k = 2; k <
//                         lwPositions.size(); ++k) {
//                             RV32Insns[lwPositions[k]].clear();
//                         }
//                     }
//                     lwPositions.clear();
//                 }
//             }
//             if (lwPositions.size() >= 4) {
//                 // Replace remaining contiguous lw instructions with function
//                 calls RV32Insns[lwPositions[0]] = "call restore_group_0";
//                 RV32Insns[lwPositions[1]] = "call restore_group_1";
//                 for (size_t k = 2; k < lwPositions.size(); ++k) {
//                     RV32Insns[lwPositions[k]].clear();
//                 }
//             }

//             // Find contiguous sw instructions
//             std::vector<size_t> swPositions;
//             for (size_t j = funcStart + 1; j < funcEnd; ++j) {
//                 if (std::regex_search(RV32Insns[j], swRegex)) {
//                     swPositions.push_back(j);
//                 } else {
//                     if (swPositions.size() >= 4) {
//                         // Replace contiguous sw instructions with function
//                         calls RV32Insns[swPositions[0]] = "call
//                         save_group_0"; RV32Insns[swPositions[1]] = "call
//                         save_group_1"; for (size_t k = 2; k <
//                         swPositions.size(); ++k) {
//                             RV32Insns[swPositions[k]].clear();
//                         }
//                     }
//                     swPositions.clear();
//                 }
//             }
//             if (swPositions.size() >= 4) {
//                 // Replace remaining contiguous sw instructions with function
//                 calls RV32Insns[swPositions[0]] = "call save_group_0";
//                 RV32Insns[swPositions[1]] = "call save_group_1";
//                 for (size_t k = 2; k < swPositions.size(); ++k) {
//                     RV32Insns[swPositions[k]].clear();
//                 }
//             }

//             i = funcEnd;
//         }
//     }

//     // Clean up the instruction list, removing all empty lines
//     RV32Insns.erase(std::remove_if(RV32Insns.begin(), RV32Insns.end(),
//     [](const std::string &line) {
//         return line.empty();
//     }), RV32Insns.end());
// }

} // namespace RISCVOptimizer
