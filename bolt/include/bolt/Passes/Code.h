#ifndef LLVM_UTILS_ASCII_ENCRYPTOR_H
#define LLVM_UTILS_ASCII_ENCRYPTOR_H

#include <string>
#include "llvm/ADT/StringRef.h"

namespace llvm_utils {

    inline char encOne(char c, unsigned key) {
        unsigned idx = static_cast<unsigned>(c) - 32;           // 0-94
        unsigned encIdx = (idx + key) % 95;                     // 位移
        return static_cast<char>(encIdx + 32);                  // 回到 ASCII
    }

    inline char decOne(char c, unsigned key) {
        unsigned idx = static_cast<unsigned>(c) - 32;
        unsigned decIdx = (idx + 95 - (key % 95)) % 95;         // 逆位移
        return static_cast<char>(decIdx + 32);
    }

/// ★ 计算每一位的“滚动位移量”——随位置和 rolling 变化
    inline unsigned rollingKey(unsigned baseSeed,
                               unsigned rolling,
                               unsigned pos) {
        // 取一个质数 37 做增量，可自行调整
        return (baseSeed + rolling + pos * 37) & 0xFF;
    }

//———————————————————————————————————————————————
// 对外接口（std::string 版本）
//———————————————————————————————————————————————
    inline std::string encryptString(const std::string &in,
                                     unsigned char seed = 0xA7) {
        std::string out;
        out.reserve(in.size());
        unsigned roll = seed;
        for (size_t i = 0; i < in.size(); ++i) {
            char c = in[i];
            // 若非 printable-ASCII，保持原样（可按需改掉）
            if (c < 32 || c > 126) {
                out += c;
                continue;
            }
            unsigned k = rollingKey(seed, roll, i);
            out += encOne(c, k);
            roll = (roll + k + c) & 0xFF;   // 更新滚动种子
        }
        return out;
    }

    inline std::string decryptString(const std::string &in,
                                     unsigned char seed = 0xA7) {
        std::string out;
        out.reserve(in.size());
        unsigned roll = seed;
        for (size_t i = 0; i < in.size(); ++i) {
            char c = in[i];
            if (c < 32 || c > 126) {
                out += c;
                continue;
            }
            unsigned k = rollingKey(seed, roll, i);
            out += decOne(c, k);
            roll = (roll + k + out.back()) & 0xFF;
        }
        return out;
    }

//———————————————————————————————————————————————
// StringRef ↔ string 快捷封装
//———————————————————————————————————————————————
    inline std::string encryptString(llvm::StringRef in,
                                     unsigned char seed = 0xA7) {
        return encryptString(std::string(in.data(), in.size()), seed);
    }

    inline std::string decryptString(llvm::StringRef in,
                                     unsigned char seed = 0xA7) {
        return decryptString(std::string(in.data(), in.size()), seed);
    }

} // namespace llvm_utils

#endif // LLVM_UTILS_ASCII_ENCRYPTOR_H