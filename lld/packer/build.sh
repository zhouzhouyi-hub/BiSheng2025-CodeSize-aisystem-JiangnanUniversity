#!/usr/bin/env bash
set -euo pipefail

CLANG=../../build2/bin/clang
TRIPLE="--target=riscv32 -mabi=ilp32d"
OUT=out

# 输入 payload 及其源目录，文件大的我们才考虑！
PAYLOADS=(
  "../bm1/bm1.final.out"
#  "../bm2/bm2.final.out" # 打开汇编器第二题进行循环优化时不建议开启此功能!
#  "../bm3/qrduino.final"
  "../4743/rnd.final.out"
#  "../bm4/huffbench.final.out"
#  "../bm5/sglib-combined.final.out"
)

# 清理环境，重建输出目录
rm -rf "$OUT" && mkdir -p "$OUT"

echo "==> 1) 构建 stub_min → stub.elf"
"$CLANG" $TRIPLE -Os -ffreestanding -nostdlib -fno-pie -fno-pic \
    -Wl,-T,src/linker.ld \
    -o "$OUT/stub.elf" src/stub_min.c
    # 无用：-no-pie

# 保证 payloads 子目录存在
mkdir -p payloads

# 循环处理每个 payload
for SRC in "${PAYLOADS[@]}"; do
  FNAME=$(basename "$SRC")                  # e.g. bm1.final.out or qrduino.final
  BASENAME=${FNAME/.final*/}                # e.g. bm1 or qrduino
  PACKNAME=${FNAME/.final/.pack}             # e.g. bm1.pack.out or qrduino.pack

  echo
  echo "==> 处理 $SRC"
  echo "    → 拷贝到 payloads/$FNAME"
  cp "$SRC" "payloads/$FNAME"

  echo "    → 调用 pack.py 生成 out/packed.elf"
  python3 pack.py "$OUT/stub.elf" "payloads/$FNAME" "$OUT/packed.elf"

  echo "    → 编译 loader + hdr + payload → out/$PACKNAME"
  "$CLANG" $TRIPLE -Os -ffreestanding -nostdlib  \
      -static -fno-pie -fno-pic \
      -Wl,-T,src/linker.ld \
      src/loader.c "$OUT/hdr.c" "$OUT/payload.c" \
      -o "$OUT/$PACKNAME"
#      -Wl,-z,max-page-size=4 -Wl,--no-rosegment 会导致段错！
# -nostartfiles -no-pie 没必要的代码

  echo "    → strip unneeded symbols and sections"
  ../../build2/bin/llvm-strip --strip-unneeded "$OUT/$PACKNAME"    # 去掉未用符号和调试区 citeturn0search0

  echo "    → compress .text and .data sections"
  ../../build2/bin/llvm-objcopy \
    --strip-all --strip-sections --remove-section=.gnu.stack --remove-section=.riscv.attributes --remove-section=.note.gnu.build-id \
    "$OUT/$PACKNAME"                                            # 压缩代码和数据节 citeturn0search1

  echo "    → 拷回到 $(dirname "$SRC")/$PACKNAME"
  cp "$OUT/$PACKNAME" "$(dirname "$SRC")/$PACKNAME"
done

echo
echo "全部完成："
ls -lh out
