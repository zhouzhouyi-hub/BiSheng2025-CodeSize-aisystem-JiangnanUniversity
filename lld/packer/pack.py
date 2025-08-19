#!/usr/bin/env python3
import sys, os, importlib

# ─── 1) 把 vendor 放到最前面，屏蔽系统安装的 zopfli ──────────────────────────────
vendor = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vendor")
sys.path.insert(0, vendor)

# ─── 2) 预设最保底的 compress_payload（内置 zlib）───────────────────────────────
import zlib
def compress_payload(data: bytes) -> bytes:
    return zlib.compress(data, level=zlib.Z_BEST_COMPRESSION)

# ─── 3) 尝试挂载 Zopfli C-extension 接口（monkey-patch）────────────────────────
try:
    ext = importlib.import_module("zopfli.zopfli")
    import zopfli  # now picks up vendor/zopfli first
    # 如果有 ZopfliCompressor，就挂上
    if hasattr(ext, "ZopfliCompressor"):
        zopfli.ZopfliCompressor = ext.ZopfliCompressor
    # 如果有 compress，就挂上
    if hasattr(ext, "compress"):
        zopfli.compress = ext.compress
    # 常量
    zopfli.ZOPFLI_FORMAT_ZLIB = getattr(ext, "ZOPFLI_FORMAT_ZLIB", 1)
    print("✅ 使用了 Zopfli C-extension")
except ImportError:
    print("⚠️ 无法加载 Zopfli C-extension，后续使用纯 Python 或 zlib 回退")

# ─── 4) 根据可用接口，重新绑定 compress_payload ────────────────────────
try:
    import zopfli
    print("zopfli from:", zopfli.__file__)

    # 4.1) zopflipy C 扩展提供的类接口
    if hasattr(zopfli, 'ZopfliCompressor'):
        def compress_payload(data: bytes) -> bytes:
            comp = zopfli.ZopfliCompressor(
                zopfli.ZOPFLI_FORMAT_ZLIB, numiterations=30
            )
            return comp.compress(data) + comp.flush()
        print("🏎️ Using ZopfliCompressor (C extension)")

    # 4.2) python-zopfli C 扩展或纯 Python pyzopfli.zlib，顶层 compress()
    elif hasattr(zopfli, 'compress'):
        def compress_payload(data: bytes) -> bytes:
            try:
                # C 扩展签名: compress(data, format, numiterations)
                return zopfli.compress(data, zopfli.ZOPFLI_FORMAT_ZLIB, 60) # 理论越大越好1000能提高0.06分
            except TypeError:
                # 纯 Python 签名: compress(data, numiterations)
                return zopfli.compress(data, 60)
        print("🚀 Using zopfli.compress")

    # 4.3) vendor/zopfli 里只有 zlib.py，没有顶层 compress
    else:
        try:
            from zopfli.zlib import compress as _zlib_compress
            def compress_payload(data: bytes) -> bytes:
                return _zlib_compress(data, 60)
            print("🐢 Using pure-Python zopfli.zlib.compress")
        except ImportError:
            # 如果此处也失败，就让下面的 except ImportError 走系统 zlib
            raise ImportError("no zopfli.compress and no zopfli.zlib")

except ImportError:
    # 4.4) 全部 zopfli 接口都不可用，回退到系统 zlib
    import zlib
    def compress_payload(data: bytes) -> bytes:
        return zlib.compress(data, level=zlib.Z_BEST_COMPRESSION)
    print("⚠️ zopfli 全部不可用，回退到内置 zlib")

# ─── 5) 其余脚本逻辑 ─────────────────────────────────────────────────────────────

import struct, pathlib
from elftools.elf.elffile import ELFFile

# 参数检查
if len(sys.argv) != 4:
    print("usage: pack.py stub.elf orig.elf packed.elf")
    sys.exit(1)

stub_path, orig_path, packed_path = sys.argv[1:4]
out_dir = os.path.dirname(packed_path) or '.'
os.makedirs(out_dir, exist_ok=True)

# 1) 读取 stub
stub_data = pathlib.Path(stub_path).read_bytes()

# 2) 构建 header
with open(orig_path, "rb") as f:
    elf = ELFFile(f)
    entry = elf.header["e_entry"]
    loads = [seg for seg in elf.iter_segments() if seg["p_type"] == "PT_LOAD"]

MAGIC = 0x52565058
hdr = struct.pack("<III", MAGIC, entry, len(loads))
for seg in loads:
    hdr += struct.pack(
        "<IIIIII",
        seg["p_vaddr"], seg["p_filesz"], seg["p_memsz"],
        seg["p_flags"], seg["p_align"], seg["p_offset"]
    )

# 3) 压缩 payload
orig_bytes = pathlib.Path(orig_path).read_bytes()
payload = compress_payload(orig_bytes)

# 4) 替换 marker
marker = b"\xDE\xAD\xBE\xEF"
marker_pos = stub_data.find(marker)
if marker_pos == -1:
    sys.exit("Error: Marker 0xDEADBEEF not found")
stub_data = (
        stub_data[:marker_pos]
        + struct.pack("<I", len(hdr))
        + stub_data[marker_pos+4:]
)

# 5) 写入最终文件
with open(packed_path, "wb") as f:
    f.write(stub_data + hdr + payload)

# 6) 生成二进制文件
hdr_bin = os.path.join(out_dir, "hdr.bin")
payload_bin = os.path.join(out_dir, "payload.bin")
with open(hdr_bin,     "wb") as f: f.write(hdr)
with open(payload_bin, "wb") as f: f.write(payload)

# 7) 生成 C 文件
def generate_c_file(bin_path, c_path, symbol):
    symbol_base = f"_binary_{symbol}"
    data        = pathlib.Path(bin_path).read_bytes()
    with open(c_path, "w") as f:
        f.write('#include <stddef.h>\n')
        f.write(f'__attribute__((used, aligned(1), visibility("default"))) '
                f'unsigned char {symbol_base}_start[] = {{\n')
        for i, b in enumerate(data):
            if i % 12 == 0:
                f.write("  ")
            f.write(f'0x{b:02x}, ')
            if (i+1) % 12 == 0:
                f.write("\n")
        f.write('\n};\n')
        f.write(
            f'__attribute__((used, visibility("default"))) '
            f'unsigned int {symbol_base}_len = {len(data)};\n'
        )

generate_c_file(hdr_bin,     os.path.join(out_dir, "hdr.c"),     "hdr_bin")
generate_c_file(payload_bin, os.path.join(out_dir, "payload.c"), "payload_bin")

print(f"Pack successful → {packed_path}")
