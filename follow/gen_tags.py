#!/usr/bin/env python3
"""生成 tagStandard41h12 的 tag 图 —— 借 dt_apriltags 自带的 libapriltag.so。

    /home/ysc/follow_env/bin/python gen_tags.py 1 3 6 18 61 63 68

原理：dt_apriltags 的 Detector 暴露了 ctypes 句柄（.libc），apriltag C 库里
有 apriltag_to_image(fam, idx)，直接调它拿原始模块图，再放大 + 加白色静区。

    apriltag_family_t *apriltag_family_create(const char *famname);
    image_u8_t *apriltag_to_image(apriltag_family_t *fam, int idx);

（本机没装 libapriltag 的话，把本文件传到相机那台机器上跑。）
"""

import argparse
import ctypes
import os
import sys

import cv2
import numpy as np

from dt_apriltags import Detector


def get_lib():
    """拿到 (ctypes 库, family 指针)。

    两个坑：
      · 这个 libapriltag.so 没导出 apriltag_family_create，但导出了
        apriltag_to_image —— family 指针从 Detector.tag_families 拿
        （值是 LP__ApriltagFamily，本身就是 ctypes 指针）。
      · apriltag_to_image 返回的 image_u8_t* 不要用 ctypes 结构体去解，
        字段对齐对不上会段错误（踩过）。改成调库里同样导出的
        image_u8_write_pnm() 落盘，再用 cv2 读回来 —— 绕开整个结构体。
    """
    det = Detector(families="tagStandard41h12")
    lib = det.libc
    lib.apriltag_to_image.restype = ctypes.c_void_p
    lib.apriltag_to_image.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.image_u8_write_pnm.restype = ctypes.c_int
    lib.image_u8_write_pnm.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    fam = det.tag_families["tagStandard41h12"]
    return lib, fam


def raw_grid(lib, fam, idx, tmp="/tmp/_gen_tag.pnm"):
    """返回 tag 的原始模块图（numpy uint8，0/255），无静区。"""
    ptr = lib.apriltag_to_image(ctypes.cast(fam, ctypes.c_void_p), idx)
    if not ptr:
        return None
    if lib.image_u8_write_pnm(ctypes.c_void_p(ptr), tmp.encode()) != 0:
        return None
    a = cv2.imread(tmp, cv2.IMREAD_GRAYSCALE)
    if a is None:
        return None
    return (a > 127).astype(np.uint8) * 255      # 二值化


def render(grid, module_px, quiet_modules):
    """放大 + 加白色静区。静区是解码必需的（白边不够会认不出）。"""
    q = quiet_modules * module_px
    big = cv2.resize(grid, (grid.shape[1] * module_px,
                            grid.shape[0] * module_px),
                     interpolation=cv2.INTER_NEAREST)
    return cv2.copyMakeBorder(big, q, q, q, q, cv2.BORDER_CONSTANT, value=255)


def main():
    ap = argparse.ArgumentParser(description="生成 tagStandard41h12 tag 图")
    ap.add_argument("ids", type=int, nargs="+", help="tag id，如 1 3 6 18")
    ap.add_argument("--module-px", type=int, default=40,
                    help="每个模块多少像素（默认 40）")
    ap.add_argument("--quiet", type=int, default=4,
                    help="静区宽度（模块数，默认 4）")
    ap.add_argument("--out", default=".", help="输出目录")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    lib, fam = get_lib()

    # 自检用的检测器：生成完立刻回读，确认 id 对得上。
    # 不验的话，生成了认不出的图 = 白干，而且要等到上机才发现。
    verifier = Detector(families="tagStandard41h12")

    print(f"{'id':>4}  {'模块网格':>9}  {'输出尺寸':>11}  自检      文件")
    n_ok = 0
    for idx in args.ids:
        g = raw_grid(lib, fam, idx)
        if g is None:
            print(f"{idx:>4}  生成失败（该 id 不在本 family 里？）")
            continue
        out = render(g, args.module_px, args.quiet)
        name = f"tag41_12_{idx:05d}.png"
        path = os.path.join(args.out, name)
        cv2.imwrite(path, out)

        # 自检：把生成的大图拿去检测，看回来的 id 对不对
        res = verifier.detect(out)
        got = sorted(int(r.tag_id) for r in res)
        ok = idx in got
        n_ok += ok
        mark = "✅" if ok else f"❌ 认出 {got}"
        print(f"{idx:>4}  {g.shape[1]}x{g.shape[0]:<6}  "
              f"{out.shape[1]}x{out.shape[0]:<7}  {mark:<9} {path}")

    print(f"\n自检通过 {n_ok}/{len(args.ids)}")
    print("打印注意：")
    print("  · 静区（白边）必须一起打印出来，否则认不出")
    print("  · 所有 tag 打印成【同一尺寸】，方便比较")
    print("  · dt_apriltags 的 tag_size = 黑框外沿边长，不是纸张尺寸")
    return 0 if n_ok == len(args.ids) else 1


if __name__ == "__main__":
    sys.exit(main())
