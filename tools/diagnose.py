#!/usr/bin/env python3
"""
诊断 FAST-LIVO 输出的点云：颜色读不读得到、点数、尺度、密度。

这是建彩色 mesh 前的命门检查 —— Open3D 对 PCL 的 packed rgb 字段
兼容性时好时坏，先确认颜色能读到，再谈建网格。

用法:
    python diagnose_pointcloud.py PCD/rgb_map_voxel_0.030000.ply
    python diagnose_pointcloud.py PCD/rgb_map.pcd
    # 不传参数则自动扫描当前目录和 ./PCD 下的 rgb_*.ply / rgb_*.pcd
"""

import sys
import glob
from pathlib import Path

import numpy as np
import open3d as o3d


def diagnose(path):
    path = Path(path)
    print("=" * 64)
    print(f"文件: {path}")
    print(f"大小: {path.stat().st_size / 1e6:.1f} MB")
    print("-" * 64)

    pcd = o3d.io.read_point_cloud(str(path))

    n = len(pcd.points)
    print(f"点数:        {n:,}")
    print(f"有颜色:      {pcd.has_colors()}")
    print(f"有法线:      {pcd.has_normals()}")

    if n == 0:
        print("⚠️  读到 0 个点 —— Open3D 没能解析这个文件！")
        print("    建议改用 .ply 版本，或用 PCL 转一道。")
        return

    # 颜色检查（命门）
    if pcd.has_colors():
        cols = np.asarray(pcd.colors)
        cmin = cols.min(axis=0)
        cmax = cols.max(axis=0)
        cmean = cols.mean(axis=0)
        print(f"颜色范围:    min={cmin.round(3)} max={cmax.round(3)}")
        print(f"颜色均值:    {cmean.round(3)}")
        # 全 0 或全相同 = 实际上没颜色
        if np.allclose(cmin, cmax):
            print("⚠️  所有点颜色相同 —— 等于没颜色，可能颜色字段没读对！")
        elif cmax.max() < 0.02:
            print("⚠️  颜色几乎全黑 —— 可能 packed rgb 解码错误！")
        else:
            print("✅ 颜色看起来正常。")
    else:
        print("⚠️  Open3D 没读到颜色字段！")
        print("    如果这是 rgb_map，说明 packed rgb 没被识别。")
        print("    解决：优先用 .ply 版本；或用下面的 PCL header 检查字段名。")

    # 尺度 / 密度
    bbox = pcd.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()
    print(f"包围盒尺寸:  X={extent[0]:.2f}m  Y={extent[1]:.2f}m  Z={extent[2]:.2f}m")

    # 估算平均点间距（采样 2000 点算最近邻）
    sample_n = min(2000, n)
    idx = np.random.choice(n, sample_n, replace=False)
    pts = np.asarray(pcd.points)
    tree = o3d.geometry.KDTreeFlann(pcd)
    dists = []
    for i in idx:
        _, _, d2 = tree.search_knn_vector_3d(pts[i], 2)  # 最近的非自身点
        if len(d2) > 1:
            dists.append(np.sqrt(d2[1]))
    if dists:
        med = np.median(dists)
        print(f"中位点间距:  {med*100:.2f} cm  （建网格的 BPA 半径/Poisson 参数据此定）")

    print()


def main():
    if len(sys.argv) > 1:
        targets = sys.argv[1:]
    else:
        # 自动找 rgb 文件，优先 ply
        targets = []
        for pat in ("rgb_*.ply", "PCD/rgb_*.ply", "rgb_*.pcd", "PCD/rgb_*.pcd"):
            targets += sorted(glob.glob(pat))
        # 去重保序
        seen = set()
        targets = [t for t in targets if not (t in seen or seen.add(t))]
        if not targets:
            print("没找到 rgb_*.ply / rgb_*.pcd，请手动传入路径：")
            print("  python diagnose_pointcloud.py PCD/rgb_map_voxel_0.030000.ply")
            return

    for t in targets:
        diagnose(t)

    print("=" * 64)
    print("下一步：把上面的输出贴回来，我据此配建网格参数。")
    print("       重点看 rgb_map_voxel_0.030000.ply 的『有颜色』和『中位点间距』。")


if __name__ == "__main__":
    main()
