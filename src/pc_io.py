#!/usr/bin/env python3
"""
公用点云读写。重点解决 FAST-LIVO / PCL 输出的 packed RGB 字段
被 Open3D 读不出来的问题。

FAST-LIVO 存 RGB 时常把颜色打包进一个名为 `rgb` 的 float32 字段
（24-bit RGB 塞进 float 的尾数），Open3D 的 read_point_cloud 对
这种格式兼容性时好时坏：有时返回 has_colors()=False，有时颜色全黑。

本模块的 read_colored_cloud() 会：
  1. 先用 Open3D 正常读
  2. 如果没读到颜色，回退到手动解析 PLY/PCD 的 packed rgb
"""

from pathlib import Path

import numpy as np
import open3d as o3d


def _unpack_rgb_float(rgb_float):
    """把 packed float32 RGB 解成 (N,3) 的 0~1 颜色。"""
    rgb_uint = rgb_float.astype(np.float32).view(np.uint32)
    r = ((rgb_uint >> 16) & 255).astype(np.float64) / 255.0
    g = ((rgb_uint >> 8) & 255).astype(np.float64) / 255.0
    b = (rgb_uint & 255).astype(np.float64) / 255.0
    return np.stack([r, g, b], axis=1)


def _try_manual_pcd_rgb(path):
    """
    手动解析 ASCII / binary PCD 的 packed rgb 字段。
    只在 Open3D 没读到颜色时作为回退。返回 (points, colors) 或 None。
    """
    path = Path(path)
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            header_lines.append(line.decode("ascii", errors="ignore").strip())
            if header_lines[-1].startswith("DATA"):
                break
            if len(header_lines) > 50:
                return None

        header = {}
        for ln in header_lines:
            parts = ln.split()
            if parts:
                header[parts[0]] = parts[1:]

        fields = header.get("FIELDS", [])
        if "rgb" not in fields and "rgba" not in fields:
            return None  # 没有 packed rgb，交给 Open3D

        data_type = header.get("DATA", ["ascii"])[0]
        n_points = int(header.get("POINTS", [0])[0])

        # 简单情形：ASCII。binary 的话直接交回 Open3D 处理 xyz，颜色另说
        if data_type != "ascii":
            return None

        rgb_idx = fields.index("rgb") if "rgb" in fields else fields.index("rgba")
        xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")

        pts = np.empty((n_points, 3), dtype=np.float64)
        rgb_packed = np.empty(n_points, dtype=np.float64)

        for i in range(n_points):
            vals = f.readline().split()
            if not vals:
                break
            pts[i] = (float(vals[xi]), float(vals[yi]), float(vals[zi]))
            rgb_packed[i] = float(vals[rgb_idx])

        colors = _unpack_rgb_float(rgb_packed)
        return pts, colors

    return None


def read_colored_cloud(path, verbose=True):
    """
    读取点云，尽最大努力拿到颜色。
    返回 open3d.geometry.PointCloud。
    """
    path = str(path)
    pcd = o3d.io.read_point_cloud(path)

    has_real_color = pcd.has_colors()
    if has_real_color:
        cols = np.asarray(pcd.colors)
        # 颜色全相同/全黑 = 实际没读到
        if cols.size and (np.allclose(cols.min(0), cols.max(0))
                          or cols.max() < 0.02):
            has_real_color = False

    if not has_real_color:
        if verbose:
            print(f"  [pc_io] Open3D 没读到有效颜色，尝试手动解 packed rgb...")
        manual = _try_manual_pcd_rgb(path)
        if manual is not None:
            pts, colors = manual
            pcd2 = o3d.geometry.PointCloud()
            pcd2.points = o3d.utility.Vector3dVector(pts)
            pcd2.colors = o3d.utility.Vector3dVector(colors)
            if verbose:
                print(f"  [pc_io] 手动解析成功，{len(pts)} 点带颜色。")
            return pcd2
        elif verbose:
            print(f"  [pc_io] 手动解析未成功（可能是 binary PCD）。")
            print(f"  [pc_io] 建议：优先用 .ply 版本，PLY 的颜色字段更标准。")

    if verbose:
        print(f"  [pc_io] 读到 {len(pcd.points)} 点, has_color={pcd.has_colors()}")
    return pcd


def write_cloud(pcd, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(path), pcd)


def write_mesh(mesh, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(path), mesh)
