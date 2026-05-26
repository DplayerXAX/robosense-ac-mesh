#!/usr/bin/env python3
"""
Shared point cloud IO. Primarily solves the problem of FAST-LIVO / PCL
packed-RGB fields not being read by Open3D.

When storing RGB, FAST-LIVO often packs the color into a single float32 field
named `rgb` (24-bit RGB stuffed into the float mantissa). Open3D's
read_point_cloud has inconsistent support for this format: sometimes it returns
has_colors()=False, sometimes the colors come out all black.

read_colored_cloud() here:
  1. Reads normally with Open3D first
  2. If no color is found, falls back to manually parsing the PLY/PCD packed rgb
"""

from pathlib import Path

import numpy as np
import open3d as o3d


def _unpack_rgb_float(rgb_float):
    """Unpack packed float32 RGB into (N,3) colors in 0~1 range."""
    rgb_uint = rgb_float.astype(np.float32).view(np.uint32)
    r = ((rgb_uint >> 16) & 255).astype(np.float64) / 255.0
    g = ((rgb_uint >> 8) & 255).astype(np.float64) / 255.0
    b = (rgb_uint & 255).astype(np.float64) / 255.0
    return np.stack([r, g, b], axis=1)


def _try_manual_pcd_rgb(path):
    """
    Manually parse the packed rgb field of an ASCII PCD.
    Only used as a fallback when Open3D fails to read colors.
    Returns (points, colors) or None.
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
            return None  # no packed rgb, leave it to Open3D

        data_type = header.get("DATA", ["ascii"])[0]
        n_points = int(header.get("POINTS", [0])[0])

        # Simple case: ASCII. For binary, hand xyz back to Open3D; color is separate
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
    Read a point cloud, making a best effort to obtain colors.
    Returns an open3d.geometry.PointCloud.
    """
    path = str(path)
    pcd = o3d.io.read_point_cloud(path)

    has_real_color = pcd.has_colors()
    if has_real_color:
        cols = np.asarray(pcd.colors)
        # all-same / all-black = effectively no color
        if cols.size and (np.allclose(cols.min(0), cols.max(0))
                          or cols.max() < 0.02):
            has_real_color = False

    if not has_real_color:
        if verbose:
            print("  [pc_io] Open3D found no valid color, trying manual packed rgb...")
        manual = _try_manual_pcd_rgb(path)
        if manual is not None:
            pts, colors = manual
            pcd2 = o3d.geometry.PointCloud()
            pcd2.points = o3d.utility.Vector3dVector(pts)
            pcd2.colors = o3d.utility.Vector3dVector(colors)
            if verbose:
                print(f"  [pc_io] Manual parse OK, {len(pts)} colored points.")
            return pcd2
        elif verbose:
            print("  [pc_io] Manual parse failed (likely a binary PCD).")
            print("  [pc_io] Tip: prefer the .ply version, its color fields are more standard.")

    if verbose:
        print(f"  [pc_io] Read {len(pcd.points)} points, has_color={pcd.has_colors()}")
    return pcd


def write_cloud(pcd, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(path), pcd)


def write_mesh(mesh, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(path), mesh)
