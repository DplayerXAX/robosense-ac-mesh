#!/usr/bin/env python3
"""
Diagnose a FAST-LIVO output cloud: whether color is readable, point count,
scale, density.

This is the critical check before building a colored mesh -- Open3D's support
for PCL packed rgb is hit or miss, so confirm color reads first, then mesh.

Usage:
    python diagnose.py PCD/rgb_map_voxel_0.030000.ply
    python diagnose.py PCD/rgb_map.pcd
    # no arg: auto-scan current dir and ./PCD for rgb_*.ply / rgb_*.pcd
"""

import sys
import glob
from pathlib import Path

import numpy as np
import open3d as o3d


def diagnose(path):
    path = Path(path)
    print("=" * 64)
    print(f"file: {path}")
    print(f"size: {path.stat().st_size / 1e6:.1f} MB")
    print("-" * 64)

    pcd = o3d.io.read_point_cloud(str(path))

    n = len(pcd.points)
    print(f"points:      {n:,}")
    print(f"has color:   {pcd.has_colors()}")
    print(f"has normals: {pcd.has_normals()}")

    if n == 0:
        print("WARNING: read 0 points -- Open3D failed to parse this file!")
        print("         Try the .ply version, or convert via PCL.")
        return

    # color check (the critical one)
    if pcd.has_colors():
        cols = np.asarray(pcd.colors)
        cmin = cols.min(axis=0)
        cmax = cols.max(axis=0)
        cmean = cols.mean(axis=0)
        print(f"color range: min={cmin.round(3)} max={cmax.round(3)}")
        print(f"color mean:  {cmean.round(3)}")
        # all 0 or all identical = effectively no color
        if np.allclose(cmin, cmax):
            print("WARNING: all points same color -- effectively no color, field may be misread!")
        elif cmax.max() < 0.02:
            print("WARNING: color almost all black -- packed rgb decode may be wrong!")
        else:
            print("OK: color looks normal.")
    else:
        print("WARNING: Open3D found no color field!")
        print("    If this is rgb_map, the packed rgb was not recognized.")
        print("    Fix: prefer the .ply version; or check field names via PCL header.")

    # scale / density
    bbox = pcd.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()
    print(f"bbox size:   X={extent[0]:.2f}m  Y={extent[1]:.2f}m  Z={extent[2]:.2f}m")

    # estimate mean point spacing (sample 2000 points, nearest neighbor)
    sample_n = min(2000, n)
    idx = np.random.choice(n, sample_n, replace=False)
    pts = np.asarray(pcd.points)
    tree = o3d.geometry.KDTreeFlann(pcd)
    dists = []
    for i in idx:
        _, _, d2 = tree.search_knn_vector_3d(pts[i], 2)  # nearest non-self point
        if len(d2) > 1:
            dists.append(np.sqrt(d2[1]))
    if dists:
        med = np.median(dists)
        print(f"median spacing: {med*100:.2f} cm  (use for BPA radius / Poisson params)")

    print()


def main():
    if len(sys.argv) > 1:
        targets = sys.argv[1:]
    else:
        # auto-find rgb files, prefer ply
        targets = []
        for pat in ("rgb_*.ply", "PCD/rgb_*.ply", "rgb_*.pcd", "PCD/rgb_*.pcd"):
            targets += sorted(glob.glob(pat))
        # dedupe, keep order
        seen = set()
        targets = [t for t in targets if not (t in seen or seen.add(t))]
        if not targets:
            print("No rgb_*.ply / rgb_*.pcd found, pass a path manually:")
            print("  python diagnose.py PCD/rgb_map_voxel_0.030000.ply")
            return

    for t in targets:
        diagnose(t)

    print("=" * 64)
    print("Next: paste the output back, parameters can be tuned from it.")
    print("      Focus on 'has color' and 'median spacing' of rgb_map_voxel_0.030000.ply.")


if __name__ == "__main__":
    main()
