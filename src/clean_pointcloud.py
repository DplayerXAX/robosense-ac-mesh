#!/usr/bin/env python3
"""
Step 1: clean + plane-smooth the global colored point cloud from FAST-LIVO.

Input:  FAST-LIVO's rgb_map*.ply (already fused, already colored)
Output: clean.ply (after cleaning) and optional smooth.ply (after plane smoothing)

This is the script described in README sections 7 & 8 that was always missing
from the original repo.

Pipeline:
    read colored cloud -> remove NaN -> voxel downsample -> statistical outlier
                      -> radius outlier -> [optional] plane smoothing -> normals -> save

Usage:
    python clean_pointcloud.py \
        --input PCD/rgb_map_voxel_0.030000.ply \
        --out_dir outputs/processed \
        --voxel 0.03 \
        --plane_smooth
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).parent))
from pc_io import read_colored_cloud, write_cloud


def snap_points_to_axis_planes(pcd, dist_thr=0.05, axis_angle_deg=12.0,
                               min_plane_points=3000, max_planes=8):
    """
    Plane smoothing: RANSAC finds large planes, and for planes whose normal is
    close to the X/Y/Z axis, projects nearby points onto the plane.
    Good for indoor walls / floors / ceilings.
    """
    if len(pcd.points) < min_plane_points:
        return pcd

    pts = np.asarray(pcd.points).copy()
    has_colors = pcd.has_colors()
    colors = np.asarray(pcd.colors).copy() if has_colors else None

    remaining = np.arange(len(pts))
    axes = np.eye(3)
    cos_thr = np.cos(np.deg2rad(axis_angle_deg))
    snapped_total = 0

    for pid in range(max_planes):
        if len(remaining) < min_plane_points:
            break

        sub = o3d.geometry.PointCloud()
        sub.points = o3d.utility.Vector3dVector(pts[remaining])

        try:
            model, inliers = sub.segment_plane(
                distance_threshold=dist_thr, ransac_n=3, num_iterations=1000)
        except Exception as e:
            print("  plane segmentation failed:", e)
            break

        if len(inliers) < min_plane_points:
            break

        a, b, c, d = model
        n = np.array([a, b, c])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            break
        n, d = n / norm, d / norm

        axis_dot = np.max(np.abs(axes @ n))
        global_idx = remaining[np.array(inliers)]

        if axis_dot < cos_thr:
            # not an axis-aligned plane (e.g. slope/furniture); skip, drop from pool
            remaining = np.delete(remaining, inliers)
            continue

        dist = pts[global_idx] @ n + d
        pts[global_idx] = pts[global_idx] - dist[:, None] * n
        snapped_total += len(global_idx)
        print(f"  plane {pid}: smoothed {len(global_idx)} points, "
              f"normal={n.round(2)}, axis_dot={axis_dot:.3f}")
        remaining = np.delete(remaining, inliers)

    pcd.points = o3d.utility.Vector3dVector(pts)
    if has_colors:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    print(f"  plane smoothing done, {snapped_total} points smoothed in total")
    return pcd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="FAST-LIVO rgb_map*.ply")
    ap.add_argument("--out_dir", default="outputs/processed")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="voxel downsample size (set 0 to skip if already 3cm)")
    ap.add_argument("--stat_nb", type=int, default=20)
    ap.add_argument("--stat_std", type=float, default=2.0)
    ap.add_argument("--radius_nb", type=int, default=16)
    ap.add_argument("--radius", type=float, default=0.10)
    ap.add_argument("--plane_smooth", action="store_true",
                    help="enable plane smoothing (good for indoor walls/floors, not for complex objects)")
    ap.add_argument("--normal_radius_scale", type=float, default=3.0)
    args = ap.parse_args()

    print(f"[1/6] reading {args.input}")
    pcd = read_colored_cloud(args.input)
    print(f"      original points: {len(pcd.points):,}")

    print("[2/6] removing NaN/Inf")
    pcd = pcd.remove_non_finite_points()

    if args.voxel > 0:
        print(f"[3/6] voxel downsample voxel={args.voxel}")
        pcd = pcd.voxel_down_sample(args.voxel)
        print(f"      after downsample: {len(pcd.points):,}")
    else:
        print("[3/6] skipping voxel downsample")

    print(f"[4/6] statistical outlier removal nb={args.stat_nb} std={args.stat_std}")
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=args.stat_nb, std_ratio=args.stat_std)
    print(f"      remaining: {len(pcd.points):,}")

    print(f"[4.5] radius outlier removal nb={args.radius_nb} r={args.radius}")
    pcd, _ = pcd.remove_radius_outlier(
        nb_points=args.radius_nb, radius=args.radius)
    print(f"      remaining: {len(pcd.points):,}")

    out_dir = Path(args.out_dir)
    voxel_for_normal = args.voxel if args.voxel > 0 else 0.03

    print("[5/6] estimating normals")
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel_for_normal * args.normal_radius_scale, max_nn=30))

    clean_path = out_dir / "global_colored_clean.ply"
    write_cloud(pcd, clean_path)
    print(f"      saved {clean_path}  (has_color={pcd.has_colors()})")

    if args.plane_smooth:
        print("[6/6] plane smoothing")
        smooth = o3d.geometry.PointCloud(pcd)
        smooth = snap_points_to_axis_planes(smooth)
        smooth.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_for_normal * args.normal_radius_scale, max_nn=30))
        smooth_path = out_dir / "global_colored_smooth.ply"
        write_cloud(smooth, smooth_path)
        print(f"      saved {smooth_path}")
        print("\nTip: compare clean vs smooth in CloudCompare.")
        print("     If smoothing hurts edge detail, mesh from clean instead.")
    else:
        print("[6/6] skipping plane smoothing (no --plane_smooth)")

    print("\nDone. Next: python make_colored_mesh.py --input", clean_path)


if __name__ == "__main__":
    main()
