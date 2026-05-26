#!/usr/bin/env python3
"""
第 1 步：清理 + 平面平滑 FAST-LIVO 输出的全局彩色点云。

输入：FAST-LIVO 的 rgb_map*.ply（已融合已上色）
输出：clean.ply（清理后）和可选 smooth.ply（平面平滑后）

对应 README 第 7、8 节描述但仓库里一直缺失的脚本。

流程:
    读彩色点云 -> 去 NaN -> 体素下采样 -> 统计离群点去除
              -> 半径离群点去除 -> [可选]平面平滑 -> 法线估计 -> 存

用法:
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
    平面平滑：RANSAC 找大平面，只对法线接近 X/Y/Z 轴的平面，
    把附近点投影到平面上。适合室内墙/地/天花板。
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
            print("  平面分割失败:", e)
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
            # 不是轴对齐平面（比如斜面/家具），不动，移出候选
            remaining = np.delete(remaining, inliers)
            continue

        dist = pts[global_idx] @ n + d
        pts[global_idx] = pts[global_idx] - dist[:, None] * n
        snapped_total += len(global_idx)
        print(f"  plane {pid}: 平滑 {len(global_idx)} 点, "
              f"normal={n.round(2)}, axis_dot={axis_dot:.3f}")
        remaining = np.delete(remaining, inliers)

    pcd.points = o3d.utility.Vector3dVector(pts)
    if has_colors:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    print(f"  平面平滑完成，共平滑 {snapped_total} 点")
    return pcd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="FAST-LIVO rgb_map*.ply")
    ap.add_argument("--out_dir", default="outputs/processed")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="体素下采样尺寸（FAST-LIVO 已 3cm 的话可设 0 跳过）")
    ap.add_argument("--stat_nb", type=int, default=20)
    ap.add_argument("--stat_std", type=float, default=2.0)
    ap.add_argument("--radius_nb", type=int, default=16)
    ap.add_argument("--radius", type=float, default=0.10)
    ap.add_argument("--plane_smooth", action="store_true",
                    help="开启平面平滑（室内墙/地推荐，复杂物体不建议）")
    ap.add_argument("--normal_radius_scale", type=float, default=3.0)
    args = ap.parse_args()

    print(f"[1/6] 读取 {args.input}")
    pcd = read_colored_cloud(args.input)
    print(f"      原始点数: {len(pcd.points):,}")

    print("[2/6] 去除 NaN/Inf")
    pcd = pcd.remove_non_finite_points()

    if args.voxel > 0:
        print(f"[3/6] 体素下采样 voxel={args.voxel}")
        pcd = pcd.voxel_down_sample(args.voxel)
        print(f"      下采样后: {len(pcd.points):,}")
    else:
        print("[3/6] 跳过体素下采样")

    print(f"[4/6] 统计离群点去除 nb={args.stat_nb} std={args.stat_std}")
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=args.stat_nb, std_ratio=args.stat_std)
    print(f"      剩余: {len(pcd.points):,}")

    print(f"[4.5] 半径离群点去除 nb={args.radius_nb} r={args.radius}")
    pcd, _ = pcd.remove_radius_outlier(
        nb_points=args.radius_nb, radius=args.radius)
    print(f"      剩余: {len(pcd.points):,}")

    out_dir = Path(args.out_dir)
    voxel_for_normal = args.voxel if args.voxel > 0 else 0.03

    print("[5/6] 法线估计")
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel_for_normal * args.normal_radius_scale, max_nn=30))

    clean_path = out_dir / "global_colored_clean.ply"
    write_cloud(pcd, clean_path)
    print(f"      已存 {clean_path}  (has_color={pcd.has_colors()})")

    if args.plane_smooth:
        print("[6/6] 平面平滑")
        smooth = o3d.geometry.PointCloud(pcd)
        smooth = snap_points_to_axis_planes(smooth)
        smooth.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_for_normal * args.normal_radius_scale, max_nn=30))
        smooth_path = out_dir / "global_colored_smooth.ply"
        write_cloud(smooth, smooth_path)
        print(f"      已存 {smooth_path}")
        print("\n提示：在 CloudCompare 里对比 clean 和 smooth，")
        print("      平滑伤了边缘细节就用 clean 去建网格。")
    else:
        print("[6/6] 跳过平面平滑（未加 --plane_smooth）")

    print("\n完成。下一步：python make_colored_mesh.py --input", clean_path)


if __name__ == "__main__":
    main()
