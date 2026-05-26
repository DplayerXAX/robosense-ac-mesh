#!/usr/bin/env python3
"""
第 2 步：从清理后的彩色点云建 mesh，并保留顶点颜色。

重写自旧 make_mesh.py，主要改动:
  1. 参数适配 FAST-LIVO 3cm 数据（旧版 VOXEL=0.08 会再糊一层）
  2. Poisson depth 默认 11（室内更清晰，旧 make_mesh 用 10 偏糊）
  3. 输出 .ply 而非 .obj —— PLY 存顶点色，STL/OBJ 不方便
  4. 支持 Poisson 和 Ball Pivoting 两种算法切换
  5. 显式保证颜色传到 mesh 顶点（Poisson 通常自带，BPA 需手动转色）

用法:
    # Poisson（默认，室内推荐）
    python make_colored_mesh.py \
        --input outputs/processed/global_colored_clean.ply \
        --out outputs/mesh/colored_mesh.ply \
        --method poisson --depth 11

    # Ball Pivoting（表面型点云，不易造假面）
    python make_colored_mesh.py \
        --input outputs/processed/global_colored_clean.ply \
        --method bpa
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).parent))
from pc_io import read_colored_cloud, write_mesh


def transfer_colors(mesh, pcd):
    """把点云颜色用最近邻转到 mesh 顶点（BPA 后需要）。"""
    if not pcd.has_colors():
        print("  点云没颜色，跳过转色")
        return mesh
    tree = o3d.geometry.KDTreeFlann(pcd)
    pts_colors = np.asarray(pcd.colors)
    verts = np.asarray(mesh.vertices)
    vcolors = np.zeros((len(verts), 3))
    for i, v in enumerate(verts):
        _, idx, _ = tree.search_knn_vector_3d(v, 1)
        vcolors[i] = pts_colors[idx[0]]
    mesh.vertex_colors = o3d.utility.Vector3dVector(vcolors)
    print(f"  已转色到 {len(verts):,} 个顶点")
    return mesh


def poisson_reconstruct(pcd, depth, density_quantile):
    print(f"  Poisson 重建 depth={depth} ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, n_threads=-1)
    print(f"  原始 mesh: {len(mesh.triangles):,} 三角面")

    # 按密度裁掉低置信区域（Poisson 在稀疏处会造假面）
    densities = np.asarray(densities)
    thr = np.quantile(densities, density_quantile)
    mesh.remove_vertices_by_mask(densities < thr)
    print(f"  密度裁剪后: {len(mesh.triangles):,} 三角面")
    return mesh


def bpa_reconstruct(pcd, voxel):
    # 半径取点间距的 2/3/4 倍（README 第 9.1 节建议）
    radii = [voxel * k for k in (2, 3, 4)]
    print(f"  Ball Pivoting 半径: {radii}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii))
    print(f"  BPA mesh: {len(mesh.triangles):,} 三角面")
    return mesh


def clean_mesh(mesh, min_cluster_tris):
    print("  清理 mesh ...")
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    # 去掉小碎片
    clusters, n_tris, _ = mesh.cluster_connected_triangles()
    clusters = np.asarray(clusters)
    n_tris = np.asarray(n_tris)
    if len(n_tris):
        mesh.remove_triangles_by_mask(n_tris[clusters] < min_cluster_tris)
        mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--out", default="outputs/mesh/colored_mesh.ply")
    ap.add_argument("--method", choices=["poisson", "bpa"], default="poisson")
    ap.add_argument("--depth", type=int, default=11, help="Poisson 深度，室内 11")
    ap.add_argument("--density_quantile", type=float, default=0.05,
                    help="Poisson 裁掉密度最低的这一比例")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="BPA 半径基准（≈点间距）")
    ap.add_argument("--min_cluster_tris", type=int, default=2000)
    ap.add_argument("--simplify", type=int, default=0,
                    help=">0 则额外输出简化版（Unity/Unreal 用），值为目标面数")
    args = ap.parse_args()

    print(f"[1/4] 读取点云 {args.input}")
    pcd = read_colored_cloud(args.input)
    print(f"      点数 {len(pcd.points):,}, has_color={pcd.has_colors()}")

    if not pcd.has_normals():
        print("      点云无法线，估计法线...")
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.voxel * 3, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)

    print(f"[2/4] {args.method} 重建")
    if args.method == "poisson":
        mesh = poisson_reconstruct(pcd, args.depth, args.density_quantile)
    else:
        mesh = bpa_reconstruct(pcd, args.voxel)

    print("[3/4] 清理 + 保色")
    mesh = clean_mesh(mesh, args.min_cluster_tris)
    # Poisson 顶点是新生成的，颜色不一定准，统一重新转色最稳
    mesh = transfer_colors(mesh, pcd)

    print(f"[4/4] 保存 {args.out}")
    write_mesh(mesh, args.out)
    print(f"      顶点 {len(mesh.vertices):,}, 面 {len(mesh.triangles):,}, "
          f"has_vertex_color={mesh.has_vertex_colors()}")

    if args.simplify > 0:
        target = min(args.simplify, len(mesh.triangles))
        lite = mesh.simplify_quadric_decimation(target_number_of_triangles=target)
        lite.compute_vertex_normals()
        lite_path = Path(args.out).with_name(Path(args.out).stem + "_lite.ply")
        write_mesh(lite, lite_path)
        print(f"      简化版 {lite_path} ({target:,} 面，游戏引擎用)")

    print("\n完成。用 tools/view.py 查看，或拖进 MeshLab/CloudCompare。")


if __name__ == "__main__":
    main()
