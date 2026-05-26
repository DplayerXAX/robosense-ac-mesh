#!/usr/bin/env python3
"""
Step 2: reconstruct a mesh from the cleaned colored cloud, keeping vertex colors.

Rewritten from the old make_mesh.py. Key changes:
  1. Parameters tuned for FAST-LIVO 3cm data (old VOXEL=0.08 blurred it again)
  2. Poisson depth defaults to 11 (sharper indoors; old make_mesh used 10, blurry)
  3. Outputs .ply instead of .obj -- PLY stores vertex colors, STL/OBJ awkward
  4. Supports both Poisson and Ball Pivoting
  5. Explicitly ensures colors reach mesh vertices (Poisson usually carries them,
     BPA needs a manual color transfer)

Usage:
    # Poisson (default, recommended indoors)
    python make_colored_mesh.py \
        --input outputs/processed/global_colored_clean.ply \
        --out outputs/mesh/colored_mesh.ply \
        --method poisson --depth 11

    # Ball Pivoting (surface-like clouds, avoids fake faces)
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
    """Transfer cloud colors to mesh vertices via nearest neighbor (needed after BPA)."""
    if not pcd.has_colors():
        print("  cloud has no color, skipping transfer")
        return mesh
    tree = o3d.geometry.KDTreeFlann(pcd)
    pts_colors = np.asarray(pcd.colors)
    verts = np.asarray(mesh.vertices)
    vcolors = np.zeros((len(verts), 3))
    for i, v in enumerate(verts):
        _, idx, _ = tree.search_knn_vector_3d(v, 1)
        vcolors[i] = pts_colors[idx[0]]
    mesh.vertex_colors = o3d.utility.Vector3dVector(vcolors)
    print(f"  transferred color to {len(verts):,} vertices")
    return mesh


def poisson_reconstruct(pcd, depth, density_quantile):
    print(f"  Poisson reconstruction depth={depth} ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, n_threads=-1)
    print(f"  raw mesh: {len(mesh.triangles):,} triangles")

    # crop low-confidence regions by density (Poisson invents faces in sparse areas)
    densities = np.asarray(densities)
    thr = np.quantile(densities, density_quantile)
    mesh.remove_vertices_by_mask(densities < thr)
    print(f"  after density crop: {len(mesh.triangles):,} triangles")
    return mesh


def bpa_reconstruct(pcd, voxel):
    # radii at 2/3/4x point spacing (README section 9.1 suggestion)
    radii = [voxel * k for k in (2, 3, 4)]
    print(f"  Ball Pivoting radii: {radii}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii))
    print(f"  BPA mesh: {len(mesh.triangles):,} triangles")
    return mesh


def clean_mesh(mesh, min_cluster_tris):
    print("  cleaning mesh ...")
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    # drop small fragments
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
    ap.add_argument("--depth", type=int, default=11, help="Poisson depth, 11 for indoors")
    ap.add_argument("--density_quantile", type=float, default=0.05,
                    help="Poisson: crop this fraction of lowest density")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="BPA radius base (approx point spacing)")
    ap.add_argument("--min_cluster_tris", type=int, default=2000)
    ap.add_argument("--simplify", type=int, default=0,
                    help=">0 also outputs a decimated version (for Unity/Unreal), value = target triangle count")
    args = ap.parse_args()

    print(f"[1/4] reading cloud {args.input}")
    pcd = read_colored_cloud(args.input)
    print(f"      points {len(pcd.points):,}, has_color={pcd.has_colors()}")

    if not pcd.has_normals():
        print("      cloud has no normals, estimating...")
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.voxel * 3, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(30)

    print(f"[2/4] {args.method} reconstruction")
    if args.method == "poisson":
        mesh = poisson_reconstruct(pcd, args.depth, args.density_quantile)
    else:
        mesh = bpa_reconstruct(pcd, args.voxel)

    print("[3/4] cleaning + keeping color")
    mesh = clean_mesh(mesh, args.min_cluster_tris)
    # Poisson creates new vertices whose color may be off, so re-transfer to be safe
    mesh = transfer_colors(mesh, pcd)

    print(f"[4/4] saving {args.out}")
    write_mesh(mesh, args.out)
    print(f"      vertices {len(mesh.vertices):,}, faces {len(mesh.triangles):,}, "
          f"has_vertex_color={mesh.has_vertex_colors()}")

    if args.simplify > 0:
        target = min(args.simplify, len(mesh.triangles))
        lite = mesh.simplify_quadric_decimation(target_number_of_triangles=target)
        lite.compute_vertex_normals()
        lite_path = Path(args.out).with_name(Path(args.out).stem + "_lite.ply")
        write_mesh(lite, lite_path)
        print(f"      decimated version {lite_path} ({target:,} faces, for game engines)")

    print("\nDone. View with tools/view.py, or drag into MeshLab/CloudCompare.")


if __name__ == "__main__":
    main()
