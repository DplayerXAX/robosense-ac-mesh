#!/usr/bin/env python3
"""
View a point cloud or mesh.

Usage:
    python view.py outputs/processed/global_colored_clean.ply
    python view.py outputs/mesh/colored_mesh.ply
"""

import os
import sys

os.environ.setdefault("XDG_SESSION_TYPE", "x11")

import open3d as o3d


def main():
    if len(sys.argv) < 2:
        print("usage: python view.py <file.ply>")
        return

    path = sys.argv[1]

    # try reading as mesh first; if no faces, treat as point cloud
    mesh = o3d.io.read_triangle_mesh(path)
    if len(mesh.triangles) > 0:
        print(f"Mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} faces")
        print(f"vertex colors: {mesh.has_vertex_colors()}")
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([mesh])
        return

    pcd = o3d.io.read_point_cloud(path)
    print(f"Point cloud: {len(pcd.points):,} points")
    print(f"color: {pcd.has_colors()}  normals: {pcd.has_normals()}")
    bbox = pcd.get_axis_aligned_bounding_box()
    print(f"bbox: {bbox.get_extent().round(2)}")
    o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    main()
