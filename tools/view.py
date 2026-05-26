#!/usr/bin/env python3
"""
查看点云或 mesh。

用法:
    python view.py outputs/processed/global_colored_clean.ply
    python view.py outputs/mesh/colored_mesh.ply
"""

import os
import sys

os.environ.setdefault("XDG_SESSION_TYPE", "x11")

import open3d as o3d


def main():
    if len(sys.argv) < 2:
        print("用法: python view.py <file.ply>")
        return

    path = sys.argv[1]
    low = path.lower()

    # 先按 mesh 试读，没有面再当点云
    mesh = o3d.io.read_triangle_mesh(path)
    if len(mesh.triangles) > 0:
        print(f"Mesh: {len(mesh.vertices):,} 顶点, {len(mesh.triangles):,} 面")
        print(f"顶点色: {mesh.has_vertex_colors()}")
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([mesh])
        return

    pcd = o3d.io.read_point_cloud(path)
    print(f"点云: {len(pcd.points):,} 点")
    print(f"颜色: {pcd.has_colors()}  法线: {pcd.has_normals()}")
    bbox = pcd.get_axis_aligned_bounding_box()
    print(f"包围盒: {bbox.get_extent().round(2)}")
    o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    main()
