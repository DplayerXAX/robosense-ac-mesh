import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("merged.ply")
print(f"points: {len(pcd.points)}")
print(f"bounding box: {pcd.get_axis_aligned_bounding_box()}")
print(f"bounding box size: {pcd.get_axis_aligned_bounding_box().get_extent()}")
print(f"has color: {pcd.has_colors()}")
print(f"has normal: {pcd.has_normals()}")

o3d.visualization.draw_geometries([pcd])