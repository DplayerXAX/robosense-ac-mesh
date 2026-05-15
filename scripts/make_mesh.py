import open3d as o3d
import numpy as np
import os

os.environ["XDG_SESSION_TYPE"] = "x11"  

INPUT = "merged.ply"
OUTPUT = "mesh.obj"
VOXEL_SIZE = 0.08
POISSON_DEPTH = 10      
MIN_CLUSTER_TRIS = 5000

print("[1/6] read point cloud...")
pcd = o3d.io.read_point_cloud(INPUT)
print(f"  original point: {len(pcd.points):,}")

print("[2/6] sample + remove noise...")
pcd = pcd.voxel_down_sample(VOXEL_SIZE)
print(f"  down smaple: {len(pcd.points):,}")
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
print(f" remove noise: {len(pcd.points):,}")

print("[3/6] estimate normals...")
pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
    radius=VOXEL_SIZE * 3, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(k=30)

print(f"[4/6] Poisson rebuild (depth={POISSON_DEPTH})...")
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd, depth=POISSON_DEPTH, n_threads=-1)
print(f"  original mesh: {len(mesh.triangles):,} triangles")

print("[5/6] post processing...")
densities = np.asarray(densities)
mesh.remove_vertices_by_mask(densities < np.quantile(densities, 0.05))

mesh.remove_degenerate_triangles()
mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices()
mesh.remove_non_manifold_edges()

clusters, n_tris, _ = mesh.cluster_connected_triangles()
clusters = np.asarray(clusters)
n_tris = np.asarray(n_tris)
mesh.remove_triangles_by_mask(n_tris[clusters] < MIN_CLUSTER_TRIS)
mesh.remove_unreferenced_vertices()

mesh.compute_vertex_normals()
print(f"  Final: {len(mesh.vertices):,} vertex, {len(mesh.triangles):,} triangles")

print("[6/6] Save...")
o3d.io.write_triangle_mesh(OUTPUT, mesh)
print(f"  → {OUTPUT}")

target_tris = min(200000, len(mesh.triangles))
mesh_lite = mesh.simplify_quadric_decimation(target_number_of_triangles=target_tris)
mesh_lite.compute_vertex_normals()
o3d.io.write_triangle_mesh("mesh_lite.obj", mesh_lite)
print(f"  → mesh_lite.obj ({target_tris:,} triangles, for Unity/Unreal)")

vis = o3d.visualization.Visualizer()
vis.create_window(visible=False, width=1920, height=1080)
vis.add_geometry(mesh)
vis.poll_events()
vis.update_renderer()
vis.capture_screen_image("mesh_preview.png", do_render=True)
vis.destroy_window()
print("  → mesh_preview.png")

print("\nFinish")