import os
os.environ["XDG_SESSION_TYPE"] = "x11"
import open3d as o3d
import numpy as np

poses_file = "results/latest/super_sensor_2026_05_11_14_46_41_poses_kitti.txt"
trajectory = []
with open(poses_file) as f:
    for line in f:
        T = np.array(line.strip().split(), dtype=np.float64).reshape(3, 4)
        trajectory.append(T[:3, 3])
trajectory = np.array(trajectory)
print(f"tra: {len(trajectory)} poses")

pcd = o3d.io.read_point_cloud("merged.ply")
pcd = pcd.voxel_down_sample(0.08)
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
print(f"points: {len(pcd.points)}")

pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.24, max_nn=30))

points = np.asarray(pcd.points)
normals = np.asarray(pcd.normals)

traj_pcd = o3d.geometry.PointCloud()
traj_pcd.points = o3d.utility.Vector3dVector(trajectory)
traj_tree = o3d.geometry.KDTreeFlann(traj_pcd)

for i in range(len(points)):
    _, idx, _ = traj_tree.search_knn_vector_3d(points[i], 1)
    direction = trajectory[idx[0]] - points[i] 
    direction = direction / (np.linalg.norm(direction) + 1e-8)
    if np.dot(normals[i], direction) < 0:        
        normals[i] = -normals[i]

pcd.normals = o3d.utility.Vector3dVector(normals)
print("Done")

# === Poisson ===
print("Poisson reconstruct...")
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd, depth=11, n_threads=-1)

densities = np.asarray(densities)
mesh.remove_vertices_by_mask(densities < np.quantile(densities, 0.05))
mesh.remove_degenerate_triangles()
mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices()
mesh.remove_non_manifold_edges()

clusters, n_tris, _ = mesh.cluster_connected_triangles()
clusters = np.asarray(clusters)
n_tris = np.asarray(n_tris)
mesh.remove_triangles_by_mask(n_tris[clusters] < 5000)
mesh.remove_unreferenced_vertices()
mesh.compute_vertex_normals()

print(f"Mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
o3d.io.write_triangle_mesh("mesh_v2.obj", mesh)
o3d.visualization.draw_geometries([mesh])