import os
os.environ["XDG_SESSION_TYPE"] = "x11"
import open3d as o3d
import numpy as np

# === 读位姿，提取轨迹 ===
poses_file = "results/latest/super_sensor_2026_05_11_14_46_41_poses_kitti.txt"
trajectory = []
with open(poses_file) as f:
    for line in f:
        T = np.array(line.strip().split(), dtype=np.float64).reshape(3, 4)
        trajectory.append(T[:3, 3])
trajectory = np.array(trajectory)
print(f"轨迹: {len(trajectory)} 个位姿")

# === 读点云 ===
pcd = o3d.io.read_point_cloud("merged.ply")
pcd = pcd.voxel_down_sample(0.08)
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
print(f"点数: {len(pcd.points)}")

# === 估计法向量 ===
pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.24, max_nn=30))

# === 对每个点，找最近的轨迹点，让法向量朝向它 ===
print("方向化法向量（朝向最近的扫描位置）...")
points = np.asarray(pcd.points)
normals = np.asarray(pcd.normals)

# 用 KDTree 找每个点最近的轨迹点
traj_pcd = o3d.geometry.PointCloud()
traj_pcd.points = o3d.utility.Vector3dVector(trajectory)
traj_tree = o3d.geometry.KDTreeFlann(traj_pcd)

for i in range(len(points)):
    _, idx, _ = traj_tree.search_knn_vector_3d(points[i], 1)
    direction = trajectory[idx[0]] - points[i]   # 从点指向轨迹
    direction = direction / (np.linalg.norm(direction) + 1e-8)
    if np.dot(normals[i], direction) < 0:        # 法向量和方向反了
        normals[i] = -normals[i]

pcd.normals = o3d.utility.Vector3dVector(normals)
print("完成")

# === Poisson ===
print("Poisson 重建...")
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

print(f"Mesh: {len(mesh.vertices)} 顶点, {len(mesh.triangles)} 三角形")
o3d.io.write_triangle_mesh("mesh_v2.obj", mesh)
o3d.visualization.draw_geometries([mesh])