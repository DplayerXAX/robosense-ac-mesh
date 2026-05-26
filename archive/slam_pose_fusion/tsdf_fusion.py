import os
import argparse
import numpy as np
import open3d as o3d
from tqdm import tqdm


def load_poses_txt(pose_file):
    """
    poses.txt format:
    frame_000000.ply r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz 0 0 0 1

    Returns:
        dict: filename -> 4x4 numpy matrix
    """
    poses = {}

    with open(pose_file, "r") as f:
        lines = f.readlines()

    for line_idx, line in enumerate(lines):
        line = line.strip()
        if not line or line.startswith("#"):
            continue

        parts = line.split()
        if len(parts) != 17:
            raise ValueError(
                f"Line {line_idx + 1} format error. "
                f"Expected 17 columns: filename + 16 matrix values, got {len(parts)}"
            )

        filename = parts[0]
        values = list(map(float, parts[1:]))
        T = np.array(values, dtype=np.float64).reshape(4, 4)
        poses[filename] = T

    return poses


def pointcloud_to_rgbd_from_world_pcd(
    pcd_world,
    T_sensor_to_world,
    width=1280,
    height=720,
    fx=700.0,
    fy=700.0,
    cx=640.0,
    cy=360.0,
    depth_scale=1000.0,
    depth_trunc=20.0,
):
    """
    Convert one LiDAR-frame colored point cloud into a virtual RGBD image,
    so it can be integrated into Open3D TSDF.

    Important assumption:
        The input PLY point cloud is in LiDAR local frame, not world frame.

    LiDAR coordinate convention assumed here:
        lidar_x = forward / depth
        lidar_y = left/right
        lidar_z = up

    Virtual camera coordinate convention used for Open3D RGBD:
        cam_x = right
        cam_y = down
        cam_z = forward / depth

    Conversion:
        cam_x = -lidar_y
        cam_y = -lidar_z
        cam_z =  lidar_x
    """

    points_lidar = np.asarray(pcd_world.points)

    if len(points_lidar) == 0:
        return None, None

    if pcd_world.has_colors():
        colors = np.asarray(pcd_world.colors)
    else:
        colors = np.ones_like(points_lidar) * 0.7

    points_sensor = np.empty_like(points_lidar)

    # camera right  = - lidar left/right
    points_sensor[:, 0] = -points_lidar[:, 1]

    # camera down   = - lidar up
    points_sensor[:, 1] = -points_lidar[:, 2]

    # camera depth  = lidar forward
    points_sensor[:, 2] = points_lidar[:, 0]

    x = points_sensor[:, 0]
    y = points_sensor[:, 1]
    z = points_sensor[:, 2]

    # z is now real depth
    valid = np.isfinite(points_sensor).all(axis=1)
    valid &= z > 0.1
    valid &= z < depth_trunc

    x = x[valid]
    y = y[valid]
    z = z[valid]
    c = colors[valid]

    if len(z) == 0:
        return None, None

    # Project to virtual pinhole image
    u = np.round((fx * x / z) + cx).astype(np.int32)
    v = np.round((fy * y / z) + cy).astype(np.int32)

    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)

    u = u[inside]
    v = v[inside]
    z = z[inside]
    c = c[inside]

    if len(z) == 0:
        return None, None

    depth = np.zeros((height, width), dtype=np.float32)
    color = np.zeros((height, width, 3), dtype=np.uint8)

    # Z-buffer:
    # If multiple points project to the same pixel,
    # keep the closest one.
    for ui, vi, zi, ci in zip(u, v, z, c):
        old_z = depth[vi, ui]
        if old_z == 0 or zi < old_z:
            depth[vi, ui] = zi
            color[vi, ui] = np.clip(ci * 255.0, 0, 255).astype(np.uint8)

    depth_o3d = o3d.geometry.Image((depth * depth_scale).astype(np.uint16))
    color_o3d = o3d.geometry.Image(color)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=depth_scale,
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=False,
    )

    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width,
        height,
        fx,
        fy,
        cx,
        cy,
    )

    return rgbd, intrinsic


def clean_mesh(mesh, keep_largest=True, smooth_iter=0):
    print("[INFO] Cleaning mesh...")

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()

    if keep_largest:
        triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)

        if len(cluster_n_triangles) > 0:
            largest_cluster_idx = cluster_n_triangles.argmax()
            triangles_to_remove = triangle_clusters != largest_cluster_idx
            mesh.remove_triangles_by_mask(triangles_to_remove)
            mesh.remove_unreferenced_vertices()

    if smooth_iter > 0:
        mesh = mesh.filter_smooth_simple(number_of_iterations=smooth_iter)

    mesh.compute_vertex_normals()
    return mesh


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--ply_dir", required=True, help="Directory containing per-frame colored PLY files")
    parser.add_argument("--pose_file", required=True, help="poses.txt file")
    parser.add_argument("--out_mesh", default="tsdf_mesh.ply", help="Output mesh path")

    parser.add_argument("--voxel_size", type=float, default=0.02, help="TSDF voxel size in meters")
    parser.add_argument("--sdf_trunc", type=float, default=0.08, help="TSDF truncation distance in meters")

    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fx", type=float, default=700.0)
    parser.add_argument("--fy", type=float, default=700.0)
    parser.add_argument("--cx", type=float, default=640.0)
    parser.add_argument("--cy", type=float, default=360.0)

    parser.add_argument("--depth_trunc", type=float, default=20.0)
    parser.add_argument("--max_frames", type=int, default=-1)
    parser.add_argument("--smooth_iter", type=int, default=0)

    args = parser.parse_args()

    print("[INFO] Loading poses...")
    poses = load_poses_txt(args.pose_file)

    filenames = sorted([
        f for f in os.listdir(args.ply_dir)
        if f.lower().endswith(".ply")
    ])

    filenames = [f for f in filenames if f in poses]

    if args.max_frames > 0:
        filenames = filenames[:args.max_frames]

    print(f"[INFO] Found {len(filenames)} frames with poses.")

    if len(filenames) == 0:
        raise RuntimeError("No matching PLY frames and poses found.")

    print("[INFO] Creating TSDF volume...")
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel_size,
        sdf_trunc=args.sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    for filename in tqdm(filenames):
        ply_path = os.path.join(args.ply_dir, filename)
        T_sensor_to_world = poses[filename]

        pcd = o3d.io.read_point_cloud(ply_path)

        if len(pcd.points) == 0:
            print(f"[WARN] Empty point cloud: {filename}")
            continue

        rgbd, intrinsic = pointcloud_to_rgbd_from_world_pcd(
            pcd_world=pcd,
            T_sensor_to_world=T_sensor_to_world,
            width=args.width,
            height=args.height,
            fx=args.fx,
            fy=args.fy,
            cx=args.cx,
            cy=args.cy,
            depth_trunc=args.depth_trunc,
        )

        if rgbd is None:
            print(f"[WARN] Failed to create RGBD from: {filename}")
            continue

        # Open3D integrate expects extrinsic = camera-to-world inverse convention?
        # In most Open3D examples, extrinsic is camera pose inverse: world-to-camera.
        extrinsic = np.linalg.inv(T_sensor_to_world)

        volume.integrate(
            rgbd,
            intrinsic,
            extrinsic,
        )

    print("[INFO] Extracting mesh...")
    mesh = volume.extract_triangle_mesh()

    print(f"[INFO] Raw mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")

    mesh = clean_mesh(
        mesh,
        keep_largest=True,
        smooth_iter=args.smooth_iter,
    )

    print(f"[INFO] Clean mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")

    print(f"[INFO] Saving mesh to: {args.out_mesh}")
    o3d.io.write_triangle_mesh(args.out_mesh, mesh)

    print("[DONE]")


if __name__ == "__main__":
    main()