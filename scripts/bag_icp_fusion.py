#!/usr/bin/env python3

import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

import numpy as np
import open3d as o3d


# ============================================================
# Default parameters, can be overwritten by argparse
# ============================================================

BAG_PATH = None
POINT_TOPIC = None
OUT_DIR = None

FRAME_STEP = 1
MAX_FRAMES = -1

MAX_POINTS_PER_FRAME = 100000

VOXEL_SIZE = 0.06

ICP_DISTANCE_COARSE = 0.60
ICP_DISTANCE_FINE = 0.25

SAVE_EVERY = 20

USE_AXIS_SNAPPED_NORMALS = True
NORMAL_AXIS_ANGLE_DEG = 10.0
NORMAL_RADIUS_SCALE = 4.0

USE_CONSISTENT_NORMAL_ORIENTATION = False
PRINT_NORMAL_SNAP_RATIO = False

MIN_FITNESS = 0.15
MAX_RMSE = 0.60


# ============================================================
# Final plane smoothing
# ============================================================

USE_PLANE_SMOOTHING = True

PLANE_DISTANCE_THRESHOLD = 0.05
PLANE_AXIS_ANGLE_DEG = 12.0
MIN_PLANE_POINTS = 3000
MAX_PLANES = 8


# ============================================================
# PointCloud2 datatype map
# ============================================================

DATATYPE_MAP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="ICP fusion from ROS bag PointCloud2 topic."
    )

    parser.add_argument(
        "-i",
        "--input",
        "--bag",
        dest="input_bag",
        required=True,
        help="Input ROS bag path.",
    )

    parser.add_argument(
        "-o",
        "--output",
        "--out_dir",
        dest="output_dir",
        required=True,
        help="Output directory for fused point clouds and poses.npy.",
    )

    parser.add_argument(
        "--topic",
        default="/rs_lidar/points",
        help="PointCloud2 topic name.",
    )

    parser.add_argument(
        "--frame_step",
        type=int,
        default=1,
        help="Use every N-th frame. 1 means use all frames.",
    )

    parser.add_argument(
        "--max_frames",
        type=int,
        default=-1,
        help="Maximum processed frames after frame_step. Use -1 for no limit.",
    )

    parser.add_argument(
        "--max_points_per_frame",
        type=int,
        default=100000,
        help="Randomly sample this many points per frame before downsampling. Use 0 for no limit.",
    )

    parser.add_argument(
        "--voxel_size",
        type=float,
        default=0.06,
        help="Voxel size for downsampling.",
    )

    parser.add_argument(
        "--icp_distance_coarse",
        type=float,
        default=0.60,
        help="Coarse ICP max correspondence distance.",
    )

    parser.add_argument(
        "--icp_distance_fine",
        type=float,
        default=0.25,
        help="Fine ICP max correspondence distance.",
    )

    parser.add_argument(
        "--save_every",
        type=int,
        default=20,
        help="Save intermediate global map every N used frames.",
    )

    parser.add_argument(
        "--min_fitness",
        type=float,
        default=0.15,
        help="Minimum ICP fitness. Frames below this are skipped.",
    )

    parser.add_argument(
        "--max_rmse",
        type=float,
        default=0.60,
        help="Maximum ICP inlier RMSE. Frames above this are skipped.",
    )

    parser.add_argument(
        "--disable_axis_snapped_normals",
        action="store_true",
        help="Disable normal snapping to major axes.",
    )

    parser.add_argument(
        "--normal_axis_angle_deg",
        type=float,
        default=10.0,
        help="Angle threshold for snapping normals to major axes.",
    )

    parser.add_argument(
        "--normal_radius_scale",
        type=float,
        default=4.0,
        help="Normal estimation radius = voxel_size * normal_radius_scale.",
    )

    parser.add_argument(
        "--consistent_normals",
        action="store_true",
        help="Use Open3D consistent normal orientation.",
    )

    parser.add_argument(
        "--print_normal_snap_ratio",
        action="store_true",
        help="Print ratio of normals snapped to axes.",
    )

    parser.add_argument(
        "--disable_plane_smoothing",
        action="store_true",
        help="Disable final plane smoothing.",
    )

    parser.add_argument(
        "--plane_distance_threshold",
        type=float,
        default=0.05,
        help="RANSAC plane distance threshold.",
    )

    parser.add_argument(
        "--plane_axis_angle_deg",
        type=float,
        default=12.0,
        help="Only snap planes whose normal is within this angle to major axes.",
    )

    parser.add_argument(
        "--min_plane_points",
        type=int,
        default=3000,
        help="Minimum inlier points required for a plane.",
    )

    parser.add_argument(
        "--max_planes",
        type=int,
        default=8,
        help="Maximum number of planes to snap.",
    )

    return parser.parse_args()


def normalize_field_name(name):
    return str(name).strip()


def read_point_field(raw, field, point_step):
    dtype = DATATYPE_MAP[field.datatype]
    size = np.dtype(dtype).itemsize

    count = len(raw) // point_step
    raw2 = raw[:count * point_step].reshape(count, point_step)

    values = raw2[:, field.offset:field.offset + size].copy().view(dtype).reshape(-1)

    return values


def pointcloud2_to_xyz_intensity(msg):
    """
    Read x/y/z and optional intensity from sensor_msgs/msg/PointCloud2.
    Colors will be None if intensity field is not present.
    """

    fields = {normalize_field_name(f.name): f for f in msg.fields}

    if not all(k in fields for k in ["x", "y", "z"]):
        print("Existing fields:")
        for f in msg.fields:
            print(
                "  name=", repr(f.name),
                "normalized=", repr(normalize_field_name(f.name)),
                "offset=", f.offset,
                "datatype=", f.datatype,
                "count=", f.count,
            )
        raise RuntimeError("PointCloud2 does not contain x/y/z fields.")

    point_step = msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if len(raw) < point_step:
        return np.empty((0, 3), dtype=np.float64), None

    x = read_point_field(raw, fields["x"], point_step).astype(np.float64)
    y = read_point_field(raw, fields["y"], point_step).astype(np.float64)
    z = read_point_field(raw, fields["z"], point_step).astype(np.float64)

    pts = np.stack([x, y, z], axis=1)
    valid = np.isfinite(pts).all(axis=1)

    pts = pts[valid]

    intensity = None
    if "intensity" in fields:
        intensity = read_point_field(raw, fields["intensity"], point_step).astype(np.float64)
        intensity = intensity[valid]

    return pts, intensity


def intensity_to_gray_color(intensity):
    """
    intensity -> grayscale color.
    Open3D color range: 0~1.
    """

    if intensity is None or len(intensity) == 0:
        return None

    lo = np.percentile(intensity, 2)
    hi = np.percentile(intensity, 98)

    gray = (intensity - lo) / (hi - lo + 1e-9)
    gray = np.clip(gray, 0.0, 1.0)

    colors = np.stack([gray, gray, gray], axis=1)

    return colors


def intensity_to_heat_color(intensity):
    """
    intensity -> heatmap color.
    """

    if intensity is None or len(intensity) == 0:
        return None

    lo = np.percentile(intensity, 2)
    hi = np.percentile(intensity, 98)

    t = (intensity - lo) / (hi - lo + 1e-9)
    t = np.clip(t, 0.0, 1.0)

    colors = np.zeros((len(t), 3), dtype=np.float64)

    colors[:, 0] = np.clip(2.0 * t, 0.0, 1.0)
    colors[:, 1] = np.clip(2.0 * (1.0 - np.abs(t - 0.5)), 0.0, 1.0)
    colors[:, 2] = np.clip(2.0 * (1.0 - t), 0.0, 1.0)

    return colors


def make_pcd(points, colors=None):
    """
    Generate Open3D point cloud from points and optional colors.

    Steps:
      - Randomly limit points to MAX_POINTS_PER_FRAME
      - Voxel downsample with VOXEL_SIZE
      - Statistical outlier removal
      - Keep colors if provided
    """

    if MAX_POINTS_PER_FRAME is not None and len(points) > MAX_POINTS_PER_FRAME:
        idx = np.random.choice(len(points), MAX_POINTS_PER_FRAME, replace=False)
        points = points[idx]

        if colors is not None:
            colors = colors[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    pcd = pcd.voxel_down_sample(VOXEL_SIZE)

    if len(pcd.points) > 100:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=2.0,
        )

    return pcd


def snap_normals_to_axes(pcd, angle_deg=10.0):
    """
    Snap normals to nearest major axis, x/y/z and their negatives, if within angle_deg.
    """

    if not pcd.has_normals():
        return pcd, 0.0

    normals = np.asarray(pcd.normals)

    if len(normals) == 0:
        return pcd, 0.0

    axes = np.array(
        [
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
        ],
        dtype=np.float64,
    )

    threshold = np.cos(np.deg2rad(angle_deg))

    dots = normals @ axes.T
    best_idx = np.argmax(dots, axis=1)
    best_dot = dots[np.arange(len(normals)), best_idx]

    mask = best_dot > threshold

    snapped = normals.copy()
    snapped[mask] = axes[best_idx[mask]]

    pcd.normals = o3d.utility.Vector3dVector(snapped)

    ratio = float(np.mean(mask) * 100.0)

    return pcd, ratio


def estimate_normals(pcd, snap_axis=True, label="pcd"):
    if len(pcd.points) < 30:
        return pcd

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=VOXEL_SIZE * NORMAL_RADIUS_SCALE,
            max_nn=30,
        )
    )

    if USE_CONSISTENT_NORMAL_ORIENTATION:
        try:
            pcd.orient_normals_consistent_tangent_plane(30)
        except Exception:
            pass

    if USE_AXIS_SNAPPED_NORMALS and snap_axis:
        pcd, ratio = snap_normals_to_axes(
            pcd,
            angle_deg=NORMAL_AXIS_ANGLE_DEG,
        )

        if PRINT_NORMAL_SNAP_RATIO:
            print(
                f"{label}: axis-snapped normals "
                f"{ratio:.1f}% within {NORMAL_AXIS_ANGLE_DEG:.1f} deg"
            )

    return pcd


def run_two_stage_icp(source, target, init):
    """
    Run two-stage point-to-plane ICP.
    """

    result_coarse = o3d.pipelines.registration.registration_icp(
        source,
        target,
        ICP_DISTANCE_COARSE,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=30,
        ),
    )

    result_fine = o3d.pipelines.registration.registration_icp(
        source,
        target,
        ICP_DISTANCE_FINE,
        result_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=30,
        ),
    )

    return result_coarse, result_fine


def snap_points_to_major_axis_planes(
    pcd,
    distance_threshold=0.05,
    axis_angle_deg=12.0,
    min_plane_points=3000,
    max_planes=8,
):
    """
    Final plane smoothing:

    - RANSAC find planes
    - Only keep planes whose normal is close to X/Y/Z axis
    - Project points in the plane to the plane
    """

    if len(pcd.points) < min_plane_points:
        return pcd

    pts = np.asarray(pcd.points).copy()

    has_colors = pcd.has_colors()
    colors = None
    if has_colors:
        colors = np.asarray(pcd.colors).copy()

    remaining = np.arange(len(pts))

    axes = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    cos_thr = np.cos(np.deg2rad(axis_angle_deg))
    snapped_total = 0

    for plane_id in range(max_planes):
        if len(remaining) < min_plane_points:
            break

        sub = o3d.geometry.PointCloud()
        sub.points = o3d.utility.Vector3dVector(pts[remaining])

        if has_colors:
            sub.colors = o3d.utility.Vector3dVector(colors[remaining])

        try:
            plane_model, inliers = sub.segment_plane(
                distance_threshold=distance_threshold,
                ransac_n=3,
                num_iterations=1000,
            )
        except Exception as e:
            print("Plane segmentation failed:", e)
            break

        if len(inliers) < min_plane_points:
            break

        a, b, c, d = plane_model
        n = np.array([a, b, c], dtype=np.float64)
        norm = np.linalg.norm(n)

        if norm < 1e-9:
            break

        n = n / norm
        d = d / norm

        axis_dot = np.max(np.abs(axes @ n))
        global_idx = remaining[np.array(inliers)]

        if axis_dot < cos_thr:
            print(
                f"plane {plane_id}: skip non-axis plane, "
                f"points={len(inliers)}, normal={n}, axis_dot={axis_dot:.3f}"
            )
            remaining = np.delete(remaining, inliers)
            continue

        dist = pts[global_idx] @ n + d
        pts[global_idx] = pts[global_idx] - dist[:, None] * n

        snapped_total += len(global_idx)

        print(
            f"plane {plane_id}: snapped {len(global_idx)} points, "
            f"normal={n}, d={d:.3f}, axis_dot={axis_dot:.3f}"
        )

        remaining = np.delete(remaining, inliers)

    pcd.points = o3d.utility.Vector3dVector(pts)

    if has_colors:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"Plane smoothing done. Total snapped points: {snapped_total}")

    return pcd


def apply_args_to_globals(args):
    global BAG_PATH
    global POINT_TOPIC
    global OUT_DIR

    global FRAME_STEP
    global MAX_FRAMES
    global MAX_POINTS_PER_FRAME

    global VOXEL_SIZE
    global ICP_DISTANCE_COARSE
    global ICP_DISTANCE_FINE
    global SAVE_EVERY

    global USE_AXIS_SNAPPED_NORMALS
    global NORMAL_AXIS_ANGLE_DEG
    global NORMAL_RADIUS_SCALE
    global USE_CONSISTENT_NORMAL_ORIENTATION
    global PRINT_NORMAL_SNAP_RATIO

    global MIN_FITNESS
    global MAX_RMSE

    global USE_PLANE_SMOOTHING
    global PLANE_DISTANCE_THRESHOLD
    global PLANE_AXIS_ANGLE_DEG
    global MIN_PLANE_POINTS
    global MAX_PLANES

    BAG_PATH = Path(args.input_bag)
    POINT_TOPIC = args.topic
    OUT_DIR = Path(args.output_dir)

    FRAME_STEP = args.frame_step
    MAX_FRAMES = args.max_frames

    MAX_POINTS_PER_FRAME = (
        None if args.max_points_per_frame <= 0 else args.max_points_per_frame
    )

    VOXEL_SIZE = args.voxel_size

    ICP_DISTANCE_COARSE = args.icp_distance_coarse
    ICP_DISTANCE_FINE = args.icp_distance_fine

    SAVE_EVERY = args.save_every

    USE_AXIS_SNAPPED_NORMALS = not args.disable_axis_snapped_normals
    NORMAL_AXIS_ANGLE_DEG = args.normal_axis_angle_deg
    NORMAL_RADIUS_SCALE = args.normal_radius_scale
    USE_CONSISTENT_NORMAL_ORIENTATION = args.consistent_normals
    PRINT_NORMAL_SNAP_RATIO = args.print_normal_snap_ratio

    MIN_FITNESS = args.min_fitness
    MAX_RMSE = args.max_rmse

    USE_PLANE_SMOOTHING = not args.disable_plane_smoothing
    PLANE_DISTANCE_THRESHOLD = args.plane_distance_threshold
    PLANE_AXIS_ANGLE_DEG = args.plane_axis_angle_deg
    MIN_PLANE_POINTS = args.min_plane_points
    MAX_PLANES = args.max_planes


def print_config():
    print("[CONFIG] BAG_PATH:", BAG_PATH)
    print("[CONFIG] POINT_TOPIC:", POINT_TOPIC)
    print("[CONFIG] OUT_DIR:", OUT_DIR)

    print("[CONFIG] FRAME_STEP:", FRAME_STEP)
    print("[CONFIG] MAX_FRAMES:", MAX_FRAMES)
    print("[CONFIG] MAX_POINTS_PER_FRAME:", MAX_POINTS_PER_FRAME)

    print("[CONFIG] VOXEL_SIZE:", VOXEL_SIZE)
    print("[CONFIG] ICP_DISTANCE_COARSE:", ICP_DISTANCE_COARSE)
    print("[CONFIG] ICP_DISTANCE_FINE:", ICP_DISTANCE_FINE)

    print("[CONFIG] SAVE_EVERY:", SAVE_EVERY)

    print("[CONFIG] USE_AXIS_SNAPPED_NORMALS:", USE_AXIS_SNAPPED_NORMALS)
    print("[CONFIG] NORMAL_AXIS_ANGLE_DEG:", NORMAL_AXIS_ANGLE_DEG)
    print("[CONFIG] NORMAL_RADIUS_SCALE:", NORMAL_RADIUS_SCALE)
    print("[CONFIG] USE_CONSISTENT_NORMAL_ORIENTATION:", USE_CONSISTENT_NORMAL_ORIENTATION)

    print("[CONFIG] MIN_FITNESS:", MIN_FITNESS)
    print("[CONFIG] MAX_RMSE:", MAX_RMSE)

    print("[CONFIG] USE_PLANE_SMOOTHING:", USE_PLANE_SMOOTHING)
    print("[CONFIG] PLANE_DISTANCE_THRESHOLD:", PLANE_DISTANCE_THRESHOLD)
    print("[CONFIG] PLANE_AXIS_ANGLE_DEG:", PLANE_AXIS_ANGLE_DEG)
    print("[CONFIG] MIN_PLANE_POINTS:", MIN_PLANE_POINTS)
    print("[CONFIG] MAX_PLANES:", MAX_PLANES)


def main():
    args = parse_args()
    apply_args_to_globals(args)

    if not BAG_PATH.exists():
        raise FileNotFoundError(f"Input bag not found: {BAG_PATH}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    print_config()

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    global_map = None
    global_map_down = None

    last_pose = np.eye(4)

    poses = []
    used_frames = 0
    skipped_frames = 0
    processed_frames = 0

    with AnyReader([BAG_PATH], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == POINT_TOPIC]

        if not conns:
            print("Cannot find topic:", POINT_TOPIC)
            print("Available topics:")
            for c in reader.connections:
                print(c.topic, c.msgtype)
            return

        print("Start reading:", POINT_TOPIC)
        print("Color mode: intensity grayscale")

        for frame_idx, (conn, timestamp, rawdata) in enumerate(
            reader.messages(connections=conns)
        ):
            if frame_idx % FRAME_STEP != 0:
                continue

            if MAX_FRAMES > 0 and processed_frames >= MAX_FRAMES:
                print(f"Reached MAX_FRAMES={MAX_FRAMES}, stop.")
                break

            processed_frames += 1

            msg = reader.deserialize(rawdata, conn.msgtype)

            try:
                points, intensity = pointcloud2_to_xyz_intensity(msg)
            except Exception as e:
                print(f"frame {frame_idx}: invalid PointCloud2, skip. error={e}")
                skipped_frames += 1
                continue

            if len(points) < 500:
                print(f"frame {frame_idx}: too few points, skip")
                skipped_frames += 1
                continue

            colors = intensity_to_gray_color(intensity)
            source = make_pcd(points, colors)

            if len(source.points) < 500:
                print(f"frame {frame_idx}: too few points after downsample, skip")
                skipped_frames += 1
                continue

            source = estimate_normals(
                source,
                snap_axis=True,
                label=f"frame {frame_idx} source",
            )

            if global_map is None:
                global_map = o3d.geometry.PointCloud(source)

                global_map_down = global_map.voxel_down_sample(VOXEL_SIZE * 2.0)
                global_map_down = estimate_normals(
                    global_map_down,
                    snap_axis=True,
                    label="global init",
                )

                poses.append(last_pose.copy())
                used_frames += 1

                print(
                    f"frame {frame_idx}: init map, "
                    f"points={len(global_map.points)}, "
                    f"has_color={global_map.has_colors()}"
                )
                continue

            init = last_pose.copy()

            try:
                result_coarse, result = run_two_stage_icp(
                    source,
                    global_map_down,
                    init,
                )
            except Exception as e:
                print(f"frame {frame_idx}: ICP failed: {e}")
                skipped_frames += 1
                continue

            if result.fitness < MIN_FITNESS or result.inlier_rmse > MAX_RMSE:
                print(
                    f"frame {frame_idx}: ICP bad, skip, "
                    f"coarse_fitness={result_coarse.fitness:.3f}, "
                    f"coarse_rmse={result_coarse.inlier_rmse:.3f}, "
                    f"fine_fitness={result.fitness:.3f}, "
                    f"fine_rmse={result.inlier_rmse:.3f}"
                )
                skipped_frames += 1
                continue

            pose = result.transformation
            last_pose = pose

            source_global = o3d.geometry.PointCloud(source)
            source_global.transform(pose)

            global_map += source_global

            global_map = global_map.voxel_down_sample(VOXEL_SIZE)

            global_map_down = global_map.voxel_down_sample(VOXEL_SIZE * 2.0)
            global_map_down = estimate_normals(
                global_map_down,
                snap_axis=True,
                label=f"global after frame {frame_idx}",
            )

            poses.append(pose.copy())
            used_frames += 1

            print(
                f"frame {frame_idx}: "
                f"coarse_fit={result_coarse.fitness:.3f}, "
                f"coarse_rmse={result_coarse.inlier_rmse:.3f}, "
                f"fine_fit={result.fitness:.3f}, "
                f"fine_rmse={result.inlier_rmse:.3f}, "
                f"map_points={len(global_map.points)}, "
                f"has_color={global_map.has_colors()}, "
                f"used={used_frames}, "
                f"skipped={skipped_frames}"
            )

            if SAVE_EVERY > 0 and used_frames % SAVE_EVERY == 0:
                mid_path = OUT_DIR / f"global_map_{used_frames:05d}_color.ply"
                o3d.io.write_point_cloud(str(mid_path), global_map)
                print("mid save:", mid_path)

    if global_map is None:
        print("No map generated.")
        return

    raw_ply = OUT_DIR / "global_map_raw_color.ply"
    o3d.io.write_point_cloud(str(raw_ply), global_map)

    print("Save raw colored point cloud:", raw_ply)
    print("Raw has color:", global_map.has_colors())

    if USE_PLANE_SMOOTHING:
        smoothed_map = o3d.geometry.PointCloud(global_map)

        smoothed_map = snap_points_to_major_axis_planes(
            smoothed_map,
            distance_threshold=PLANE_DISTANCE_THRESHOLD,
            axis_angle_deg=PLANE_AXIS_ANGLE_DEG,
            min_plane_points=MIN_PLANE_POINTS,
            max_planes=MAX_PLANES,
        )

        smoothed_ply = OUT_DIR / "global_map_smoothed_color.ply"
        o3d.io.write_point_cloud(str(smoothed_ply), smoothed_map)

        print("Save smoothed colored point cloud:", smoothed_ply)
        print("Smoothed has color:", smoothed_map.has_colors())
    else:
        smoothed_ply = None

    if poses:
        poses_np = np.stack(poses, axis=0)
        poses_path = OUT_DIR / "poses.npy"
        np.save(str(poses_path), poses_np)
    else:
        poses_path = None

    print("Done")
    print("Raw point cloud:", raw_ply)
    print("Smoothed point cloud:", smoothed_ply)
    print("Save poses:", poses_path)
    print("Processed frames:", processed_frames)
    print("Total used frames:", used_frames)
    print("Total skipped frames:", skipped_frames)


if __name__ == "__main__":
    main()
