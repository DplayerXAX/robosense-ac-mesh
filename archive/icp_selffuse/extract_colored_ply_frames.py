from pathlib import Path
from rosbags.highlevel import AnyReader

import numpy as np
import open3d as o3d


BAG_PATH = Path("../colored_bag/ac_colored_points.bag")
POINT_TOPIC = "/rslidar_points_motion_rgb"

OUT_DIR = Path("colored_ply_frames")
OUT_DIR.mkdir(exist_ok=True)

FRAME_STEP = 1

MAX_POINTS_PER_FRAME = None
# MAX_POINTS_PER_FRAME = 200000


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


def normalize_name(name):
    return str(name).strip()


def get_field_map(msg):
    return {normalize_name(f.name): f for f in msg.fields}


def build_raw_matrix(msg):
    """
    Turn pointcloud2 data into a 2D matrix
    shape = (point_count, point_step)
    """
    point_step = msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if len(raw) < point_step:
        return None

    count = len(raw) // point_step
    raw = raw[:count * point_step]
    raw2 = raw.reshape(count, point_step)

    return raw2


def read_field_from_raw2(raw2, field):
    """
    按 PointField 的 offset / datatype 读取字段。
    """
    dtype = DATATYPE_MAP[field.datatype]
    size = np.dtype(dtype).itemsize

    values = raw2[:, field.offset:field.offset + size].copy()
    values = values.view(dtype).reshape(-1)

    return values


def decode_rgb_field_from_raw2(raw2, field):
    data4 = raw2[:, field.offset:field.offset + 4].copy()

    if field.datatype == 7:  # FLOAT32
        rgb_float = data4.view(np.float32).reshape(-1)
        rgb_uint = rgb_float.view(np.uint32)

    elif field.datatype == 6:  # UINT32
        rgb_uint = data4.view(np.uint32).reshape(-1)

    else:
        rgb_uint = data4.view(np.uint32).reshape(-1)

    r = ((rgb_uint >> 16) & 255).astype(np.float64) / 255.0
    g = ((rgb_uint >> 8) & 255).astype(np.float64) / 255.0
    b = (rgb_uint & 255).astype(np.float64) / 255.0

    colors = np.stack([r, g, b], axis=1)

    return colors


def pointcloud2_to_xyz_rgb(msg):
    fields = get_field_map(msg)

    required = ["x", "y", "z"]

    missing = [k for k in required if k not in fields]
    if missing:
        print("Existing fields:")
        for f in msg.fields:
            print(
                "  name=", repr(f.name),
                "normalized=", repr(normalize_name(f.name)),
                "offset=", f.offset,
                "datatype=", f.datatype,
                "count=", f.count,
            )
        raise RuntimeError(f"PointCloud2 missing fields: {missing}")

    raw2 = build_raw_matrix(msg)
    if raw2 is None:
        return np.empty((0, 3), dtype=np.float64), None

    x = read_field_from_raw2(raw2, fields["x"]).astype(np.float64)
    y = read_field_from_raw2(raw2, fields["y"]).astype(np.float64)
    z = read_field_from_raw2(raw2, fields["z"]).astype(np.float64)

    points = np.stack([x, y, z], axis=1)

    valid = np.isfinite(points).all(axis=1)
    points = points[valid]

    colors = None

    if "rgb" in fields:
        colors = decode_rgb_field_from_raw2(raw2, fields["rgb"])
        colors = colors[valid]

    elif "rgba" in fields:
        colors = decode_rgb_field_from_raw2(raw2, fields["rgba"])
        colors = colors[valid]

    else:
        print("Warning: no rgb/rgba field found. Exporting geometry only.")

    return points, colors


def main():
    if not BAG_PATH.exists():
        raise FileNotFoundError(f"Cannot find bag: {BAG_PATH}")

    saved = 0

    with AnyReader([BAG_PATH]) as reader:
        print("Topics:")
        for c in reader.connections:
            print(" ", c.topic, c.msgtype, "count:", c.msgcount)

        conns = [c for c in reader.connections if c.topic == POINT_TOPIC]

        if not conns:
            print("Cannot find topic:", POINT_TOPIC)
            print("Available topics:")
            for c in reader.connections:
                print(" ", c.topic, c.msgtype, "count:", c.msgcount)
            return

        print("Start extracting:", POINT_TOPIC)
        print("Output dir:", OUT_DIR)

        for frame_idx, (conn, timestamp, rawdata) in enumerate(
            reader.messages(connections=conns)
        ):
            if frame_idx % FRAME_STEP != 0:
                continue

            msg = reader.deserialize(rawdata, conn.msgtype)

            try:
                points, colors = pointcloud2_to_xyz_rgb(msg)
            except Exception as e:
                print(f"frame {frame_idx}: invalid PointCloud2, skip. error={e}")
                print("  width:", getattr(msg, "width", None))
                print("  height:", getattr(msg, "height", None))
                print("  point_step:", getattr(msg, "point_step", None))
                print("  fields:", [repr(f.name) for f in getattr(msg, "fields", [])])
                continue

            if len(points) == 0:
                print(f"frame {frame_idx}: empty, skip")
                continue

            if MAX_POINTS_PER_FRAME is not None and len(points) > MAX_POINTS_PER_FRAME:
                idx = np.random.choice(
                    len(points),
                    MAX_POINTS_PER_FRAME,
                    replace=False
                )

                points = points[idx]

                if colors is not None:
                    colors = colors[idx]

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            if colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(colors)

            out_path = OUT_DIR / f"frame_{saved:06d}.ply"
            o3d.io.write_point_cloud(str(out_path), pcd)

            print(
                f"saved {out_path}, "
                f"points={len(points)}, "
                f"has_color={pcd.has_colors()}"
            )

            saved += 1

    print("Done.")
    print("Total saved:", saved)
    print("Output dir:", OUT_DIR)


if __name__ == "__main__":
    main()