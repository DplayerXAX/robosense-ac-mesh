from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import numpy as np
import open3d as o3d
import os

BAG_PATH = Path("super_sensor_2026_05_11_14_46_41.bag")
POINT_TOPIC = "/rs_lidar/points"
OUT_DIR = "output"

os.makedirs(OUT_DIR, exist_ok=True)

typestore = get_typestore(Stores.ROS2_HUMBLE)

with AnyReader([BAG_PATH], default_typestore=typestore) as reader:
    connections = [c for c in reader.connections if c.topic == POINT_TOPIC]

    for i, (conn, timestamp, rawdata) in enumerate(
        reader.messages(connections=connections)
    ):
        msg = reader.deserialize(rawdata, conn.msgtype)

        points = np.frombuffer(
            msg.data, dtype=np.float32
        ).reshape(-1, msg.point_step // 4)[:, :3]

        if len(points) == 0:
            continue

        pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(points)
        )

        out = f"{OUT_DIR}/cloud_{i:04d}.ply"
        o3d.io.write_point_cloud(out, pcd)
        print("Saved:", out)