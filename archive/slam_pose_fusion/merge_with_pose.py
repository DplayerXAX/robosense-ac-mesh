from rosbags.highlevel import AnyReader
from pathlib import Path
import numpy as np
import open3d as o3d
import sys

bag_path = Path(sys.argv[1])
poses_file = sys.argv[2]  # results/latest/xxx_poses_kitti.txt
output = sys.argv[3] if len(sys.argv) > 3 else "merged.ply"

poses = []
with open(poses_file) as f:
    for line in f:
        T = np.eye(4)
        T[:3, :4] = np.array(line.strip().split(), dtype=np.float64).reshape(3, 4)
        poses.append(T)
print(f"read {len(poses)} poses")

merged = o3d.geometry.PointCloud()
voxel_size = 0.05  

with AnyReader([bag_path]) as reader:
    conns = [c for c in reader.connections if c.topic == "/rs_lidar/points"]
    
    for i, (conn, _, rawdata) in enumerate(reader.messages(connections=conns)):
        if i >= len(poses):
            break
        
        msg = reader.deserialize(rawdata, conn.msgtype)
        
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        xyz = np.zeros((len(data), 3), dtype=np.float32)
        xyz[:, 0] = data[:, 0:4].view(np.float32).flatten()
        xyz[:, 1] = data[:, 4:8].view(np.float32).flatten()
        xyz[:, 2] = data[:, 8:12].view(np.float32).flatten()
        
        valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.5)
        xyz = xyz[valid]
        
        T = poses[i]
        xyz_world = (T[:3, :3] @ xyz.T).T + T[:3, 3]
        
        frame_pcd = o3d.geometry.PointCloud()
        frame_pcd.points = o3d.utility.Vector3dVector(xyz_world)
        merged += frame_pcd
        
        if i % 30 == 0:
            merged = merged.voxel_down_sample(voxel_size)
            print(f"  frame {i}/{len(poses)}  points: {len(merged.points)}")

merged = merged.voxel_down_sample(voxel_size)
print(f"\nfinal points: {len(merged.points)}")

o3d.io.write_point_cloud(output, merged)
print(f"save to: {output}")