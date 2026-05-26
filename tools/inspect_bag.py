#!/usr/bin/env python3
"""
检查 ROS bag：topic 列表、频率、PointCloud2 字段、IMU 样本。
合并了原来的 info.py / inspect_bag_topics.py / inspect_pointcloud_fields.py / check.py。

注意：现在主线用 FAST-LIVO 成品点云，这个工具主要用于调试原始 bag，
      平时不需要。

用法:
    python inspect_bag.py <bag_path>                 # 概览
    python inspect_bag.py <bag_path> --fields        # 看 PointCloud2 字段
    python inspect_bag.py <bag_path> --imu           # 看 IMU 样本
"""

import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader

DTYPE = {1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16",
         5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64"}


def overview(reader):
    dur = reader.duration / 1e9
    print(f"时长: {dur:.2f}s  消息数: {reader.message_count}")
    print(f"{'Topic':<40}{'Type':<42}{'Count':>8}{'Hz':>8}")
    print("-" * 98)
    for c in reader.connections:
        hz = c.msgcount / dur if dur > 0 else 0
        print(f"{c.topic:<40}{c.msgtype:<42}{c.msgcount:>8}{hz:>8.1f}")


def show_fields(reader, topic):
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        # 没指定就找第一个 PointCloud2
        conns = [c for c in reader.connections if "PointCloud2" in c.msgtype][:1]
    for conn in conns:
        for _, _, raw in reader.messages(connections=[conn]):
            msg = reader.deserialize(raw, conn.msgtype)
            print(f"\ntopic: {conn.topic}")
            print(f"  width={msg.width} height={msg.height} "
                  f"point_step={msg.point_step} is_dense={msg.is_dense}")
            print("  fields:")
            for f in msg.fields:
                print(f"    {f.name:<14} offset={f.offset:<4} "
                      f"type={DTYPE.get(f.datatype, '?')}")
            break


def show_imu(reader, topic="/rs_imu"):
    conns = [c for c in reader.connections if c.topic == topic]
    for conn in conns:
        for _, _, raw in reader.messages(connections=[conn]):
            msg = reader.deserialize(raw, conn.msgtype)
            g, a = msg.angular_velocity, msg.linear_acceleration
            print(f"\nIMU [{conn.topic}] frame={msg.header.frame_id}")
            print(f"  gyro  : {g.x:.4f} {g.y:.4f} {g.z:.4f}")
            print(f"  accel : {a.x:.4f} {a.y:.4f} {a.z:.4f}")
            break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--fields", action="store_true")
    ap.add_argument("--imu", action="store_true")
    ap.add_argument("--topic", default="/rs_lidar/points")
    args = ap.parse_args()

    with AnyReader([Path(args.bag)]) as reader:
        print(f"=== {args.bag} ===")
        overview(reader)
        if args.fields:
            show_fields(reader, args.topic)
        if args.imu:
            show_imu(reader)


if __name__ == "__main__":
    main()
