from rosbags.highlevel import AnyReader
from pathlib import Path
import sys

bag_path = Path(sys.argv[1])

with AnyReader([bag_path]) as reader:
    
    pc_conns = [c for c in reader.connections if c.topic == "/rs_lidar/points"]
    for conn in pc_conns:
        print(f"=== PointCloud2 fields ===")
        for _, _, rawdata in reader.messages(connections=[conn]):
            msg = reader.deserialize(rawdata, conn.msgtype)
            print(f"frame_id:    {msg.header.frame_id}")
            print(f"width:       {msg.width}")
            print(f"height:      {msg.height}")
            print(f"point_step:  {msg.point_step}")
            print(f"is_dense:    {msg.is_dense}")
            print(f"fields:")
            for f in msg.fields:
                # datatype: 1=INT8 2=UINT8 3=INT16 4=UINT16 5=INT32 6=UINT32 7=FLOAT32 8=FLOAT64
                dtype_name = {1:"INT8",2:"UINT8",3:"INT16",4:"UINT16",5:"INT32",6:"UINT32",7:"FLOAT32",8:"FLOAT64"}.get(f.datatype, "?")
                print(f"  {f.name:<15} offset={f.offset:<4} type={dtype_name}")
            break
    
    imu_conns = [c for c in reader.connections if c.topic == "/rs_imu"]
    for conn in imu_conns:
        print(f"\n=== IMU sample ===")
        for _, _, rawdata in reader.messages(connections=[conn]):
            msg = reader.deserialize(rawdata, conn.msgtype)
            print(f"frame_id: {msg.header.frame_id}")
            print(f"gyro  (rad/s): x={msg.angular_velocity.x:.4f} y={msg.angular_velocity.y:.4f} z={msg.angular_velocity.z:.4f}")
            print(f"accel (m/s²):  x={msg.linear_acceleration.x:.4f} y={msg.linear_acceleration.y:.4f} z={msg.linear_acceleration.z:.4f}")
            break