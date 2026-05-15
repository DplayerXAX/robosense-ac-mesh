from pathlib import Path
from rosbags.highlevel import AnyReader

BAG_PATH = Path("colored_bag/ac_colored_points.bag")

with AnyReader([BAG_PATH]) as reader:
    print("Topics:")
    for c in reader.connections:
        print(" ", c.topic, c.msgtype, "count:", c.msgcount)

    print("\nFirst PointCloud2-like message fields:")

    for c in reader.connections:
        if "PointCloud2" not in c.msgtype:
            continue

        for conn, timestamp, rawdata in reader.messages(connections=[c]):
            msg = reader.deserialize(rawdata, conn.msgtype)

            print("topic:", conn.topic)
            print("msgtype:", conn.msgtype)
            print("height:", msg.height)
            print("width:", msg.width)
            print("point_step:", msg.point_step)
            print("row_step:", msg.row_step)
            print("is_bigendian:", msg.is_bigendian)
            print("fields:")

            for f in msg.fields:
                print(
                    "  name=", repr(f.name),
                    "offset=", f.offset,
                    "datatype=", f.datatype,
                    "count=", f.count
                )

            raise SystemExit