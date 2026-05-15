from pathlib import Path
from rosbags.highlevel import AnyReader

BAG_PATH = Path("super_sensor_2026_05_11_14_46_41.bag")

with AnyReader([BAG_PATH]) as reader:
    print("Topics:")
    for c in reader.connections:
        print(c.topic, c.msgtype, "count:", c.msgcount)