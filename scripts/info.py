from rosbags.highlevel import AnyReader
from pathlib import Path
import sys

bag_path = Path(sys.argv[1])

with AnyReader([bag_path]) as reader:
    print(f"=== Bag Info: {bag_path} ===")
    print(f"Duration: {(reader.duration / 1e9):.2f} s")
    print(f"Start:    {reader.start_time / 1e9:.2f}")
    print(f"End:      {reader.end_time / 1e9:.2f}")
    print(f"Messages: {reader.message_count}")
    print()
    print(f"{'Topic':<40} {'Type':<40} {'Count':>8} {'Hz':>8}")
    print("-" * 100)
    
    duration_s = reader.duration / 1e9
    for conn in reader.connections:
        hz = conn.msgcount / duration_s if duration_s > 0 else 0
        print(f"{conn.topic:<40} {conn.msgtype:<40} {conn.msgcount:>8} {hz:>8.1f}")