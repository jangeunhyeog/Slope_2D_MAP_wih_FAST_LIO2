from rosbags.rosbag1 import Reader
from rosbags.typesys import get_types_from_msg

bag_path = '/home/jeh/dev_ws/bags/HKU_MB_2020-09-20-13-34-51.bag'

try:
    with Reader(bag_path) as reader:
        print(f"Bag: {bag_path}")
        print(f"Duration: {reader.duration}s")
        print(f"Start: {reader.start_time}")
        print(f"End: {reader.end_time}")
        print(f"Message count: {reader.message_count}")
        print("\nTopics:")
        for connection in reader.connections:
            print(f" - {connection.topic} ({connection.msgtype})")
except Exception as e:
    print(f"Error reading bag: {e}")
