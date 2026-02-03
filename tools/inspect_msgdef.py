from rosbags.rosbag2 import Reader
src_path = '/home/jeh/dev_ws/bags/HKU_MB_converted'
with Reader(src_path) as reader:
    conn = next(iter(reader.connections))
    print(f"Type: {type(conn.msgdef)}")
    print(f"Dir: {dir(conn.msgdef)}")
    print(f"Value: {conn.msgdef}")
    print(f"Digest: {conn.digest}")
