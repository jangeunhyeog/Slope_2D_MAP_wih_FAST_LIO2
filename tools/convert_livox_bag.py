from rosbags.rosbag2 import Reader, Writer

src_path = '/home/jeh/dev_ws/bags/HKU_MB_new'
dst_path = '/home/jeh/dev_ws/bags/HKU_MB_v2'

import os
import shutil

if os.path.exists(dst_path):
    shutil.rmtree(dst_path)

print(f"Converting {src_path} -> {dst_path}...")

# Hardcoded hashes from original metadata
LIDAR_HASH = 'RIHS01_94041b4794f52c1d81def2989107fc898a62dacb7a39d5dbe80d4b55e538bf6d'
IMU_HASH   = 'RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b'
ZERO_HASH  = 'RIHS01_' + '0' * 64

with Reader(src_path) as reader, Writer(dst_path, version=5) as writer:
    conn_map = {}
    for conn in reader.connections:
        topic = conn.topic
        msgtype = conn.msgtype
        
        digest = conn.digest
        if not digest:
            if topic == '/livox/lidar':
                digest = LIDAR_HASH
            elif topic == '/livox/imu':
                digest = IMU_HASH
            else:
                digest = ZERO_HASH

        # If it is the lidar topic, change type
        if topic == '/livox/lidar':
            print(f"Updating type for topic {topic} from {msgtype} to livox_ros_driver2/msg/CustomMsg")
            msgtype = 'livox_ros_driver2/msg/CustomMsg'
        
        new_conn = writer.add_connection(
            topic=topic,
            msgtype=msgtype,
            msgdef=conn.msgdef.data, # Access .data here
            rihs01=digest,
            serialization_format=conn.ext.serialization_format,
            offered_qos_profiles=conn.ext.offered_qos_profiles
        )
        conn_map[conn.id] = new_conn

    count = 0
    for conn, timestamp, data in reader.messages():
        writer.write(conn_map[conn.id], timestamp, data)
        count += 1
        if count % 10000 == 0:
            print(f"Processed {count} messages...")

print(f"Done. Processed {count} messages.")
