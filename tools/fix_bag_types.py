from rosbags.rosbag2 import Reader, Writer
from rosbags.interfaces import ConnectionExtRosbag2
# from rosbags.serde import serialize_cdr, ros1_to_cdr
# from rosbags.typesys import get_types_from_msg, register_types

# LIVOX_MSG_DEF = ... (removed)

# register_types(...) (removed)
# from rosbags.typesys.types import livox_ros_driver2__msg__CustomMsg as CustomMsg2
# We also need the source type to deserialize if needed, or we safely assume binary compatibility?
# Actually, rosbags might not have livox_ros_driver/msg/CustomMsg by default if not standard?
# But we are reading an already converted db3 bag which has type definitions.
# Using the converted bag is easier.

src_path = '/home/jeh/dev_ws/bags/HKU_MB_converted'
dst_path = '/home/jeh/dev_ws/bags/HKU_MB_fixed'

import os
import shutil

if os.path.exists(dst_path):
    shutil.rmtree(dst_path)

with Reader(src_path) as reader, Writer(dst_path, version=9) as writer:
    conn_map = {}
    for conn in reader.connections:
        topic = conn.topic
        msgtype = conn.msgtype
        
        # If it is the lidar topic, change type
        if topic == '/livox/lidar':
            msgtype = 'livox_ros_driver2/msg/CustomMsg'
            # We reuse the msg definition/serialization from source if binary compatible, 
            # but we need to register the new connection.
            # actually we don't need to deserialize if we trust the binary is identical.
            # Livox v1 and v2 CustomMsg are binary compatible (mostly).
        
        digest = conn.digest
        if not digest:
            if topic == '/livox/lidar':
                digest = 'RIHS01_94041b4794f52c1d81def2989107fc898a62dacb7a39d5dbe80d4b55e538bf6d'
            elif topic == '/livox/imu':
                digest = 'RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b'
            else:
                digest = 'RIHS01_0000000000000000000000000000000000000000000000000000000000000000'

        new_conn = writer.add_connection(
            topic,
            msgtype,
            msgdef=conn.msgdef.data,
            rihs01=digest,
            serialization_format=conn.ext.serialization_format,
            offered_qos_profiles=conn.ext.offered_qos_profiles
        )
        conn_map[conn.id] = new_conn

    for conn, timestamp, data in reader.messages():
        writer.write(conn_map[conn.id], timestamp, data)

print("Bag fixed and saved to", dst_path)
