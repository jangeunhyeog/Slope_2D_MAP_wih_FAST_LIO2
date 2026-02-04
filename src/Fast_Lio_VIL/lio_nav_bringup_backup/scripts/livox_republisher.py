#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Ideally we would import this, but if the package is deleted, we can't.
# However, the user might still have old bags. 
# Problem: We DELETED livox_ros_driver (v1) earlier.
# If we need this script, we need the v1 message definitions.
# BUT, we can't import what doesn't exist.
# Strategy: If the user needs to run old bags, they will need the v1 package back.
# For now, we will create a placeholder or a generic remapper IF the types are available.
# Actually, assuming the user might revert or re-add it, let's keep the code simple.
# If livox_ros_driver is missing, this script will fail to import.
# We will wrap imports in try-except to be safe.

try:
    from livox_ros_driver.msg import CustomMsg as CustomMsgV1
except ImportError:
    CustomMsgV1 = None

from livox_ros_driver2.msg import CustomMsg as CustomMsgV2
from livox_ros_driver2.msg import CustomPoint as CustomPointV2

class LivoxRepublisher(Node):
    def __init__(self):
        super().__init__('livox_republisher')
        
        if CustomMsgV1 is None:
            self.get_logger().error("livox_ros_driver (v1) not found! Cannot republish legacy messages.")
            # We exit or just spin doing nothing? Better to warn.
            return

        self.sub = self.create_subscription(
            CustomMsgV1,
            '/livox/lidar',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            CustomMsgV2,
            '/livox/lidar_v2',
            10
        )
        self.get_logger().info('Livox Republisher Started: /livox/lidar (v1) -> /livox/lidar_v2 (v2)')

    def callback(self, msg_v1):
        msg_v2 = CustomMsgV2()
        msg_v2.header = msg_v1.header
        msg_v2.timebase = msg_v1.timebase
        msg_v2.point_num = msg_v1.point_num
        msg_v2.lidar_id = msg_v1.lidar_id
        msg_v2.rsvd = msg_v1.rsvd
        
        # Convert points
        msg_v2.points = []
        for p_v1 in msg_v1.points:
            p_v2 = CustomPointV2()
            p_v2.offset_time = p_v1.offset_time
            p_v2.x = p_v1.x
            p_v2.y = p_v1.y
            p_v2.z = p_v1.z
            p_v2.reflectivity = p_v1.reflectivity
            p_v2.tag = p_v1.tag
            p_v2.line = p_v1.line
            msg_v2.points.append(p_v2)
            
        self.pub.publish(msg_v2)

def main(args=None):
    rclpy.init(args=args)
    node = LivoxRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
