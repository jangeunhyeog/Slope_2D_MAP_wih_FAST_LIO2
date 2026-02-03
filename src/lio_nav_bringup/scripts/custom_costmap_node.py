#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import numpy as np
import cv2

class CustomCostmapNode(Node):
    def __init__(self):
        super().__init__('custom_costmap_node')
        
        # Parameters to mimic Nav2 Inflation Layer
        self.declare_parameter('inflation_radius', 1.0) # Meters
        self.declare_parameter('cost_scaling_factor', 5.0) # Decay rate
        self.declare_parameter('robot_radius', 0.3) # Inscribed radius (Meters)
        self.declare_parameter('roi_radius', 15.0) # Meters (Only inflate within this radius from robot)

        # QoS to drop old messages
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            qos
        )
        self.pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Custom Costmap Node Started (ROI Optimized)")

    def map_callback(self, msg):
        try:
            width = msg.info.width
            height = msg.info.height
            res = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            
            # Get Robot Position
            try:
                # Assuming map frame is camera_init and robot is body
                # We need robot pose in Map Frame
                trans = self.tf_buffer.lookup_transform(msg.header.frame_id, 'body', rclpy.time.Time())
                rx_m = trans.transform.translation.x
                ry_m = trans.transform.translation.y
            except Exception as e:
                # If cannot get TF, default to center of map or skip (warn periodically)
                self.get_logger().warn(f"Could not get robot pose: {str(e)}", throttle_duration_sec=2.0)
                return

            # Grid Coords
            gx = int((rx_m - origin_x) / res)
            gy = int((ry_m - origin_y) / res)

            # ROI Grid Bounds
            roi_r_m = self.get_parameter('roi_radius').value
            roi_r_px = int(roi_r_m / res)
            
            x1 = max(0, gx - roi_r_px)
            x2 = min(width, gx + roi_r_px)
            y1 = max(0, gy - roi_r_px)
            y2 = min(height, gy + roi_r_px)
            
            if x2 <= x1 or y2 <= y1:
                return # Out of bounds or empty

            # 1. Convert to Numpy Array
            # We enforce strict 0 (Free), 100 (Occupied), -1 (Unknown)
            # This prevents "weird colors" from intermediate values in source map
            grid_full = np.array(msg.data, dtype=np.int8).reshape((height, width))
            
            # Initialize Output with -1 (Unknown)
            cost_grid = np.full((height, width), -1, dtype=np.int8)

            # Process ONLY the ROI
            roi_grid_raw = grid_full[y1:y2, x1:x2]
            
            # Create a Cleaned Version of ROI
            # Rule: -1 stays -1. >= 50 becomes 100. Else 0.
            roi_grid_clean = np.zeros_like(roi_grid_raw, dtype=np.int8)
            
            mask_unknown = (roi_grid_raw == -1)
            mask_lethal_input = (roi_grid_raw >= 50) # Threshold for obstacles
            
            roi_grid_clean[mask_unknown] = -1
            roi_grid_clean[mask_lethal_input] = 100
            # Others remain 0 (Free)
            
            # --- HOLE FILLING (Morphological Closing) ---
            # Goal: Fill small -1s (unknown) that are surrounded by 0s (free)
            # 1. Create a binary mask of Free Space (0)
            # We want to expand free space into small unknown pockets.
            # 1 = Free, 0 = Not Free (Obstacle or Unknown)
            # Wait, closing operation fills "black holes" in "white objects".
            # If we want to fill Unknown (-1) with Free (0), we should treat Free as the foreground (1).
            
            mask_free_space = (roi_grid_clean == 0).astype(np.uint8)
            
            # 2. Apply Closing (Dilation then Erosion)
            # This connects free space regions, bridging over small unknown gaps.
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask_free_closed = cv2.morphologyEx(mask_free_space, cv2.MORPH_CLOSE, kernel)
            
            # 3. Update roi_grid_clean
            # Where the closed mask says "Free" (1) BUT the original map was "Unknown" (-1),
            # we update it to Free (0).
            # Safety: Do NOT overwrite Obstacles (100).
            # The mask_free_closed might have expanded into obstacles if we are not careful?
            # Closing generally preserves shape but fills holes.
            # However, if free space surrounds an obstacle, closing shouldn't eat the obstacle unless it's smaller than kernel.
            # To be safe: Only update if original was -1.
            
            fill_candidates = (mask_free_closed == 1) & (roi_grid_clean == -1)
            roi_grid_clean[fill_candidates] = 0
            # --------------------------------------------
            
            # Prepare Binary for Distance Transform (0=Obstacle, 1=Free)
            # We treat Unknown as Free for distance transform safety? 
            # Or ignore? Standard is: Inflate obstacles.
            binary_map = np.ones_like(roi_grid_clean, dtype=np.uint8)
            binary_map[roi_grid_clean == 100] = 0 # Obstacles are 0
            
            # Check if there are any obstacles in ROI
            if np.any(binary_map == 0):
                # Run Distance Transform
                dist_in_pixels = cv2.distanceTransform(binary_map, cv2.DIST_L2, 3)
                dist_m = dist_in_pixels * res

                # Inflation Params
                robot_radius = self.get_parameter('robot_radius').value
                inflation_radius = self.get_parameter('inflation_radius').value
                scaling_factor = self.get_parameter('cost_scaling_factor').value

                # Masking
                mask_lethal_inflated = dist_m <= robot_radius
                mask_inflated = (dist_m > robot_radius) & (dist_m <= inflation_radius)

                factor = np.exp(-1.0 * scaling_factor * (dist_m[mask_inflated] - robot_radius))
                inflated_values = (factor * 98).astype(np.int8) # Max 98 to correspond to Inscribed-1

                # Construct Final ROI Slice
                # Start with the clean grid
                # cost_roi_slice = roi_grid_clean.copy() # <--- REPLACED: Do not copy free space (0)
                # Instead, start with -1 (which it is, because cost_grid is initialized to -1)
                # But we need to construct the slice in a temp array to merge layers
                
                # Create a temporary slice filled with -1 (transparent)
                cost_roi_slice = np.full_like(roi_grid_clean, -1)
                
                # Apply Lethal Inflation (Robot Radius) - Overwrite anything
                # USER REQUEST: Wants obstacles to be Black.
                # The Best way to achieve this with "Costmap" scheme is to make them TRANSPARENT (-1)
                # so the underlying black map shows through.
                # cost_roi_slice[mask_lethal_inflated] = 100
                
                # Apply Obstacles from Input
                # cost_roi_slice[roi_grid_clean == 100] = 100
                
                # Apply Gradient Inflation
                # We want to overlay inflation on top of transparent space
                # But we must NOT overwrite Lethal (which is now transparent -1)
                
                inflation_layer = np.zeros_like(roi_grid_clean, dtype=np.int8)
                inflation_layer[mask_inflated] = inflated_values
                
                # Logic:
                # We want to write inflation ONLY in the inflation zone.
                # We do NOT touch the lethal zone (mask_lethal_inflated) so it stays -1.
                
                # mask_inflated is strictly outside robot_radius (> radius).
                # mask_lethal is inside (<= radius).
                # So they are disjoint.
                
                # Simply apply inflation where mask_inflated is true.
                # Since cost_roi_slice is initialized to -1, this is perfect.
                
                # Check for overlaps with roi_grid_clean? 
                # roi_grid_clean obstacles (100) are inside lethal zone. So they stay -1.
                
                cost_roi_slice[mask_inflated] = inflated_values

                # Put back into the global grid
                # Since global grid is -1, and we want to preserve that...
                # But wait, cost_roi_slice now has -1s for free space.
                # So we can just assign it directly.
                cost_grid[y1:y2, x1:x2] = cost_roi_slice
            else:
                # No obstacles.
                # If we want transparency, we return ALL -1s.
                # But wait, if there are NO obstacles, do we show nothing?
                # Yes, for an overlay, free space is transparent.
                # So if no obstacles, the whole ROI remains -1 (as initialized).
                pass 
                
            # 5. Publish
            out_msg = OccupancyGrid()
            out_msg.header = msg.header
            out_msg.info = msg.info
            out_msg.data = cost_grid.flatten().tolist()
            
            self.pub.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate costmap: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CustomCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
