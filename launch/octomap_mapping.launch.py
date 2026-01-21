import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05,
                'frame_id': 'camera_init',
                'sensor_model.max_range': 100.0,
                'latch': False,
                'filter_ground': False, # We want to project everything for now, or true to clean up
                # 2D Projection parameters
                'occupancy_min_z': 0.1, # Ignore ground noise
                'occupancy_max_z': 2.0, # Ignore ceiling/high objects
            }],
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('projected_map', '/map')
            ]
        )
    ])
