from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fast_lio',
            executable='elevation_mapping_node',
            name='elevation_mapping_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'grid_resolution': 0.2},
                {'map_size_x': 100.0},
                {'map_size_y': 100.0},
                {'robot_height': 2.0},
                {'obstacle_height_thresh': 0.3},
                {'max_slope_degree': 30.0},
                {'min_valid_points': 1},
                {'sensor_height': 0.8}, # Set this to your actual LiDAR height (e.g. 0.8)
                {'ceiling_cutoff': 2.0}, # Filter points > robot_z + 1.0m
                {'input_topic': '/cloud_registered'},
                {'map_frame_id': 'camera_init'}
            ]
        )
    ])
