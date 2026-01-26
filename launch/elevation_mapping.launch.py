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
                {'grid_resolution': 0.1},
                {'map_size_x': 100.0},
                {'map_size_y': 100.0},
                {'robot_height': 2.0},
                {'obstacle_height_thresh': 0.3},
                {'input_topic': '/cloud_registered'},
                {'map_frame_id': 'camera_init'}
            ]
        )
    ])
