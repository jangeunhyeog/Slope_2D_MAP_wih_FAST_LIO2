import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Paths
    fast_lio_path = get_package_share_directory('fast_lio')
    nav2_bringup_path = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('lio_nav_bringup')
    
    # Configuration (local)
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_compatibility = LaunchConfiguration('bag_compatibility')
    
    # Arguments
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')
        
    declare_bag_compat = DeclareLaunchArgument(
        'bag_compatibility', default_value='false',
        description='Enable legacy bag compatibility node')

    # 1. Fast-LIO Mapping
    # We include mapping.launch.py but ensure RViz is disabled so we can spawn our own or Nav2's
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_path, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'false' # We will manage RViz centrally
        }.items()
    )

    # 2. Elevation Mapping
    elevation_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_path, 'launch', 'elevation_mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # 3. Nav2 Bringup (Disabled by user request)
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup_path, 'launch', 'navigation_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'params_file': nav2_params_file
    #     }.items()
    # )

    # 4. Livox Republisher (Optional)
    republisher_node = Node(
        package='lio_nav_bringup',
        executable='livox_republisher.py',
        output='screen',
        condition=IfCondition(bag_compatibility),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. RViz (Nav2 Default or Custom?)
    # Let's use Nav2's RViz config as a base, or just run RViz with a custom view if we had one.
    # For now, let's use the one from Fast-LIO but we might need to add Nav2 plugins manually in UI.
    # Better: Use Nav2's rviz launch which is usually 'rviz_launch.py' inside nav2_bringup
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(fast_lio_path, 'rviz', 'fastlio.rviz')], # Start with FastLIO view
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_sim_time,
        declare_bag_compat,
        fast_lio_launch,
        elevation_mapping_launch,
        # nav2_launch,
        republisher_node,
        rviz_node
    ])
