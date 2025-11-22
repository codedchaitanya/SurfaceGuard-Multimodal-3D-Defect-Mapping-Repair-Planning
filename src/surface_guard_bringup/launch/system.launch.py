import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get Package Paths
    perception_pkg = get_package_share_directory('surface_guard_perception')
    bringup_pkg = get_package_share_directory('surface_guard_bringup')

    # 2. Define Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Path to the config file inside the perception package
    config_file = os.path.join(perception_pkg, 'config', 'perception_params.yaml')
    rviz_config_file = os.path.join(bringup_pkg, 'rviz', 'debug_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # --- NODE 1: The Geometric Processor (C++) ---
        Node(
            package='surface_guard_perception',
            executable='wall_processor_node',
            name='wall_processor',
            output='screen',
            parameters=[config_file],
            # If using a bag file that publishes to a different topic, remap here:
            # remappings=[('/camera/depth/color/points', '/my_bag_topic')]
        ),

        # --- NODE 2: The Visual Detector (Python/PyTorch) ---
        Node(
            package='surface_guard_perception',
            executable='crack_detector_node.py',
            name='crack_detector',
            output='screen',
            parameters=[config_file]
        ),

        # --- NODE 3: Visualization (RViz2) ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        
        # Note: We are NOT launching the camera driver here. 
        # It is better to run the camera driver or play a rosbag in a separate terminal.
    ])