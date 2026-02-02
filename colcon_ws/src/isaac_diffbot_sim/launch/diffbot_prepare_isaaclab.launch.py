"""
Launch file for preparing diffbot for Isaac Lab.

This launch file:
1. Processes the xacro file to generate URDF
2. Generates ArticulationCfg for Isaac Lab
3. Converts URDF to USD (via Isaac Lab, REST API, or Isaac Sim)

Usage:
    ros2 launch isaac_diffbot_sim diffbot_prepare_isaaclab.launch.py

    # With custom output directory
    ros2 launch isaac_diffbot_sim diffbot_prepare_isaaclab.launch.py output_dir:=/path/to/output

    # With REST API server running in Isaac Lab container
    ros2 launch isaac_diffbot_sim diffbot_prepare_isaaclab.launch.py api_url:=http://localhost:8081
"""

import os
import pathlib

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Get package directories
    diffbot_description_path = get_package_share_directory('diffbot_description')

    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='',
        description='Output directory for generated files (default: same as URDF)'
    )

    fixed_base_arg = DeclareLaunchArgument(
        'fixed_base',
        default_value='false',
        description='Whether to fix the robot base to the world'
    )

    api_url_arg = DeclareLaunchArgument(
        'api_url',
        default_value='http://localhost:8081',
        description='REST API URL for USD conversion (if not running in Isaac Lab environment)'
    )

    # Process xacro to generate URDF
    xacro_file = os.path.join(
        diffbot_description_path,
        'robots',
        'diffbot.urdf.xacro'
    )
    urdf_path = os.path.join(diffbot_description_path, 'robots', 'diffbot.urdf')

    # Process xacro with use_sim:=true to include Isaac-specific tags
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    # Write URDF file
    with open(urdf_path, 'w') as f:
        f.write(robot_desc)

    print(f'[diffbot_prepare_isaaclab] Generated URDF: {urdf_path}')

    # Prepare robot for Isaac Lab node
    prepare_robot_node = Node(
        package='isaac_ros2_scripts',
        executable='prepare_robot_for_isaaclab',
        name='prepare_robot_for_isaaclab',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path,
            'output_dir': LaunchConfiguration('output_dir'),
            'fixed_base': LaunchConfiguration('fixed_base'),
            'api_url': LaunchConfiguration('api_url'),
        }],
    )

    return LaunchDescription([
        output_dir_arg,
        fixed_base_arg,
        api_url_arg,
        prepare_robot_node,
    ])
