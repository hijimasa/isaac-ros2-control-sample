"""
Launch file for Isaac Lab REST API server.

This launch file starts the REST API server for Isaac Lab control.

Usage:
    ros2 launch isaac_diffbot_sim isaaclab_api.launch.py

    # With custom port
    ros2 launch isaac_diffbot_sim isaaclab_api.launch.py port:=8082

API Endpoints (default: http://localhost:8081):
    GET  /health                - Health check
    POST /convert_urdf          - Convert URDF to USD
    POST /generate_config       - Generate ArticulationCfg
    POST /prepare_robot         - Combined conversion + config generation
    POST /training/config       - Set training configuration
    GET  /training/config       - Get training configuration
    POST /training/start        - Start training
    POST /training/stop         - Stop training
    GET  /training/status       - Get training status

API Documentation:
    http://localhost:8081/docs (Swagger UI)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Host address for REST API server'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8081',
        description='Port number for REST API server'
    )

    # Isaac Lab REST API node
    isaaclab_api_node = Node(
        package='isaac_ros2_scripts',
        executable='isaaclab_api',
        name='isaaclab_api',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
        }],
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        isaaclab_api_node,
    ])
