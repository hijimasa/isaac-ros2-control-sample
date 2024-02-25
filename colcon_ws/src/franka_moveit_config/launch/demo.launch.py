from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

import os
import pathlib
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


import xacro

def generate_launch_description():
    # Command-line arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        #default_value="mock_components",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="franka_moveit_config")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    tutorial_mode = LaunchConfiguration("rviz_tutorial")
    rviz_base = os.path.join(
        get_package_share_directory("franka_moveit_config"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_empty_config = os.path.join(rviz_base, "moveit_empty.rviz")
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(tutorial_mode),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=UnlessCondition(tutorial_mode),
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("franka_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"],
    )

    isaac_franka_moveit_config_path = os.path.join(
        get_package_share_directory('franka_moveit_config'))

    xacro_file = os.path.join(isaac_franka_moveit_config_path,
                              'config',
                              'panda.urdf.xacro')
    urdf_path = os.path.join(isaac_franka_moveit_config_path, 'config', 'panda.urdf')
    # xacroをロード
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()
    
    panda_description_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_description'))
    urdf_model_path = os.path.join(panda_description_path, 'urdf', 'panda.urdf')

    isaac_spawn_robot = Node(
        package="isaac_ros2_scripts",
        executable="spawn_robot",
        parameters=[{'urdf_path': str(urdf_model_path),
                    'x' : 0.0,
                    'y' : 0.0,
                    'z' : 0.0,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : 0.0,
                    'fixed': True,
                    }],
    )

    isaac_prepare_robot_controller = Node(
        package="isaac_ros2_scripts",
        executable="prepare_robot_controller",
        parameters=[{'urdf_path': str(urdf_path)}],
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=isaac_spawn_robot,
                    on_exit=[isaac_prepare_robot_controller],
                )
            ),
            tutorial_arg,
            ros2_control_hardware_type,
            rviz_node,
            rviz_node_tutorial,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            hand_controller_spawner,
            isaac_spawn_robot,
        ]
    )
