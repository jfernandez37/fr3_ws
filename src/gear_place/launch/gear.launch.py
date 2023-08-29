from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xacro
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Generate robot description
    urdf = (
        get_package_share_directory("gear_place_description")
        + "/urdf/gear_fr3.urdf.xacro"
    )
    doc = xacro.process_file(urdf)

    robot_description_content = doc.toprettyxml(indent="  ")

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Camera
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch", "/rs_launch.py"]
        ),
        launch_arguments={
            "pointcloud.enable": "true",
            "pointcloud.ordered_pc": "true",
            "pointcloud.stream_filter": "0",
            "depth_module.profile": "'640x480x30'",
        }.items(),
    )

    start_rviz = LaunchConfiguration("rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("gear_place_description"), "rviz", "visualize.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_task_board",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
        condition=IfCondition(start_rviz),
    )

    # Custom Nodes
    robot_commander_node = Node(
        package="gear_place",
        executable="robot_commander_node",
        output="screen",
        parameters=[
            robot_description,
        ],
    )

    conveyor_node = Node(
        package="conveyor_controller", executable="conveyor_controller"
    )

    supervisor = Node(
        package="gear_place",
        executable="gear_place_node.py",
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        conveyor_node,
        # realsense,
        rviz_node,
        robot_commander_node,
        supervisor,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="false", description="start rviz node?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
