from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xacro
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Generate robot description
    urdf = get_package_share_directory("gear_place_description") + "/urdf/gear_fr3.urdf.xacro"
    doc = xacro.process_file(urdf)

    robot_description_content = doc.toprettyxml(indent='  ')

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Camera
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch", "/rs_launch.py"]
        )
    )

    # Custom Nodes
    robot_commander_node = Node(
        package="gmcs_panda_task_board",
        executable="robot_commander_node",
        output="screen",
        parameters=[
            robot_description,
        ],
    )

    nodes_to_start = [
        robot_state_publisher,
        realsense,
        robot_commander_node
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])