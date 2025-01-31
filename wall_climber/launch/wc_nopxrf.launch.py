import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from ament_index_python import get_package_share_directory

urdf_pkg = FindPackageShare("sally_description")
urdf_path = LaunchConfiguration("urdf_path")
rviz_path = LaunchConfiguration("rviz_path")

robot_description = ParameterValue(
    Command(["xacro ", urdf_path, " ", LaunchConfiguration("xacro_args")]),
    value_type=str,
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=os.path.join(
                    get_package_share_directory("sally_description"),
                    "urdf/sally.urdf.xacro",
                ),
                description="Robot description file",
            ),
            DeclareLaunchArgument(
                "xacro_args",
                default_value="",
                description="Xacro arguments (e.g. 'param:=value')",
            ),
            DeclareLaunchArgument(
                "rviz_path",
                default_value=os.path.join(
                    get_package_share_directory("wall_climber"),
                    "rviz/sally.rviz",
                ),
                description="Rviz configuration",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                respawn=True,
                parameters=[{"dev": "/dev/input/js0"}],
            ),
            ExecuteProcess(
                cmd=[
                    "xterm",
                    "-T",
                    "Sally",
                    "-fa",
                    "Monospace",
                    "-fs",
                    "15",
                    "-e",
                    "ros2",
                    "run",
                    "wall_climber",
                    "main",
                ],
                output="screen",
            ),
            Node(
                package="microstrain_inertial_driver",
                executable="microstrain_inertial_driver_node",
                name="microstrain_inertial_driver",
                output="log",
                parameters=[
                    os.path.join(
                        get_package_share_directory("wall_climber"),
                        "config/microstrain_inertial_params.yml",
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_path]
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
        ]
    )
