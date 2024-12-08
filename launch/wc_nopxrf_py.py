import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    urdf_path = LaunchConfiguration("urdfpath")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdfpath",
                default_value=os.path.join(
                    get_package_share_directory("sally_description"),
                    "urdf/sally.urdf.xacro",
                ),
                description="Path to the URDF file",
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
                output="screen",
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
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": Command(["xacro ", urdf_path])}],
            ),
        ]
    )
