import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Path to the rover URDF
    urdf_file = os.path.expanduser(
        "~/vslam_ws/src/rover_description/urdf/rover.urdf"
    )

    with open(urdf_file, "r") as infp:
        robot_description = infp.read()

    # RViz argument
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2"
        ),

        # Publish joint states
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
        ),

        # Publish robot TF from URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description
                }
            ]
        ),

        # Start RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz)
        ),
    ])
