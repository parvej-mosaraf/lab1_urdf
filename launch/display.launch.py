#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    pkg_root = Path(__file__).resolve().parents[1]
    urdf_path = pkg_root / "urdf" / "lab1_robot.urdf"

    robot_description = urdf_path.read_text()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen")

    return LaunchDescription([rsp, jsp, rviz])
