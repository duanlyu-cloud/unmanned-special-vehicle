#!/usr/bin/env python3
"""
Launch chassis controller with waypoints config.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    chassis_share = get_package_share_directory("chassis_control")
    waypoints_yaml = os.path.join(chassis_share, "config", "waypoints.yaml")

    chassis_node = Node(
        package="chassis_control",
        executable="chassis_controller_node.py",
        name="chassis_controller",
        output="screen",
        parameters=[{"waypoints_yaml_path": waypoints_yaml}],
    )

    return LaunchDescription([chassis_node])
