import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    task_control_share = get_package_share_directory("task_control")
    poses_yaml = os.path.join(task_control_share, "config", "poses.yaml")

    task_manager_node = Node(
        package="task_control",
        executable="task_manager_node",
        name="task_manager",
        output="screen",
        parameters=[{"poses_yaml_path": poses_yaml}],
    )

    return LaunchDescription([task_manager_node])
