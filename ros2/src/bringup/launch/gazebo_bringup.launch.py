import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    # Gazebo + robot spawn + controllers
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ar_gazebo"), "/launch", "/ar_gazebo.launch.py"]
        ),
    )

    # MoveIt move_group + RViz
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ar_moveit_config"), "/launch", "/ar_moveit.launch.py"]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Task manager node
    task_control_share = get_package_share_directory("task_control")
    poses_yaml = os.path.join(task_control_share, "config", "poses.yaml")

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ar_moveit_config"), "srdf", "ar.srdf.xacro"]
            ),
            " ",
            "name:=ar",
        ]
    )

    task_manager_node = Node(
        package="task_control",
        executable="task_manager_node",
        name="task_manager",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"poses_yaml_path": poses_yaml},
            {"robot_description_semantic": robot_description_semantic_content},
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        task_manager_node,
    ])
