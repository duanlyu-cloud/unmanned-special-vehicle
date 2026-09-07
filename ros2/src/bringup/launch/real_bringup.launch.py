import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    calibrate = LaunchConfiguration("calibrate")
    serial_port = LaunchConfiguration("serial_port")

    # robot_state_publisher (URDF)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ar_hardware_interface"), "urdf", "ar.urdf.xacro"]
            ),
            " ",
            "name:=ar",
            " serial_port:=",
            serial_port,
            " calibrate:=",
            calibrate,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control controller_manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            os.path.join(
                get_package_share_directory("ar_hardware_interface"),
                "config",
                "controllers.yaml",
            ),
        ],
    )

    # Load joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Load joint_trajectory_controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # MoveIt move_group + RViz
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ar_moveit_config"), "/launch", "/ar_moveit.launch.py"]
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
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
            {"use_sim_time": False},
            {"poses_yaml_path": poses_yaml},
            {"robot_description_semantic": robot_description_semantic_content},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "calibrate",
            default_value="True",
            description="Calibrate the robot on startup",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for Teensy 4.1",
        ),
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        moveit_launch,
        task_manager_node,
    ])
