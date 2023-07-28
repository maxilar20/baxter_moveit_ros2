from moveit_configs_utils import MoveItConfigsBuilder


from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "baxter", package_name="baxter_moveit_ros2"
    ).to_moveit_configs()

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    ld.add_action(
        Node(
            package="baxter_joint_controller",
            executable="controller",
        )
    )

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("baxter_moveit_ros2", "config/baxter_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch as much as possible in components
    ld.add_action(
        ComposableNodeContainer(
            name="moveit_servo_demo_container",
            namespace="/",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="moveit_servo",
                    plugin="moveit_servo::JoyToServoPub",
                    name="controller_to_servo_node",
                ),
                ComposableNode(
                    package="joy",
                    plugin="joy::Joy",
                    name="joy_node",
                    parameters=[{"deadzone": 0.1}],
                ),
            ],
            output="screen",
        )
    )

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
            output="screen",
        )
    )

    return ld
