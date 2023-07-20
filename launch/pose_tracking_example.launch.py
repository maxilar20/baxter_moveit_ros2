import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "baxter", package_name="baxter_moveit_ros2"
    ).to_moveit_configs()

    # Get parameters for the Pose Tracking node
    pose_tracker_params = {
        "moveit_servo_baxter": ParameterBuilder("baxter_moveit_ros2")
        .yaml("config/pose_tracking_settings.yaml")
        .to_dict()
    }

    servo_params = {
        "moveit_servo_baxter": ParameterBuilder("baxter_moveit_ros2")
        .yaml("config/baxter_pose_tracking.yaml")
        .to_dict()
    }

    # RViz
    rviz_config_file = (
        get_package_share_directory("baxter_moveit_ros2")
        + "/config/demo_rviz_pose_tracking.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # prefix=['xterm -e gdb -ex run --args'],
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict()],
    )

    # # Publishes tf's for the robot
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[moveit_config.robot_description],
    # )

    # # A node to publish world -> panda_link0 transform
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    # )

    pose_tracking_node = Node(
        package="moveit_servo_baxter",
        executable="servo_pose_tracking_demo",
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen",
        parameters=[moveit_config.to_dict(), servo_params, pose_tracker_params],
    )

    # # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("moveit_resources_panda_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[moveit_config.robot_description, ros2_controllers_path],
    #     output="screen",
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager-timeout",
    #         "300",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # panda_arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["panda_arm_controller", "-c", "/controller_manager"],
    # )

    return LaunchDescription(
        [
            rviz_node,
            # static_tf,
            pose_tracking_node,
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # panda_arm_controller_spawner,
            # robot_state_publisher,
        ]
    )
