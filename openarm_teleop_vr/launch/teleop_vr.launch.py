#!/usr/bin/env python3
"""Launch the OpenArm VR teleop pipeline (bridge + teleop node).

The robot *backend* (what consumes the joint commands and publishes
``/joint_states``) is launched separately and selected with ``backend``:

* ``backend:=isaac`` (default) -- run the Isaac Sim node yourself in another
  terminal with IsaacLab's python (it cannot share this ament process):
      ~/IsaacLab/isaaclab.sh -p \
          ~/ROS_WS/src/openarm_teleop_vr/isaac/openarm_isaac_teleop.py
* ``backend:=real`` -- bring up OpenArm's ros2_control + forward-position
  controllers (separate openarm bringup repo) on the same command topics.

This launch only starts the parts that live in this package.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_urdf = "/home/student/ROS_WS/openarm_v10_bimanual.urdf"

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("openarm_teleop_vr"), "config", "teleop_params.yaml"
        ]),
        description="Path to teleop parameter YAML",
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf_path", default_value=default_urdf,
        description="Flat OpenArm bimanual URDF for Pinocchio IK",
    )
    backend_arg = DeclareLaunchArgument(
        "backend", default_value="isaac", choices=["isaac", "real"],
        description="Robot backend (informational; backend is started separately)",
    )
    bridge_arg = DeclareLaunchArgument(
        "start_bridge", default_value="true",
        description="Also start the C++ UDP->ROS bridge",
    )

    bridge_node = Node(
        package="openarm_teleop_bridge_vr",
        executable="openarm_teleop_bridge_vr_node",
        name="openarm_teleop_bridge_vr_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_bridge")),
    )

    teleop_node = Node(
        package="openarm_teleop_vr",
        executable="openarm_teleop_vr_node",
        name="openarm_teleop_vr_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"urdf_path": LaunchConfiguration("urdf_path")},
        ],
    )

    # For the Isaac backend, also start the ROS<->sim UDP relay (the Isaac process
    # itself cannot run rclpy due to a Python 3.11 vs 3.10 mismatch).
    isaac_relay = Node(
        package="openarm_teleop_vr",
        executable="openarm_isaac_relay",
        name="openarm_isaac_relay",
        output="screen",
        condition=LaunchConfigurationEquals("backend", "isaac"),
    )

    return LaunchDescription([
        config_arg, urdf_arg, backend_arg, bridge_arg,
        LogInfo(msg=["Teleop backend = ", LaunchConfiguration("backend"),
                     " (start the Isaac/real backend process separately; see header)"]),
        bridge_node,
        teleop_node,
        isaac_relay,
    ])
