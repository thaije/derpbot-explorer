"""
Launch file for DerpBot autonomous exploration.

Starts:
  1. slam_toolbox (online async) — builds /map from /derpbot_0/scan
  2. Nav2 stack (planner + controller + behaviours + costmaps)
  3. agent_node — frontier explorer + detector + tracker

Usage:
  ros2 launch derpbot_autonomy.launch.py
  ros2 launch derpbot_autonomy.launch.py use_sim_time:=true

TF frames (confirmed from sim):
  map → odom → base_footprint → base_link → camera_link
                                           → lidar_link
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Resolve config dir relative to this launch file's location
_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
_CONFIG_DIR = os.path.join(_LAUNCH_DIR, "..", "config")
_AGENT_DIR = os.path.join(_LAUNCH_DIR, "..", "agent")

SLAM_PARAMS = os.path.join(_CONFIG_DIR, "slam_toolbox_params.yaml")
NAV2_PARAMS = os.path.join(_CONFIG_DIR, "derpbot_nav2_params.yaml")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # --- slam_toolbox: online async mode ---
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[SLAM_PARAMS, {"use_sim_time": use_sim_time}],
        # scan_topic is set directly in slam_toolbox_params.yaml; no remapping needed.
        # slam_toolbox uses TF for odom (not an odom topic subscription).
    )

    # --- Nav2 bringup (navigation only — no AMCL, slam_toolbox provides localisation) ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": NAV2_PARAMS,
            "use_composition": "False",
            "use_respawn": "False",
            # odom topic remapped via params; cmd_vel via collision_monitor output
        }.items(),
    )

    # --- Agent node — starts 5 s after Nav2 to allow costmaps to initialise ---
    agent_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="agent_pkg",
                executable="agent_node",
                name="derpbot_agent",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                # If running as plain Python script (no ROS package install):
                # prefix="python3 " + os.path.join(_AGENT_DIR, "agent_node.py"),
            )
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            slam_toolbox_node,
            nav2_launch,
            agent_node,
        ]
    )
