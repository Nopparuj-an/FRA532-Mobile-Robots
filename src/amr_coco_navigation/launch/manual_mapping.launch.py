#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    config_dir = os.path.join(
        get_package_share_directory("amr_coco_navigation"), "config"
    )
    config_file = os.path.join(config_dir, "mapper_params_online_sync.yaml")

    rviz_config_dir = os.path.join(config_dir, "rviz")
    rviz_config_file = os.path.join(rviz_config_dir, "mapping.rviz")

    launch_description = LaunchDescription()

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="sync_slam_toolbox_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, config_file],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    launch_description.add_action(declare_use_sim_time)
    launch_description.add_action(slam_toolbox)
    # launch_description.add_action(rviz)

    return launch_description
