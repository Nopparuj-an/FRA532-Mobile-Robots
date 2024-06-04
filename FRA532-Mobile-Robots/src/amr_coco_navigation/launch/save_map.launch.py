#!/usr/bin/env python3

import time
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions.node import Node, ExecuteProcess
from launch.substitutions import FindExecutable
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    workspace_path = get_package_share_directory("amr_coco_navigation").split(
        "install/"
    )[0]
    maps_path = workspace_path + "maps"
    map_name = str(time.strftime("%Y-%m-%d-%H-%M-%S"))

    mkdir_maps = ExecuteProcess(
        cmd=[FindExecutable(name="mkdir"), " -p ", maps_path], shell=True
    )
    map_saver_cli = Node(
        package="nav2_map_server",
        executable="map_saver_cli",
        name="map_saver_cli",
        output="screen",
        arguments=["-f", maps_path + "/" + map_name],
        parameters=[{"save_map_timeout": 10000.0}],
    )

    delay_map_saver_cli_after_mkdir_maps = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=mkdir_maps, on_exit=[map_saver_cli])
    )
    return LaunchDescription([mkdir_maps, delay_map_saver_cli_after_mkdir_maps])
