from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
import os
import sys

def generate_launch_description():
    description_launch_file_dir = os.path.join(
        get_package_share_directory("amr_coco_description"), "launch"
    )
    
    realsense_launch_file_dir = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch"
    )

    rplidar_launch_file_dir = os.path.join(
        get_package_share_directory("rplidar_ros"), "launch"
    )

    visualize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [description_launch_file_dir, "/description.launch.py"]
        ),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [realsense_launch_file_dir, "/rs_launch.py"]
        ),
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [rplidar_launch_file_dir, "/rplidar.launch.py"]
        ),
    )
    # localize_launch_file_dir = os.path.join(
    #     get_package_share_directory("robot_localization"), "launch"
    # )

    # localize = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [localize_launch_file_dir, "/ekf.launch.py"]
    #     ),
    # )


    pub_odom = Node(
        package='amr_coco',
        executable='odom_publisher.py',
        name='odom_publisher',
        # remappings={("/odom", "/example/odom"),("/tf", "raw_transform")},
    )

    launch_description = LaunchDescription()
    launch_description.add_action(pub_odom)
    launch_description.add_action(visualize)
    launch_description.add_action(realsense)
    launch_description.add_action(rplidar)

    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()