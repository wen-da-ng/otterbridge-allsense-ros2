"""ros2 launch allsense_ros2 bridge.launch.py url:="http://<phone>:8765/?token=…" [config:=my.yaml]"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("allsense_ros2"), "config", "bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("url", description="Link from the app (Home → Connect) (includes the token)"),
        DeclareLaunchArgument("config", default_value=default_config, description="Parameter file"),
        Node(
            package="allsense_ros2",
            executable="bridge",
            name="allsense_bridge",
            output="screen",
            parameters=[LaunchConfiguration("config"), {"url": LaunchConfiguration("url")}],
        ),
    ])
