"""ALLSENSE_TOKEN=… ros2 launch allsense_ros2 bridge.launch.py url:="http://<phone>:8765/" [token_file:=…] [config:=my.yaml]

The token goes in $ALLSENSE_TOKEN or a file (token_file), not in url: ROS parameters are readable by
anyone on the DDS domain, and launch arguments show in the process list."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bridge(context):
    params = {"url": LaunchConfiguration("url").perform(context)}
    token_file = LaunchConfiguration("token_file").perform(context)
    if token_file:  # otherwise the parameter file's token_file (if any) applies
        params["token_file"] = token_file
    return [Node(
        package="allsense_ros2",
        executable="bridge",
        name="allsense_bridge",
        output="screen",
        parameters=[LaunchConfiguration("config").perform(context), params],
    )]


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("allsense_ros2"), "config", "bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("url", description="The phone's address from the app's link (Home → Connect), without ?token=…"),
        DeclareLaunchArgument("token_file", default_value="",
                              description="File holding the pairing token (or set $ALLSENSE_TOKEN)"),
        DeclareLaunchArgument("config", default_value=default_config, description="Parameter file"),
        OpaqueFunction(function=_bridge),
    ])
