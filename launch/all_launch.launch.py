import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

package_name = "amd_sevtsv"


def generate_launch_description():

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch/load_token.launch.py",
            )
        )
    )
    # mockup_node = Node(
    #     package="amd_sevtsv",
    #     # namespace="minhdeptria",
    #     executable="mockup",
    #     # name="sim",
    #     # parameters=[config],
    # )

    return LaunchDescription([launch_include])
