import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # rviz_config = os.path.join(
    #     get_package_share_directory("turtle_tf2_py"), "rviz", "turtle_rviz.rviz"
    # )
    config = os.path.join(
        get_package_share_directory("amd_sevtsv"), "config", "params.yaml"
    )
    # print("config", config)

    servercontrol_node = Node(
        package="amd_sevtsv",
        # namespace="minhdeptria",
        executable="query_db",
        # name="sim",
        parameters=[config],
    )

    mission_control_node = Node(
        package="amd_sevtsv",
        # namespace="minhdeptria",
        executable="mission_control",
        # name="sim",
        # parameters=[config],
    )

    # mockup_node = Node(
    #     package="amd_sevtsv",
    #     # namespace="minhdeptria",
    #     executable="mockup",
    #     # name="sim",
    #     # parameters=[config],
    # )

    return LaunchDescription([servercontrol_node])
