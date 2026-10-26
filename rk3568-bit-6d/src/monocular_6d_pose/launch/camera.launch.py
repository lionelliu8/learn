from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="monocular_6d_pose",
            executable="pose_node",
            output="screen",
            parameters=[os.path.join(get_package_share_directory("monocular_6d_pose"), "config", "pose.yaml")],
        )
    ])
