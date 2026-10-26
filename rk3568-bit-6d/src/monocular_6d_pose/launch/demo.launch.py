from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="monocular_6d_pose", executable="synthetic_camera_node", output="screen"),
        Node(
            package="monocular_6d_pose",
            executable="pose_node",
            output="screen",
            parameters=[{"marker_length": 0.08, "marker_id": 0}],
        ),
    ])
