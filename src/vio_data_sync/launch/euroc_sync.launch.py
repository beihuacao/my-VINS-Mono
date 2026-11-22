from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vio_data_sync",
            executable="data_sync_node",
            name="vio_data_sync",
            output="screen",
            parameters=[{
                "image_topic": "/cam0/image_raw",
                "imu_topic": "/imu0",
                "time_offset": 0.0,
            }],
        )
    ])
