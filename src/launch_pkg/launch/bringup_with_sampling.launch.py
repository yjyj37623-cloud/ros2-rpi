#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gps_driver'),
                'launch',
                'gps.launch.py'
            )
        )
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('imu_driver'),
                'launch',
                'imu.launch.py'
            )
        )
    )

    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_serial_bridge'),
                'launch',
                'serial_bridge.launch.py'
            )
        )
    )

    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_data_fusion'),
                'launch',
                'data_fusion.launch.py'
            )
        )
    )

    gimbal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sciroad1'),
                'launch',
                'gimbal.launch.py'
            )
        )
    )

    sampling_node = Node(
        package='sciroad1',
        executable='keepsampling_node',
        name='keepsampling_node',
        output='screen'
    )

    return LaunchDescription([
        gps_launch,
        imu_launch,
        serial_launch,
        fusion_launch,
        gimbal_launch,
        sampling_node,
    ])
