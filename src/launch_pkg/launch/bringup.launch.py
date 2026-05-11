#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    return LaunchDescription([
        gps_launch,        # GPS 节点
        imu_launch,        # IMU 节点
        serial_launch,     # 数传节点，entry_point 可执行名字改为 serial_bridge
        fusion_launch,     # 数据处理节点
        gimbal_launch,     # 转台节点
    ])

