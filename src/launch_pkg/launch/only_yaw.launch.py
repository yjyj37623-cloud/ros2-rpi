#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 1. GPS 节点
        Node(
            package='gps_driver',
            executable='gps_node',
            name='gps_node',
            output='screen'
        ),

        # 2. 数传节点（only_yaw）
        Node(
            package='ros2_serial_bridge',  # ✅ 这里改对了！
            executable='serial_bridge_yaw_node',
            name='serial_bridge_yaw_node',
            output='screen'
        ),

        # 3. 数据融合节点（only_yaw）
        Node(
            package='ros2_data_fusion',    # ✅ 这里改对了！
            executable='data_fusion_yaw_node',
            name='data_fusion_yaw_node',
            output='screen'
        ),

        # 4. 转台控制节点（only_yaw）
        Node(
            package='sciroad1',
            executable='turntable_control_yaw_node',
            name='turntable_control_yaw_node',
            output='screen'
        ),

    ])

