#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='ros2_serial_bridge',
            executable='serial_bridge',   # <-- 改成 setup.py 里定义的 entry point 名
            name='ros2_serial_bridge',
            output='screen',
            parameters=[
                {
                    'port': '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D3091391-if00-port0',   # 根据你的设备调整
                    'baud': 57600
                }
            ]
        )

    ])

