from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_driver',
            executable='imu_node',
            name='imu_publisher',
            output='screen',
            parameters=[
                {'port': '/dev/serial/by-id/usb-Silicon_Labs_HandsFree_IMU_USB_to_UART_Bridge_Controller_0001-if00-port0'},
                {'baudrate': 921600}
            ]
        )
    ])

