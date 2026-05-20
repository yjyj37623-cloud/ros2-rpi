from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_driver',
            executable='gps_node',
            name='gps_publisher',
            output='screen',
            parameters=[
                {'port': '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CAABb13AL20-if00-port0'},
                {'baudrate': 115200}
            ]
        )
    ])

