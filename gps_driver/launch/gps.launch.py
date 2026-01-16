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
                {'port': '/dev/ttyUSB1'},
                {'baudrate': 115200}
            ]
        )
    ])

