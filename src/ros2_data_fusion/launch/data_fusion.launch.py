from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_data_fusion',
            executable='data_fusion_node',
            name='data_fusion_node',
            output='screen'
        )
    ])

