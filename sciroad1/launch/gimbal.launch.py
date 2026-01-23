from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sciroad1',
            executable='gimbal_control_node',
            name='gimbal_control_node',
            output='screen',
            parameters=[
                {'acc': 2.0},
                {'dec': 2.0},
                {'vel': 3.0},
                {'vel_limit': 5.0}  # ⚠️ 新增最大角速度参数
            ]
        )
    ])

