import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pinky_line_follower',
            executable='line_follower_node',
            name='line_follower',
            output='screen',
            parameters=[{
                'speed': 0.1,
                'kp': 0.8,
                'kd': 0.2,
                'threshold': 500
            }]
        )
    ])
