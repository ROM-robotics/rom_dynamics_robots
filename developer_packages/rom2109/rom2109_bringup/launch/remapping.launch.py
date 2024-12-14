from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rom2109_bringup',
            executable='remapping',
            name='remapping_mode',
            output='screen'
        )
    ])
