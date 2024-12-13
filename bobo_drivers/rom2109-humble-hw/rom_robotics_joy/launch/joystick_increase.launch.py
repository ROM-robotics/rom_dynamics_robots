from launch import LaunchDescription
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
    )
    test_node = Node(
        package='rom_robotics_joy',
        executable='joy',
        name='increase_joy',
        parameters=[
                {
                    'linear_increase_button': 4,  # "Y" button
                    'linear_decrease_button': 0,  # "X" button
                    'rotate_increase_button': 6,  # "Y" button
                    'rotate_decrease_button': 7,  # "X" button
                    'axis_linear': 1,      # Axis for linear motion
                    'axis_angular': 2      # Axis for angular motion
                }
            ],
        output = 'screen'
    )

    return LaunchDescription([
        joy_node,
        test_node,
    ])