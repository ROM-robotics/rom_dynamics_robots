#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import RegisterEventHandler,  TimerAction


def generate_launch_description():
    rom_robot_name = os.environ.get('ROM_ROBOT_MODEL', 'rom2109')

    switch_node = Node(
        name="switch",
        package=f'{rom_robot_name}_switch',
        executable="switch_mode.py",
        output='screen',
    )

    tk_node = Node(
        package=f'{rom_robot_name}_switch',
        executable='tk.py',
        name='tk',
        output='screen',
    )

    trigger_node = Node(
        package=f'{rom_robot_name}_switch',
        executable='first_time_trigger',
        name='first_time_trigger_node',
        output='screen',
    )

    # Delay start of tk_node after `switch_node`
    delay_tk_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=switch_node,
            on_start=[tk_node],
        )
    )

    delayed_trigger_time_node = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[trigger_node]
    )

    return LaunchDescription(
        [
            switch_node,
            delay_tk_node,
            delayed_trigger_time_node,
        ]
    )

