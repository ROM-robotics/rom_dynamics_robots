#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    rom_robot_name = os.environ.get('ROM_ROBOT_MODEL', 'rom2109')
    use_lidar = LaunchConfiguration('use_lidar')
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(f'{rom_robot_name}_description'), "urdf", f'{rom_robot_name}_hw_control.urdf.xacro']
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(f'{rom_robot_name}_controller'),
            "config",
            f'{rom_robot_name}_controllers.yaml',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("stm32f1_system_interface"), "rviz", "diffbot.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    sed_command_false = ExecuteProcess(
        cmd=[
            'sed', '-i', 
            's/enable_odom_tf: true/enable_odom_tf: false/g', 
            f'/home/mr_robot/Desktop/Git/rom_dynamics_robots/drivers/stm32f103_driver/{rom_robot_name}_controller/config/{rom_robot_name}_controllers.yaml'
        ],
        output='screen',
        # cmd=['ros2', 'param', 'set', '/diff_cont', 'enagle_odom_tf', 'True'],
        condition=UnlessCondition(LaunchConfiguration('odom_tf'))
    )
    sed_command_true = ExecuteProcess(
        cmd=[
            'sed', '-i', 
            's/enable_odom_tf: false/enable_odom_tf: true/g', 
            f'/home/mr_robot/Desktop/Git/rom_dynamics_robots/drivers/stm32f103_driver/{rom_robot_name}_controller/config/{rom_robot_name}_controllers.yaml'
        ],
        output='screen',
        # cmd=['ros2', 'param', 'set', '/diff_cont', 'enagle_odom_tf', 'False'],
        condition=IfCondition(LaunchConfiguration('odom_tf'))
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"],
    )

    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_base_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[base_controller_spawner],
        )
    )

    twist_mux_params = os.path.join(get_package_share_directory(f'{rom_robot_name}_controller'), 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_controller/cmd_vel_unstamped')]
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(f'{rom_robot_name}_controller'), 'launch', 'rplidar.launch.py')]),
        launch_arguments={'use_lidar': use_lidar}.items(),
        condition=IfCondition(use_lidar)
    )

    nodes = [
        DeclareLaunchArgument('use_lidar', default_value='false', description='Use lidar or Not.'),
        control_node,
        sed_command_false,
        sed_command_true,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_base_controller_spawner_after_joint_state_broadcaster_spawner,
        twist_mux_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        rplidar_launch,
    ]

    return LaunchDescription(nodes)
