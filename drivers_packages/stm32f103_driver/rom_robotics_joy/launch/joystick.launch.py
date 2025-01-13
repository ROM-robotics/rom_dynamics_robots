from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joy = LaunchConfiguration('use_joy')
    
    use_joystick = os.environ.get('USE_JOYSTICK', 'false')
    if use_joystick == 'true' :
        joystick_declare = DeclareLaunchArgument('use_joy',default_value='true',description='Use joystick if true')
    else:
        joystick_declare = DeclareLaunchArgument('use_joy',default_value='false',description='Use joystick if true')

    joy_params = os.path.join(get_package_share_directory('rom_robotics_joy'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params,{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_joy),
    )
    teleop_node = Node(
        package="teleop_twist_joy",
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params,{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_joy_to_twist')],
        condition=IfCondition(use_joy),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false',description='Use sim time if true'),
        joystick_declare,
        joy_node,
        teleop_node,
    ])