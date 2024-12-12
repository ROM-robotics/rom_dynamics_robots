#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_composition = LaunchConfiguration('use_composition')
    use_slamtoolbox = LaunchConfiguration('use_slamtoolbox')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')
    
    declare_use_slamtoolbox_cmd = DeclareLaunchArgument(
        'use_slamtoolbox', default_value='False',
        description='Use use_slamtoolbox if True')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use use_sim_time if True')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        arguments=[
            "yaml_filename",os.path.join(get_package_share_directory('bobo_nav2'), 'maps', 'maze1.yaml')
            ],
    )

    # match_map = Node(
    #     package='',
    #     executable='',
    #     arguments=['model',
    #         ],
    # )

    carto_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bobo_carto"), "launch", "cartographer_localization.launch.py"
            )), condition=UnlessCondition(LaunchConfiguration('use_slamtoolbox')),
            launch_arguments={'use_sim_time': use_sim_time}.items() 
    )

    slamtoolbox_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("bobo_nav2"), "launch", "slamtb_online_async_localization.launch.py"
            )), condition=IfCondition(LaunchConfiguration('use_slamtoolbox')),
            launch_arguments={'use_sim_time': use_sim_time}.items() 
    )

    # composable true လုပ်ရန် 
    sim_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("bobo_nav2"),"launch","sim_navigation.launch.py"
            )]), launch_arguments={'use_composition': use_composition}.items()
    )

    # set_dynamic_client = Node(
    #     package='',
    #     executable='',
    #     arguments=['model',
    #         ],
    # )
    
    # mode_assist = Node(
    #     package='',
    #     executable='',
    #     arguments=['mode','navi_mode'],
    # )

    return LaunchDescription(
        [
            declare_use_composition_cmd,
            declare_use_slamtoolbox_cmd,
            declare_use_sim_time_cmd,
            map_server,
            carto_localization,
            slamtoolbox_localization,
            sim_navigation
        ]
    )
    

# လိုအပ်ရင် delay node သုံးရန်
# carto နှင့် slamtoolbox switch ရန် use_slamtoolbox က default အားဖြင့် false 
# navigation ကို composable true လုပ်ရန် 

# ORIGINAL

# / Nodes /
# map_server
# yoyo_pose_monitor
# 3DCam.py
# set_dynamic_client
# mode_assist.py

# / launch /
# map_match.launch
# cartographer_localization.launch
# move_base.launch
# provider.launch
# path_server.launch
# socker_wifi.launch