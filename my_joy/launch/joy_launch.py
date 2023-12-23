import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from glob import glob

def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    print(config_filepath)

    # return launch.LaunchDescription([
    #     launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
    #     launch.actions.DeclareLaunchArgument('joy_config', default_value='ps3'),
    #     launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

    #     launch_ros.actions.Node(
    #         package='joy', executable='joy_node', name='joy_node',
    #         parameters=[{
    #             'dev': joy_dev,
    #             'deadzone': 0.3,
    #             'autorepeat_rate': 20.0,
    #         }]),
    #     launch_ros.actions.Node(
    #         package='my_joy', 
    #         executable='my_joy',
    #         name='my_joy_node',
    #         ),
    # ])