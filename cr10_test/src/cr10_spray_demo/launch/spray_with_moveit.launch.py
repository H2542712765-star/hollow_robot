#!/usr/bin/env python3
"""Optional launcher that starts a selected cr10_moveit launch file first.

For the user's current two-part system it is usually clearer to launch the
Gazebo/MoveIt system normally, then run ``spray_demo.launch.py`` separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    moveit_launch_file = LaunchConfiguration('moveit_launch_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    startup_delay = LaunchConfiguration('startup_delay')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'moveit_launch_file',
            default_value='moveit_gazebo.launch.py',
            description='Launch file inside cr10_moveit/launch',
        ),
        DeclareLaunchArgument(
            'startup_delay',
            default_value='10.0',
            description='Delay before starting the spray node',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('cr10_moveit'),
                    'launch',
                    moveit_launch_file,
                ])
            ),
        ),
        TimerAction(
            period=startup_delay,
            actions=[
                Node(
                    package='cr10_spray_demo',
                    executable='spray_demo_node',
                    name='cr10_spray_demo',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([
                            FindPackageShare('cr10_spray_demo'),
                            'config',
                            'spray_demo.yaml',
                        ]),
                        {
                            'use_sim_time': use_sim_time,
                        },
                    ],
                )
            ],
        ),
    ])
