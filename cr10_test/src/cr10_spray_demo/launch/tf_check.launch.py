#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('planning_frame', default_value='dummy_link'),
        DeclareLaunchArgument('tool_link', default_value='Link6'),
        DeclareLaunchArgument('timeout', default_value='15.0'),
        Node(
            package='cr10_spray_demo',
            executable='tf_check_node',
            name='cr10_tf_check',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'planning_frame': LaunchConfiguration('planning_frame'),
                'tool_link': LaunchConfiguration('tool_link'),
                'timeout': LaunchConfiguration('timeout'),
            }],
        ),
    ])
