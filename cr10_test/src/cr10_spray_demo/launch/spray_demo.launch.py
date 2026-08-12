#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    package_share = get_package_share_directory('cr10_spray_demo')
    default_params = os.path.join(package_share, 'config', 'spray_demo.yaml')

    params_file = LaunchConfiguration('params_file').perform(context)
    planning_frame = LaunchConfiguration('planning_frame').perform(context).strip()
    tool_link = LaunchConfiguration('tool_link').perform(context).strip()
    planning_group = LaunchConfiguration('planning_group').perform(context).strip()

    # Empty launch arguments do not override YAML.  This prevents stale launch
    # defaults from silently replacing a corrected spray_demo.yaml.
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }
    if planning_frame:
        overrides['planning_frame'] = planning_frame
    if tool_link:
        overrides['tool_link'] = tool_link
    if planning_group:
        overrides['planning_group'] = planning_group

    return [
        Node(
            package='cr10_spray_demo',
            executable='spray_demo_node',
            name='cr10_spray_demo',
            output='screen',
            parameters=[params_file or default_params, overrides],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('cr10_spray_demo')
    default_params = os.path.join(package_share, 'config', 'spray_demo.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Spray demo parameter YAML file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo /clock',
        ),
        DeclareLaunchArgument(
            'planning_frame',
            default_value='',
            description='Optional TF frame override; empty keeps YAML value',
        ),
        DeclareLaunchArgument(
            'tool_link',
            default_value='',
            description='Optional tool-link override; empty keeps YAML value',
        ),
        DeclareLaunchArgument(
            'planning_group',
            default_value='',
            description='Optional MoveIt group override; empty keeps YAML value',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
