#!/usr/bin/env python3
"""
Scenario Launch File
Launches HullSentinal configured for any of the 6 benchmark scenarios.
Usage: ros2 launch hull_bringup scenario.launch.py scenario:=nominal gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('hull_bringup')
    desc_dir = get_package_share_directory('hull_description')

    scenario_name = LaunchConfiguration('scenario').perform(context)
    gui = LaunchConfiguration('gui')

    # Map scenario name to number of cleaners
    num_cleaners = 3 if scenario_name == 'swarm_scaling' else 1

    # Load scenario specific yaml if present
    cfg_file = os.path.join(bringup_dir, 'config', f'scenario_{scenario_name}.yaml')
    if not os.path.exists(cfg_file):
        # Fallback to nominal
        cfg_file = os.path.join(bringup_dir, 'config', 'scenario_1_nominal.yaml')

    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_dir, 'launch', 'view_robots.launch.py'))
    )

    sim_node = Node(
        package='hull_sim',
        executable='sim_engine_node',
        name='sim_engine_node',
        output='screen',
        parameters=[cfg_file, {'num_cleaners': num_cleaners, 'scenario_name': scenario_name}]
    )

    belief_node = Node(
        package='hull_swarm',
        executable='belief_map_node',
        name='belief_map_node',
        output='screen'
    )

    utility_node = Node(
        package='hull_swarm',
        executable='utility_scorer',
        name='utility_scorer',
        output='screen',
        parameters=[cfg_file]
    )

    verification_node = Node(
        package='hull_swarm',
        executable='verification_monitor',
        name='verification_monitor',
        output='screen',
        parameters=[cfg_file]
    )

    mapper_ctrl = Node(
        package='hull_control',
        executable='crawler_controller',
        name='mapper_crawler_controller',
        output='screen',
        parameters=[{'robot_id': 'mapper_1', 'role': 'mapper', 'max_linear_speed': 0.60}]
    )

    cleaner_nodes = []
    robots_list = ['mapper_1']
    for i in range(num_cleaners):
        cid = f'cleaner_{i+1}'
        robots_list.append(cid)
        c_ctrl = Node(
            package='hull_control',
            executable='crawler_controller',
            name=f'{cid}_crawler_controller',
            output='screen',
            parameters=[cfg_file, {'robot_id': cid, 'role': 'cleaner', 'max_linear_speed': 0.50}]
        )
        c_tool = Node(
            package='hull_control',
            executable='tool_controller',
            name=f'{cid}_tool_controller',
            output='screen',
            parameters=[{'robot_id': cid, 'clean_duration': 2.50}]
        )
        cleaner_nodes.extend([c_ctrl, c_tool])

    safety_node = Node(
        package='hull_control',
        executable='safety_barrier',
        name='safety_barrier',
        output='screen',
        parameters=[cfg_file, {'robots': robots_list, 'safety_radius': 0.55}]
    )

    marker_node = Node(
        package='hull_bringup',
        executable='marker_publisher',
        name='marker_publisher',
        output='screen'
    )

    geofence_node = Node(
        package='hull_control',
        executable='geofence_node',
        name='geofence_node',
        output='screen',
        parameters=[{'robots': robots_list, 'x_min': 0.8, 'x_max': 19.2, 'y_min': 0.8, 'y_max': 5.2}]
    )

    return [
        desc_launch,
        sim_node,
        belief_node,
        utility_node,
        verification_node,
        mapper_ctrl,
        *cleaner_nodes,
        safety_node,
        geofence_node,
        marker_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='1_nominal', description='Scenario name'),
        DeclareLaunchArgument('gui', default_value='false', description='GUI mode'),
        OpaqueFunction(function=launch_setup)
    ])
