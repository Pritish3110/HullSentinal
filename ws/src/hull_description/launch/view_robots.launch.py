#!/usr/bin/env python3
"""Launch file to view robots and ship hull in RViz with robot_state_publisher"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    desc_dir = get_package_share_directory('hull_description')

    mapper_xacro = os.path.join(desc_dir, 'urdf', 'mapper_robot.urdf.xacro')
    mapper_xml = xacro.process_file(mapper_xacro, mappings={'robot_name': 'mapper_1'}).toxml()

    cleaner_xacro = os.path.join(desc_dir, 'urdf', 'cleaner_robot.urdf.xacro')
    cleaner_xml = xacro.process_file(cleaner_xacro, mappings={'robot_name': 'cleaner_1'}).toxml()

    hull_xacro = os.path.join(desc_dir, 'urdf', 'ship_hull.urdf.xacro')
    hull_xml = xacro.process_file(hull_xacro).toxml()

    mapper_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='mapper_state_publisher',
        namespace='mapper_1',
        output='screen',
        parameters=[{
            'robot_description': mapper_xml,
            'use_sim_time': False
        }],
        remappings=[('/robot_description', '/mapper_1/robot_description')]
    )

    cleaner_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cleaner_state_publisher',
        namespace='cleaner_1',
        output='screen',
        parameters=[{
            'robot_description': cleaner_xml,
            'use_sim_time': False
        }],
        remappings=[('/robot_description', '/cleaner_1/robot_description')]
    )

    hull_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='hull_state_publisher',
        namespace='hull',
        output='screen',
        parameters=[{
            'robot_description': hull_xml,
            'use_sim_time': False
        }],
        remappings=[('/robot_description', '/hull/robot_description')]
    )

    return LaunchDescription([
        hull_state_pub,
        mapper_state_pub,
        cleaner_state_pub
    ])
