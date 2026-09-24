#!/usr/bin/env python3
"""
HullSentinal Full System Launch File
Brings up complete closed-loop cognitive robotics pipeline:
Simulation Engine + Description TFs + Motion Controllers +
Belief Map + Utility Scorer + Verification Monitor + Markers + (Optional) RViz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('hull_bringup')
    desc_dir = get_package_share_directory('hull_description')

    rviz_config_file = os.path.join(bringup_dir, 'config', 'rviz_config.rviz')

    gui_arg = DeclareLaunchArgument('gui', default_value='false', description='Launch RViz2 GUI')
    num_cleaners_arg = DeclareLaunchArgument('num_cleaners', default_value='1', description='Number of cleaning robots')
    scenario_arg = DeclareLaunchArgument('scenario', default_value='nominal', description='Scenario name')

    gui = LaunchConfiguration('gui')
    num_cleaners = LaunchConfiguration('num_cleaners')
    scenario = LaunchConfiguration('scenario')

    # 1. Robot Descriptions (URDF / State Publishers)
    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_dir, 'launch', 'view_robots.launch.py'))
    )

    # 2. Simulation Engine (Kinematics, Manifold physics, Ground Truth Defects)
    sim_node = Node(
        package='hull_sim',
        executable='sim_engine_node',
        name='sim_engine_node',
        output='screen',
        parameters=[{
            'num_cleaners': num_cleaners,
            'scenario_name': scenario,
            'dt': 0.05
        }]
    )

    # 3. Cognition: Belief Map Node
    belief_node = Node(
        package='hull_swarm',
        executable='belief_map_node',
        name='belief_map_node',
        output='screen'
    )

    # 4. Cognition: Utility Scorer Node
    utility_node = Node(
        package='hull_swarm',
        executable='utility_scorer',
        name='utility_scorer',
        output='screen'
    )

    # 5. Cognition: Verification Monitor Node
    verification_node = Node(
        package='hull_swarm',
        executable='verification_monitor',
        name='verification_monitor',
        output='screen'
    )

    # 6. Control: Mapper Crawler Controller (Lawnmower sweep)
    mapper_ctrl = Node(
        package='hull_control',
        executable='crawler_controller',
        name='mapper_crawler_controller',
        output='screen',
        parameters=[{
            'robot_id': 'mapper_1',
            'role': 'mapper',
            'max_linear_speed': 1.50
        }]
    )

    # 7. Control: Cleaner 1 Crawler Controller (P2P Navigation)
    cleaner_ctrl = Node(
        package='hull_control',
        executable='crawler_controller',
        name='cleaner_1_crawler_controller',
        output='screen',
        parameters=[{
            'robot_id': 'cleaner_1',
            'role': 'cleaner',
            'max_linear_speed': 1.60
        }]
    )

    # 8. Control: Cleaner 1 Tool State Machine
    cleaner_tool = Node(
        package='hull_control',
        executable='tool_controller',
        name='cleaner_1_tool_controller',
        output='screen',
        parameters=[{
            'robot_id': 'cleaner_1',
            'clean_duration': 0.90
        }]
    )

    # 9. Control: Inter-Robot Safety Barrier
    safety_node = Node(
        package='hull_control',
        executable='safety_barrier',
        name='safety_barrier',
        output='screen',
        parameters=[{
            'robots': ['mapper_1', 'cleaner_1'],
            'safety_radius': 0.55
        }]
    )

    # 10. Control: Geofence Monitor Node
    geofence_node = Node(
        package='hull_control',
        executable='geofence_node',
        name='geofence_node',
        output='screen',
        parameters=[{
            'robots': ['mapper_1', 'cleaner_1'],
            'x_min': 0.8,
            'x_max': 19.2,
            'y_min': 0.8,
            'y_max': 5.2
        }]
    )

    # 11. Visualization Marker Publisher
    marker_node = Node(
        package='hull_bringup',
        executable='marker_publisher',
        name='marker_publisher',
        output='screen'
    )

    # 12. Visual Demo Snapshot Node
    snapshot_node = Node(
        package='hull_bringup',
        executable='demo_snapshot',
        name='demo_snapshot',
        output='screen'
    )

    # 13. RViz2 (GUI condition)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(gui),
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        num_cleaners_arg,
        scenario_arg,
        desc_launch,
        sim_node,
        belief_node,
        utility_node,
        verification_node,
        mapper_ctrl,
        cleaner_ctrl,
        cleaner_tool,
        safety_node,
        geofence_node,
        marker_node,
        snapshot_node,
        rviz_node
    ])
