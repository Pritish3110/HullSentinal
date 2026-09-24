#!/usr/bin/env python3
"""
HullSentinal V2 Full System Launch File
Brings up complete cognitive multi-robot ship hull inspection and cleaning pipeline:
1. URDF robot_state_publishers for hull, mapper_1, and cleaner_1
2. Simulation Engine (physics, 14 defects, deterministic crack escalation)
3. Belief Map Node (SQLAlchemy world state, lifecycle tracking)
4. Lane Coordinator Node (mapper-cleaner lane synchronization & emergency override)
5. Utility Scorer Node (8-connected A* path cost + risk weighting)
6. Verification Monitor Node (closed-loop inspection rescan handshake)
7. Mapper Controller Node (deterministic 3-lane serpentine sweep + obstacle detours)
8. Cleaner Crawler Controller Node (A* waypoint following & perimeter safety)
9. Cleaner Tool Controller Node (state machine, priority context stack, cavitation tool)
10. Safety Barrier Node (predictive conflict detection, mapper priority, deadlock sidestep)
11. Geofence Node (24m x 8m manifold perimeter watchdog)
12. Single-Owner Marker Publisher (stable IDs, delete-all, 24x8m heatmap, ASCII HUD)
13. Demo Snapshot Node (captures high-res visual evidence to docs/)
14. RViz2 GUI (optional when gui:=true)
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
    master_config_file = os.path.join(bringup_dir, 'config', 'hullsentinal_v2.yaml')

    gui_arg = DeclareLaunchArgument('gui', default_value='false', description='Launch RViz2 GUI')
    num_cleaners_arg = DeclareLaunchArgument('num_cleaners', default_value='1', description='Number of cleaning robots')
    scenario_arg = DeclareLaunchArgument('scenario', default_value='nominal', description='Scenario name')
    config_arg = DeclareLaunchArgument('config_file', default_value=master_config_file, description='Path to master configuration YAML')

    gui = LaunchConfiguration('gui')
    num_cleaners = LaunchConfiguration('num_cleaners')
    scenario = LaunchConfiguration('scenario')
    config_file = LaunchConfiguration('config_file')

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
            'config_file': config_file,
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
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    # 4. Cognition: Lane Coordinator Node
    coord_node = Node(
        package='hull_swarm',
        executable='lane_coordinator_node',
        name='lane_coordinator_node',
        output='screen'
    )

    # 5. Cognition: Utility Scorer Node
    utility_node = Node(
        package='hull_swarm',
        executable='utility_scorer',
        name='utility_scorer',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'cleaner_robot_id': 'cleaner_1'
        }]
    )

    # 6. Cognition: Verification Monitor Node
    verification_node = Node(
        package='hull_swarm',
        executable='verification_monitor',
        name='verification_monitor',
        output='screen'
    )

    # 7. Control: Mapper Controller (Serpentine sweep with obstacle detours)
    mapper_ctrl = Node(
        package='hull_control',
        executable='mapper_controller',
        name='mapper_controller',
        output='screen',
        parameters=[{
            'robot_id': 'mapper_1',
            'config_file': config_file,
            'goal_tolerance': 0.35
        }]
    )

    # 8. Control: Cleaner 1 Crawler Controller (A* Waypoint Pursuit)
    cleaner_ctrl = Node(
        package='hull_control',
        executable='crawler_controller',
        name='cleaner_1_crawler_controller',
        output='screen',
        parameters=[{
            'robot_id': 'cleaner_1',
            'role': 'cleaner',
            'config_file': config_file,
            'goal_tolerance': 0.30
        }]
    )

    # 9. Control: Cleaner 1 Tool State Machine & Emergency Context Stack
    cleaner_tool = Node(
        package='hull_control',
        executable='tool_controller',
        name='cleaner_1_tool_controller',
        output='screen',
        parameters=[{
            'robot_id': 'cleaner_1',
            'config_file': config_file,
            'arrival_distance': 0.40
        }]
    )

    # 10. Control: Inter-Robot Dynamic Safety Barrier
    safety_node = Node(
        package='hull_control',
        executable='safety_barrier',
        name='safety_barrier',
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    # 11. Control: Geofence Monitor Node (24m x 8m manifold watchdog)
    geofence_node = Node(
        package='hull_control',
        executable='geofence_node',
        name='geofence_node',
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    # 12. Visualization: Single-Owner Marker Publisher
    marker_node = Node(
        package='hull_bringup',
        executable='marker_publisher',
        name='marker_publisher',
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    # 13. Visual Demo Snapshot Node
    snapshot_node = Node(
        package='hull_bringup',
        executable='demo_snapshot',
        name='demo_snapshot',
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    # 14. RViz2 (GUI condition)
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
        config_arg,
        desc_launch,
        sim_node,
        belief_node,
        coord_node,
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
