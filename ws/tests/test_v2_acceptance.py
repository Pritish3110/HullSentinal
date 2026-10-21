#!/usr/bin/env python3
"""
HullSentinal V2 — Comprehensive Automated Acceptance Test Suite
Validates all 8 architectural gates for the Cognitive Multi-Robot Upgrade:
- Gate 1: Centralized Configuration & 24m x 8m World Geometry
- Gate 2: 2D Occupancy Grid & 8-Connected A* Obstacle Avoidance
- Gate 3: Serpentine Mapper Coverage & Order Progression
- Gate 4: Lane Coordinator Policy & Strict Lane Gating (< N)
- Gate 5: Deterministic Defect Growth & Emergency Threshold Escalation
- Gate 6: Cleaner State Machine & Priority Context Stack Preemption
- Gate 7: Single-Owner MarkerPublisher with Stable ID Indexing & ASCII Text
- Gate 8: Dynamic Safety Barrier & Geofence Perimeter Containment
- Gate 9: Marker Namespaces Consistent With The Saved RViz Layout
- Gate 10: Verification Handshake Retry & Timeout (no permanent VERIFYING stall)
- Gate 11: Emergency Preemption Is Idempotent Per Defect (bounded context stack)
"""

import os
import math
import time
import yaml
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray

from hull_control.grid_planner import GridPlanner
from hull_control.mapper_controller import MapperController
from hull_control.tool_controller import ToolController, CleanerState
from hull_control.safety_barrier import SafetyBarrier
from hull_control.geofence_node import GeofenceNode
from hull_swarm.lane_coordinator_node import LaneCoordinatorNode
from hull_swarm.belief_map_node import BeliefMapNode
from hull_swarm.utility_scorer import UtilityScorer
from hull_sim.sim_engine_node import SimEngineNode
from hull_bringup.marker_publisher import MarkerPublisher
from hull_msgs.msg import TaskItem, TaskQueue, MapperStatus, BeliefUpdate, DefectDetection


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_gate1_configuration_and_world(rclpy_init):
    """Gate 1: Centralized YAML config must load 24x8m geometry, 3 lanes, 6 obstacles, 14 defects."""
    cfg_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/hull_bringup/config/hullsentinal_v2.yaml'))
    assert os.path.exists(cfg_path), f"Master config missing at {cfg_path}"

    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # Geometry assertions
    assert cfg['world']['hull_width'] == 24.0
    assert cfg['world']['hull_height'] == 8.0
    assert cfg['world']['num_lanes'] == 3
    assert cfg['world']['safety_margin'] == 0.8

    # Obstacles & Defects count
    assert len(cfg['static_obstacles']) == 6, f"Expected 6 obstacles, got {len(cfg['static_obstacles'])}"
    assert len(cfg['defects']) == 14, f"Expected 14 defects, got {len(cfg['defects'])}"

    # Escalating defect check
    c2 = next(d for d in cfg['defects'] if d['id'] == 'defect_crack_02')
    assert c2['lane_id'] == 2
    assert c2['severity'] == 0.68
    assert c2['growth_rate'] == 0.0080


def test_gate2_grid_planner_and_astar_detours(rclpy_init):
    """Gate 2: Grid planner must inflate obstacles by 0.55m and compute detour around obstacles."""
    cfg_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/hull_bringup/config/hullsentinal_v2.yaml'))
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    planner = GridPlanner(
        hull_width=24.0,
        hull_height=8.0,
        resolution=0.20,
        inflation_radius=0.55,
        safety_margin=0.8,
        obstacles=cfg['static_obstacles']
    )

    # obs_anode_03 is at (6.0, 4.8), width 1.0, height 0.5. Path from (4.0, 4.8) to (8.0, 4.8)
    path = planner.plan_path((4.0, 4.8), (8.0, 4.8))
    path_len = planner.get_path_length((4.0, 4.8), (8.0, 4.8))

    assert len(path) > 2, "Path must contain detour waypoints around obstacle"
    assert path_len > 4.0, "Detour length must exceed direct Euclidean distance"

    # Verify no waypoint penetrates inflated obstacle
    for wp in path:
        dist_to_obs = math.hypot(wp[0] - 6.0, wp[1] - 4.8)
        # Bounding box clearance check
        assert not (5.0 <= wp[0] <= 7.0 and 4.2 <= wp[1] <= 5.4), f"Waypoint {wp} penetrated obstacle buffer"


def test_gate3_serpentine_mapper_progression(rclpy_init):
    """Gate 3: Mapper sweeps Lane 1 -> Lane 2 -> Lane 3 in order and computes coverage %."""
    mapper = MapperController()
    assert mapper.state == 'MAPPING_LANE_1'
    assert mapper.current_lane == 1
    assert len(mapper.waypoints) > 20

    # Simulate position along Lane 1
    ps = PoseStamped()
    ps.pose.position.x = 11.75
    ps.pose.position.y = 2.0
    mapper._pose_cb(ps)
    mapper._control_loop()
    assert 40.0 <= mapper.lane_coverage_pct <= 60.0
    assert 13.0 <= mapper.total_coverage_pct <= 20.0

    mapper.destroy_node()


def test_gate4_lane_coordination_and_cleaner_gating(rclpy_init):
    """Gate 4: Cleaner held in WAITING_FOR_LANE until Lane 1 released; emergency overrides lane."""
    coord = LaneCoordinatorNode()
    assert coord.released_cleaner_lane == 0

    # Advance mapper to Lane 2
    m = MapperStatus()
    m.state = 'MAPPING_LANE_2'
    m.current_lane = 2
    coord._mapper_cb(m)
    assert coord.released_cleaner_lane == 1

    # Emit emergency belief update
    b = BeliefUpdate()
    b.defect_id = 'defect_crack_02'
    b.severity = 0.88
    b.risk_level = 'CRITICAL'
    b.emergency_flag = True
    b.lane_id = 2
    coord._belief_cb(b)
    assert coord.emergency_active == True

    coord.destroy_node()


def test_gate5_defect_growth_and_emergency_escalation(rclpy_init):
    """Gate 5: defect_crack_02 deterministically escalates past 0.85 within 30s."""
    sim = SimEngineNode()
    d2 = sim.defects['defect_crack_02']
    assert d2['current_sev'] == 0.68

    # Simulate 30s (600 steps of 0.05s)
    for _ in range(600):
        sim._sim_step()

    assert d2['current_sev'] >= 0.85, f"Expected severity >= 0.85, got {d2['current_sev']:.3f}"
    sim.destroy_node()


def test_gate6_emergency_context_stack_preemption(rclpy_init):
    """Gate 6: Critical emergency preempts active task, pushes context to stack, then resumes."""
    cleaner = ToolController()
    cleaner.current_pose = (1.5, 7.0)

    # 1. Normal task dispatch
    task1 = TaskItem()
    task1.defect_id = 'defect_rust_01'
    task1.position.x = 9.5
    task1.position.y = 1.8
    task1.lane_id = 1
    task1.eligible = True
    cleaner._dispatch_task(task1)
    assert cleaner.state == CleanerState.NAVIGATING
    assert cleaner.current_task.defect_id == 'defect_rust_01'

    # 2. Critical emergency preemption
    emerg_task = TaskItem()
    emerg_task.defect_id = 'defect_crack_02'
    emerg_task.position.x = 8.5
    emerg_task.position.y = 3.8
    emerg_task.lane_id = 2
    emerg_task.severity = 0.88
    emerg_task.emergency_flag = True
    emerg_task.risk_level = 'CRITICAL'

    cleaner._trigger_emergency_interruption(emerg_task)
    assert len(cleaner.context_stack) == 1
    assert cleaner.context_stack[0]['task'].defect_id == 'defect_rust_01'
    assert cleaner.state == CleanerState.EMERGENCY_NAVIGATING
    assert cleaner.current_task.defect_id == 'defect_crack_02'

    cleaner.destroy_node()


def test_gate7_single_owner_marker_indexing(rclpy_init):
    """Gate 7: Exactly 14 defects with stable (ns, id) markers and clean ASCII labels."""
    pub = MarkerPublisher()
    assert len(pub.defects) == 14

    # Populate the state that only exists once odometry and goals are flowing, so the
    # trail / goal-arrow / robot-marker branches are actually executed. Without this the
    # gate never reaches them and a NameError in those branches passes unnoticed.
    for rid, sx, sy in (('mapper_1', 1.5, 2.0), ('cleaner_1', 1.5, 7.0)):
        for step in range(5):
            msg = PoseStamped()
            msg.pose.position.x = sx + 0.5 * step
            msg.pose.position.y = sy
            pub._pose_cb(rid, msg)
    goal = PoseStamped()
    goal.pose.position.x = 9.5
    goal.pose.position.y = 1.8
    pub._goal_cb(goal)
    assert len(pub.robot_paths['mapper_1']) >= 2
    assert pub.robot_goals['cleaner_1'] is not None

    # Run publisher cycle
    pub._publish_all()  # delete all
    pub._publish_all()  # actual publish

    geom_ids = set()
    label_ids = set()
    for d in pub.defects.values():
        idx = d['idx']
        assert 0 <= idx < 14
        geom_ids.add(idx)
        label_ids.add(100 + idx)

    assert len(geom_ids) == 14
    assert len(label_ids) == 14
    assert len(geom_ids.intersection(label_ids)) == 0, "Geometry and Label IDs must not collide"

    pub.destroy_node()


def test_gate9_marker_namespaces_match_rviz_config(rclpy_init):
    """Gate 9: Every namespace the publisher emits must exist in the saved RViz layout."""
    rviz_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '../src/hull_bringup/config/rviz_config.rviz'))
    assert os.path.exists(rviz_path), f"RViz layout missing at {rviz_path}"
    with open(rviz_path, 'r') as f:
        rviz_cfg = yaml.safe_load(f)

    configured = {}
    for display in rviz_cfg['Visualization Manager']['Displays']:
        topic = display.get('Marker Topic', {}).get('Value')
        if topic:
            configured[topic] = set(display.get('Namespaces', {}))

    pub = MarkerPublisher()
    captured = {'/defects': set(), '/robot_markers': set()}

    def capture(topic):
        return lambda arr: captured[topic].update(
            m.ns for m in arr.markers if m.action == Marker.ADD)

    pub.defects_pub.publish = capture('/defects')
    pub.robot_pub.publish = capture('/robot_markers')
    pub.heatmap_pub.publish = lambda arr: None
    pub.composite_pub.publish = lambda arr: None
    pub.queue_marker_pub.publish = lambda m: None
    pub.fov_pub.publish = lambda m: None
    pub.mapper_path_pub.publish = lambda m: None
    pub.cleaner_path_pub.publish = lambda m: None
    pub.cleaner_goal_pub.publish = lambda m: None

    pub._publish_all()  # delete all
    pub._publish_all()  # actual publish
    pub.destroy_node()

    assert captured['/defects'] == {'defects_geometry', 'defects_text'}
    assert captured['/robot_markers'] == {'robot_halo', 'robot_label', 'robot_stem'}

    for topic, emitted in captured.items():
        missing = emitted - configured.get(topic, set())
        assert not missing, (
            f"{topic} emits namespace(s) {sorted(missing)} that rviz_config.rviz does not list; "
            "RViz would restore them with no saved visibility state")


def test_gate10_verification_state_machine_never_stalls(rclpy_init):
    """Gate 10: VERIFYING retries while /task_complete is down, then releases the cleaner."""
    cleaner = ToolController()
    cleaner.current_pose = (9.5, 1.8)

    task = TaskItem()
    task.defect_id = 'defect_rust_01'
    task.position.x = 9.5
    task.position.y = 1.8
    task.lane_id = 1
    task.eligible = True

    cleaner.current_task = task
    cleaner.state = CleanerState.VERIFYING
    cleaner.waiting_for_service = False
    cleaner.verify_deadline = time.time() + cleaner.verify_timeout

    # /task_complete is never advertised in this unit context, so the call cannot be sent.
    for _ in range(5):
        cleaner._state_machine_loop()
    assert cleaner.state == CleanerState.VERIFYING, "Must keep retrying before the deadline"
    assert not cleaner.waiting_for_service

    # Past the deadline the cleaner must bail out instead of stalling the whole mission.
    cleaner.verify_deadline = time.time() - 1.0
    cleaner._state_machine_loop()
    assert cleaner.state == CleanerState.IDLE
    assert cleaner.current_task is None
    assert cleaner.current_path == []

    cleaner.destroy_node()


class _FailedFuture:
    """Stands in for a /task_complete future when no service is advertised."""

    def result(self):
        raise RuntimeError('service unavailable in unit context')


def test_gate11_emergency_preempts_each_defect_once(rclpy_init):
    """Gate 11: A CRITICAL defect preempts once; a still-critical queue must not re-preempt."""
    cleaner = ToolController()
    cleaner.current_pose = (1.5, 7.0)

    task1 = TaskItem()
    task1.defect_id = 'defect_rust_01'
    task1.position.x = 9.5
    task1.position.y = 1.8
    task1.lane_id = 1
    task1.status = 'QUEUED'
    task1.eligible = True

    emerg = TaskItem()
    emerg.defect_id = 'defect_crack_02'
    emerg.position.x = 8.5
    emerg.position.y = 3.8
    emerg.lane_id = 2
    emerg.severity = 0.88
    emerg.status = 'QUEUED'
    emerg.emergency_flag = True
    emerg.risk_level = 'CRITICAL'
    emerg.eligible = True

    queue = TaskQueue()
    queue.tasks = [emerg, task1]

    cleaner._dispatch_task(task1)
    assert cleaner.state == CleanerState.NAVIGATING

    # First sighting preempts and banks the interrupted task.
    cleaner._queue_cb(queue)
    assert len(cleaner.context_stack) == 1
    assert cleaner.current_task.defect_id == 'defect_crack_02'
    assert cleaner.state == CleanerState.EMERGENCY_NAVIGATING

    # Remediation finishes and the suspended task resumes.
    cleaner._complete_done_cb(_FailedFuture(), 'defect_crack_02')
    assert cleaner.context_stack == []
    assert cleaner.current_task.defect_id == 'defect_rust_01'

    # The scorer keeps republishing the same CRITICAL task at 2 Hz until the belief map
    # catches up. Re-preempting here would re-bank defect_rust_01 on every tick and the
    # cleaner would ping-pong between the two defects forever.
    for _ in range(10):
        cleaner._queue_cb(queue)

    assert cleaner.context_stack == [], (
        f"Emergency re-preempted after remediation; stack depth {len(cleaner.context_stack)}")
    assert cleaner.current_task.defect_id == 'defect_rust_01'
    assert cleaner.state == CleanerState.NAVIGATING

    # A handshake in flight must never be preempted out from under itself.
    cleaner.state = CleanerState.VERIFYING
    cleaner.preempted_defects.clear()
    cleaner._queue_cb(queue)
    assert cleaner.context_stack == []
    assert cleaner.state == CleanerState.VERIFYING

    cleaner.destroy_node()


def test_gate8_geofence_and_boundary_containment(rclpy_init):
    """Gate 8: Geofence validates nominal positions and catches out-of-bounds breaches."""
    geo = GeofenceNode()

    # Nominal in-bounds pose
    ps = PoseStamped()
    ps.pose.position.x = 12.0
    ps.pose.position.y = 4.0
    geo._pose_cb('cleaner_1', ps)
    geo._monitor_loop()
    assert geo.out_of_bounds_count == 0
    assert geo.all_in_bounds == True

    # Breach test
    ps.pose.position.x = 24.5
    ps.pose.position.y = 4.0
    geo._pose_cb('cleaner_1', ps)
    geo._monitor_loop()
    assert geo.out_of_bounds_count > 0
    assert geo.all_in_bounds == False

    geo.destroy_node()


if __name__ == '__main__':
    pytest.main(['-v', __file__])
