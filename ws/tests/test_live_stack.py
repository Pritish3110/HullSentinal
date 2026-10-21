#!/usr/bin/env python3
"""
HullSentinal V2 — Live Stack Integration Smoke Test

The gate suite exercises each node in isolation, so a node that constructs correctly but
dies on its first timer tick still shows up green. This test brings the real launch file
up and asserts that every node survives startup and that every visualization topic is
actually being published, which is the failure mode that leaves RViz empty.
"""

import os
import shutil
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node

STARTUP_GRACE_SEC = 18.0
DISCOVERY_SEC = 6.0

# Published by marker_publisher; an empty RViz means one of these went missing.
REQUIRED_TOPICS = [
    '/defects',
    '/robot_markers',
    '/belief_map/heatmap',
    '/task_queue_marker',
    '/mapper_1/sensor_fov',
    '/mapper_1/path_marker',
    '/cleaner_1/path_marker',
    '/visualization_marker_array',
    # Core pipeline
    '/defect_detections',
    '/belief_updates',
    '/task_queue',
    '/mapper/lane_state',
    '/lane_coordination/events',
    '/geofence_status',
    '/mapper_1/pose',
    '/cleaner_1/pose',
    '/mapper_1/cmd_vel',
    '/cleaner_1/cmd_vel',
]

# ros2 launch reports a crashed node as a clean exit when the node swallows the
# exception, so both spellings have to be treated as failures.
DEATH_MARKERS = ('process has died', 'finished cleanly', 'Fatal error')


@pytest.fixture(scope='module')
def live_stack(tmp_path_factory):
    if shutil.which('ros2') is None:
        pytest.skip('ros2 CLI not on PATH; source the ROS 2 and workspace setup files')
    if 'hull_bringup' not in os.environ.get('AMENT_PREFIX_PATH', ''):
        pytest.skip('hull_bringup not in AMENT_PREFIX_PATH; source ws/install/setup.bash')

    log_path = tmp_path_factory.mktemp('live_stack') / 'launch.log'
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'hull_bringup', 'hull_full.launch.py', 'gui:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            time.sleep(STARTUP_GRACE_SEC)
            assert proc.poll() is None, 'launch exited during startup'
            yield proc, log_path
        finally:
            # The launch runs in its own session, so signal the whole group.
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=15)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            time.sleep(2.0)


def test_no_node_dies_during_startup(live_stack):
    """Every launched node must still be alive after the startup grace period."""
    _, log_path = live_stack
    log = log_path.read_text()

    dead = [line for line in log.splitlines()
            if any(marker in line for marker in DEATH_MARKERS)]
    assert not dead, 'node(s) exited during startup:\n' + '\n'.join(dead[:10])


def test_all_visualization_topics_are_published(live_stack):
    """Each required topic must exist and have at least one live publisher."""
    _, _ = live_stack

    shutdown_after = not rclpy.ok()
    if shutdown_after:
        rclpy.init()
    node = Node('live_stack_probe')
    try:
        deadline = time.time() + DISCOVERY_SEC
        published = {}
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            published = {name for name, _ in node.get_topic_names_and_types()
                         if node.count_publishers(name) > 0}
            if all(topic in published for topic in REQUIRED_TOPICS):
                break

        missing = [topic for topic in REQUIRED_TOPICS if topic not in published]
        assert not missing, f'topics absent or unpublished: {missing}'
    finally:
        node.destroy_node()
        if shutdown_after and rclpy.ok():
            rclpy.shutdown()
