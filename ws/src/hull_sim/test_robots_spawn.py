#!/usr/bin/env python3
"""
Test Robot Spawning and Topic Response
Gate verification for Phase 3:
1. Spawns mapper and cleaner bots in simulation
2. Checks that pose and status topics publish
3. Commands a velocity Twist and confirms robot displacement
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from hull_msgs.msg import RobotStatus, MissionStatus


class RobotSpawnTester(Node):
    def __init__(self):
        super().__init__('robot_spawn_tester')
        self.mapper_poses = []
        self.cleaner_poses = []
        self.statuses = {}

        self.mapper_sub = self.create_subscription(
            PoseStamped, '/mapper_1/pose', self._mapper_pose_cb, 10)
        self.cleaner_sub = self.create_subscription(
            PoseStamped, '/cleaner_1/pose', self._cleaner_pose_cb, 10)
        self.status_sub = self.create_subscription(
            RobotStatus, '/cleaner_1/status', self._cleaner_status_cb, 10)
        self.mission_sub = self.create_subscription(
            MissionStatus, '/hull_sim/mission_status', self._mission_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cleaner_1/cmd_vel', 10)
        self.mission_status_received = False

    def _mapper_pose_cb(self, msg):
        self.mapper_poses.append((msg.pose.position.x, msg.pose.position.y))

    def _cleaner_pose_cb(self, msg):
        self.cleaner_poses.append((msg.pose.position.x, msg.pose.position.y))

    def _cleaner_status_cb(self, msg):
        self.statuses['cleaner_1'] = msg

    def _mission_cb(self, msg):
        self.mission_status_received = True


def test_spawn_and_response():
    rclpy.init()
    tester = RobotSpawnTester()

    print("[Phase 3 Test] Waiting for sim engine publishers...")
    start = time.time()
    while time.time() - start < 6.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        if tester.mapper_poses and tester.cleaner_poses and tester.mission_status_received:
            break

    assert len(tester.mapper_poses) > 0, "Gate Failure: /mapper_1/pose topic not publishing"
    assert len(tester.cleaner_poses) > 0, "Gate Failure: /cleaner_1/pose topic not publishing"
    assert tester.mission_status_received, "Gate Failure: /hull_sim/mission_status not publishing"
    print(f"[Phase 3 Test] Both robots detected! Initial cleaner pos: {tester.cleaner_poses[-1]}")

    # Send command to cleaner_1
    init_pos = tester.cleaner_poses[-1]
    cmd = Twist()
    cmd.linear.x = 0.50
    cmd.angular.z = 0.0

    print("[Phase 3 Test] Sending Twist command (vx=0.5) to cleaner_1 for 2.0s...")
    t_end = time.time() + 2.0
    while time.time() < t_end:
        tester.cmd_pub.publish(cmd)
        rclpy.spin_once(tester, timeout_sec=0.05)

    final_pos = tester.cleaner_poses[-1]
    displacement = math.hypot(final_pos[0] - init_pos[0], final_pos[1] - init_pos[1])
    print(f"[Phase 3 Test] Final cleaner pos: {final_pos}, Displacement: {displacement:.3f}m")

    assert displacement > 0.15, f"Gate Failure: Cleaner robot did not displace (disp={displacement:.3f}m)"
    print("[Phase 3 Test] PASSED: All bots spawned, topics published, and bot responded to command!")

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    test_spawn_and_response()
