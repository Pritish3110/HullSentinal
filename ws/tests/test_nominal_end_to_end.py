#!/usr/bin/env python3
"""
Nominal Scenario End-to-End Integration Test
Verifies the complete closed-loop cycle:
Detection -> Belief Registration -> Utility Scoring -> Task Dispatch ->
Cleaning Execution -> Completion Handshake -> Verification Re-scan -> Marked DONE
"""

import time
import rclpy
from rclpy.node import Node
from hull_msgs.msg import DefectDetection, TaskQueue, BeliefUpdate, MissionStatus


class NominalEndToEndVerifier(Node):
    def __init__(self):
        super().__init__('nominal_e2e_verifier')

        self.detections = []
        self.queue_updates = []
        self.belief_updates = []
        self.verified_done = []
        self.requeued = []

        self.detect_sub = self.create_subscription(
            DefectDetection, '/defect_detections', self._detect_cb, 20)
        self.queue_sub = self.create_subscription(
            TaskQueue, '/task_queue', self._queue_cb, 10)
        self.update_sub = self.create_subscription(
            BeliefUpdate, '/belief_updates', self._update_cb, 20)

    def _detect_cb(self, msg):
        self.detections.append(msg)

    def _queue_cb(self, msg):
        if msg.tasks:
            self.queue_updates.append(msg.tasks)

    def _update_cb(self, msg):
        self.belief_updates.append(msg)
        if msg.status == 'DONE':
            self.verified_done.append(msg.defect_id)
            self.get_logger().info(f"[E2E Test] Verified DONE defect: {msg.defect_id}")
        elif msg.status == 'RE_QUEUED':
            self.requeued.append(msg.defect_id)
            self.get_logger().info(f"[E2E Test] Re-queued defect: {msg.defect_id}")


def run_e2e_nominal_test():
    rclpy.init()
    verifier = NominalEndToEndVerifier()

    print("=======================================================")
    print(" HullSentinal Phase 4 Gate: Full Nominal End-to-End Test")
    print(" Monitoring closed-loop task execution...")
    print("=======================================================")

    start_time = time.time()
    timeout = 25.0  # seconds

    while time.time() - start_time < timeout:
        rclpy.spin_once(verifier, timeout_sec=0.1)
        if len(verifier.verified_done) >= 1:
            print(f"[E2E Test] SUCCESS: Completed closed-loop cycle in {time.time() - start_time:.2f}s!")
            print(f"  - Total detections: {len(verifier.detections)}")
            print(f"  - Task queue updates: {len(verifier.queue_updates)}")
            print(f"  - Verified DONE defects: {verifier.verified_done}")
            break

    assert len(verifier.detections) > 0, "E2E Failure: No defect detections received from mapper"
    assert len(verifier.queue_updates) > 0, "E2E Failure: No prioritized tasks in task queue"
    assert len(verifier.verified_done) >= 1, "E2E Failure: No defect reached verified DONE state within timeout"

    print("\n[Phase 4 Gate PASSED] Complete nominal scenario ran end-to-end with verified clean completion!")
    verifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_e2e_nominal_test()
