#!/usr/bin/env python3
"""
Naive Baseline Node
Implements conventional naive inspection & cleaning strategy:
- FIFO task queuing (no utility scoring, no structural risk weights)
- Immediate task completion without verification (open-loop)
Used as comparison baseline against HullSentinal cognitive architecture.
"""

from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from hull_msgs.msg import DefectDetection, TaskQueue, TaskItem
from hull_msgs.srv import TaskComplete


class NaiveBaselineNode(Node):
    def __init__(self):
        super().__init__('naive_baseline_node')

        self.fifo_queue = deque()
        self.dispatched_tasks = set()
        self.completed_tasks = set()

        # Subscriptions
        self.detect_sub = self.create_subscription(
            DefectDetection, '/defect_detections', self._detect_cb, 20)

        # Master queue publisher
        self.queue_pub = self.create_publisher(TaskQueue, '/task_queue', 10)

        # Provide open-loop /task_complete service
        self.complete_srv = self.create_service(
            TaskComplete, '/task_complete', self._complete_cb)

        # Loop timer (1 Hz)
        self.timer = self.create_timer(1.0, self._publish_queue)
        self.get_logger().info('NaiveBaselineNode active (FIFO, open-loop, no verification)')

    def _detect_cb(self, msg):
        if msg.defect_id not in self.dispatched_tasks and msg.defect_id not in self.completed_tasks:
            item = TaskItem()
            item.defect_id = msg.defect_id
            item.defect_type = msg.defect_type
            item.utility_score = 0.5  # Static dummy score
            item.position = msg.position
            item.status = 'QUEUED'
            self.fifo_queue.append(item)
            self.dispatched_tasks.add(msg.defect_id)

    def _complete_cb(self, request, response):
        d_id = request.defect_id
        self.completed_tasks.add(d_id)
        self.get_logger().info(f"[Naive Baseline] Defect {d_id} blindly marked DONE (open-loop)")
        response.verification_triggered = False
        response.message = f'Blindly marked {d_id} as done'
        return response

    def _publish_queue(self):
        msg = TaskQueue()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Only active remaining tasks
        active = [t for t in self.fifo_queue if t.defect_id not in self.completed_tasks]
        msg.tasks = active
        self.queue_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NaiveBaselineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
