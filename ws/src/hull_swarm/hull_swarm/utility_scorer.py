#!/usr/bin/env python3
"""
Utility Scorer Node
Computes dynamic multi-attribute priority scores for all active defects:
U(d) = (severity * risk_weight * confidence) - alpha * distance_cost + requeue_bonus
"""

import math
import heapq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from hull_msgs.msg import TaskQueue, TaskItem


class UtilityScorer(Node):
    def __init__(self):
        super().__init__('utility_scorer')

        self.declare_parameter('weight_crack', 1.0)
        self.declare_parameter('weight_rust', 0.70)
        self.declare_parameter('weight_biofouling', 0.40)
        self.declare_parameter('distance_alpha', 0.05)
        self.declare_parameter('requeue_escalation', 0.15)
        self.declare_parameter('cleaner_robot_id', 'cleaner_1')

        self.w_crack = self.get_parameter('weight_crack').get_parameter_value().double_value
        self.w_rust = self.get_parameter('weight_rust').get_parameter_value().double_value
        self.w_bio = self.get_parameter('weight_biofouling').get_parameter_value().double_value
        self.alpha = self.get_parameter('distance_alpha').get_parameter_value().double_value
        self.requeue_esc = self.get_parameter('requeue_escalation').get_parameter_value().double_value
        self.cleaner_id = self.get_parameter('cleaner_robot_id').get_parameter_value().string_value

        self.cleaner_pose = (0.5, 5.0)  # Default fallback position
        self.active_tasks = []

        # Subscriptions
        self.active_sub = self.create_subscription(
            TaskQueue, '/belief_map/active_defects', self._active_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.cleaner_id}/pose', self._pose_cb, 10)

        # Publisher for prioritized task queue
        self.queue_pub = self.create_publisher(TaskQueue, '/task_queue', 10)

        # Re-score loop timer (2 Hz)
        self.timer = self.create_timer(0.5, self._score_and_publish)
        self.get_logger().info('UtilityScorer initialized with risk weights (crack=1.0, rust=0.7, bio=0.4)')

    def _pose_cb(self, msg):
        self.cleaner_pose = (msg.pose.position.x, msg.pose.position.y)

    def _active_cb(self, msg):
        self.active_tasks = msg.tasks

    def _score_and_publish(self):
        if not self.active_tasks:
            return

        cx, cy = self.cleaner_pose
        scored_heap = []

        for task in self.active_tasks:
            # Determine structural risk weight
            dtype = task.defect_type.lower()
            if 'crack' in dtype:
                w_risk = self.w_crack
            elif 'rust' in dtype or 'corrosion' in dtype:
                w_risk = self.w_rust
            else:
                w_risk = self.w_bio

            # Spatial distance from cleaner to defect
            dist = math.hypot(task.position.x - cx, task.position.y - cy)

            # Severity estimate approximation based on type if not explicitly supplied
            base_sev = 0.85 if 'crack' in dtype else (0.70 if 'rust' in dtype else 0.55)

            # Re-queue escalation bonus
            esc_bonus = self.requeue_esc if task.status == 'RE_QUEUED' else 0.0

            # Utility formula
            utility = (base_sev * w_risk * 0.90) - (self.alpha * dist) + esc_bonus
            utility = max(0.01, utility)

            task_copy = TaskItem()
            task_copy.defect_id = task.defect_id
            task_copy.defect_type = task.defect_type
            task_copy.utility_score = float(utility)
            task_copy.position = task.position
            task_copy.status = 'QUEUED' if task.status in ('DETECTED', 'PENDING') else task.status

            # Python heapq is min-heap; push (-utility) to sort descending
            heapq.heappush(scored_heap, (-utility, task_copy))

        # Build ordered queue
        ordered_tasks = []
        while scored_heap:
            _, t_item = heapq.heappop(scored_heap)
            ordered_tasks.append(t_item)

        out_msg = TaskQueue()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.tasks = ordered_tasks
        self.queue_pub.publish(out_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = UtilityScorer()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        except Exception:
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
