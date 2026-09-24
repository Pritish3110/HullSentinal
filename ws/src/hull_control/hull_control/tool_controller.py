#!/usr/bin/env python3
"""
Tool Controller Node
Executes the cleaning robot high-level state machine:
IDLE -> NAVIGATING -> CLEANING -> REPORTING -> IDLE
Interacts with task queue, crawler controller, and verification service.
Publishes CLEANING belief update when cleaning cycle begins.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from hull_msgs.msg import TaskQueue, BeliefUpdate
from hull_msgs.srv import TaskComplete


class ToolController(Node):
    def __init__(self):
        super().__init__('tool_controller')

        self.declare_parameter('robot_id', 'cleaner_1')
        self.declare_parameter('clean_duration', 2.8)
        self.declare_parameter('arrival_distance', 0.40)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.clean_duration = self.get_parameter('clean_duration').get_parameter_value().double_value
        self.arrival_dist = self.get_parameter('arrival_distance').get_parameter_value().double_value

        self.state = 'IDLE'
        self.current_task = None
        self.current_pose = None
        self.clean_start_time = 0.0

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/pose', self._pose_cb, 10)
        self.queue_sub = self.create_subscription(
            TaskQueue, '/task_queue', self._queue_cb, 10)

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, f'/{self.robot_id}/target_goal', 10)
        self.tool_cmd_pub = self.create_publisher(
            Bool, f'/{self.robot_id}/tool_cmd', 10)
        self.belief_pub = self.create_publisher(
            BeliefUpdate, '/belief_updates', 20)

        # Service client for task completion
        self.complete_cli = self.create_client(TaskComplete, '/task_complete')

        # State machine update loop (10 Hz)
        self.timer = self.create_timer(0.1, self._state_loop)
        self.get_logger().info(f'ToolController active for {self.robot_id}')

    def _pose_cb(self, msg):
        self.current_pose = msg.pose

    def _queue_cb(self, msg):
        if self.state == 'IDLE' and msg.tasks:
            # Pick highest priority task that is QUEUED
            for task in msg.tasks:
                if task.status in ('QUEUED', 'RE_QUEUED'):
                    self.current_task = task
                    self.state = 'NAVIGATING'
                    # Dispatch goal
                    ps = PoseStamped()
                    ps.header.stamp = self.get_clock().now().to_msg()
                    ps.header.frame_id = 'hull_world'
                    ps.pose.position = task.position
                    self.goal_pub.publish(ps)
                    self.get_logger().info(f"[{self.robot_id}] Assigned task {task.defect_id} (utility: {task.utility_score:.2f}) -> NAVIGATING")
                    break

    def _state_loop(self):
        if not self.current_pose or not self.current_task:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        tx = self.current_task.position.x
        ty = self.current_task.position.y
        dist = math.hypot(tx - x, ty - y)

        if self.state == 'NAVIGATING':
            if dist <= self.arrival_dist:
                # Arrived at defect site -> CLEANING
                self.state = 'CLEANING'
                self.clean_start_time = time.time()
                # Activate cleaning tool
                self.tool_cmd_pub.publish(Bool(data=True))

                # Publish BeliefUpdate -> CLEANING
                b_msg = BeliefUpdate()
                b_msg.defect_id = self.current_task.defect_id
                b_msg.status = 'CLEANING'
                b_msg.severity = 0.5
                b_msg.confidence = 0.95
                b_msg.stamp = self.get_clock().now().to_msg()
                self.belief_pub.publish(b_msg)

                self.get_logger().info(f"[{self.robot_id}] Reached defect {self.current_task.defect_id} -> Tool Active -> CLEANING")

        elif self.state == 'CLEANING':
            elapsed = time.time() - self.clean_start_time
            if elapsed >= self.clean_duration:
                # Deactivate cleaning tool
                self.tool_cmd_pub.publish(Bool(data=False))
                self.state = 'REPORTING'
                self.get_logger().info(f"[{self.robot_id}] Cleaning cycle finished -> Calling /task_complete")
                self._report_complete()

        elif self.state == 'REPORTING':
            pass

    def _report_complete(self):
        if not self.complete_cli.service_is_ready():
            self.get_logger().warn('/task_complete service not available yet, waiting...')
            return

        req = TaskComplete.Request()
        req.defect_id = self.current_task.defect_id
        req.robot_id = self.robot_id
        req.reported_success = True

        future = self.complete_cli.call_async(req)
        future.add_done_callback(self._report_response_cb)

    def _report_response_cb(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"[{self.robot_id}] /task_complete response: {res.message}")
        except Exception as e:
            self.get_logger().error(f"[{self.robot_id}] /task_complete call failed: {e}")
        finally:
            self.current_task = None
            self.state = 'IDLE'


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ToolController()
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
