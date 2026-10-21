#!/usr/bin/env python3
"""
HullSentinal V2 — Utility Scorer Node
Computes dynamic multi-attribute priority scores for all active defects using:
- Real 8-connected A* obstacle-inflated path cost
- Structural risk type weights
- True belief severity & confidence
- Re-queue escalation bonus
- Immediate emergency override bonus (+10.0)
- Lane eligibility gating (ineligible tasks held at lowest priority)
"""

import os
import math
import heapq
import time
import yaml
import traceback
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

from hull_msgs.msg import TaskQueue, TaskItem
from hull_control.grid_planner import GridPlanner


class UtilityScorer(Node):
    def __init__(self):
        super().__init__('utility_scorer')

        self.declare_parameter('config_file', '')
        self.declare_parameter('cleaner_robot_id', 'cleaner_1')

        self.cleaner_id = self.get_parameter('cleaner_robot_id').get_parameter_value().string_value

        # Load master config
        self.cfg = self._load_config()
        util_cfg = self.cfg.get('utility', {})
        self.w_sev = float(util_cfg.get('w1_severity_risk', 1.00))
        self.w_path = float(util_cfg.get('w2_path_cost', 0.08))
        self.w_age = float(util_cfg.get('w3_age_bonus', 0.05))
        self.requeue_bonus = float(util_cfg.get('requeue_bonus', 0.20))
        self.emergency_bonus = float(util_cfg.get('emergency_override_bonus', 10.0))

        risk_w = util_cfg.get('risk_weights', {})
        self.w_crack = float(risk_w.get('crack', 1.00))
        self.w_rust = float(risk_w.get('rust', 0.70))
        self.w_bio = float(risk_w.get('biofouling', 0.40))

        # Instantiate A* Grid Planner with 0.55m obstacle inflation
        obstacles = self.cfg.get('static_obstacles', [])
        world_cfg = self.cfg.get('world', {})
        hw = float(world_cfg.get('hull_width', 24.0))
        hh = float(world_cfg.get('hull_height', 8.0))
        margin = float(world_cfg.get('safety_margin', 0.8))
        self.grid_planner = GridPlanner(
            hull_width=hw,
            hull_height=hh,
            resolution=0.20,
            inflation_radius=0.55,
            safety_margin=margin,
            obstacles=obstacles
        )

        self.cleaner_pose = (1.5, 7.0)  # Default start spawn
        self.active_tasks = []
        self.last_log_time = 0.0

        # Subscriptions
        self.active_sub = self.create_subscription(
            TaskQueue, '/belief_map/active_defects', self._active_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.cleaner_id}/pose', self._pose_cb, 10)

        # Publisher for prioritized task queue
        self.queue_pub = self.create_publisher(TaskQueue, '/task_queue', 10)

        # Re-score loop timer (2 Hz)
        self.timer = self.create_timer(0.5, self._score_and_publish)
        self.get_logger().info(
            f'[UTILITY] UtilityScorer V2 initialized with A* grid planner '
            f'({len(obstacles)} obstacles, weights: sev={self.w_sev}, path={self.w_path}, emerg={self.emergency_bonus})'
        )

    def _load_config(self):
        custom_path = self.get_parameter('config_file').get_parameter_value().string_value
        candidate_paths = [
            custom_path,
            os.path.join(get_package_share_directory('hull_bringup'), 'config', 'hullsentinal_v2.yaml')
            if 'hull_bringup' in os.environ.get('AMENT_PREFIX_PATH', '') else '',
            '/home/pritish_3110/Personal Files/Syllabus/Projects/HullSentinal/ws/src/hull_bringup/config/hullsentinal_v2.yaml',
        ]
        for path in candidate_paths:
            if path and os.path.exists(path):
                with open(path, 'r') as f:
                    return yaml.safe_load(f)
        return {}

    def _pose_cb(self, msg: PoseStamped):
        self.cleaner_pose = (msg.pose.position.x, msg.pose.position.y)

    def _active_cb(self, msg: TaskQueue):
        self.active_tasks = msg.tasks

    def _score_and_publish(self):
        if not self.active_tasks:
            # Still publish, otherwise subscribers keep rendering the last
            # non-empty queue forever once every defect is verified clean.
            empty = TaskQueue()
            empty.header.stamp = self.get_clock().now().to_msg()
            self.queue_pub.publish(empty)
            return

        cx, cy = self.cleaner_pose
        scored_heap = []
        counter = 0

        for task in self.active_tasks:
            dtype = task.defect_type.lower()
            if 'crack' in dtype:
                w_risk = self.w_crack
            elif 'rust' in dtype or 'corrosion' in dtype:
                w_risk = self.w_rust
            else:
                w_risk = self.w_bio

            # Compute real A* obstacle-inflated path distance
            gx, gy = task.position.x, task.position.y
            path_len = self.grid_planner.get_path_length(self.cleaner_pose, (gx, gy))

            # Severity from message (fallback to type base if 0)
            sev = task.severity if task.severity > 0.01 else (
                0.85 if 'crack' in dtype else (0.70 if 'rust' in dtype else 0.55)
            )

            # Re-queue escalation bonus
            r_bonus = self.requeue_bonus if task.status == 'RE_QUEUED' else 0.0

            # Emergency override bonus
            is_emergency = task.emergency_flag or (task.risk_level == 'CRITICAL')
            e_bonus = self.emergency_bonus if is_emergency else 0.0

            # Eligibility gating: if lane not yet released and not emergency, penalize
            is_eligible = task.eligible or is_emergency
            if not is_eligible:
                utility = -500.0 - path_len
            else:
                # Core utility formula
                utility = (self.w_sev * sev * w_risk) - (self.w_path * path_len) + r_bonus + e_bonus

            task_copy = TaskItem()
            task_copy.defect_id = task.defect_id
            task_copy.defect_type = task.defect_type
            task_copy.utility_score = float(utility)
            task_copy.position = task.position
            task_copy.status = 'QUEUED' if task.status in ('DETECTED', 'PENDING') else task.status
            task_copy.lane_id = int(task.lane_id)
            task_copy.risk_level = task.risk_level
            task_copy.emergency_flag = bool(is_emergency)
            task_copy.eligible = bool(is_eligible)
            task_copy.path_cost = float(path_len)
            task_copy.severity = float(sev)
            task_copy.confidence = float(task.confidence)

            # Push (-utility, counter, task_copy)
            counter += 1
            heapq.heappush(scored_heap, (-utility, counter, task_copy))

        # Build ordered queue
        ordered_tasks = []
        while scored_heap:
            _, _, t_item = heapq.heappop(scored_heap)
            ordered_tasks.append(t_item)

        # Log top choice breakdown
        cur_t = time.time()
        if ordered_tasks and (cur_t - self.last_log_time >= 3.0):
            self.last_log_time = cur_t
            top = ordered_tasks[0]
            if top.eligible:
                self.get_logger().info(
                    f"[UTILITY] Top Task: {top.defect_id} ({top.defect_type}) Lane {top.lane_id} | "
                    f"Score: {top.utility_score:.2f} [sev={top.severity:.2f}, path={top.path_cost:.2f}m, "
                    f"risk={top.risk_level}, emerg={top.emergency_flag}]"
                )
            else:
                self.get_logger().info(
                    f"[UTILITY] Top Task: {top.defect_id} (HELD - WAITING_FOR_LANE {top.lane_id})"
                )

        out_msg = TaskQueue()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.tasks = ordered_tasks
        self.queue_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = UtilityScorer()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # Never swallow a fatal error: a silent exit makes a dead node look healthy.
        if node is not None:
            node.get_logger().fatal('[UTILITY] Fatal error, shutting down:\n' + traceback.format_exc())
        else:
            traceback.print_exc()
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
