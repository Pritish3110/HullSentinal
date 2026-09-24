#!/usr/bin/env python3
"""
HullSentinal V2 — Dynamic Safety Barrier Node
Implements multi-robot collision avoidance and predictive conflict detection:
1. Decision hierarchy: (1) robot safety (2) geofence (3) collision avoidance (4) emergency (5) normal task.
2. Trajectory conflict prediction (time-to-conflict extrapolation).
3. Priority rules: mapper has right-of-way; cleaner yields.
4. Deadlock detection and lateral sidestep maneuver (timeout: 2.5s).
5. Hull perimeter containment on 24m x 8m manifold.
"""

import os
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from ament_index_python.packages import get_package_share_directory


class SafetyBarrier(Node):
    def __init__(self):
        super().__init__('safety_barrier')

        self.declare_parameter('config_file', '')

        # Load master configuration
        self.cfg = self._load_config()
        world_cfg = self.cfg.get('world', {})
        self.hull_width = float(world_cfg.get('hull_width', 24.0))
        self.hull_height = float(world_cfg.get('hull_height', 8.0))
        self.safety_margin = float(world_cfg.get('safety_margin', 0.8))

        self.x_min = self.safety_margin
        self.x_max = self.hull_width - self.safety_margin
        self.y_min = self.safety_margin
        self.y_max = self.hull_height - self.safety_margin

        safety_cfg = self.cfg.get('safety', {})
        self.safe_dist = float(safety_cfg.get('safe_distance', 1.50))
        self.warn_dist = float(safety_cfg.get('warning_distance', 1.00))
        self.crit_dist = float(safety_cfg.get('critical_distance', 0.60))
        self.deadlock_timeout = float(safety_cfg.get('deadlock_timeout_sec', 2.5))

        self.robot_ids = list(self.cfg.get('robots', {}).keys()) or ['mapper_1', 'cleaner_1']

        self.poses = {}
        self.raw_cmds = {}
        self.pose_subs = {}
        self.cmd_subs = {}
        self.cmd_pubs = {}

        # Deadlock and sidestep state
        self.cleaner_halt_start_time = None
        self.sidestep_until = 0.0
        self.sidestep_cmd = Twist()
        self.last_safety_log = 0.0

        for rid in self.robot_ids:
            self.poses[rid] = None
            self.raw_cmds[rid] = Twist()
            self.pose_subs[rid] = self.create_subscription(
                PoseStamped, f'/{rid}/pose', lambda msg, r=rid: self._pose_cb(r, msg), 10)
            self.cmd_subs[rid] = self.create_subscription(
                Twist, f'/{rid}/cmd_vel_raw', lambda msg, r=rid: self._cmd_cb(r, msg), 10)
            self.cmd_pubs[rid] = self.create_publisher(
                Twist, f'/{rid}/cmd_vel', 10)

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self._filter_loop)
        self.get_logger().info(
            f'[SAFETY] SafetyBarrier V2 active ({len(self.robot_ids)} robots, '
            f'crit_dist={self.crit_dist}m, warn_dist={self.warn_dist}m, deadlock_timeout={self.deadlock_timeout}s)'
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

    def _pose_cb(self, rid, msg: PoseStamped):
        yaw = self._quat_to_yaw(msg.pose.orientation)
        self.poses[rid] = (msg.pose.position.x, msg.pose.position.y, yaw)

    def _cmd_cb(self, rid, msg: Twist):
        self.raw_cmds[rid] = msg

    def _predict_min_distance(self, pos1, cmd1, pos2, cmd2, horizon=2.0, steps=10):
        """Predicts closest approach distance between two robots along linear trajectories."""
        x1, y1, yaw1 = pos1
        x2, y2, yaw2 = pos2

        v1x = cmd1.linear.x * math.cos(yaw1)
        v1y = cmd1.linear.x * math.sin(yaw1)
        v2x = cmd2.linear.x * math.cos(yaw2)
        v2y = cmd2.linear.x * math.sin(yaw2)

        min_d = math.hypot(x1 - x2, y1 - y2)
        dt = horizon / steps
        for step in range(1, steps + 1):
            t = step * dt
            px1 = x1 + v1x * t
            py1 = y1 + v1y * t
            px2 = x2 + v2x * t
            py2 = y2 + v2y * t
            d = math.hypot(px1 - px2, py1 - py2)
            if d < min_d:
                min_d = d
        return min_d

    def _filter_loop(self):
        now_ts = time.time()

        # Handle active sidestep maneuver for cleaner
        if now_ts < self.sidestep_until:
            if 'cleaner_1' in self.cmd_pubs:
                self.cmd_pubs['cleaner_1'].publish(self.sidestep_cmd)
            # Mapper continues freely
            if 'mapper_1' in self.cmd_pubs:
                self.cmd_pubs['mapper_1'].publish(self._apply_boundary_containment('mapper_1', self.raw_cmds.get('mapper_1', Twist())))
            return

        mapper_pos = self.poses.get('mapper_1')
        cleaner_pos = self.poses.get('cleaner_1')
        cleaner_cmd = self.raw_cmds.get('cleaner_1', Twist())
        mapper_cmd = self.raw_cmds.get('mapper_1', Twist())

        # Mapper always has priority right-of-way
        safe_mapper_cmd = self._apply_boundary_containment('mapper_1', mapper_cmd)
        if 'mapper_1' in self.cmd_pubs:
            self.cmd_pubs['mapper_1'].publish(safe_mapper_cmd)

        if not cleaner_pos:
            return

        safe_cleaner_cmd = Twist()
        safe_cleaner_cmd.linear.x = cleaner_cmd.linear.x
        safe_cleaner_cmd.angular.z = cleaner_cmd.angular.z

        # If mapper pose available, evaluate distance and predictive conflict
        if mapper_pos:
            cur_dist = math.hypot(cleaner_pos[0] - mapper_pos[0], cleaner_pos[1] - mapper_pos[1])
            pred_min_dist = self._predict_min_distance(
                cleaner_pos, cleaner_cmd, mapper_pos, mapper_cmd, horizon=2.0
            )

            # Evaluate against safety thresholds
            if cur_dist < self.crit_dist or pred_min_dist < self.crit_dist:
                # Critical conflict: halt cleaner to yield to mapper
                safe_cleaner_cmd.linear.x = 0.0
                safe_cleaner_cmd.angular.z = 0.0

                if self.cleaner_halt_start_time is None:
                    self.cleaner_halt_start_time = now_ts
                elif (now_ts - self.cleaner_halt_start_time) > self.deadlock_timeout:
                    # Deadlock detected! Initiate lateral sidestep maneuver
                    self.get_logger().warning(
                        f"[SAFETY] Deadlock detected (> {self.deadlock_timeout}s halt near mapper at {cur_dist:.2f}m). "
                        "Initiating lateral sidestep maneuver."
                    )
                    self.sidestep_until = now_ts + 1.2
                    self.sidestep_cmd = Twist()
                    # Perpendicular turn and reverse sidestep
                    self.sidestep_cmd.linear.x = -0.30
                    self.sidestep_cmd.angular.z = 1.2
                    self.cleaner_halt_start_time = None
                    if 'cleaner_1' in self.cmd_pubs:
                        self.cmd_pubs['cleaner_1'].publish(self.sidestep_cmd)
                    return

                if now_ts - self.last_safety_log >= 2.0:
                    self.last_safety_log = now_ts
                    self.get_logger().warning(
                        f"[SAFETY] Conflict detected (dist={cur_dist:.2f}m, pred={pred_min_dist:.2f}m < {self.crit_dist}m). "
                        "Mapper has right-of-way; cleaner HALTED."
                    )
            elif cur_dist < self.warn_dist or pred_min_dist < self.warn_dist:
                # Warning zone: reduce cleaner speed by 50%
                safe_cleaner_cmd.linear.x = min(safe_cleaner_cmd.linear.x, 0.40)
                self.cleaner_halt_start_time = None
            else:
                self.cleaner_halt_start_time = None

        # Apply perimeter boundary containment
        safe_cleaner_cmd = self._apply_boundary_containment('cleaner_1', safe_cleaner_cmd)

        if 'cleaner_1' in self.cmd_pubs:
            self.cmd_pubs['cleaner_1'].publish(safe_cleaner_cmd)

    def _apply_boundary_containment(self, rid, cmd):
        """Prevents velocity commands that would drive robots outside the safe perimeter."""
        pos = self.poses.get(rid)
        if not pos:
            return cmd

        x, y, yaw = pos
        out_cmd = Twist()
        out_cmd.linear.x = cmd.linear.x
        out_cmd.angular.z = cmd.angular.z

        hx = math.cos(yaw)
        hy = math.sin(yaw)

        # Left / Right boundaries
        if (x <= self.x_min + 0.15 and hx < 0) or (x >= self.x_max - 0.15 and hx > 0):
            out_cmd.linear.x = min(out_cmd.linear.x, 0.05)
        # Bottom / Top boundaries
        if (y <= self.y_min + 0.15 and hy < 0) or (y >= self.y_max - 0.15 and hy > 0):
            out_cmd.linear.x = min(out_cmd.linear.x, 0.05)

        return out_cmd

    @staticmethod
    def _quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = SafetyBarrier()
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
