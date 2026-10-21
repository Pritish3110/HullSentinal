#!/usr/bin/env python3
"""
HullSentinal V2 — Crawler Controller Node
Generates surface motion commands for cleaner robots (following A* waypoints to target defect).
Master configuration loaded from hullsentinal_v2.yaml.
Clamps all targets and velocities within the 24m x 8m hull workspace bounds.
"""

import os
import math
import yaml
import traceback
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from ament_index_python.packages import get_package_share_directory


class CrawlerController(Node):
    def __init__(self):
        super().__init__('crawler_controller')

        self.declare_parameter('robot_id', 'cleaner_1')
        self.declare_parameter('role', 'cleaner')
        self.declare_parameter('config_file', '')
        self.declare_parameter('goal_tolerance', 0.30)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.role = self.get_parameter('role').get_parameter_value().string_value
        self.goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        # Load master config
        self.cfg = self._load_config()
        world_cfg = self.cfg.get('world', {})
        self.hull_width = float(world_cfg.get('hull_width', 24.0))
        self.hull_height = float(world_cfg.get('hull_height', 8.0))
        self.safety_margin = float(world_cfg.get('safety_margin', 0.8))

        self.x_min = self.safety_margin
        self.x_max = self.hull_width - self.safety_margin
        self.y_min = self.safety_margin
        self.y_max = self.hull_height - self.safety_margin

        r_cfg = self.cfg.get('robots', {}).get(self.robot_id, {})
        self.v_max = float(r_cfg.get('max_linear_speed', 1.40))
        self.w_max = float(r_cfg.get('max_angular_speed', 1.50))

        self.current_pose = None
        self.target_goal = None

        # Pose subscription
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/pose', self._pose_cb, 10)

        # Target goal subscription
        self.goal_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/target_goal', self._goal_cb, 10)

        # Raw velocity publisher (routed through safety barrier)
        self.cmd_pub = self.create_publisher(
            Twist, f'/{self.robot_id}/cmd_vel_raw', 10)

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info(
            f'[CLEANER] CrawlerController active for {self.robot_id} (role: {self.role}, '
            f'bounds: [{self.x_min:.1f}, {self.x_max:.1f}] x [{self.y_min:.1f}, {self.y_max:.1f}], v_max={self.v_max} m/s)'
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
        self.current_pose = msg.pose

    def _goal_cb(self, msg: PoseStamped):
        raw_x = msg.pose.position.x
        raw_y = msg.pose.position.y
        clamped_x = float(max(self.x_min, min(self.x_max, raw_x)))
        clamped_y = float(max(self.y_min, min(self.y_max, raw_y)))
        self.target_goal = (clamped_x, clamped_y)

    def _control_loop(self):
        if not self.current_pose:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self._quat_to_yaw(self.current_pose.orientation)

        if not self.target_goal:
            # Idle stationary
            self.cmd_pub.publish(Twist())
            return

        gx, gy = self.target_goal
        gx = float(max(self.x_min, min(self.x_max, gx)))
        gy = float(max(self.y_min, min(self.y_max, gy)))

        dist = math.hypot(gx - x, gy - y)
        if dist < self.goal_tol:
            # Reached waypoint
            self.cmd_pub.publish(Twist())
            return

        # Pure pursuit guidance
        desired_yaw = math.atan2(gy - y, gx - x)
        yaw_err = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))

        cmd = Twist()
        if abs(yaw_err) > 0.5:
            # Turn towards waypoint
            cmd.linear.x = 0.08
            cmd.angular.z = float(max(-self.w_max, min(self.w_max, 2.2 * yaw_err)))
        else:
            speed = min(self.v_max, max(0.20, 0.9 * dist))
            cmd.linear.x = float(speed)
            cmd.angular.z = float(max(-self.w_max, min(self.w_max, 1.8 * yaw_err)))

        # Boundary containment
        hx = math.cos(yaw)
        hy = math.sin(yaw)
        if (x <= self.x_min + 0.15 and hx < 0) or (x >= self.x_max - 0.15 and hx > 0):
            cmd.linear.x = min(cmd.linear.x, 0.05)
        if (y <= self.y_min + 0.15 and hy < 0) or (y >= self.y_max - 0.15 and hy > 0):
            cmd.linear.x = min(cmd.linear.x, 0.05)

        self.cmd_pub.publish(cmd)

    @staticmethod
    def _quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = CrawlerController()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # Never swallow a fatal error: a silent exit makes a dead node look healthy.
        if node is not None:
            node.get_logger().fatal('[CLEANER] Fatal error, shutting down:\n' + traceback.format_exc())
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
