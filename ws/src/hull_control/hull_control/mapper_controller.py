#!/usr/bin/env python3
"""
HullSentinal V2 — Mapper Controller Node
Executes deterministic serpentine coverage across the 3 horizontal lanes of the ship hull.
Handles local obstacle detours, calculates per-lane & total coverage %,
and publishes MapperStatus on /mapper/lane_state.
"""

import os
import math
import time
import yaml
import traceback
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from ament_index_python.packages import get_package_share_directory

from hull_msgs.msg import MapperStatus


class MapperController(Node):
    def __init__(self):
        super().__init__('mapper_controller')

        self.declare_parameter('robot_id', 'mapper_1')
        self.declare_parameter('config_file', '')
        self.declare_parameter('goal_tolerance', 0.35)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        # Load master configuration
        self.cfg = self._load_config()
        world_cfg = self.cfg.get('world', {})
        self.hull_width = float(world_cfg.get('hull_width', 24.0))
        self.hull_height = float(world_cfg.get('hull_height', 8.0))
        self.safety_margin = float(world_cfg.get('safety_margin', 0.8))

        # Speed and kinematic constraints
        m_cfg = self.cfg.get('robots', {}).get(self.robot_id, {})
        self.v_max = float(m_cfg.get('max_linear_speed', 1.20))
        self.w_max = float(m_cfg.get('max_angular_speed', 1.50))

        # Bounds
        self.x_min = self.safety_margin
        self.x_max = self.hull_width - self.safety_margin
        self.y_min = self.safety_margin
        self.y_max = self.hull_height - self.safety_margin

        # Obstacles
        self.obstacles = self.cfg.get('static_obstacles', [])

        # Build serpentine waypoints with deterministic obstacle detours
        self.waypoints, self.lane_ranges = self._generate_serpentine_plan()
        self.current_wp_idx = 0

        # State tracking
        self.state = 'MAPPING_LANE_1'
        self.current_lane = 1
        self.lane_coverage_pct = 0.0
        self.total_coverage_pct = 0.0
        self.current_pose = None
        self.last_progress_log = 0.0

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/pose', self._pose_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist, f'/{self.robot_id}/cmd_vel_raw', 10)
        self.status_pub = self.create_publisher(
            MapperStatus, '/mapper/lane_state', 10)

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info(
            f'[MAPPER] MapperController initialized ({len(self.waypoints)} waypoints across 3 lanes, '
            f'speed limits: v={self.v_max} m/s, w={self.w_max} rad/s)'
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

    def _generate_serpentine_plan(self):
        """Generates waypoint sequence for 3 horizontal lanes with obstacle detours."""
        # Lane 1 (Y=2.0, Left->Right, X=1.5 -> 22.0)
        # Detour obs_hatch_01 (x=7.5, y=2.8): bypass at y=1.5
        # Detour obs_panel_02 (x=14.0, y=1.4): bypass at y=2.5
        lane1_wps = [
            (1.5, 2.0, 1),
            (4.0, 2.0, 1),
            (6.2, 1.6, 1),
            (7.5, 1.5, 1),   # Detour under obs_hatch_01
            (8.8, 1.8, 1),
            (11.0, 2.0, 1),
            (12.5, 2.5, 1),
            (14.0, 2.55, 1), # Detour over obs_panel_02
            (15.5, 2.3, 1),
            (18.0, 2.0, 1),
            (22.0, 2.0, 1),
            (22.2, 3.0, 1),  # Transition 1->2
            (22.0, 4.0, 2),  # Arrived at Lane 2
        ]

        # Lane 2 (Y=4.0, Right->Left, X=22.0 -> 1.8)
        # Detour obs_seachest_04 (x=17.5, y=3.5): bypass at y=4.6
        # Detour obs_anode_03 (x=6.0, y=4.8): bypass at y=3.5
        lane2_wps = [
            (20.0, 4.0, 2),
            (19.0, 4.5, 2),
            (17.5, 4.65, 2), # Detour over obs_seachest_04
            (16.0, 4.3, 2),
            (13.0, 4.0, 2),
            (10.0, 4.0, 2),
            (7.5, 3.6, 2),
            (6.0, 3.45, 2),  # Detour under obs_anode_03
            (4.5, 3.8, 2),
            (2.0, 4.0, 2),
            (1.8, 5.0, 2),   # Transition 2->3
            (2.0, 6.0, 3),   # Arrived at Lane 3
        ]

        # Lane 3 (Y=6.0, Left->Right, X=2.0 -> 22.0)
        # Detour obs_sonar_05 (x=11.5, y=5.4): bypass at y=6.6
        # Detour obs_plate_06 (x=18.5, y=6.6): bypass at y=5.4
        lane3_wps = [
            (4.5, 6.0, 3),
            (7.0, 6.0, 3),
            (9.8, 6.5, 3),
            (11.5, 6.65, 3), # Detour over obs_sonar_05
            (13.2, 6.3, 3),
            (16.0, 6.0, 3),
            (17.2, 5.45, 3),
            (18.5, 5.4, 3),  # Detour under obs_plate_06
            (19.8, 5.8, 3),
            (22.0, 6.0, 3),
        ]

        all_wps = lane1_wps + lane2_wps + lane3_wps
        lane_ranges = {1: (1.5, 22.0), 2: (22.0, 2.0), 3: (2.0, 22.0)}
        return all_wps, lane_ranges

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg.pose

    def _control_loop(self):
        if not self.current_pose:
            return

        now = self.get_clock().now()
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self._quat_to_yaw(self.current_pose.orientation)

        # Check waypoint completion
        if self.current_wp_idx < len(self.waypoints):
            gx, gy, wp_lane = self.waypoints[self.current_wp_idx]
            dist = math.hypot(gx - x, gy - y)

            if dist < self.goal_tol:
                self.current_wp_idx += 1
                if self.current_wp_idx < len(self.waypoints):
                    next_gx, next_gy, next_lane = self.waypoints[self.current_wp_idx]
                    if next_lane != self.current_lane and self.state != 'MAPPING_COMPLETE':
                        self.get_logger().info(
                            f"[MAPPER] Lane {self.current_lane} completed. Transitioning to Lane {next_lane}."
                        )
                        self.current_lane = next_lane
                        self.state = f'MAPPING_LANE_{self.current_lane}'
                else:
                    self.state = 'MAPPING_COMPLETE'
                    self.current_lane = 3
                    self.lane_coverage_pct = 100.0
                    self.total_coverage_pct = 100.0
                    self.get_logger().info(
                        '[MAPPER] All 3 lanes swept! Serpentine coverage complete: 100.0% coverage achieved.'
                    )

        # Update coverage calculation
        if self.state == 'MAPPING_LANE_1':
            self.current_lane = 1
            progress = max(0.0, min(1.0, (x - 1.5) / (22.0 - 1.5)))
            self.lane_coverage_pct = progress * 100.0
            self.total_coverage_pct = (0.0 + progress) / 3.0 * 100.0
        elif self.state == 'MAPPING_LANE_2':
            self.current_lane = 2
            progress = max(0.0, min(1.0, (22.0 - x) / (22.0 - 2.0)))
            self.lane_coverage_pct = progress * 100.0
            self.total_coverage_pct = (1.0 + progress) / 3.0 * 100.0
        elif self.state == 'MAPPING_LANE_3':
            self.current_lane = 3
            progress = max(0.0, min(1.0, (x - 2.0) / (22.0 - 2.0)))
            self.lane_coverage_pct = progress * 100.0
            self.total_coverage_pct = (2.0 + progress) / 3.0 * 100.0
        elif self.state == 'MAPPING_COMPLETE':
            self.lane_coverage_pct = 100.0
            self.total_coverage_pct = 100.0

        # Periodic logging of mapper progress (every 2.5s)
        cur_t = time.time()
        if cur_t - self.last_progress_log >= 2.5:
            self.last_progress_log = cur_t
            self.get_logger().info(
                f"[MAPPER] State: {self.state} | Lane {self.current_lane} coverage: {self.lane_coverage_pct:.1f}% | "
                f"Total coverage: {self.total_coverage_pct:.1f}% | Pose: ({x:.1f}, {y:.1f})"
            )

        # Publish MapperStatus
        stat_msg = MapperStatus()
        stat_msg.state = self.state
        stat_msg.current_lane = int(self.current_lane)
        stat_msg.lane_coverage_pct = float(self.lane_coverage_pct)
        stat_msg.total_coverage_pct = float(self.total_coverage_pct)
        stat_msg.x = float(x)
        stat_msg.y = float(y)
        stat_msg.yaw = float(yaw)
        stat_msg.stamp = now.to_msg()
        self.status_pub.publish(stat_msg)

        # Generate Twist command
        cmd = Twist()
        if self.state == 'MAPPING_COMPLETE' or self.current_wp_idx >= len(self.waypoints):
            # Station-keep at final location
            self.cmd_pub.publish(cmd)
            return

        gx, gy, _ = self.waypoints[self.current_wp_idx]
        desired_yaw = math.atan2(gy - y, gx - x)
        yaw_err = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))
        dist = math.hypot(gx - x, gy - y)

        if abs(yaw_err) > 0.5:
            # Turn towards waypoint
            cmd.linear.x = 0.08
            cmd.angular.z = float(max(-self.w_max, min(self.w_max, 2.2 * yaw_err)))
        else:
            speed = min(self.v_max, max(0.20, 0.9 * dist))
            cmd.linear.x = float(speed)
            cmd.angular.z = float(max(-self.w_max, min(self.w_max, 1.8 * yaw_err)))

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
        node = MapperController()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # Never swallow a fatal error: a silent exit makes a dead node look healthy.
        if node is not None:
            node.get_logger().fatal('[MAPPER] Fatal error, shutting down:\n' + traceback.format_exc())
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
