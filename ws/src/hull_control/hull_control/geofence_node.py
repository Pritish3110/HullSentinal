#!/usr/bin/env python3
"""
HullSentinal V2 — Geofence Monitor Node
Continuously validates that all robots remain strictly within the vessel hull manifold.
Master configuration loaded from hullsentinal_v2.yaml.
Hull dimensions: 24.0m x 8.0m. Safe operational zone: [0.8, 23.2] x [0.8, 7.2].
"""

import os
import json
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('robots', ['mapper_1', 'cleaner_1'])

        self.cfg = self._load_config()
        world_cfg = self.cfg.get('world', {})
        self.hull_width = float(world_cfg.get('hull_width', 24.0))
        self.hull_height = float(world_cfg.get('hull_height', 8.0))
        self.safety_margin = float(world_cfg.get('safety_margin', 0.8))

        self.x_min = self.safety_margin
        self.x_max = self.hull_width - self.safety_margin
        self.y_min = self.safety_margin
        self.y_max = self.hull_height - self.safety_margin

        self.hard_x_min = 0.3
        self.hard_x_max = self.hull_width - 0.3
        self.hard_y_min = 0.3
        self.hard_y_max = self.hull_height - 0.3

        self.robot_ids = list(self.cfg.get('robots', {}).keys()) or ['mapper_1', 'cleaner_1']

        self.poses = {}
        self.out_of_bounds_count = 0
        self.warning_count = 0

        # Subscriptions for robot poses
        self.subs = {}
        for rid in self.robot_ids:
            self.poses[rid] = None
            self.subs[rid] = self.create_subscription(
                PoseStamped, f'/{rid}/pose', lambda msg, r=rid: self._pose_cb(r, msg), 10)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/geofence_status', 10)

        # Monitor loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self._monitor_loop)
        self.get_logger().info(
            f'[SAFETY] GeofenceNode V2 active for robots {self.robot_ids} '
            f'(Safe zone: [{self.x_min:.1f}, {self.x_max:.1f}] x [{self.y_min:.1f}, {self.y_max:.1f}], '
            f'Hard bounds: [{self.hard_x_min:.1f}, {self.hard_x_max:.1f}] x [{self.hard_y_min:.1f}, {self.hard_y_max:.1f}])'
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

    def _pose_cb(self, rid, msg):
        self.poses[rid] = (msg.pose.position.x, msg.pose.position.y)

    def _monitor_loop(self):
        all_in_bounds = True
        robot_status = {}

        for rid, pos in self.poses.items():
            if pos is None:
                continue

            x, y = pos
            in_safe = (self.x_min <= x <= self.x_max) and (self.y_min <= y <= self.y_max)
            in_hard = (self.hard_x_min <= x <= self.hard_x_max) and (self.hard_y_min <= y <= self.hard_y_max)

            if not in_hard:
                self.out_of_bounds_count += 1
                all_in_bounds = False
                status = 'CRITICAL_OOB'
                self.get_logger().error(
                    f"[SAFETY] GEOFENCE BREACH: {rid} breached hard hull bounds at ({x:.2f}, {y:.2f})!"
                )
            elif not in_safe:
                self.warning_count += 1
                status = 'PERIMETER_WARNING'
            else:
                status = 'NOMINAL'

            robot_status[rid] = {
                'x': x,
                'y': y,
                'status': status,
                'in_safe_zone': in_safe,
                'in_hard_bounds': in_hard
            }

        status_msg = {
            'out_of_bounds_count': self.out_of_bounds_count,
            'warning_count': self.warning_count,
            'all_in_bounds': all_in_bounds,
            'robots': robot_status
        }

        msg = String()
        msg.data = json.dumps(status_msg)
        self.status_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = GeofenceNode()
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
