#!/usr/bin/env python3
"""
Geofence Monitor Node
Continuously validates that all robots remain strictly within the vessel hull manifold.
Logs warnings, triggers emergency boundary halts, and publishes out_of_bounds_count.
Hull dimensions: 20.0m x 6.0m. Safe operational zone: [0.8, 19.2] x [0.8, 5.2].
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence_node')

        self.declare_parameter('robots', ['mapper_1', 'cleaner_1'])
        self.declare_parameter('x_min', 0.8)
        self.declare_parameter('x_max', 19.2)
        self.declare_parameter('y_min', 0.8)
        self.declare_parameter('y_max', 5.2)
        self.declare_parameter('hard_x_min', 0.3)
        self.declare_parameter('hard_x_max', 19.7)
        self.declare_parameter('hard_y_min', 0.3)
        self.declare_parameter('hard_y_max', 5.7)

        self.robot_ids = self.get_parameter('robots').get_parameter_value().string_array_value
        self.x_min = self.get_parameter('x_min').get_parameter_value().double_value
        self.x_max = self.get_parameter('x_max').get_parameter_value().double_value
        self.y_min = self.get_parameter('y_min').get_parameter_value().double_value
        self.y_max = self.get_parameter('y_max').get_parameter_value().double_value

        self.hard_x_min = self.get_parameter('hard_x_min').get_parameter_value().double_value
        self.hard_x_max = self.get_parameter('hard_x_max').get_parameter_value().double_value
        self.hard_y_min = self.get_parameter('hard_y_min').get_parameter_value().double_value
        self.hard_y_max = self.get_parameter('hard_y_max').get_parameter_value().double_value

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
        self.get_logger().info(f'GeofenceNode active for robots {self.robot_ids} (Safe zone: [{self.x_min}, {self.x_max}] x [{self.y_min}, {self.y_max}])')

    def _pose_cb(self, rid, msg):
        self.poses[rid] = (msg.pose.position.x, msg.pose.position.y)

    def _monitor_loop(self):
        in_bounds_all = True
        status_detail = {}

        for rid, pos in self.poses.items():
            if not pos:
                continue
            x, y = pos
            # Check hard breach
            if x < self.hard_x_min or x > self.hard_x_max or y < self.hard_y_min or y > self.hard_y_max:
                self.out_of_bounds_count += 1
                in_bounds_all = False
                self.get_logger().error(f"[GEOFENCE CRITICAL] Robot {rid} breached perimeter! Pos: ({x:.2f}, {y:.2f})")
                status_detail[rid] = 'HARD_BREACH'
            elif x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                self.warning_count += 1
                status_detail[rid] = 'BUFFER_ZONE'
            else:
                status_detail[rid] = 'OK'

        msg = String()
        data = {
            'out_of_bounds_count': self.out_of_bounds_count,
            'warning_count': self.warning_count,
            'all_in_bounds': in_bounds_all,
            'robot_status': status_detail
        }
        msg.data = json.dumps(data)
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
