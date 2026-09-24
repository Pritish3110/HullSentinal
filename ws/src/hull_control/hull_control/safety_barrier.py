#!/usr/bin/env python3
"""
Safety Barrier Node
Filters raw velocity commands using control barrier certificates / distance margins
to prevent collisions between mapping and cleaning robots on the hull surface,
and repels robots away from the perimeter to prevent off-grid excursions.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class SafetyBarrier(Node):
    def __init__(self):
        super().__init__('safety_barrier')

        self.declare_parameter('robots', ['mapper_1', 'cleaner_1'])
        self.declare_parameter('safety_radius', 0.55)
        self.declare_parameter('x_min', 0.8)
        self.declare_parameter('x_max', 19.2)
        self.declare_parameter('y_min', 0.8)
        self.declare_parameter('y_max', 5.2)

        self.robot_ids = self.get_parameter('robots').get_parameter_value().string_array_value
        self.safety_r = self.get_parameter('safety_radius').get_parameter_value().double_value
        self.x_min = self.get_parameter('x_min').get_parameter_value().double_value
        self.x_max = self.get_parameter('x_max').get_parameter_value().double_value
        self.y_min = self.get_parameter('y_min').get_parameter_value().double_value
        self.y_max = self.get_parameter('y_max').get_parameter_value().double_value

        self.poses = {}
        self.raw_cmds = {}
        self.pose_subs = {}
        self.cmd_subs = {}
        self.cmd_pubs = {}

        for rid in self.robot_ids:
            self.poses[rid] = None
            self.raw_cmds[rid] = Twist()
            self.pose_subs[rid] = self.create_subscription(
                PoseStamped, f'/{rid}/pose', lambda msg, r=rid: self._pose_cb(r, msg), 10)
            self.cmd_subs[rid] = self.create_subscription(
                Twist, f'/{rid}/cmd_vel_raw', lambda msg, r=rid: self._cmd_cb(r, msg), 10)
            self.cmd_pubs[rid] = self.create_publisher(
                Twist, f'/{rid}/cmd_vel', 10)

        # Filtering timer (20 Hz)
        self.timer = self.create_timer(0.05, self._filter_loop)
        self.get_logger().info(f'SafetyBarrier active for robots: {self.robot_ids} with hull perimeter containment')

    def _pose_cb(self, rid, msg):
        self.poses[rid] = (msg.pose.position.x, msg.pose.position.y)

    def _cmd_cb(self, rid, msg):
        self.raw_cmds[rid] = msg

    def _filter_loop(self):
        for rid in self.robot_ids:
            cmd = self.raw_cmds[rid]
            pos = self.poses[rid]

            if not pos:
                self.cmd_pubs[rid].publish(cmd)
                continue

            # 1. Check proximities to other robots
            min_dist = float('inf')
            for other_id in self.robot_ids:
                if other_id == rid or not self.poses[other_id]:
                    continue
                opos = self.poses[other_id]
                d = math.hypot(pos[0] - opos[0], pos[1] - opos[1])
                if d < min_dist:
                    min_dist = d

            safe_cmd = Twist()
            safe_cmd.angular.z = cmd.angular.z

            if min_dist < 0.35:
                # Critical collision proximity: complete halt & rotate away
                safe_cmd.linear.x = 0.0
                safe_cmd.angular.z = 0.5
            elif min_dist < self.safety_r:
                # Scale velocity down proportionally
                vel_factor = max(0.2, (min_dist - 0.35) / (self.safety_r - 0.35))
                safe_cmd.linear.x = cmd.linear.x * vel_factor
            else:
                safe_cmd.linear.x = cmd.linear.x

            # 2. Perimeter Containment Barrier
            px, py = pos
            if px < self.x_min:
                safe_cmd.linear.x = min(safe_cmd.linear.x, 0.0)
                safe_cmd.angular.z = 0.6
            elif px > self.x_max:
                safe_cmd.linear.x = min(safe_cmd.linear.x, 0.0)
                safe_cmd.angular.z = 0.6

            if py < self.y_min:
                safe_cmd.linear.x = min(safe_cmd.linear.x, 0.0)
                safe_cmd.angular.z = 0.6
            elif py > self.y_max:
                safe_cmd.linear.x = min(safe_cmd.linear.x, 0.0)
                safe_cmd.angular.z = 0.6

            self.cmd_pubs[rid].publish(safe_cmd)


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
