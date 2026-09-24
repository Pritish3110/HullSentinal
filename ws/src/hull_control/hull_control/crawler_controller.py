#!/usr/bin/env python3
"""
Crawler Controller Node
Generates surface motion commands for mapper robot (sweep pattern)
and cleaner robots (point-to-point targeting).
Strictly clamps all targets and velocities within the hull workspace bounds.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class CrawlerController(Node):
    def __init__(self):
        super().__init__('crawler_controller')

        self.declare_parameter('robot_id', 'mapper_1')
        self.declare_parameter('role', 'mapper')
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('x_min', 0.8)
        self.declare_parameter('x_max', 19.2)
        self.declare_parameter('y_min', 0.8)
        self.declare_parameter('y_max', 5.2)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.role = self.get_parameter('role').get_parameter_value().string_value
        self.v_max = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.w_max = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.x_min = self.get_parameter('x_min').get_parameter_value().double_value
        self.x_max = self.get_parameter('x_max').get_parameter_value().double_value
        self.y_min = self.get_parameter('y_min').get_parameter_value().double_value
        self.y_max = self.get_parameter('y_max').get_parameter_value().double_value

        self.current_pose = None
        self.target_goal = None
        self.sweep_index = 0

        # Define lawnmower sweep waypoints for mapper (well within bounds [0.8, 19.2] x [0.8, 5.2])
        self.sweep_waypoints = [
            (2.0, 1.5), (6.0, 1.5), (10.0, 1.5), (14.0, 1.5), (18.0, 1.5),
            (18.0, 3.0), (14.0, 3.0), (10.0, 3.0), (6.0, 3.0), (2.0, 3.0),
            (2.0, 4.5), (6.0, 4.5), (10.0, 4.5), (14.0, 4.5), (18.0, 4.5),
            (10.0, 3.0), (2.0, 1.5)
        ]

        # Pose subscription
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/pose', self._pose_cb, 10)

        # Target goal subscription (for cleaner)
        self.goal_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_id}/target_goal', self._goal_cb, 10)

        # Raw velocity publisher (routed through safety barrier)
        self.cmd_pub = self.create_publisher(
            Twist, f'/{self.robot_id}/cmd_vel_raw', 10)

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info(f'CrawlerController started for {self.robot_id} (role: {self.role}, bounds: [{self.x_min}, {self.x_max}] x [{self.y_min}, {self.y_max}])')

    def _pose_cb(self, msg):
        self.current_pose = msg.pose

    def _goal_cb(self, msg):
        # Strictly clamp received goal inside workspace bounds
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

        # Select target waypoint based on role
        if self.role == 'mapper':
            gx, gy = self.sweep_waypoints[self.sweep_index]
            dist = math.hypot(gx - x, gy - y)
            if dist < self.goal_tol:
                self.sweep_index = (self.sweep_index + 1) % len(self.sweep_waypoints)
                gx, gy = self.sweep_waypoints[self.sweep_index]
        else:
            if not self.target_goal:
                # Idle stationary
                self.cmd_pub.publish(Twist())
                return
            gx, gy = self.target_goal
            dist = math.hypot(gx - x, gy - y)
            if dist < self.goal_tol:
                # At goal
                self.cmd_pub.publish(Twist())
                return

        # Ensure goal remains clamped
        gx = float(max(self.x_min, min(self.x_max, gx)))
        gy = float(max(self.y_min, min(self.y_max, gy)))

        # Pure pursuit / proportional heading & velocity guidance
        desired_yaw = math.atan2(gy - y, gx - x)
        yaw_err = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))
        dist = math.hypot(gx - x, gy - y)

        cmd = Twist()
        if abs(yaw_err) > 0.6:
            # Turn in place if heading error is large
            cmd.linear.x = 0.05
            cmd.angular.z = float(max(-self.w_max, min(self.w_max, 2.0 * yaw_err)))
        else:
            # Drive forward with proportional steering
            speed = min(self.v_max, max(0.15, 0.8 * dist))
            cmd.linear.x = float(speed)
            cmd.angular.z = float(max(-self.w_max, min(self.w_max, 1.8 * yaw_err)))

        # Proactive edge braking: prevent driving towards perimeter if already near edge
        heading_x = math.cos(yaw)
        heading_y = math.sin(yaw)
        if (x <= self.x_min + 0.1 and heading_x < 0) or (x >= self.x_max - 0.1 and heading_x > 0):
            cmd.linear.x = min(cmd.linear.x, 0.05)
        if (y <= self.y_min + 0.1 and heading_y < 0) or (y >= self.y_max - 0.1 and heading_y > 0):
            cmd.linear.x = min(cmd.linear.x, 0.05)

        self.cmd_pub.publish(cmd)

    @staticmethod
    def _quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CrawlerController()
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
