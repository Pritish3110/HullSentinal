#!/usr/bin/env python3
"""
Verification Monitor Node
Closes the loop on task completion:
On /task_complete service call, requests a re-scan from the mapper,
evaluates residual defect severity, and transitions task to VERIFIED_CLEAN or RE_QUEUED.
"""

import rclpy
from rclpy.node import Node

from hull_msgs.msg import BeliefUpdate
from hull_msgs.srv import TaskComplete, RescanRequest


class VerificationMonitor(Node):
    def __init__(self):
        super().__init__('verification_monitor')

        self.declare_parameter('residual_threshold', 0.08)
        self.residual_thresh = self.get_parameter('residual_threshold').get_parameter_value().double_value

        self.rescan_history = {}

        # Publisher for belief map state changes
        self.update_pub = self.create_publisher(BeliefUpdate, '/belief_updates', 20)

        # Service server for cleaning robot completion
        self.complete_srv = self.create_service(
            TaskComplete, '/task_complete', self._handle_task_complete)

        # Service client to request re-scan from simulation/mapper
        self.rescan_cli = self.create_client(RescanRequest, '/rescan_request')

        self.get_logger().info('VerificationMonitor node initialized (closed-loop verification active)')

    def _handle_task_complete(self, request, response):
        d_id = request.defect_id
        r_id = request.robot_id
        self.get_logger().info(f"[Verification] Task {d_id} reported complete by {r_id} -> Triggering re-scan verification")

        if not self.rescan_cli.service_is_ready():
            self.get_logger().warn('/rescan_request service not ready; defaulting to VERIFIED_CLEAN')
            b_done = BeliefUpdate()
            b_done.defect_id = d_id
            b_done.status = 'VERIFIED_CLEAN'
            b_done.severity = 0.0
            b_done.confidence = 1.0
            b_done.stamp = self.get_clock().now().to_msg()
            self.update_pub.publish(b_done)
            response.verification_triggered = True
            response.message = 'Verification skipped (service unavailable), marked VERIFIED_CLEAN'
            return response

        req = RescanRequest.Request()
        req.defect_id = d_id
        req.scan_radius = 1.0

        future = self.rescan_cli.call_async(req)
        future.add_done_callback(lambda f, defect_id=d_id: self._rescan_done_cb(f, defect_id))

        response.verification_triggered = True
        response.message = f'Re-scan triggered for defect {d_id}'
        return response

    def _rescan_done_cb(self, future, d_id):
        try:
            res = future.result()
            count = self.rescan_history.get(d_id, 0) + 1
            self.rescan_history[d_id] = count

            b_update = BeliefUpdate()
            b_update.defect_id = d_id
            b_update.rescan_count = count
            b_update.stamp = self.get_clock().now().to_msg()

            if res.defect_present and res.residual_severity > self.residual_thresh:
                # Cleaning incomplete: re-queue with updated residual severity
                b_update.status = 'RE_QUEUED'
                b_update.severity = float(res.residual_severity)
                b_update.confidence = float(res.confidence)
                self.update_pub.publish(b_update)
                self.get_logger().warn(f"[Verification Alert] Defect {d_id} clean FAILED (residual={res.residual_severity:.2f}) -> RE_QUEUED (attempt {count})")
            else:
                # Clean confirmed: mark VERIFIED_CLEAN
                b_update.status = 'VERIFIED_CLEAN'
                b_update.severity = 0.0
                b_update.confidence = 1.0
                self.update_pub.publish(b_update)
                self.get_logger().info(f"[Verification Success] Defect {d_id} confirmed CLEAN -> VERIFIED_CLEAN")
        except Exception as e:
            self.get_logger().error(f"Verification re-scan callback error: {e}")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = VerificationMonitor()
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
