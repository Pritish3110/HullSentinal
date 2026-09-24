#!/usr/bin/env python3
"""
HullSentinal V2 — Lane Coordinator Node
Enforces multi-robot operational synchronization:
- Cleaner is restricted to lanes strictly less than mapper active lane (< N).
- Cleaner waits until Lane 1 is released upon mapper transition to Lane 2.
- Emits LANE_k_RELEASED, MAPPER_ENTERED_LANE_k, MAPPING_COMPLETE events.
- Immediate EMERGENCY_OVERRIDE authorization when a defect reaches CRITICAL severity (>= 0.85).
"""

import time
import rclpy
from rclpy.node import Node

from hull_msgs.msg import MapperStatus, BeliefUpdate, LaneCoordination


class LaneCoordinatorNode(Node):
    def __init__(self):
        super().__init__('lane_coordinator_node')

        # Coordination state
        self.active_mapper_lane = 1
        self.released_cleaner_lane = 0  # 0 indicates cleaner must hold (WAITING_FOR_LANE)
        self.mapping_complete = False
        self.emergency_active = False
        self.emergency_defects = set()

        # Subscriptions
        self.mapper_sub = self.create_subscription(
            MapperStatus, '/mapper/lane_state', self._mapper_cb, 10)
        self.belief_sub = self.create_subscription(
            BeliefUpdate, '/belief_updates', self._belief_cb, 20)

        # Publisher for coordination events
        self.event_pub = self.create_publisher(
            LaneCoordination, '/lane_coordination/events', 10)

        # Heartbeat timer (2 Hz)
        self.timer = self.create_timer(0.5, self._publish_heartbeat)

        self.get_logger().info(
            '[COORDINATION] LaneCoordinatorNode V2 active. Policy: cleaner authorized for lanes < mapper_lane. '
            'Initial status: WAITING_FOR_LANE (holding until Lane 1 released).'
        )

    def _mapper_cb(self, msg: MapperStatus):
        now = self.get_clock().now()

        if msg.state == 'MAPPING_COMPLETE' and not self.mapping_complete:
            self.mapping_complete = True
            self.active_mapper_lane = 3
            self.released_cleaner_lane = 3
            self.get_logger().info(
                '[COORDINATION] Event: MAPPING_COMPLETE. All lanes (1, 2, 3) fully released for cleaner operations.'
            )
            self._emit_event('MAPPING_COMPLETE')
            return

        # Check if mapper advanced to a new lane
        if msg.current_lane > self.active_mapper_lane:
            old_lane = self.active_mapper_lane
            self.active_mapper_lane = msg.current_lane
            self.get_logger().info(
                f"[COORDINATION] Mapper advanced: Lane {old_lane} -> Lane {self.active_mapper_lane} "
                f"({msg.state})"
            )

            # Release previous lane to cleaner: cleaner cleans lanes < mapper_lane
            new_released = self.active_mapper_lane - 1
            if new_released > self.released_cleaner_lane:
                self.released_cleaner_lane = new_released
                event_name = f'LANE_{self.released_cleaner_lane}_RELEASED'
                self.get_logger().info(
                    f"[COORDINATION] Event: {event_name}. Cleaner authorized to clean Lane {self.released_cleaner_lane}."
                )
                self._emit_event(event_name)

    def _belief_cb(self, msg: BeliefUpdate):
        # Check for CRITICAL defect triggering immediate emergency override
        if msg.emergency_flag or msg.risk_level == 'CRITICAL':
            if msg.defect_id not in self.emergency_defects:
                self.emergency_defects.add(msg.defect_id)
                self.emergency_active = True
                self.get_logger().warning(
                    f"[COORDINATION] Event: EMERGENCY_OVERRIDE for defect {msg.defect_id} (sev={msg.severity:.2f}) "
                    f"in Lane {msg.lane_id}! Bypassing standard lane synchronization."
                )
                self._emit_event(f'EMERGENCY_OVERRIDE_{msg.defect_id}')

        # If an emergency defect was verified clean, remove from emergency set
        if msg.status == 'VERIFIED_CLEAN' and msg.defect_id in self.emergency_defects:
            self.emergency_defects.remove(msg.defect_id)
            if not self.emergency_defects:
                self.emergency_active = False
                self.get_logger().info(
                    f"[COORDINATION] Emergency defect {msg.defect_id} resolved and verified clean. "
                    "Emergency override cleared."
                )
                self._emit_event('EMERGENCY_CLEARED')

    def _emit_event(self, event_name: str):
        msg = LaneCoordination()
        msg.event = event_name
        msg.active_mapper_lane = int(self.active_mapper_lane)
        msg.released_cleaner_lane = int(self.released_cleaner_lane)
        msg.emergency_active = bool(self.emergency_active)
        msg.stamp = self.get_clock().now().to_msg()
        self.event_pub.publish(msg)

    def _publish_heartbeat(self):
        msg = LaneCoordination()
        msg.event = 'HEARTBEAT'
        msg.active_mapper_lane = int(self.active_mapper_lane)
        msg.released_cleaner_lane = int(self.released_cleaner_lane)
        msg.emergency_active = bool(self.emergency_active)
        msg.stamp = self.get_clock().now().to_msg()
        self.event_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = LaneCoordinatorNode()
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
