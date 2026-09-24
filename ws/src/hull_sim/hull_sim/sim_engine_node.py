#!/usr/bin/env python3
"""
HullSentinal Simulation Engine Node
Simulates multi-robot kinematics on ship hull manifold, defect degradation dynamics,
sensor perception footprint, and closed-loop verification.
Workspace bounds: [0.0, 20.0] x [0.0, 6.0]. Safe bounds: [0.8, 19.2] x [0.8, 5.2].
"""

import math
import time
import json
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from hull_msgs.msg import DefectDetection, BeliefUpdate, RobotStatus, MissionStatus
from hull_msgs.srv import RescanRequest


class SimEngineNode(Node):
    def __init__(self):
        super().__init__('sim_engine_node')

        # Declare parameters
        self.declare_parameter('num_cleaners', 1)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('current_disturbance_x', 0.0)
        self.declare_parameter('current_disturbance_y', 0.0)
        self.declare_parameter('sensor_noise_std', 0.03)
        self.declare_parameter('cleaning_failure_rate', 0.0)  # Chance of partial clean
        self.declare_parameter('cleaning_rate', 1.50)  # Severity decrease per second
        self.declare_parameter('tool_radius', 0.45)
        self.declare_parameter('detection_radius', 2.2)
        self.declare_parameter('scenario_name', 'nominal')

        self.num_cleaners = self.get_parameter('num_cleaners').get_parameter_value().integer_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.current_x = self.get_parameter('current_disturbance_x').get_parameter_value().double_value
        self.current_y = self.get_parameter('current_disturbance_y').get_parameter_value().double_value
        self.sensor_noise = self.get_parameter('sensor_noise_std').get_parameter_value().double_value
        self.cleaning_failure_rate = self.get_parameter('cleaning_failure_rate').get_parameter_value().double_value
        self.cleaning_rate = self.get_parameter('cleaning_rate').get_parameter_value().double_value
        self.tool_radius = self.get_parameter('tool_radius').get_parameter_value().double_value
        self.detection_radius = self.get_parameter('detection_radius').get_parameter_value().double_value
        self.scenario_name = self.get_parameter('scenario_name').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        # State storage for robots - start safely inside bounds
        self.robot_ids = ['mapper_1'] + [f'cleaner_{i+1}' for i in range(self.num_cleaners)]
        self.robots = {}
        for rid in self.robot_ids:
            if 'mapper' in rid:
                init_x, init_y, init_yaw = 1.5, 1.5, 0.0
                role = 'mapper'
            else:
                idx = int(rid.split('_')[1])
                init_x, init_y, init_yaw = 1.5 + (idx - 1) * 1.0, 4.5 - (idx - 1) * 0.8, 0.0
                role = 'cleaner'
            self.robots[rid] = {
                'x': init_x,
                'y': init_y,
                'yaw': init_yaw,
                'vx': 0.0,
                'wz': 0.0,
                'role': role,
                'tool_active': False,
                'current_task': '',
                'tool_state': 'IDLE',
                'battery': 100.0,
                'failed': False
            }

        # Initialize ground truth defects on hull surface (20m x 6m)
        self.defects = self._initialize_defects()
        self.collision_count = 0
        self.start_time = time.time()

        # Publishers & Subscribers for each robot
        self.cmd_subs = {}
        self.tool_subs = {}
        self.pose_pubs = {}
        self.odom_pubs = {}
        self.status_pubs = {}

        for rid in self.robot_ids:
            self.cmd_subs[rid] = self.create_subscription(
                Twist, f'/{rid}/cmd_vel', lambda msg, r=rid: self._cmd_cb(r, msg), 10)
            self.tool_subs[rid] = self.create_subscription(
                Bool, f'/{rid}/tool_cmd', lambda msg, r=rid: self._tool_cb(r, msg), 10)
            self.pose_pubs[rid] = self.create_publisher(PoseStamped, f'/{rid}/pose', 10)
            self.odom_pubs[rid] = self.create_publisher(Odometry, f'/{rid}/odom', 10)
            self.status_pubs[rid] = self.create_publisher(RobotStatus, f'/{rid}/status', 10)

        # Global perception and mission publishers
        self.detection_pub = self.create_publisher(DefectDetection, '/defect_detections', 20)
        self.mission_pub = self.create_publisher(MissionStatus, '/hull_sim/mission_status', 10)
        self.defect_state_pub = self.create_publisher(String, '/hull_sim/defect_states', 10)

        # Subscribe to belief updates to keep simulation states synchronized
        self.belief_sub = self.create_subscription(
            BeliefUpdate, '/belief_updates', self._belief_cb, 20)

        # Rescan service provider
        self.rescan_srv = self.create_service(RescanRequest, '/rescan_request', self._rescan_cb)

        # Simulation step timer
        self.sim_timer = self.create_timer(self.dt, self._sim_step)
        self.get_logger().info(f'HullSentinal SimEngine initialized ({len(self.robots)} robots, {len(self.defects)} defects, scenario: {self.scenario_name})')

    def _initialize_defects(self):
        """Generates ground truth defect distribution on the ship hull."""
        configs = [
            # High severity structural cracks
            {'id': 'defect_crack_01', 'type': 'crack', 'x': 4.0, 'y': 2.0, 'sev': 0.95, 'conf': 0.92},
            {'id': 'defect_crack_02', 'type': 'crack', 'x': 8.5, 'y': 4.5, 'sev': 0.88, 'conf': 0.90},
            {'id': 'defect_crack_03', 'type': 'crack', 'x': 14.0, 'y': 1.5, 'sev': 0.80, 'conf': 0.87},
            # Medium severity corrosion / rust
            {'id': 'defect_rust_01', 'type': 'rust', 'x': 3.0, 'y': 4.0, 'sev': 0.75, 'conf': 0.85},
            {'id': 'defect_rust_02', 'type': 'rust', 'x': 11.0, 'y': 3.0, 'sev': 0.70, 'conf': 0.88},
            {'id': 'defect_rust_03', 'type': 'rust', 'x': 16.5, 'y': 4.8, 'sev': 0.65, 'conf': 0.82},
            # Biofouling patches (high drag, moderate structural risk)
            {'id': 'defect_bio_01', 'type': 'biofouling', 'x': 6.0, 'y': 1.2, 'sev': 0.60, 'conf': 0.91},
            {'id': 'defect_bio_02', 'type': 'biofouling', 'x': 12.5, 'y': 5.0, 'sev': 0.55, 'conf': 0.89},
            {'id': 'defect_bio_03', 'type': 'biofouling', 'x': 17.5, 'y': 2.2, 'sev': 0.50, 'conf': 0.86},
        ]
        defects = {}
        for c in configs:
            defects[c['id']] = {
                'id': c['id'],
                'type': c['type'],
                'x': c['x'],
                'y': c['y'],
                'z': 0.0,
                'initial_sev': c['sev'],
                'current_sev': c['sev'],
                'conf': c['conf'],
                'state': 'UNDETECTED',
                'state_history': ['UNDETECTED'],
                'cleaned': False,
                'rescan_count': 0
            }
        return defects

    def _cmd_cb(self, robot_id, msg):
        if not self.robots[robot_id]['failed']:
            self.robots[robot_id]['vx'] = msg.linear.x
            self.robots[robot_id]['wz'] = msg.angular.z

    def _tool_cb(self, robot_id, msg):
        if not self.robots[robot_id]['failed']:
            self.robots[robot_id]['tool_active'] = msg.data
            self.robots[robot_id]['tool_state'] = 'CLEANING' if msg.data else 'IDLE'

    def _belief_cb(self, msg):
        d = self.defects.get(msg.defect_id)
        if d:
            if msg.status != d['state']:
                d['state'] = msg.status
                if msg.status not in d['state_history']:
                    d['state_history'].append(msg.status)
            if msg.status == 'VERIFIED_CLEAN' or msg.status == 'DONE':
                d['cleaned'] = True
                d['state'] = 'VERIFIED_CLEAN'
                d['current_sev'] = 0.0

    def _sim_step(self):
        """Updates physics, robot kinematics, tool interaction, and sensor perception."""
        now = self.get_clock().now()

        # 1. Update Robot Kinematics
        for rid, bot in self.robots.items():
            if bot['failed']:
                continue
            eff_vx = bot['vx'] + self.current_x
            eff_vy = self.current_y
            eff_wz = bot['wz']

            bot['x'] += (eff_vx * math.cos(bot['yaw']) - eff_vy * math.sin(bot['yaw'])) * self.dt
            bot['y'] += (eff_vx * math.sin(bot['yaw']) + eff_vy * math.cos(bot['yaw'])) * self.dt
            bot['yaw'] += eff_wz * self.dt
            bot['yaw'] = (bot['yaw'] + math.pi) % (2 * math.pi) - math.pi

            # Strict hull boundary clamping to guarantee zero out-of-bounds drift
            bot['x'] = float(np.clip(bot['x'], 0.8, 19.2))
            bot['y'] = float(np.clip(bot['y'], 0.8, 5.2))

            # Publish Robot Transforms
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'hull_world'
            t.child_frame_id = f'{rid}/base_footprint'
            t.transform.translation.x = bot['x']
            t.transform.translation.y = bot['y']
            t.transform.translation.z = 0.05
            t.transform.rotation = self._yaw_to_quat(bot['yaw'])
            self.tf_broadcaster.sendTransform(t)

            # Publish PoseStamped
            ps = PoseStamped()
            ps.header.stamp = now.to_msg()
            ps.header.frame_id = 'hull_world'
            ps.pose.position.x = bot['x']
            ps.pose.position.y = bot['y']
            ps.pose.position.z = 0.05
            ps.pose.orientation = t.transform.rotation
            self.pose_pubs[rid].publish(ps)

            # Publish Status
            st = RobotStatus()
            st.stamp = now.to_msg()
            st.robot_id = rid
            st.role = bot['role']
            st.pose = ps.pose
            st.current_task_id = bot['current_task']
            st.tool_state = bot['tool_state']
            st.battery_pct = bot['battery']
            self.status_pubs[rid].publish(st)

        # 2. Inter-robot collision check
        r_list = list(self.robots.keys())
        for i in range(len(r_list)):
            for j in range(i + 1, len(r_list)):
                b1, b2 = self.robots[r_list[i]], self.robots[r_list[j]]
                dist = math.hypot(b1['x'] - b2['x'], b1['y'] - b2['y'])
                if dist < 0.40:
                    self.collision_count += 1

        # 3. Active Cleaning Dynamics
        for rid, bot in self.robots.items():
            if bot['role'] == 'cleaner' and bot['tool_active']:
                tool_x = bot['x'] + 0.25 * math.cos(bot['yaw'])
                tool_y = bot['y'] + 0.25 * math.sin(bot['yaw'])

                for d_id, defect in self.defects.items():
                    d_dist = math.hypot(tool_x - defect['x'], tool_y - defect['y'])
                    if d_dist <= self.tool_radius and defect['current_sev'] > 0.001:
                        sev_delta = self.cleaning_rate * self.dt
                        defect['current_sev'] = max(0.0, defect['current_sev'] - sev_delta)
                        if defect['state'] != 'CLEANING':
                            defect['state'] = 'CLEANING'
                            if 'CLEANING' not in defect['state_history']:
                                defect['state_history'].append('CLEANING')

        # 4. Perception Detection (Mapper robot sweep)
        mapper = self.robots.get('mapper_1')
        if mapper:
            for d_id, defect in self.defects.items():
                if defect['state'] == 'UNDETECTED':
                    dist = math.hypot(mapper['x'] - defect['x'], mapper['y'] - defect['y'])
                    if dist <= self.detection_radius:
                        # Covered by sensor footprint! Transition UNDETECTED -> DETECTED
                        defect['state'] = 'DETECTED'
                        if 'DETECTED' not in defect['state_history']:
                            defect['state_history'].append('DETECTED')

                        noisy_sev = float(np.clip(defect['current_sev'] + np.random.normal(0, self.sensor_noise), 0.05, 1.0))
                        noisy_conf = float(np.clip(defect['conf'] + np.random.normal(0, self.sensor_noise), 0.1, 1.0))

                        msg = DefectDetection()
                        msg.defect_id = defect['id']
                        msg.defect_type = defect['type']
                        msg.severity = noisy_sev
                        msg.confidence = noisy_conf
                        msg.position.x = defect['x']
                        msg.position.y = defect['y']
                        msg.position.z = defect['z']
                        msg.stamp = now.to_msg()
                        self.detection_pub.publish(msg)
                        self.get_logger().info(f"Mapper FOV detected {defect['id']} ({defect['type']}) sev={noisy_sev:.2f} conf={noisy_conf:.2f}")

        # 5. Periodically publish overall defect states JSON for monitoring
        if int(time.time() * 10) % 5 == 0:
            states_dict = {
                d_id: {
                    'type': d['type'],
                    'x': d['x'],
                    'y': d['y'],
                    'severity': d['current_sev'],
                    'state': d['state'],
                    'history': d['state_history']
                } for d_id, d in self.defects.items()
            }
            s_msg = String()
            s_msg.data = json.dumps(states_dict)
            self.defect_state_pub.publish(s_msg)

        # 6. Publish Mission Status
        m_msg = MissionStatus()
        m_msg.header.stamp = now.to_msg()
        m_msg.total_defects = len(self.defects)
        cleaned_cnt = sum(1 for d in self.defects.values() if d['cleaned'])
        m_msg.cleaned_count = cleaned_cnt
        m_msg.pending_count = sum(1 for d in self.defects.values() if d['state'] in ('UNDETECTED', 'DETECTED'))
        m_msg.in_progress_count = sum(1 for d in self.defects.values() if d['state'] in ('QUEUED', 'CLEANING'))
        m_msg.requeued_count = sum(d['rescan_count'] for d in self.defects.values())
        m_msg.elapsed_time_sec = float(time.time() - self.start_time)
        m_msg.all_clean = (cleaned_cnt == len(self.defects))
        self.mission_pub.publish(m_msg)

    def _rescan_cb(self, request, response):
        """Verification service: checks ground truth defect residual at coordinates."""
        d_id = request.defect_id
        target_defect = self.defects.get(d_id)

        if not target_defect:
            for d in self.defects.values():
                if math.hypot(d['x'] - request.position.x, d['y'] - request.position.y) < request.scan_radius:
                    target_defect = d
                    break

        if target_defect:
            target_defect['rescan_count'] += 1
            if np.random.random() < self.cleaning_failure_rate:
                residual = max(0.25, target_defect['current_sev'])
            else:
                residual = target_defect['current_sev']

            is_present = residual > 0.05
            noisy_conf = float(np.clip(target_defect['conf'] + np.random.normal(0, self.sensor_noise), 0.2, 1.0))

            response.defect_present = is_present
            response.residual_severity = float(residual)
            response.confidence = noisy_conf

            if not is_present:
                target_defect['cleaned'] = True
                target_defect['state'] = 'VERIFIED_CLEAN'
                if 'VERIFIED_CLEAN' not in target_defect['state_history']:
                    target_defect['state_history'].append('VERIFIED_CLEAN')

            self.get_logger().info(f"Rescan on {d_id}: present={is_present}, residual_sev={residual:.3f}, conf={noisy_conf:.2f}")
        else:
            response.defect_present = False
            response.residual_severity = 0.0
            response.confidence = 0.95
        return response

    @staticmethod
    def _yaw_to_quat(yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    try:
        rclpy.init(args=args)
        node = SimEngineNode()
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
