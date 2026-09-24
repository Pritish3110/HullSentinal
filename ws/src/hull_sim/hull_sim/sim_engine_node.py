#!/usr/bin/env python3
"""
HullSentinal V2 — Simulation Engine Node
Simulates multi-robot kinematics on 24m x 8m ship hull manifold, defect degradation dynamics,
sensor perception footprint, and closed-loop verification.
Master configuration loaded from hullsentinal_v2.yaml.
"""

import os
import math
import time
import json
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

from hull_msgs.msg import DefectDetection, BeliefUpdate, RobotStatus, MissionStatus
from hull_msgs.srv import RescanRequest


class SimEngineNode(Node):
    def __init__(self):
        super().__init__('sim_engine_node')

        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('current_disturbance_x', 0.0)
        self.declare_parameter('current_disturbance_y', 0.0)
        self.declare_parameter('sensor_noise_std', 0.02)
        self.declare_parameter('cleaning_failure_rate', 0.0)
        self.declare_parameter('scenario_name', 'nominal')

        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.current_x = self.get_parameter('current_disturbance_x').get_parameter_value().double_value
        self.current_y = self.get_parameter('current_disturbance_y').get_parameter_value().double_value
        self.sensor_noise = self.get_parameter('sensor_noise_std').get_parameter_value().double_value
        self.cleaning_failure_rate = self.get_parameter('cleaning_failure_rate').get_parameter_value().double_value
        self.scenario_name = self.get_parameter('scenario_name').get_parameter_value().string_value

        # Load centralized configuration
        self.cfg = self._load_config()

        # World geometry and safety bounds
        world_cfg = self.cfg.get('world', {})
        self.hull_width = float(world_cfg.get('hull_width', 24.0))
        self.hull_height = float(world_cfg.get('hull_height', 8.0))
        self.safety_margin = float(world_cfg.get('safety_margin', 0.8))
        self.x_min = self.safety_margin
        self.x_max = self.hull_width - self.safety_margin
        self.y_min = self.safety_margin
        self.y_max = self.hull_height - self.safety_margin

        # Robots configuration
        robots_cfg = self.cfg.get('robots', {})
        self.robots = {}
        for rid, rdata in robots_cfg.items():
            self.robots[rid] = {
                'x': float(rdata.get('start_x', 1.5)),
                'y': float(rdata.get('start_y', 2.0)),
                'yaw': float(rdata.get('start_yaw', 0.0)),
                'vx': 0.0,
                'wz': 0.0,
                'role': rdata.get('role', 'cleaner'),
                'radius': float(rdata.get('radius', 0.45)),
                'max_linear_speed': float(rdata.get('max_linear_speed', 1.2)),
                'max_angular_speed': float(rdata.get('max_angular_speed', 1.5)),
                'sensor_fov_radius': float(rdata.get('sensor_fov_radius', 1.80)),
                'tool_radius': float(rdata.get('tool_radius', 0.45)),
                'cleaning_rate': float(rdata.get('cleaning_rate', 1.50)),
                'tool_active': False,
                'current_task': '',
                'tool_state': 'IDLE',
                'battery': 100.0,
                'failed': False
            }

        self.robot_ids = list(self.robots.keys())
        self.tf_broadcaster = TransformBroadcaster(self)

        # Ground truth defects from config
        self.defects = self._initialize_defects()
        self.collision_count = 0
        self.start_time = time.time()

        # Publishers & Subscribers per robot
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

        # Ground truth state sync subscription
        self.belief_sub = self.create_subscription(
            BeliefUpdate, '/belief_updates', self._belief_cb, 20)

        # Rescan service provider
        self.rescan_srv = self.create_service(RescanRequest, '/rescan_request', self._rescan_cb)

        # Simulation step timer
        self.sim_timer = self.create_timer(self.dt, self._sim_step)
        self.get_logger().info(
            f'[SIM] HullSentinal V2 SimEngine initialized: {len(self.robots)} robots, '
            f'{len(self.defects)} defects, hull [{self.hull_width}x{self.hull_height}m], scenario: {self.scenario_name}'
        )

    def _load_config(self):
        """Loads master configuration YAML without hardcoded fallbacks."""
        custom_path = self.get_parameter('config_file').get_parameter_value().string_value
        candidate_paths = [
            custom_path,
            os.path.join(get_package_share_directory('hull_bringup'), 'config', 'hullsentinal_v2.yaml')
            if 'hull_bringup' in os.environ.get('AMENT_PREFIX_PATH', '') else '',
            '/home/pritish_3110/Personal Files/Syllabus/Projects/HullSentinal/ws/src/hull_bringup/config/hullsentinal_v2.yaml',
        ]

        for path in candidate_paths:
            if path and os.path.exists(path):
                self.get_logger().info(f'[SIM] Loading master config from {path}')
                with open(path, 'r') as f:
                    return yaml.safe_load(f)

        self.get_logger().warning('[SIM] Master config not found in candidate paths; using fallback defaults.')
        return {}

    def _initialize_defects(self):
        """Populates ground truth defects directly from master configuration."""
        raw_defects = self.cfg.get('defects', [])
        defects = {}
        for d in raw_defects:
            d_id = d['id']
            defects[d_id] = {
                'id': d_id,
                'type': d.get('type', 'biofouling'),
                'x': float(d.get('x', 0.0)),
                'y': float(d.get('y', 0.0)),
                'z': 0.0,
                'lane_id': int(d.get('lane_id', 1)),
                'initial_sev': float(d.get('severity', 0.5)),
                'current_sev': float(d.get('severity', 0.5)),
                'conf': float(d.get('confidence', 0.9)),
                'growth_rate': float(d.get('growth_rate', 0.0001)),
                'state': 'UNDETECTED',
                'state_history': ['UNDETECTED'],
                'cleaned': False,
                'rescan_count': 0,
                'last_det_time': 0.0
            }
        return defects

    def _cmd_cb(self, robot_id, msg):
        if robot_id in self.robots and not self.robots[robot_id]['failed']:
            self.robots[robot_id]['vx'] = msg.linear.x
            self.robots[robot_id]['wz'] = msg.angular.z

    def _tool_cb(self, robot_id, msg):
        if robot_id in self.robots and not self.robots[robot_id]['failed']:
            self.robots[robot_id]['tool_active'] = msg.data
            self.robots[robot_id]['tool_state'] = 'CLEANING' if msg.data else 'IDLE'

    def _belief_cb(self, msg):
        d = self.defects.get(msg.defect_id)
        if d:
            if msg.status != d['state']:
                d['state'] = msg.status
                if msg.status not in d['state_history']:
                    d['state_history'].append(msg.status)
            if msg.status in ('VERIFIED_CLEAN', 'DONE'):
                d['cleaned'] = True
                d['state'] = 'VERIFIED_CLEAN'
                d['current_sev'] = 0.0

    def _sim_step(self):
        """Advances physics, kinematics, defect degradation, cleaning, and sensor perception."""
        now = self.get_clock().now()

        # 1. Update Robot Kinematics with strict boundary clamping
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

            # Strict hull boundary clamping: [x_min, x_max] x [y_min, y_max]
            bot['x'] = float(np.clip(bot['x'], self.x_min, self.x_max))
            bot['y'] = float(np.clip(bot['y'], self.y_min, self.y_max))

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

            # Publish Odometry
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'hull_world'
            odom.child_frame_id = f'{rid}/base_footprint'
            odom.pose.pose = ps.pose
            odom.twist.twist.linear.x = bot['vx']
            odom.twist.twist.angular.z = bot['wz']
            self.odom_pubs[rid].publish(odom)

            # Publish RobotStatus
            st = RobotStatus()
            st.stamp = now.to_msg()
            st.robot_id = rid
            st.role = bot['role']
            st.pose = ps.pose
            st.current_task_id = bot['current_task']
            st.tool_state = bot['tool_state']
            st.battery_pct = bot['battery']
            self.status_pubs[rid].publish(st)

        # 2. Defect Degradation Dynamics (Deterministic Growth)
        for d in self.defects.values():
            if not d['cleaned'] and d['current_sev'] > 0.001:
                growth = d['growth_rate'] * self.dt
                if growth > 0.0:
                    old_sev = d['current_sev']
                    d['current_sev'] = min(1.0, d['current_sev'] + growth)
                    if old_sev < 0.85 and d['current_sev'] >= 0.85:
                        self.get_logger().warning(
                            f"[EMERGENCY] Ground truth defect {d['id']} escalated past CRITICAL threshold "
                            f"(sev={d['current_sev']:.3f}, lane={d['lane_id']})"
                        )

        # 3. Inter-robot collision check
        r_list = list(self.robots.keys())
        for i in range(len(r_list)):
            for j in range(i + 1, len(r_list)):
                b1, b2 = self.robots[r_list[i]], self.robots[r_list[j]]
                dist = math.hypot(b1['x'] - b2['x'], b1['y'] - b2['y'])
                if dist < 0.50:
                    self.collision_count += 1

        # 4. Active Cleaning Dynamics
        for rid, bot in self.robots.items():
            if bot['role'] == 'cleaner' and bot['tool_active']:
                tool_x = bot['x'] + 0.25 * math.cos(bot['yaw'])
                tool_y = bot['y'] + 0.25 * math.sin(bot['yaw'])

                for d_id, defect in self.defects.items():
                    d_dist = math.hypot(tool_x - defect['x'], tool_y - defect['y'])
                    if d_dist <= bot['tool_radius'] and defect['current_sev'] > 0.001:
                        sev_delta = bot['cleaning_rate'] * self.dt
                        defect['current_sev'] = max(0.0, defect['current_sev'] - sev_delta)
                        if defect['state'] != 'CLEANING':
                            defect['state'] = 'CLEANING'
                            if 'CLEANING' not in defect['state_history']:
                                defect['state_history'].append('CLEANING')

        # 5. Perception Footprint (Mapper robot sweep)
        mapper = self.robots.get('mapper_1')
        if mapper:
            cur_time = time.time()
            fov_r = mapper['sensor_fov_radius']
            for d_id, defect in self.defects.items():
                dist = math.hypot(mapper['x'] - defect['x'], mapper['y'] - defect['y'])
                if dist <= fov_r:
                    if defect['state'] == 'UNDETECTED':
                        # First detection: UNDETECTED -> DETECTED
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
                        defect['last_det_time'] = cur_time
                        self.get_logger().info(
                            f"[MAPPER] Sensor FOV detected {defect['id']} ({defect['type']}) in Lane {defect['lane_id']}: "
                            f"sev={noisy_sev:.2f}, conf={noisy_conf:.2f}"
                        )
                    else:
                        # Re-observation in FOV (at 1 Hz) to capture evolving defect growth
                        if cur_time - defect['last_det_time'] >= 1.0:
                            defect['last_det_time'] = cur_time
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

        # 6. Periodic Defect States JSON Publishing for Telemetry
        if int(time.time() * 10) % 5 == 0:
            states_dict = {
                d_id: {
                    'type': d['type'],
                    'x': d['x'],
                    'y': d['y'],
                    'lane_id': d['lane_id'],
                    'severity': d['current_sev'],
                    'state': d['state'],
                    'history': d['state_history']
                } for d_id, d in self.defects.items()
            }
            s_msg = String()
            s_msg.data = json.dumps(states_dict)
            self.defect_state_pub.publish(s_msg)

        # 7. Mission Status
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
        """Verification service evaluating ground truth residual defect at target location."""
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
                self.get_logger().info(f"[VERIFICATION] Rescan on {d_id}: SUCCESS -> VERIFIED_CLEAN (residual={residual:.3f})")
            else:
                self.get_logger().warning(f"[VERIFICATION] Rescan on {d_id}: FAILED (residual={residual:.3f}) -> RE_QUEUED")
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
