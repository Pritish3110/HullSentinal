#!/usr/bin/env python3
"""
HullSentinal V2 — Single-Owner RViz2 Visualization & Marker Publisher
Strict sole owner of /defects markers in RViz.
- Stable (namespace, id) indexing per defect: updates in-place without duplicate ghost labels.
- Clean delete-all on startup.
- Geometry by type: crack=cube, rust=cylinder, biofouling=sphere.
- Color by state: UNDETECTED=grey, DETECTED=yellow, QUEUED=orange, CLEANING=pulsing red, VERIFIED_CLEAN=green.
- 24m x 8m belief heatmap.
- Centered, non-overlapping HUD text panel displaying full multi-robot cognitive status.
"""

import os
import math
import time
import json
import yaml
from collections import deque
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory

from hull_msgs.msg import DefectDetection, BeliefUpdate, TaskQueue, MapperStatus, LaneCoordination, MissionStatus


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')

        self.declare_parameter('config_file', '')

        # Load master config
        self.cfg = self._load_config()
        world_cfg = self.cfg.get('world', {})
        self.hull_width = float(world_cfg.get('hull_width', 24.0))
        self.hull_height = float(world_cfg.get('hull_height', 8.0))

        # Initialize all 14 defects from master config with stable integer IDs
        raw_defects = self.cfg.get('defects', [])
        self.defects = {}
        self.defect_id_to_idx = {}
        for idx, d in enumerate(raw_defects):
            d_id = d['id']
            self.defect_id_to_idx[d_id] = idx
            self.defects[d_id] = {
                'idx': idx,
                'type': d.get('type', 'biofouling'),
                'x': float(d.get('x', 0.0)),
                'y': float(d.get('y', 0.0)),
                'z': 0.05,
                'lane_id': int(d.get('lane_id', 1)),
                'sev': float(d.get('severity', 0.5)),
                'risk': 'NORMAL',
                'emergency': False,
                'state': 'UNDETECTED'
            }

        # Robot states
        self.robot_poses = {'mapper_1': (1.5, 2.0, 0.05), 'cleaner_1': (1.5, 7.0, 0.05)}
        self.robot_paths = {'mapper_1': deque(maxlen=600), 'cleaner_1': deque(maxlen=600)}
        self.robot_goals = {'cleaner_1': None}
        self.cleaner_tool_active = False

        # Coordination and mission tracking
        self.mapper_state = 'MAPPING_LANE_1'
        self.mapper_lane = 1
        self.mapper_lane_cov = 0.0
        self.mapper_total_cov = 0.0
        self.released_cleaner_lane = 0
        self.emergency_active = False
        self.out_of_bounds_count = 0
        self.all_in_bounds = True
        self.mission_cleaned_cnt = 0
        self.ranked_tasks = []
        self.has_deleted_all = False

        # Subscriptions
        self.create_subscription(PoseStamped, '/mapper_1/pose', lambda m: self._pose_cb('mapper_1', m), 10)
        self.create_subscription(PoseStamped, '/cleaner_1/pose', lambda m: self._pose_cb('cleaner_1', m), 10)
        self.create_subscription(PoseStamped, '/cleaner_1/target_goal', self._goal_cb, 10)
        self.create_subscription(Bool, '/cleaner_1/tool_cmd', self._tool_cb, 10)

        self.create_subscription(DefectDetection, '/defect_detections', self._detect_cb, 20)
        self.create_subscription(BeliefUpdate, '/belief_updates', self._belief_cb, 20)
        self.create_subscription(TaskQueue, '/task_queue', self._queue_cb, 10)
        self.create_subscription(MapperStatus, '/mapper/lane_state', self._mapper_status_cb, 10)
        self.create_subscription(LaneCoordination, '/lane_coordination/events', self._lane_coord_cb, 10)
        self.create_subscription(MissionStatus, '/hull_sim/mission_status', self._mission_cb, 10)
        self.create_subscription(String, '/geofence_status', self._geofence_cb, 10)

        # Publishers
        self.defects_pub = self.create_publisher(MarkerArray, '/defects', 10)
        self.heatmap_pub = self.create_publisher(MarkerArray, '/belief_map/heatmap', 10)
        self.queue_marker_pub = self.create_publisher(Marker, '/task_queue_marker', 10)
        self.fov_pub = self.create_publisher(Marker, '/mapper_1/sensor_fov', 10)
        self.mapper_path_pub = self.create_publisher(Marker, '/mapper_1/path_marker', 10)
        self.cleaner_path_pub = self.create_publisher(Marker, '/cleaner_1/path_marker', 10)
        self.cleaner_goal_pub = self.create_publisher(Marker, '/cleaner_1/goal_marker', 10)
        self.composite_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self._publish_all)
        self.get_logger().info(
            f'[VISUALIZATION] MarkerPublisher V2 active as single-owner of /defects '
            f'({len(self.defects)} defects with stable IDs, 24x8m heatmap, ASCII HUD)'
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

    def _pose_cb(self, rid, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_poses[rid] = (x, y, 0.05)
        path = self.robot_paths[rid]
        if not path or math.hypot(x - path[-1][0], y - path[-1][1]) > 0.08:
            path.append((x, y, 0.03))

    def _goal_cb(self, msg: PoseStamped):
        self.robot_goals['cleaner_1'] = (msg.pose.position.x, msg.pose.position.y)

    def _tool_cb(self, msg: Bool):
        self.cleaner_tool_active = msg.data

    def _detect_cb(self, msg: DefectDetection):
        d_id = msg.defect_id
        if d_id in self.defects:
            if self.defects[d_id]['state'] == 'UNDETECTED':
                self.defects[d_id]['state'] = 'DETECTED'
            self.defects[d_id]['sev'] = float(msg.severity)

    def _belief_cb(self, msg: BeliefUpdate):
        d_id = msg.defect_id
        if d_id in self.defects:
            self.defects[d_id]['state'] = msg.status
            self.defects[d_id]['sev'] = float(msg.severity)
            if msg.risk_level:
                self.defects[d_id]['risk'] = msg.risk_level
            self.defects[d_id]['emergency'] = bool(msg.emergency_flag)
            if msg.status in ('VERIFIED_CLEAN', 'DONE'):
                self.defects[d_id]['state'] = 'VERIFIED_CLEAN'
                self.defects[d_id]['sev'] = 0.0

    def _queue_cb(self, msg: TaskQueue):
        self.ranked_tasks = msg.tasks
        for task in msg.tasks:
            d_id = task.defect_id
            if d_id in self.defects:
                curr = self.defects[d_id]['state']
                if curr not in ('CLEANING', 'VERIFIED_CLEAN', 'DONE'):
                    self.defects[d_id]['state'] = 'QUEUED'

    def _mapper_status_cb(self, msg: MapperStatus):
        self.mapper_state = msg.state
        self.mapper_lane = msg.current_lane
        self.mapper_lane_cov = msg.lane_coverage_pct
        self.mapper_total_cov = msg.total_coverage_pct

    def _lane_coord_cb(self, msg: LaneCoordination):
        self.released_cleaner_lane = msg.released_cleaner_lane
        self.emergency_active = msg.emergency_active

    def _mission_cb(self, msg: MissionStatus):
        self.mission_cleaned_cnt = msg.cleaned_count

    def _geofence_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.out_of_bounds_count = data.get('out_of_bounds_count', 0)
            self.all_in_bounds = data.get('all_in_bounds', True)
        except Exception:
            pass

    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        t_sec = time.time()

        # Initial clean slate delete-all
        if not self.has_deleted_all:
            del_arr = MarkerArray()
            del_m = Marker()
            del_m.action = Marker.DELETEALL
            del_arr.markers.append(del_m)
            self.defects_pub.publish(del_arr)
            self.has_deleted_all = True
            return

        # 1. Sole-Owner Defect Markers Array with Stable IDs
        defects_array = MarkerArray()

        for d_id, d in self.defects.items():
            idx = d['idx']
            state = d['state']
            dtype = d['type']
            sev = d['sev']
            x, y, z = d['x'], d['y'], d['z']

            # Geometry Marker (Stable ID = idx)
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'hull_world'
            m.ns = 'defect_geometry'
            m.id = idx
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0

            # Shape by type
            if dtype == 'crack':
                m.type = Marker.CUBE
                base_scale = 0.22 + 0.35 * sev
                m.scale.x = 0.15
                m.scale.y = base_scale * 1.6
                m.scale.z = 0.06
            elif dtype == 'rust':
                m.type = Marker.CYLINDER
                base_scale = 0.22 + 0.32 * sev
                m.scale.x = base_scale * 1.8
                m.scale.y = base_scale * 1.8
                m.scale.z = 0.05
            else:  # biofouling
                m.type = Marker.SPHERE
                base_scale = 0.24 + 0.34 * sev
                m.scale.x = base_scale * 1.8
                m.scale.y = base_scale * 1.8
                m.scale.z = base_scale * 0.9

            # Color by state:
            if state == 'UNDETECTED':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.50, 0.52, 0.58, 0.35
                m.scale.x *= 0.6
                m.scale.y *= 0.6
            elif state == 'DETECTED':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.90, 0.10, 0.95
            elif state == 'QUEUED':
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.55, 0.05, 0.95
            elif state == 'CLEANING':
                pulse = 1.0 + 0.18 * math.sin(t_sec * 8.0)
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.12, 0.12, 1.0
                m.scale.x *= pulse
                m.scale.y *= pulse
            elif state in ('VERIFIED_CLEAN', 'DONE'):
                m.color.r, m.color.g, m.color.b, m.color.a = 0.08, 0.95, 0.25, 0.95
                m.scale.x *= 0.85
                m.scale.y *= 0.85
            else:  # RE_QUEUED
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.40, 0.0, 1.0

            defects_array.markers.append(m)

            # Progress bar for CLEANING state (Stable ID = 200 + idx)
            p_bar = Marker()
            p_bar.header.stamp = now
            p_bar.header.frame_id = 'hull_world'
            p_bar.ns = 'defect_progress'
            p_bar.id = 200 + idx
            if state == 'CLEANING':
                p_bar.type = Marker.CUBE
                p_bar.action = Marker.ADD
                p_bar.pose.position.x = x
                p_bar.pose.position.y = y + 0.35
                p_bar.pose.position.z = z + 0.08
                p_bar.pose.orientation.w = 1.0
                p_bar.scale.x = 0.50 * max(0.1, sev)
                p_bar.scale.y = 0.08
                p_bar.scale.z = 0.02
                p_bar.color.r, p_bar.color.g, p_bar.color.b, p_bar.color.a = 0.0, 1.0, 0.3, 0.9
            else:
                p_bar.action = Marker.DELETE
            defects_array.markers.append(p_bar)

            # Text Label Marker (Stable ID = 100 + idx)
            txt = Marker()
            txt.header.stamp = now
            txt.header.frame_id = 'hull_world'
            txt.ns = 'defect_label'
            txt.id = 100 + idx
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = x
            txt.pose.position.y = y + 0.45
            txt.pose.position.z = 0.22
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.22

            short_id = d_id.replace('defect_', '').upper()
            if state == 'UNDETECTED':
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.6, 0.6, 0.6, 0.40
                txt.text = f"[{short_id}]"
            elif state in ('VERIFIED_CLEAN', 'DONE'):
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.2, 1.0, 0.4, 0.95
                txt.text = f"[{short_id}] CLEAN"
            elif state == 'CLEANING':
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 0.2, 0.2, 1.0
                txt.text = f"[{short_id}] CLEANING ({sev:.2f})"
            elif d.get('emergency', False) or d.get('risk') == 'CRITICAL':
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 0.1, 0.1, 1.0
                txt.text = f"[{short_id}] CRITICAL ({sev:.2f})"
            else:
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 0.95, 0.9, 0.95
                txt.text = f"[{short_id}] {dtype.upper()} | {state} ({sev:.2f})"

            defects_array.markers.append(txt)

        self.defects_pub.publish(defects_array)

        # 2. Publish 24m x 8m Belief Heatmap
        heatmap_array = MarkerArray()
        h_id = 0
        for gx in range(0, int(self.hull_width), 2):
            for gy in range(0, int(self.hull_height), 1):
                hm = Marker()
                hm.header.stamp = now
                hm.header.frame_id = 'hull_world'
                hm.ns = 'belief_heatmap'
                hm.id = h_id
                h_id += 1
                hm.type = Marker.CUBE
                hm.action = Marker.ADD
                hm.pose.position.x = gx + 1.0
                hm.pose.position.y = gy + 0.5
                hm.pose.position.z = 0.005
                hm.pose.orientation.w = 1.0
                hm.scale.x = 1.90
                hm.scale.y = 0.90
                hm.scale.z = 0.01

                cell_sev = 0.0
                for d in self.defects.values():
                    if d['state'] != 'UNDETECTED':
                        dist = math.hypot(d['x'] - (gx + 1.0), d['y'] - (gy + 0.5))
                        if dist < 2.0:
                            cell_sev += d['sev'] * math.exp(-dist)

                intensity = min(1.0, cell_sev * 0.8)
                hm.color.r = float(intensity)
                hm.color.g = float(0.1 + 0.3 * (1.0 - intensity))
                hm.color.b = float(0.5 * (1.0 - intensity))
                hm.color.a = float(0.08 + 0.25 * intensity)
                heatmap_array.markers.append(hm)

        self.heatmap_pub.publish(heatmap_array)

        # 3. Publish HUD Text Marker (/task_queue_marker)
        hud = Marker()
        hud.header.stamp = now
        hud.header.frame_id = 'hull_world'
        hud.ns = 'hud_display'
        hud.id = 9999
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD
        hud.pose.position.x = 1.2
        hud.pose.position.y = 7.4
        hud.pose.position.z = 0.50
        hud.pose.orientation.w = 1.0
        hud.scale.z = 0.20
        hud.color.r, hud.color.g, hud.color.b, hud.color.a = 0.92, 0.95, 1.0, 0.95

        active_cnt = sum(1 for d in self.defects.values() if d['state'] in ('DETECTED', 'QUEUED', 'CLEANING'))
        cur_target = self.robot_goals.get('cleaner_1')
        target_str = f"({cur_target[0]:.1f}, {cur_target[1]:.1f})" if cur_target else "HOLDING / IDLE"

        hud_lines = [
            "==================================================",
            "   HULLSENTINAL V2: COGNITIVE MULTI-ROBOT HUD",
            "==================================================",
            f"MAPPER: {self.mapper_state} | LANE {self.mapper_lane} ({self.mapper_lane_cov:.0f}%) | COV: {self.mapper_total_cov:.1f}%",
            f"CLEANER: {'CLEANING' if self.cleaner_tool_active else 'NAV'} | TARGET: {target_str}",
            f"COORDINATION: RELEASED_LANE={self.released_cleaner_lane} | EMERGENCY: {self.emergency_active}",
            f"DEFECTS: CLEANED {self.mission_cleaned_cnt}/{len(self.defects)} | ACTIVE {active_cnt}",
            f"GEOFENCE: {'IN BOUNDS' if self.all_in_bounds else 'BREACH'} (BREACHES: {self.out_of_bounds_count})",
            "--------------------------------------------------",
            "PRIORITIZED TASK QUEUE:"
        ]

        if self.ranked_tasks:
            for i, t in enumerate(self.ranked_tasks[:4]):
                elig_tag = "ELIGIBLE" if t.eligible else "HELD"
                hud_lines.append(
                    f" #{i+1} [{t.defect_id.replace('defect_', '').upper()}] L{t.lane_id} | "
                    f"Score:{t.utility_score:5.2f} | {elig_tag}"
                )
        else:
            hud_lines.append(" (No active tasks in queue)")
        hud_lines.append("==================================================")

        hud.text = "\n".join(hud_lines)
        self.queue_marker_pub.publish(hud)

        # 4. Mapper Sensor FOV Footprint
        mx, my, _ = self.robot_poses['mapper_1']
        fov = Marker()
        fov.header.stamp = now
        fov.header.frame_id = 'hull_world'
        fov.ns = 'sensor_fov'
        fov.id = 8888
        fov.type = Marker.CYLINDER
        fov.action = Marker.ADD
        fov.pose.position.x = mx
        fov.pose.position.y = my
        fov.pose.position.z = 0.08
        fov.pose.orientation.w = 1.0
        fov.scale.x = 3.60  # Diameter = 2 * 1.80m FOV
        fov.scale.y = 3.60
        fov.scale.z = 0.02
        fov.color.r, fov.color.g, fov.color.b, fov.color.a = 0.05, 0.85, 0.95, 0.22
        self.fov_pub.publish(fov)

        # 5. Trajectory Trails
        for rid, pub, cr, cg, cb in [
            ('mapper_1', self.mapper_path_pub, 0.1, 0.8, 1.0),
            ('cleaner_1', self.cleaner_path_pub, 1.0, 0.5, 0.1)
        ]:
            path = self.robot_paths[rid]
            if len(path) >= 2:
                trail = Marker()
                trail.header.stamp = now
                trail.header.frame_id = 'hull_world'
                trail.ns = f'{rid}_trail'
                trail.id = 7777
                trail.type = Marker.LINE_STRIP
                trail.action = Marker.ADD
                trail.pose.orientation.w = 1.0
                trail.scale.x = 0.05
                trail.color.r, trail.color.g, trail.color.b, trail.color.a = cr, cg, cb, 0.70
                trail.points = [Point(x=p[0], y=p[1], z=p[2]) for p in path]
                pub.publish(trail)

        # 6. Cleaner Target Goal Arrow
        c_goal = self.robot_goals['cleaner_1']
        if c_goal:
            cx, cy, _ = self.robot_poses['cleaner_1']
            arrow = Marker()
            arrow.header.stamp = now
            arrow.header.frame_id = 'hull_world'
            arrow.ns = 'cleaner_goal_arrow'
            arrow.id = 6666
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.08
            arrow.scale.y = 0.15
            arrow.scale.z = 0.15
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.8, 0.0, 0.85
            arrow.points = [
                Point(x=cx, y=cy, z=0.15),
                Point(x=c_goal[0], y=c_goal[1], z=0.15)
            ]
            self.cleaner_goal_pub.publish(arrow)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MarkerPublisher()
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
